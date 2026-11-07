"""Bounded manual recording and nonblocking controlling-terminal input."""

import math
import os
import queue
import termios
import threading
import time

import numpy as np


class TerminalKeyboard:
    """Read /dev/tty because ROS launch gives child stdin a pipe.

    Canonical mode and signal handling stay unchanged. The node's wall timer
    reads nonblocking; navigation and launch never wait for keyboard input.
    """

    def __init__(self):
        try:
            self.fd = os.open('/dev/tty', os.O_RDONLY | os.O_NONBLOCK | os.O_NOCTTY)
            if not os.isatty(self.fd):
                raise OSError('not a terminal')
            self.discard()
        except OSError as exc:
            self.close()
            raise RuntimeError(
                'Manual recording requires an interactive controlling terminal. '
                'Run ros2 launch in a terminal (docker exec -it when using Docker).'
            ) from exc

    def enter(self):
        try:
            data = os.read(self.fd, 4096)
        except BlockingIOError:
            return False
        if not data:
            raise RuntimeError('Recording terminal closed (EOF)')
        return b'\n' in data

    def discard(self):
        termios.tcflush(self.fd, termios.TCIFLUSH)

    def close(self):
        if getattr(self, 'fd', None) is not None:
            os.close(self.fd)
            self.fd = None


class ManualRecorder:
    """Only STOP submits audio; only executor-side poll publishes results."""

    def __init__(self, stream_factory, transcribe, publish, report=print,
                 microphone='unknown', sample_rate=16000, max_duration=30.0,
                 clock=time.monotonic):
        if not math.isfinite(max_duration) or not 0.5 <= max_duration <= 300:
            raise ValueError('max_recording_duration must be between 0.5 and 300 seconds')
        self.stream_factory = stream_factory
        self.transcribe = transcribe
        self.publish = publish
        self.report = report
        self.microphone = microphone
        self.sample_rate = sample_rate
        self.max_duration = max_duration
        self.clock = clock
        self.state = 'WAITING'
        self.frames = []
        self.samples = 0
        self.stream = None
        self.error = None
        self.lock = threading.Lock()
        self.results = queue.Queue(maxsize=1)
        self.worker = None
        self.closed = False
        self.started = 0.0
        self.last_callback = 0.0
        self.last_enter = -math.inf

    def enter(self):
        now = self.clock()
        if self.closed or self.state == 'TRANSCRIBING' or now - self.last_enter < 0.25:
            return
        self.last_enter = now
        if self.state == 'WAITING':
            self.start()
        elif self.state == 'RECORDING':
            self.stop_recording()

    def start(self):
        self.frames = []
        self.samples = 0
        self.error = None
        self.started = self.last_callback = self.clock()
        try:
            self.stream = self.stream_factory(self.capture)
            self.sample_rate = float(self.stream.samplerate)
            if self.sample_rate != 16000:
                raise ValueError(f'Expected 16000 Hz, got {self.sample_rate}')
            self.state = 'RECORDING'
            self.stream.start()
            self.report('[RECORDING] Microphone recording started.\n'
                        'Speak your Vietnamese navigation command.\n\n'
                        '[ENTER] Stop recording and recognize')
        except Exception as exc:
            self.fail(f'Microphone failure: {exc}')

    def capture(self, indata, frames, timing, status):
        with self.lock:
            if self.state != 'RECORDING' or self.error:
                return
            self.last_callback = self.clock()
            if status:
                self.error = f'Audio stream discontinuity: {status}'
            elif (indata.dtype != np.float32 or indata.shape != (frames, 1)
                  or not np.isfinite(indata).all()):
                self.error = 'Invalid audio: expected finite mono float32 samples'
            elif self.samples + frames > int(self.max_duration * self.sample_rate):
                self.error = 'Maximum recording duration exceeded; recording discarded'
            else:
                self.frames.append(indata[:, 0].copy())
                self.samples += frames

    def release_stream(self):
        stream, self.stream = self.stream, None
        if stream is not None:
            try:
                stream.stop()
            finally:
                stream.close()

    def idle(self):
        self.frames = []
        self.state = 'WAITING'
        self.report('[READY] Press ENTER to record another command.')

    def fail(self, reason):
        was_recording = self.state == 'RECORDING'
        with self.lock:
            self.state = 'WAITING'
        try:
            self.release_stream()
        except Exception as exc:
            reason += f'; stream cleanup: {exc}'
        if was_recording:
            self.audio_summary()
        self.report('[FAILED] ' + reason)
        self.idle()

    def stop_recording(self):
        with self.lock:
            self.state = 'TRANSCRIBING'
        try:
            self.release_stream()
            self.report('[RECORDING] Stopped.')
            audio, rms = self.audio_summary()
            if self.error:
                raise ValueError(self.error)
            if self.clock() - self.started >= self.max_duration:
                raise ValueError('Maximum recording duration exceeded; recording discarded')
            self.frames = []
            # Reject digital silence without imposing the old voiced-frame gate.
            if not audio.size or rms < 1e-5:
                raise ValueError('Empty or silent recording; no command published')
            self.report('[WHISPER] Processing recorded speech...')
            self.worker = threading.Thread(target=self.recognize, args=(audio,), daemon=True)
            self.worker.start()
        except Exception as exc:
            self.fail(str(exc))

    def audio_summary(self):
        audio = (np.concatenate(self.frames) if self.frames
                 else np.empty(0, dtype=np.float32))
        peak = float(np.max(np.abs(audio))) if audio.size else 0.0
        rms = float(np.sqrt(np.mean(audio * audio))) if audio.size else 0.0
        self.report(
            f'[AUDIO] Microphone: {self.microphone}; duration={audio.size / self.sample_rate:.3f}s; '
            f'samples={audio.size}; sample_rate={self.sample_rate:g} Hz; '
            f'channels=1; dtype=float32; blocksize=320; peak={peak:.6f}; RMS={rms:.6f}'
        )
        if peak > 1.0:
            self.report('[AUDIO] WARNING: samples exceed nominal [-1, 1] range; '
                        'check microphone gain/distortion if recognition is inaccurate')
        return audio, rms

    def recognize(self, audio):
        started = self.clock()
        try:
            text = self.transcribe(audio).strip()
            result = (text, None, self.clock() - started)
        except Exception as exc:
            result = ('', str(exc), self.clock() - started)
        self.results.put_nowait(result)

    def poll(self):
        if self.closed:
            return
        if self.state == 'RECORDING':
            if self.clock() - self.started >= self.max_duration:
                self.fail('Maximum recording duration exceeded; recording discarded')
            elif self.error:
                self.fail(self.error)
            elif not self.stream.active or self.clock() - self.last_callback > 3:
                self.fail('Microphone stopped or callback timed out')
        elif self.state == 'TRANSCRIBING':
            try:
                text, error, duration = self.results.get_nowait()
            except queue.Empty:
                return
            self.report(f'[WHISPER] Recognition duration: {duration:.3f}s')
            if error:
                self.report('[WHISPER] Failed: ' + error)
            elif text:
                self.report('[WHISPER] Recognized: ' + text)
                self.publish(text)
                self.report('[ROS] Published: /recognized_speech')
            else:
                self.report('[WHISPER] No speech recognized; no command published')
            self.idle()

    def close(self):
        self.closed = True
        with self.lock:
            self.state = 'WAITING'
        try:
            self.release_stream()
        finally:
            self.frames = []
            # Native Whisper cannot cancel a running call. The daemon worker owns
            # no ROS resources; process exit releases it without late publication.
            if self.worker:
                self.worker.join(timeout=1.0)
