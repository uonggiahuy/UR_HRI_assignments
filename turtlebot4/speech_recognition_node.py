"""Live microphone capture with bounded silence segmentation and CPU Whisper."""

from collections import deque
import os
from pathlib import Path
import queue
import sys
import threading
import time

import numpy as np

from turtlebot4.voice_command_parser import TranscriptGate


class UtteranceBuffer:
    """20 ms frames; bounded pre-roll, utterance length and end silence."""

    def __init__(
        self, threshold=0.012, silence_frames=35, max_frames=400, min_frames=15
    ):
        self.threshold = threshold
        self.silence_frames = silence_frames
        self.max_frames = max_frames
        self.min_frames = min_frames
        self.preroll = deque(maxlen=10)
        self.frames = []
        self.quiet = 0
        self.voiced = 0
        self.dropping = False

    def feed(self, frame):
        frame = np.asarray(frame, dtype=np.float32).reshape(-1)
        voiced = bool(len(frame) and np.sqrt(np.mean(frame * frame)) >= self.threshold)
        if self.dropping:
            self.quiet = 0 if voiced else self.quiet + 1
            if self.quiet >= self.silence_frames:
                self.reset()
            return None
        if not self.frames:
            self.preroll.append(frame.copy())
            if not voiced:
                return None
            self.frames = list(self.preroll)
            self.preroll.clear()
        else:
            self.frames.append(frame.copy())
        self.voiced += int(voiced)
        self.quiet = 0 if voiced else self.quiet + 1
        if len(self.frames) >= self.max_frames:
            # Reject overlong/noisy utterances; never split one phrase into missions.
            self.frames = []
            self.dropping = True
            return None
        if self.quiet >= self.silence_frames:
            audio = (
                np.concatenate(self.frames) if self.voiced >= self.min_frames else None
            )
            self.reset()
            return audio
        return None

    def reset(self):
        self.frames = []
        self.quiet = self.voiced = 0
        self.dropping = False
        self.preroll.clear()


def runtime_python():
    cache = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache"))
    return Path(
        os.environ.get(
            "TURTLEBOT4_SPEECH_PYTHON", cache / "turtlebot4/speech-venv/bin/python"
        )
    )


def main(args=None):
    python = runtime_python()
    if Path(sys.prefix).resolve() != python.parent.parent.resolve():
        if not python.is_file():
            print("Run scripts/prepare_assignment7_runtime.sh first", file=sys.stderr)
            return 1
        native = str(python.parent.parent / "native/usr/lib/x86_64-linux-gnu")
        for variable in ("LIBRARY_PATH", "LD_LIBRARY_PATH"):
            os.environ[variable] = native + os.pathsep + os.environ.get(variable, "")
        os.execv(
            str(python),
            [str(python), "-m", "turtlebot4.speech_recognition_node", *sys.argv[1:]],
        )
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    import sounddevice as sd
    from faster_whisper import WhisperModel

    class Recognizer(Node):
        def __init__(self):
            super().__init__("speech_recognition_node")
            model = self.declare_parameter("whisper_model", "base").value
            device = self.declare_parameter("microphone_device", "").value
            device = (
                int(device) if device and str(device).isdigit() else (device or None)
            )
            threshold = float(self.declare_parameter("vad_threshold", 0.012).value)
            if not 0.001 <= threshold <= 0.5:
                raise ValueError("Invalid VAD threshold")
            inputs = [d for d in sd.query_devices() if d["max_input_channels"] > 0]
            if not inputs:
                raise RuntimeError(
                    "No microphone input: /dev/snd and audio-server socket not connected to Docker"
                )
            sd.check_input_settings(
                device=device, channels=1, dtype="float32", samplerate=16000
            )
            self.get_logger().info(
                f'Microphone: {sd.query_devices(device, "input")["name"]}'
            )
            self.model = WhisperModel(
                model, device="cpu", compute_type="int8", cpu_threads=2, num_workers=1
            )
            if not self.model.model.is_multilingual:
                raise ValueError(
                    "A multilingual Whisper model is required for Vietnamese"
                )
            self.audio = queue.Queue(maxsize=100)
            self.results = queue.Queue(maxsize=2)
            self.stop = threading.Event()
            self.inference = threading.Event()
            self.processing_since = 0.0
            self.buffer = UtteranceBuffer(threshold=threshold)
            self.gate = TranscriptGate()
            self.last_audio = time.monotonic()
            self.fatal = None
            self.publisher = self.create_publisher(String, "/recognized_speech", 10)
            self.stream = sd.InputStream(
                device=device,
                samplerate=16000,
                channels=1,
                dtype="float32",
                blocksize=320,
                callback=self.capture,
            )
            self.stream.start()
            self.worker = threading.Thread(target=self.recognize, daemon=True)
            self.worker.start()
            self.create_timer(0.1, self.poll)
            self.get_logger().info(
                "Listening to live microphone; Vietnamese / CPU INT8"
            )

        def capture(self, indata, frames, timing, status):
            self.last_audio = time.monotonic()
            if status:
                self.fatal = f"Audio stream discontinuity: {status}"
                return
            if self.inference.is_set():
                return
            try:
                self.audio.put_nowait(indata[:, 0].copy())
            except queue.Full:
                self.fatal = (
                    "Audio queue overflow: utterance discarded; restart microphone"
                )

        def recognize(self):
            while not self.stop.is_set():
                try:
                    frame = self.audio.get(timeout=0.2)
                except queue.Empty:
                    continue
                audio = self.buffer.feed(frame)
                if audio is None:
                    continue
                try:
                    self.inference.set()
                    self.processing_since = time.monotonic()
                    self.get_logger().info("Recognizing one live utterance...")
                    segments, _ = self.model.transcribe(
                        audio,
                        language="vi",
                        beam_size=3,
                        condition_on_previous_text=False,
                        temperature=0.0,
                        vad_filter=False,
                    )
                    text = " ".join(
                        s.text.strip() for s in segments if s.no_speech_prob < 0.6
                    ).strip()
                    # Discard queued sound captured during inference: it cannot
                    # become a delayed mission after a previous utterance.
                    while not self.audio.empty():
                        try:
                            self.audio.get_nowait()
                        except queue.Empty:
                            break
                    self.buffer.reset()
                    if text and self.gate.accept(text, time.monotonic()):
                        self.results.put_nowait(text)
                    elif not text:
                        self.get_logger().info(
                            "Empty recognition; no command published"
                        )
                except Exception as exc:
                    self.fatal = f"Recognizer error: {exc}"
                    return
                finally:
                    self.inference.clear()
                    self.processing_since = 0.0

        def poll(self):
            if (
                self.inference.is_set()
                and time.monotonic() - self.processing_since > 20.0
            ):
                raise RuntimeError("Recognizer inference timed out after 20 seconds")
            if (
                self.fatal
                or time.monotonic() - self.last_audio > 3.0
                or not self.stream.active
            ):
                raise RuntimeError(
                    self.fatal or "Microphone disconnected or audio capture timed out"
                )
            try:
                text = self.results.get_nowait()
            except queue.Empty:
                return
            self.get_logger().info("Recognized Vietnamese (LIVE): " + text)
            self.publisher.publish(String(data=text))

        def close(self):
            self.stop.set()
            self.stream.stop()
            self.stream.close()
            self.worker.join(timeout=5.0)

    rclpy.init(args=args)
    node = None
    try:
        node = Recognizer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"SPEECH BLOCKED/FAILED: {exc}", file=sys.stderr)
        return 1
    finally:
        if node:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
