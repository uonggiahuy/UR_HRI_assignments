"""Enter-controlled microphone capture and Vietnamese CPU Whisper inference."""

from collections import deque
import os
from pathlib import Path
import sys

import numpy as np

from turtlebot4.manual_recording import ManualRecorder, TerminalKeyboard


class UtteranceBuffer:
    """Legacy segmentation API retained for compatibility, unused in manual mode."""

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
            model = self.declare_parameter("whisper_model", "small").value
            device = self.declare_parameter("microphone_device", "").value
            device = (
                int(device) if device and str(device).isdigit() else (device or None)
            )
            mode = self.declare_parameter("recording_mode", "manual").value
            language = self.declare_parameter("language", "vi").value
            maximum = float(self.declare_parameter("max_recording_duration", 30.0).value)
            if mode != "manual" or language != "vi":
                raise ValueError("Use recording_mode=manual and language=vi")
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
            self.get_logger().info(
                f"[WHISPER] Loading multilingual model {model} (CPU INT8)..."
            )
            self.model = WhisperModel(
                model, device="cpu", compute_type="int8", cpu_threads=2, num_workers=1
            )
            if not self.model.model.is_multilingual:
                raise ValueError(
                    "A multilingual Whisper model is required for Vietnamese"
                )
            self.publisher = self.create_publisher(String, "/recognized_speech", 10)
            self.keyboard = TerminalKeyboard()
            self.recorder = ManualRecorder(
                stream_factory=lambda callback: sd.InputStream(
                    device=device, samplerate=16000, channels=1, dtype="float32",
                    blocksize=320, callback=callback,
                ),
                transcribe=self.transcribe,
                publish=lambda text: self.publisher.publish(String(data=text)),
                report=lambda text: print(text, flush=True),
                microphone=sd.query_devices(device, "input")["name"],
                max_duration=maximum,
            )
            self.create_timer(0.05, self.poll)
            print(
                "========================================\n"
                " ASSIGNMENT 7 - VOICE NAVIGATION\n"
                "========================================\n\n"
                f"Microphone: READY ({self.recorder.microphone})\n"
                f"Whisper model: {model}\nRecording mode: MANUAL\n\n"
                "[ENTER] Start recording\n[CTRL+C] Exit\n\n"
                "Status: WAITING FOR INPUT", flush=True,
            )

        def transcribe(self, audio):
            segments, info = self.model.transcribe(
                audio, language="vi", beam_size=3,
                condition_on_previous_text=False, temperature=0.0, vad_filter=False,
            )
            segments = list(segments)
            accepted = [s for s in segments if s.no_speech_prob < 0.6]
            print(
                f"[WHISPER] Inference finished: language={info.language}; "
                f"segments={len(segments)}; low-confidence rejected={len(segments) - len(accepted)}",
                flush=True,
            )
            return " ".join(s.text.strip() for s in accepted).strip()

        def poll(self):
            # Drain terminal events before accepting a completed inference. Keys
            # typed while busy must never become a delayed recording request.
            was_busy = self.recorder.state != "WAITING"
            if self.keyboard.enter():
                self.recorder.enter()
            self.recorder.poll()
            if was_busy and self.recorder.state == "WAITING":
                self.keyboard.discard()

        def close(self):
            self.keyboard.close()
            self.recorder.close()

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
