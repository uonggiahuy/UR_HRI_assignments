"""Official MediaPipe Tasks inference on a real V4L2 webcam, latest frame only."""
import json
import os
from pathlib import Path
import sys
import signal
import threading
import time
import uuid

from turtlebot4.gesture_logic import GestureFilter
from turtlebot4.yolo_detector import LatestFrame


def runtime_python():
    cache = Path(os.environ.get('XDG_CACHE_HOME', Path.home()/'.cache'))
    return Path(os.environ.get('TURTLEBOT4_GESTURE_PYTHON', cache/'turtlebot4/gesture-venv/bin/python')).expanduser()


def main(args=None):
    python = runtime_python()
    if Path(sys.prefix).resolve() != python.parent.parent.resolve():
        if not python.is_file():
            print('Run scripts/prepare_gesture_runtime.sh first', file=sys.stderr)
            return 1
        os.execv(str(python), [str(python), '-m', 'turtlebot4.gesture_recognition_node', *sys.argv[1:]])
    import cv2
    import mediapipe as mp
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class Recognizer(Node):
        def __init__(self):
            super().__init__('gesture_recognition_node')
            cache = Path(os.environ.get('XDG_CACHE_HOME', Path.home()/'.cache'))
            defaults = dict(webcam_device=0, model_path=str(cache/'turtlebot4/models/gesture_recognizer.task'),
                            show_camera=True, width=640, height=480, fps=20., camera_timeout=.7,
                            opencv_threads=2,
                            confidence=.65, stable_frames=3, cooldown=.7, rearm_frames=4,
                            lost_interval=.3, wave_window=2.4, wave_amplitude=.045,
                            wave_reversals=3, smoothing=.8)
            self.o = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
            o = self.o
            if not 1 <= o['opencv_threads'] <= 4:
                raise ValueError('opencv_threads must be between 1 and 4')
            cv2.setNumThreads(o['opencv_threads'])
            if o['webcam_device'] < 0 or o['width'] < 64 or o['height'] < 64 or not 1 <= o['fps'] <= 60 or not .1 <= o['camera_timeout'] <= 2:
                raise ValueError('Invalid webcam settings')
            self.filter = GestureFilter(**{k: o[k] for k in (
                'confidence', 'stable_frames', 'cooldown', 'rearm_frames', 'lost_interval',
                'wave_window', 'wave_amplitude', 'wave_reversals', 'smoothing')})
            self.publisher = self.create_publisher(String, '/detected_gesture', 1)
            self.state = dict(state='IDLE', selected=None, reason='Waiting for robot')
            self.create_subscription(String, '/assignment6/state', self.status, 1)
            self.session, self.sequence = str(uuid.uuid4()), 0
            self.slot, self.stop = LatestFrame(), threading.Event()
            self.fatal = None
            self.last_capture = self.last_output = time.monotonic()
            self.previous_gray = None
            self.last_timestamp = -1
            self.window = 'TurtleBot 4 — Hand Gesture Control'
            if o['show_camera'] and not os.environ.get('DISPLAY'):
                raise RuntimeError('DISPLAY absent; set show_camera:=false or configure X11')
            # Never accept a file or simulated OAK-D topic as the webcam source.
            self.capture = cv2.VideoCapture(o['webcam_device'], cv2.CAP_V4L2)
            if not self.capture.isOpened():
                self.capture.release()
                raise RuntimeError(f"Cannot open real webcam /dev/video{o['webcam_device']}; check Docker --device and permissions")
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, o['width'])
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, o['height'])
            self.capture.set(cv2.CAP_PROP_FPS, o['fps'])
            self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            try:
                options = mp.tasks.vision.GestureRecognizerOptions(
                    base_options=mp.tasks.BaseOptions(model_asset_path=str(Path(o['model_path']).expanduser()),
                                                     delegate=mp.tasks.BaseOptions.Delegate.CPU),
                    running_mode=mp.tasks.vision.RunningMode.VIDEO, num_hands=1,
                    min_hand_detection_confidence=o['confidence'],
                    min_hand_presence_confidence=o['confidence'],
                    min_tracking_confidence=o['confidence'])
                self.model = mp.tasks.vision.GestureRecognizer.create_from_options(options)
                if o['show_camera']:
                    cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
            except Exception:
                self.capture.release()
                raise
            self.worker = threading.Thread(target=self.acquire, daemon=True, name='real_webcam')
            self.worker.start()
            self.create_timer(1/o['fps'], self.process)
            self.create_timer(.1, self.watchdog)
            self.get_logger().info(f"Real webcam /dev/video{o['webcam_device']}: {self.capture.get(cv2.CAP_PROP_FRAME_WIDTH)}x{self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT)}, requested FPS={o['fps']}")

        def status(self, msg):
            try:
                self.state = json.loads(msg.data)
            except ValueError:
                pass

        def acquire(self):
            try:
                while not self.stop.is_set():
                    ok, frame = self.capture.read()
                    if not ok:
                        raise RuntimeError('Real webcam read failed')
                    self.last_capture = time.monotonic()
                    self.slot.put((self.last_capture, frame))
            except Exception as exc:
                self.fatal = str(exc)

        def publish(self, label='None', confidence=0., event=False, hand=False, camera=True, raw='None', open_hand=False):
            self.sequence += 1
            self.publisher.publish(String(data=json.dumps(dict(
                version=1, session=self.session, sequence=self.sequence, stamp=time.time(),
                gesture=label, raw_gesture=raw, confidence=float(confidence), event=bool(event),
                hand_present=bool(hand), camera_ok=bool(camera), open_hand=bool(open_hand)))))

        def process(self):
            item = self.slot.take(timeout=0)
            if item is None:
                return
            captured, frame = item
            if time.monotonic()-captured > self.o['camera_timeout']:
                return
            gray = cv2.resize(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (160, 120))
            shift = 0.
            if self.previous_gray is not None:
                flow = cv2.calcOpticalFlowFarneback(self.previous_gray, gray, None, .5, 3, 15, 3, 5, 1.2, 0)
                # Median over most of the image estimates camera translation.
                shift = float(np.median(flow[..., 0]))/160
            self.previous_gray = gray
            timestamp = max(self.last_timestamp+1, int(captured*1000))
            self.last_timestamp = timestamp
            result = self.model.recognize_for_video(
                mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)), timestamp)
            hand = bool(result.hand_landmarks)
            category = result.gestures[0][0] if result.gestures and result.gestures[0] else None
            raw, score = (category.category_name, category.score) if category else ('None', 0.)
            # Palm/finger center also moves when a person waves by pivoting
            # their wrist while keeping the wrist itself nearly stationary.
            wrist = sum(result.hand_landmarks[0][i].x for i in (0, 9, 12))/3 if hand else None
            open_hand = False
            hand_score = result.handedness[0][0].score if result.handedness and result.handedness[0] else 0.
            if hand:
                points = result.hand_landmarks[0]
                distance = lambda i: (points[i].x-points[0].x)**2 + (points[i].y-points[0].y)**2
                open_hand = sum(distance(tip) > 1.3*distance(pip) for tip, pip in ((8, 6), (12, 10), (16, 14), (20, 18))) >= 3
            label, event = self.filter.update(raw, score, wrist, captured, shift, open_hand, hand_score)
            if label == 'Wave':
                score = hand_score
            # During execution a visible open palm always stops, including Wave.
            if self.state.get('state') == 'EXECUTING' and (raw == 'Open_Palm' or open_hand):
                label, event = 'Open_Palm', True
            if time.monotonic()-captured > self.o['camera_timeout']:
                self.publish(camera=False)
                return
            self.publish(label, score, event, hand, raw=raw, open_hand=open_hand)
            now = time.monotonic()
            fps = 1/max(.001, now-self.last_output)
            self.last_output = now
            if self.o['show_camera']:
                for landmarks in result.hand_landmarks:
                    points = [(int(p.x*frame.shape[1]), int(p.y*frame.shape[0])) for p in landmarks]
                    for a, b in mp.solutions.hands.HAND_CONNECTIONS:
                        cv2.line(frame, points[a], points[b], (0, 220, 0), 2)
                    for point in points:
                        cv2.circle(frame, point, 3, (0, 0, 255), -1)
                lines = [f'Gesture: {label} ({score:.2f}) | {fps:.1f} FPS',
                         f'Selected: {self.state.get("selected") or "None"}',
                         f'State: {self.state.get("state")}', str(self.state.get('reason', '')),
                         'Ready: '+str(self.state.get('readiness_reason', 'Waiting for behavior node')),
                         f'Wave: {self.filter.wave_progress()}/{self.filter.reversals} direction changes',
                         'Open Palm: STOP | Closed Fist: CANCEL | q: exit']
                color = (0, 0, 255) if self.state.get('state') == 'STOPPED' else (0, 220, 255)
                for i, line in enumerate(lines):
                    cv2.putText(frame, line, (10, 28+28*i), cv2.FONT_HERSHEY_SIMPLEX, .55, color, 2)
                cv2.imshow(self.window, frame)
                if cv2.waitKey(1) & 0xff == ord('q') or cv2.getWindowProperty(self.window, cv2.WND_PROP_VISIBLE) < 1:
                    self.fatal = 'Camera window closed by operator'

        def watchdog(self):
            if self.fatal or time.monotonic()-self.last_capture > self.o['camera_timeout']:
                self.publish(camera=False)
                self.fatal = self.fatal or 'Webcam frame timeout'

        def close(self):
            self.publish(camera=False)
            self.stop.set()
            self.slot.close()
            self.capture.release()
            self.worker.join(timeout=1.)
            self.model.close()
            if self.o['show_camera']:
                cv2.destroyAllWindows()

    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    previous_term = signal.signal(signal.SIGTERM, interrupt)
    node = None
    try:
        node = Recognizer()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.05)
            if node.fatal:
                raise RuntimeError(node.fatal)
        return 0
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(f'GESTURE FAIL: {exc}', file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGTERM, previous_term)


if __name__ == '__main__':
    sys.exit(main())
