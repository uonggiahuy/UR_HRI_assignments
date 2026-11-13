"""Real webcam FER inference in one worker, bounded latest-frame acquisition."""
import json
import os
from pathlib import Path
import signal
import sys
import threading
import time
import uuid
from turtlebot4.emotion_logic import EmotionFilter, primary_face, BEHAVIORS
from turtlebot4.yolo_detector import LatestFrame
from turtlebot4.emotion_phone import normalized_roi, qualification_reason, score_margin


def runtime_python():
    cache = Path(os.environ.get('XDG_CACHE_HOME', Path.home()/'.cache'))
    return Path(os.environ.get('TURTLEBOT4_EMOTION_PYTHON', cache/'turtlebot4/emotion-venv/bin/python')).expanduser()


def main(args=None):
    python = runtime_python()
    if Path(sys.prefix).resolve() != python.parent.parent.resolve():
        if not python.is_file():
            raise RuntimeError('Run scripts/prepare_assignment8_runtime.sh first')
        os.execv(str(python), [str(python), '-m', 'turtlebot4.emotion_recognition_node', *sys.argv[1:]])
    # CPU avoids coupling ROS/Gazebo's GPU libraries to TensorFlow CUDA.
    os.environ['CUDA_VISIBLE_DEVICES'] = '-1'
    os.environ.setdefault('TF_NUM_INTRAOP_THREADS', '2')
    os.environ.setdefault('TF_NUM_INTEROP_THREADS', '1')
    import cv2
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock, ClockType
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    from fer import FER

    class Recognizer(Node):
        def __init__(self):
            super().__init__('emotion_recognition_node')
            defaults = dict(camera_device='/dev/video0', emotion_model='fer', confidence_threshold=.65,
                            stable_frames=4, cooldown=1., no_face_timeout=.7,
                            detection_interval=.15, show_camera=True, camera_timeout=.7,
                            width=640, height=480, face_selection_policy='reject_multiple',
                            phone_mode=False, minimum_margin=0.0)
            self.o = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
            o = self.o
            if (o['emotion_model'] != 'fer' or o['face_selection_policy'] != 'reject_multiple'
                    or not .05 <= o['detection_interval'] <= .5 or not .1 <= o['camera_timeout'] <= 2
                    or not 160 <= o['width'] <= 1280 or not 120 <= o['height'] <= 720
                    or not str(o['camera_device']).startswith('/dev/video')):
                raise ValueError('Invalid webcam/backend settings')
            if o['show_camera'] and not os.environ.get('DISPLAY'):
                raise RuntimeError('Webcam display requires DISPLAY; use show_camera:=false for diagnostics')
            cv2.setNumThreads(2)
            self.filter = EmotionFilter(o['confidence_threshold'], o['stable_frames'], o['cooldown'], o['no_face_timeout'])
            self.roi, self.drag_start, self.drag_end = None, None, None
            self.roi_generation = 0
            self.phone_used = False
            self.operator_stop = self.manual_rearm = False
            self.filter_reason = 'Waiting for camera'
            self.frame_size = (o['width'], o['height'])
            if not 0 <= o['minimum_margin'] <= .5:
                raise ValueError('Invalid minimum score margin')
            self.publisher = self.create_publisher(String, '/detected_emotion', 1)
            self.state, self.state_seen = {}, -float('inf')
            self.create_subscription(String, '/assignment8/state', self.status, 1)
            self.session, self.sequence = str(uuid.uuid4()), 0
            self.slot, self.results, self.stop = LatestFrame(), LatestFrame(), threading.Event()
            self.last_capture = self.last_result = -float('inf')
            self.fatal, self.last_box, self.last_primary = None, None, False
            self.output = dict(label='None', event=False, control_ok=False)
            self.capture_frames = 0
            self.scores = {}
            self.prediction = 'None'
            self.score, self.latency, self.frames, self.started = 0., 0., 0, time.monotonic()
            self.window = 'TurtleBot 4 - Facial expression (visible expression prediction)'
            self.model = FER(mtcnn=False)
            self.capture = cv2.VideoCapture(o['camera_device'], cv2.CAP_V4L2)
            if not self.capture.isOpened():
                self.capture.release()
                raise RuntimeError('Cannot open real webcam '+o['camera_device'])
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, o['width'])
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, o['height'])
            self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if o['show_camera']:
                cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.window, 960, 720)
                cv2.setMouseCallback(self.window, self.mouse)
            self.workers = [threading.Thread(target=f, daemon=True) for f in (self.acquire, self.infer)]
            for worker in self.workers:
                worker.start()
            self.wall = Clock(clock_type=ClockType.STEADY_TIME)
            self.create_timer(.05, self.process, clock=self.wall)

        def status(self, msg):
            try:
                self.state = json.loads(msg.data)
                self.state_seen = time.monotonic()
            except ValueError:
                pass

        def mouse(self, event, x, y, flags, userdata):
            if not self.o['phone_mode']:
                return
            if event == cv2.EVENT_LBUTTONDOWN:
                self.drag_start = self.drag_end = (x, y)
                self.operator_stop = True
                self.roi_generation += 1
                self.last_primary = False
                self.output.update(event=False, control_ok=False)
            elif event == cv2.EVENT_MOUSEMOVE and self.drag_start is not None:
                self.drag_end = (x, y)
            elif event == cv2.EVENT_LBUTTONUP and self.drag_start is not None:
                self.roi = normalized_roi(self.drag_start, (x, y), *self.frame_size)
                self.drag_start = self.drag_end = None
                self.roi_generation += 1
                self.reset_selection()

        def reset_selection(self):
            self.filter = EmotionFilter(self.o['confidence_threshold'], self.o['stable_frames'],
                                        self.o['cooldown'], self.o['no_face_timeout'])
            self.phone_used = False
            self.operator_stop = True
            self.last_primary = False
            self.output.update(event=False, control_ok=False)

        def acquire(self):
            try:
                while not self.stop.is_set():
                    ok, frame = self.capture.read()
                    if not ok:
                        raise RuntimeError('Webcam disconnected/read failed')
                    self.last_capture = time.monotonic()
                    self.capture_frames += 1
                    self.slot.put((self.last_capture, frame))
            except Exception as exc:
                self.fatal = str(exc)

        def infer(self):
            try:
                while not self.stop.is_set():
                    item = self.slot.take(.1)
                    if item is None:
                        continue
                    stamp, frame = item
                    started = time.monotonic()
                    generation, roi = self.roi_generation, self.roi
                    if self.o['phone_mode'] and (roi is None or self.drag_start is not None):
                        faces = []
                    elif roi is not None and self.o['phone_mode']:
                        h, w = frame.shape[:2]
                        x0, y0, x1, y1 = int(roi[0]*w), int(roi[1]*h), int(roi[2]*w), int(roi[3]*h)
                        faces = self.model.detect_emotions(frame[y0:y1, x0:x1])
                        for face in faces:
                            face['box'][0] += x0
                            face['box'][1] += y0
                    else:
                        faces = self.model.detect_emotions(frame)
                    self.results.put((stamp, frame, faces, time.monotonic()-started, generation))
                    self.stop.wait(max(0., self.o['detection_interval']-(time.monotonic()-started)))
            except Exception as exc:
                self.fatal = 'FER inference failed: '+str(exc)

        def publish(self, camera_ok):
            self.sequence += 1
            d = dict(version=1, session=self.session, sequence=self.sequence, stamp=time.time(),
                     confidence=self.score, scores=self.scores,
                     phone_mode=self.o['phone_mode'], manual_rearm=self.manual_rearm,
                     operator_stop=self.operator_stop, minimum_margin=self.o['minimum_margin'],
                     qualification_reason=self.filter_reason, inference_ms=self.latency*1000,
                     inference_fps=self.frames/max(.001, time.monotonic()-self.started),
                     capture_fps=self.capture_frames/max(.001, time.monotonic()-self.started), primary_face=self.last_primary, camera_ok=camera_ok,
                     **self.output)
            if not camera_ok:
                d.update(control_ok=False, event=False)
            self.publisher.publish(String(data=json.dumps(d)))
            # Heartbeats must never repeat an event id/trigger.
            self.output['event'] = False
            self.manual_rearm = self.operator_stop = False

        def process(self):
            now = time.monotonic()
            item = self.results.take(0.)
            camera_ok = self.fatal is None and now-self.last_capture <= self.o['camera_timeout']
            frame = None
            if item:
                stamp, frame, faces, self.latency, generation = item
                self.frame_size = (frame.shape[1], frame.shape[0])
                if generation != self.roi_generation or self.drag_start is not None:
                    faces = []
                self.last_result = stamp
                primary = primary_face(faces)
                self.scores = faces[0]['emotions'] if primary else {}
                label, self.score, self.last_box = primary if primary else ('None', 0., None)
                self.prediction = label
                # Empty scene allows the bounded no-face timeout. Multiple faces
                # and unsupported dominant labels stop immediately.
                eligible = len(faces) <= 1 and (not faces or primary is not None)
                idle = (now-self.state_seen < .7 and self.state.get('state') == 'IDLE'
                        and self.state.get('ready', False))
                filter_score = self.score
                if score_margin(self.scores) < self.o['minimum_margin'] and label != 'Neutral':
                    filter_score = min(self.score, self.o['confidence_threshold']-.001)
                self.output = self.filter.update(label, filter_score, stamp, idle, camera_ok, eligible)
                self.last_primary = primary is not None
                self.filter_reason = qualification_reason(label, self.score, self.scores,
                    self.filter.count, self.o['confidence_threshold'], self.o['stable_frames'],
                    self.o['minimum_margin'])
                if self.o['phone_mode']:
                    # Holding a photograph never starts/repeats movement automatically.
                    self.output['event'] = False
                    if self.filter_reason:
                        self.filter.armed = False
                elif self.filter_reason and label != 'Neutral':
                    self.output['event'] = False
                self.frames += 1
                if self.output['event']:
                    self.get_logger().info(json.dumps(dict(expression=label, confidence=self.score,
                        inference_ms=round(self.latency*1000, 2), event=self.sequence+1)))
            if now-self.last_result > self.o['camera_timeout']:
                camera_ok = False
            if frame is not None and self.o['show_camera']:
                if self.last_box is not None:
                    x, y, w, h = map(int, self.last_box)
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                label = self.output['label']
                if self.o['phone_mode'] and self.roi is not None:
                    h, w = frame.shape[:2]
                    cv2.rectangle(frame, (int(self.roi[0]*w), int(self.roi[1]*h)),
                                  (int(self.roi[2]*w), int(self.roi[3]*h)), (255, 180, 0), 2)
                if self.drag_start is not None and self.drag_end is not None:
                    cv2.rectangle(frame, self.drag_start, self.drag_end, (255, 180, 0), 2)
                hint = ('PHONE: drag ROI | Space: run once | R: rearm | S: stop'
                        if self.o['phone_mode'] else 'WEBCAM: Neutral to rearm | S: stop')
                lines = [f'{self.prediction}: {self.score:.2f} | FER {self.frames/max(.001, now-self.started):.1f} FPS | {self.latency*1000:.0f} ms',
                         BEHAVIORS.get(self.state.get('selected') or label, 'No accepted expression'),
                         str(self.state.get('state', 'WAITING'))+' | '+str(self.state.get('reason', '')),
                         str(self.state.get('readiness_reason', 'Waiting for controller')),
                         self.filter_reason or ('Already used: press R to rearm' if self.phone_used else
                             'Qualified: press Space' if self.o['phone_mode'] else 'Expression qualified'), hint]
                for index, line in enumerate(lines):
                    cv2.putText(frame, line, (8, 25+index*25), cv2.FONT_HERSHEY_SIMPLEX, .48, (0, 255, 255), 1)
                cv2.imshow(self.window, frame)
            key = cv2.waitKey(1) & 0xff if self.o['show_camera'] else 255
            if key in (27, ord('q')):
                raise KeyboardInterrupt
            if key == ord('s'):
                self.operator_stop = True
                self.phone_used = True
            if key == ord('r') and self.o['phone_mode']:
                self.roi_generation += 1
                self.reset_selection()
            if key == ord(' ') and self.o['phone_mode']:
                if (not self.phone_used and not self.filter_reason and self.last_primary and camera_ok
                        and now-self.state_seen < .7 and self.state.get('ready', False)
                        and self.state.get('state') in ('IDLE', 'STOPPED')):
                    self.output.update(label=self.prediction, event=True)
                    self.manual_rearm, self.phone_used = True, True
                else:
                    self.get_logger().warning('Phone request rejected: '+
                        (self.filter_reason or str(self.state.get('readiness_reason', 'No fresh controller state'))))
            self.publish(camera_ok)
            if self.fatal:
                raise RuntimeError(self.fatal)

        def close(self):
            self.stop.set()
            self.output.update(event=False, control_ok=False)
            self.publish(False)
            self.slot.close()
            self.results.close()
            for worker in self.workers:
                worker.join(timeout=1.)
            self.capture.release()
            cv2.destroyAllWindows()

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    previous = signal.signal(signal.SIGTERM, interrupt)
    node = None
    try:
        node = Recognizer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGTERM, previous)


if __name__ == '__main__':
    main()
