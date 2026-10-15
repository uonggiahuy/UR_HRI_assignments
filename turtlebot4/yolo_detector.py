# SPDX-License-Identifier: AGPL-3.0-or-later
"""Real YOLO inference in one worker, with a capacity-one latest-image slot."""
from collections import deque
import json
import os
from pathlib import Path
import sys
import threading
import time


class LatestFrame:
    """Atomic replacement prevents inference backlog; wake shutdown explicitly."""
    def __init__(self):
        self.condition = threading.Condition()
        self.value = None
        self.closed = False

    def put(self, value):
        with self.condition:
            self.value = value
            self.condition.notify()

    def take(self, timeout=0.5):
        with self.condition:
            self.condition.wait_for(lambda: self.value is not None or self.closed, timeout)
            value, self.value = self.value, None
            return value

    def close(self):
        with self.condition:
            self.closed = True
            self.condition.notify_all()


def runtime_python():
    default = Path(os.environ.get('XDG_CACHE_HOME', Path.home() / '.cache')) / 'turtlebot4/yolo-venv/bin/python'
    return Path(os.environ.get('TURTLEBOT4_YOLO_PYTHON', default)).expanduser()


def main(args=None):
    # Console scripts use the ROS interpreter. Re-exec only this process into
    # the isolated system-site-packages venv; ROS PYTHONPATH stays available.
    python = runtime_python()
    if Path(sys.prefix).resolve() != python.parent.parent.resolve():
        if not python.is_file():
            print('YOLO FAIL: run scripts/prepare_yolo_runtime.sh first', file=sys.stderr)
            return 1
        os.execv(str(python), [str(python), '-m', 'turtlebot4.yolo_detector', *sys.argv[1:]])
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from cv_bridge import CvBridge

    class Detector(Node):
        def __init__(self):
            super().__init__('yolo_detector')
            defaults = dict(image_topic='/oakd/rgb/preview/image_raw',
                            output_topic='/assignment5/yolo/image_annotated', model='yolo11n.pt',
                            image_size=640, confidence=.35, device='cpu', threads=2,
                            max_fps=5., max_frame_age=1., camera_timeout=8.)
            self.options = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
            o = self.options
            if (not 0 < o['confidence'] <= 1 or o['image_size'] < 32 or
                    o['max_fps'] <= 0 or o['threads'] < 1 or o['max_frame_age'] <= 0):
                raise ValueError('Invalid YOLO processing parameters')
            self.slot = LatestFrame()
            self.results = deque(maxlen=1)
            self.lock = threading.Lock()
            self.stop = threading.Event()
            self.bridge = CvBridge()
            self.last_camera = time.monotonic()
            self.last_output = 0.
            self.last_log = 0.
            self.output_times = deque(maxlen=30)
            self.fatal = None
            self.output = self.create_publisher(Image, o['output_topic'], qos_profile_sensor_data)
            self.events = self.create_publisher(String, '/assignment5/yolo/detections', 10)
            self.create_subscription(Image, o['image_topic'], self.receive, qos_profile_sensor_data)
            self.create_timer(.03, self.publish_result)
            self.create_timer(1., self.watchdog)
            self.worker = threading.Thread(target=self.infer, name='yolo_inference', daemon=True)
            self.worker.start()

        def receive(self, msg):
            self.last_camera = time.monotonic()
            self.slot.put(msg)

        def infer(self):
            try:
                import torch
                from ultralytics import YOLO
                torch.set_num_threads(self.options['threads'])
                cache = Path(os.environ.get('XDG_CACHE_HOME', Path.home() / '.cache')) / 'turtlebot4/models'
                cache.mkdir(parents=True, exist_ok=True)
                model_path = Path(self.options['model']).expanduser()
                if not model_path.is_absolute() and model_path.name == str(model_path):
                    model_path = cache / model_path
                model = YOLO(str(model_path))
                self.get_logger().info(f'YOLO loaded {model_path}; device={self.options["device"]}')
                while not self.stop.is_set():
                    msg = self.slot.take()
                    if msg is None:
                        continue
                    started = time.monotonic()
                    now = self.get_clock().now().nanoseconds / 1e9
                    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                    if not -.1 <= now - stamp <= self.options['max_frame_age']:
                        continue
                    frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                    result = model.predict(frame, imgsz=self.options['image_size'],
                                           conf=self.options['confidence'],
                                           device=self.options['device'], verbose=False)[0]
                    detections = [{'class': result.names[int(box.cls.item())],
                                   'confidence': float(box.conf.item()),
                                   'xyxy': box.xyxy[0].tolist()} for box in result.boxes]
                    annotated = self.bridge.cv2_to_imgmsg(result.plot(), encoding='bgr8')
                    annotated.header = msg.header
                    elapsed = time.monotonic() - started
                    with self.lock:
                        self.results.append((annotated, detections, elapsed))
                    self.stop.wait(max(0., 1 / self.options['max_fps'] - elapsed))
            except Exception as exc:
                self.fatal = f'{type(exc).__name__}: {exc}'

        def publish_result(self):
            with self.lock:
                item = self.results.pop() if self.results else None
                self.results.clear()
            if item is None:
                return
            msg, detections, duration = item
            self.last_output = time.monotonic()
            self.output_times.append(self.last_output)
            self.output.publish(msg)
            event = dict(stamp=msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                         frame_id=msg.header.frame_id, detections=detections,
                         inference_seconds=duration)
            self.events.publish(String(data=json.dumps(event)))
            if time.monotonic() - self.last_log >= 2.:
                labels = ', '.join(f'{d["class"]} {d["confidence"]:.2f}' for d in detections) or 'none'
                span = self.output_times[-1] - self.output_times[0]
                effective = (len(self.output_times) - 1) / span if span > 0 else 0.
                self.get_logger().info(f'YOLO objects=[{labels}], inference={1/max(duration,.001):.1f} FPS, '
                                       f'effective={effective:.1f} FPS')
                self.last_log = time.monotonic()

        def watchdog(self):
            if time.monotonic() - self.last_camera > self.options['camera_timeout']:
                self.fatal = f'Camera absent/stale: {self.options["image_topic"]}'
            if self.last_output and time.monotonic() - self.last_output > self.options['camera_timeout']:
                self.fatal = 'YOLO stopped producing frames'

        def close(self):
            self.stop.set()
            self.slot.close()
            self.worker.join(timeout=5.)

    rclpy.init(args=args)
    node = None
    try:
        node = Detector()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.1)
            if node.fatal:
                raise RuntimeError(node.fatal)
        return 0
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(f'YOLO FAIL: {exc}', file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
