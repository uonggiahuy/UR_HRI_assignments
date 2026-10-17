"""Optional HighGUI viewer of annotated images; inference stays in the detector."""
from copy import copy
import os
import subprocess
import sys

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

WINDOW_TITLE = 'TurtleBot 4 — YOLO Object Detection'


def gui_unavailable_reason():
    """Probe in a child: some GUI backends abort rather than raise on bad X11."""
    if not os.environ.get('DISPLAY'):
        return 'DISPLAY is unset; configure container X11 access'
    probe = (
        "import cv2; cv2.namedWindow('YOLO GUI check', cv2.WINDOW_AUTOSIZE); "
        "cv2.waitKey(1); cv2.destroyAllWindows(); cv2.waitKey(1)"
    )
    try:
        result = subprocess.run([sys.executable, '-c', probe], capture_output=True,
                                text=True, timeout=5., check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return str(exc)
    if result.returncode:
        return f'HighGUI/X11 check failed ({result.returncode}): {result.stderr.strip()[-600:]}'
    return None


def display_size(width, height, max_width, max_height):
    """Fit the source into the display bounds without changing aspect ratio."""
    if min(width, height, max_width, max_height) <= 0:
        raise ValueError('Image and display dimensions must be positive')
    scale = min(max_width / width, max_height / height)
    return max(1, round(width * scale)), max(1, round(height * scale))


class ImageWindow:
    """Main-thread GUI pump with a single replaceable ROS image slot."""

    def __init__(self, bridge, width, height):
        self.bridge = bridge
        self.width, self.height = width, height
        self.latest = None
        self.closed = False
        self.displayed = False
        cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_AUTOSIZE)

    def receive(self, message):
        self.latest = message

    def pump(self):
        message, self.latest = self.latest, None
        try:
            if message is not None:
                image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
                height, width = image.shape[:2]
                size = display_size(width, height, self.width, self.height)
                cv2.imshow(WINDOW_TITLE, cv2.resize(image, size))
                self.displayed = True
        finally:
            # Pump events even while the camera is quiet or a frame is malformed.
            key = cv2.waitKey(1) & 0xff
            # GTK3 does not implement WND_PROP_VISIBLE; AUTOSIZE exists on GTK/Qt.
            self.closed = key in (27, ord('q')) or (self.displayed and
                cv2.getWindowProperty(WINDOW_TITLE, cv2.WND_PROP_AUTOSIZE) < 0)

    def close(self):
        self.latest = None
        cv2.destroyAllWindows()
        cv2.waitKey(1)


class YoloViewer(Node):
    def __init__(self):
        super().__init__('yolo_viewer')
        self.declare_parameter('image_topic', '/assignment5/yolo/image_annotated')
        self.declare_parameter('display_width', 640)
        self.declare_parameter('display_height', 480)
        self.window = None
        self.done = False

    def open(self):
        width = int(self.get_parameter('display_width').value)
        height = int(self.get_parameter('display_height').value)
        display_size(1, 1, width, height)
        reason = gui_unavailable_reason()
        if reason:
            self.get_logger().warning(
                f'Optional YOLO window disabled: {reason}. Use show_yolo_window:=false; '
                'detector, annotated topic and mission remain independent.')
            return False
        self.window = ImageWindow(CvBridge(), width, height)
        # Sensor-data compatibility, but keep just one unread image in DDS.
        qos = copy(qos_profile_sensor_data)
        qos.depth = 1
        topic = self.get_parameter('image_topic').value
        self.create_subscription(Image, topic, self.window.receive, qos)
        # GUI events must continue even when Gazebo pauses /clock.
        self.create_timer(1. / 30., self.update, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f'YOLO window opened: {WINDOW_TITLE}; topic={topic}; q/Esc closes viewer')
        return True

    def update(self):
        try:
            self.window.pump()
            self.done = self.window.closed
        except CvBridgeError as exc:
            self.get_logger().warning(f'Cannot display annotated image: {exc}', throttle_duration_sec=5.)
        except cv2.error as exc:
            self.get_logger().warning(f'Optional YOLO window closed: {exc}')
            self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YoloViewer()
        if node.open():
            while rclpy.ok() and not node.done:
                rclpy.spin_once(node, timeout_sec=.1)
        return 0
    except (KeyboardInterrupt, ExternalShutdownException):
        return 0
    except (cv2.error, ValueError) as exc:
        if node:
            node.get_logger().warning(f'Optional YOLO window unavailable: {exc}')
        return 0
    finally:
        if node is not None:
            if node.window is not None:
                try:
                    node.window.close()
                except cv2.error:
                    pass  # Display may already have gone away during shutdown.
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
