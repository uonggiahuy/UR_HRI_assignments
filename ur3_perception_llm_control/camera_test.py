"""Receive one real Gazebo RGB frame through ROS, cv_bridge, and OpenCV."""

from __future__ import annotations

import argparse
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


IMAGE_TOPIC = "/camera/image_raw"


class CameraSmokeTest(Node):
    def __init__(self, output: Path | None) -> None:
        super().__init__("camera_smoke_test")
        self._bridge = CvBridge()
        self._output = output
        self.succeeded = False
        self.failure: str | None = None
        self._subscription = self.create_subscription(
            Image, IMAGE_TOPIC, self._on_image, qos_profile_sensor_data,
        )

    def _on_image(self, message: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            if frame.ndim != 3 or frame.shape[2] != 3:
                raise ValueError(f"expected three channels, got shape {frame.shape}")
            if frame.shape[1] != message.width or frame.shape[0] != message.height:
                raise ValueError("OpenCV dimensions disagree with the ROS image")
            if self._output is not None and not cv2.imwrite(str(self._output), frame):
                raise OSError(f"failed to write debug frame to {self._output}")
            self.get_logger().info(
                f"RGB frame received: {frame.shape[1]}x{frame.shape[0]}, "
                f"{frame.shape[2]} channels, ROS encoding={message.encoding}, "
                f"OpenCV dtype={frame.dtype}, frame={message.header.frame_id}"
            )
            self.succeeded = True
        except (ValueError, OSError, cv2.error) as exc:
            self.failure = str(exc)


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Read one RGB camera frame through cv_bridge and OpenCV")
    parser.add_argument("--output", type=Path, help="optional path for one debug PNG frame")
    parser.add_argument("--timeout", type=float, default=15.0)
    parsed, ros_args = parser.parse_known_args(args)
    if parsed.timeout <= 0:
        parser.error("--timeout must be positive")
    rclpy.init(args=ros_args)
    node = CameraSmokeTest(parsed.output)
    try:
        deadline = time.monotonic() + parsed.timeout
        while rclpy.ok() and not node.succeeded and node.failure is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.5, max(0.0, deadline - time.monotonic())))
        if node.failure is not None:
            raise RuntimeError(node.failure)
        if not node.succeeded:
            raise RuntimeError(f"no valid RGB image on {IMAGE_TOPIC} within {parsed.timeout:g} s")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
