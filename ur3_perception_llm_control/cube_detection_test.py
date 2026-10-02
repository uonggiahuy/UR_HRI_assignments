"""Live M4 RGB-only diagnostic; optional test-only scene-pose error report."""

from __future__ import annotations

import argparse
from collections import deque
import json
from math import hypot
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from ur3_perception_llm_control.cube_detector import (
    CUBE_NAMES, CubeDetector, DetectionError, PerceptionConfig, evaluate_stability,
)
from ur3_perception_llm_control.planar_mapper import Calibration, CubeTopMapper, PlanarMapper


class FrameCollector(Node):
    def __init__(self, detector: CubeDetector, frame_count: int) -> None:
        super().__init__("cube_detection_test")
        self.bridge = CvBridge()
        self.detector = detector
        self.samples = deque(maxlen=frame_count)
        self.last_frame = None
        self.failures = []
        self.timestamps = deque(maxlen=frame_count)
        self.subscription = self.create_subscription(
            Image, "/camera/image_raw", self.on_image, qos_profile_sensor_data)

    def on_image(self, message: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            if message.header.frame_id != self.detector.mapper.tabletop.calibration.image_frame:
                raise DetectionError("unexpected image frame")
            timestamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
            sample = self.detector.detect(frame, timestamp)
            self.samples.append(sample)
            self.timestamps.append(timestamp)
            self.last_frame = frame
        except (DetectionError, ValueError, cv2.error) as exc:
            self.failures.append(str(exc))
            self.samples.clear()
            self.timestamps.clear()


def run(args: argparse.Namespace) -> None:
    calibration = Calibration.from_file(args.calibration)
    config = PerceptionConfig.from_file(args.perception)
    mapper = CubeTopMapper(PlanarMapper(calibration), config.camera_xyz, config.cube_top_z_m)
    detector = CubeDetector(config, mapper)
    rclpy.init()
    node = FrameCollector(detector, config.frames)
    try:
        deadline = time.monotonic() + args.timeout
        while rclpy.ok() and len(node.samples) < config.frames and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
        if len(node.samples) < config.frames:
            raise RuntimeError(f"only {len(node.samples)}/{config.frames} complete frames; failures={node.failures[-3:]}")
        stability = evaluate_stability(list(node.samples), config)
        if not stability.stable:
            raise RuntimeError(f"unstable cube detections: max spread {stability.max_spread_m * 1000:.2f} mm")
        print(f"RGB frames={len(node.samples)}, rate={((len(node.timestamps)-1)/(node.timestamps[-1]-node.timestamps[0])):.2f} Hz, "
              f"max XY spread={stability.max_spread_m*1000:.3f} mm; stable={stability.stable}")
        latest = node.samples[-1]
        errors = []
        oracle = None
        if args.oracle_poses_json:
            # Read a saved Gazebo pose/info sample strictly for this diagnostic.
            poses = json.loads(args.oracle_poses_json.read_text(encoding="utf-8"))["pose"]
            oracle = {pose["name"]: pose["position"] for pose in poses
                     if pose.get("name") in CUBE_NAMES}
            if set(oracle) != set(CUBE_NAMES):
                raise ValueError("Gazebo oracle sample lacks all five cube poses")
        annotated = node.last_frame.copy()
        for name in CUBE_NAMES:
            block = latest[name]
            line = (f"{name}: pixel=({block.pixel_center[0]:.2f}, {block.pixel_center[1]:.2f}) "
                    f"XY=({block.world_xy[0]:.5f}, {block.world_xy[1]:.5f}) m "
                    f"area={block.contour_area_px} quality={block.quality:.3f}")
            if oracle is not None:
                # Validation oracle only: never passed to the detector or mapper.
                pose = oracle[name]
                error = hypot(block.world_xy[0]-pose["x"], block.world_xy[1]-pose["y"])
                errors.append(error)
                line += f" oracle_error={error*1000:.3f} mm"
            print(line)
            x, y, w, h = block.bbox_xywh
            cv2.rectangle(annotated, (x, y), (x+w-1, y+h-1), (255, 255, 255), 1)
            cv2.circle(annotated, (round(block.pixel_center[0]), round(block.pixel_center[1])),
                       3, (255, 255, 255), 1)
            cv2.putText(annotated, name, (x-10, y-7), cv2.FONT_HERSHEY_SIMPLEX,
                        0.35, (255, 255, 255), 1)
        if errors:
            print(f"test-only oracle: mean={sum(errors)/len(errors)*1000:.3f} mm, "
                  f"max={max(errors)*1000:.3f} mm")
            if max(errors) > 0.010:
                raise RuntimeError("localization exceeds 10 mm maximum")
        if args.output and not cv2.imwrite(str(args.output), annotated):
            raise OSError(f"could not save {args.output}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv: list[str] | None = None) -> None:
    share = Path(get_package_share_directory("ur3_perception_llm_control"))
    parser = argparse.ArgumentParser(description="M4 RGB cube localization diagnostic")
    parser.add_argument("--calibration", type=Path, default=share / "config/camera_calibration.yaml")
    parser.add_argument("--perception", type=Path, default=share / "config/perception.yaml")
    parser.add_argument("--oracle-poses-json", type=Path,
                        help="saved Gazebo dynamic_pose/info JSON; test oracle only")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--timeout", type=float, default=25.0)
    args = parser.parse_args(argv)
    if args.timeout <= 0:
        parser.error("timeout must be positive")
    run(args)


if __name__ == "__main__":
    main()
