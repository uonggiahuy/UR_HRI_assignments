"""Live M3 check of fixed tabletop references using RGB and CameraInfo only."""

from __future__ import annotations

import argparse
from math import hypot, isclose
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from ur3_perception_llm_control.planar_mapper import Calibration, PlanarMapper
from ur3_perception_llm_control.workcell_scene import load_scene


class CameraSamples(Node):
    def __init__(self) -> None:
        super().__init__("homography_test")
        self.image: Image | None = None
        self.info: CameraInfo | None = None
        self._image_subscription = self.create_subscription(
            Image, "/camera/image_raw", self._on_image, qos_profile_sensor_data
        )
        self._info_subscription = self.create_subscription(
            CameraInfo, "/camera/camera_info", self._on_info, qos_profile_sensor_data
        )

    def _on_image(self, message: Image) -> None:
        self.image = message

    def _on_info(self, message: CameraInfo) -> None:
        self.info = message


def _check_camera_info(calibration: Calibration, info: CameraInfo) -> None:
    if (info.width, info.height, info.header.frame_id, info.distortion_model) != (
        calibration.image_width, calibration.image_height,
        calibration.image_frame, calibration.distortion_model,
    ):
        raise ValueError("live CameraInfo dimensions, frame, or distortion model changed")
    if len(info.d) != len(calibration.distortion_coefficients) or len(info.k) != 9:
        raise ValueError("live CameraInfo D or K shape changed")
    if any(not isclose(a, b, abs_tol=1e-6, rel_tol=0.0)
           for a, b in zip(info.d, calibration.distortion_coefficients)):
        raise ValueError("live distortion coefficients changed")
    if any(not isclose(a, b, abs_tol=1e-3, rel_tol=0.0)
           for a, b in zip(info.k, calibration.k)):
        raise ValueError("live K matrix changed")


def _run(calibration_path: Path, scene_path: Path, output: Path | None,
         timeout: float) -> None:
    calibration = Calibration.from_file(calibration_path)
    mapper = PlanarMapper(calibration)
    scene = load_scene(str(scene_path))
    table = scene["table"]
    table_top = table["pose"]["z"] + table["size"]["z"] / 2.0
    if not isclose(calibration.plane_z_m, table_top, abs_tol=1e-9):
        raise ValueError("calibrated plane differs from the canonical tabletop")
    if calibration.plane_frame != scene["robot"]["world_frame"]:
        raise ValueError("calibrated frame differs from the workcell world frame")

    rclpy.init()
    node = CameraSamples()
    try:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and (node.image is None or node.info is None) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.5, max(0.0, deadline - time.monotonic())))
        if node.image is None or node.info is None:
            raise RuntimeError("RGB image or CameraInfo did not arrive")
        _check_camera_info(calibration, node.info)
        if (node.image.width, node.image.height, node.image.header.frame_id) != (
            calibration.image_width, calibration.image_height, calibration.image_frame,
        ):
            raise ValueError("live RGB image dimensions or frame changed")
        frame = CvBridge().imgmsg_to_cv2(node.image, desired_encoding="bgr8")
        if frame.shape != (calibration.image_height, calibration.image_width, 3):
            raise ValueError("RGB frame has unexpected OpenCV shape")

        print("image_to_world_h:")
        print(mapper.image_to_world_h)
        print(f"calibrated plane: {calibration.plane_frame} z={calibration.plane_z_m:.3f} m")
        calibration_errors = []
        for (name, u, v), (_, x, y) in zip(calibration.image_points, calibration.world_points):
            mapped = mapper.pixel_to_world_xy(u, v)
            error = hypot(mapped[0] - x, mapped[1] - y)
            calibration_errors.append(error)
            print(f"calibration {name}: ({u:.3f}, {v:.3f}) px -> {mapped}; error={error*1000:.3f} mm")
            cv2.circle(frame, (round(u), round(v)), 4, (255, 255, 255), 1)

        zone_errors = []
        for name, u, v in calibration.validation_pixels:
            if name not in scene["zones"]:
                raise ValueError(f"unknown fixed validation zone {name}")
            pose = scene["zones"][name]["pose"]
            expected = (float(pose["x"]), float(pose["y"]))
            mapped = mapper.pixel_to_world_xy(u, v)
            error = hypot(mapped[0] - expected[0], mapped[1] - expected[1])
            zone_errors.append(error)
            print(f"validation {name}: ({u:.1f}, {v:.1f}) px -> {mapped}; "
                  f"scene XY={expected}; error={error*1000:.3f} mm")
            cv2.circle(frame, (round(u), round(v)), 5, (255, 255, 255), 1)
            cv2.putText(frame, name, (round(u) - 50, round(v) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        round_trip_errors = []
        for x, y in ((0.0, 0.38), (-0.20, 0.30), (0.20, 0.50)):
            u, v = mapper.world_xy_to_pixel(x, y)
            recovered = mapper.pixel_to_world_xy(u, v)
            round_trip_errors.append(hypot(recovered[0] - x, recovered[1] - y))
        print(f"validation mean={sum(zone_errors)/len(zone_errors)*1000:.3f} mm, "
              f"max={max(zone_errors)*1000:.3f} mm; "
              f"round-trip max={max(round_trip_errors)*1000:.6f} mm")
        if max(zone_errors) > 0.010:
            raise RuntimeError("fixed-reference XY error exceeds 10 mm")
        if output is not None and not cv2.imwrite(str(output), frame):
            raise OSError(f"could not save {output}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Validate tabletop homography on fixed scene references")
    package = Path(get_package_share_directory("ur3_perception_llm_control"))
    parser.add_argument("--calibration", type=Path, default=package / "config" / "camera_calibration.yaml")
    parser.add_argument("--scene", type=Path, default=package / "config" / "scene.yaml")
    parser.add_argument("--output", type=Path, help="optional single annotated RGB frame")
    parser.add_argument("--timeout", type=float, default=15.0)
    parsed = parser.parse_args(args)
    if parsed.timeout <= 0:
        parser.error("--timeout must be positive")
    _run(parsed.calibration, parsed.scene, parsed.output, parsed.timeout)


if __name__ == "__main__":
    main()
