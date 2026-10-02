"""Live M5 diagnostic: RGB -> M4 detections -> immutable symbolic snapshot."""

from __future__ import annotations

import argparse
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
import rclpy

from ur3_perception_llm_control.cube_detection_test import FrameCollector
from ur3_perception_llm_control.cube_detector import CUBE_NAMES, CubeDetector, PerceptionConfig, evaluate_stability
from ur3_perception_llm_control.perception_state import (
    DEFAULT_MAX_AGE_SEC, PerceptionSnapshot, WorkcellGeometry,
)
from ur3_perception_llm_control.planar_mapper import Calibration, CubeTopMapper, PlanarMapper


def run(args: argparse.Namespace) -> None:
    calibration = Calibration.from_file(args.calibration)
    config = PerceptionConfig.from_file(args.perception)
    mapper = CubeTopMapper(PlanarMapper(calibration), config.camera_xyz, config.cube_top_z_m)
    detector = CubeDetector(config, mapper)
    geometry = WorkcellGeometry.from_scene_file(args.scene)
    rclpy.init()
    node = FrameCollector(detector, config.frames, node_name="perception_state_test", use_sim_time=True)
    try:
        deadline = time.monotonic() + args.timeout
        while rclpy.ok() and len(node.samples) < config.frames and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
        if len(node.samples) < config.frames:
            raise RuntimeError(f"only {len(node.samples)}/{config.frames} complete RGB frames; "
                               f"failures={node.failures[-3:]}")
        stability = evaluate_stability(list(node.samples), config)
        if not stability.stable:
            raise RuntimeError(f"unstable RGB observations: {stability.max_spread_m*1000:.3f} mm")
        snapshot = PerceptionSnapshot.from_detections(node.samples[-1], geometry)
        now_sec = node.get_clock().now().nanoseconds / 1e9
        snapshot.require_fresh(now_sec, args.max_age)
        print(f"RGB frames={config.frames}, stable=True, max_xy_spread={stability.max_spread_m*1000:.3f} mm")
        print("OBJECTS")
        for name in CUBE_NAMES:
            x, y = snapshot.object_world_xy[name]
            print(f"{name}: {snapshot.object_locations[name]} XY=({x:.5f}, {y:.5f}) m")
        print("ZONES")
        for name in sorted(snapshot.zone_occupancy):
            print(f"{name}: {snapshot.zone_occupancy[name] or 'empty'}")
        print(f"observation_timestamp={snapshot.observation_timestamp_sec:.3f} s "
              f"observation_age={now_sec-snapshot.observation_timestamp_sec:.3f} s "
              f"fresh=True max_age={args.max_age:.3f} s held_object=None")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv: list[str] | None = None) -> None:
    share = Path(get_package_share_directory("ur3_perception_llm_control"))
    parser = argparse.ArgumentParser(description="M5 camera-derived five-cube symbolic state")
    parser.add_argument("--calibration", type=Path, default=share / "config/camera_calibration.yaml")
    parser.add_argument("--perception", type=Path, default=share / "config/perception.yaml")
    parser.add_argument("--scene", type=Path, default=share / "config/scene.yaml",
                        help="fixed table/zone geometry and cube dimensions only")
    parser.add_argument("--max-age", type=float, default=DEFAULT_MAX_AGE_SEC)
    parser.add_argument("--timeout", type=float, default=25.0)
    args = parser.parse_args(argv)
    if args.timeout <= 0 or args.max_age <= 0:
        parser.error("timeout and max-age must be positive")
    run(args)


if __name__ == "__main__":
    main()
