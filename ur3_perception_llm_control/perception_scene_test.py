"""Live M6 RGB-to-authoritative-MoveIt scene diagnostic; no robot motion."""

from __future__ import annotations

import argparse
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
import rclpy

from ur3_perception_llm_control.cube_detection_test import FrameCollector
from ur3_perception_llm_control.cube_detector import CUBE_NAMES, CubeDetector, PerceptionConfig, evaluate_stability
from ur3_perception_llm_control.perception_scene import PerceptionPlanningSceneSynchronizer
from ur3_perception_llm_control.perception_state import (
    DEFAULT_MAX_AGE_SEC, PerceptionSnapshot, WorkcellGeometry,
)
from ur3_perception_llm_control.planar_mapper import Calibration, CubeTopMapper, PlanarMapper
from ur3_perception_llm_control.planning_scene import PlanningSceneManager
from ur3_perception_llm_control.workcell_scene import load_scene


def run(args: argparse.Namespace) -> None:
    calibration = Calibration.from_file(args.calibration)
    config = PerceptionConfig.from_file(args.perception)
    geometry = WorkcellGeometry.from_scene_file(args.scene)
    scene = load_scene(str(args.scene))
    mapper = CubeTopMapper(PlanarMapper(calibration), config.camera_xyz, config.cube_top_z_m)
    detector = CubeDetector(config, mapper)
    rclpy.init()
    node = FrameCollector(detector, config.frames, node_name="perception_scene_test", use_sim_time=True)
    manager = None
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
        manager = PlanningSceneManager(node, scene)
        synchronizer = PerceptionPlanningSceneSynchronizer(manager, scene, geometry)
        now_sec = node.get_clock().now().nanoseconds / 1e9
        report = synchronizer.apply_snapshot(snapshot, now_sec, args.max_age)
        if report.attached_ids:
            raise RuntimeError(f"M6 static-scene acceptance expected no attachments: {sorted(report.attached_ids)}")
        print(f"RGB frames={config.frames}, stable=True, snapshot_age={now_sec-snapshot.observation_timestamp_sec:.3f} s")
        print("CUBE  LOCATION  CAMERA_XY  REQUESTED_MOVEIT_XYZ  AUTHORITATIVE_MOVEIT_XYZ  SYNC_ERROR")
        for name in CUBE_NAMES:
            xy = snapshot.object_world_xy[name]
            requested = report.requested_xyz[name]
            actual = report.authoritative_xyz[name]
            print(f"{name} {snapshot.object_locations[name]} ({xy[0]:.5f},{xy[1]:.5f}) "
                  f"({requested[0]:.5f},{requested[1]:.5f},{requested[2]:.5f}) "
                  f"({actual[0]:.5f},{actual[1]:.5f},{actual[2]:.5f}) "
                  f"{report.sync_errors_m[name]*1000:.3f} mm")
        print(f"M6 PASS: 5 WORLD cubes, 0 attached, static objects verified, visual-only zones absent; "
              f"max_sync_error={report.max_sync_error_m*1000:.3f} mm "
              f"service_accepted={report.service_accepted}")
    finally:
        if manager is not None:
            manager.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv: list[str] | None = None) -> None:
    share = Path(get_package_share_directory("ur3_perception_llm_control"))
    parser = argparse.ArgumentParser(description="M6 camera-derived MoveIt collision-scene diagnostic")
    parser.add_argument("--calibration", type=Path, default=share / "config/camera_calibration.yaml")
    parser.add_argument("--perception", type=Path, default=share / "config/perception.yaml")
    parser.add_argument("--scene", type=Path, default=share / "config/scene.yaml")
    parser.add_argument("--max-age", type=float, default=DEFAULT_MAX_AGE_SEC)
    parser.add_argument("--timeout", type=float, default=25.0)
    args = parser.parse_args(argv)
    if args.timeout <= 0 or args.max_age <= 0:
        parser.error("timeout and max-age must be positive")
    run(args)


if __name__ == "__main__":
    main()
