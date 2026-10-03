"""Read live RGB and request a validated M11 plan without robot execution."""

from __future__ import annotations

import argparse
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
import rclpy

from ur3_perception_llm_control.cube_detection_test import FrameCollector
from ur3_perception_llm_control.cube_detector import CubeDetector, PerceptionConfig, evaluate_stability
from ur3_perception_llm_control.llm_planner import PlannerConfigurationError
from ur3_perception_llm_control.perception_state import PerceptionSnapshot, WorkcellGeometry
from ur3_perception_llm_control.planar_mapper import Calibration, CubeTopMapper, PlanarMapper
from ur3_perception_llm_control.scene_aware_planner import (
    SceneAwareLLMPlanner, build_scene_context, extract_placement_goal, simulate_goal,
)
from ur3_perception_llm_control.world_state import WorldState


def run(options: argparse.Namespace) -> bool:
    geometry = WorkcellGeometry.from_scene_file(options.scene)
    config = PerceptionConfig.from_file(options.perception)
    calibration = Calibration.from_file(options.calibration)
    detector = CubeDetector(config, CubeTopMapper(PlanarMapper(calibration),
                                                 config.camera_xyz, config.cube_top_z_m))
    rclpy.init()
    node = FrameCollector(detector, config.frames, node_name="m11_scene_aware_planner_test",
                          use_sim_time=True)
    try:
        deadline = time.monotonic() + options.camera_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if len(node.samples) == config.frames and evaluate_stability(list(node.samples), config).stable:
                break
        else:
            raise RuntimeError(f"No stable RGB snapshot: {node.failures[-3:]}")
        now = node.get_clock().now().nanoseconds / 1e9
        snapshot = PerceptionSnapshot.from_detections(node.samples[-1], geometry)
        state = WorldState.from_perception_snapshot(snapshot, now, geometry)
        print(build_scene_context(state))
        print(f"Command: {options.command}")
        if extract_placement_goal(options.command) is None:
            print("FAIL: unsupported or ambiguous placement request; no LLM call or robot motion")
            return False
        try:
            planner = SceneAwareLLMPlanner.from_environment()
        except PlannerConfigurationError as error:
            print(f"FAIL: {error}; no LLM call or robot motion")
            return False
        print(f"Model: {planner._config.model}")
        result = planner.plan(options.command, snapshot, now, geometry)
        print(f"Raw candidate: {result.candidate_json}")
        print(f"Validation: {result.status.value}: {result.message}")
        if result.accepted:
            goal = extract_placement_goal(options.command)
            assert goal is not None and result.validation is not None
            simulated = state.copy()
            for step in result.validation.steps:
                _, next_state = simulate_goal((step,), simulated, goal)
                simulated = next_state
                print(f"{step} -> target={simulated.object_locations[goal.object_name]}, "
                      f"destination={simulated.zone_occupancy[goal.destination_zone] or 'empty'}")
            satisfied, _ = simulate_goal(result.validation.steps, state, goal)
            print(f"PASS: goal satisfied={satisfied}; no robot motion")
        else:
            print("FAIL: no executable plan; no robot motion")
        return result.accepted
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv: list[str] | None = None) -> None:
    share = Path(get_package_share_directory("ur3_perception_llm_control"))
    parser = argparse.ArgumentParser(description="M11 RGB scene-aware planner, planning only")
    parser.add_argument("command", help="one natural-language placement command")
    parser.add_argument("--scene", type=Path, default=share / "config/scene.yaml")
    parser.add_argument("--perception", type=Path, default=share / "config/perception.yaml")
    parser.add_argument("--calibration", type=Path, default=share / "config/camera_calibration.yaml")
    parser.add_argument("--camera-timeout", type=float, default=30.0)
    options = parser.parse_args(argv)
    if options.camera_timeout <= 0:
        parser.error("camera timeout must be positive")
    if not run(options):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
