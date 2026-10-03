"""Live manual blocker plan through the Assignment 03 validator and executor."""

from __future__ import annotations

import argparse
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
import yaml

from ur3_perception_llm_control.cube_detection_test import FrameCollector
from ur3_perception_llm_control.cube_detector import CubeDetector, PerceptionConfig, evaluate_stability
from ur3_perception_llm_control.gripper import ParallelJawGripper
from ur3_perception_llm_control.motion_primitives import ManipulationMotionPrimitives
from ur3_perception_llm_control.moveit_interface import ARM_JOINTS, MoveItArmInterface
from ur3_perception_llm_control.perception_scene import PerceptionPlanningSceneSynchronizer
from ur3_perception_llm_control.perception_state import PerceptionSnapshot, WorkcellGeometry
from ur3_perception_llm_control.physical_grasp import PhysicalGraspManager
from ur3_perception_llm_control.planar_mapper import Calibration, CubeTopMapper, PlanarMapper
from ur3_perception_llm_control.planning_scene import PlanningSceneManager
from ur3_perception_llm_control.robot_skills import RobotSkills
from ur3_perception_llm_control.skill_executor import SkillExecutor
from ur3_perception_llm_control.task_validator import TaskStatus, TaskValidator
from ur3_perception_llm_control.temporary_position import TemporaryPositionPlanner
from ur3_perception_llm_control.workcell_scene import load_scene
from ur3_perception_llm_control.world_state import BLOCKS, WorldState


MANUAL_PLAN = {"plan": [
    {"skill": "pick", "object": "blue_cube"},
    {"skill": "place_temp", "object": "blue_cube"},
    {"skill": "pick", "object": "red_cube"},
    {"skill": "place", "object": "red_cube", "zone": "zone_b"},
    {"skill": "home"},
]}


def _yaml(path: Path) -> dict:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"{path} must be a mapping")
    return data


def run(options: argparse.Namespace) -> None:
    scene = load_scene(str(options.scene))
    motion = _yaml(options.motion)
    geometry = WorkcellGeometry.from_scene_file(options.scene)
    config = PerceptionConfig.from_file(options.perception)
    calibration = Calibration.from_file(options.calibration)
    detector = CubeDetector(config, CubeTopMapper(PlanarMapper(calibration),
                                                 config.camera_xyz, config.cube_top_z_m))
    rclpy.init()
    node = FrameCollector(detector, config.frames, node_name="m10_blocker_execution_test",
                          use_sim_time=True)
    interface = manager = gripper = physical = None
    try:
        manipulation = motion["manipulation"]
        interface = MoveItArmInterface(
            node, planning_group=str(motion["planning_group"]),
            planning_frame=str(motion["planning_frame"]),
            planning_tip=str(motion["planning_tip"]),
            application_tip=str(motion["application_tip"]),
            tool0_to_tcp_z=float(motion["tool0_to_tcp_z"]),
            velocity_scaling=float(motion["velocity_scaling"]),
            acceleration_scaling=float(motion["acceleration_scaling"]),
            planning_time=float(motion["planning_time"]),
            planning_attempts=int(motion["planning_attempts"]),
        )
        manager = PlanningSceneManager(
            node, scene, attachment_link=str(manipulation["attachment_link"]),
            touch_links=tuple(str(link) for link in manipulation["touch_links"]),
        )
        gripper = ParallelJawGripper(node)
        physical = PhysicalGraspManager(node)
        synchronizer = PerceptionPlanningSceneSynchronizer(manager, scene, geometry)
        last_stamp = [-1.0]

        def now_sec() -> float:
            return node.get_clock().now().nanoseconds / 1e9

        def snapshot_source() -> PerceptionSnapshot:
            deadline = time.monotonic() + options.camera_timeout
            while rclpy.ok() and time.monotonic() < deadline:
                if (len(node.samples) == config.frames and node.timestamps[-1] > last_stamp[0]
                        and evaluate_stability(list(node.samples), config).stable):
                    snapshot = PerceptionSnapshot.from_detections(node.samples[-1], geometry)
                    snapshot.require_fresh(now_sec())
                    last_stamp[0] = snapshot.observation_timestamp_sec
                    return snapshot
                rclpy.spin_once(node, timeout_sec=0.1)
            raise RuntimeError(f"No stable fresh RGB snapshot: {node.failures[-3:]}")

        if set(motion["home"]) != set(ARM_JOINTS):
            raise RuntimeError("HOME must define exactly the six arm joints")
        primitives = ManipulationMotionPrimitives(interface, scene, motion)
        skills = RobotSkills(
            interface=interface, gripper=gripper, primitives=primitives,
            scene_manager=manager, physical_grasp=physical,
            snapshot_source=snapshot_source, scene_synchronizer=synchronizer,
            now_sec=now_sec, home_configuration=motion["home"],
            temporary_planner=TemporaryPositionPlanner(scene, geometry, _yaml(options.temporary)),
            temporary_scene=scene, temporary_motion=motion,
            require_place_verification=True,
        )
        if not interface.wait_until_ready() or not physical.wait_until_ready():
            raise RuntimeError("MoveIt, controllers, or physical joint baseline unavailable")
        if not physical.ready_for_pick():
            raise RuntimeError("stale physical attachment before motion")
        initial = snapshot_source()
        if initial.object_locations["blue_cube"] != "zone_b" or initial.object_locations["red_cube"] != "table":
            raise RuntimeError("manual blocker layout not observed from RGB")
        initial_report = synchronizer.apply_snapshot(initial, now_sec())
        if initial_report.attached_ids:
            raise RuntimeError("stale MoveIt attachment before motion")
        state = WorldState.from_perception_snapshot(initial, now_sec(), geometry)
        validator = TaskValidator(assignment03=True)
        direct = validator.validate({"plan": [
            {"skill": "pick", "object": "red_cube"},
            {"skill": "place", "object": "red_cube", "zone": "zone_b"},
            {"skill": "home"},
        ]}, state)
        if direct.status != TaskStatus.ZONE_OCCUPIED:
            raise RuntimeError("occupied direct-place plan was not rejected")
        validation = validator.validate(MANUAL_PLAN, state)
        if not validation.accepted:
            raise RuntimeError(f"manual blocker plan rejected: {validation.status}: {validation.message}")
        simulated = state.copy()
        for index, step in enumerate(validation.steps, 1):
            validator._simulate(step, simulated)
            print(f"VALIDATED {index} {step}: held={simulated.held_object} "
                  f"blue={simulated.object_locations['blue_cube']} "
                  f"red={simulated.object_locations['red_cube']} "
                  f"zone_b={simulated.zone_occupancy['zone_b']}", flush=True)
        print(f"INITIAL RGB: blue={initial.object_world_xy['blue_cube']} zone_b; "
              f"red={initial.object_world_xy['red_cube']} table; "
              f"scene_error={initial_report.max_sync_error_m*1000:.3f}mm", flush=True)
        selected = [None]

        def on_success(index: int, step: dict[str, str]) -> None:
            if step["skill"] == "pick" and step["object"] == "blue_cube":
                selected[0] = skills.reserved_temporary
            observation = skills.last_verified_snapshot
            print(f"EXECUTED {index} {step}: held={state.held_object} "
                  f"zone_b={state.zone_occupancy['zone_b']} "
                  f"camera_blue={observation.object_locations['blue_cube'] if observation else 'pending'} "
                  f"camera_red={observation.object_locations['red_cube'] if observation else 'pending'}",
                  flush=True)

        result = SkillExecutor(skills, state, snapshot_source=snapshot_source,
                               now_sec=now_sec, on_step_success=on_success).execute(validation)
        if result.status != TaskStatus.SUCCESS:
            raise RuntimeError(f"blocker execution failed after {result.completed_steps}: {result.message}")
        final = snapshot_source()
        report = synchronizer.apply_snapshot(final, now_sec())
        if (final.object_locations["blue_cube"] != "table"
                or final.object_locations["red_cube"] != "zone_b"
                or final.zone_occupancy["zone_b"] != "red_cube"
                or state.held_object is not None or state.zone_occupancy["zone_b"] != "red_cube"
                or set(report.authoritative_xyz) != set(BLOCKS) or report.attached_ids
                or not physical.ready_for_pick()):
            raise RuntimeError("final camera, task, MoveIt, or physical state disagrees")
        print(f"SELECTED TEMP: {selected[0]}", flush=True)
        print(f"FINAL RGB: blue={final.object_locations['blue_cube']} {final.object_world_xy['blue_cube']}; "
              f"red={final.object_locations['red_cube']} {final.object_world_xy['red_cube']}; "
              f"zone_b={final.zone_occupancy['zone_b']}", flush=True)
        print(f"FINAL MOVEIT: WORLD={sorted(report.authoritative_xyz)} "
              f"ATTACHED={sorted(report.attached_ids)} error={report.max_sync_error_m*1000:.3f}mm", flush=True)
        print("M10 LIVE PASS", flush=True)
    finally:
        if physical is not None:
            physical.destroy()
        if gripper is not None:
            gripper.destroy()
        if manager is not None:
            manager.destroy()
        if interface is not None:
            interface.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv: list[str] | None = None) -> None:
    share = Path(get_package_share_directory("ur3_perception_llm_control"))
    parser = argparse.ArgumentParser(description="M10 manual physical blocker plan")
    parser.add_argument("--scene", type=Path, default=share / "config/scene.yaml")
    parser.add_argument("--motion", type=Path, default=share / "config/robot_motion.yaml")
    parser.add_argument("--temporary", type=Path, default=share / "config/temporary_position.yaml")
    parser.add_argument("--perception", type=Path, default=share / "config/perception.yaml")
    parser.add_argument("--calibration", type=Path, default=share / "config/camera_calibration.yaml")
    parser.add_argument("--camera-timeout", type=float, default=30.0)
    options = parser.parse_args(argv)
    if options.camera_timeout <= 0:
        parser.error("camera timeout must be positive")
    run(options)


if __name__ == "__main__":
    main()
