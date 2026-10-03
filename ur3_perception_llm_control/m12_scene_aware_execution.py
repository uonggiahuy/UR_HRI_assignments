"""Live Assignment 03 natural-language planning and physical execution."""

from __future__ import annotations

import argparse
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
import yaml

from ur3_perception_llm_control.cube_detection_test import FrameCollector
from ur3_perception_llm_control.cube_detector import CubeDetector, PerceptionConfig, evaluate_stability
from ur3_perception_llm_control.assignment3_m12_transaction import (
    plan_and_execute, verify_placement_observation,
)
from ur3_perception_llm_control.gripper import FINGER_JOINTS, OPEN_POSITION, ParallelJawGripper
from ur3_perception_llm_control.motion_primitives import ManipulationMotionPrimitives
from ur3_perception_llm_control.moveit_interface import ARM_JOINTS, MoveItArmInterface
from ur3_perception_llm_control.perception_scene import PerceptionPlanningSceneSynchronizer
from ur3_perception_llm_control.perception_state import PerceptionSnapshot, WorkcellGeometry
from ur3_perception_llm_control.physical_grasp import PhysicalGraspManager
from ur3_perception_llm_control.planar_mapper import Calibration, CubeTopMapper, PlanarMapper
from ur3_perception_llm_control.planning_scene import PlanningSceneManager
from ur3_perception_llm_control.robot_skills import RobotSkills
from ur3_perception_llm_control.scene_aware_planner import SceneAwareLLMPlanner, build_scene_context, extract_placement_goal
from ur3_perception_llm_control.skill_executor import SkillExecutor
from ur3_perception_llm_control.task_validator import TaskStatus
from ur3_perception_llm_control.temporary_position import TemporaryPositionPlanner
from ur3_perception_llm_control.workcell_scene import load_scene
from ur3_perception_llm_control.world_state import BLOCKS, WorldState


HOME_JOINT_TOLERANCE_RAD = 0.03
FINGER_OPEN_TOLERANCE_M = 0.003


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
    node = FrameCollector(detector, config.frames, node_name="m12_scene_aware_execution",
                          use_sim_time=True)
    interface = manager = gripper = physical = skills = state = None
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
        initial_report = synchronizer.apply_snapshot(initial, now_sec())
        if initial_report.attached_ids:
            raise RuntimeError("stale MoveIt attachment before motion")
        state = WorldState.from_perception_snapshot(initial, now_sec(), geometry)
        goal = extract_placement_goal(options.command)
        print(f"[PERCEPTION]\n{build_scene_context(state)}", flush=True)
        print(f"[USER] {options.command}", flush=True)
        print(f"[SCENE] initial sync error={initial_report.max_sync_error_m*1000:.3f}mm", flush=True)
        if goal is None:
            raise RuntimeError("unsupported or low-level request; no LLM call or motion")
        planner = SceneAwareLLMPlanner.from_environment()
        print(f"[MODEL] {planner._config.model}", flush=True)
        selected = [None]

        def on_success(index: int, step: dict[str, str]) -> None:
            if step["skill"] == "pick" and skills.reserved_temporary is not None:
                selected[0] = skills.reserved_temporary
            observation = skills.last_verified_snapshot
            print(f"[EXECUTION] step {index}: {step}", flush=True)
            if step["skill"] in ("place", "place_temp"):
                verify_placement_observation(observation, state, physical.ready_for_pick())
                print(f"[CAMERA VERIFY] {step['object']}={observation.object_locations[step['object']]} "
                      f"{goal.destination_zone}={observation.zone_occupancy[goal.destination_zone] or 'empty'}",
                      flush=True)

        def before_execute(current: PerceptionSnapshot) -> None:
            report = synchronizer.apply_snapshot(current, now_sec())
            if report.attached_ids or set(report.authoritative_xyz) != set(BLOCKS):
                raise RuntimeError("STALE_PLAN: authoritative MoveIt scene changed")
            if not physical.ready_for_pick():
                raise RuntimeError("STALE_PLAN: physical attachment changed")
            print("[STALE WORLD] PASS", flush=True)

        executor = SkillExecutor(skills, state, snapshot_source=snapshot_source,
                                 now_sec=now_sec, on_step_success=on_success)
        def on_validated(raw_candidate: str) -> None:
            print(f"[LLM RAW] {raw_candidate}", flush=True)
            print("[VALIDATOR] PASS\n[GOAL CHECK] PASS", flush=True)

        raw, result = plan_and_execute(options.command, initial, geometry, state, planner,
                                       executor, snapshot_source, now_sec, before_execute,
                                       on_validated)
        if result.status != TaskStatus.SUCCESS:
            raise RuntimeError(f"execution failed after {result.completed_steps}: {result.message}")
        final = snapshot_source()
        report = synchronizer.apply_snapshot(final, now_sec())
        joints = interface.current_joint_positions()
        home_error = (max(abs(joints[name] - float(motion["home"][name])) for name in ARM_JOINTS)
                      if joints is not None and all(name in joints for name in ARM_JOINTS)
                      else float("inf"))
        if (dict(final.object_locations) != state.object_locations
                or dict(final.zone_occupancy) != state.zone_occupancy
                or final.object_locations[goal.object_name] != goal.destination_zone
                or state.held_object is not None
                or set(report.authoritative_xyz) != set(BLOCKS) or report.attached_ids
                or not physical.ready_for_pick()
                or joints is None
                or home_error > HOME_JOINT_TOLERANCE_RAD
                or any(name not in joints or abs(joints[name] - OPEN_POSITION) > FINGER_OPEN_TOLERANCE_M
                       for name in FINGER_JOINTS)):
            raise RuntimeError("final camera, task, MoveIt, or physical state disagrees")
        print(f"[TEMP] {selected[0]}", flush=True)
        print(f"[FINAL RGB] locations={dict(final.object_locations)} zones={dict(final.zone_occupancy)}", flush=True)
        print(f"[FINAL MOVEIT] WORLD={sorted(report.authoritative_xyz)} "
              f"ATTACHED={sorted(report.attached_ids)} error={report.max_sync_error_m*1000:.3f}mm", flush=True)
        print(f"[FINAL] TASK SUCCESS; physical attachment detached; gripper open; "
              f"HOME max error={home_error:.6f}rad", flush=True)
    except Exception:
        if state is not None:
            print(f"[STOP] expected task state: objects={state.object_locations} "
                  f"zones={state.zone_occupancy} held={state.held_object}", flush=True)
        if skills is not None and skills.last_verified_snapshot is not None:
            observed = skills.last_verified_snapshot
            print(f"[STOP] last verified RGB: objects={dict(observed.object_locations)} "
                  f"zones={dict(observed.zone_occupancy)}", flush=True)
        if manager is not None:
            try:
                scene_now = manager.get(timeout=2.0)
            except (RuntimeError, ValueError):
                scene_now = None
            if scene_now is not None:
                world = {item.id for item in scene_now.world.collision_objects} & set(BLOCKS)
                attached = {item.object.id for item in scene_now.robot_state.attached_collision_objects}
                print(f"[STOP] MoveIt WORLD={sorted(world)} ATTACHED={sorted(attached)}", flush=True)
        if physical is not None:
            print(f"[STOP] physical baseline ready={physical.ready_for_pick()} "
                  f"attached={sorted(name for name in BLOCKS if physical.is_attached(name))}", flush=True)
        raise
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
    parser = argparse.ArgumentParser(description="Assignment 03 LLM-driven physical task")
    parser.add_argument("command", help="natural-language placement request")
    parser.add_argument("--scene", type=Path, default=share / "config/scene.yaml")
    parser.add_argument("--motion", type=Path, default=share / "config/robot_motion.yaml")
    parser.add_argument("--temporary", type=Path, default=share / "config/temporary_position.yaml")
    parser.add_argument("--perception", type=Path, default=share / "config/perception.yaml")
    parser.add_argument("--calibration", type=Path, default=share / "config/camera_calibration.yaml")
    parser.add_argument("--camera-timeout", type=float, default=30.0)
    options = parser.parse_args(argv)
    if options.camera_timeout <= 0:
        parser.error("camera timeout must be positive")
    if extract_placement_goal(options.command) is None:
        print("[FINAL] TASK REJECTED: unsupported or low-level request; no LLM call or motion",
              flush=True)
        raise SystemExit(1)
    try:
        run(options)
    except RuntimeError as error:
        print(f"[FINAL] TASK FAILED: {error}", flush=True)
        raise SystemExit(1) from error


if __name__ == "__main__":
    main()
