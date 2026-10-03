"""Persistent, fail-closed Assignment 03 interactive runtime."""

from __future__ import annotations

import argparse
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
import rclpy

from ur3_perception_llm_control.assignment3_m12_transaction import (
    plan_and_execute,
    verify_placement_observation,
)
from ur3_perception_llm_control.cube_detection_test import FrameCollector
from ur3_perception_llm_control.cube_detector import (
    CubeDetector,
    PerceptionConfig,
    evaluate_stability,
)
from ur3_perception_llm_control.gripper import FINGER_JOINTS, OPEN_POSITION, ParallelJawGripper
from ur3_perception_llm_control.m12_scene_aware_execution import (
    FINGER_OPEN_TOLERANCE_M,
    HOME_JOINT_TOLERANCE_RAD,
    _yaml,
)
from ur3_perception_llm_control.motion_primitives import ManipulationMotionPrimitives
from ur3_perception_llm_control.moveit_interface import ARM_JOINTS, MoveItArmInterface
from ur3_perception_llm_control.perception_scene import PerceptionPlanningSceneSynchronizer
from ur3_perception_llm_control.perception_state import PerceptionSnapshot, WorkcellGeometry
from ur3_perception_llm_control.physical_grasp import PhysicalGraspManager
from ur3_perception_llm_control.planar_mapper import Calibration, CubeTopMapper, PlanarMapper
from ur3_perception_llm_control.planning_scene import PlanningSceneManager
from ur3_perception_llm_control.robot_skills import RobotSkills
from ur3_perception_llm_control.scene_aware_planner import (
    SceneAwareLLMPlanner,
    build_scene_context,
    extract_placement_goal,
)
from ur3_perception_llm_control.skill_executor import SkillExecutor
from ur3_perception_llm_control.task_validator import TaskStatus
from ur3_perception_llm_control.temporary_position import TemporaryPositionPlanner
from ur3_perception_llm_control.workcell_scene import load_scene
from ur3_perception_llm_control.world_state import BLOCKS, WorldState


class Assignment3Runtime:
    """Own long-lived ROS resources and create one isolated task per command."""

    def __init__(self, options: argparse.Namespace) -> None:
        self._options = options
        self._scene = load_scene(str(options.scene))
        self._motion = _yaml(options.motion)
        self._geometry = WorkcellGeometry.from_scene_file(options.scene)
        perception = PerceptionConfig.from_file(options.perception)
        calibration = Calibration.from_file(options.calibration)
        detector = CubeDetector(
            perception,
            CubeTopMapper(PlanarMapper(calibration), perception.camera_xyz, perception.cube_top_z_m),
        )
        self._perception = perception
        self._last_stamp = -1.0
        self._node: FrameCollector | None = None
        self._interface: MoveItArmInterface | None = None
        self._manager: PlanningSceneManager | None = None
        self._gripper: ParallelJawGripper | None = None
        self._physical: PhysicalGraspManager | None = None
        self._skills: RobotSkills | None = None
        self._synchronizer: PerceptionPlanningSceneSynchronizer | None = None
        self._planner: SceneAwareLLMPlanner | None = None
        self._detector = detector

    def start(self) -> None:
        """Initialize all ROS and grasp resources once for this process."""
        rclpy.init()
        self._node = FrameCollector(self._detector, self._perception.frames,
                                    node_name="assignment3_runtime", use_sim_time=True)
        manipulation = self._motion["manipulation"]
        self._interface = MoveItArmInterface(
            self._node, planning_group=str(self._motion["planning_group"]),
            planning_frame=str(self._motion["planning_frame"]),
            planning_tip=str(self._motion["planning_tip"]),
            application_tip=str(self._motion["application_tip"]),
            tool0_to_tcp_z=float(self._motion["tool0_to_tcp_z"]),
            velocity_scaling=float(self._motion["velocity_scaling"]),
            acceleration_scaling=float(self._motion["acceleration_scaling"]),
            planning_time=float(self._motion["planning_time"]),
            planning_attempts=int(self._motion["planning_attempts"]),
        )
        self._manager = PlanningSceneManager(
            self._node, self._scene, attachment_link=str(manipulation["attachment_link"]),
            touch_links=tuple(str(link) for link in manipulation["touch_links"]),
        )
        self._gripper = ParallelJawGripper(self._node)
        self._physical = PhysicalGraspManager(self._node)
        self._synchronizer = PerceptionPlanningSceneSynchronizer(
            self._manager, self._scene, self._geometry)
        if set(self._motion["home"]) != set(ARM_JOINTS):
            raise RuntimeError("HOME must define exactly the six arm joints")
        self._skills = RobotSkills(
            interface=self._interface, gripper=self._gripper,
            primitives=ManipulationMotionPrimitives(self._interface, self._scene, self._motion),
            scene_manager=self._manager, physical_grasp=self._physical,
            snapshot_source=self._snapshot_source, scene_synchronizer=self._synchronizer,
            now_sec=self._now_sec, home_configuration=self._motion["home"],
            temporary_planner=TemporaryPositionPlanner(
                self._scene, self._geometry, _yaml(self._options.temporary)),
            temporary_scene=self._scene, temporary_motion=self._motion,
            require_place_verification=True,
        )
        # This is deliberately the sole DetachableJoint baseline reset.
        if not self._interface.wait_until_ready() or not self._physical.wait_until_ready():
            raise RuntimeError("MoveIt, controllers, or physical joint baseline unavailable")
        if not self._physical.ready_for_pick():
            raise RuntimeError("stale physical attachment before runtime start")
        self._planner = SceneAwareLLMPlanner.from_environment()

    def _now_sec(self) -> float:
        assert self._node is not None
        return self._node.get_clock().now().nanoseconds / 1e9

    def _snapshot_source(self) -> PerceptionSnapshot:
        assert self._node is not None
        deadline = time.monotonic() + self._options.camera_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if (len(self._node.samples) == self._perception.frames
                    and self._node.timestamps[-1] > self._last_stamp
                    and evaluate_stability(list(self._node.samples), self._perception).stable):
                snapshot = PerceptionSnapshot.from_detections(self._node.samples[-1], self._geometry)
                snapshot.require_fresh(self._now_sec())
                self._last_stamp = snapshot.observation_timestamp_sec
                return snapshot
            rclpy.spin_once(self._node, timeout_sec=0.1)
        raise RuntimeError(f"No stable fresh RGB snapshot: {self._node.failures[-3:]}")

    def _safe_idle(self) -> bool:
        """Only return to the prompt when all persistent physical state is safe."""
        assert self._physical and self._synchronizer
        try:
            snapshot = self._snapshot_source()
            report = self._synchronizer.apply_snapshot(snapshot, self._now_sec())
            return (self._physical.ready_for_pick()
                    and not report.attached_ids
                    and set(report.authoritative_xyz) == set(BLOCKS))
        except (RuntimeError, ValueError, TypeError, KeyError):
            return False

    def execute_command(self, command: str) -> bool:
        """Run one new RGB/LLM/validator/executor transaction and discard it."""
        assert self._skills and self._physical and self._synchronizer and self._planner
        self._skills.clear_temporary_position()
        try:
            initial = self._snapshot_source()
            initial_report = self._synchronizer.apply_snapshot(initial, self._now_sec())
            if initial_report.attached_ids or set(initial_report.authoritative_xyz) != set(BLOCKS):
                raise RuntimeError("stale MoveIt attachment before planning")
            if not self._physical.ready_for_pick():
                raise RuntimeError("stale physical attachment before planning")
            state = WorldState.from_perception_snapshot(initial, self._now_sec(), self._geometry)
            goal = extract_placement_goal(command)
            print(f"[PERCEPTION]\n{build_scene_context(state)}", flush=True)
            print(f"[USER] {command}", flush=True)
            if goal is None:
                raise RuntimeError("unsupported or low-level request; no LLM call or motion")
            selected = [None]

            def on_success(index: int, step: dict[str, str]) -> None:
                if step["skill"] == "pick" and self._skills.reserved_temporary is not None:
                    selected[0] = self._skills.reserved_temporary
                print(f"[EXECUTION] step {index}: {step}", flush=True)
                if step["skill"] in ("place", "place_temp"):
                    observation = self._skills.last_verified_snapshot
                    verify_placement_observation(observation, state, self._physical.ready_for_pick())
                    assert observation is not None
                    print(f"[CAMERA VERIFY] {step['object']}={observation.object_locations[step['object']]} "
                          f"{goal.destination_zone}="
                          f"{observation.zone_occupancy[goal.destination_zone] or 'empty'}", flush=True)

            def before_execute(current: PerceptionSnapshot) -> None:
                report = self._synchronizer.apply_snapshot(current, self._now_sec())
                if report.attached_ids or set(report.authoritative_xyz) != set(BLOCKS):
                    raise RuntimeError("STALE_PLAN: authoritative MoveIt scene changed")
                if not self._physical.ready_for_pick():
                    raise RuntimeError("STALE_PLAN: physical attachment changed")
                print("[STALE WORLD] PASS", flush=True)

            def on_validated(raw: str) -> None:
                print(f"[LLM RAW] {raw}", flush=True)
                print("[VALIDATOR] PASS\n[GOAL CHECK] PASS", flush=True)

            executor = SkillExecutor(self._skills, state, snapshot_source=self._snapshot_source,
                                     now_sec=self._now_sec, on_step_success=on_success)
            _, result = plan_and_execute(command, initial, self._geometry, state, self._planner,
                                         executor, self._snapshot_source, self._now_sec,
                                         before_execute, on_validated)
            if result.status != TaskStatus.SUCCESS:
                raise RuntimeError(f"execution failed after {result.completed_steps}: {result.message}")
            final = self._snapshot_source()
            report = self._synchronizer.apply_snapshot(final, self._now_sec())
            joints = self._interface.current_joint_positions()
            home_error = (max(abs(joints[name] - float(self._motion["home"][name])) for name in ARM_JOINTS)
                          if joints is not None and all(name in joints for name in ARM_JOINTS)
                          else float("inf"))
            if (dict(final.object_locations) != state.object_locations
                    or dict(final.zone_occupancy) != state.zone_occupancy
                    or final.object_locations[goal.object_name] != goal.destination_zone
                    or state.held_object is not None or report.attached_ids
                    or set(report.authoritative_xyz) != set(BLOCKS)
                    or not self._physical.ready_for_pick() or home_error > HOME_JOINT_TOLERANCE_RAD
                    or joints is None
                    or any(name not in joints or abs(joints[name] - OPEN_POSITION) > FINGER_OPEN_TOLERANCE_M
                           for name in FINGER_JOINTS)):
                raise RuntimeError("final camera, task, MoveIt, or physical state disagrees")
            print(f"[TEMP] {selected[0]}", flush=True)
            print(f"[FINAL RGB] locations={dict(final.object_locations)} zones={dict(final.zone_occupancy)}", flush=True)
            print(f"[FINAL MOVEIT] WORLD={sorted(report.authoritative_xyz)} ATTACHED=[] "
                  f"error={report.max_sync_error_m * 1000:.3f}mm", flush=True)
            print("TASK SUCCESS", flush=True)
            return True
        except (RuntimeError, ValueError, TypeError, KeyError) as error:
            print(f"TASK FAILED: {error}", flush=True)
            if not self._safe_idle():
                raise RuntimeError("runtime state inconsistent; terminating fail-closed") from error
            return False
        finally:
            self._skills.clear_temporary_position()

    def close(self) -> None:
        for resource in (self._physical, self._gripper, self._manager, self._interface):
            if resource is not None:
                resource.destroy()
        if self._node is not None:
            self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv: list[str] | None = None) -> None:
    share = Path(get_package_share_directory("ur3_perception_llm_control"))
    parser = argparse.ArgumentParser(description="Persistent Assignment 03 interactive runtime")
    parser.add_argument("--scene", type=Path, default=share / "config/scene.yaml")
    parser.add_argument("--motion", type=Path, default=share / "config/robot_motion.yaml")
    parser.add_argument("--temporary", type=Path, default=share / "config/temporary_position.yaml")
    parser.add_argument("--perception", type=Path, default=share / "config/perception.yaml")
    parser.add_argument("--calibration", type=Path, default=share / "config/camera_calibration.yaml")
    parser.add_argument("--camera-timeout", type=float, default=30.0)
    options = parser.parse_args(argv)
    if options.camera_timeout <= 0:
        parser.error("camera timeout must be positive")
    runtime = Assignment3Runtime(options)
    try:
        runtime.start()
        print("Assignment 03 ready.", flush=True)
        while True:
            try:
                command = input("Command> ").strip()
            except EOFError:
                print("", flush=True)
                break
            if command.casefold() in ("exit", "quit"):
                break
            if command:
                runtime.execute_command(command)
    except KeyboardInterrupt:
        print("", flush=True)
    finally:
        runtime.close()


if __name__ == "__main__":
    main()
