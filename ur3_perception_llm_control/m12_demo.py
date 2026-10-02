"""ROS-backed M12 command-line demo entry point."""

from __future__ import annotations

import argparse
import json
import logging

import rclpy
from rclpy.node import Node

from ur3_perception_llm_control.gazebo_sync import GazeboAttachmentSynchronizer
from ur3_perception_llm_control.gripper import ParallelJawGripper
from ur3_perception_llm_control.llm_planner import LLMPlanner
from ur3_perception_llm_control.m12_runtime import M12Runtime, RuntimeResult, RuntimeStatus
from ur3_perception_llm_control.motion_primitives import ManipulationMotionPrimitives
from ur3_perception_llm_control.moveit_interface import ARM_JOINTS, MoveItArmInterface
from ur3_perception_llm_control.planning_scene import PlanningSceneManager
from ur3_perception_llm_control.robot_skills import RobotSkills
from ur3_perception_llm_control.robot_skills_test import TIMEOUT, _package_file, _require_controllers, _same_pose, _yaml
from ur3_perception_llm_control.skill_executor import SkillExecutor
from ur3_perception_llm_control.world_state import LEGACY_STUDENT_OBJECTS, WorldState
from ur3_perception_llm_control.workcell_scene import iter_models, load_scene


class MultiObjectGazeboSync:
    """Route the existing one-object M6 synchronizer for all whitelisted cubes."""

    def __init__(self, synchronizers: dict[str, GazeboAttachmentSynchronizer]) -> None:
        self._synchronizers = synchronizers
        self._active: GazeboAttachmentSynchronizer | None = None

    def wait_until_ready(self) -> bool:
        return all(sync.wait_until_ready() for sync in self._synchronizers.values())

    def attach(self, object_name: str, attachment_link: str, relative_pose) -> bool:
        sync = self._synchronizers.get(object_name)
        if sync is None or not sync.attach(object_name, attachment_link, relative_pose):
            return False
        self._active = sync
        return True

    def release(self, world_pose) -> bool:
        if self._active is None:
            return False
        result = self._active.release(world_pose)
        self._active = None
        return result

    def set_world_pose(self, object_name: str, world_pose) -> bool:
        sync = self._synchronizers.get(object_name)
        return sync is not None and sync.set_world_pose(object_name, world_pose)

    def model_pose(self, object_name: str):
        sync = self._synchronizers.get(object_name)
        return None if sync is None else sync.model_pose(object_name)

    def destroy(self) -> None:
        for sync in self._synchronizers.values():
            sync.destroy()


def _build_runtime(node: Node) -> tuple[M12Runtime, tuple[object, ...]]:
    scene = load_scene(_package_file("config/scene.yaml"))
    motion = _yaml("config/robot_motion.yaml")
    settings = motion["manipulation"]
    interface = MoveItArmInterface(node, planning_group=str(motion["planning_group"]),
        planning_frame=str(motion["planning_frame"]), planning_tip=str(motion["planning_tip"]),
        application_tip=str(motion["application_tip"]), tool0_to_tcp_z=float(motion["tool0_to_tcp_z"]),
        velocity_scaling=float(motion["velocity_scaling"]), acceleration_scaling=float(motion["acceleration_scaling"]),
        planning_time=float(motion["planning_time"]), planning_attempts=int(motion["planning_attempts"]))
    manager = PlanningSceneManager(node, scene, attachment_link=str(settings["attachment_link"]),
        touch_links=tuple(str(link) for link in settings["touch_links"]))
    # M12 owns a deterministic execution boundary.  Do not depend on the
    # launch-time scene helper surviving a DDS startup race.
    if not manager.apply():
        raise RuntimeError("M12 could not apply the canonical Planning Scene")
    verified, detail = manager.verify()
    if not verified:
        raise RuntimeError(f"M12 Planning Scene verification failed: {detail}")
    node.get_logger().info(f"M12 canonical Planning Scene ready: {detail}")
    gripper = ParallelJawGripper(node)
    models = {model.name: model for model in iter_models(scene) if model.name in LEGACY_STUDENT_OBJECTS}
    gazebo_sync = MultiObjectGazeboSync({name: GazeboAttachmentSynchronizer(node, model=model) for name, model in models.items()})
    home = {name: float(value) for name, value in motion["home"].items()}
    if set(home) != set(ARM_JOINTS):
        raise RuntimeError("HOME must define exactly the six arm joints")
    skills = RobotSkills(interface=interface, gripper=gripper,
        primitives=ManipulationMotionPrimitives(interface, scene, motion), scene_manager=manager,
        gazebo_sync=gazebo_sync, home_configuration=home)
    if not interface.wait_until_ready() or not gazebo_sync.wait_until_ready():
        raise RuntimeError("MoveIt or Gazebo is not ready")
    _require_controllers(node)
    state = WorldState.from_scene_file(_package_file("config/scene.yaml"))
    return M12Runtime(LLMPlanner.from_environment(), SkillExecutor(skills, state), state), (interface, gripper, manager, gazebo_sync)


def _verify_consistency(runtime: M12Runtime, interface: MoveItArmInterface,
                        manager: PlanningSceneManager, gazebo_sync: MultiObjectGazeboSync,
                        scene: dict, motion: dict) -> None:
    """Reject success if task state, MoveIt, or Gazebo disagree after a command."""
    current = manager.get(TIMEOUT)
    if current is None:
        raise RuntimeError("cannot verify final MoveIt planning scene")
    world_items = current.world.collision_objects
    world = {item.id: item for item in world_items}
    attached_ids = {item.object.id for item in current.robot_state.attached_collision_objects}
    if runtime.world_state.held_object is not None or attached_ids:
        raise RuntimeError("final state still contains an attachment")
    for object_name in LEGACY_STUDENT_OBJECTS:
        if sum(item.id == object_name for item in world_items) != 1 or object_name not in world:
            raise RuntimeError(f"MoveIt does not contain exactly one WORLD {object_name}")
        location = runtime.world_state.object_locations[object_name]
        if location in runtime.world_state.zone_occupancy:
            if runtime.world_state.zone_occupancy[location] != object_name:
                raise RuntimeError(f"WorldState occupancy disagrees for {object_name}")
            expected = ManipulationMotionPrimitives(interface, scene, motion).placement_world_pose(object_name, location)
            if not _same_pose(world[object_name].pose, expected.pose, 1e-5):
                raise RuntimeError(f"MoveIt {object_name} is not at its logical {location} pose")
        elif location != "table":
            raise RuntimeError(f"invalid logical location for {object_name}: {location}")
        gazebo_pose = gazebo_sync.model_pose(object_name)
        if gazebo_pose is None or not _same_pose(gazebo_pose, world[object_name].pose):
            raise RuntimeError(f"Gazebo and MoveIt disagree for {object_name}")


def _print_result(command: str, result: RuntimeResult) -> None:
    print(f"USER COMMAND:\n{command}\n\nPLAN:")
    if result.plan:
        print(json.dumps({"plan": list(result.plan)}, ensure_ascii=False))
    elif result.planner and result.planner.candidate_json:
        print(result.planner.candidate_json)
    else:
        print("<none>")
    print("\nEXECUTION:")
    for index, step in enumerate(result.plan, 1):
        arguments = ", ".join(value for key, value in step.items() if key != "skill")
        print(f"[{index}] {step['skill']}({arguments})")
    print("\nTASK SUCCESS" if result.status == RuntimeStatus.SUCCESS else f"\nTASK FAILED: {result.message}")


def main(args: list[str] | None = None) -> None:
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    parser = argparse.ArgumentParser(description="M12 natural-language UR3e demo")
    parser.add_argument("--command", required=True)
    parser.add_argument("--student-id", help="runtime-only M9 override; never written to config")
    parsed = parser.parse_args(args)
    rclpy.init()
    node, dependencies = Node("m12_runtime"), ()
    try:
        runtime, dependencies = _build_runtime(node)
        result = runtime.run(parsed.command, parsed.student_id)
        _print_result(parsed.command, result)
        if result.status != RuntimeStatus.SUCCESS:
            raise SystemExit(1)
        # Existing public skills have completed; verify all externally
        # observable layers before reporting a successful demo.
        _verify_consistency(runtime, dependencies[0], dependencies[2], dependencies[3],
                            load_scene(_package_file("config/scene.yaml")), _yaml("config/robot_motion.yaml"))
        _require_controllers(node)
    finally:
        for dependency in dependencies:
            dependency.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
