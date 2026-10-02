"""LIVE M8 integration: validate and execute the fixed pick/place/home JSON plan."""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node

from ur3_perception_llm_control.gazebo_sync import GazeboAttachmentSynchronizer
from ur3_perception_llm_control.gripper import ParallelJawGripper
from ur3_perception_llm_control.motion_primitives import ManipulationMotionPrimitives
from ur3_perception_llm_control.moveit_interface import ARM_JOINTS, MoveItArmInterface
from ur3_perception_llm_control.planning_scene import PlanningSceneManager
from ur3_perception_llm_control.robot_skills import RobotSkills
from ur3_perception_llm_control.robot_skills_test import (
    OBJECT, TIMEOUT, ZONE, _package_file, _require_controllers, _same_pose, _scene_objects, _yaml,
)
from ur3_perception_llm_control.skill_executor import SkillExecutor
from ur3_perception_llm_control.task_validator import TaskStatus, TaskValidator
from ur3_perception_llm_control.world_state import WorldState
from ur3_perception_llm_control.workcell_scene import iter_models, load_scene


PLAN = {
    "plan": [
        {"skill": "pick", "object": "red_cube"},
        {"skill": "place", "object": "red_cube", "zone": "zone_b"},
        {"skill": "home"},
    ]
}


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node("m8_plan_test")
    interface = gripper = manager = gazebo_sync = None
    try:
        scene = load_scene(_package_file("config/scene.yaml"))
        motion = _yaml("config/robot_motion.yaml")
        settings = motion["manipulation"]
        interface = MoveItArmInterface(
            node, planning_group=str(motion["planning_group"]),
            planning_frame=str(motion["planning_frame"]), planning_tip=str(motion["planning_tip"]),
            application_tip=str(motion["application_tip"]), tool0_to_tcp_z=float(motion["tool0_to_tcp_z"]),
            velocity_scaling=float(motion["velocity_scaling"]),
            acceleration_scaling=float(motion["acceleration_scaling"]),
            planning_time=float(motion["planning_time"]), planning_attempts=int(motion["planning_attempts"]),
        )
        manager = PlanningSceneManager(
            node, scene, attachment_link=str(settings["attachment_link"]),
            touch_links=tuple(str(link) for link in settings["touch_links"]),
        )
        gripper = ParallelJawGripper(node)
        gazebo_sync = GazeboAttachmentSynchronizer(
            node, model=next(model for model in iter_models(scene) if model.name == OBJECT)
        )
        home = {name: float(value) for name, value in motion["home"].items()}
        if set(home) != set(ARM_JOINTS):
            raise RuntimeError("HOME must define exactly the six arm joints")
        skills = RobotSkills(
            interface=interface, gripper=gripper,
            primitives=ManipulationMotionPrimitives(interface, scene, motion),
            scene_manager=manager, gazebo_sync=gazebo_sync, home_configuration=home,
        )
        if not interface.wait_until_ready() or not gazebo_sync.wait_until_ready():
            raise RuntimeError("MoveIt or Gazebo is not ready")
        _require_controllers(node)

        world_state = WorldState.from_scene_file(_package_file("config/scene.yaml"))
        validation = TaskValidator().validate(json.dumps(PLAN), world_state)
        if not validation.accepted:
            raise RuntimeError(f"plan rejected: {validation.status.value}: {validation.message}")
        print("PLAN ACCEPTED")
        execution = SkillExecutor(skills, world_state).execute(validation)
        if execution.status != TaskStatus.SUCCESS:
            raise RuntimeError(f"task failed: {execution.status.value}: {execution.message}")
        print("\n[1] pick(red_cube) ........ SUCCESS")
        print("[2] place(red_cube, zone_b) SUCCESS")
        print("[3] home() ................ SUCCESS")
        print("\nTASK SUCCESS")

        world, attached = _scene_objects(manager)
        expected = ManipulationMotionPrimitives(interface, scene, motion).placement_world_pose(OBJECT, ZONE)
        gazebo_pose = gazebo_sync.model_pose(OBJECT)
        if attached or OBJECT not in world or not _same_pose(world[OBJECT].pose, expected.pose, 1e-5):
            raise RuntimeError("MoveIt state does not show red_cube in zone_b")
        if gazebo_pose is None or not _same_pose(gazebo_pose, expected.pose):
            raise RuntimeError("Gazebo state does not show red_cube in zone_b")
        if (world_state.held_object is not None
                or world_state.object_locations[OBJECT] != ZONE
                or world_state.zone_occupancy[ZONE] != OBJECT):
            raise RuntimeError("M8 WorldState does not match the final robot state")
        _require_controllers(node)
        node.get_logger().info("M8 hard-coded plan integration PASSED")
    finally:
        if gazebo_sync is not None:
            gazebo_sync.destroy()
        if gripper is not None:
            gripper.destroy()
        if manager is not None:
            manager.destroy()
        if interface is not None:
            interface.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
