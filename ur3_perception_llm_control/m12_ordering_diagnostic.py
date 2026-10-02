"""Single-process M12 yellow-transit diagnostic; not part of production routing."""

from __future__ import annotations

import argparse

import rclpy
from rclpy.node import Node

from ur3_perception_llm_control.gazebo_sync import GazeboAttachmentSynchronizer
from ur3_perception_llm_control.gripper import ParallelJawGripper
from ur3_perception_llm_control.m12_demo import MultiObjectGazeboSync
from ur3_perception_llm_control.motion_primitives import ManipulationMotionPrimitives
from ur3_perception_llm_control.moveit_interface import ARM_JOINTS, MoveItArmInterface, MotionResult
from ur3_perception_llm_control.planning_scene import PlanningSceneManager
from ur3_perception_llm_control.robot_skills import RobotSkills, SkillStatus
from ur3_perception_llm_control.robot_skills_test import _package_file, _require_controllers, _yaml
from ur3_perception_llm_control.world_state import LEGACY_STUDENT_OBJECTS
from ur3_perception_llm_control.workcell_scene import iter_models, load_scene


def _require(label: str, status: SkillStatus) -> None:
    print(f"{label}: {status.value}")
    if status != SkillStatus.SUCCESS:
        raise RuntimeError(f"{label} failed: {status.value}")


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="M12 yellow transit A/B/C diagnostic")
    parser.add_argument("--case", choices=("A", "B", "C"), required=True)
    parsed = parser.parse_args(args)

    rclpy.init()
    node = Node("m12_ordering_diagnostic")
    interface = gripper = manager = gazebo_sync = None
    try:
        scene = load_scene(_package_file("config/scene.yaml"))
        motion = _yaml("config/robot_motion.yaml")
        settings = motion["manipulation"]
        interface = MoveItArmInterface(
            node, planning_group=str(motion["planning_group"]), planning_frame=str(motion["planning_frame"]),
            planning_tip=str(motion["planning_tip"]), application_tip=str(motion["application_tip"]),
            tool0_to_tcp_z=float(motion["tool0_to_tcp_z"]), velocity_scaling=float(motion["velocity_scaling"]),
            acceleration_scaling=float(motion["acceleration_scaling"]), planning_time=float(motion["planning_time"]),
            planning_attempts=int(motion["planning_attempts"]),
        )
        manager = PlanningSceneManager(node, scene, attachment_link=str(settings["attachment_link"]),
            touch_links=tuple(str(link) for link in settings["touch_links"]))
        models = {model.name: model for model in iter_models(scene) if model.name in LEGACY_STUDENT_OBJECTS}
        gazebo_sync = MultiObjectGazeboSync({name: GazeboAttachmentSynchronizer(node, model=model) for name, model in models.items()})
        gripper = ParallelJawGripper(node)
        home = {name: float(value) for name, value in motion["home"].items()}
        if set(home) != set(ARM_JOINTS):
            raise RuntimeError("HOME must define exactly the six arm joints")
        primitives = ManipulationMotionPrimitives(interface, scene, motion)
        skills = RobotSkills(interface=interface, gripper=gripper, primitives=primitives,
            scene_manager=manager, gazebo_sync=gazebo_sync, home_configuration=home)
        if not interface.wait_until_ready() or not gazebo_sync.wait_until_ready():
            raise RuntimeError("MoveIt or Gazebo is not ready")
        _require_controllers(node)
        if not manager.apply():
            raise RuntimeError("canonical Planning Scene apply failed")
        verified, detail = manager.verify()
        if not verified:
            raise RuntimeError(f"canonical Planning Scene verification failed: {detail}")
        initial = manager.get()
        if initial is None or initial.robot_state.attached_collision_objects:
            raise RuntimeError("canonical scene has an unexpected attachment")
        print(f"CASE {parsed.case}: canonical scene verified")

        _require("home", skills.home())
        if parsed.case in ("B", "C"):
            _require("pick blue_cube", skills.pick("blue_cube"))
            _require("place blue_cube zone_a", skills.place("blue_cube", "zone_a"))
            _require("home after blue", skills.home())
        if parsed.case == "C":
            _require("pick red_cube", skills.pick("red_cube"))
            _require("place red_cube zone_b", skills.place("red_cube", "zone_b"))
            _require("home after red", skills.home())
            # This invokes the existing collision-aware IK, target validity,
            # and planner path without descending or grasping yellow.
            result = primitives.move_above("yellow_cube")
            print(f"yellow move_above planning result: {result.value}")
            if result != MotionResult.SUCCESS:
                raise RuntimeError(f"CASE C transit failure: {result.value}")
            print("CASE C transit planning PASS")
            return
        _require("pick yellow_cube", skills.pick("yellow_cube"))
        print(f"CASE {parsed.case} PASS")
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
