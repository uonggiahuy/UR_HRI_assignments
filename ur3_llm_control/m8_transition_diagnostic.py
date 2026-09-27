"""Compare M7 ``pick`` behavior from startup versus the configured HOME pose."""

from __future__ import annotations

import argparse

import rclpy
from rclpy.node import Node

from ur3_llm_control.gazebo_sync import GazeboAttachmentSynchronizer
from ur3_llm_control.gripper import ParallelJawGripper
from ur3_llm_control.motion_primitives import ManipulationMotionPrimitives
from ur3_llm_control.moveit_interface import ARM_JOINTS, MoveItArmInterface
from ur3_llm_control.planning_scene import PlanningSceneManager
from ur3_llm_control.robot_skills import RobotSkills
from ur3_llm_control.robot_skills_test import (
    OBJECT,
    _package_file,
    _require_controllers,
    _scene_objects,
    _yaml,
)
from ur3_llm_control.workcell_scene import iter_models, load_scene


def _joint_report(interface: MoveItArmInterface) -> str:
    positions = interface.current_joint_positions()
    if positions is None:
        return "unavailable"
    return ", ".join(f"{name}={positions[name]:.4f}" for name in ARM_JOINTS)


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--home-first", action="store_true")
    arguments, ros_args = parser.parse_known_args(args=args)

    rclpy.init(args=ros_args)
    node = Node("m8_transition_diagnostic")
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
        print("STARTUP JOINT STATE: " + _joint_report(interface))
        if arguments.home_first:
            home_status = skills.home()
            print("HOME RESULT: " + home_status.value)
            if home_status.value != "SUCCESS":
                raise RuntimeError("HOME failed; pick was not attempted")
            print("HOME JOINT STATE: " + _joint_report(interface))
        pick_status = skills.pick(OBJECT)
        world, attached = _scene_objects(manager)
        lifecycle = "ATTACHED" if OBJECT in attached and OBJECT not in world else "WORLD"
        print(f"PICK RESULT: {pick_status.value}")
        print(f"RED_CUBE LIFECYCLE: {lifecycle}")
        _require_controllers(node)
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
