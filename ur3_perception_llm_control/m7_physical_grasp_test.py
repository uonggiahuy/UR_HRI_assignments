"""Single-cube M7 live diagnostic; Gazebo poses are read-only test oracles."""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node

from ur3_perception_llm_control.gripper import ParallelJawGripper
from ur3_perception_llm_control.motion_primitives import ManipulationMotionPrimitives
from ur3_perception_llm_control.moveit_interface import ARM_JOINTS, MotionResult, MoveItArmInterface
from ur3_perception_llm_control.physical_grasp import PhysicalGraspManager, model_pose
from ur3_perception_llm_control.planning_scene import PlanningSceneManager
from ur3_perception_llm_control.robot_skills import RobotSkills, SkillStatus
from ur3_perception_llm_control.robot_skills_test import _package_file, _require_controllers, _yaml
from ur3_perception_llm_control.workcell_scene import load_scene


OBJECT = "red_cube"
ZONE = "zone_a"


def _require(label: str, condition: bool) -> None:
    if not condition:
        raise RuntimeError(f"M7 {label} failed; stopping motion")
    print(f"M7 {label}: PASS", flush=True)


def _pose() -> tuple[float, float, float]:
    pose = model_pose(OBJECT)
    if pose is None:
        raise RuntimeError("Gazebo red_cube read-only pose unavailable")
    return pose.position.x, pose.position.y, pose.position.z


def _scene_state(manager: PlanningSceneManager, expected: str) -> bool:
    scene = manager.get()
    if scene is None:
        return False
    world = {item.id for item in scene.world.collision_objects}
    attached = {item.object.id for item in scene.robot_state.attached_collision_objects}
    return (OBJECT in world, OBJECT in attached) == (
        (True, False) if expected == "WORLD" else (False, True)
    )


def _hold(node: Node, manager: PlanningSceneManager, physical: PhysicalGraspManager, seconds: float) -> None:
    initial = _pose()
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if not physical.is_attached(OBJECT):
            raise RuntimeError("M7 physical joint detached during hold; stopping motion")
    final = _pose()
    _require("hold height", final[2] > 0.36)
    _require("hold drift", math.dist(initial, final) < 0.015)
    _require("MoveIt attached during hold", _scene_state(manager, "ATTACHED"))
    print(f"M7 hold {seconds:.1f}s: cube {initial} -> {final}", flush=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node("m7_physical_grasp_test")
    interface = gripper = manager = physical = None
    try:
        scene = load_scene(_package_file("config/scene.yaml"))
        motion = _yaml("config/robot_motion.yaml")
        settings = motion["manipulation"]
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
            node, scene, attachment_link=str(settings["attachment_link"]),
            touch_links=tuple(str(link) for link in settings["touch_links"]),
        )
        gripper = ParallelJawGripper(node)
        physical = PhysicalGraspManager(node)
        home = {name: float(value) for name, value in motion["home"].items()}
        _require("HOME configuration", set(home) == set(ARM_JOINTS))
        primitives = ManipulationMotionPrimitives(interface, scene, motion)
        skills = RobotSkills(
            interface=interface, gripper=gripper, primitives=primitives,
            scene_manager=manager, physical_grasp=physical, home_configuration=home,
        )
        _require("MoveIt ready", interface.wait_until_ready())
        _require_controllers(node)
        _require("all initial physical joints detached", physical.wait_until_ready())
        _require("MoveIt WORLD before pick", _scene_state(manager, "WORLD"))
        initial = _pose()
        _require("home", skills.home() == SkillStatus.SUCCESS)
        _require("pick red_cube", skills.pick(OBJECT) == SkillStatus.SUCCESS)
        _require("dual attachment", physical.is_attached(OBJECT) and _scene_state(manager, "ATTACHED"))
        lifted = _pose()
        _require("physical lift", lifted[2] > initial[2] + 0.06)
        _hold(node, manager, physical, 3.2)
        _require("lateral MoveIt motion", primitives.move_above(ZONE) == MotionResult.SUCCESS)
        lateral = _pose()
        _require("lateral physical transport", abs(lateral[1] - lifted[1]) > 0.045)
        _hold(node, manager, physical, 1.0)
        _require("place and release", skills.place(OBJECT, ZONE) == SkillStatus.SUCCESS)
        _require("dual release", not physical.is_attached(OBJECT) and _scene_state(manager, "WORLD"))
        release_pose = _pose()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        settled = _pose()
        _require("physical settling", abs(settled[2] - 0.3245) < 0.018)
        _require("independent after retreat", math.dist(release_pose, settled) < 0.025)
        _require("final home", skills.home() == SkillStatus.SUCCESS)
        _require_controllers(node)
        print(f"M7 PASS: initial={initial}, lift={lifted}, lateral={lateral}, "
              f"release={release_pose}, settled={settled}", flush=True)
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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
