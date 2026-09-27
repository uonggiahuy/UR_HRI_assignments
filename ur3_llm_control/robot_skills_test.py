"""Direct M7 integration test: HOME, pick red_cube, place zone_b, HOME."""

from __future__ import annotations

import math
import os

from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
import rclpy
from rclpy.node import Node
import yaml

from ur3_llm_control.gazebo_sync import GazeboAttachmentSynchronizer
from ur3_llm_control.gripper import ParallelJawGripper
from ur3_llm_control.motion_primitives import ManipulationMotionPrimitives
from ur3_llm_control.moveit_interface import ARM_JOINTS, MoveItArmInterface
from ur3_llm_control.planning_scene import PlanningSceneManager
from ur3_llm_control.robot_skills import RobotSkills, SkillStatus
from ur3_llm_control.workcell_scene import iter_models, load_scene


TIMEOUT = 30.0
OBJECT = "red_cube"
ZONE = "zone_b"
UNCHANGED_OBJECTS = ("yellow_cube", "blue_cube")


def _package_file(relative_path: str) -> str:
    return os.path.join(get_package_share_directory("ur3_llm_control"), relative_path)


def _yaml(relative_path: str) -> dict:
    with open(_package_file(relative_path), encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"{relative_path} must contain a mapping")
    return value


def _require(label: str, result: SkillStatus) -> None:
    if result != SkillStatus.SUCCESS:
        raise RuntimeError(f"{label}: {result.value}")


def _scene_objects(manager: PlanningSceneManager) -> tuple[dict, dict]:
    scene = manager.get(TIMEOUT)
    if scene is None:
        raise RuntimeError("/get_planning_scene is unavailable")
    return (
        {item.id: item for item in scene.world.collision_objects},
        {item.object.id: item for item in scene.robot_state.attached_collision_objects},
    )


def _require_lifecycle(manager: PlanningSceneManager, expected: str) -> None:
    world, attached = _scene_objects(manager)
    if expected == "WORLD":
        if OBJECT not in world or OBJECT in attached:
            raise RuntimeError("red_cube is not exclusively in WORLD")
    elif expected == "ATTACHED":
        item = attached.get(OBJECT)
        if OBJECT in world or item is None:
            raise RuntimeError("red_cube is not exclusively ATTACHED")
        if item.link_name != manager.attachment_link:
            raise RuntimeError("red_cube has the wrong attachment link")
    else:
        raise ValueError(f"unknown lifecycle state {expected}")


def _same_pose(left, right, tolerance: float = 0.012) -> bool:
    return all(
        math.isclose(a, b, abs_tol=tolerance)
        for a, b in zip(
            (left.position.x, left.position.y, left.position.z),
            (right.position.x, right.position.y, right.position.z),
        )
    )


def _require_controllers(node: Node) -> None:
    client = node.create_client(ListControllers, "/controller_manager/list_controllers")
    try:
        if not client.wait_for_service(timeout_sec=TIMEOUT):
            raise RuntimeError("controller manager is unavailable")
        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=TIMEOUT)
        response = future.result() if future.done() else None
        states = {} if response is None else {item.name: item.state for item in response.controller}
        unhealthy = [
            name for name in (
                "joint_trajectory_controller", "gripper_controller", "joint_state_broadcaster"
            ) if states.get(name) != "active"
        ]
        if unhealthy:
            raise RuntimeError("inactive controllers: " + ", ".join(unhealthy))
    finally:
        node.destroy_client(client)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node("robot_skills_test")
    interface = gripper = manager = gazebo_sync = None
    try:
        scene = load_scene(_package_file("config/scene.yaml"))
        motion = _yaml("config/robot_motion.yaml")
        settings = motion["manipulation"]
        interface = MoveItArmInterface(
            node,
            planning_group=str(motion["planning_group"]),
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
            node, scene,
            attachment_link=str(settings["attachment_link"]),
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
        world_before, _ = _scene_objects(manager)
        unchanged_before = {name: world_before[name].pose for name in UNCHANGED_OBJECTS}

        _require("home", skills.home())
        _require("pick red_cube", skills.pick(OBJECT))
        _require_lifecycle(manager, "ATTACHED")
        _require("place red_cube -> zone_b", skills.place(OBJECT, ZONE))
        _require_lifecycle(manager, "WORLD")
        _require("final home", skills.home())

        world_after, attached_after = _scene_objects(manager)
        expected = ManipulationMotionPrimitives(
            interface, scene, motion
        ).placement_world_pose(OBJECT, ZONE)
        if attached_after or not _same_pose(world_after[OBJECT].pose, expected.pose, 1e-5):
            raise RuntimeError("red_cube did not finish WORLD-only at zone_b")
        for name, pose in unchanged_before.items():
            if not _same_pose(world_after[name].pose, pose, 1e-5):
                raise RuntimeError(f"{name} changed during the M7 sequence")
        gazebo_pose = gazebo_sync.model_pose(OBJECT)
        if gazebo_pose is None or not _same_pose(gazebo_pose, expected.pose):
            raise RuntimeError("Gazebo red_cube did not finish at zone_b")
        _require_controllers(node)
        node.get_logger().info("M7 RobotSkills integration PASSED")
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
