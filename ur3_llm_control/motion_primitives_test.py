"""M6 one-cube integration test for manipulation primitives and attachment."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
import rclpy
from rclpy.node import Node
import yaml

from ur3_llm_control.gripper import ParallelJawGripper
from ur3_llm_control.gazebo_sync import GazeboAttachmentSynchronizer
from ur3_llm_control.motion_primitives import ManipulationMotionPrimitives
from ur3_llm_control.moveit_interface import (
    ARM_JOINTS,
    MotionResult,
    MoveItArmInterface,
)
from ur3_llm_control.planning_scene import PlanningSceneManager
from ur3_llm_control.workcell_scene import load_scene


TIMEOUT = 30.0
OBJECT_NAME = "red_cube"


def _package_file(relative_path: str) -> str:
    return os.path.join(get_package_share_directory("ur3_llm_control"), relative_path)


def _load_yaml(relative_path: str) -> dict:
    with open(_package_file(relative_path), encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"{relative_path} must contain a mapping")
    return value


def _require_success(node: Node, label: str, result: MotionResult) -> None:
    node.get_logger().info(f"{label}: {result.value}")
    if result != MotionResult.SUCCESS:
        raise RuntimeError(f"{label} returned {result.value}")


def _verify_lifecycle(
    node: Node, manager: PlanningSceneManager, expected: str
) -> None:
    scene = manager.get()
    if scene is None:
        raise RuntimeError("/get_planning_scene did not return a scene")
    world_ids = {item.id for item in scene.world.collision_objects}
    attached = {
        item.object.id: item for item in scene.robot_state.attached_collision_objects
    }
    if expected == "WORLD":
        if OBJECT_NAME not in world_ids or OBJECT_NAME in attached:
            raise RuntimeError(
                f"Expected {OBJECT_NAME} only in WORLD; world={sorted(world_ids)}, "
                f"attached={sorted(attached)}"
            )
    elif expected == "ATTACHED":
        if OBJECT_NAME in world_ids or OBJECT_NAME not in attached:
            raise RuntimeError(
                f"Expected {OBJECT_NAME} only ATTACHED; world={sorted(world_ids)}, "
                f"attached={sorted(attached)}"
            )
        item = attached[OBJECT_NAME]
        if item.link_name != manager.attachment_link:
            raise RuntimeError("Attached object uses the wrong attachment link")
        if set(item.touch_links) != set(manager.touch_links):
            raise RuntimeError("Attached object uses the wrong gripper touch links")
    else:
        raise ValueError(f"Unknown lifecycle state {expected}")
    node.get_logger().info(f"Planning Scene lifecycle: {OBJECT_NAME} is {expected}")


def _verify_controllers(node: Node) -> None:
    client = node.create_client(ListControllers, "/controller_manager/list_controllers")
    try:
        if not client.wait_for_service(timeout_sec=TIMEOUT):
            raise RuntimeError("controller manager is unavailable")
        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=TIMEOUT)
        response = future.result() if future.done() else None
        if response is None:
            raise RuntimeError("controller manager did not return a response")
        states = {item.name: item.state for item in response.controller}
        unhealthy = [
            name
            for name in ("joint_trajectory_controller", "gripper_controller")
            if states.get(name) != "active"
        ]
        if unhealthy:
            raise RuntimeError("Inactive controllers: " + ", ".join(unhealthy))
    finally:
        node.destroy_client(client)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node("motion_primitives_test")
    interface = None
    gripper = None
    manager = None
    gazebo_sync = None
    try:
        scene = load_scene(_package_file("config/scene.yaml"))
        motion = _load_yaml("config/robot_motion.yaml")
        settings = motion["manipulation"]
        manager = PlanningSceneManager(
            node,
            scene,
            attachment_link=str(settings["attachment_link"]),
            touch_links=tuple(str(link) for link in settings["touch_links"]),
        )
        gazebo_sync = GazeboAttachmentSynchronizer(node)
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
        gripper = ParallelJawGripper(node)
        primitives = ManipulationMotionPrimitives(interface, scene, motion)
        home = {name: float(value) for name, value in motion["home"].items()}
        if set(home) != set(ARM_JOINTS):
            raise ValueError("HOME must define exactly the six UR arm joints")
        if not interface.wait_until_ready():
            raise RuntimeError("MoveIt interface did not become ready")
        if not gazebo_sync.wait_until_ready():
            raise RuntimeError("Gazebo set-pose bridge did not become ready")
        _verify_controllers(node)
        _verify_lifecycle(node, manager, "WORLD")

        _require_success(node, "HOME", interface.move_to_joint_configuration(home))
        gripper.open()
        _require_success(node, "move_above", primitives.move_above(OBJECT_NAME))
        if not manager.allow_grasp_contact(OBJECT_NAME):
            raise RuntimeError("Could not allow scoped target/gripper grasp contact")
        _require_success(node, "descend", primitives.descend(OBJECT_NAME))
        gripper.close()
        if not manager.attach_object(OBJECT_NAME):
            raise RuntimeError("attach_object failed")
        _verify_lifecycle(node, manager, "ATTACHED")
        relative_pose = manager.attached_pose(OBJECT_NAME)
        if relative_pose is None or not gazebo_sync.attach(
            OBJECT_NAME, manager.attachment_link, relative_pose
        ):
            raise RuntimeError("Gazebo attachment synchronization did not start")
        _require_success(node, "retreat with attached cube", primitives.retreat())

        _require_success(
            node,
            "safe pose with attached cube",
            interface.move_to_joint_configuration(home),
        )
        _require_success(
            node, "move_above placement", primitives.move_above(OBJECT_NAME)
        )
        _require_success(node, "descend placement", primitives.descend(OBJECT_NAME))
        release_pose = primitives.target_world_pose(OBJECT_NAME)
        if not gazebo_sync.release(release_pose):
            raise RuntimeError("Gazebo attachment synchronization did not release")
        if not manager.detach_object(OBJECT_NAME, release_pose):
            raise RuntimeError("detach_object failed")
        _verify_lifecycle(node, manager, "WORLD")
        gripper.open()
        _require_success(node, "retreat after detach", primitives.retreat())
        if not gazebo_sync.set_world_pose(OBJECT_NAME, release_pose):
            raise RuntimeError("Gazebo final release pose update failed")
        if not manager.clear_grasp_contact(OBJECT_NAME):
            raise RuntimeError("Could not clear temporary post-release contact")
        _require_success(
            node, "return HOME", interface.move_to_joint_configuration(home)
        )
        _verify_controllers(node)
        node.get_logger().info("M6 manipulation primitives integration PASSED")
    except Exception as exc:
        node.get_logger().error(f"M6 manipulation primitives integration FAILED: {exc}")
        raise
    finally:
        if gripper is not None:
            gripper.destroy()
        if interface is not None:
            interface.destroy()
        if manager is not None:
            manager.destroy()
        if gazebo_sync is not None:
            gazebo_sync.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
