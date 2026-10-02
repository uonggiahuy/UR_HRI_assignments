"""Run the M5 Planning Scene and collision-aware motion acceptance checks."""

from __future__ import annotations

import math
import os

from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK, GetStateValidity
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml

from ur3_perception_llm_control.moveit_interface import (
    ARM_JOINTS,
    MotionResult,
    MoveItArmInterface,
)
from ur3_perception_llm_control.planning_scene import PlanningSceneManager
from ur3_perception_llm_control.workcell_scene import load_scene


TIMEOUT = 30.0


def _package_file(relative_path: str) -> str:
    return os.path.join(
        get_package_share_directory("ur3_perception_llm_control"), relative_path
    )


def _load_yaml(relative_path: str) -> dict:
    with open(_package_file(relative_path), encoding="utf-8") as stream:
        result = yaml.safe_load(stream)
    if not isinstance(result, dict):
        raise ValueError(f"{relative_path} must contain a mapping")
    return result


def _pose_from_values(
    frame_id: str, position: list[float], orientation: list[float]
) -> PoseStamped:
    if len(position) != 3 or len(orientation) != 4:
        raise ValueError("Pose needs XYZ and XYZW values")
    result = PoseStamped()
    result.header.frame_id = frame_id
    result.pose.position.x, result.pose.position.y, result.pose.position.z = (
        float(value) for value in position
    )
    (
        result.pose.orientation.x,
        result.pose.orientation.y,
        result.pose.orientation.z,
        result.pose.orientation.w,
    ) = (float(value) for value in orientation)
    return result


def _safe_pose(config: dict) -> PoseStamped:
    values = config["test_gripper_tcp_pose"]
    return _pose_from_values(
        str(values["frame_id"]), values["position"], values["orientation_xyzw"]
    )


def _table_collision_pose(scene: dict, config: dict) -> PoseStamped:
    """Place the TCP just inside the table, deriving geometry only from YAML."""
    table = scene["table"]
    mount = scene["robot"]["mount_pose"]
    table_pose = table["pose"]
    table_size = table["size"]
    table_top_world = float(table_pose["z"]) + float(table_size["z"]) / 2.0
    penetration = min(0.02, float(table_size["z"]) / 4.0)
    position = [
        float(table_pose["x"]) - float(mount["x"]),
        float(table_pose["y"]) - float(mount["y"]),
        table_top_world - penetration - float(mount["z"]),
    ]
    safe_values = config["test_gripper_tcp_pose"]
    return _pose_from_values(
        str(config["planning_frame"]),
        position,
        safe_values["orientation_xyzw"],
    )


def _call(node: Node, client, request, label: str):
    if not client.wait_for_service(timeout_sec=TIMEOUT):
        raise RuntimeError(f"{label} is unavailable")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=TIMEOUT)
    if not future.done() or future.cancelled() or future.result() is None:
        raise RuntimeError(f"{label} did not return a response")
    return future.result()


def _verify_robot_model(node: Node) -> None:
    client = node.create_client(GetParameters, "/move_group/get_parameters")
    try:
        request = GetParameters.Request()
        request.names = ["robot_description"]
        response = _call(node, client, request, "move_group robot_description")
        if len(response.values) != 1:
            raise RuntimeError("move_group returned no robot_description")
        value = response.values[0]
        if value.type != ParameterType.PARAMETER_STRING:
            raise RuntimeError("move_group robot_description is not a string")
        required_names = (
            "gripper_base",
            "left_finger_joint",
            "right_finger_joint",
            "gripper_tcp",
        )
        missing = [name for name in required_names if f'name="{name}"' not in value.string_value]
        if missing:
            raise RuntimeError("MoveIt robot model is missing: " + ", ".join(missing))
        node.get_logger().info("MoveIt robot model includes the M3 gripper and gripper_tcp")
    finally:
        node.destroy_client(client)


def _verify_controllers(node: Node) -> None:
    client = node.create_client(
        ListControllers, "/controller_manager/list_controllers"
    )
    try:
        response = _call(
            node, client, ListControllers.Request(), "controller manager"
        )
        states = {controller.name: controller.state for controller in response.controller}
        required = ("joint_trajectory_controller", "gripper_controller")
        unhealthy = [name for name in required if states.get(name) != "active"]
        if unhealthy:
            raise RuntimeError("Controllers are not active: " + ", ".join(unhealthy))
        node.get_logger().info("Arm and gripper controllers are active")
    finally:
        node.destroy_client(client)


def _positions_close(
    before: dict[str, float], after: dict[str, float], tolerance: float = 1e-4
) -> bool:
    return all(
        name in before
        and name in after
        and math.isclose(before[name], after[name], abs_tol=tolerance)
        for name in ARM_JOINTS
    )


def _verify_table_collision(
    node: Node,
    interface: MoveItArmInterface,
    pose: PoseStamped,
    table_name: str,
) -> None:
    """Prove the target has IK but is invalid specifically against the table."""
    positions = interface.current_joint_positions()
    if positions is None:
        raise RuntimeError("No current state for unconstrained IK check")
    state = RobotState()
    state.joint_state = JointState()
    state.joint_state.name = list(positions)
    state.joint_state.position = list(positions.values())

    ik_client = node.create_client(GetPositionIK, "/compute_ik")
    validity_client = node.create_client(GetStateValidity, "/check_state_validity")
    try:
        ik_request = GetPositionIK.Request()
        ik_request.ik_request.group_name = interface.planning_group
        ik_request.ik_request.robot_state = state
        ik_request.ik_request.avoid_collisions = False
        ik_request.ik_request.ik_link_name = interface.planning_tip
        ik_request.ik_request.pose_stamped = interface._gripper_tcp_to_tool0(pose)
        ik_request.ik_request.timeout = Duration(seconds=2.0).to_msg()
        ik_response = _call(node, ik_client, ik_request, "unconstrained IK")
        if ik_response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                "Table test target is not kinematically reachable without collisions"
            )

        validity_request = GetStateValidity.Request()
        validity_request.robot_state = ik_response.solution
        validity_request.group_name = interface.planning_group
        validity_response = _call(
            node, validity_client, validity_request, "table collision validity"
        )
        contacts = {
            body
            for contact in validity_response.contacts
            for body in (contact.contact_body_1, contact.contact_body_2)
        }
        if validity_response.valid or table_name not in contacts:
            raise RuntimeError(
                f"Target did not report the expected {table_name} collision; "
                f"contacts={sorted(contacts)}"
            )
        node.get_logger().info(
            f"Unconstrained IK is reachable and state validity reports {table_name}"
        )
    finally:
        node.destroy_client(ik_client)
        node.destroy_client(validity_client)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node("planning_scene_test")
    manager = None
    interface = None
    try:
        scene = load_scene(_package_file("config/scene.yaml"))
        motion = _load_yaml("config/robot_motion.yaml")

        manager = PlanningSceneManager(node, scene)
        verified, detail = manager.verify()
        if not verified:
            raise RuntimeError(f"Planning Scene verification failed: {detail}")
        node.get_logger().info(
            f"/get_planning_scene verified {manager.object_ids}: {detail}; zones absent"
        )
        _verify_robot_model(node)
        _verify_controllers(node)

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
        if not interface.wait_until_ready():
            raise RuntimeError("MoveIt interface did not become ready")

        safe_result = interface.move_to_pose(_safe_pose(motion))
        if safe_result != MotionResult.SUCCESS:
            raise RuntimeError(f"Safe target returned {safe_result.value}")
        node.get_logger().info("Safe target planned and executed with the scene active")

        before = interface.current_joint_positions()
        if before is None:
            raise RuntimeError("No robot state before collision test")
        colliding_pose = _table_collision_pose(scene, motion)
        _verify_table_collision(
            node, interface, colliding_pose, str(scene["table"]["name"])
        )
        collision_result = interface.move_to_pose(colliding_pose)
        if collision_result not in (
            MotionResult.INVALID_TARGET,
            MotionResult.PLANNING_FAILED,
        ):
            raise RuntimeError(
                f"Table-colliding target was not rejected: {collision_result.value}"
            )
        after = interface.current_joint_positions()
        if after is None or not _positions_close(before, after):
            raise RuntimeError("Arm moved while rejecting the table-colliding target")
        node.get_logger().info(
            "Table-colliding target rejected without trajectory execution or arm motion"
        )

        _verify_controllers(node)
        node.get_logger().info("M5 Planning Scene acceptance PASSED")
    except Exception as exc:
        node.get_logger().error(f"M5 Planning Scene acceptance FAILED: {exc}")
        raise
    finally:
        if interface is not None:
            interface.destroy()
        if manager is not None:
            manager.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
