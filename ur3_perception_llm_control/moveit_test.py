"""Run the M4 current -> home -> test pose -> home acceptance sequence."""

from __future__ import annotations

import math
import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
import yaml

from ur3_perception_llm_control.moveit_interface import (
    ARM_JOINTS,
    GRIPPER_JOINTS,
    MotionResult,
    MoveItArmInterface,
)


def _load_config() -> dict:
    path = os.path.join(
        get_package_share_directory("ur3_perception_llm_control"),
        "config",
        "robot_motion.yaml",
    )
    with open(path, encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError("robot_motion.yaml must contain a mapping")
    return config


def _pose_from_config(config: dict) -> PoseStamped:
    position = config["position"]
    orientation = config["orientation_xyzw"]
    if len(position) != 3 or len(orientation) != 4:
        raise ValueError("Configured pose needs XYZ and XYZW values")
    pose = PoseStamped()
    pose.header.frame_id = str(config["frame_id"])
    pose.pose.position.x = float(position[0])
    pose.pose.position.y = float(position[1])
    pose.pose.position.z = float(position[2])
    pose.pose.orientation.x = float(orientation[0])
    pose.pose.orientation.y = float(orientation[1])
    pose.pose.orientation.z = float(orientation[2])
    pose.pose.orientation.w = float(orientation[3])
    return pose


def _require_success(node: Node, label: str, result: MotionResult) -> None:
    node.get_logger().info(f"{label}: {result.value}")
    if result != MotionResult.SUCCESS:
        raise RuntimeError(f"{label} returned {result.value}")


def _positions_close(
    before: dict[str, float],
    after: dict[str, float],
    names,
    tolerance: float,
) -> bool:
    return all(
        name in before
        and name in after
        and math.isclose(before[name], after[name], abs_tol=tolerance)
        for name in names
    )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node("moveit_test")
    interface = None
    try:
        config = _load_config()
        home = {name: float(value) for name, value in config["home"].items()}
        if set(home) != set(ARM_JOINTS):
            raise ValueError("HOME must define exactly the six UR arm joints")
        interface = MoveItArmInterface(
            node,
            planning_group=str(config["planning_group"]),
            planning_frame=str(config["planning_frame"]),
            planning_tip=str(config["planning_tip"]),
            application_tip=str(config["application_tip"]),
            tool0_to_tcp_z=float(config["tool0_to_tcp_z"]),
            velocity_scaling=float(config["velocity_scaling"]),
            acceleration_scaling=float(config["acceleration_scaling"]),
            planning_time=float(config["planning_time"]),
            planning_attempts=int(config["planning_attempts"]),
        )
        if not interface.wait_until_ready():
            raise RuntimeError("MoveIt interface did not become ready")

        initial = interface.current_joint_positions()
        if initial is None:
            raise RuntimeError("No current robot state")
        node.get_logger().info(
            "Current arm state: "
            + ", ".join(f"{name}={initial[name]:.4f}" for name in ARM_JOINTS)
        )
        initial_gripper = {name: initial[name] for name in GRIPPER_JOINTS}

        _require_success(
            node,
            "current -> HOME",
            interface.move_to_joint_configuration(home),
        )
        _require_success(
            node,
            "HOME -> safe gripper_tcp pose",
            interface.move_to_pose(_pose_from_config(config["test_gripper_tcp_pose"])),
        )
        _require_success(
            node,
            "safe gripper_tcp pose -> HOME",
            interface.move_to_joint_configuration(home),
        )

        before_invalid = interface.current_joint_positions()
        invalid_result = interface.move_to_pose(
            _pose_from_config(config["invalid_gripper_tcp_pose"])
        )
        node.get_logger().info(f"Invalid target: {invalid_result.value}")
        if invalid_result != MotionResult.INVALID_TARGET:
            raise RuntimeError(
                "Unreachable target was not classified as INVALID_TARGET"
            )
        after_invalid = interface.current_joint_positions()
        if before_invalid is None or after_invalid is None:
            raise RuntimeError("Joint feedback unavailable after invalid-target test")
        if not _positions_close(
            before_invalid, after_invalid, ARM_JOINTS, tolerance=1e-4
        ):
            raise RuntimeError("Arm moved while rejecting the invalid target")
        if not _positions_close(
            initial_gripper, after_invalid, GRIPPER_JOINTS, tolerance=1e-4
        ):
            raise RuntimeError("Arm motion changed a gripper joint")
        node.get_logger().info(
            "M4 sequence PASSED; invalid target caused no execution and "
            "gripper joints were unchanged"
        )
    except Exception as exc:
        node.get_logger().error(f"M4 sequence FAILED: {exc}")
        raise
    finally:
        if interface is not None:
            interface.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
