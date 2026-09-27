"""Completion-aware action API for the assignment parallel-jaw gripper."""

from typing import Sequence

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


GRIPPER_ACTION = "/gripper_controller/follow_joint_trajectory"
FINGER_JOINTS = ("left_finger_joint", "right_finger_joint")
# The 45 mm assignment cubes require a 45 mm or wider jaw gap.  This leaves a
# 2 mm total clearance while MoveIt's attachment owns the grasp constraint.
CLOSED_POSITION = 0.0235
OPEN_POSITION = 0.0375


class ParallelJawGripper:
    """Send symmetric finger trajectories and wait for controller completion."""

    def __init__(
        self,
        node: Node,
        *,
        action_name: str = GRIPPER_ACTION,
        server_timeout: float = 15.0,
        result_timeout: float = 15.0,
        motion_duration: float = 2.0,
    ) -> None:
        self._node = node
        self._client = ActionClient(node, FollowJointTrajectory, action_name)
        self._server_timeout = server_timeout
        self._result_timeout = result_timeout
        self._motion_duration = motion_duration

    def open(self) -> None:
        """Open both fingers to the 75 mm clear-width configuration."""
        self._command((OPEN_POSITION, OPEN_POSITION), "open")

    def close(self) -> None:
        """Close both fingers to the cube-safe 47 mm clear-width configuration."""
        self._command((CLOSED_POSITION, CLOSED_POSITION), "close")

    def destroy(self) -> None:
        """Release the action client explicitly before its node is destroyed."""
        self._client.destroy()

    def _command(self, positions: Sequence[float], label: str) -> None:
        if not self._client.wait_for_server(timeout_sec=self._server_timeout):
            raise RuntimeError(
                f"gripper action server unavailable after {self._server_timeout:.1f} s"
            )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(FINGER_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.velocities = [0.0, 0.0]
        point.time_from_start = Duration(seconds=self._motion_duration).to_msg()
        goal.trajectory.points = [point]
        goal.goal_time_tolerance = Duration(seconds=2.0).to_msg()

        send_future = self._client.send_goal_async(goal)
        self._wait_for_future(send_future, self._server_timeout, f"send {label} goal")
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"gripper controller rejected the {label} goal")

        result_future = goal_handle.get_result_async()
        try:
            self._wait_for_future(
                result_future,
                self._result_timeout,
                f"complete {label} goal",
            )
        except RuntimeError:
            goal_handle.cancel_goal_async()
            raise

        response = result_future.result()
        if response is None:
            raise RuntimeError(f"gripper {label} goal returned no result")
        if response.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                f"gripper {label} goal ended with action status {response.status}"
            )
        if response.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(
                f"gripper {label} trajectory failed: {response.result.error_string} "
                f"(code {response.result.error_code})"
            )
        self._node.get_logger().info(f"Gripper {label} completed")

    def _wait_for_future(self, future, timeout: float, operation: str) -> None:
        import rclpy

        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError(f"timed out waiting to {operation} after {timeout:.1f} s")
