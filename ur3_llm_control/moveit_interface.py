"""Reusable, completion-aware MoveIt interface for the UR arm."""

from __future__ import annotations

from copy import deepcopy
from enum import Enum
import math
import time
from typing import Mapping, Optional, Sequence, Tuple

from action_msgs.msg import GoalStatus
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    RobotState,
    RobotTrajectory,
)
from moveit_msgs.srv import GetPositionIK, GetStateValidity
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


ARM_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
GRIPPER_JOINTS = ("left_finger_joint", "right_finger_joint")


class MotionResult(str, Enum):
    """Stable application-facing result categories."""

    SUCCESS = "SUCCESS"
    PLANNING_FAILED = "PLANNING_FAILED"
    EXECUTION_FAILED = "EXECUTION_FAILED"
    INVALID_TARGET = "INVALID_TARGET"
    CANCELLED = "CANCELLED"
    NOT_READY = "NOT_READY"


class MoveItArmInterface:
    """Plan with MoveIt and execute only MoveIt-generated trajectories.

    Pose targets are poses of ``gripper_tcp``. The configured MoveIt group
    ends at ``tool0``, so the fixed tool0-to-TCP offset is removed before IK.
    """

    def __init__(
        self,
        node: Node,
        *,
        planning_group: str = "ur_manipulator",
        planning_frame: str = "base_link",
        planning_tip: str = "tool0",
        application_tip: str = "gripper_tcp",
        tool0_to_tcp_z: float = 0.080,
        velocity_scaling: float = 0.10,
        acceleration_scaling: float = 0.10,
        planning_time: float = 5.0,
        planning_attempts: int = 5,
        wait_timeout: float = 30.0,
        operation_timeout: float = 30.0,
        joint_state_max_age: float = 1.0,
    ) -> None:
        self._node = node
        self.planning_group = planning_group
        self.planning_frame = planning_frame
        self.planning_tip = planning_tip
        self.application_tip = application_tip
        self.tool0_to_tcp_z = tool0_to_tcp_z
        self.velocity_scaling = velocity_scaling
        self.acceleration_scaling = acceleration_scaling
        self.planning_time = planning_time
        self.planning_attempts = planning_attempts
        self.wait_timeout = wait_timeout
        self.operation_timeout = operation_timeout
        self.joint_state_max_age = joint_state_max_age
        self._validate_settings()

        self._move_client = ActionClient(node, MoveGroup, "/move_action")
        self._execute_client = ActionClient(
            node, ExecuteTrajectory, "/execute_trajectory"
        )
        self._ik_client = node.create_client(GetPositionIK, "/compute_ik")
        self._validity_client = node.create_client(
            GetStateValidity, "/check_state_validity"
        )
        self._controller_client = node.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self._joint_state_sub = node.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self._latest_joint_state: Optional[JointState] = None
        self._joint_state_received_at: Optional[float] = None
        self._joint_state_sequence = 0
        self._planning_goal_handle = None
        self._execution_goal_handle = None

    def _validate_settings(self) -> None:
        if not self.planning_group or not self.planning_frame:
            raise ValueError("planning group and frame must be non-empty")
        if self.planning_tip != "tool0" or self.application_tip != "gripper_tcp":
            raise ValueError("M4 requires tool0 planning and gripper_tcp targets")
        if not 0.0 < self.tool0_to_tcp_z < 0.5:
            raise ValueError("tool0_to_tcp_z must be a positive distance")
        if not 0.0 < self.velocity_scaling <= 1.0:
            raise ValueError("velocity_scaling must be in (0, 1]")
        if not 0.0 < self.acceleration_scaling <= 1.0:
            raise ValueError("acceleration_scaling must be in (0, 1]")
        if self.planning_time <= 0.0 or self.planning_attempts < 1:
            raise ValueError("planning time and attempts must be positive")

    def _on_joint_state(self, message: JointState) -> None:
        self._latest_joint_state = deepcopy(message)
        self._joint_state_received_at = time.monotonic()
        self._joint_state_sequence += 1

    def wait_until_ready(self, timeout: Optional[float] = None) -> bool:
        """Wait for MoveIt, the arm controller, and a fresh robot state."""
        deadline = time.monotonic() + (timeout or self.wait_timeout)
        while time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.1)
            services_ready = (
                self._ik_client.wait_for_service(timeout_sec=0.1)
                and self._validity_client.wait_for_service(timeout_sec=0.1)
                and self._controller_client.wait_for_service(timeout_sec=0.1)
            )
            actions_ready = (
                self._move_client.wait_for_server(timeout_sec=0.1)
                and self._execute_client.wait_for_server(timeout_sec=0.1)
            )
            if (
                services_ready
                and actions_ready
                and self._current_robot_state(log_error=False) is not None
                and self._arm_controller_active(log_error=False)
            ):
                return True
        self._node.get_logger().error(
            "MoveIt, arm controller, or a fresh /joint_states sample is unavailable"
        )
        return False

    def current_joint_positions(self) -> Optional[dict[str, float]]:
        """Return a name-to-position snapshot when joint feedback is fresh."""
        state = self._current_robot_state(log_error=False)
        if state is None:
            return None
        return dict(zip(state.joint_state.name, state.joint_state.position))

    def move_to_joint_configuration(
        self, joint_positions: Mapping[str, float]
    ) -> MotionResult:
        """Plan and execute a collision-checked six-joint arm target."""
        if set(joint_positions) != set(ARM_JOINTS):
            self._node.get_logger().error(
                "Joint target must contain exactly the six UR arm joints"
            )
            return MotionResult.INVALID_TARGET
        if not all(math.isfinite(float(value)) for value in joint_positions.values()):
            self._node.get_logger().error("Joint target contains a non-finite value")
            return MotionResult.INVALID_TARGET
        return self._plan_and_execute(joint_positions, "joint configuration")

    def move_to_pose(self, gripper_tcp_pose: PoseStamped) -> MotionResult:
        """Move the application TCP to a desired pose via tool0 planning."""
        normalized = self._validated_pose(gripper_tcp_pose)
        if normalized is None:
            return MotionResult.INVALID_TARGET
        if not self.wait_until_ready():
            return MotionResult.NOT_READY
        state = self._current_robot_state()
        if state is None:
            return MotionResult.NOT_READY

        tool0_pose = self._gripper_tcp_to_tool0(normalized)
        current = dict(zip(state.joint_state.name, state.joint_state.position))
        self._node.get_logger().info(
            "IK seed arm state: " + ", ".join(f"{name}={current[name]:.4f}" for name in ARM_JOINTS)
        )
        candidates: list[tuple[float, dict[str, float]]] = []
        seen: set[tuple[int, ...]] = set()
        seeds = self._ik_seed_positions(current)
        for index, seed in enumerate(seeds, 1):
            self._node.get_logger().info(f"IK seed {index}/{len(seeds)}: " + ", ".join(f"{n}={seed[n]:.3f}" for n in ARM_JOINTS))
            seed_state = self._state_with_arm_positions(state, seed)
            if seed_state is None:
                continue
            request = GetPositionIK.Request(); request.ik_request.group_name = self.planning_group
            request.ik_request.robot_state = seed_state; request.ik_request.avoid_collisions = True
            request.ik_request.ik_link_name = self.planning_tip; request.ik_request.pose_stamped = tool0_pose
            request.ik_request.timeout = Duration(seconds=2.0).to_msg()
            response = self._call_service(self._ik_client, request, "collision-aware IK")
            if response is None or response.error_code.val != MoveItErrorCodes.SUCCESS:
                continue
            solution = dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))
            if not all(name in solution for name in ARM_JOINTS):
                continue
            candidate = {n: self._nearest_equivalent_revolute(solution[n], current[n]) for n in ARM_JOINTS}
            key = tuple(round(candidate[n] * 10000) for n in ARM_JOINTS)
            if key in seen:
                continue
            seen.add(key)
            target = self._state_with_arm_positions(state, candidate)
            if target is None or not self._state_is_valid(target, "gripper_tcp pose"):
                self._node.get_logger().info(f"IK candidate {index}: state validity INVALID")
                continue
            distance = self._joint_distance(candidate, current)
            self._node.get_logger().info(f"IK candidate {index}: state validity VALID; distance={distance:.4f}")
            candidates.append((distance, candidate))
        if not candidates:
            return MotionResult.INVALID_TARGET
        for index, (_, candidate) in enumerate(sorted(candidates, key=lambda item: item[0]), 1):
            result, trajectory = self._plan(state, candidate, f"gripper_tcp pose candidate {index}")
            self._node.get_logger().info(f"planning candidate {index}: {'SUCCESS' if result == MotionResult.SUCCESS else 'FAIL'}")
            if result == MotionResult.SUCCESS and trajectory is not None:
                self._node.get_logger().info(f"selected candidate: {index}")
                return self._execute(trajectory, "gripper_tcp pose")
        return MotionResult.PLANNING_FAILED

    @staticmethod
    def _nearest_equivalent_revolute(value: float, reference: float) -> float:
        """Represent an equivalent revolute target nearest the measured state."""
        return value + (2.0 * math.pi) * round((reference - value) / (2.0 * math.pi))

    @staticmethod
    def _joint_distance(left: Mapping[str, float], right: Mapping[str, float]) -> float:
        return math.sqrt(sum((left[name] - right[name]) ** 2 for name in ARM_JOINTS))

    @staticmethod
    def _ik_seed_positions(current: Mapping[str, float]) -> tuple[dict[str, float], ...]:
        """Deterministic bounded UR seeds: current first, then elbow/shoulder/wrist flips."""
        offsets = ((), (("elbow_joint", math.pi),), (("shoulder_pan_joint", math.pi),), (("wrist_1_joint", math.pi),))
        result = []
        for changes in offsets:
            seed = dict(current)
            for name, offset in changes:
                seed[name] = max(-2.0 * math.pi, min(2.0 * math.pi, seed[name] + offset))
            result.append(seed)
        return tuple(result)

    def stop(self) -> bool:
        """Request cancellation of any active planning or execution goal."""
        cancelled = False
        for goal_handle in (
            self._execution_goal_handle,
            self._planning_goal_handle,
        ):
            if goal_handle is not None:
                future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(
                    self._node, future, timeout_sec=2.0
                )
                cancelled = future.done() or cancelled
        return cancelled

    def destroy(self) -> None:
        """Release clients and subscriptions owned by the interface."""
        self._move_client.destroy()
        self._execute_client.destroy()
        self._node.destroy_client(self._ik_client)
        self._node.destroy_client(self._validity_client)
        self._node.destroy_client(self._controller_client)
        self._node.destroy_subscription(self._joint_state_sub)

    def _plan_and_execute(
        self, joint_positions: Mapping[str, float], label: str
    ) -> MotionResult:
        if not self.wait_until_ready():
            return MotionResult.NOT_READY
        start_state = self._current_robot_state()
        if start_state is None:
            return MotionResult.NOT_READY
        target_state = self._state_with_arm_positions(start_state, joint_positions)
        if target_state is None or not self._state_is_valid(target_state, label):
            return MotionResult.INVALID_TARGET

        plan_result, trajectory = self._plan(start_state, joint_positions, label)
        if plan_result != MotionResult.SUCCESS or trajectory is None:
            return plan_result
        if not self._arm_controller_active():
            return MotionResult.NOT_READY
        return self._execute(trajectory, label)

    def _plan(
        self,
        start_state: RobotState,
        joint_positions: Mapping[str, float],
        label: str,
    ) -> Tuple[MotionResult, Optional[RobotTrajectory]]:
        goal = MoveGroup.Goal()
        goal.request.group_name = self.planning_group
        goal.request.start_state = start_state
        goal.request.goal_constraints = [self._joint_constraints(joint_positions)]
        goal.request.num_planning_attempts = self.planning_attempts
        goal.request.allowed_planning_time = self.planning_time
        goal.request.max_velocity_scaling_factor = self.velocity_scaling
        goal.request.max_acceleration_scaling_factor = self.acceleration_scaling
        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        send_future = self._move_client.send_goal_async(goal)
        if not self._wait_for_future(send_future, self.operation_timeout):
            self._node.get_logger().error(f"{label}: planning request timed out")
            return MotionResult.PLANNING_FAILED, None
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._node.get_logger().error(f"{label}: planning goal was rejected")
            return MotionResult.PLANNING_FAILED, None
        self._planning_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, self.operation_timeout):
            goal_handle.cancel_goal_async()
            self._planning_goal_handle = None
            self._node.get_logger().error(f"{label}: planning result timed out")
            return MotionResult.PLANNING_FAILED, None
        wrapped_result = result_future.result()
        self._planning_goal_handle = None
        if wrapped_result is None:
            return MotionResult.PLANNING_FAILED, None
        if wrapped_result.status == GoalStatus.STATUS_CANCELED:
            return MotionResult.CANCELLED, None
        result = wrapped_result.result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self._node.get_logger().error(
                f"{label}: MoveIt planning failed with code {result.error_code.val}"
            )
            return MotionResult.PLANNING_FAILED, None
        if not self._trajectory_is_valid(result.planned_trajectory):
            self._node.get_logger().error(f"{label}: MoveIt returned an invalid trajectory")
            return MotionResult.PLANNING_FAILED, None
        self._node.get_logger().info(
            f"{label}: collision-aware plan succeeded in {result.planning_time:.3f} s"
        )
        return MotionResult.SUCCESS, result.planned_trajectory

    def _execute(self, trajectory: RobotTrajectory, label: str) -> MotionResult:
        sequence_before = self._joint_state_sequence
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        send_future = self._execute_client.send_goal_async(goal)
        if not self._wait_for_future(send_future, self.operation_timeout):
            return MotionResult.EXECUTION_FAILED
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._node.get_logger().error(f"{label}: execution goal was rejected")
            return MotionResult.EXECUTION_FAILED
        self._execution_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, self.operation_timeout):
            goal_handle.cancel_goal_async()
            self._execution_goal_handle = None
            self._node.get_logger().error(f"{label}: execution result timed out")
            return MotionResult.EXECUTION_FAILED
        wrapped_result = result_future.result()
        self._execution_goal_handle = None
        if wrapped_result is None:
            return MotionResult.EXECUTION_FAILED
        if wrapped_result.status == GoalStatus.STATUS_CANCELED:
            return MotionResult.CANCELLED
        if wrapped_result.result.error_code.val != MoveItErrorCodes.SUCCESS:
            self._node.get_logger().error(
                f"{label}: execution failed with MoveIt code "
                f"{wrapped_result.result.error_code.val}"
            )
            return MotionResult.EXECUTION_FAILED
        if not self._wait_for_fresh_joint_state(sequence_before):
            return MotionResult.EXECUTION_FAILED
        self._node.get_logger().info(f"{label}: execution completed")
        return MotionResult.SUCCESS

    def _state_is_valid(self, state: RobotState, label: str) -> bool:
        request = GetStateValidity.Request()
        request.robot_state = state
        request.group_name = self.planning_group
        response = self._call_service(
            self._validity_client, request, f"state validity for {label}"
        )
        if response is None or not response.valid:
            contacts = [] if response is None else response.contacts
            pairs = [
                f"{contact.contact_body_1}/{contact.contact_body_2}"
                for contact in contacts[:3]
            ]
            detail = ", ".join(pairs) if pairs else "bounds or collision"
            self._node.get_logger().warning(
                f"{label}: target state rejected ({detail})"
            )
            return False
        return True

    def _arm_controller_active(self, log_error: bool = True) -> bool:
        if not self._controller_client.service_is_ready():
            return False
        response = self._call_service(
            self._controller_client,
            ListControllers.Request(),
            "controller state",
            log_error=log_error,
        )
        active = response is not None and any(
            controller.name == "joint_trajectory_controller"
            and controller.state == "active"
            for controller in response.controller
        )
        if not active and log_error:
            self._node.get_logger().error(
                "joint_trajectory_controller is not active"
            )
        return active

    def _current_robot_state(self, log_error: bool = True) -> Optional[RobotState]:
        message = self._latest_joint_state
        received_at = self._joint_state_received_at
        valid = (
            message is not None
            and received_at is not None
            and len(message.name) == len(message.position)
            and set(ARM_JOINTS).issubset(message.name)
            and set(GRIPPER_JOINTS).issubset(message.name)
            and time.monotonic() - received_at <= self.joint_state_max_age
        )
        if not valid:
            if log_error:
                self._node.get_logger().error(
                    "A fresh complete arm-and-gripper /joint_states sample is required"
                )
            return None
        state = RobotState()
        state.joint_state = deepcopy(message)
        state.is_diff = False
        return state

    @staticmethod
    def _state_with_arm_positions(
        start_state: RobotState, joint_positions: Mapping[str, float]
    ) -> Optional[RobotState]:
        state = deepcopy(start_state)
        index = {name: offset for offset, name in enumerate(state.joint_state.name)}
        if not all(name in index for name in ARM_JOINTS):
            return None
        positions = list(state.joint_state.position)
        for name in ARM_JOINTS:
            positions[index[name]] = float(joint_positions[name])
        state.joint_state.position = positions
        state.joint_state.velocity = []
        state.joint_state.effort = []
        state.is_diff = False
        return state

    @staticmethod
    def _joint_constraints(joint_positions: Mapping[str, float]) -> Constraints:
        constraints = Constraints()
        constraints.name = "m4_arm_target"
        for name in ARM_JOINTS:
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(joint_positions[name])
            joint.tolerance_above = 0.001
            joint.tolerance_below = 0.001
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        return constraints

    @staticmethod
    def _trajectory_is_valid(trajectory: RobotTrajectory) -> bool:
        joint_trajectory = trajectory.joint_trajectory
        names = list(joint_trajectory.joint_names)
        if set(names) != set(ARM_JOINTS) or not joint_trajectory.points:
            return False
        previous_time = -1.0
        for point in joint_trajectory.points:
            if len(point.positions) != len(names):
                return False
            values: Sequence[float] = point.positions
            if not all(math.isfinite(value) for value in values):
                return False
            current_time = (
                float(point.time_from_start.sec)
                + float(point.time_from_start.nanosec) * 1e-9
            )
            if current_time < previous_time:
                return False
            previous_time = current_time
        return previous_time >= 0.0

    def _validated_pose(self, pose: PoseStamped) -> Optional[PoseStamped]:
        if pose.header.frame_id != self.planning_frame:
            self._node.get_logger().error(
                f"Pose frame must be {self.planning_frame}, got {pose.header.frame_id!r}"
            )
            return None
        values = (
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        )
        if not all(math.isfinite(value) for value in values):
            self._node.get_logger().error("Pose contains a non-finite value")
            return None
        q = pose.pose.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm < 1e-9:
            self._node.get_logger().error("Pose quaternion has zero length")
            return None
        result = deepcopy(pose)
        result.pose.orientation.x /= norm
        result.pose.orientation.y /= norm
        result.pose.orientation.z /= norm
        result.pose.orientation.w /= norm
        return result

    def _gripper_tcp_to_tool0(self, tcp_pose: PoseStamped) -> PoseStamped:
        q = tcp_pose.pose.orientation
        offset = self._rotate_vector(
            (q.x, q.y, q.z, q.w), (0.0, 0.0, self.tool0_to_tcp_z)
        )
        tool0 = deepcopy(tcp_pose)
        tool0.pose.position.x -= offset[0]
        tool0.pose.position.y -= offset[1]
        tool0.pose.position.z -= offset[2]
        self._node.get_logger().info(
            "Converted gripper_tcp target to tool0 target: "
            f"tcp=({tcp_pose.pose.position.x:.3f}, "
            f"{tcp_pose.pose.position.y:.3f}, {tcp_pose.pose.position.z:.3f}), "
            f"tool0=({tool0.pose.position.x:.3f}, "
            f"{tool0.pose.position.y:.3f}, {tool0.pose.position.z:.3f})"
        )
        return tool0

    @staticmethod
    def _rotate_vector(
        quaternion: tuple[float, float, float, float],
        vector: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        qx, qy, qz, qw = quaternion
        vx, vy, vz = vector
        tx = 2.0 * (qy * vz - qz * vy)
        ty = 2.0 * (qz * vx - qx * vz)
        tz = 2.0 * (qx * vy - qy * vx)
        return (
            vx + qw * tx + qy * tz - qz * ty,
            vy + qw * ty + qz * tx - qx * tz,
            vz + qw * tz + qx * ty - qy * tx,
        )

    def _call_service(
        self, client, request, label: str, *, log_error: bool = True
    ):
        future = client.call_async(request)
        if not self._wait_for_future(future, self.operation_timeout):
            future.cancel()
            if log_error:
                self._node.get_logger().error(f"Timed out waiting for {label}")
            return None
        return future.result()

    def _wait_for_future(self, future, timeout: float) -> bool:
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout)
        return future.done()

    def _wait_for_fresh_joint_state(self, sequence: int) -> bool:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.1)
            if (
                self._joint_state_sequence > sequence
                and self._current_robot_state(log_error=False) is not None
            ):
                return True
        self._node.get_logger().error(
            "No fresh valid /joint_states feedback after execution"
        )
        return False
