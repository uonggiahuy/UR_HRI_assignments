"""Small YAML-backed manipulation motions expressed at ``gripper_tcp``."""

from __future__ import annotations

import math
from typing import Mapping

from geometry_msgs.msg import PoseStamped

from ur3_llm_control.moveit_interface import MotionResult, MoveItArmInterface


class ManipulationMotionPrimitives:
    """Approach, descend, and retreat named scene.yaml targets with MoveIt only."""

    def __init__(
        self,
        interface: MoveItArmInterface,
        scene: Mapping[str, object],
        motion: Mapping[str, object],
    ) -> None:
        self._interface = interface
        self._scene = scene
        settings = self._mapping(motion.get("manipulation"), "manipulation")
        self._orientation = self._vector(
            settings.get("orientation_xyzw"), 4, "manipulation.orientation_xyzw"
        )
        self._approach_clearance = self._positive(
            settings.get("approach_clearance"), "manipulation.approach_clearance"
        )
        self._grasp_z_offset = self._number(
            settings.get("grasp_z_offset"), "manipulation.grasp_z_offset"
        )
        self._retreat_clearance = self._positive(
            settings.get("retreat_clearance"), "manipulation.retreat_clearance"
        )
        self._last_target: str | None = None

    def move_above(self, target: str) -> MotionResult:
        """Collision-plan to the configured approach clearance above a target."""
        self._last_target = target
        return self._interface.move_to_pose(
            self._target_pose(target, self._approach_clearance)
        )

    def descend(self, target: str) -> MotionResult:
        """Collision-plan to the configured gripper-TCP grasp height."""
        self._last_target = target
        return self._interface.move_to_pose(
            self._target_pose(target, self._grasp_z_offset)
        )

    def retreat(self) -> MotionResult:
        """Collision-plan upward from the most recently named target."""
        if self._last_target is None:
            raise RuntimeError("retreat requires a preceding move_above or descend")
        return self._interface.move_to_pose(
            self._target_pose(self._last_target, self._retreat_clearance)
        )

    def target_world_pose(self, target: str) -> PoseStamped:
        """Return the canonical scene.yaml pose for placement or inspection."""
        model = self._target(target)
        robot = self._mapping(self._scene["robot"], "robot")
        pose = PoseStamped()
        pose.header.frame_id = str(robot["world_frame"])
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = model
        pose.pose.orientation.w = 1.0
        return pose

    def _target_pose(self, target: str, world_z_offset: float) -> PoseStamped:
        model = self._target(target)
        robot = self._mapping(self._scene["robot"], "robot")
        mount = self._mapping(robot["mount_pose"], "robot.mount_pose")
        pose = PoseStamped()
        pose.header.frame_id = self._interface.planning_frame
        pose.pose.position.x = model[0] - self._number(mount["x"], "robot.mount_pose.x")
        pose.pose.position.y = model[1] - self._number(mount["y"], "robot.mount_pose.y")
        mount_z = self._number(mount["z"], "robot.mount_pose.z")
        pose.pose.position.z = model[2] + world_z_offset - mount_z
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = self._orientation
        return pose

    def _target(self, target: str) -> tuple[float, float, float]:
        for group in ("objects", "zones"):
            entries = self._mapping(self._scene.get(group), group)
            candidate = entries.get(target)
            if candidate is not None:
                target_data = self._mapping(candidate, target)
                pose = self._mapping(target_data.get("pose"), f"{target}.pose")
                return tuple(
                    self._number(pose[name], f"{target}.pose.{name}")
                    for name in ("x", "y", "z")
                )
        raise ValueError(f"Unknown scene.yaml target: {target}")

    @staticmethod
    def _mapping(value: object, label: str) -> Mapping[str, object]:
        if not isinstance(value, Mapping):
            raise ValueError(f"{label} must be a mapping")
        return value

    @staticmethod
    def _number(value: object, label: str) -> float:
        try:
            number = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{label} must be numeric") from exc
        if not math.isfinite(number):
            raise ValueError(f"{label} must be finite")
        return number

    @classmethod
    def _positive(cls, value: object, label: str) -> float:
        number = cls._number(value, label)
        if number <= 0.0:
            raise ValueError(f"{label} must be positive")
        return number

    @classmethod
    def _vector(cls, value: object, size: int, label: str) -> tuple[float, ...]:
        if not isinstance(value, (list, tuple)) or len(value) != size:
            raise ValueError(f"{label} must have {size} values")
        result = tuple(cls._number(item, label) for item in value)
        if math.isclose(sum(item * item for item in result), 0.0):
            raise ValueError(f"{label} must be a non-zero quaternion")
        return result
