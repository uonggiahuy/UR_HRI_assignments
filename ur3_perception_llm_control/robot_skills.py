"""Public, fail-closed skills with legacy or physical Gazebo grasp backends."""

from __future__ import annotations

from enum import Enum
from math import hypot
from typing import Callable, Mapping

from geometry_msgs.msg import PoseStamped

from ur3_perception_llm_control.gazebo_sync import GazeboAttachmentSynchronizer
from ur3_perception_llm_control.gripper import ParallelJawGripper
from ur3_perception_llm_control.motion_primitives import ManipulationMotionPrimitives
from ur3_perception_llm_control.moveit_interface import MotionResult, MoveItArmInterface
from ur3_perception_llm_control.planning_scene import PlanningSceneManager
from ur3_perception_llm_control.physical_grasp import PhysicalGraspManager
from ur3_perception_llm_control.perception_scene import PerceptionPlanningSceneSynchronizer, SceneSyncError
from ur3_perception_llm_control.perception_state import PerceptionSnapshot
from ur3_perception_llm_control.temporary_position import TemporaryPlacement, TemporaryPositionError, TemporaryPositionPlanner
from ur3_perception_llm_control.temporary_position_moveit import MoveItTemporaryFeasibility
from ur3_perception_llm_control.world_state import BLOCKS, LEGACY_STUDENT_OBJECTS, PLAN_LAYOUT_TOLERANCE_M


class SkillStatus(str, Enum):
    """Stable result values returned by :class:`RobotSkills`."""

    SUCCESS = "SUCCESS"
    FAILED = "FAILED"
    INVALID_OBJECT = "INVALID_OBJECT"
    INVALID_ZONE = "INVALID_ZONE"
    PLANNING_FAILED = "PLANNING_FAILED"
    EXECUTION_FAILED = "EXECUTION_FAILED"


class RobotSkills:
    """Compose validated arm, gripper, scene, and Gazebo primitives into skills.

    The caller owns lifecycle of the injected dependencies. A skill
    returns immediately on its first failed operation and never carries a plan
    or motion result into a later operation.
    """

    VALID_OBJECTS = LEGACY_STUDENT_OBJECTS
    VALID_ZONES = frozenset(("zone_a", "zone_b", "zone_c"))

    def __init__(
        self,
        *,
        interface: MoveItArmInterface,
        gripper: ParallelJawGripper,
        primitives: ManipulationMotionPrimitives,
        scene_manager: PlanningSceneManager,
        gazebo_sync: GazeboAttachmentSynchronizer | None = None,
        physical_grasp: PhysicalGraspManager | None = None,
        snapshot_source: Callable[[], PerceptionSnapshot] | None = None,
        scene_synchronizer: PerceptionPlanningSceneSynchronizer | None = None,
        now_sec: Callable[[], float] | None = None,
        home_configuration: Mapping[str, float],
        temporary_planner: TemporaryPositionPlanner | None = None,
        temporary_scene: Mapping[str, object] | None = None,
        temporary_motion: Mapping[str, object] | None = None,
        require_place_verification: bool = False,
    ) -> None:
        if (gazebo_sync is None) == (physical_grasp is None):
            raise ValueError("Provide exactly one Gazebo grasp backend")
        if physical_grasp is not None and any(value is None for value in
                                             (snapshot_source, scene_synchronizer, now_sec)):
            raise ValueError("Physical grasp requires camera snapshots, scene sync, and a clock")
        self._interface = interface
        self._gripper = gripper
        self._primitives = primitives
        self._scene_manager = scene_manager
        self._gazebo_sync = gazebo_sync
        self._physical_grasp = physical_grasp
        self._snapshot_source = snapshot_source
        self._scene_synchronizer = scene_synchronizer
        self._now_sec = now_sec
        self._last_observation_sec: float | None = None
        self._home_configuration = dict(home_configuration)
        self._temporary_planner = temporary_planner
        self._temporary_scene = temporary_scene
        self._temporary_motion = temporary_motion
        self._require_place_verification = require_place_verification
        self._reserved_temporary: TemporaryPlacement | None = None
        self._reservation_snapshot: PerceptionSnapshot | None = None
        self.last_verified_snapshot: PerceptionSnapshot | None = None

    @property
    def reserved_temporary(self) -> TemporaryPlacement | None:
        return self._reserved_temporary

    def clear_temporary_position(self) -> None:
        self._reserved_temporary = None
        self._reservation_snapshot = None

    def reserve_temporary_position(self, object_name: str,
                                   snapshot: PerceptionSnapshot) -> SkillStatus:
        """Reserve one immutable M9 target before any task motion."""
        self.clear_temporary_position()
        if (self._physical_grasp is None or self._temporary_planner is None
                or self._temporary_scene is None or self._temporary_motion is None
                or self._scene_synchronizer is None or self._now_sec is None
                or object_name not in BLOCKS or not self._physical_grasp.ready_for_pick()):
            return SkillStatus.FAILED
        try:
            now = self._now_sec()
            snapshot.require_fresh(now)
            report = self._scene_synchronizer.apply_snapshot(snapshot, now)
            checker = MoveItTemporaryFeasibility(
                self._interface, self._primitives, self._temporary_scene,
                self._temporary_motion, snapshot, report, object_name,
            )
            target, _ = self._temporary_planner.find_temporary_position(
                object_name, snapshot, self._now_sec(), checker,
            )
            self._reserved_temporary = target
            self._reservation_snapshot = snapshot
            self._last_observation_sec = snapshot.observation_timestamp_sec
            return SkillStatus.SUCCESS
        except (SceneSyncError, TemporaryPositionError, ValueError, TypeError, KeyError, RuntimeError) as exc:
            self._interface._node.get_logger().error(f"Temporary reservation failed: {exc}")
            return SkillStatus.FAILED

    def home(self) -> SkillStatus:
        """Plan and execute the validated HOME joint configuration."""
        return self._motion(self._interface.move_to_joint_configuration(self._home_configuration))

    def pick(self, object_name: str) -> SkillStatus:
        """Pick one configured cube through the validated motion and scene APIs.

        A fresh simulator starts in the stock straight-elbow posture, from
        which the approach planner can choose an invalid arm/gripper transit.
        Enter the configured, collision-checked HOME posture before every
        approach so callers never need to expose this transit as a task step.
        """
        if object_name not in (BLOCKS if self._physical_grasp is not None else self.VALID_OBJECTS):
            return SkillStatus.INVALID_OBJECT

        observed_pose = None
        if self._physical_grasp is not None:
            if not self._physical_grasp.ready_for_pick():
                return SkillStatus.FAILED
            observed_pose = self._synchronize_observation(object_name)
            if observed_pose is None:
                return SkillStatus.FAILED

        status = self.home()
        if status != SkillStatus.SUCCESS:
            return status
        status = self._gripper_command(self._gripper.open)
        if status != SkillStatus.SUCCESS:
            return status
        status = self._motion(self._primitives.move_above_world_pose(observed_pose)
                              if observed_pose is not None else self._primitives.move_above(object_name))
        if status != SkillStatus.SUCCESS:
            return status
        if not self._scene_call(self._scene_manager.allow_grasp_contact, object_name):
            return SkillStatus.FAILED
        status = self._motion(self._primitives.descend_to_world_pose(observed_pose)
                              if observed_pose is not None else self._primitives.descend(object_name))
        if status != SkillStatus.SUCCESS:
            return status
        status = self._gripper_command(self._gripper.close)
        if status != SkillStatus.SUCCESS:
            return status
        if self._physical_grasp is not None:
            return self._physical_pick_attachment(object_name, observed_pose)
        if not self._scene_call(self._scene_manager.attach_object, object_name):
            return SkillStatus.FAILED

        # Read the attachment created by this pick, rather than retaining a
        # pose from before the scene transition.
        attached_pose = self._scene_manager.attached_pose(object_name)
        if attached_pose is None:
            return SkillStatus.FAILED
        if not self._gazebo_sync.attach(
            object_name, self._scene_manager.attachment_link, attached_pose
        ):
            return SkillStatus.FAILED
        return self._motion(self._primitives.retreat(object_name))

    def place(self, object_name: str, zone_name: str) -> SkillStatus:
        """Place an attached configured cube at the configured zone pose."""
        if object_name not in (BLOCKS if self._physical_grasp is not None else self.VALID_OBJECTS):
            return SkillStatus.INVALID_OBJECT
        if zone_name not in self.VALID_ZONES:
            return SkillStatus.INVALID_ZONE

        if self._physical_grasp is not None:
            return self._physical_place(object_name, zone_name)

        # A public caller may construct this layer after a successful pick.
        # Refresh the authoritative attachment before moving, instead of using
        # an attachment transform retained by a previous call.
        attached_pose = self._scene_manager.attached_pose(object_name)
        if attached_pose is None:
            return SkillStatus.FAILED
        if not self._gazebo_sync.attach(
            object_name, self._scene_manager.attachment_link, attached_pose
        ):
            return SkillStatus.FAILED

        status = self._motion(self._primitives.move_above(zone_name))
        if status != SkillStatus.SUCCESS:
            return status
        status = self._motion(
            self._primitives.descend_to_placement(object_name, zone_name)
        )
        if status != SkillStatus.SUCCESS:
            return status

        # The placement pose comes from the zone top and cube geometry in
        # scene.yaml; it is fetched only after the successful descent.
        placement_pose = self._primitives.placement_world_pose(object_name, zone_name)
        if not self._gazebo_sync.release(placement_pose):
            return SkillStatus.FAILED
        if not self._scene_call(
            self._scene_manager.detach_object, object_name, placement_pose
        ):
            return SkillStatus.FAILED
        status = self._gripper_command(self._gripper.open)
        if status != SkillStatus.SUCCESS:
            return status
        status = self._motion(self._primitives.retreat(zone_name))
        if status != SkillStatus.SUCCESS:
            return status
        if not self._gazebo_sync.set_world_pose(object_name, placement_pose):
            return SkillStatus.FAILED
        if not self._scene_call(self._scene_manager.clear_grasp_contact, object_name):
            return SkillStatus.FAILED
        return SkillStatus.SUCCESS

    def place_temp(self, object_name: str) -> SkillStatus:
        """Place the held physical cube at its previously reserved table slot."""
        target = self._reserved_temporary
        try:
            if (self._physical_grasp is None or target is None
                    or target.object_name != object_name):
                return SkillStatus.FAILED
            placement = PoseStamped()
            placement.header.frame_id = self._scene_synchronizer.frame_id
            placement.pose.position.x = target.x
            placement.pose.position.y = target.y
            placement.pose.position.z = target.z
            placement.pose.orientation.w = 1.0
            return self._physical_place_at(object_name, placement, expected_location="table")
        finally:
            self.clear_temporary_position()

    def _exclusive_state(self, name: str, expected: str) -> bool:
        scene = self._scene_manager.get()
        if scene is None:
            return False
        world = {item.id for item in scene.world.collision_objects}
        attached = {item.object.id for item in scene.robot_state.attached_collision_objects}
        return (name in world, name in attached) == (
            (True, False) if expected == "WORLD" else (False, True)
        )

    def _synchronize_observation(self, name: str) -> PoseStamped | None:
        """Use one fresh RGB frame for both the verified scene and pick pose."""
        assert self._snapshot_source and self._scene_synchronizer and self._now_sec
        try:
            snapshot = self._snapshot_source()
            if not isinstance(snapshot, PerceptionSnapshot):
                return None
            if self._reservation_snapshot is not None:
                reserved = self._reservation_snapshot
                if (dict(snapshot.object_locations) != dict(reserved.object_locations)
                        or any(hypot(snapshot.object_world_xy[cube][0] - reserved.object_world_xy[cube][0],
                                     snapshot.object_world_xy[cube][1] - reserved.object_world_xy[cube][1])
                               > PLAN_LAYOUT_TOLERANCE_M for cube in BLOCKS)):
                    return None
            report = self._scene_synchronizer.apply_snapshot(snapshot, self._now_sec())
            if report.attached_ids or name not in report.authoritative_xyz:
                return None
            snapshot.require_fresh(self._now_sec())
            xyz = report.requested_xyz[name]
            if tuple(snapshot.object_world_xy[name]) != tuple(xyz[:2]):
                return None
            pose = PoseStamped()
            pose.header.frame_id = self._scene_synchronizer.frame_id
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = xyz
            pose.pose.orientation.w = 1.0
            self._last_observation_sec = snapshot.observation_timestamp_sec
            return pose
        except (SceneSyncError, ValueError, TypeError, KeyError, RuntimeError) as exc:
            self._interface._node.get_logger().error(f"Pick observation/scene sync failed: {exc}")
            return None

    def _physical_pick_attachment(self, name: str, observed_pose: PoseStamped) -> SkillStatus:
        """Commit Gazebo then MoveIt, or release and restore before retreat."""
        physical = self._physical_grasp
        assert physical is not None
        scene = self._scene_manager.get()
        if scene is None:
            return SkillStatus.FAILED
        world_object = next((item for item in scene.world.collision_objects if item.id == name), None)
        if world_object is None or not self._exclusive_state(name, "WORLD"):
            return SkillStatus.FAILED
        original = PoseStamped()
        original.header.frame_id = world_object.header.frame_id
        original.pose = world_object.pose
        if not physical.attach(name) or not physical.is_attached(name):
            if physical.is_attached(name):
                physical.release(name)
            self._interface.stop()
            return SkillStatus.FAILED
        if not self._scene_call(self._scene_manager.attach_object, name) or not self._exclusive_state(name, "ATTACHED"):
            # An apply call can report failure after changing the scene. Query
            # and reverse either successful side while the robot is stationary.
            if self._exclusive_state(name, "ATTACHED"):
                self._scene_call(self._scene_manager.detach_object, name, original)
            if physical.is_attached(name):
                physical.release(name)
            self._interface.stop()
            return SkillStatus.FAILED
        if not physical.is_attached(name):
            self._scene_call(self._scene_manager.detach_object, name, original)
            self._interface.stop()
            return SkillStatus.FAILED
        return self._motion(self._primitives.retreat_from_world_pose(observed_pose))

    def _physical_place(self, name: str, zone_name: str) -> SkillStatus:
        """At rest, move MoveIt to WORLD, release joint, then lift and open.

        If physical release fails, restore the MoveIt attachment while the
        wrist is stationary. No retreat occurs in an inconsistent state.
        """
        placement = self._primitives.placement_world_pose(name, zone_name)
        return self._physical_place_at(
            name, placement,
            expected_location=zone_name if self._require_place_verification else None,
        )

    def _physical_place_at(self, name: str, placement: PoseStamped,
                           *, expected_location: str | None) -> SkillStatus:
        physical = self._physical_grasp
        assert physical is not None
        if not physical.is_attached(name) or not self._exclusive_state(name, "ATTACHED"):
            return SkillStatus.FAILED
        for operation in (
            lambda: self._primitives.move_above_world_pose(placement),
            lambda: self._primitives.descend_to_world_pose(placement),
        ):
            status = self._motion(operation())
            if status != SkillStatus.SUCCESS:
                return status
            if not physical.is_attached(name):
                self._interface.stop()
                return SkillStatus.FAILED
        detached = self._scene_call(self._scene_manager.detach_object, name, placement)
        if not detached or not self._exclusive_state(name, "WORLD"):
            self._interface._node.get_logger().error("Physical place: MoveIt WORLD transition failed")
            if self._exclusive_state(name, "WORLD"):
                self._scene_call(self._scene_manager.attach_object, name)
            self._interface.stop()
            return SkillStatus.FAILED
        release_ok = physical.release(name)
        if not release_ok or physical.is_attached(name):
            self._interface._node.get_logger().error("Physical place: Gazebo detach verification failed")
            if physical.is_attached(name):
                self._scene_call(self._scene_manager.attach_object, name)
            self._interface.stop()
            return SkillStatus.FAILED
        status = self._motion(self._primitives.retreat_from_world_pose(placement))
        if status != SkillStatus.SUCCESS:
            return status
        status = self._gripper_command(self._gripper.open)
        if status != SkillStatus.SUCCESS:
            return status
        if not self._scene_call(self._scene_manager.clear_grasp_contact, name):
            self._interface._node.get_logger().error("Physical place: grasp contact reset failed")
            return SkillStatus.FAILED
        # The overhead camera cannot see the released cube while the wrist is
        # still over its zone. Move to the verified HOME posture before RGB.
        status = self.home()
        if status != SkillStatus.SUCCESS:
            return status
        # The cube settles under Gazebo physics; refresh from a later RGB frame.
        assert self._snapshot_source and self._scene_synchronizer and self._now_sec
        try:
            snapshot = self._snapshot_source()
            if (not isinstance(snapshot, PerceptionSnapshot)
                    or self._last_observation_sec is None
                    or snapshot.observation_timestamp_sec <= self._last_observation_sec):
                return SkillStatus.FAILED
            report = self._scene_synchronizer.apply_snapshot(snapshot, self._now_sec())
            if name in report.attached_ids:
                return SkillStatus.FAILED
            if expected_location is not None:
                if snapshot.object_locations[name] != expected_location:
                    return SkillStatus.FAILED
                if expected_location == "table":
                    previous = (self._reservation_snapshot.object_locations[name]
                                if self._reservation_snapshot is not None else None)
                    if previous in ("zone_a", "zone_b", "zone_c") and snapshot.zone_occupancy[previous] is not None:
                        return SkillStatus.FAILED
                elif snapshot.zone_occupancy[expected_location] != name:
                    return SkillStatus.FAILED
            self.last_verified_snapshot = snapshot
            self._last_observation_sec = snapshot.observation_timestamp_sec
        except (SceneSyncError, ValueError, TypeError, KeyError, RuntimeError) as exc:
            self._interface._node.get_logger().error(f"Post-release observation/scene sync failed: {exc}")
            return SkillStatus.FAILED
        return SkillStatus.SUCCESS

    def _motion(self, result: MotionResult) -> SkillStatus:
        if result == MotionResult.SUCCESS:
            return SkillStatus.SUCCESS
        # The M4 interface creates a fresh plan for every call.  Explicitly
        # cancel any in-flight action before returning a failed skill status.
        self._interface.stop()
        if result == MotionResult.PLANNING_FAILED:
            return SkillStatus.PLANNING_FAILED
        if result == MotionResult.EXECUTION_FAILED:
            return SkillStatus.EXECUTION_FAILED
        return SkillStatus.FAILED

    def _gripper_command(self, command) -> SkillStatus:
        try:
            command()
        except Exception as exc:
            self._interface._node.get_logger().error(f"Gripper command failed: {exc}")
            self._interface.stop()
            return SkillStatus.FAILED
        return SkillStatus.SUCCESS

    @staticmethod
    def _scene_call(operation, *args) -> bool:
        try:
            return bool(operation(*args))
        except Exception:
            return False
