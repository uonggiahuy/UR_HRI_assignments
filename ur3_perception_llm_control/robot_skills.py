"""Public, fail-closed skills with legacy or physical Gazebo grasp backends."""

from __future__ import annotations

from enum import Enum
from typing import Mapping

from geometry_msgs.msg import PoseStamped

from ur3_perception_llm_control.gazebo_sync import GazeboAttachmentSynchronizer
from ur3_perception_llm_control.gripper import ParallelJawGripper
from ur3_perception_llm_control.motion_primitives import ManipulationMotionPrimitives
from ur3_perception_llm_control.moveit_interface import MotionResult, MoveItArmInterface
from ur3_perception_llm_control.planning_scene import PlanningSceneManager
from ur3_perception_llm_control.physical_grasp import PhysicalGraspManager
from ur3_perception_llm_control.world_state import LEGACY_STUDENT_OBJECTS


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
        home_configuration: Mapping[str, float],
    ) -> None:
        if (gazebo_sync is None) == (physical_grasp is None):
            raise ValueError("Provide exactly one Gazebo grasp backend")
        self._interface = interface
        self._gripper = gripper
        self._primitives = primitives
        self._scene_manager = scene_manager
        self._gazebo_sync = gazebo_sync
        self._physical_grasp = physical_grasp
        self._home_configuration = dict(home_configuration)

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
        if object_name not in self.VALID_OBJECTS:
            return SkillStatus.INVALID_OBJECT

        status = self.home()
        if status != SkillStatus.SUCCESS:
            return status
        status = self._gripper_command(self._gripper.open)
        if status != SkillStatus.SUCCESS:
            return status
        status = self._motion(self._primitives.move_above(object_name))
        if status != SkillStatus.SUCCESS:
            return status
        if not self._scene_call(self._scene_manager.allow_grasp_contact, object_name):
            return SkillStatus.FAILED
        status = self._motion(self._primitives.descend(object_name))
        if status != SkillStatus.SUCCESS:
            return status
        status = self._gripper_command(self._gripper.close)
        if status != SkillStatus.SUCCESS:
            return status
        if self._physical_grasp is not None:
            return self._physical_pick_attachment(object_name)
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
        if object_name not in self.VALID_OBJECTS:
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

    def _exclusive_state(self, name: str, expected: str) -> bool:
        scene = self._scene_manager.get()
        if scene is None:
            return False
        world = {item.id for item in scene.world.collision_objects}
        attached = {item.object.id for item in scene.robot_state.attached_collision_objects}
        return (name in world, name in attached) == (
            (True, False) if expected == "WORLD" else (False, True)
        )

    def _physical_pick_attachment(self, name: str) -> SkillStatus:
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
        return self._motion(self._primitives.retreat(name))

    def _physical_place(self, name: str, zone_name: str) -> SkillStatus:
        """At rest, move MoveIt to WORLD, release joint, then open jaws.

        If physical release fails, restore the MoveIt attachment while the
        wrist is stationary. No retreat occurs in an inconsistent state.
        """
        physical = self._physical_grasp
        assert physical is not None
        if not physical.is_attached(name) or not self._exclusive_state(name, "ATTACHED"):
            return SkillStatus.FAILED
        for operation in (
            lambda: self._primitives.move_above(zone_name),
            lambda: self._primitives.descend_to_placement(name, zone_name),
        ):
            status = self._motion(operation())
            if status != SkillStatus.SUCCESS:
                return status
            if not physical.is_attached(name):
                self._interface.stop()
                return SkillStatus.FAILED
        placement = self._primitives.placement_world_pose(name, zone_name)
        detached = self._scene_call(self._scene_manager.detach_object, name, placement)
        if not detached or not self._exclusive_state(name, "WORLD"):
            if self._exclusive_state(name, "WORLD"):
                self._scene_call(self._scene_manager.attach_object, name)
            self._interface.stop()
            return SkillStatus.FAILED
        release_ok = physical.release(name)
        if not release_ok or physical.is_attached(name):
            if physical.is_attached(name):
                self._scene_call(self._scene_manager.attach_object, name)
            self._interface.stop()
            return SkillStatus.FAILED
        status = self._gripper_command(self._gripper.open)
        if status != SkillStatus.SUCCESS:
            return status
        status = self._motion(self._primitives.retreat(zone_name))
        if status != SkillStatus.SUCCESS:
            return status
        if not self._scene_call(self._scene_manager.clear_grasp_contact, name):
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
        except Exception:
            self._interface.stop()
            return SkillStatus.FAILED
        return SkillStatus.SUCCESS

    @staticmethod
    def _scene_call(operation, *args) -> bool:
        try:
            return bool(operation(*args))
        except Exception:
            return False
