"""Plan-only MoveIt feasibility for a hypothetical held cube."""

from __future__ import annotations

from math import isclose
from typing import Mapping

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene

from ur3_perception_llm_control.motion_primitives import ManipulationMotionPrimitives
from ur3_perception_llm_control.moveit_interface import MotionResult, MoveItArmInterface
from ur3_perception_llm_control.perception_scene import SceneSyncReport
from ur3_perception_llm_control.perception_state import PerceptionSnapshot
from ur3_perception_llm_control.planning_scene import collision_object
from ur3_perception_llm_control.temporary_position import TemporaryPlacement
from ur3_perception_llm_control.workcell_scene import iter_models
from ur3_perception_llm_control.world_state import BLOCKS


class MoveItTemporaryFeasibility:
    """Check approach and placement plans without executing or mutating WORLD.

    Each plan-only request locally removes the relocated cube from WORLD and
    adds one equal-size box at gripper_tcp. This models future held transport
    without creating a persistent WORLD+ATTACHED duplicate.
    """

    def __init__(self, interface: MoveItArmInterface,
                 primitives: ManipulationMotionPrimitives,
                 scene: Mapping[str, object], motion: Mapping[str, object],
                 snapshot: PerceptionSnapshot, report: SceneSyncReport,
                 object_name: str) -> None:
        if object_name not in BLOCKS or report.attached_ids:
            raise ValueError("feasibility requires a known, WORLD-only cube")
        if set(report.authoritative_xyz) != set(BLOCKS):
            raise ValueError("authoritative scene is incomplete")
        for name in BLOCKS:
            observed = snapshot.object_world_xy[name]
            requested = report.requested_xyz[name]
            authoritative = report.authoritative_xyz[name]
            if (not all(isclose(observed[i], requested[i], abs_tol=1e-9) for i in (0, 1))
                    or not all(isclose(requested[i], authoritative[i], abs_tol=0.001)
                               for i in range(3))):
                raise ValueError("Planning Scene and snapshot disagree")
        self.interface = interface
        self.primitives = primitives
        self.home = {name: float(value) for name, value in motion["home"].items()}
        self.frame_id = str(scene["robot"]["world_frame"])
        settings = motion["manipulation"]
        attachment_link = str(settings["attachment_link"])
        models = {model.name: model for model in iter_models(scene)}
        local = PlanningScene()
        local.is_diff = True
        local.robot_state.is_diff = True
        removal = CollisionObject()
        removal.header.frame_id = self.frame_id
        removal.id = object_name
        removal.operation = CollisionObject.REMOVE
        local.world.collision_objects = [removal]
        attached = AttachedCollisionObject()
        attached.link_name = attachment_link
        attached.touch_links = [str(link) for link in settings["touch_links"]]
        attached.object = collision_object(models[object_name], attachment_link)
        attached.object.pose.position.x = 0.0
        attached.object.pose.position.y = 0.0
        attached.object.pose.position.z = 0.0
        attached.object.pose.orientation.x = 0.0
        attached.object.pose.orientation.y = 0.0
        attached.object.pose.orientation.z = 0.0
        attached.object.pose.orientation.w = 1.0
        local.robot_state.attached_collision_objects = [attached]
        self.scene_diff = local
        self.last_result: MotionResult | None = None

    def __call__(self, candidate: TemporaryPlacement) -> bool:
        center = PoseStamped()
        center.header.frame_id = self.frame_id
        center.pose.position.x = candidate.x
        center.pose.position.y = candidate.y
        center.pose.position.z = candidate.z
        center.pose.orientation.w = 1.0
        poses = self.primitives.placement_planning_poses(center)
        self.last_result = self.interface.plan_pose_sequence(
            poses, start_joint_positions=self.home, scene_diff=self.scene_diff,
        )
        return self.last_result == MotionResult.SUCCESS
