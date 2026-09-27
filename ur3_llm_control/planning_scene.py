"""Manage the assignment workcell in MoveIt's Planning Scene."""

from __future__ import annotations

import math
import os
from typing import Iterable, Sequence

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    CollisionObject,
    ObjectColor,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
import rclpy
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

from ur3_llm_control.workcell_scene import BoxModel, iter_models, load_scene


SERVICE_TIMEOUT = 30.0


def _quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, ...]:
    """Return an XYZW quaternion for fixed-axis roll, pitch, and yaw."""
    cr = math.cos(roll / 2.0)
    sr = math.sin(roll / 2.0)
    cp = math.cos(pitch / 2.0)
    sp = math.sin(pitch / 2.0)
    cy = math.cos(yaw / 2.0)
    sy = math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def collision_models(scene: dict[str, object]) -> tuple[BoxModel, ...]:
    """Return only physical obstacles; semantic zones are intentionally absent."""
    return tuple(model for model in iter_models(scene) if model.collision)


def collision_object(model: BoxModel, frame_id: str) -> CollisionObject:
    """Convert one canonical box model into a MoveIt collision object."""
    result = CollisionObject()
    result.header.frame_id = frame_id
    result.id = model.name

    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = list(model.size)

    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = model.pose[:3]
    quaternion = _quaternion_from_rpy(*model.pose[3:])
    (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ) = quaternion

    # MoveIt canonicalizes the object's global transform into ``pose`` and
    # keeps geometry-local primitive poses relative to it.
    result.pose = pose
    local_pose = Pose()
    local_pose.orientation.w = 1.0
    result.primitives = [primitive]
    result.primitive_poses = [local_pose]
    result.operation = CollisionObject.ADD
    return result


def object_color(model: BoxModel) -> ObjectColor:
    """Convert the canonical RGBA value for RViz Planning Scene display."""
    result = ObjectColor()
    result.id = model.name
    result.color.r, result.color.g, result.color.b, result.color.a = model.color
    return result


def planning_scene_diff(scene: dict[str, object]) -> PlanningScene:
    """Build a Planning Scene diff directly from the canonical scene mapping."""
    robot = scene["robot"]
    if not isinstance(robot, dict):
        raise ValueError("scene.robot must be a mapping")
    frame_id = str(robot["world_frame"])
    models = collision_models(scene)

    result = PlanningScene()
    result.is_diff = True
    result.robot_state.is_diff = True
    result.world.collision_objects = [
        collision_object(model, frame_id) for model in models
    ]
    result.object_colors = [object_color(model) for model in models]
    return result


def _close_sequence(actual: Sequence[float], expected: Sequence[float]) -> bool:
    return len(actual) == len(expected) and all(
        math.isclose(left, right, rel_tol=0.0, abs_tol=1e-7)
        for left, right in zip(actual, expected)
    )


def scene_matches(actual: PlanningScene, expected: PlanningScene) -> tuple[bool, str]:
    """Compare returned world box names, frames, dimensions, and poses."""
    actual_by_id = {item.id: item for item in actual.world.collision_objects}
    expected_by_id = {item.id: item for item in expected.world.collision_objects}
    if set(actual_by_id) != set(expected_by_id):
        return False, (
            f"object IDs differ: actual={sorted(actual_by_id)}, "
            f"expected={sorted(expected_by_id)}"
        )

    for object_id, wanted in expected_by_id.items():
        found = actual_by_id[object_id]
        if found.header.frame_id != wanted.header.frame_id:
            return False, f"{object_id}: frame differs"
        if len(found.primitives) != 1 or len(found.primitive_poses) != 1:
            return False, f"{object_id}: expected exactly one primitive and pose"
        if found.primitives[0].type != SolidPrimitive.BOX:
            return False, f"{object_id}: primitive is not a box"
        if not _close_sequence(
            found.primitives[0].dimensions, wanted.primitives[0].dimensions
        ):
            return False, f"{object_id}: dimensions differ"
        found_pose = found.pose
        wanted_pose = wanted.pose
        found_values = (
            found_pose.position.x,
            found_pose.position.y,
            found_pose.position.z,
            found_pose.orientation.x,
            found_pose.orientation.y,
            found_pose.orientation.z,
            found_pose.orientation.w,
        )
        wanted_values = (
            wanted_pose.position.x,
            wanted_pose.position.y,
            wanted_pose.position.z,
            wanted_pose.orientation.x,
            wanted_pose.orientation.y,
            wanted_pose.orientation.z,
            wanted_pose.orientation.w,
        )
        if not _close_sequence(found_values, wanted_values):
            return False, f"{object_id}: pose differs"
        found_local_pose = found.primitive_poses[0]
        wanted_local_pose = wanted.primitive_poses[0]
        if not _close_sequence(
            (
                found_local_pose.position.x,
                found_local_pose.position.y,
                found_local_pose.position.z,
                found_local_pose.orientation.x,
                found_local_pose.orientation.y,
                found_local_pose.orientation.z,
                found_local_pose.orientation.w,
            ),
            (
                wanted_local_pose.position.x,
                wanted_local_pose.position.y,
                wanted_local_pose.position.z,
                wanted_local_pose.orientation.x,
                wanted_local_pose.orientation.y,
                wanted_local_pose.orientation.z,
                wanted_local_pose.orientation.w,
            ),
        ):
            return False, f"{object_id}: primitive-local pose differs"
    return True, "names, primitive dimensions, and poses match"


class PlanningSceneManager:
    """Small service-based interface to MoveIt's authoritative scene monitor."""

    def __init__(self, node: Node, scene: dict[str, object]) -> None:
        self._node = node
        self._expected = planning_scene_diff(scene)
        self._apply_client = node.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )
        self._get_client = node.create_client(GetPlanningScene, "/get_planning_scene")

    @property
    def object_ids(self) -> tuple[str, ...]:
        return tuple(item.id for item in self._expected.world.collision_objects)

    def apply(self, timeout: float = SERVICE_TIMEOUT) -> bool:
        if not self._apply_client.wait_for_service(timeout_sec=timeout):
            self._node.get_logger().error("/apply_planning_scene is unavailable")
            return False
        request = ApplyPlanningScene.Request()
        request.scene = self._expected
        response = self._call(self._apply_client, request, timeout)
        return response is not None and response.success

    def get(self, timeout: float = SERVICE_TIMEOUT) -> PlanningScene | None:
        if not self._get_client.wait_for_service(timeout_sec=timeout):
            self._node.get_logger().error("/get_planning_scene is unavailable")
            return None
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.OBJECT_COLORS
        )
        response = self._call(self._get_client, request, timeout)
        return None if response is None else response.scene

    def verify(self, timeout: float = SERVICE_TIMEOUT) -> tuple[bool, str]:
        actual = self.get(timeout)
        if actual is None:
            return False, "no response from /get_planning_scene"
        return scene_matches(actual, self._expected)

    def destroy(self) -> None:
        self._node.destroy_client(self._apply_client)
        self._node.destroy_client(self._get_client)

    def _call(self, client, request, timeout: float):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout)
        if not future.done() or future.cancelled():
            self._node.get_logger().error("Planning Scene service call timed out")
            return None
        try:
            return future.result()
        except Exception as exc:  # rclpy service exceptions are runtime-specific
            self._node.get_logger().error(f"Planning Scene service failed: {exc}")
            return None


def _default_scene_path() -> str:
    return os.path.join(
        get_package_share_directory("ur3_llm_control"), "config", "scene.yaml"
    )


def main(args: Iterable[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Node("workcell_planning_scene")
    node.declare_parameter("scene_config", _default_scene_path())
    manager = None
    try:
        scene = load_scene(str(node.get_parameter("scene_config").value))
        manager = PlanningSceneManager(node, scene)
        if not manager.apply():
            raise RuntimeError("MoveIt rejected the Planning Scene diff")
        verified, detail = manager.verify()
        if not verified:
            raise RuntimeError(detail)
        node.get_logger().info(
            f"Planning Scene active ({', '.join(manager.object_ids)}): {detail}"
        )
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        node.get_logger().fatal(f"Planning Scene setup failed: {exc}")
        raise
    finally:
        if manager is not None:
            manager.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
