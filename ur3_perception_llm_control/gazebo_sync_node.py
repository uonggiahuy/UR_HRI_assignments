"""Keep Gazebo visuals synchronized with authoritative MoveIt attachments."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
import rclpy
from rclpy.node import Node

from ur3_perception_llm_control.gazebo_sync import GazeboAttachmentSynchronizer
from ur3_perception_llm_control.workcell_scene import iter_models, load_scene


class GazeboAttachmentSyncNode(Node):
    """Observe one MoveIt attachment and synchronize just its Gazebo model."""

    def __init__(self) -> None:
        super().__init__("gazebo_attachment_sync")
        self.declare_parameter("object_name", "red_cube")
        self.declare_parameter("world_frame", "world")
        self._object_name = str(self.get_parameter("object_name").value)
        scene_path = os.path.join(
            get_package_share_directory("ur3_perception_llm_control"), "config", "scene.yaml"
        )
        scene = load_scene(scene_path)
        model = next(
            candidate for candidate in iter_models(scene)
            if candidate.name == self._object_name
        )
        self._sync = GazeboAttachmentSynchronizer(
            self,
            world_frame=str(self.get_parameter("world_frame").value),
            model=model,
        )
        self._client = self.create_client(GetPlanningScene, "/get_planning_scene")
        self._future = None
        self._active_link: str | None = None
        self._timer = self.create_timer(0.1, self._poll)

    def destroy_node(self) -> bool:
        self._timer.cancel()
        self._sync.destroy()
        self.destroy_client(self._client)
        return super().destroy_node()

    def _poll(self) -> None:
        if self._future is not None:
            if not self._future.done():
                return
            try:
                response = self._future.result()
            except Exception as exc:
                self.get_logger().error(f"Planning-scene observation failed: {exc}")
                response = None
            self._future = None
            self._consume(response)
            return
        if not self._client.service_is_ready():
            return
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        self._future = self._client.call_async(request)

    def _consume(self, response) -> None:
        if response is None:
            return
        for item in response.scene.robot_state.attached_collision_objects:
            if item.object.id == self._object_name:
                if self._active_link != item.link_name:
                    if self._sync.attach(item.object.id, item.link_name, item.object.pose):
                        self._active_link = item.link_name
                        self.get_logger().info(
                            f"Following {item.object.id} on {item.link_name} from MoveIt attachment"
                        )
                return
        if self._active_link is not None:
            self._sync.stop()
            self._active_link = None
            self.get_logger().info(f"Stopped following released {self._object_name}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboAttachmentSyncNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
