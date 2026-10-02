"""Publish the canonical task zones as RViz visuals, without collision geometry."""

from __future__ import annotations

from ament_index_python.packages import get_package_share_directory
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from ur3_perception_llm_control.planning_scene import _quaternion_from_rpy
from ur3_perception_llm_control.workcell_scene import EXPECTED_ZONES, iter_models, load_scene


TOPIC = "/workcell/zone_markers"


def zone_marker_array(scene: dict[str, object], stamp=None) -> MarkerArray:
    """Build exactly three scene-derived, visual-only RViz cube markers."""
    frame_id = str(scene["robot"]["world_frame"])
    zones = {model.name: model for model in iter_models(scene) if model.name in EXPECTED_ZONES}
    result = MarkerArray()
    for marker_id, name in enumerate(EXPECTED_ZONES):
        model = zones[name]
        marker = Marker()
        marker.header.frame_id = frame_id
        if stamp is not None:
            marker.header.stamp = stamp
        marker.ns = "workcell_zones"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = model.pose[:3]
        (marker.pose.orientation.x, marker.pose.orientation.y,
         marker.pose.orientation.z, marker.pose.orientation.w) = _quaternion_from_rpy(*model.pose[3:])
        marker.scale.x, marker.scale.y, marker.scale.z = model.size
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = model.color
        result.markers.append(marker)
    return result


class ZoneMarkers(Node):
    def __init__(self) -> None:
        super().__init__("workcell_zone_markers")
        default_scene = Path(get_package_share_directory("ur3_perception_llm_control")) / "config" / "scene.yaml"
        self.declare_parameter("scene_config", str(default_scene))
        scene = load_scene(str(self.get_parameter("scene_config").value))
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._publisher = self.create_publisher(MarkerArray, TOPIC, qos)
        self._scene = scene
        self._timer = self.create_timer(1.0, self.publish_markers)
        self.publish_markers()

    def publish_markers(self) -> None:
        self._publisher.publish(zone_marker_array(self._scene, self.get_clock().now().to_msg()))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ZoneMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
