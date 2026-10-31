"""Persistent RViz outlines, UTF-8 names and distinct approach arrows."""

import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from turtlebot4.map_segmentation import COLORS, load_regions


def build_markers(config, selected="", stamp=None):
    import math

    markers = []
    extent = [p for r in config["regions"] for p in r["polygon_map"]]
    legend_x = max(p[0] for p in extent) + 9.0
    legend_top = max(p[1] for p in extent) - 2.0
    legend_step = (max(p[1] for p in extent) - min(p[1] for p in extent)) / 10.0
    for index, (region, color) in enumerate(zip(config["regions"], COLORS)):

        def marker(kind, offset):
            m = Marker()
            m.header.frame_id = "map"
            if stamp is not None:
                m.header.stamp = stamp
            m.ns = "assignment7"
            m.id = index * 4 + offset
            m.type = kind
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.color.r, m.color.g, m.color.b = [
                int(color[i : i + 2], 16) / 255 for i in (1, 3, 5)
            ]
            m.color.a = 1.0
            return m

        line = marker(Marker.LINE_STRIP, 0)
        line.scale.x = 0.12 if region["id"] == selected else 0.045
        line.points = [
            Point(x=float(x), y=float(y), z=0.05)
            for x, y in region["polygon_map"] + [region["polygon_map"][0]]
        ]
        markers.append(line)
        text = marker(Marker.TEXT_VIEW_FACING, 1)
        p = region["representative_point"]
        text.pose.position.x, text.pose.position.y, text.pose.position.z = (
            p["x"],
            p["y"],
            0.5,
        )
        # Retain UTF-8 labels for consumers; embedded legend supplies full glyphs.
        text.color.a = 0.0
        text.scale.z = 0.75
        text.pose.position.x = legend_x
        text.pose.position.y = legend_top - index * legend_step
        text.text = region["id"] + " — " + region["name"]
        markers.append(text)
        letter = marker(Marker.TEXT_VIEW_FACING, 3)
        letter.text = region["id"]
        letter.scale.z = 1.1
        letter.pose.position.x, letter.pose.position.y, letter.pose.position.z = (
            p["x"],
            p["y"],
            0.7,
        )
        markers.append(letter)
        if region["approach_goal"]:
            arrow = marker(Marker.ARROW, 2)
            p = region["approach_goal"]
            arrow.pose.position.x, arrow.pose.position.y, arrow.pose.position.z = (
                p["x"],
                p["y"],
                0.12,
            )
            arrow.pose.orientation.z, arrow.pose.orientation.w = math.sin(
                p["yaw"] / 2
            ), math.cos(p["yaw"] / 2)
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.55, 0.10, 0.10
            if region["id"] == selected:
                arrow.color.r, arrow.color.g, arrow.color.b = 1.0, 1.0, 0.0
            markers.append(arrow)
    legend = Marker()
    legend.header.frame_id = "map"
    legend.ns = "assignment7"
    legend.id = 1000
    legend.type = Marker.MESH_RESOURCE
    legend.action = Marker.ADD
    legend.mesh_resource = "package://turtlebot4/maps/semantic_legend.dae"
    legend.mesh_use_embedded_materials = True
    legend.pose.orientation.w = 1.0
    legend.pose.position.x = max(p[0] for p in extent) + 2.0
    legend.pose.position.y = min(p[1] for p in extent) + 3.0
    legend.pose.position.z = 0.25
    legend.scale.x = legend.scale.y = legend.scale.z = 1.0
    # Alpha/color zero allows embedded texture colors through unchanged in RViz.
    markers.append(legend)
    return MarkerArray(markers=markers)


class SemanticMapNode(Node):
    def __init__(self):
        super().__init__("semantic_map_node")
        share = Path(get_package_share_directory("turtlebot4"))
        config, _ = load_regions(
            self.declare_parameter(
                "regions_file", str(share / "maps/regions.yaml")
            ).value,
            self.declare_parameter("map_file", str(share / "maps/map.yaml")).value,
        )
        self.config, self.selected = config, ""
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(MarkerArray, "/assignment7/regions", qos)
        self.status = self.create_publisher(String, "/assignment7/regions_status", qos)
        self.create_subscription(String, "/assignment7/state", self.state, 10)
        self.publish()

    def publish(self):
        self.publisher.publish(
            build_markers(self.config, self.selected, self.get_clock().now().to_msg())
        )
        self.status.publish(
            String(
                data=json.dumps(
                    dict(
                        map_sha256=self.config["map_sha256"],
                        regions=[
                            dict(
                                id=r["id"],
                                name=r["name"],
                                validation=r["validation"],
                                reachability=r["reachability"],
                            )
                            for r in self.config["regions"]
                        ],
                    ),
                    ensure_ascii=False,
                )
            )
        )

    def state(self, msg):
        try:
            selected = json.loads(msg.data).get("current_destination", "") or ""
        except (ValueError, TypeError):
            return
        if selected != self.selected:
            self.selected = selected
            self.publish()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SemanticMapNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
