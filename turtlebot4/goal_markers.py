"""Optional RViz goal labels from the same map-bound mission configuration."""
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray

from turtlebot4.waypoint_navigator import load_goals


def make_markers(config, stamp):
    """Show target centers and labels; these are targets, not a planned path."""
    result = MarkerArray()
    for index, goal in enumerate(config['goals']):
        for kind in (Marker.SPHERE, Marker.TEXT_VIEW_FACING):
            marker = Marker()
            marker.header.frame_id = config['frame_id']
            marker.header.stamp = stamp
            marker.ns = 'assignment5_goals'
            marker.id = index * 2 + (kind == Marker.TEXT_VIEW_FACING)
            marker.type = kind
            marker.action = Marker.ADD
            marker.pose.position = Point(x=goal['x'],
                                         y=goal['y'] + (.30 if kind == Marker.TEXT_VIEW_FACING else 0.),
                                         z=.15 if kind == Marker.SPHERE else .6)
            marker.pose.orientation.w = 1.
            marker.scale.x = marker.scale.y = .20
            marker.scale.z = .20 if kind == Marker.SPHERE else .28
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
                (.1, .2, .85, 1.) if kind == Marker.TEXT_VIEW_FACING else (1., .65, .1, 1.))
            marker.text = f"Goal {index + 1}"
            result.markers.append(marker)
    return result


class GoalMarkers(Node):
    def __init__(self):
        super().__init__('goal_markers')
        share = Path(get_package_share_directory('turtlebot4'))
        filename = self.declare_parameter('goals_file', str(share / 'config/goals.yaml')).value
        self.config = load_goals(Path(filename), share / 'maps/map.yaml')
        self.publisher = self.create_publisher(
            MarkerArray, '/assignment5/goal_markers',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(1., self.publish)
        self.publish()

    def publish(self):
        self.publisher.publish(make_markers(self.config, self.get_clock().now().to_msg()))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GoalMarkers()
        rclpy.spin(node)
        return 0
    except (KeyboardInterrupt, ExternalShutdownException):
        return 0
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
