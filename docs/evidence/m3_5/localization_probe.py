"""Bounded, stationary warehouse-spawn verification; no velocity commands.

Only initializes AMCL after actual ground truth confirms the documented spawn.
Run with ROS and the assignment overlay sourced, base and localization active.
"""
from collections import Counter
import json
import math
from pathlib import Path
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

from turtlebot4.map_saver import map_qos, validate_saved_map_files
from turtlebot4.localization_diagnostics import validate_map_compatibility, validate_amcl_pose_msg


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    rclpy.init()
    node = Node('m35_independent_localization_probe')
    data = {}
    samples = {'clock': [], 'scan': [], 'odom': [], 'map_to_odom': []}
    def record(key, msg):
        data[key] = msg
        if key in ('scan', 'odom'):
            samples[key].append(msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)
        if key == 'clock':
            samples[key].append(msg.clock.sec + msg.clock.nanosec / 1e9)
    def tf_callback(msg):
        for transform in msg.transforms:
            if transform.header.frame_id == 'map' and transform.child_frame_id == 'odom':
                samples['map_to_odom'].append(transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9)
    for key, cls, topic, qos in [
        ('map', OccupancyGrid, '/map', map_qos()),
        ('pose', PoseWithCovarianceStamped, '/amcl_pose', map_qos()),
        ('truth', Odometry, '/sim_ground_truth_pose', qos_profile_sensor_data),
        ('odom', Odometry, '/odom', qos_profile_sensor_data),
        ('scan', LaserScan, '/scan', qos_profile_sensor_data),
        ('clock', Clock, '/clock', 10),
    ]:
        node.create_subscription(cls, topic, lambda msg, key=key: record(key, msg), qos)
    node.create_subscription(TFMessage, '/tf', tf_callback, 100)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    try:
        deadline = time.monotonic() + 15
        while not all(key in data for key in ('truth', 'map', 'scan', 'odom', 'clock')):
            assert time.monotonic() < deadline, list(data)
            rclpy.spin_once(node, timeout_sec=0.1)
        truth = data['truth'].pose.pose
        assert math.hypot(truth.position.x, truth.position.y) < 0.03
        assert abs(yaw(truth.orientation)) < 0.03
        initial = PoseWithCovarianceStamped()
        initial.header.frame_id = 'map'
        initial.header.stamp = data['clock'].clock
        initial.pose.pose.orientation.w = 1.0
        initial.pose.covariance[0] = initial.pose.covariance[7] = 0.25
        initial.pose.covariance[35] = 0.068
        publisher = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        deadline = time.monotonic() + 5
        while publisher.get_subscription_count() == 0:
            assert time.monotonic() < deadline
            rclpy.spin_once(node, timeout_sec=0.1)
        publisher.publish(initial)
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        states = {}
        for name in ('map_server', 'amcl'):
            client = node.create_client(GetState, '/' + name + '/get_state')
            assert client.wait_for_service(timeout_sec=5)
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            assert future.done() and future.result().current_state.id == 3
            states[name] = future.result().current_state.label
        nodes = sorted(node.get_node_names())
        assert 'slam_toolbox' not in nodes
        meta = validate_saved_map_files(Path('/root/ros2_ws/src/turtlebot4/maps/map.yaml'),
                                        Path('/root/ros2_ws/src/turtlebot4/maps/map.pgm'))
        counts = validate_map_compatibility(data['map'], meta)
        assert Counter(data['map'].data) == Counter({0: 80090, 100: 1938, -1: 137449})
        pose = validate_amcl_pose_msg(data['pose'])
        truth = data['truth'].pose.pose
        estimated = data['pose'].pose.pose
        distance_error = math.hypot(estimated.position.x - truth.position.x,
                                    estimated.position.y - truth.position.y)
        yaw_error = abs(math.atan2(math.sin(yaw(estimated.orientation) - yaw(truth.orientation)),
                                  math.cos(yaw(estimated.orientation) - yaw(truth.orientation))))
        assert distance_error < 0.3 and yaw_error < 0.3
        transforms = {}
        for parent, child in [('map', 'odom'), ('odom', 'base_link'), ('map', 'base_link')]:
            tf = buffer.lookup_transform(parent, child, rclpy.time.Time())
            transforms[parent + ' -> ' + child] = {
                'stamp': tf.header.stamp.sec + tf.header.stamp.nanosec / 1e9,
                'translation': [tf.transform.translation.x, tf.transform.translation.y,
                                tf.transform.translation.z],
            }
        timing = {}
        for key, values in samples.items():
            assert len(values) > 5 and max(values) > min(values), (key, values)
            timing[key] = {'samples': len(values), 'first': values[0], 'last': values[-1]}
        assert abs(transforms['map -> base_link']['stamp'] - samples['clock'][-1]) < 2
        result = {
            'status': 'PASS', 'wall_timestamp': time.time(), 'lifecycle': states,
            'map': counts, 'pose': pose, 'ground_truth': {
                'x': truth.position.x, 'y': truth.position.y, 'yaw': yaw(truth.orientation)},
            'position_error_m': distance_error, 'yaw_error_rad': yaw_error,
            'timing': timing, 'transforms': transforms, 'nodes': nodes,
            'tf_publishers': sorted({info.node_name for info in node.get_publishers_info_by_topic('/tf')}),
            'scope': 'fresh warehouse simulation; stationary AMCL initialization; no motion',
        }
        print(json.dumps(result, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
