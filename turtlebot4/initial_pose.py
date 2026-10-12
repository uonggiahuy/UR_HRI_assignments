"""Initialize real AMCL in map coordinates and verify scan-driven output."""
import argparse
import math
import sys
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import SetInitialPose
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_ros import Buffer, TransformListener, TransformException
import yaml

from turtlebot4.diagnostics import seconds, scan_valid, odometry_valid
from turtlebot4.localization_diagnostics import (
    LocalizationDiagnosticsNode, validate_amcl_pose_msg, validate_map_compatibility,
)
from turtlebot4.localization_runner import (
    bounded_timeout, check_no_active_slam, resolve_saved_map_path,
)
from turtlebot4.map_saver import validate_saved_map_files
from turtlebot4.simulation_launch import validate_namespace


def finite(value):
    result = float(value)
    if not math.isfinite(result):
        raise argparse.ArgumentTypeError('coordinate/angle must be finite')
    return result


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument('--x', required=True, type=finite, help='Current map-frame x (m)')
    result.add_argument('--y', required=True, type=finite, help='Current map-frame y (m)')
    result.add_argument('--yaw-deg', required=True, type=finite, help='Current map-frame yaw (degrees)')
    result.add_argument('--namespace', default='', type=validate_namespace)
    result.add_argument('--map', default=None, help='Matching saved map YAML')
    result.add_argument('--timeout', type=bounded_timeout, default=20.0)
    result.add_argument('--interface', choices=('auto', 'service', 'topic'), default='auto',
                        help='auto prefers AMCL SetInitialPose; falls back only if service absent')
    return result


def make_pose(x, y, yaw_deg, stamp):
    if not all(math.isfinite(v) for v in (x, y, yaw_deg)):
        raise ValueError('Pose coordinates must be finite')
    if seconds(stamp) <= 0:
        raise ValueError('A nonzero simulation timestamp is required')
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = stamp
    msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
    angle = math.radians(math.remainder(yaw_deg, 360.0))
    msg.pose.pose.orientation.z = math.sin(angle / 2)
    msg.pose.pose.orientation.w = math.cos(angle / 2)
    covariance = [0.0] * 36
    covariance[0] = covariance[7] = 0.25
    covariance[35] = math.radians(15.0) ** 2
    msg.pose.covariance = covariance
    return msg


def wait_until(node, predicate, deadline, description):
    while time.monotonic() < deadline:
        if predicate():
            return
        rclpy.spin_once(node, timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())))
    raise TimeoutError(f'Timed out waiting for {description}')


def call(node, service_type, name, request, deadline):
    client = node.create_client(service_type, name)
    future = None
    try:
        wait_until(node, client.service_is_ready, deadline, name)
        future = client.call_async(request)
        wait_until(node, future.done, deadline, name + ' response')
        if future.exception():
            raise RuntimeError(f'{name}: {future.exception()}')
        return future.result()
    finally:
        if future is not None and not future.done():
            client.remove_pending_request(future)
            future.cancel()
        node.destroy_client(client)


def lifecycle_active(node, name, deadline):
    response = call(node, GetState, name + '/get_state', GetState.Request(), deadline)
    if response.current_state.id != 3:
        raise RuntimeError(f'{name} lifecycle is {response.current_state.label}, expected ACTIVE')
    return response.current_state.label


class InitialPoseNode(LocalizationDiagnosticsNode):
    def __init__(self, namespace=''):
        super().__init__(namespace)
        # Initialization must observe a new publication, never a retained old pose.
        self.destroy_subscription(self.sub_amcl)
        prefix = '/' + self.namespace if self.namespace else ''
        self.sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped, prefix + '/amcl_pose', self._amcl_cb,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE))
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.initial_clock = None
        self.clock_time = 0.0
        self.tf_stamps = {}
        self.map_tf_updates = 0

    def _clock_cb(self, msg):
        super()._clock_cb(msg)
        self.clock_time = seconds(msg.clock)
        if self.initial_clock is None:
            self.initial_clock = self.clock_time

    def _tf_cb(self, msg):
        super()._tf_cb(msg)
        for tf in msg.transforms:
            edge = (tf.header.frame_id, tf.child_frame_id)
            self.tf_stamps[edge] = seconds(tf.header.stamp)
            if edge == ('map', 'odom'):
                self.map_tf_updates += 1

    def sensors_ready(self):
        return (self.initial_clock is not None and self.clock_time > self.initial_clock
                and self.scan_msg is not None and scan_valid(self.scan_msg)
                and self.odom_msg is not None and odometry_valid(self.odom_msg)
                and abs(self.clock_time - seconds(self.scan_msg.header.stamp)) < 1.5
                and abs(self.clock_time - seconds(self.odom_msg.header.stamp)) < 1.5)

    def output_ready(self, sent, previous_count, previous_tf_count):
        if self.amcl_pose_count <= previous_count or self.amcl_pose_msg is None:
            return False
        # AMCL stamps its new estimate with the scan acquisition time. A scan
        # already in flight can precede receipt of the initialization request.
        if (seconds(self.amcl_pose_msg.header.stamp) < sent - 0.25
                or abs(self.clock_time - seconds(self.amcl_pose_msg.header.stamp)) > 1.5):
            return False  # Reject retained output from an earlier initialization.
        validate_amcl_pose_msg(self.amcl_pose_msg)
        if not self.sensors_ready() or self.map_tf_updates < previous_tf_count + 2:
            return False
        for edge in (('map', 'odom'), ('odom', 'base_link')):
            stamp = self.tf_stamps.get(edge, 0.0)
            if stamp < sent or not -1.5 <= self.clock_time - stamp <= 1.5:
                return False
        try:
            tf = self.buffer.lookup_transform('map', 'base_link', Time())
            return abs(self.clock_time - seconds(tf.header.stamp)) < 1.5
        except TransformException:
            return False


def choose_interface(node, prefix, requested):
    """Prefer the discovered AMCL service; topic requires its actual subscriber."""
    service_name = prefix + '/set_initial_pose'
    available = any(name == service_name and 'nav2_msgs/srv/SetInitialPose' in types
                    for name, types in node.get_service_names_and_types())
    if requested == 'service' or (requested == 'auto' and available):
        return 'service'
    topic = prefix + '/initialpose'
    endpoints = node.get_subscriptions_info_by_topic(topic)
    if not any(e.node_name == 'amcl' and e.node_namespace.rstrip('/') == prefix
               and e.topic_type == 'geometry_msgs/msg/PoseWithCovarianceStamped' for e in endpoints):
        raise RuntimeError(f'No verified AMCL subscriber on {topic}')
    return 'topic'


def initialize(node, args):
    path = resolve_saved_map_path(args.map)
    image = yaml.safe_load(path.read_text())['image']
    expected = validate_saved_map_files(path, path.parent / image)
    prefix = '/' + args.namespace if args.namespace else ''
    deadline = time.monotonic() + args.timeout
    wait_until(node, lambda: node.sensors_ready() and node.map_msg is not None,
               deadline, 'progressing simulation clock, fresh scan/odom and Map Server map')
    check_no_active_slam(node.get_node_names())
    for name in ('amcl', 'map_server'):
        if sum(n == name and ns.rstrip('/') == prefix
               for n, ns in node.get_node_names_and_namespaces()) != 1:
            raise RuntimeError(f'Require exactly one {prefix}/{name}')
        lifecycle_active(node, prefix + '/' + name, deadline)
    validate_map_compatibility(node.map_msg, expected)
    request = GetParameters.Request()
    request.names = ['use_sim_time', 'global_frame_id', 'odom_frame_id', 'base_frame_id']
    values = call(node, GetParameters, prefix + '/amcl/get_parameters', request, deadline).values
    if not (values[0].bool_value and [v.string_value for v in values[1:]] == ['map', 'odom', 'base_link']):
        raise RuntimeError('AMCL must use simulation time and map/odom/base_link frames')
    service_name = prefix + '/set_initial_pose'
    use_service = choose_interface(node, prefix, args.interface) == 'service'
    pose = make_pose(args.x, args.y, args.yaw_deg, node.get_clock().now().to_msg())
    previous_count, previous_tf_count = node.amcl_pose_count, node.map_tf_updates
    if use_service:
        request = SetInitialPose.Request()
        request.pose = pose
        call(node, SetInitialPose, service_name, request, deadline)
        interface = service_name
    else:
        topic = prefix + '/initialpose'
        publisher = node.create_publisher(PoseWithCovarianceStamped, topic, 1)
        try:
            wait_until(node, lambda: publisher.get_subscription_count() > 0,
                       deadline, 'AMCL initialpose topic match')
            pose.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(pose)
            wait_until(node, lambda: node.output_ready(seconds(pose.header.stamp), previous_count,
                                                       previous_tf_count), deadline,
                       'fresh AMCL pose and scan-driven map -> odom -> base_link')
        finally:
            node.destroy_publisher(publisher)
        interface = topic
    wait_until(node, lambda: node.output_ready(seconds(pose.header.stamp), previous_count,
                                               previous_tf_count), deadline,
               'fresh AMCL pose and scan-driven map -> odom -> base_link (inspect AMCL logs)')
    check_no_active_slam(node.get_node_names())
    lifecycle_active(node, prefix + '/amcl', deadline)
    return interface, validate_amcl_pose_msg(node.amcl_pose_msg)


def main(argv=None):
    args = parser().parse_args(argv)
    rclpy.init()
    node = InitialPoseNode(args.namespace)
    try:
        interface, pose = initialize(node, args)
        print(f'PASS: initialized via {interface}; fresh AMCL pose {pose["position"]}, '
              f'yaw {pose["yaw_deg"]} deg; fresh map -> odom -> base_link. '
              'Verify scan/map alignment before navigation.')
        return 0
    except (RuntimeError, ValueError, OSError, TimeoutError) as exc:
        print(f'FAIL: {exc}', file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
