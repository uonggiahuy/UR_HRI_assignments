"""Bounded, read-only ROS graph and simulator observations."""

import argparse
from collections import defaultdict, deque
from functools import partial
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

from turtlebot4.simulation_launch import validate_namespace


def bounded_timeout(value):
    duration = float(value)
    if not math.isfinite(duration) or not 1 <= duration <= 120:
        raise argparse.ArgumentTypeError('timeout must be finite and between 1 and 120 seconds')
    return duration


def seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


def scan_valid(message):
    metadata = [message.range_min, message.range_max, message.angle_min,
                message.angle_max, message.angle_increment]
    return (bool(message.header.frame_id) and all(map(math.isfinite, metadata))
            and 0 <= message.range_min < message.range_max
            and message.angle_increment > 0 and message.angle_max > message.angle_min
            and any(math.isfinite(x) and message.range_min <= x <= message.range_max
                    for x in message.ranges))


def odometry_values(message):
    p, q = message.pose.pose.position, message.pose.pose.orientation
    linear, angular = message.twist.twist.linear, message.twist.twist.angular
    return [p.x, p.y, p.z, q.x, q.y, q.z, q.w,
            linear.x, linear.y, linear.z, angular.x, angular.y, angular.z]


def odometry_valid(message):
    values = odometry_values(message)
    return (bool(message.header.frame_id and message.child_frame_id)
            and all(map(math.isfinite, values))
            and abs(sum(x * x for x in values[3:7]) - 1.0) < 0.01)


def discover_topic(topics, expected, type_name, namespace=''):
    if expected in topics:
        return expected if topics[expected] == [type_name] else None
    prefix = '/' + namespace.strip('/') + '/' if namespace else '/'
    candidates = [name for name, types in topics.items()
                  if types == [type_name] and name.startswith(prefix)
                  and '_internal' not in name.split('/')]
    return candidates[0] if len(candidates) == 1 else None


def frame_chain(parents, root, target):
    chain, seen = [target], set()
    while chain[-1] != root:
        child = chain[-1]
        if child in seen or child not in parents:
            return None
        seen.add(child)
        chain.append(parents[child])
    return list(reversed(chain))


def same_transform(first, second, tolerance=1e-6):
    translation_equal = all(abs(a - b) <= tolerance for a, b in zip(first[:3], second[:3]))
    # q and -q encode the same rotation, including wheel joints crossing a full turn.
    rotation_equal = (all(abs(a - b) <= tolerance for a, b in zip(first[3:], second[3:]))
                      or all(abs(a + b) <= tolerance for a, b in zip(first[3:], second[3:])))
    return translation_equal and rotation_equal


def endpoint_details(endpoint):
    qos = endpoint.qos_profile
    return {'node': endpoint.node_namespace.rstrip('/') + '/' + endpoint.node_name,
            'type': endpoint.topic_type, 'qos': {
                'reliability': qos.reliability.name, 'durability': qos.durability.name,
                'history': qos.history.name, 'depth': qos.depth}}


def observation_qos(key):
    # Reliable transient-local is required to replay retained static TF in CycloneDDS.
    return QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE if key == 'tf_static'
                      else ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL if key == 'tf_static'
                      else DurabilityPolicy.VOLATILE)


class Observations:
    def __init__(self):
        self.count = 0
        self.invalid = 0
        self.latest = None
        self.times = deque(maxlen=4096)
        self.stamps = deque(maxlen=4096)
        self.backwards = False

    def add(self, message, stamp, valid=True):
        self.count += 1
        self.invalid += not valid
        self.latest = message
        self.times.append(time.monotonic())
        if self.stamps and stamp < self.stamps[-1]:
            self.backwards = True
        self.stamps.append(stamp)

    def summary(self):
        result = {'received': self.count, 'invalid': self.invalid,
                  'stamp_regression': self.backwards}
        if len(self.times) > 1:
            elapsed = self.times[-1] - self.times[0]
            result['observed_wall_hz'] = (len(self.times) - 1) / elapsed if elapsed > 0 else None
            delta = self.stamps[-1] - self.stamps[0]
            result['sim_stamp_delta_sec'] = delta
            result['observed_sim_hz'] = (len(self.stamps) - 1) / delta if delta > 0 else None
        return result


class SimulationProbe(Node):
    def __init__(self, namespace):
        super().__init__('assignment4_simulation_probe')
        self.namespace = namespace
        self.topics = {}
        self.subscriptions_owned = []
        self.subscriptions_by_key = {}
        self.samples = {name: Observations() for name in ('clock', 'scan', 'odom')}
        self.parents = defaultdict(set)
        self.authorities = defaultdict(set)
        self.transform_samples = {}
        self.transform_conflicts = set()
        self.conflict_evidence = []
        self.duplicate_transforms = set()
        self.tf_counts = defaultdict(int)
        self.invalid_tf = 0

    def expected(self, name):
        return '/' + '/'.join(filter(None, (self.namespace, name)))

    def discover(self, allow_fallback=False):
        graph = dict(self.get_topic_names_and_types())
        specs = {'clock': ('/clock', 'rosgraph_msgs/msg/Clock', Clock),
                 'scan': (self.expected('scan'), 'sensor_msgs/msg/LaserScan', LaserScan),
                 'odom': (self.expected('odom'), 'nav_msgs/msg/Odometry', Odometry),
                 'tf': (self.expected('tf'), 'tf2_msgs/msg/TFMessage', TFMessage),
                 'tf_static': (self.expected('tf_static'), 'tf2_msgs/msg/TFMessage', TFMessage)}
        for key, (expected, type_name, message_type) in specs.items():
            # TF topics share a type; select their exact names to avoid ambiguity.
            topic = (expected if graph.get(expected) == [type_name] else None) if key.startswith('tf') else discover_topic(
                graph, expected, type_name, '' if key == 'clock' else self.namespace)
            if topic != expected and not allow_fallback:
                topic = None
            if not topic:
                continue
            if self.topics.get(key) == topic:
                continue
            if key in self.subscriptions_by_key:
                previous = self.subscriptions_by_key.pop(key)
                self.destroy_subscription(previous)
                self.subscriptions_owned.remove(previous)
                if key in self.samples:
                    self.samples[key] = Observations()
            self.topics[key] = topic
            qos = observation_qos(key)
            if key.startswith('tf'):
                callback = partial(self.record_tf, key)
            else:
                callback = partial(self.record_sample, key)
            subscription = self.create_subscription(message_type, topic, callback, qos)
            self.subscriptions_owned.append(subscription)
            self.subscriptions_by_key[key] = subscription

    def record_sample(self, key, message):
        stamp = seconds(message.clock if key == 'clock' else message.header.stamp)
        valid = scan_valid(message) if key == 'scan' else odometry_valid(message) if key == 'odom' else True
        self.samples[key].add(message, stamp, valid)

    def record_tf(self, key, message):
        self.tf_counts[key] += 1
        message_children = set()
        for transform in message.transforms:
            parent, child = transform.header.frame_id, transform.child_frame_id
            t, q = transform.transform.translation, transform.transform.rotation
            values = [t.x, t.y, t.z, q.x, q.y, q.z, q.w]
            if (not parent or not child or parent == child or not all(map(math.isfinite, values))
                    or abs(sum(x * x for x in values[3:]) - 1.0) > 0.01):
                self.invalid_tf += 1
                continue
            self.parents[child].add(parent)
            edge = (parent, child)
            if child in message_children:
                self.duplicate_transforms.add(edge)
            message_children.add(child)
            self.authorities[edge].add(key)
            stamp = seconds(transform.header.stamp)
            previous = self.transform_samples.get(edge)
            if previous and (key == 'tf_static' or stamp == previous[0]):
                if not same_transform(values, previous[1]):
                    self.transform_conflicts.add(edge)
                    if len(self.conflict_evidence) < 10:
                        self.conflict_evidence.append({'edge': list(edge), 'stamp': stamp,
                                                       'previous': previous[1], 'current': values})
            self.transform_samples[edge] = (stamp, values)

    def endpoints(self, topic, subscribers=False):
        getter = self.get_subscriptions_info_by_topic if subscribers else self.get_publishers_info_by_topic
        return [endpoint_details(x) for x in getter(topic)] if topic else []

    def report(self, expect_rviz):
        checks = {}

        def record(name, good, **evidence):
            checks[name] = {'status': 'PASS' if good else 'FAIL', **evidence}

        for key in ('clock', 'scan', 'odom'):
            sample = self.samples[key]
            progressing = len(sample.stamps) > 1 and sample.stamps[-1] > sample.stamps[0]
            record(key, sample.count > 1 and not sample.invalid and not sample.backwards and progressing,
                   topic=self.topics.get(key), publishers=self.endpoints(self.topics.get(key)),
                   **sample.summary())
        clock_stamps = self.samples['clock'].stamps
        ages = {key: clock_stamps[-1] - self.samples[key].stamps[-1]
                for key in ('scan', 'odom') if clock_stamps and self.samples[key].stamps}
        record('simulation_time_alignment', len(ages) == 2 and all(-0.2 <= age <= 2.0 for age in ages.values()),
               header_age_sim_seconds=ages, allowed_age_sim_seconds=[-0.2, 2.0])
        if self.samples['scan'].latest:
            scan = self.samples['scan'].latest
            checks['scan'].update(frame_id=scan.header.frame_id, ranges=len(scan.ranges),
                                 valid_ranges=sum(math.isfinite(x) and scan.range_min <= x <= scan.range_max
                                                  for x in scan.ranges))
        if self.samples['odom'].latest:
            odom = self.samples['odom'].latest
            checks['odom'].update(frame_id=odom.header.frame_id, child_frame_id=odom.child_frame_id)
            if odometry_valid(odom):
                checks['odom']['pose_and_twist'] = odometry_values(odom)

        graph = dict(self.get_topic_names_and_types())
        cmd = self.expected('cmd_vel')
        if cmd not in graph:
            candidates = [name for name in graph if name.endswith('/cmd_vel')
                          and (not self.namespace or name.startswith('/' + self.namespace + '/'))]
            cmd = candidates[0] if len(candidates) == 1 else None
        subscribers = self.endpoints(cmd, subscribers=True)
        # Installed Create 3 bringup identifies motion_control as the robot-side consumer.
        robot_side = [x for x in subscribers if x['node'].split('/')[-1] == 'motion_control']
        record('cmd_vel', bool(robot_side) and len(graph.get(cmd, [])) == 1,
               topic=cmd, types=graph.get(cmd, []), subscribers=subscribers,
               robot_side_subscribers=robot_side)

        conflicts = {child: sorted(parents) for child, parents in self.parents.items() if len(parents) != 1}
        duplicate_edges = [list(edge) for edge, sources in self.authorities.items() if len(sources) > 1]
        duplicate_edges += [list(edge) for edge in sorted(self.duplicate_transforms)]
        parents = {child: next(iter(values)) for child, values in self.parents.items() if len(values) == 1}
        odom, scan = self.samples['odom'].latest, self.samples['scan'].latest
        base_chain = frame_chain(parents, odom.header.frame_id, odom.child_frame_id) if odom else None
        lidar_chain = frame_chain(parents, odom.child_frame_id, scan.header.frame_id) if odom and scan else None
        cycles = [child for child in parents if self.has_cycle(parents, child)]
        record('tf', bool(base_chain and lidar_chain) and not conflicts and not duplicate_edges
               and not cycles and not self.invalid_tf and not self.transform_conflicts and self.tf_counts['tf'] > 0
               and self.tf_counts['tf_static'] > 0,
               base_chain=base_chain, lidar_chain=lidar_chain, conflicts=conflicts,
               duplicate_edges=duplicate_edges, cycles=cycles, invalid_transforms=self.invalid_tf,
               conflicting_samples=[list(edge) for edge in sorted(self.transform_conflicts)],
               conflict_evidence=self.conflict_evidence,
               authority_limit='Installed Humble rclpy does not expose publisher GIDs in callbacks; '
               'identical transforms from distinct publishers cannot be attributed per edge.',
               message_counts=dict(self.tf_counts), edges=[[p, c] for c, p in sorted(parents.items())],
               publishers={key: self.endpoints(self.topics.get(key)) for key in ('tf', 'tf_static')})
        nodes = [ns.rstrip('/') + '/' + name for name, ns in self.get_node_names_and_namespaces()]
        duplicate_nodes = sorted({name for name in nodes if nodes.count(name) > 1})
        record('node_uniqueness', not duplicate_nodes, duplicates=duplicate_nodes, nodes=sorted(nodes))
        if expect_rviz:
            record('rviz_node', self.expected('rviz2') in nodes, expected=self.expected('rviz2'))
            checks['rviz_visualization'] = {'status': 'MANUAL VERIFICATION REQUIRED',
                                          'reason': 'Node presence cannot verify graphical display content.'}
        return checks

    @staticmethod
    def has_cycle(parents, child):
        seen = set()
        while child in parents:
            if child in seen:
                return True
            seen.add(child)
            child = parents[child]
        return False


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument('--timeout', type=bounded_timeout, default=20.0,
                        help='Total wall-clock observation window, 1–120 seconds')
    result.add_argument('--namespace', default='')
    result.add_argument('--expect-rviz', action='store_true')
    result.add_argument('--output', type=Path, help='Optional JSON evidence file')
    return result


def main(argv=None):
    arguments = parser().parse_args(argv)
    try:
        namespace = validate_namespace(arguments.namespace)
    except ValueError as error:
        parser().error(str(error))
    rclpy.init(args=[])
    probe = SimulationProbe(namespace)
    try:
        start = time.monotonic()
        deadline = start + arguments.timeout
        next_discovery = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            if time.monotonic() >= next_discovery:
                probe.discover(allow_fallback=time.monotonic() - start >= min(2.0, arguments.timeout / 2))
                next_discovery = time.monotonic() + 0.2
            rclpy.spin_once(probe, timeout_sec=min(0.1, max(0, deadline - time.monotonic())))
        checks = probe.report(arguments.expect_rviz)
        report = {'namespace': namespace, 'observation_wall_seconds': arguments.timeout, 'checks': checks}
        for name, check in checks.items():
            print(f"{check['status']}: {name}: {json.dumps(check, allow_nan=False)}")
        if arguments.output:
            arguments.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        return int(any(check['status'] == 'FAIL' for check in checks.values()))
    finally:
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
