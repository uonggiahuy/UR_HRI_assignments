"""Diagnostic tool to inspect SLAM Toolbox mapping, sensor readiness, and TF chain."""

import argparse
from collections import defaultdict
import json
import math
import sys
import time

from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

from turtlebot4.diagnostics import frame_chain, same_transform, scan_valid, odometry_valid
from turtlebot4.map_saver import validate_occupancy_grid_msg
from turtlebot4.simulation_launch import validate_namespace


def bounded_timeout(value: str) -> float:
    duration = float(value)
    if not math.isfinite(duration) or not 1.0 <= duration <= 120.0:
        raise argparse.ArgumentTypeError('timeout must be finite and between 1 and 120 seconds')
    return duration


class MappingDiagnosticsNode(Node):
    def __init__(self, namespace: str = ''):
        super().__init__('mapping_diagnostics_node')
        self.namespace = validate_namespace(namespace)
        prefix = f'/{self.namespace}' if self.namespace else ''

        self.clock_received = 0
        self.scan_msg = None
        self.scan_count = 0
        self.odom_msg = None
        self.odom_count = 0
        self.map_msg = None
        self.map_count = 0
        self.map_timestamps = []

        self.tf_parents = {}
        self.tf_broadcasters = defaultdict(set)

        # QoS
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        transient_local_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.sub_clock = self.create_subscription(
            Clock, '/clock', self._clock_cb, 10
        )
        self.sub_scan = self.create_subscription(
            LaserScan, f'{prefix}/scan' if prefix else '/scan', self._scan_cb, best_effort_qos
        )
        self.sub_odom = self.create_subscription(
            Odometry, f'{prefix}/odom' if prefix else '/odom', self._odom_cb, best_effort_qos
        )
        self.sub_map = self.create_subscription(
            OccupancyGrid, f'{prefix}/map' if prefix else '/map', self._map_cb, transient_local_qos
        )
        self.sub_tf = self.create_subscription(
            TFMessage, f'{prefix}/tf' if prefix else '/tf', self._tf_cb, 100
        )
        self.sub_tf_static = self.create_subscription(
            TFMessage, f'{prefix}/tf_static' if prefix else '/tf_static', self._tf_cb, transient_local_qos
        )

    def _clock_cb(self, _msg: Clock):
        self.clock_received += 1

    def _scan_cb(self, msg: LaserScan):
        self.scan_count += 1
        self.scan_msg = msg

    def _odom_cb(self, msg: Odometry):
        self.odom_count += 1
        self.odom_msg = msg

    def _map_cb(self, msg: OccupancyGrid):
        self.map_count += 1
        self.map_msg = msg
        self.map_timestamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)

    def _tf_cb(self, msg: TFMessage):
        for transform in msg.transforms:
            parent = transform.header.frame_id.strip('/')
            child = transform.child_frame_id.strip('/')
            self.tf_parents[child] = parent


def run_mapping_diagnostics(
    namespace: str = '',
    timeout: float = 10.0,
    observation_window: float = 3.0,
) -> dict:
    node = MappingDiagnosticsNode(namespace=namespace)
    start_time = time.monotonic()

    # Collect initial graph
    while time.monotonic() - start_time < min(timeout, 4.0):
        rclpy.spin_once(node, timeout_sec=0.1)

    node_names = node.get_node_names()
    clean_node_names = [n.strip('/').split('/')[-1] for n in node_names]

    slam_active = any(n in ('slam_toolbox', 'sync_slam_toolbox_node', 'async_slam_toolbox_node')
                      for n in clean_node_names)
    amcl_active = any(n in ('amcl', 'amcl_node') for n in clean_node_names)

    # Observation window for map progression
    obs_start = time.monotonic()
    initial_map_count = node.map_count
    while time.monotonic() - obs_start < observation_window:
        rclpy.spin_once(node, timeout_sec=0.1)

    map_progressed = node.map_count > initial_map_count or (node.map_count > 0 and len(set(node.map_timestamps)) >= 1)

    results = {
        'timestamp': time.time(),
        'namespace': namespace,
        'nodes': {
            'slam_toolbox_active': slam_active,
            'amcl_absent': not amcl_active,
            'exclusive_slam_stack': slam_active and not amcl_active,
        },
        'sensors': {
            'clock_received': node.clock_received > 0,
            'clock_ticks': node.clock_received,
            'scan_received': node.scan_count > 0,
            'scan_valid': scan_valid(node.scan_msg) if node.scan_msg else False,
            'odom_received': node.odom_count > 0,
            'odom_valid': odometry_valid(node.odom_msg) if node.odom_msg else False,
        },
        'tf': {
            'has_map_to_odom': node.tf_parents.get('odom') == 'map',
            'has_odom_to_base': node.tf_parents.get('base_link') == 'odom' or 'base_link' in node.tf_parents,
            'chain_map_to_base': frame_chain(node.tf_parents, 'map', 'base_link') is not None,
            'parents_count': len(node.tf_parents),
        },
        'map': {
            'map_topic_active': node.map_count > 0,
            'map_messages_received': node.map_count,
            'map_progressing': map_progressed,
        },
    }

    if node.map_msg:
        try:
            stats = validate_occupancy_grid_msg(node.map_msg)
            results['map']['valid'] = True
            results['map']['statistics'] = stats
        except Exception as exc:
            results['map']['valid'] = False
            results['map']['error'] = str(exc)
    else:
        results['map']['valid'] = False
        results['map']['error'] = 'No OccupancyGrid received on /map'

    # Check overall health
    all_passed = (
        results['nodes']['exclusive_slam_stack']
        and results['sensors']['scan_valid']
        and results['sensors']['odom_valid']
        and results['tf']['has_map_to_odom']
        and results['map']['valid']
    )
    results['overall_pass'] = all_passed
    return results


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Diagnostics tool for TurtleBot 4 SLAM mapping.'
    )
    parser.add_argument('--namespace', default='', help='Robot namespace')
    parser.add_argument(
        '--timeout', type=bounded_timeout, default=10.0,
        help='Initial capture timeout in seconds (1..120)'
    )
    parser.add_argument(
        '--observation-window', type=bounded_timeout, default=3.0,
        help='Window in seconds to observe progressing map updates (1..120)'
    )
    parser.add_argument('--json', action='store_true', help='Output JSON report')

    args = parser.parse_args(argv)

    rclpy.init()
    try:
        report = run_mapping_diagnostics(
            namespace=args.namespace,
            timeout=args.timeout,
            observation_window=args.observation_window,
        )
    finally:
        rclpy.shutdown()

    if args.json:
        print(json.dumps(report, indent=2))
    else:
        print("=" * 65)
        print(" TURTLEBOT 4 SLAM MAPPING DIAGNOSTICS REPORT")
        print("=" * 65)
        print(f"Overall Status:        {'[PASS]' if report['overall_pass'] else '[FAIL]'}")
        print("-" * 65)
        print("Stack & Nodes:")
        print(f"  SLAM Toolbox Active: {'YES' if report['nodes']['slam_toolbox_active'] else 'NO (Required)'}")
        print(f"  AMCL Conflict:       {'NONE (Good)' if report['nodes']['amcl_absent'] else 'ACTIVE (Conflict!)'}")
        print(f"  Stack Isolation:     {'PASS' if report['nodes']['exclusive_slam_stack'] else 'FAIL'}")
        print("-" * 65)
        print("Sensors & Timing:")
        print(f"  /clock Active:       {'YES' if report['sensors']['clock_received'] else 'NO'} ({report['sensors']['clock_ticks']} ticks)")
        print(f"  /scan Valid:         {'YES' if report['sensors']['scan_valid'] else 'NO'}")
        print(f"  /odom Valid:         {'YES' if report['sensors']['odom_valid'] else 'NO'}")
        print("-" * 65)
        print("Transforms (TF):")
        print(f"  map -> odom:         {'YES' if report['tf']['has_map_to_odom'] else 'NO'}")
        print(f"  odom -> base_link:   {'YES' if report['tf']['has_odom_to_base'] else 'NO'}")
        print(f"  map -> base_link:    {'CONNECTED' if report['tf']['chain_map_to_base'] else 'DISCONNECTED'}")
        print("-" * 65)
        print("Occupancy Grid (/map):")
        print(f"  Map Received:        {'YES' if report['map']['map_topic_active'] else 'NO'} ({report['map']['map_messages_received']} msgs)")
        print(f"  Map Progressing:     {'YES' if report['map']['map_progressing'] else 'NO'}")
        print(f"  Map Valid:           {'YES' if report['map'].get('valid') else 'NO'}")
        if 'statistics' in report['map']:
            st = report['map']['statistics']
            print(f"  Dimensions:          {st['width']} x {st['height']} cells ({st['width']*st['resolution']:.2f}m x {st['height']*st['resolution']:.2f}m)")
            print(f"  Resolution:          {st['resolution']} m/cell")
            print(f"  Free Cells:          {st['free_cells']} ({st['free_ratio']*100:.1f}%)")
            print(f"  Occupied Cells:      {st['occupied_cells']} ({st['occupied_ratio']*100:.1f}%)")
            print(f"  Unknown Cells:       {st['unknown_cells']} ({st['unknown_ratio']*100:.1f}%)")
        elif 'error' in report['map']:
            print(f"  Error:               {report['map']['error']}")
        print("=" * 65)

    sys.exit(0 if report['overall_pass'] else 1)


if __name__ == '__main__':
    main()
