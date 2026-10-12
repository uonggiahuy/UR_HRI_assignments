"""Diagnostic tool to inspect Map Server, AMCL localization, TF chain, and pose health."""

import argparse
import json
import math
from pathlib import Path
import sys
import time
import yaml

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

from turtlebot4.diagnostics import frame_chain, scan_valid, odometry_valid, seconds
from turtlebot4.localization_runner import resolve_saved_map_path
from turtlebot4.map_saver import occupancy_signature, validate_saved_map_files
from turtlebot4.simulation_launch import validate_namespace


def bounded_timeout(value: str) -> float:
    duration = float(value)
    if not math.isfinite(duration) or not 1.0 <= duration <= 120.0:
        raise argparse.ArgumentTypeError('timeout must be finite and between 1 and 120 seconds')
    return duration


def validate_amcl_pose_msg(msg: PoseWithCovarianceStamped) -> dict:
    """Validate that AMCL pose and covariance contain finite, well-conditioned values."""
    if msg.header.frame_id != 'map':
        raise ValueError("AMCL pose message has incorrect frame_id (expected 'map')")

    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    coords = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]

    if not all(math.isfinite(c) for c in coords):
        raise ValueError(f"AMCL pose contains non-finite coordinates: {coords}")

    quat_norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
    if abs(quat_norm - 1.0) > 0.05:
        raise ValueError(f"AMCL pose orientation quaternion is not normalized (norm^2={quat_norm:.4f})")

    cov = msg.pose.covariance
    if len(cov) != 36:
        raise ValueError(f"AMCL covariance length {len(cov)} is invalid (expected 36)")

    if not all(math.isfinite(c) for c in cov):
        raise ValueError("AMCL covariance contains non-finite elements")

    # Diagonal elements (variances) must be non-negative
    var_x, var_y, var_yaw = cov[0], cov[7], cov[35]
    if var_x < 0 or var_y < 0 or var_yaw < 0:
        raise ValueError(f"AMCL covariance diagonal contains negative variances: x={var_x}, y={var_y}, yaw={var_yaw}")

    # Calculate 2D yaw from quaternion
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return {
        'frame_id': msg.header.frame_id,
        'stamp_sec': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
        'position': [round(p.x, 3), round(p.y, 3), round(p.z, 3)],
        'yaw_rad': round(yaw, 3),
        'yaw_deg': round(math.degrees(yaw), 1),
        'variance': {
            'x': round(var_x, 5),
            'y': round(var_y, 5),
            'yaw': round(var_yaw, 5),
        },
        'covariance_diagonal': [round(cov[i * 7], 5) for i in range(6)],
    }


def validate_map_compatibility(msg: OccupancyGrid, expected_yaml: dict) -> dict:
    """Verify live OccupancyGrid metadata against saved map metadata."""
    info = msg.info
    exp_res = expected_yaml['resolution']
    exp_w = expected_yaml['width']
    exp_h = expected_yaml['height']
    exp_origin = expected_yaml['origin']

    if abs(info.resolution - exp_res) > 1e-4:
        raise ValueError(f"Live map resolution {info.resolution} does not match expected {exp_res}")

    if info.width != exp_w or info.height != exp_h:
        raise ValueError(f"Live map dimensions {info.width}x{info.height} do not match expected {exp_w}x{exp_h}")

    if abs(info.origin.position.x - exp_origin[0]) > 0.05 or abs(info.origin.position.y - exp_origin[1]) > 0.05:
        raise ValueError(
            f"Live map origin ({info.origin.position.x:.2f}, {info.origin.position.y:.2f}) "
            f"differs from expected ({exp_origin[0]:.2f}, {exp_origin[1]:.2f})"
        )

    free_count = sum(1 for c in msg.data if 0 <= c <= 25)
    occupied_count = sum(1 for c in msg.data if c > 25)
    unknown_count = sum(1 for c in msg.data if c == -1)

    if len(msg.data) != info.width * info.height or any(c not in (-1, 0, 100) for c in msg.data):
        raise ValueError('Live trinary map has invalid occupancy data')
    for key, count in (('free_cells', free_count), ('occupied_cells', occupied_count),
                       ('unknown_cells', unknown_count)):
        if key in expected_yaml and count != expected_yaml[key]:
            raise ValueError(f'Live map {key} {count} does not match saved PGM {expected_yaml[key]}')
    if ('occupancy_sha256' in expected_yaml
            and occupancy_signature(msg.data) != expected_yaml['occupancy_sha256']):
        raise ValueError('Live map cell layout does not match saved PGM')

    return {
        'width': info.width,
        'height': info.height,
        'resolution': info.resolution,
        'origin': [info.origin.position.x, info.origin.position.y, info.origin.position.z],
        'total_cells': info.width * info.height,
        'free_cells': free_count,
        'occupied_cells': occupied_count,
        'unknown_cells': unknown_count,
    }


class LocalizationDiagnosticsNode(Node):
    def __init__(self, namespace: str = ''):
        super().__init__('localization_diagnostics_node', namespace=validate_namespace(namespace))
        self.namespace = validate_namespace(namespace)
        prefix = f'/{self.namespace}' if self.namespace else ''

        from rclpy.parameter import Parameter
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.clock_received = 0
        self.scan_msg = None
        self.odom_msg = None
        self.map_msg = None
        self.amcl_pose_msg = None
        self.amcl_pose_count = 0

        self.tf_parents = {}
        self.tf_times = {}
        self.clock_time = 0.0
        self.first_clock_time = None

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

        self.sub_clock = self.create_subscription(Clock, '/clock', self._clock_cb, 10)
        self.sub_scan = self.create_subscription(
            LaserScan, f'{prefix}/scan' if prefix else '/scan', self._scan_cb, best_effort_qos
        )
        self.sub_odom = self.create_subscription(
            Odometry, f'{prefix}/odom' if prefix else '/odom', self._odom_cb, best_effort_qos
        )
        self.sub_map = self.create_subscription(
            OccupancyGrid, f'{prefix}/map' if prefix else '/map', self._map_cb, transient_local_qos
        )
        self.sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped, f'{prefix}/amcl_pose' if prefix else '/amcl_pose',
            self._amcl_cb, transient_local_qos
        )
        self.sub_tf = self.create_subscription(
            TFMessage, f'{prefix}/tf' if prefix else '/tf', self._tf_cb, 100
        )
        self.sub_tf_static = self.create_subscription(
            TFMessage, f'{prefix}/tf_static' if prefix else '/tf_static', self._tf_cb, transient_local_qos
        )

    def _clock_cb(self, _msg: Clock):
        self.clock_received += 1
        self.clock_time = seconds(_msg.clock)
        if self.first_clock_time is None:
            self.first_clock_time = self.clock_time

    def _scan_cb(self, msg: LaserScan):
        self.scan_msg = msg

    def _odom_cb(self, msg: Odometry):
        self.odom_msg = msg

    def _map_cb(self, msg: OccupancyGrid):
        self.map_msg = msg

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        self.amcl_pose_count += 1
        self.amcl_pose_msg = msg

    def _tf_cb(self, msg: TFMessage):
        for transform in msg.transforms:
            parent = transform.header.frame_id.strip('/')
            child = transform.child_frame_id.strip('/')
            self.tf_parents[child] = parent
            self.tf_times[(parent, child)] = seconds(transform.header.stamp)


def run_localization_diagnostics(
    map_yaml_path: Path | str | None = None,
    namespace: str = '',
    timeout: float = 10.0,
) -> dict:
    resolved_map = resolve_saved_map_path(map_yaml_path)
    image_ref = yaml.safe_load(resolved_map.read_text())['image']
    expected_meta = validate_saved_map_files(resolved_map, resolved_map.parent / image_ref)

    node = LocalizationDiagnosticsNode(namespace=namespace)
    start_time = time.monotonic()

    # Collect graph and topics
    while time.monotonic() - start_time < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        # If we have all required elements, we can conclude early
        if (node.clock_received > 5 and node.scan_msg and node.odom_msg and
            node.map_msg and node.amcl_pose_msg and node.tf_parents.get('odom') == 'map'):
            break

    node_names = node.get_node_names()
    clean_node_names = [n.strip('/').split('/')[-1] for n in node_names]

    slam_active = any(n in ('slam_toolbox', 'sync_slam_toolbox_node', 'async_slam_toolbox_node')
                      for n in clean_node_names)
    amcl_present = any(n in ('amcl', 'amcl_node') for n in clean_node_names)
    map_server_present = any(n in ('map_server',) for n in clean_node_names)

    # Node presence does not establish lifecycle ACTIVE. Query the real services.
    from turtlebot4.initial_pose import lifecycle_active
    prefix = '/' + validate_namespace(namespace) if namespace else ''
    states = {}
    for name in ('amcl', 'map_server'):
        try:
            states[name] = lifecycle_active(node, prefix + '/' + name, time.monotonic() + 2.0)
        except (RuntimeError, TimeoutError) as exc:
            states[name] = str(exc)
    amcl_active = states['amcl'] == 'active'
    map_server_active = states['map_server'] == 'active'
    clock_progressing = (node.first_clock_time is not None
                         and node.clock_time > node.first_clock_time)
    tf_fresh = all(-1.5 <= node.clock_time - node.tf_times.get(edge, -math.inf) <= 1.5
                   for edge in (('map', 'odom'), ('odom', 'base_link')))

    results = {
        'timestamp': time.time(),
        'namespace': namespace,
        'map_file': str(resolved_map),
        'nodes': {
            'amcl_active': amcl_active,
            'amcl_present': amcl_present,
            'map_server_present': map_server_present,
            'lifecycle_states': states,
            'map_server_active': map_server_active,
            'slam_absent': not slam_active,
            'exclusive_localization_stack': amcl_active and map_server_active and not slam_active,
        },
        'sensors': {
            'clock_received': node.clock_received > 0,
            'clock_progressing': clock_progressing,
            'clock_ticks': node.clock_received,
            'scan_valid': scan_valid(node.scan_msg) if node.scan_msg else False,
            'odom_valid': odometry_valid(node.odom_msg) if node.odom_msg else False,
        },
        'tf': {
            'has_map_to_odom': node.tf_parents.get('odom') == 'map',
            'dynamic_tf_fresh': tf_fresh,
            'has_odom_to_base': node.tf_parents.get('base_link') == 'odom' or 'base_link' in node.tf_parents,
            'chain_map_to_base': frame_chain(node.tf_parents, 'map', 'base_link') is not None,
            'parents_count': len(node.tf_parents),
        },
        'map_server': {
            'map_topic_active': node.map_msg is not None,
        },
        'amcl': {
            'pose_received': node.amcl_pose_msg is not None,
            'pose_count': node.amcl_pose_count,
        },
    }

    if node.map_msg:
        try:
            map_stats = validate_map_compatibility(node.map_msg, expected_meta)
            results['map_server']['valid'] = True
            results['map_server']['statistics'] = map_stats
        except Exception as exc:
            results['map_server']['valid'] = False
            results['map_server']['error'] = str(exc)
    else:
        results['map_server']['valid'] = False
        results['map_server']['error'] = 'No OccupancyGrid received from Map Server'

    if node.amcl_pose_msg:
        try:
            pose_data = validate_amcl_pose_msg(node.amcl_pose_msg)
            results['amcl']['valid'] = True
            results['amcl']['pose_data'] = pose_data
        except Exception as exc:
            results['amcl']['valid'] = False
            results['amcl']['error'] = str(exc)
    else:
        results['amcl']['valid'] = False
        results['amcl']['error'] = 'No /amcl_pose received (run set_initial_pose --x X --y Y --yaw-deg ANGLE in map coordinates)'

    all_passed = (
        results['nodes']['exclusive_localization_stack']
        and clock_progressing
        and tf_fresh
        and results['sensors']['scan_valid']
        and results['sensors']['odom_valid']
        and results['tf']['has_map_to_odom']
        and results['tf']['chain_map_to_base']
        and results['map_server']['valid']
        and results['amcl']['valid']
    )
    results['overall_pass'] = all_passed
    node.destroy_node()
    return results


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Diagnostics tool for TurtleBot 4 AMCL localization.'
    )
    parser.add_argument(
        '--map', default=None, help='Path to map YAML file (default: maps/map.yaml)'
    )
    parser.add_argument('--namespace', default='', help='Robot namespace')
    parser.add_argument(
        '--timeout', type=bounded_timeout, default=10.0,
        help='Observation timeout in seconds (1..120)'
    )
    parser.add_argument('--json', action='store_true', help='Output JSON report')

    args = parser.parse_args(argv)

    rclpy.init()
    try:
        report = run_localization_diagnostics(
            map_yaml_path=args.map,
            namespace=args.namespace,
            timeout=args.timeout,
        )
    finally:
        if rclpy.ok():
            rclpy.shutdown()

    if args.json:
        print(json.dumps(report, indent=2))
    else:
        print("=" * 68)
        print(" TURTLEBOT 4 AMCL LOCALIZATION DIAGNOSTICS REPORT")
        print("=" * 68)
        print(f"Overall Status:        {'[PASS]' if report['overall_pass'] else '[FAIL]'}")
        print("-" * 68)
        print("Stack & Nodes:")
        print(f"  AMCL Active:         {'YES' if report['nodes']['amcl_active'] else 'NO (Required)'}")
        print(f"  Map Server Active:   {'YES' if report['nodes']['map_server_active'] else 'NO (Required)'}")
        print(f"  SLAM Conflict:       {'NONE (Good)' if report['nodes']['slam_absent'] else 'ACTIVE (Conflict!)'}")
        print(f"  Stack Isolation:     {'PASS' if report['nodes']['exclusive_localization_stack'] else 'FAIL'}")
        print("-" * 68)
        print("Sensors & Timing:")
        print(f"  /clock Active:       {'YES' if report['sensors']['clock_received'] else 'NO'} ({report['sensors']['clock_ticks']} ticks)")
        print(f"  /scan Valid:         {'YES' if report['sensors']['scan_valid'] else 'NO'}")
        print(f"  /odom Valid:         {'YES' if report['sensors']['odom_valid'] else 'NO'}")
        print("-" * 68)
        print("Transforms (TF):")
        print(f"  map -> odom:         {'YES (AMCL)' if report['tf']['has_map_to_odom'] else 'NO'}")
        print(f"  odom -> base_link:   {'YES' if report['tf']['has_odom_to_base'] else 'NO'}")
        print(f"  map -> base_link:    {'CONNECTED' if report['tf']['chain_map_to_base'] else 'DISCONNECTED'}")
        print("-" * 68)
        print("Map Server (/map):")
        print(f"  Map Received:        {'YES' if report['map_server']['map_topic_active'] else 'NO'}")
        print(f"  Map Valid:           {'YES' if report['map_server'].get('valid') else 'NO'}")
        if 'statistics' in report['map_server']:
            st = report['map_server']['statistics']
            print(f"  Dimensions:          {st['width']} x {st['height']} cells ({st['width']*st['resolution']:.2f}m x {st['height']*st['resolution']:.2f}m)")
            print(f"  Resolution:          {st['resolution']} m/cell")
            print(f"  Free Cells:          {st['free_cells']}")
            print(f"  Occupied Cells:      {st['occupied_cells']}")
            print(f"  Unknown Cells:       {st['unknown_cells']}")
        elif 'error' in report['map_server']:
            print(f"  Error:               {report['map_server']['error']}")
        print("-" * 68)
        print("AMCL Localization (/amcl_pose):")
        print(f"  Pose Received:       {'YES' if report['amcl']['pose_received'] else 'NO'} ({report['amcl']['pose_count']} msgs)")
        print(f"  Pose Valid:          {'YES' if report['amcl'].get('valid') else 'NO'}")
        if 'pose_data' in report['amcl']:
            pd = report['amcl']['pose_data']
            print(f"  Position (x,y,z):    {pd['position']}")
            print(f"  Heading (yaw):       {pd['yaw_deg']} deg ({pd['yaw_rad']} rad)")
            print(f"  Variance (x, y, yaw): {pd['variance']}")
        elif 'error' in report['amcl']:
            print(f"  Error:               {report['amcl']['error']}")
        print("=" * 68)

    sys.exit(0 if report['overall_pass'] else 1)


if __name__ == '__main__':
    main()
