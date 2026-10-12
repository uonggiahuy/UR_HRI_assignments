"""Bounded live readiness checks shared by Nav2 runner and operator diagnostics."""
import argparse
import ast
from collections import Counter
import json
import math
from pathlib import Path
import time
import uuid

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, PolygonStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException
import yaml

from turtlebot4.diagnostics import scan_valid, odometry_valid, seconds
from turtlebot4.localization_diagnostics import validate_amcl_pose_msg, validate_map_compatibility
from turtlebot4.localization_runner import bounded_timeout, resolve_saved_map_path
from turtlebot4.map_saver import map_qos, validate_saved_map_files
from turtlebot4.simulation_launch import validate_namespace


def upstream_lifecycle_nodes() -> list[str]:
    """Discover the complete installed launch list, refusing unknown conventions."""
    path = Path(get_package_share_directory('nav2_bringup')) / 'launch/navigation_launch.py'
    for item in ast.walk(ast.parse(path.read_text())):
        if isinstance(item, ast.Assign) and any(
                isinstance(target, ast.Name) and target.id == 'lifecycle_nodes' for target in item.targets):
            result = ast.literal_eval(item.value)
            if result and all(isinstance(name, str) for name in result):
                return result
    raise RuntimeError(f'Cannot discover lifecycle_nodes in installed launch: {path}')


def check_graph(names: list[str], navigation: bool = False) -> None:
    counts = Counter(names)
    for name, count in counts.items():
        if count > 1:
            raise RuntimeError(f'Duplicate ROS node: {name} ({count} instances)')
    leaves = [name.strip('/').split('/')[-1] for name in names]
    if any(name in leaves for name in ('slam_toolbox', 'sync_slam_toolbox_node', 'async_slam_toolbox_node')):
        raise RuntimeError('Active SLAM conflicts with AMCL/navigation')
    for name in ('amcl', 'map_server'):
        if leaves.count(name) != 1:
            raise RuntimeError(f'Require exactly one existing {name}')
    if not navigation and any(name in leaves for name in upstream_lifecycle_nodes() + ['lifecycle_manager_navigation']):
        raise RuntimeError('Navigation stack already running; duplicate launch rejected')
    if 'turtlebot4_teleop' in leaves:
        raise RuntimeError('Stop teleop before navigation')


def costmap_stamp(msg: Costmap) -> float:
    """Humble raw topics leave update_time zero; their header carries publish time.

    A nonzero update_time remains authoritative. Fresh footprints are checked
    separately because publication alone does not prove the TF cache advanced.
    """
    update = seconds(msg.metadata.update_time)
    return update if update > 0.0 else seconds(msg.header.stamp)


def validate_costmap_freshness(msg: Costmap, now: float | None = None) -> None:
    """Constant-time motion guard; do not recount a full grid between callbacks."""
    meta = msg.metadata
    if (meta.size_x == 0 or meta.size_y == 0 or len(msg.data) != meta.size_x * meta.size_y
            or not math.isfinite(meta.resolution) or meta.resolution <= 0):
        raise ValueError('Invalid costmap dimensions/resolution/data')
    if now is not None and abs(now - costmap_stamp(msg)) > 3.0:
        raise ValueError('Costmap update timestamp is stale')


def validate_costmap(msg: Costmap, now: float | None = None) -> dict:
    validate_costmap_freshness(msg, now)
    meta = msg.metadata
    counts = Counter(msg.data)
    return {'width': meta.size_x, 'height': meta.size_y, 'resolution': meta.resolution,
            'unknown': counts[255], 'lethal': counts[254], 'free': counts[0],
            'inflated': sum(count for cost, count in counts.items() if 0 < cost < 254)}


def cell_index(x: float, y: float, msg) -> int:
    """Map a world coordinate into either OccupancyGrid or raw Costmap."""
    if isinstance(msg, Costmap):
        meta = msg.metadata
        width, height, resolution, origin = meta.size_x, meta.size_y, meta.resolution, meta.origin
    else:
        meta = msg.info
        width, height, resolution, origin = meta.width, meta.height, meta.resolution, meta.origin
    # Map/costmap grids are axis-aligned in this verified upstream configuration.
    if abs(origin.orientation.z) > 1e-6:
        raise ValueError('Rotated grid requires a coordinate transform')
    ix = math.floor((x - origin.position.x) / resolution)
    iy = math.floor((y - origin.position.y) / resolution)
    if not (0 <= ix < width and 0 <= iy < height):
        raise ValueError('Coordinate outside map')
    return iy * width + ix


def validate_path(path: RosPath, grid: OccupancyGrid, costmap: Costmap) -> dict:
    """Check dense interpolated segments, not merely the sparse path vertices."""
    if path.header.frame_id != 'map' or not path.poses:
        raise ValueError('Expected a nonempty map-frame path')
    points = [(pose.pose.position.x, pose.pose.position.y) for pose in path.poses]
    if not all(math.isfinite(value) for point in points for value in point):
        raise ValueError('Path contains non-finite coordinates')
    samples = [points[0]]
    for start, end in zip(points, points[1:]):
        steps = max(1, math.ceil(math.dist(start, end) / (grid.info.resolution / 2)))
        samples.extend((start[0] + (end[0] - start[0]) * t / steps,
                        start[1] + (end[1] - start[1]) * t / steps) for t in range(1, steps + 1))
    for x, y in samples:
        if grid.data[cell_index(x, y, grid)] != 0:
            raise ValueError('Path traverses unknown/occupied saved-map space')
        if costmap.data[cell_index(x, y, costmap)] >= 253:
            raise ValueError('Path traverses unknown/lethal/inscribed costmap space')
    return {'poses': len(points), 'checked_samples': len(samples),
            'length_m': sum(math.dist(a, b) for a, b in zip(points, points[1:]))}


class NavigationProbe(Node):
    def __init__(self, namespace=''):
        normalized = validate_namespace(namespace)
        super().__init__('turtlebot4_navigation_probe_' + uuid.uuid4().hex[:8],
                         namespace=normalized, cli_args=['--ros-args', '-r', '/tf:=tf', '-r', '/tf_static:=tf_static'])
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.namespace = validate_namespace(namespace)
        self.prefix = '/' + self.namespace if self.namespace else ''
        self.service_clients = {}
        self.messages = {}
        self.received = {}
        self.clock_samples = []
        self.costmap_samples = {'global': [], 'local': []}
        self.tf_samples = []
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.subscriptions_owned = []
        for key, cls, topic, qos in [
            ('clock', Clock, '/clock', 10),
            ('scan', LaserScan, self.topic('scan'), qos_profile_sensor_data),
            ('odom', Odometry, self.topic('odom'), qos_profile_sensor_data),
            ('map', OccupancyGrid, self.topic('map'), map_qos()),
            ('pose', PoseWithCovarianceStamped, self.topic('amcl_pose'), map_qos()),
            ('global', Costmap, self.topic('global_costmap/costmap_raw'), map_qos()),
            ('local', Costmap, self.topic('local_costmap/costmap_raw'), map_qos()),
            ('path', RosPath, self.topic('plan'), 10),
            ('global_footprint', PolygonStamped, self.topic('global_costmap/published_footprint'), qos_profile_sensor_data),
            ('local_footprint', PolygonStamped, self.topic('local_costmap/published_footprint'), qos_profile_sensor_data),
        ]:
            self.subscriptions_owned.append(self.create_subscription(
                cls, topic, lambda msg, key=key: self.record(key, msg), qos))
        self.action = ActionClient(self, NavigateToPose, self.topic('navigate_to_pose'))

    def topic(self, name):
        return self.prefix + '/' + name

    def record(self, key, msg):
        self.messages[key] = msg
        self.received[key] = time.monotonic()
        if key == 'clock':
            self.clock_samples.append(seconds(msg.clock))
        if key in self.costmap_samples:
            self.costmap_samples[key].append(costmap_stamp(msg))

    def request(self, service_type, service, request, deadline):
        if time.monotonic() >= deadline:
            raise TimeoutError('Readiness observation deadline exhausted')
        # Keep service endpoints alive while spinning the probe, rather than
        # repeatedly destroying reliable reply readers during startup.
        key = (service_type, service)
        if key not in self.service_clients:
            self.service_clients[key] = self.create_client(service_type, service)
        client = self.service_clients[key]
        remaining = max(0.0, deadline - time.monotonic())
        if not client.wait_for_service(timeout_sec=min(remaining, 2.0)):
            if time.monotonic() >= deadline:
                raise TimeoutError('Readiness observation deadline exhausted')
            raise RuntimeError(f'Service unavailable: {service}')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=min(5.0, max(0.0, deadline - time.monotonic())))
        if not future.done() or future.result() is None:
            client.remove_pending_request(future)
            future.cancel()
            if time.monotonic() >= deadline:
                raise TimeoutError('Readiness observation deadline exhausted')
            raise RuntimeError(f'Service timed out: {service}')
        return future.result()

    def parameters(self, name, keys, deadline):
        request = GetParameters.Request(names=keys)
        values = self.request(GetParameters, self.topic(name + '/get_parameters'), request, deadline).values
        result = {}
        for key, value in zip(keys, values):
            fields = {1: 'bool_value', 2: 'integer_value', 3: 'double_value',
                      4: 'string_value', 8: 'double_array_value', 9: 'string_array_value'}
            result[key] = getattr(value, fields[value.type]) if value.type in fields else None
        return result

    def lifecycle(self, names, deadline):
        result = {}
        for name in names:
            state = self.request(GetState, self.topic(name + '/get_state'), GetState.Request(), deadline).current_state
            if state.id != 3:
                raise RuntimeError(f'{name} lifecycle is {state.label}, expected active')
            result[name] = state.label
        return result

    def localization(self, map_path, deadline):
        full_names = [ns.rstrip('/') + '/' + name for name, ns in self.get_node_names_and_namespaces()]
        check_graph(full_names, navigation=True)
        lifecycle = self.lifecycle(['map_server', 'amcl'], deadline)
        for key in ('clock', 'scan', 'odom', 'map', 'pose'):
            if key not in self.messages:
                raise RuntimeError(f'Waiting for {key}')
        now = self.clock_samples[-1]
        if any(b < a for a, b in zip(self.clock_samples, self.clock_samples[1:])):
            raise RuntimeError('Simulation clock moved backwards')
        if len(self.clock_samples) < 5 or self.clock_samples[-1] <= self.clock_samples[0]:
            raise RuntimeError('Simulation clock is not progressing')
        for key, valid in (('scan', scan_valid), ('odom', odometry_valid)):
            msg = self.messages[key]
            if (not valid(msg) or abs(now - seconds(msg.header.stamp)) > 1.0
                    or time.monotonic() - self.received[key] > 1.0):
                raise RuntimeError(f'{key} invalid or stale')
        pose = validate_amcl_pose_msg(self.messages['pose'])
        if pose['frame_id'] != 'map' or max(pose['variance'].values()) > 0.5:
            raise RuntimeError('AMCL pose is uninitialized or uncertain')
        for parent, child in [('map', 'odom'), ('odom', 'base_link'), ('map', 'base_link')]:
            try:
                tf = self.buffer.lookup_transform(parent, child, rclpy.time.Time())
            except TransformException as exc:
                raise RuntimeError(f'TF {parent} -> {child}: {exc}') from exc
            if abs(now - seconds(tf.header.stamp)) > 2.0:
                raise RuntimeError(f'Stale TF {parent} -> {child}')
        tf = self.buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        stamp = seconds(tf.header.stamp)
        self.tf_samples.append((time.monotonic(), stamp, tf.transform.translation.x, tf.transform.translation.y))
        if len(self.tf_samples) < 2 or self.tf_samples[-1][0] - self.tf_samples[0][0] < 2:
            raise RuntimeError('Observing localization stability over two seconds')
        if stamp <= self.tf_samples[0][1]:
            raise RuntimeError('Localization TF is not advancing')
        odom = self.messages['odom']
        if abs(odom.twist.twist.linear.x) < 0.02 and abs(odom.twist.twist.angular.z) < 0.05:
            if math.hypot(tf.transform.translation.x - self.tf_samples[0][2],
                          tf.transform.translation.y - self.tf_samples[0][3]) > 0.15:
                raise RuntimeError('Stationary localization is unstable')
        meta = yaml.safe_load(Path(map_path).read_text())
        expected = validate_saved_map_files(Path(map_path), Path(map_path).parent / meta['image'])
        stats = validate_map_compatibility(self.messages['map'], expected)
        if self.messages['map'].data[cell_index(tf.transform.translation.x, tf.transform.translation.y, self.messages['map'])] != 0:
            raise RuntimeError('Localized robot is outside saved-map free space')
        return {'lifecycle': lifecycle, 'map': stats, 'pose': pose,
                'clock': now, 'tf_stamp': stamp, 'stable_observation_seconds': self.tf_samples[-1][0] - self.tf_samples[0][0]}

    def navigation(self, deadline):
        managed = self.parameters('lifecycle_manager_navigation', ['node_names'], deadline)['node_names']
        if not managed or set(managed) != set(upstream_lifecycle_nodes()):
            raise RuntimeError('Navigation lifecycle list differs from installed upstream launch')
        states = self.lifecycle(managed + ['global_costmap/global_costmap', 'local_costmap/local_costmap'], deadline)
        if not self.action.server_is_ready():
            raise RuntimeError('NavigateToPose action unavailable')
        params = {
            'planner': self.parameters('planner_server', ['use_sim_time', 'GridBased.allow_unknown', 'GridBased.plugin', 'GridBased.tolerance'], deadline),
            'global': self.parameters('global_costmap/global_costmap', [
                'use_sim_time', 'track_unknown_space', 'robot_radius', 'plugins',
                'static_layer.map_subscribe_transient_local', 'obstacle_layer.scan.topic',
                'inflation_layer.inflation_radius'], deadline),
            'local': self.parameters('local_costmap/local_costmap', [
                'use_sim_time', 'robot_radius', 'plugins', 'voxel_layer.scan.topic',
                'inflation_layer.inflation_radius'], deadline),
            'controller': self.parameters('controller_server', ['use_sim_time', 'FollowPath.plugin',
                'general_goal_checker.xy_goal_tolerance', 'general_goal_checker.yaw_goal_tolerance'], deadline),
        }
        if (params['planner']['GridBased.allow_unknown'] is not False
                or params['global']['track_unknown_space'] is not True
                or params['planner']['GridBased.tolerance'] != 0.0):
            raise RuntimeError('Unsafe runtime unknown-space parameters')
        if not all(values['use_sim_time'] is True for values in params.values()):
            raise RuntimeError('Nav2 node is not using simulation time')
        maps = {}
        for key in ('global', 'local'):
            if key not in self.messages or time.monotonic() - self.received[key] > 3:
                raise RuntimeError(f'{key} costmap absent or stale')
            samples = self.costmap_samples[key]
            if len(samples) < 2 or max(samples) <= min(samples):
                raise RuntimeError(f'{key} costmap updates are not progressing')
            footprint = self.messages.get(key + '_footprint')
            if footprint is None or abs(self.clock_samples[-1] - seconds(footprint.header.stamp)) > 3.0:
                raise RuntimeError(f'{key} costmap footprint is absent or stale')
            tf = self.buffer.lookup_transform(footprint.header.frame_id, 'base_link', rclpy.time.Time())
            points = footprint.polygon.points
            if not points:
                raise RuntimeError(f'{key} footprint is empty')
            center = (sum(p.x for p in points) / len(points), sum(p.y for p in points) / len(points))
            if math.dist(center, (tf.transform.translation.x, tf.transform.translation.y)) > 0.15:
                raise RuntimeError(f'{key} footprint disagrees with live robot TF')
            maps[key] = validate_costmap(self.messages[key], self.clock_samples[-1])
        if self.localization_unknown_count() > 0 and maps['global']['unknown'] == 0:
            raise RuntimeError('Global costmap lost all unknown cells')
        interfaces = {name: [info.node_name for info in self.get_publishers_info_by_topic(self.topic(name))]
                      for name in ('plan', 'cmd_vel', 'cmd_vel_nav')}
        if not interfaces['cmd_vel'] or not self.get_subscriptions_info_by_topic(self.topic('cmd_vel')):
            raise RuntimeError('Robot command interface unavailable')
        path = None
        if 'path' in self.messages:
            path = validate_path(self.messages['path'], self.messages['map'], self.messages['global'])
        return {'lifecycle': states, 'parameters': params, 'costmaps': maps,
                'navigate_to_pose_available': True, 'interfaces': interfaces,
                'last_path': path, 'autonomous_goals_verified': False}

    def localization_unknown_count(self):
        return sum(value == -1 for value in self.messages['map'].data)


def wait_for_readiness(probe, map_path, timeout=30.0, navigation=False, process=None):
    deadline = time.monotonic() + timeout
    last_error = 'No messages received'
    while time.monotonic() < deadline:
        if process is not None and process.poll() is not None:
            raise RuntimeError(f'Owned Nav2 launch exited with code {process.returncode}')
        rclpy.spin_once(probe, timeout_sec=0.1)
        try:
            result = {'localization': probe.localization(map_path, deadline)}
            if navigation:
                result['navigation'] = probe.navigation(deadline)
            return result
        except (RuntimeError, ValueError) as exc:
            last_error = str(exc)
        except TimeoutError:
            # Do not replace the last failed sensor/TF/costmap check with a
            # service timeout caused solely by the overall observation budget.
            break
    raise TimeoutError(f'Readiness failed after {timeout}s: {last_error}')


def parser():
    result = argparse.ArgumentParser(description='Verify live Nav2 readiness; does not claim goal success.')
    result.add_argument('--namespace', default='')
    result.add_argument('--map', default=None)
    result.add_argument('--timeout', type=bounded_timeout, default=30.0)
    result.add_argument('--json', action='store_true')
    return result


def main(argv=None):
    args = parser().parse_args(argv)
    rclpy.init()
    probe = NavigationProbe(args.namespace)
    try:
        report = wait_for_readiness(probe, resolve_saved_map_path(args.map), args.timeout, navigation=True)
        report['status'] = 'PASS'
        code = 0
    except (RuntimeError, ValueError, TimeoutError) as exc:
        report = {'status': 'FAIL', 'error': str(exc)}
        code = 1
    finally:
        probe.destroy_node()
        rclpy.shutdown()
    if args.json:
        print(json.dumps(report, indent=2))
    else:
        print('Nav2 readiness: ' + report['status'])
        print(json.dumps(report, indent=2))
        print('Readiness does not prove autonomous goal success.')
    return code


if __name__ == '__main__':
    import sys
    sys.exit(main())
