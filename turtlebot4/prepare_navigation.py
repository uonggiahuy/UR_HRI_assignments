"""One-shot, fail-closed preparation of saved-map localization for Nav2."""

import argparse
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import rclpy
import yaml

from turtlebot4.initial_pose import InitialPoseNode, finite, initialize, lifecycle_active, wait_until
from turtlebot4.localization_diagnostics import validate_map_compatibility
from turtlebot4.localization_runner import bounded_timeout, check_no_active_slam, resolve_saved_map_path
from turtlebot4.localization_health import validate_reference_for_map
from turtlebot4.map_saver import validate_saved_map_files
from turtlebot4.navigation_diagnostics import NavigationProbe, wait_for_readiness
from turtlebot4.simulation_launch import validate_namespace
from turtlebot4.teleop import TeleopNode, undock_robot


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument('--map', help='Saved map YAML; defaults to TURTLEBOT4_MAP or maps/map.yaml')
    result.add_argument('--namespace', default='', type=validate_namespace)
    result.add_argument('--timeout', type=bounded_timeout, default=45.0)
    result.add_argument('--registration', help='Optional independently validated external reference JSON')
    result.add_argument('--x', type=finite, help='Independently known current map-frame x')
    result.add_argument('--y', type=finite, help='Independently known current map-frame y')
    result.add_argument('--yaw-deg', type=finite, help='Independently known current map-frame heading')
    return result


def pose_arguments(args):
    supplied = [args.x is not None, args.y is not None, args.yaw_deg is not None]
    if any(supplied) and not all(supplied):
        raise ValueError('Provide --x, --y and --yaw-deg together')
    return all(supplied)


def preflight(node, map_path, deadline):
    image = yaml.safe_load(map_path.read_text())['image']
    expected = validate_saved_map_files(map_path, map_path.parent / image)
    wait_until(node, lambda: node.sensors_ready() and node.map_msg is not None,
               deadline, 'fresh clock, LiDAR, odometry and saved map')
    check_no_active_slam(node.get_node_names())
    prefix = '/' + node.namespace if node.namespace else ''
    for name in ('map_server', 'amcl'):
        lifecycle_active(node, prefix + '/' + name, deadline)
    validate_map_compatibility(node.map_msg, expected)


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        explicit_pose = pose_arguments(args)
        map_path = resolve_saved_map_path(args.map)
        if args.registration:
            validate_reference_for_map(json.loads(Path(args.registration).read_text()), map_path)
    except (ValueError, OSError) as exc:
        print(f'LOCALIZATION FAIL — {exc}', file=sys.stderr)
        return 1
    rclpy.init()
    node = None
    try:
        node = InitialPoseNode(args.namespace)
        preflight(node, map_path, time.monotonic() + args.timeout)
        node.destroy_node()
        node = TeleopNode(args.namespace)
        dock_deadline = time.monotonic() + min(args.timeout, 5.0)
        while not node.dock_status_received and time.monotonic() < dock_deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if not node.dock_status_received:
            raise RuntimeError('Dock status unavailable; verify robot state before navigation')
        if node.is_docked and not undock_robot(node, timeout=min(args.timeout, 30.0)):
            raise RuntimeError('Undock did not complete; inspect /undock and /dock_status')
        node.destroy_node()
        node = None
        if explicit_pose:
            node = InitialPoseNode(args.namespace)
            initialize(node, SimpleNamespace(map=str(map_path), namespace=args.namespace,
                timeout=args.timeout, x=args.x, y=args.y, yaw_deg=args.yaw_deg, interface='auto'))
            node.destroy_node()
            node = None
        node = NavigationProbe(args.namespace, args.registration)
        graph_deadline = time.monotonic() + min(args.timeout, 8.0)
        while 'turtlebot4_teleop' in node.get_node_names() and time.monotonic() < graph_deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if 'turtlebot4_teleop' in node.get_node_names():
            raise RuntimeError('Teleop node is still present; wait for it to stop')
        report = wait_for_readiness(node, map_path, args.timeout)
        if not report['localization']['navigation_safe']:
            raise RuntimeError('Localization readiness rejected')
        print('LOCALIZATION READY — Safe to start Nav2')
        print('Physical localization accuracy: ' +
              ('VERIFIED' if report['localization']['ground_truth_accuracy_verified'] else 'NOT VERIFIED'))
        return 0
    except (RuntimeError, ValueError, OSError, TimeoutError) as exc:
        reason = str(exc)
        if 'uninitialized' in reason or 'Waiting for pose' in reason or 'initial pose' in reason:
            reason += '; set current pose with RViz 2D Pose Estimate, then retry'
        elif 'uncertain' in reason or 'stability' in reason or 'timestamp' in reason:
            reason += '; inspect scan/map alignment; if clear and undocked, move briefly with teleop, stop, then retry'
        print(f'LOCALIZATION FAIL — {reason}', file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
