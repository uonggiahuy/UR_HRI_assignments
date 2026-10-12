"""Start and manage SLAM Toolbox independently for TurtleBot 4 mapping."""

import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan

from turtlebot4.simulation_launch import validate_namespace


def bounded_timeout(value: str) -> float:
    duration = float(value)
    if not math.isfinite(duration) or not 1.0 <= duration <= 120.0:
        raise argparse.ArgumentTypeError('timeout must be finite and between 1 and 120 seconds')
    return duration


def check_no_duplicate_slam(node_names: list[str]) -> bool:
    """Ensure no SLAM Toolbox instance is already running."""
    for name in node_names:
        clean = name.strip('/').split('/')[-1]
        if clean in ('slam_toolbox', 'sync_slam_toolbox_node', 'async_slam_toolbox_node'):
            raise RuntimeError(
                f"Duplicate SLAM stack rejected: node '{name}' is already active. "
                "Only one SLAM instance may run at a time."
            )
    return True


def check_no_conflicting_localization(node_names: list[str]) -> bool:
    """Ensure AMCL localization is not running simultaneously with SLAM."""
    for name in node_names:
        clean = name.strip('/').split('/')[-1]
        if clean in ('amcl', 'amcl_node'):
            raise RuntimeError(
                f"Incompatible stack rejected: node '{name}' is already active. "
                "SLAM Toolbox and AMCL are mutually exclusive broadcast authorities for map -> odom."
            )
    return True


def default_slam_params_path() -> Path:
    nav_share = Path(get_package_share_directory('turtlebot4_navigation'))
    return nav_share / 'config' / 'slam.yaml'


def build_slam_launch_cmd(
    namespace: str = '',
    sync: bool = True,
    params: Path | str | None = None,
) -> list[str]:
    """Construct the ros2 launch command for upstream SLAM with explicit arguments."""
    nav_share = Path(get_package_share_directory('turtlebot4_navigation'))
    launch_path = nav_share / 'launch' / 'slam.launch.py'
    if not launch_path.is_file():
        raise FileNotFoundError(f'Upstream SLAM launch file not found: {launch_path}')

    params_path = Path(params) if params else default_slam_params_path()
    if not params_path.is_file():
        raise FileNotFoundError(f'SLAM parameters file not found: {params_path}')

    normalized_ns = validate_namespace(namespace)
    sync_str = 'true' if sync else 'false'

    cmd = [
        'ros2', 'launch', 'turtlebot4_navigation', 'slam.launch.py',
        'use_sim_time:=true',
        f'sync:={sync_str}',
    ]
    if normalized_ns:
        cmd.append(f'namespace:={normalized_ns}')
    cmd.append(f'params:={params_path}')
    return cmd


class ReadinessProbe(Node):
    """Temporary probe node to check simulator clock and sensor presence."""

    def __init__(self, scan_topic: str):
        super().__init__('slam_readiness_probe')
        self.has_clock = False
        self.has_scan = False
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self._clock_cb, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self._scan_cb, 10
        )

    def _clock_cb(self, _msg: Clock):
        self.has_clock = True

    def _scan_cb(self, _msg: LaserScan):
        self.has_scan = True


def wait_for_simulation_readiness(
    timeout: float = 15.0,
    namespace: str = '',
) -> bool:
    """Verify that base simulator is running and publishing /clock and /scan."""
    scan_topic = f'/{namespace.strip("/")}/scan' if namespace.strip('/') else '/scan'
    
    probe = ReadinessProbe(scan_topic)
    start_time = time.monotonic()
    try:
        while time.monotonic() - start_time < timeout:
            rclpy.spin_once(probe, timeout_sec=0.2)
            if probe.has_clock and probe.has_scan:
                return True
    finally:
        probe.destroy_node()

    missing = []
    if not probe.has_clock:
        missing.append('/clock')
    if not probe.has_scan:
        missing.append(scan_topic)
    raise TimeoutError(
        f"Base simulator readiness check timed out ({timeout}s). Missing: {', '.join(missing)}. "
        "Ensure Terminal 1 persistent base is running: ros2 launch turtlebot4 turtlebot4.launch.py rviz:=true"
    )


def run_slam_process(cmd: list[str]) -> int:
    """Run SLAM launch process in foreground with signal forwarding for clean Ctrl+C."""
    print(f"[start_slam] Launching: {' '.join(cmd)}")
    proc = subprocess.Popen(cmd)

    def _sig_handler(signum, _frame):
        print("\n[start_slam] Received shutdown signal. Stopping SLAM Toolbox cleanly...")
        try:
            proc.send_signal(signal.SIGINT)
        except ProcessLookupError:
            pass

    original_sigint = signal.signal(signal.SIGINT, _sig_handler)
    original_sigterm = signal.signal(signal.SIGTERM, _sig_handler)

    try:
        returncode = proc.wait()
    finally:
        signal.signal(signal.SIGINT, original_sigint)
        signal.signal(signal.SIGTERM, original_sigterm)

    print("[start_slam] SLAM Toolbox stopped. Simulator and RViz remain running.")
    return returncode


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Launch SLAM Toolbox independently for TurtleBot 4 mapping.'
    )
    parser.add_argument(
        '--namespace', default='', help='Robot namespace (e.g. "robot")'
    )
    parser.add_argument(
        '--sync', default='true', choices=['true', 'false'],
        help='Use synchronous (true) or asynchronous (false) SLAM'
    )
    parser.add_argument(
        '--params', default=None,
        help='Path to custom slam.yaml parameter file'
    )
    parser.add_argument(
        '--timeout', type=bounded_timeout, default=15.0,
        help='Readiness check timeout in seconds (1..120)'
    )
    parser.add_argument(
        '--skip-check', action='store_true',
        help='Skip live simulation readiness pre-check'
    )

    args = parser.parse_args(argv)

    rclpy.init()
    try:
        checker_node = Node('slam_precheck_node')
        try:
            names = checker_node.get_node_names()
            check_no_duplicate_slam(names)
            check_no_conflicting_localization(names)
        finally:
            checker_node.destroy_node()

        if not args.skip_check:
            print(f"[start_slam] Checking base simulator readiness (timeout {args.timeout}s)...")
            wait_for_simulation_readiness(timeout=args.timeout, namespace=args.namespace)
            print("[start_slam] Base simulator verified (/clock and LiDAR active).")
    finally:
        rclpy.shutdown()

    sync_bool = args.sync.lower() == 'true'
    cmd = build_slam_launch_cmd(
        namespace=args.namespace,
        sync=sync_bool,
        params=args.params,
    )

    code = run_slam_process(cmd)
    sys.exit(code if code is not None else 0)


if __name__ == '__main__':
    main()
