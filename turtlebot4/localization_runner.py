"""Start and manage Map Server and AMCL localization independently for TurtleBot 4."""

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
from nav_msgs.msg import Odometry

from turtlebot4.map_saver import validate_saved_map_files
from turtlebot4.simulation_launch import validate_namespace


def bounded_timeout(value: str) -> float:
    duration = float(value)
    if not math.isfinite(duration) or not 1.0 <= duration <= 120.0:
        raise argparse.ArgumentTypeError('timeout must be finite and between 1 and 120 seconds')
    return duration


def check_no_active_slam(node_names: list[str]) -> bool:
    """Ensure no SLAM Toolbox instance is running. SLAM and AMCL are mutually exclusive."""
    for name in node_names:
        clean = name.strip('/').split('/')[-1]
        if clean in ('slam_toolbox', 'sync_slam_toolbox_node', 'async_slam_toolbox_node'):
            raise RuntimeError(
                f"Active SLAM Toolbox detected: node '{name}' is active. "
                "SLAM and AMCL are mutually exclusive broadcast authorities for map -> odom. "
                "Please stop SLAM (Ctrl+C in Terminal 2) before starting localization."
            )
    return True


def check_no_duplicate_localization(node_names: list[str]) -> bool:
    """Ensure no other AMCL or Map Server instance is already running."""
    for name in node_names:
        clean = name.strip('/').split('/')[-1]
        if clean in ('amcl', 'amcl_node', 'map_server'):
            raise RuntimeError(
                f"Duplicate localization stack rejected: node '{name}' is already active. "
                "Only one localization instance may run at a time."
            )
    return True


def default_map_yaml_path() -> Path:
    """Locate the default M2 saved map in source tree or installed package share."""
    src_map = Path(__file__).resolve().parents[1] / 'maps' / 'map.yaml'
    if src_map.is_file():
        return src_map
    share_map = Path(get_package_share_directory('turtlebot4')) / 'maps' / 'map.yaml'
    if share_map.is_file():
        return share_map
    raise FileNotFoundError(
        f"M2 saved map not found at {src_map} or {share_map}. "
        "Run 'ros2 run turtlebot4 save_map' in M2 before launching localization."
    )


def default_localization_params_path() -> Path:
    share = Path(get_package_share_directory('turtlebot4'))
    return share / 'config' / 'localization.yaml'


def resolve_saved_map_path(map_arg: str | Path | None = None) -> Path:
    """Resolve and validate the map YAML file and its referenced PGM image."""
    selection = map_arg if map_arg else os.environ.get('TURTLEBOT4_MAP')
    yaml_path = Path(selection).expanduser() if selection else default_map_yaml_path()
    if selection and not yaml_path.is_absolute() and yaml_path.parent == Path('.'):
        yaml_path = default_map_yaml_path().parent / yaml_path
    if not yaml_path.is_file():
        raise FileNotFoundError(f"Specified map YAML file does not exist: {yaml_path}")

    # Validate YAML content and referenced image
    pgm_path = yaml_path.parent / 'map.pgm'
    # Use general image lookup if referenced image has a different name
    import yaml
    try:
        with open(yaml_path, 'r', encoding='utf-8') as f:
            meta = yaml.safe_load(f)
    except yaml.YAMLError as exc:
        raise ValueError(f"Invalid map YAML in {yaml_path}: {exc}") from exc

    if isinstance(meta, dict) and 'image' in meta:
        pgm_path = (yaml_path.parent / meta['image']).resolve()

    validate_saved_map_files(yaml_path, pgm_path)
    return yaml_path


def build_localization_launch_cmd(
    map_path: Path | str | None = None,
    params_path: Path | str | None = None,
    namespace: str = '',
    use_sim_time: bool = True,
) -> list[str]:
    """Construct the ros2 launch command for upstream localization with explicit arguments."""
    nav_share = Path(get_package_share_directory('turtlebot4_navigation'))
    launch_path = nav_share / 'launch' / 'localization.launch.py'
    if not launch_path.is_file():
        raise FileNotFoundError(f'Upstream localization launch file not found: {launch_path}')

    resolved_map = resolve_saved_map_path(map_path)
    resolved_params = Path(params_path) if params_path else default_localization_params_path()
    if not resolved_params.is_file():
        raise FileNotFoundError(f'Localization parameters file not found: {resolved_params}')

    normalized_ns = validate_namespace(namespace)
    sim_time_str = 'true' if use_sim_time else 'false'

    cmd = [
        'ros2', 'launch', 'turtlebot4_navigation', 'localization.launch.py',
        f'use_sim_time:={sim_time_str}',
        f'map:={resolved_map}',
        f'params:={resolved_params}',
    ]
    if normalized_ns:
        cmd.append(f'namespace:={normalized_ns}')
    return cmd


class SimulatorProbe(Node):
    """Temporary probe node to check simulator clock, LiDAR scan, and odometry."""

    def __init__(self, scan_topic: str, odom_topic: str):
        super().__init__('localization_sim_probe')
        self.has_clock = False
        self.has_scan = False
        self.has_odom = False

        self.clock_sub = self.create_subscription(
            Clock, '/clock', self._clock_cb, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self._scan_cb, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self._odom_cb, 10
        )

    def _clock_cb(self, _msg: Clock):
        self.has_clock = True

    def _scan_cb(self, _msg: LaserScan):
        self.has_scan = True

    def _odom_cb(self, _msg: Odometry):
        self.has_odom = True


def wait_for_simulator_readiness(
    timeout: float = 15.0,
    namespace: str = '',
) -> bool:
    """Verify that persistent base simulator is running (/clock, /scan, /odom)."""
    prefix = f'/{namespace.strip("/")}' if namespace.strip('/') else ''
    scan_topic = f'{prefix}/scan' if prefix else '/scan'
    odom_topic = f'{prefix}/odom' if prefix else '/odom'

    probe = SimulatorProbe(scan_topic, odom_topic)
    start_time = time.monotonic()
    try:
        while time.monotonic() - start_time < timeout:
            rclpy.spin_once(probe, timeout_sec=0.2)
            if probe.has_clock and probe.has_scan and probe.has_odom:
                return True
    finally:
        probe.destroy_node()

    missing = []
    if not probe.has_clock:
        missing.append('/clock')
    if not probe.has_scan:
        missing.append(scan_topic)
    if not probe.has_odom:
        missing.append(odom_topic)

    raise TimeoutError(
        f"Base simulator readiness check timed out ({timeout}s). Missing: {', '.join(missing)}. "
        "Ensure Terminal 1 persistent base is running: ros2 launch turtlebot4 turtlebot4.launch.py rviz:=true"
    )


def run_localization_process(cmd: list[str]) -> int:
    """Run localization launch process in foreground with signal forwarding for clean Ctrl+C."""
    print(f"[start_localization] Launching: {' '.join(cmd)}")
    proc = subprocess.Popen(cmd)

    def _sig_handler(signum, _frame):
        print("\n[start_localization] Received shutdown signal. Stopping Map Server & AMCL cleanly...")
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

    print("[start_localization] Localization stopped. Persistent simulator and RViz remain running.")
    return returncode


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Launch Map Server and AMCL localization independently for TurtleBot 4.'
    )
    parser.add_argument(
        '--map', default=None,
        help='Path to saved map YAML file (default: maps/map.yaml)'
    )
    parser.add_argument(
        '--params', default=None,
        help='Path to localization.yaml parameter file'
    )
    parser.add_argument(
        '--namespace', default='', help='Robot namespace (e.g. "robot")'
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

    # 1. Resolve and validate map file before touching ROS graph
    resolved_map = resolve_saved_map_path(args.map)
    print(f"[start_localization] Verified saved map: {resolved_map}")

    # 2. Check ROS graph preconditions: simulator ready, no SLAM, no duplicate localization
    rclpy.init()
    try:
        checker_node = Node('localization_precheck_node')
        try:
            # Allow DDS discovery time to populate graph node list
            names = []
            for _ in range(15):
                rclpy.spin_once(checker_node, timeout_sec=0.1)
                names = checker_node.get_node_names()
                if any(
                    name.strip('/').split('/')[-1] in (
                        'slam_toolbox', 'sync_slam_toolbox_node', 'async_slam_toolbox_node',
                        'amcl', 'amcl_node', 'map_server'
                    )
                    for name in names
                ):
                    break
                time.sleep(0.05)
            check_no_active_slam(names)
            check_no_duplicate_localization(names)
        finally:
            checker_node.destroy_node()

        if not args.skip_check:
            print(f"[start_localization] Checking base simulator readiness (timeout {args.timeout}s)...")
            wait_for_simulator_readiness(timeout=args.timeout, namespace=args.namespace)
            print("[start_localization] Base simulator verified (/clock, LiDAR, odometry active).")
    finally:
        rclpy.shutdown()

    # 3. Construct and run launch
    cmd = build_localization_launch_cmd(
        map_path=resolved_map,
        params_path=args.params,
        namespace=args.namespace,
        use_sim_time=True,
    )

    code = run_localization_process(cmd)
    sys.exit(code if code is not None else 0)


if __name__ == '__main__':
    main()
