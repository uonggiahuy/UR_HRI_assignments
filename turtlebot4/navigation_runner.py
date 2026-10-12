"""Foreground owner for navigation only; reuse existing AMCL and Map Server."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
import yaml

from turtlebot4.localization_runner import bounded_timeout, resolve_saved_map_path
from turtlebot4.navigation_diagnostics import NavigationProbe, check_graph, wait_for_readiness
from turtlebot4.simulation_launch import validate_namespace, process_snapshot, descendant_pids


def default_params_path():
    return Path(get_package_share_directory('turtlebot4')) / 'config/nav2.yaml'


def validate_parameters(path):
    config = yaml.safe_load(Path(path).read_text())
    planner = config['planner_server']['ros__parameters']['GridBased']
    costmap = config['global_costmap']['global_costmap']['ros__parameters']
    if (planner['allow_unknown'] is not False or costmap['track_unknown_space'] is not True
            or planner['tolerance'] != 0.0):
        raise ValueError('Require allow_unknown=false and track_unknown_space=true')
    return config


def build_navigation_launch_cmd(params_path=None, namespace=''):
    share = Path(get_package_share_directory('turtlebot4_navigation'))
    if not (share / 'launch/nav2.launch.py').is_file():
        raise FileNotFoundError('Installed TurtleBot Nav2 launch is missing')
    params = Path(params_path) if params_path else default_params_path()
    validate_parameters(params)
    cmd = ['ros2', 'launch', 'turtlebot4_navigation', 'nav2.launch.py',
           'use_sim_time:=true', f'params_file:={params.resolve()}']
    normalized = validate_namespace(namespace)
    if normalized:
        cmd.append(f'namespace:={normalized}')
    return cmd



def _terminated_process(pid):
    try:
        stat = Path(f'/proc/{pid}/stat').read_text()
        return stat[stat.rfind(')') + 2:].split()[0] == 'Z'
    except FileNotFoundError:
        return True

def stop_owned_process(process):
    """Bounded cleanup of the isolated launcher and its observed descendants."""
    snapshot = process_snapshot()
    parents = {pid: values[0] for pid, values in snapshot.items()}
    owned = {pid: snapshot[pid][1] for pid in descendant_pids(process.pid, parents)}
    if process.poll() is None:
        for sig, timeout in ((signal.SIGINT, 12), (signal.SIGTERM, 3), (signal.SIGKILL, 3)):
            try:
                os.killpg(process.pid, sig)
                process.wait(timeout=timeout)
                break
            except ProcessLookupError:
                break
            except subprocess.TimeoutExpired:
                continue
    # ROS launch children can have separate process groups. If the launcher had
    # to be killed, clean only recorded descendants with matching start times.
    for sig, timeout in ((signal.SIGTERM, 2.0), (signal.SIGKILL, 1.0)):
        current = process_snapshot()
        survivors = [pid for pid, started in owned.items()
                     if current.get(pid, (None, None))[1] == started
                     and not _terminated_process(pid)]
        for pid in survivors:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + timeout
        while survivors and time.monotonic() < deadline:
            time.sleep(0.05)
            current = process_snapshot()
            survivors = [pid for pid in survivors
                         if current.get(pid, (None, None))[1] == owned[pid]
                         and not _terminated_process(pid)]
        if not survivors:
            return
    if survivors:
        raise RuntimeError(f'Owned Nav2 descendants still present: {survivors}')


def parser():
    result = argparse.ArgumentParser(description='Start navigation only, using already-active localization.')
    result.add_argument('--namespace', default='')
    result.add_argument('--params', default=None)
    result.add_argument('--map', default=None)
    result.add_argument('--timeout', type=bounded_timeout, default=45.0)
    return result


def main(argv=None):
    args = parser().parse_args(argv)
    cmd = build_navigation_launch_cmd(args.params, args.namespace)
    map_path = resolve_saved_map_path(args.map)
    rclpy.init()
    probe = NavigationProbe(args.namespace)
    process = None
    handlers = {}
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            handlers[sig] = signal.signal(sig, interrupted)
        print('[start_navigation] Checking existing localization and sensors...', flush=True)
        wait_for_readiness(probe, map_path, args.timeout)
        names = [ns.rstrip('/') + '/' + name for name, ns in probe.get_node_names_and_namespaces()]
        check_graph(names)
        print('[start_navigation] Launching navigation only: ' + ' '.join(cmd), flush=True)
        process = subprocess.Popen(cmd, start_new_session=True)
        wait_for_readiness(probe, map_path, args.timeout, navigation=True, process=process)
        print('[start_navigation] READY: lifecycle ACTIVE, action and costmaps verified. Use RViz Nav2 Goal.', flush=True)
        # Retire readiness subscriptions before waiting; leave no idle reliable readers.
        probe.destroy_node()
        probe = None
        rclpy.shutdown()
        return process.wait()
    except KeyboardInterrupt:
        print('[start_navigation] Stopping owned navigation stack...', flush=True)
        return 0
    except (OSError, RuntimeError, ValueError, TimeoutError) as exc:
        print('[start_navigation] FAIL: ' + str(exc), file=sys.stderr, flush=True)
        return 1
    finally:
        # A second interrupt must not bypass owned-process cleanup.
        for sig in handlers:
            signal.signal(sig, signal.SIG_IGN)
        if process is not None:
            stop_owned_process(process)
        if probe is not None:
            probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
        print('[start_navigation] Navigation stopped; localization and base remain owned by their terminals.', flush=True)


if __name__ == '__main__':
    sys.exit(main())
