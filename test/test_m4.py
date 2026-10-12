"""M4 regressions, real node construction and isolated process cleanup.

Real warehouse goals are recorded separately; offline tests do not claim motion.
"""
from pathlib import Path
import signal
import subprocess
import time
from unittest.mock import Mock

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid, Path as RosPath
import pytest
import yaml

from turtlebot4 import navigation_diagnostics as diag
from turtlebot4 import navigation_runner as runner


def test_navigation_launch_arguments_and_resource_installation():
    cmd = runner.build_navigation_launch_cmd(namespace='/robot')
    assert cmd[:4] == ['ros2', 'launch', 'turtlebot4_navigation', 'nav2.launch.py']
    assert 'namespace:=robot' in cmd and 'use_sim_time:=true' in cmd
    assert f'params_file:={runner.default_params_path().resolve()}' in cmd
    assert not any('localization' in arg or 'gazebo' in arg for arg in cmd)
    assert runner.default_params_path().is_file()
    assert not any(arg.startswith('namespace:=') for arg in runner.build_navigation_launch_cmd())


@pytest.mark.parametrize('value', ['0', 'nan', 'inf', '121'])
def test_timeout_cli_rejects_unbounded_values(value):
    with pytest.raises(SystemExit):
        runner.parser().parse_args(['--timeout', value])


def test_diagnostics_cli():
    args = diag.parser().parse_args(['--json', '--timeout', '3', '--namespace', 'robot'])
    assert args.json and args.timeout == 3 and args.namespace == 'robot'


def test_upstream_managed_nodes_are_complete():
    nodes = diag.upstream_lifecycle_nodes()
    assert len(nodes) == len(set(nodes))
    assert 'velocity_smoother' in nodes and 'smoother_server' in nodes
    assert 'amcl' not in nodes and 'map_server' not in nodes


@pytest.mark.parametrize('names,error', [
    (['/amcl', '/map_server', '/slam_toolbox'], 'SLAM'),
    (['/amcl'], 'map_server'),
    (['/map_server'], 'amcl'),
    (['/amcl', '/amcl', '/map_server'], 'Duplicate'),
    (['/amcl', '/map_server', '/planner_server'], 'already running'),
    (['/amcl', '/map_server', '/turtlebot4_teleop'], 'teleop'),
])
def test_prerequisite_and_duplicate_failures(names, error):
    with pytest.raises(RuntimeError, match=error):
        diag.check_graph(names)


def test_safe_upstream_parameter_derivation():
    upstream = yaml.safe_load((Path(get_package_share_directory('turtlebot4_navigation')) / 'config/nav2.yaml').read_text())
    actual = runner.validate_parameters(runner.default_params_path())
    upstream['planner_server']['ros__parameters']['GridBased']['allow_unknown'] = False
    upstream['planner_server']['ros__parameters']['GridBased']['tolerance'] = 0.0
    assert actual == upstream
    assert actual['global_costmap']['global_costmap']['ros__parameters']['track_unknown_space'] is True


def test_unsafe_config_rejected(tmp_path):
    data = runner.validate_parameters(runner.default_params_path())
    data['planner_server']['ros__parameters']['GridBased']['allow_unknown'] = True
    path = tmp_path / 'unsafe.yaml'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='allow_unknown'):
        runner.build_navigation_launch_cmd(path)


def maps():
    grid = OccupancyGrid()
    grid.info.width, grid.info.height, grid.info.resolution = 6, 2, 1.0
    grid.info.origin.orientation.w = 1.0
    grid.data = [0] * 12
    costmap = Costmap()
    costmap.metadata.size_x, costmap.metadata.size_y, costmap.metadata.resolution = 6, 2, 1.0
    costmap.metadata.origin.orientation.w = 1.0
    costmap.data = [0] * 12
    return grid, costmap


def path():
    msg = RosPath()
    msg.header.frame_id = 'map'
    for x in (0.5, 5.5):
        pose = PoseStamped()
        pose.pose.position.x, pose.pose.position.y = x, 0.5
        msg.poses.append(pose)
    return msg


def test_sparse_path_segments_checked_for_unknown_space():
    grid, costmap = maps()
    assert diag.validate_path(path(), grid, costmap)['length_m'] == 5.0
    grid.data[3] = -1
    with pytest.raises(ValueError, match='unknown/occupied'):
        diag.validate_path(path(), grid, costmap)


def test_costmap_collision_and_unknown_path_rejected():
    grid, costmap = maps()
    for value in (253, 254, 255):
        costmap.data[3] = value
        with pytest.raises(ValueError, match='costmap'):
            diag.validate_path(path(), grid, costmap)
    assert diag.validate_costmap(costmap)['unknown'] == 1
    costmap.data = [0]
    with pytest.raises(ValueError, match='Invalid'):
        diag.validate_costmap(costmap)


def test_owned_shutdown_isolated_and_bounded(monkeypatch):
    process = Mock(pid=43210)
    process.poll.return_value = None
    process.wait.side_effect = [subprocess.TimeoutExpired('launch', 12), 0]
    send = Mock()
    monkeypatch.setattr(runner, 'process_snapshot', lambda: {})
    monkeypatch.setattr(runner.os, 'killpg', send)
    runner.stop_owned_process(process)
    assert send.call_args_list[0].args == (43210, signal.SIGINT)
    assert send.call_args_list[1].args == (43210, signal.SIGTERM)
    process.poll.return_value = 0
    send.reset_mock()
    runner.stop_owned_process(process)
    send.assert_not_called()


def test_readiness_early_child_failure():
    process = Mock(returncode=7)
    process.poll.return_value = 7
    with pytest.raises(RuntimeError, match='exited with code 7'):
        diag.wait_for_readiness(Mock(), Path('map.yaml'), 1, navigation=True, process=process)


def test_lifecycle_service_rejects_inactive():
    probe = Mock()
    probe.topic.side_effect = lambda name: '/' + name
    probe.request.return_value.current_state.id = 2
    probe.request.return_value.current_state.label = 'inactive'
    with pytest.raises(RuntimeError, match='inactive'):
        diag.NavigationProbe.lifecycle(probe, ['planner_server'], 1)


def test_rviz_navigation_resources_and_persistent_frame():
    file = Path(get_package_share_directory('turtlebot4')) / 'rviz/simulation.rviz'
    data = yaml.safe_load(file.read_text())['Visualization Manager']
    assert data['Global Options']['Fixed Frame'] == 'odom'
    assert 'nav2_rviz_plugins/GoalTool' in [tool['Class'] for tool in data['Tools']]
    names = {display['Name'] for display in data['Displays']}
    assert {'Global Path', 'Global Costmap', 'Local Costmap', 'AMCL Pose', 'RobotModel', 'Map'} <= names


def test_runner_retires_probe_before_foreground_wait(monkeypatch):
    events = []
    probe = Mock()
    probe.get_node_names_and_namespaces.return_value = [('amcl', '/'), ('map_server', '/')]
    probe.destroy_node.side_effect = lambda: events.append('destroy')
    process = Mock()
    process.poll.return_value = 0
    def wait():
        assert events == ['destroy', 'shutdown']
        return 0
    process.wait.side_effect = wait
    monkeypatch.setattr(runner, 'NavigationProbe', lambda namespace: probe)
    monkeypatch.setattr(runner, 'wait_for_readiness', Mock())
    monkeypatch.setattr(runner.subprocess, 'Popen', lambda *args, **kwargs: process)
    monkeypatch.setattr(runner.rclpy, 'init', Mock())
    monkeypatch.setattr(runner.rclpy, 'shutdown', lambda: events.append('shutdown'))
    monkeypatch.setattr(runner.rclpy, 'ok', lambda: False)
    monkeypatch.setattr(runner.signal, 'signal', Mock())
    assert runner.main([]) == 0
    assert events == ['destroy', 'shutdown']


def test_retained_costmap_cannot_mask_stale_updates():
    _, costmap = maps()
    costmap.metadata.update_time.sec = 10
    costmap.header.stamp.sec = 100
    with pytest.raises(ValueError, match='stale'):
        diag.validate_costmap(costmap, now=100)
    assert diag.validate_costmap(costmap, now=12)['free'] == 12


def test_service_clients_survive_readiness_polls(monkeypatch):
    probe = Mock(service_clients={})
    client = probe.create_client.return_value
    client.wait_for_service.return_value = True
    future = client.call_async.return_value
    future.done.return_value = True
    future.result.return_value = 'response'
    monkeypatch.setattr(diag.rclpy, 'spin_until_future_complete', Mock())
    for _ in range(2):
        assert diag.NavigationProbe.request(probe, object, '/state', object(), time.monotonic() + 10) == 'response'
    assert probe.create_client.call_count == 1
    probe.destroy_client.assert_not_called()


def test_real_probe_initializes_ros_node_and_namespace():
    import rclpy
    rclpy.init()
    probe = diag.NavigationProbe('robot')
    try:
        assert probe.topic('scan') == '/robot/scan'
        assert probe.get_namespace() == '/robot'
        assert probe.get_parameter('use_sim_time').value is True
        assert probe.service_clients == {}
    finally:
        probe.destroy_node()
        rclpy.shutdown()


def test_real_owned_cleanup_includes_separate_child_process_group():
    import os
    import sys
    child_code = "import signal,time;signal.signal(signal.SIGINT,signal.SIG_IGN);signal.signal(signal.SIGTERM,signal.SIG_IGN);time.sleep(60)"
    parent_code = "import subprocess,sys,time;subprocess.Popen([sys.executable,'-c',sys.argv[1]],start_new_session=True);time.sleep(60)"
    process = subprocess.Popen([sys.executable, '-c', parent_code, child_code],
                               start_new_session=True, stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL)
    child = None
    try:
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            snapshot = runner.process_snapshot()
            children = runner.descendant_pids(process.pid, {pid: value[0] for pid, value in snapshot.items()})
            if children:
                child = children[0]
                break
            time.sleep(0.02)
        assert child is not None
        runner.stop_owned_process(process)
        assert process.poll() is not None
        assert runner._terminated_process(child)
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3)
        if child is not None and not runner._terminated_process(child):
            os.kill(child, signal.SIGKILL)
