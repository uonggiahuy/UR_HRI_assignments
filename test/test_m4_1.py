"""Initial-pose contracts and fail-closed readiness; real ROS message classes."""
import argparse
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.srv import GetState
import pytest
import rclpy
from tf2_ros import TransformException
import yaml

from turtlebot4 import initial_pose as init
from turtlebot4.localization_diagnostics import validate_amcl_pose_msg
from turtlebot4.localization_runner import check_no_active_slam


@pytest.mark.parametrize('value', ['nan', 'inf', '-inf'])
def test_nonfinite_coordinates_rejected(value):
    with pytest.raises(argparse.ArgumentTypeError):
        init.finite(value)


def test_cli_requires_all_coordinates():
    with pytest.raises(SystemExit):
        init.parser().parse_args([])
    args = init.parser().parse_args(['--x', '-2', '--y', '3', '--yaw-deg', '180'])
    assert (args.x, args.y, args.yaw_deg, args.interface) == (-2, 3, 180, 'auto')


@pytest.mark.parametrize('angle', [-1080., -180., 0., 90., 180., 1080.])
def test_pose_units_frame_simulation_timestamp_and_covariance(angle):
    stamp = Time(sec=17, nanosec=123456789)
    msg = init.make_pose(-2.3, 4.5, angle, stamp)
    assert msg.header.frame_id == 'map' and msg.header.stamp == stamp
    assert msg.pose.pose.position.x == -2.3 and msg.pose.pose.position.y == 4.5
    q = msg.pose.pose.orientation
    assert q.x == q.y == 0.0 and math.hypot(q.z, q.w) == pytest.approx(1)
    yaw = 2 * math.atan2(q.z, q.w)
    assert math.cos(yaw) == pytest.approx(math.cos(math.radians(angle)))
    assert math.sin(yaw) == pytest.approx(math.sin(math.radians(angle)), abs=1e-12)
    assert len(msg.pose.covariance) == 36
    assert msg.pose.covariance[0] == msg.pose.covariance[7] == 0.25
    assert msg.pose.covariance[35] == pytest.approx(math.radians(15)**2)
    assert all(v == 0 for i, v in enumerate(msg.pose.covariance) if i not in (0, 7, 35))


def test_zero_clock_and_nonfinite_pose_rejected():
    with pytest.raises(ValueError, match='timestamp'):
        init.make_pose(0, 0, 0, Time())
    with pytest.raises(ValueError, match='finite'):
        init.make_pose(float('nan'), 0, 0, Time(sec=1))


def test_diagnostic_rejects_wrong_frame():
    msg = init.make_pose(0, 0, 0, Time(sec=1))
    msg.header.frame_id = 'odom'
    with pytest.raises(ValueError, match='frame_id'):
        validate_amcl_pose_msg(msg)


def test_deadline_never_spins_or_sends_request_when_expired(monkeypatch):
    node = Mock()
    monkeypatch.setattr(init.rclpy, 'spin_once', lambda *a, **k: pytest.fail('expired spin'))
    with pytest.raises(TimeoutError):
        init.wait_until(node, lambda: False, 0, 'service unavailable')
    with pytest.raises(TimeoutError):
        init.call(node, GetState, '/unavailable', GetState.Request(), 0)
    node.create_client.return_value.call_async.assert_not_called()
    node.destroy_client.assert_called_once()


def test_response_timeout_cleans_pending_request(monkeypatch):
    node = Mock()
    client = node.create_client.return_value
    future = client.call_async.return_value
    future.done.return_value = False
    calls = []
    def wait(node, predicate, deadline, description):
        calls.append(description)
        if len(calls) == 2:
            raise TimeoutError(description)
    monkeypatch.setattr(init, 'wait_until', wait)
    with pytest.raises(TimeoutError):
        init.call(node, GetState, '/slow', GetState.Request(), 1)
    client.remove_pending_request.assert_called_once_with(future)
    future.cancel.assert_called_once()
    node.destroy_client.assert_called_once_with(client)


def test_lifecycle_presence_is_not_active(monkeypatch):
    response = GetState.Response()
    response.current_state.id, response.current_state.label = 2, 'inactive'
    monkeypatch.setattr(init, 'call', lambda *a: response)
    with pytest.raises(RuntimeError, match='inactive'):
        init.lifecycle_active(Mock(), '/amcl', 1)
    response.current_state.id, response.current_state.label = 3, 'active'
    assert init.lifecycle_active(Mock(), '/amcl', 1) == 'active'


@pytest.mark.parametrize('fault', ['retained', 'no_new_message', 'stale_pose', 'stale_tf',
                                    'no_updates', 'disconnected', 'stale_sensors', 'none'])
def test_output_requires_new_pose_and_live_complete_tf(fault):
    pose = init.make_pose(1, 2, 90, Time(sec=20))
    transform = TransformStamped()
    transform.header.stamp = Time(sec=20)
    buffer = Mock()
    buffer.lookup_transform.return_value = transform
    node = SimpleNamespace(amcl_pose_count=2, amcl_pose_msg=pose, clock_time=20.,
                           map_tf_updates=4, tf_stamps={('map', 'odom'): 21.,
                                                        ('odom', 'base_link'): 20.},
                           sensors_ready=lambda: True, buffer=buffer)
    if fault == 'retained':
        pose.header.stamp = Time(sec=18)
    if fault == 'no_new_message':
        node.amcl_pose_count = 1
    if fault == 'stale_pose':
        node.clock_time = 25.
    if fault == 'stale_tf':
        node.tf_stamps[('map', 'odom')] = 17.
    if fault == 'no_updates':
        node.map_tf_updates = 3
    if fault == 'disconnected':
        buffer.lookup_transform.side_effect = TransformException('absent')
    if fault == 'stale_sensors':
        node.sensors_ready = lambda: False
    assert init.InitialPoseNode.output_ready(node, 19., 1, 2) == (fault == 'none')


def test_slam_mutual_exclusion():
    with pytest.raises(RuntimeError, match='Active SLAM'):
        check_no_active_slam(['amcl', '/tb/slam_toolbox'])


def test_rviz_baseline_and_camera_support_map_without_base_tf():
    config = yaml.safe_load((Path(__file__).parents[1] / 'rviz/simulation.rviz').read_text())
    manager = config['Visualization Manager']
    assert manager['Global Options']['Fixed Frame'] == 'odom'
    assert manager['Views']['Current']['Target Frame'] == '<Fixed Frame>'
    pose = next(t for t in manager['Tools'] if t['Class'].endswith('/SetInitialPose'))
    assert pose['Topic']['Value'] == 'initialpose'
    assert pose['Topic']['Reliability Policy'] == 'Reliable'


def test_actual_ros_service_unavailable_is_bounded():
    # Real rclpy client and transport; no running simulator required.
    rclpy.init()
    node = init.InitialPoseNode('m41_no_server')
    try:
        import time
        start = time.monotonic()
        with pytest.raises(TimeoutError, match='get_state'):
            init.call(node, GetState, '/m41_no_server/amcl/get_state', GetState.Request(), start + .2)
        assert time.monotonic() - start < 1.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('requested,service,expected', [
    ('auto', True, 'service'), ('auto', False, 'topic'),
    ('service', False, 'service'), ('topic', True, 'topic'),
])
def test_interface_selection_uses_discovered_types(requested, service, expected):
    node = Mock()
    node.get_service_names_and_types.return_value = [
        ('/tb/set_initial_pose', ['nav2_msgs/srv/SetInitialPose'])] if service else []
    node.get_subscriptions_info_by_topic.return_value = [SimpleNamespace(
        node_name='amcl', node_namespace='/tb',
        topic_type='geometry_msgs/msg/PoseWithCovarianceStamped')]
    assert init.choose_interface(node, '/tb', requested) == expected


@pytest.mark.parametrize('name,namespace,type_name', [
    ('observer', '/tb', 'geometry_msgs/msg/PoseWithCovarianceStamped'),
    ('amcl', '/other', 'geometry_msgs/msg/PoseWithCovarianceStamped'),
    ('amcl', '/tb', 'geometry_msgs/msg/PoseStamped'),
])
def test_fallback_rejects_unverified_subscriber(name, namespace, type_name):
    node = Mock()
    node.get_service_names_and_types.return_value = []
    node.get_subscriptions_info_by_topic.return_value = [SimpleNamespace(
        node_name=name, node_namespace=namespace, topic_type=type_name)]
    with pytest.raises(RuntimeError, match='verified AMCL subscriber'):
        init.choose_interface(node, '/tb', 'auto')


def test_new_scan_pose_may_precede_request_by_transport_latency():
    pose = init.make_pose(1, 2, 0, Time(sec=20, nanosec=990000000))
    transform = TransformStamped()
    transform.header.stamp = Time(sec=21)
    buffer = Mock()
    buffer.lookup_transform.return_value = transform
    node = SimpleNamespace(amcl_pose_count=2, amcl_pose_msg=pose, clock_time=21.1,
                           map_tf_updates=4, tf_stamps={('map', 'odom'): 22.,
                                                        ('odom', 'base_link'): 21.},
                           sensors_ready=lambda: True, buffer=buffer)
    assert init.InitialPoseNode.output_ready(node, 21., 1, 2)
