"""Viewer buffering, GUI failure isolation and launch opt-out regressions."""
import importlib.util
from pathlib import Path
import subprocess
from types import SimpleNamespace as NS
from unittest.mock import Mock

from cv_bridge import CvBridgeError
from launch import LaunchContext
from launch.actions import RegisterEventHandler
from launch.events.process import ProcessExited
from launch_ros.actions import Node
import numpy as np
import pytest

from turtlebot4 import yolo_viewer as viewer


@pytest.mark.parametrize('shape,expected', [
    ((320, 240), (640, 480)), ((1280, 720), (640, 360)),
    ((240, 320), (360, 480)), ((1, 1), (480, 480)),
])
def test_aspect_ratio(shape, expected):
    assert viewer.display_size(*shape, 640, 480) == expected


@pytest.mark.parametrize('dims', [(0, 1, 640, 480), (1, 1, -1, 480)])
def test_invalid_dimensions(dims):
    with pytest.raises(ValueError):
        viewer.display_size(*dims)


def test_no_display_does_not_invoke_native_gui(monkeypatch):
    monkeypatch.delenv('DISPLAY', raising=False)
    probe = Mock()
    monkeypatch.setattr(viewer.subprocess, 'run', probe)
    assert 'DISPLAY' in viewer.gui_unavailable_reason()
    probe.assert_not_called()


@pytest.mark.parametrize('result', ['abort', 'timeout', 'missing', 'ok'])
def test_isolated_gui_probe(monkeypatch, result):
    monkeypatch.setenv('DISPLAY', ':99')
    run = Mock(return_value=NS(returncode=-6 if result == 'abort' else 0, stderr='X11 failed'))
    if result == 'timeout':
        run.side_effect = subprocess.TimeoutExpired('probe', 5)
    if result == 'missing':
        run.side_effect = OSError('unavailable')
    monkeypatch.setattr(viewer.subprocess, 'run', run)
    reason = viewer.gui_unavailable_reason()
    assert (reason is None) == (result == 'ok')
    assert run.call_args.kwargs['timeout'] == 5.


@pytest.fixture
def window(monkeypatch):
    gui = Mock(WINDOW_AUTOSIZE=1, WND_PROP_AUTOSIZE=1)
    gui.waitKey.return_value = -1
    gui.getWindowProperty.return_value = 1
    gui.resize.side_effect = lambda image, size: (image, size)
    monkeypatch.setattr(viewer, 'cv2', gui)
    bridge = Mock()
    bridge.imgmsg_to_cv2.return_value = np.zeros((240, 320, 3), dtype=np.uint8)
    return viewer.ImageWindow(bridge, 640, 480), gui, bridge


def test_only_latest_image_and_no_repeated_conversion(window):
    presenter, gui, bridge = window
    for value in range(100):
        presenter.receive(value)
    presenter.pump()
    bridge.imgmsg_to_cv2.assert_called_once_with(99, desired_encoding='bgr8')
    assert gui.imshow.call_args.args[1][1] == (640, 480)
    presenter.pump()
    assert bridge.imgmsg_to_cv2.call_count == 1
    assert gui.waitKey.call_count == 2
    presenter.close()
    gui.destroyAllWindows.assert_called_once()
    assert presenter.latest is None


@pytest.mark.parametrize('key,visible', [(ord('q'), 1), (27, 1), (-1, -1)])
def test_user_closes_only_viewer(window, key, visible):
    presenter, gui, _ = window
    presenter.receive(object())
    gui.waitKey.return_value = key
    gui.getWindowProperty.return_value = visible
    presenter.pump()
    assert presenter.closed


def test_malformed_frame_still_processes_gui_events(window):
    presenter, gui, bridge = window
    bridge.imgmsg_to_cv2.side_effect = CvBridgeError('bad encoding')
    presenter.receive(object())
    with pytest.raises(CvBridgeError):
        presenter.pump()
    gui.waitKey.assert_called_once()
    assert presenter.latest is None


def test_optional_viewer_launch_does_not_own_mission_shutdown():
    path = Path(__file__).resolve().parents[1] / 'launch/assignment5.launch.py'
    spec = importlib.util.spec_from_file_location('assignment5_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    entities = module.generate_launch_description().entities
    nodes = [entity for entity in entities if isinstance(entity, Node)]
    assert len(nodes) == 3
    optional = next(node for node in nodes if node.node_executable == 'yolo_viewer')
    context = LaunchContext()
    for value in ('true', 'false'):
        context.launch_configurations['show_yolo_window'] = value
        assert optional.condition.evaluate(context) == (value == 'true')
    for handler in [e.event_handler for e in entities if isinstance(e, RegisterEventHandler)]:
        assert not handler.matches(ProcessExited(action=optional, name="viewer", cmd=["viewer"], cwd=None, env=None, pid=1, returncode=0))


def test_gtk_window_not_visible_before_first_image(window):
    presenter, gui, _ = window
    gui.getWindowProperty.return_value = -1
    presenter.pump()
    assert not presenter.closed
    gui.getWindowProperty.assert_not_called()


def test_node_uses_depth_one_sensor_qos_and_steady_gui_clock(monkeypatch):
    from rclpy.clock import ClockType
    from rclpy.qos import ReliabilityPolicy, qos_profile_sensor_data
    monkeypatch.setattr(viewer, 'gui_unavailable_reason', lambda: None)
    window = Mock()
    monkeypatch.setattr(viewer, 'ImageWindow', lambda *args: window)
    viewer.rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = viewer.YoloViewer()
    try:
        assert node.open()
        subscription = next(s for s in node.subscriptions
                            if s.topic_name == '/assignment5/yolo/image_annotated')
        assert subscription.qos_profile.depth == 1
        assert subscription.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
        assert qos_profile_sensor_data.depth == 5
        assert any(t.clock.clock_type == ClockType.STEADY_TIME for t in node.timers)
        assert all(p.topic_name != '/cmd_vel' for p in node.publishers)
    finally:
        node.destroy_node()
        viewer.rclpy.shutdown()
