"""Diagnostic regressions and the native TF2 lock regression (no robot motion)."""
from pathlib import Path
import os
import subprocess
import time
from unittest.mock import Mock

from ament_index_python.packages import get_package_prefix
from nav2_msgs.msg import Costmap
import pytest

from turtlebot4 import navigation_diagnostics as diag


def test_humble_raw_costmap_header_fallback_still_rejects_stale_data():
    msg = Costmap()
    msg.metadata.size_x = msg.metadata.size_y = 1
    msg.metadata.resolution = 0.05
    msg.data = [255]
    msg.header.stamp.sec = 100
    assert diag.validate_costmap(msg, 101)['unknown'] == 1
    with pytest.raises(ValueError, match='stale'):
        diag.validate_costmap(msg, 104)
    msg.metadata.update_time.sec = 90
    with pytest.raises(ValueError, match='stale'):
        diag.validate_costmap(msg, 100)


def test_raw_costmap_progress_uses_humble_header():
    probe = Mock(messages={}, received={}, costmap_samples={'global': []})
    msg = Costmap()
    msg.header.stamp.sec = 10
    diag.NavigationProbe.record(probe, 'global', msg)
    msg.header.stamp.sec = 11
    diag.NavigationProbe.record(probe, 'global', msg)
    assert probe.costmap_samples['global'] == [10, 11]


def test_motion_guard_does_not_recount_full_costmap(monkeypatch):
    msg = Costmap()
    msg.metadata.size_x = msg.metadata.size_y = 1
    msg.metadata.resolution = 0.05
    msg.data = [255]
    msg.header.stamp.sec = 100
    monkeypatch.setattr(diag, 'Counter', Mock(side_effect=AssertionError('Full grid recount')))
    diag.validate_costmap_freshness(msg, 101)
    with pytest.raises(ValueError, match='stale'):
        diag.validate_costmap_freshness(msg, 104)


def test_expired_budget_does_not_create_service_request():
    probe = Mock()
    with pytest.raises(TimeoutError, match='deadline exhausted'):
        diag.NavigationProbe.request(probe, object, '/state', object(), time.monotonic() - 1)
    probe.create_client.assert_not_called()


def test_service_timeout_removes_pending_future(monkeypatch):
    probe = Mock(service_clients={})
    client = probe.create_client.return_value
    client.wait_for_service.return_value = True
    future = client.call_async.return_value
    future.done.return_value = False
    monkeypatch.setattr(diag.rclpy, 'spin_until_future_complete', Mock())
    with pytest.raises(RuntimeError, match='Service timed out'):
        diag.NavigationProbe.request(probe, object, '/state', object(), time.monotonic() + 10)
    client.remove_pending_request.assert_called_once_with(future)
    future.cancel.assert_called_once()


def test_overall_timeout_preserves_actual_tf_failure(monkeypatch):
    probe = Mock()
    probe.localization.side_effect = [RuntimeError('Stale TF map -> odom'),
                                      TimeoutError('Readiness observation deadline exhausted')]
    monkeypatch.setattr(diag.rclpy, 'spin_once', Mock())
    with pytest.raises(TimeoutError, match='Stale TF map -> odom'):
        diag.wait_for_readiness(probe, Path('map.yaml'), 1)


def test_native_tf2_callback_can_queue_another_request(tmp_path):
    prefix = Path(get_package_prefix('tf2'))
    include_dirs = {prefix / 'include', prefix / 'include/tf2'}
    # Humble's message headers live beneath per-package include directories.
    include_dirs.update(Path(get_package_prefix(name)) / 'include' / name
                        for name in ('geometry_msgs', 'builtin_interfaces', 'std_msgs',
                                     'rosidl_runtime_cpp', 'rosidl_runtime_c',
                                     'rosidl_typesupport_interface', 'rcutils'))
    binary = tmp_path / 'tf2_callback_reentry'
    source = Path(__file__).with_name('tf2_callback_reentry.cpp')
    command = ['g++', '-std=c++17', str(source), '-o', str(binary)]
    command += ['-I' + str(path) for path in sorted(include_dirs)]
    command += ['-L' + str(prefix / 'lib'), '-ltf2']
    subprocess.run(command, check=True, timeout=30, capture_output=True, text=True)
    try:
        result = subprocess.run([str(binary)], check=True, timeout=3,
                                capture_output=True, text=True, env=os.environ.copy())
    except subprocess.TimeoutExpired:
        pytest.fail('Loaded TF2 deadlocks on callback reentry; source the pinned 0.25.24 overlay')
    assert 'callback reentry completed' in result.stdout
