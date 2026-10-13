"""Regression checks for the public, reference-optional startup path."""

from pathlib import Path
from types import SimpleNamespace

import pytest

from turtlebot4 import prepare_navigation, teleop
from turtlebot4.localization_runner import resolve_saved_map_path


def test_map_selection_shared_and_validated(monkeypatch, tmp_path):
    default = resolve_saved_map_path()
    monkeypatch.setenv('TURTLEBOT4_MAP', default.name)
    assert resolve_saved_map_path() == default
    monkeypatch.setenv('TURTLEBOT4_MAP', str(default))
    assert resolve_saved_map_path() == default
    monkeypatch.setenv('TURTLEBOT4_MAP', 'missing.yaml')
    with pytest.raises(FileNotFoundError):
        resolve_saved_map_path()
    assert resolve_saved_map_path(default) == default


def test_prepare_requires_complete_explicit_pose():
    args = SimpleNamespace(x=1., y=None, yaw_deg=0.)
    with pytest.raises(ValueError, match='together'):
        prepare_navigation.pose_arguments(args)
    args.y = 2.
    assert prepare_navigation.pose_arguments(args)
    args.x = args.y = args.yaw_deg = None
    assert not prepare_navigation.pose_arguments(args)


def test_dock_subscription_matches_simulator_best_effort(monkeypatch):
    import rclpy
    rclpy.init()
    node = teleop.TeleopNode()
    try:
        assert node.dock_sub.qos_profile.reliability == teleop.ReliabilityPolicy.BEST_EFFORT
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('accepted,status,dock_after,expected', [
    (False, 0, True, False),
    (True, 6, True, False),
    (True, 4, True, False),
    (True, 4, False, True),
])
def test_undock_needs_accepted_succeeded_and_confirmed_status(
        monkeypatch, accepted, status, dock_after, expected):
    class Future:
        def __init__(self, value):
            self.value = value
        def done(self):
            return True
        def result(self):
            return self.value
    class Goal:
        def __init__(self):
            self.accepted = accepted
        def get_result_async(self):
            return Future(SimpleNamespace(status=status))
    class Client:
        def __init__(self, *_):
            pass
        def destroy(self):
            pass
        def wait_for_server(self, **_):
            return True
        def send_goal_async(self, _):
            return Future(Goal())
    node = SimpleNamespace(namespace='', dock_status_received=True, is_docked=True)
    monkeypatch.setattr(teleop, 'ActionClient', Client)
    monkeypatch.setattr(teleop.rclpy, 'spin_until_future_complete', lambda *_args, **_kwargs: None)
    def spin(_node, **_kwargs):
        node.is_docked = dock_after
    monkeypatch.setattr(teleop.rclpy, 'spin_once', spin)
    assert teleop.undock_robot(node, timeout=.02) is expected


def test_prepare_rejects_invalid_external_reference_before_ros(monkeypatch, tmp_path):
    bad = tmp_path / 'reference.json'
    bad.write_text('{}')
    monkeypatch.setattr(prepare_navigation.rclpy, 'init', lambda: pytest.fail('ROS must not start'))
    assert prepare_navigation.main(['--registration', str(bad)]) == 1


def test_no_repository_documentation_path_in_public_runtime():
    root = Path(__file__).resolve().parents[1]
    files = [root / 'README.md', root / 'setup.py', *(root / 'turtlebot4').glob('*.py'),
             *(root / 'launch').glob('*.py')]
    assert all('docs/' not in path.read_text() for path in files)
