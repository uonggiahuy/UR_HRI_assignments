"""Policy regressions for the live B cancellation and sparse AMCL updates."""
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped

from turtlebot4.localization_health import covariance_health, MAX_POSE_AGE_SEC
from turtlebot4.voice_navigation_node import VoiceProbe


@pytest.mark.parametrize('parameter,speed', [('update_min_d', .05), ('update_min_a', .05)])
def test_slow_approach_updates_before_moving_pose_expires(parameter, speed):
    """Humble AMCL only corrects after exceeding the motion threshold.

    Old thresholds took 5 s / 4 s at these normal approach speeds, despite
    a healthy 61 Hz LiDAR, exceeding the unchanged 2 s pose-age policy.
    """
    config = yaml.safe_load((Path(__file__).parents[1] / 'config/localization.yaml').read_text())
    threshold = config['amcl']['ros__parameters'][parameter]
    assert 0 < threshold / speed + 1 / 62 < MAX_POSE_AGE_SEC


def test_recorded_b_covariance_still_requires_safe_termination():
    """The fix must improve estimation, never admit the recorded unsafe pose."""
    pose = PoseWithCovarianceStamped()
    pose.pose.covariance[0] = .00040931911127017884
    pose.pose.covariance[1] = pose.pose.covariance[6] = .001308332516828159
    pose.pose.covariance[7] = .09699249331414705
    pose.pose.covariance[35] = .000053616041082863677
    health = covariance_health(pose)
    assert health['position_major_std_m'] == pytest.approx(.311465, abs=1e-6)
    assert not health['acceptable']

    # Exercise the real guard up to its covariance check; capture its evidence.
    import time
    now = time.monotonic()
    stamp = SimpleNamespace(sec=100, nanosec=0)
    from collections import Counter, deque
    errors = []
    probe = SimpleNamespace(
        wall_stall_timeout=2.5, docked=False, dock_received=now,
        hazards=[], hazard_received=now, navigation_status=[], owned_goals=[],
        get_node_names=lambda: [], get_publishers_info_by_topic=lambda _: [],
        messages={k: SimpleNamespace(header=SimpleNamespace(stamp=stamp))
                  for k in ('clock', 'scan', 'odom', 'global', 'local')},
        received={k: now for k in ('clock', 'scan', 'odom', 'global', 'local')},
        clock_samples=[99., 99.2, 99.4, 99.6, 100.],
        last_clock_progress_wall=now, clock_consistent=True,
        callback_counts=Counter(), scan_intervals=deque(),
        service_diagnostics={}, last_guard_wall=now,
        get_logger=lambda: SimpleNamespace(error=errors.append),
    )
    probe.messages['pose'] = pose
    probe.timing_diagnostics = lambda t: VoiceProbe.timing_diagnostics(probe, t)
    probe.stale = lambda reason, timing: VoiceProbe.stale(probe, reason, timing)
    with pytest.raises(RuntimeError, match='Localization uncertain'):
        VoiceProbe.guard(probe)
    import json
    evidence = json.loads(errors[-1])['timing']
    assert evidence['localization_covariance'] == health
    assert evidence['scan_age_sim'] == 0
