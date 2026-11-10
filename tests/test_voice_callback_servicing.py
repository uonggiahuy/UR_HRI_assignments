"""Regression for the live delayed-worker failure; no Gazebo acceptance claims."""
import time
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan

from turtlebot4.voice_navigation_node import VoiceNavigationNode, VoiceProbe


@pytest.mark.parametrize('worker_delay', [0.0, 1.15])
def test_queued_scan_serviced_before_motion_guard(monkeypatch, worker_delay):
    """A real DDS clock callback must not trigger a guard ahead of queued scan.

    Live reproduction: worker delayed 1.15 s, clock callback age 31 us but scan
    callback age 1.160 s, despite independent scans arriving within 28.4 ms.
    """
    # Never publish synthetic measurements into a running robot graph.
    rclpy.init(args=['--ros-args', '-r', '/clock:=/a7_regression/clock',
                     '-r', '/scan:=/a7_regression/scan'])
    probe = VoiceProbe()
    assert probe.tf_listener_node is not None
    assert probe.listener.dedicated_listener_thread.is_alive()
    publisher = Node('voice_callback_regression_publisher')
    clock_pub = publisher.create_publisher(Clock, '/clock', 10)
    scan_pub = publisher.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + 3.0
        while scan_pub.get_subscription_count() < 1 or clock_pub.get_subscription_count() < 1:
            assert time.monotonic() < deadline, 'DDS endpoints did not match'
            rclpy.spin_once(publisher, timeout_sec=.01)
        old = LaserScan()
        old.header.stamp.sec = 584
        probe.record('scan', old)
        probe.received['scan'] = time.monotonic() - 1.160
        clock = Clock()
        clock.clock.sec = 586
        scan = LaserScan()
        scan.header.stamp.sec = 586
        # Queue the higher-rate clock first, matching the reproduced order.
        for _ in range(5):
            clock_pub.publish(clock)
            scan_pub.publish(scan)
        result = Future()
        checked = []

        def guard():
            checked.append(probe.timing_diagnostics())
            assert time.monotonic() - probe.received['scan'] < 1.0
            assert probe.messages['scan'].header.stamp.sec == 586
            result.set_result('ready')

        probe.guard = guard
        spin = rclpy.spin_once
        delayed = False

        def delayed_spin(*args, **kwargs):
            nonlocal delayed
            if not delayed:
                delayed = True
                time.sleep(worker_delay)
            return spin(*args, **kwargs)

        monkeypatch.setattr(rclpy, 'spin_once', delayed_spin)
        adapter = SimpleNamespace(probe=probe, check_cancel=lambda: None)
        assert VoiceNavigationNode.await_future(adapter, result, 2.0, guard=True) == 'ready'
        assert checked
        assert probe.service_diagnostics['spin_calls'] > 1
    finally:
        publisher.destroy_node()
        probe.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('wall_age, stamp_sec, stamp_nsec, reason', [
    (1.160, 98, 800000000, 'Stale scan measurement'),
    (2.6, 99, 990000000, 'Stale scan'),
])
def test_missing_scan_still_terminates_with_timing_evidence(
    wall_age, stamp_sec, stamp_nsec, reason
):
    """The unchanged one-second scan guard cannot be satisfied by fresh clock."""
    from collections import Counter, deque
    import json

    now = time.monotonic()
    errors = []
    stamp = SimpleNamespace(sec=stamp_sec, nanosec=stamp_nsec)
    probe = SimpleNamespace(
        wall_stall_timeout=2.5,
        docked=False, dock_received=now, hazards=[], hazard_received=now,
        navigation_status=[], owned_goals=[], get_node_names=lambda: [],
        get_publishers_info_by_topic=lambda _: [],
        messages={k: SimpleNamespace(header=SimpleNamespace(stamp=stamp))
                  for k in ('clock', 'scan', 'odom', 'global', 'local')},
        received={'clock': now, 'scan': now - wall_age, 'odom': now,
                  'global': now, 'local': now},
        clock_samples=[99., 99.2, 99.4, 99.6, 100.],
        last_clock_progress_wall=now, clock_consistent=True,
        callback_counts=Counter(scan=10, clock=100), scan_intervals=deque([.016]),
        service_diagnostics={'spin_calls': 50}, last_guard_wall=now - .05,
        get_logger=lambda: SimpleNamespace(error=errors.append),
    )
    probe.timing_diagnostics = lambda t: VoiceProbe.timing_diagnostics(probe, t)
    probe.stale = lambda reason, timing: VoiceProbe.stale(probe, reason, timing)
    with pytest.raises(RuntimeError, match=reason):
        VoiceProbe.guard(probe)
    evidence = json.loads(errors[0])['timing']
    assert evidence['callback_age_wall']['scan'] >= wall_age
    expected_stamp = stamp_sec + stamp_nsec * 1e-9
    assert evidence['message_stamp']['scan'] == expected_stamp
    assert evidence['simulation_clock'] == 100.
    assert evidence['scan_age_sim'] == pytest.approx(100. - expected_stamp)
    assert evidence['callback_age_wall']['clock'] < .1
    assert evidence['callback_age_wall']['odom'] < .1
    assert evidence['executor_service']['spin_calls'] == 50


@pytest.mark.parametrize('clock_age, progress_age, reason', [
    (2.047, 2.047, 'Localization uncertain or uninitialized'),
    (2.6, 2.6, 'Stale clock'),
    (.01, 2.6, 'Simulation clock not advancing'),
])
def test_simulation_pause_is_bounded_and_localization_remains_required(
    clock_age, progress_age, reason
):
    from collections import Counter, deque

    now = time.monotonic()
    stamp = SimpleNamespace(sec=99, nanosec=970000000)
    probe = SimpleNamespace(
        wall_stall_timeout=2.5,
        docked=False, dock_received=now - min(clock_age, 2.047),
        hazards=[], hazard_received=now - min(clock_age, 2.047),
        navigation_status=[], owned_goals=[], get_node_names=lambda: [],
        get_publishers_info_by_topic=lambda _: [],
        messages={k: SimpleNamespace(header=SimpleNamespace(stamp=stamp))
                  for k in ('clock', 'scan', 'odom', 'global', 'local')},
        received={k: now - clock_age for k in ('clock', 'scan', 'odom')},
        clock_samples=[100.] * 5,
        last_clock_progress_wall=now - progress_age, clock_consistent=True,
        callback_counts=Counter(), scan_intervals=deque(),
        service_diagnostics={}, last_guard_wall=None,
        get_logger=lambda: SimpleNamespace(error=lambda _: None),
    )
    # Measured global publications were ~2 s apart despite a 1 Hz setting.
    probe.received['global'] = probe.received['local'] = now - clock_age - 2.0
    probe.timing_diagnostics = lambda t: VoiceProbe.timing_diagnostics(probe, t)
    probe.stale = lambda reason, timing: VoiceProbe.stale(probe, reason, timing)
    with pytest.raises(RuntimeError, match=reason):
        VoiceProbe.guard(probe)


def test_tf_keeps_advancing_without_spinning_mission_probe():
    """Real TF delivery continues while the mission worker services no callbacks."""
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    from turtlebot4.navigation_diagnostics import NavigationProbe

    rclpy.init()
    probe = NavigationProbe(namespace='a7_tf_regression', independent_tf=True)
    publisher = Node('a7_tf_regression_publisher')
    pub = publisher.create_publisher(TFMessage, '/a7_tf_regression/tf', 10)
    thread = probe.listener.dedicated_listener_thread
    try:
        deadline = time.monotonic() + 3.0
        while pub.get_subscription_count() < 1:
            assert time.monotonic() < deadline
            rclpy.spin_once(publisher, timeout_sec=.01)
        for seconds in (100, 101):
            transform = TransformStamped()
            transform.header.frame_id = 'regression_odom'
            transform.child_frame_id = 'regression_base'
            transform.header.stamp.sec = seconds
            transform.transform.rotation.w = 1.
            pub.publish(TFMessage(transforms=[transform]))
            deadline = time.monotonic() + 2.0
            while True:
                if probe.buffer.can_transform('regression_odom', 'regression_base', rclpy.time.Time()):
                    tf = probe.buffer.lookup_transform('regression_odom', 'regression_base', rclpy.time.Time())
                    if tf.header.stamp.sec == seconds:
                        break
                assert time.monotonic() < deadline, 'TF stalled with mission worker idle'
                rclpy.spin_once(publisher, timeout_sec=.01)
        assert not probe.messages  # The mission probe itself has never spun.
    finally:
        probe.destroy_node()
        publisher.destroy_node()
        rclpy.shutdown()
    assert not thread.is_alive()
