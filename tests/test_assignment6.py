"""Offline algorithm/safety regressions; these do not prove webcam acceptance."""
import math
from pathlib import Path

import pytest

from turtlebot4.behavior_logic import Behavior, Limits
from turtlebot4.gesture_logic import GestureFilter, LABELS


@pytest.mark.parametrize('label', LABELS[1:])
def test_six_labels_static_stability(label):
    f = GestureFilter()
    results = [f.update(label, .9, None, i*.05) for i in range(15)]
    assert sum(event for _, event in results) == 1
    assert results[-1][0] == label


def test_temporal_wave_requires_real_alternating_landmark_motion():
    f = GestureFilter(smoothing=1., stable_frames=1)
    xs = [.3, .35, .4, .5, .6, .5, .4, .3, .4, .5, .6, .5, .4, .3]
    labels = [f.update('Open_Palm', .95, x, i*.08)[0] for i, x in enumerate(xs)]
    assert 'Wave' in labels


@pytest.mark.parametrize('kind', ['stable', 'jitter', 'camera', 'one_direction', 'low_confidence'])
def test_wave_rejects_non_wave_motion(kind):
    f = GestureFilter(smoothing=1.)
    for i in range(35):
        x = .5 + (.004*math.sin(i) if kind == 'jitter' else .004*i if kind in ('camera', 'one_direction') else 0.)
        label, _ = f.update('Open_Palm', .2 if kind == 'low_confidence' else .95, x,
                            i*.04, .004 if kind == 'camera' else 0.)
        assert label != 'Wave'


def test_confidence_cooldown_rearm_and_lost_frames():
    f = GestureFilter(stable_frames=2, cooldown=.5, rearm_frames=2)
    assert f.update('Victory', .1, None, 0.) == ('None', False)
    assert not f.update('Victory', .9, None, .05)[1]
    assert f.update('Victory', .9, None, .1)[1]
    for t in (.2, .3, .4):
        assert not f.update('Victory', .9, None, t)[1]
    f.update('None', 0., None, .5)
    f.update('None', 0., None, .55)
    assert not f.update('Victory', .9, None, .6)[1]
    assert f.update('Victory', .9, None, .65)[1]
    assert not f.update('Thumb_Up', .9, None, 2.)[1]


def executing(label):
    b = Behavior()
    b.gesture(label, True, 0., True)
    assert b.state == 'PENDING_CONFIRMATION'
    assert b.tick(.01, (0., 0., 0.), True) == (0., 0.)
    b.gesture('Thumb_Up', True, .1, True)
    assert b.state == 'EXECUTING'
    return b


def test_selection_confirmation_expiry_and_readiness():
    b = Behavior()
    b.gesture('Thumb_Up', True, 0., True)
    assert b.state == 'IDLE'
    b.gesture('Victory', True, 0., True)
    b.gesture('Thumb_Up', True, 9., True)
    assert b.state == 'IDLE'
    b.gesture('Victory', True, 10., True)
    b.gesture('Thumb_Up', True, 10.1, False)
    assert b.state == 'STOPPED' and b.selected is None


def test_four_full_forward_backward_cycles_measured_by_odom():
    b = executing('Pointing_Up')
    assert len(b.plan) == 8
    x, now = 0., .2
    for segment in range(8):
        sign = 1 if segment % 2 == 0 else -1
        assert b.tick(now, (x, 0., 0.), True)[0] == sign*b.limits.speed
        for _ in range(6):
            now += .4
            x += sign*.02
            b.tick(now, (x, 0., 0.), True)
        assert b.index == segment+1
    assert abs(x) < 1e-9 and b.state == 'IDLE'
    assert b.reason == 'Completed; odometry verified'


def test_circle_requires_odometry_angle_path_and_closure():
    b = executing('Victory')
    b.tick(.2, (0., 0., 0.), True)
    assert b.tick(1., (0., 0., 0.), True)[0] > 0
    for i in range(1, 361):
        a = 2*math.pi*i/360
        b.tick(.2+i*.1, (.35*math.sin(a), .35*(1-math.cos(a)), math.atan2(math.sin(a), math.cos(a))), True)
    assert b.state == 'IDLE' and 'verified' in b.reason


def test_spinning_without_circle_translation_fails():
    b = executing('Victory')
    for i in range(361):
        a = 2*math.pi*i/360
        b.tick(.2+i*.1, (0., 0., math.atan2(math.sin(a), math.cos(a))), True)
    assert b.state == 'STOPPED' and 'Circle odometry' in b.reason


@pytest.mark.parametrize('label,state', [('Open_Palm', 'STOPPED'), ('Closed_Fist', 'IDLE')])
def test_stop_cancel_interrupt_even_without_event(label, state):
    b = executing('Pointing_Up')
    b.gesture(label, False, .2, False)
    assert b.state == state and b.selected is None
    assert b.tick(.3, (0., 0., 0.), True) == (0., 0.)


def test_failed_safety_guard_interrupts_motion():
    b = executing('Victory')
    assert b.tick(.2, (0., 0., 0.), True)[0] > 0
    assert b.tick(.3, (0., 0., 0.), False) == (0., 0.)
    assert b.state == 'STOPPED'


def test_duplicate_confirmation_does_not_restart_or_queue():
    b = executing('Pointing_Up')
    plan = b.plan
    b.gesture('Thumb_Up', True, .3, True)
    b.gesture('Victory', True, .4, True)
    assert b.plan is plan and b.selected == 'Pointing_Up'
    b.abort('ROS shutdown')
    assert b.tick(.5, (0., 0., 0.), True) == (0., 0.)


@pytest.mark.parametrize('setting,value', [('speed', .2), ('radius', .1), ('distance', .3), ('angular_speed', 2.), ('speed', float('nan'))])
def test_unsafe_limits_rejected(setting, value):
    with pytest.raises(ValueError):
        Limits(**{setting: value})


def test_odometry_jump_and_timeout_never_succeed():
    b = executing('Victory')
    b.tick(.2, (0., 0., 0.), True)
    assert b.tick(.3, (2., 0., 0.), True) == (0., 0.)
    assert b.state == 'STOPPED'
    b = executing('Victory')
    b.tick(.2, (0., 0., 0.), True)
    b.tick(70., (0., 0., 0.), True)
    assert b.state == 'STOPPED' and 'timeout' in b.reason


def test_earlier_assignment_interfaces_preserved():
    root = Path(__file__).resolve().parents[1]
    for name in ('simulation', 'turtlebot4', 'assignment5', 'assignment6'):
        assert (root/'launch'/f'{name}.launch.py').is_file()
    for name in ('goals.yaml', 'yolo.yaml', 'nav2.yaml', 'localization.yaml'):
        assert (root/'config'/name).is_file()


@pytest.fixture
def controller(monkeypatch):
    from types import SimpleNamespace as NS
    import rclpy
    from sensor_msgs.msg import LaserScan
    from turtlebot4.robot_behavior_node import RobotBehavior
    import time
    rclpy.init()
    node = RobotBehavior()
    node.control_ok, node.docked, node.hazard = True, False, False
    node.pose = (0., 0., 0.)
    node.sim = 10.
    node.seen = {k: time.monotonic() for k in ('odom', 'scan', 'hazard', 'dock', 'clock', 'signal')}
    scan = LaserScan()
    scan.angle_increment, scan.range_min, scan.range_max = float(2*math.pi/639), .05, 10.
    scan.ranges = [5.]*640
    node.scan = scan
    def publishers(topic):
        name = {'/cmd_vel': 'robot_behavior_node',
                '/diffdrive_controller/cmd_vel_unstamped': 'motion_control',
                '/detected_gesture': 'gesture_recognition_node'}[topic]
        return [NS(node_name=name, node_namespace='/')]
    monkeypatch.setattr(node, 'get_publishers_info_by_topic', publishers)
    monkeypatch.setattr(node, 'get_subscriptions_info_by_topic', lambda _: [NS(node_name='motion_control')])
    commands = []
    monkeypatch.setattr(node, 'command', lambda v, w: commands.append((v, w)))
    yield node, commands
    node.destroy_node()
    rclpy.shutdown()


@pytest.mark.parametrize('input_name', ['odom', 'scan', 'hazard', 'dock', 'clock', 'signal'])
def test_individual_wall_watchdogs(controller, input_name):
    import time
    node, commands = controller
    assert node.safe()
    node.behavior = executing('Pointing_Up')
    node.seen[input_name] = time.monotonic()-3.
    node.tick()
    assert commands[-1] == (0., 0.) and node.behavior.state == 'STOPPED'


@pytest.mark.parametrize('source', ['webcam', 'no_hand', 'invalid', 'raw_palm', 'cancel'])
def test_ros_control_packet_safety(controller, source):
    import json
    import time
    from std_msgs.msg import String
    node, commands = controller
    node.session = 'test'
    node.behavior = executing('Victory')
    data = dict(version=1, session='test', sequence=1, stamp=time.time(),
                gesture='None', confidence=.9, event=False, hand_present=True, camera_ok=True)
    if source == 'webcam': data['camera_ok'] = False
    if source == 'no_hand': data['hand_present'] = False
    if source == 'invalid': data['stamp'] = time.time()-10.
    if source == 'raw_palm': data.update(gesture='Wave', raw_gesture='Open_Palm')
    if source == 'cancel': data['gesture'] = 'Closed_Fist'
    node.gesture(String(data=json.dumps(data)))
    assert commands[-1] == (0., 0.)
    assert node.behavior.state == ('IDLE' if source == 'cancel' else 'STOPPED')


@pytest.mark.parametrize('fault', ['docked', 'hazard', 'obstacle', 'nan_scan', 'partial_scan', 'teleop', 'publisher_lost', 'duplicate_recognizer'])
def test_real_node_readiness_rejects_specific_faults(controller, monkeypatch, fault):
    from types import SimpleNamespace as NS
    node, _ = controller
    assert node.safe()
    original = node.get_publishers_info_by_topic
    if fault == 'docked': node.docked = True
    if fault == 'hazard': node.hazard = True
    if fault == 'obstacle': node.scan.ranges[0] = .1
    if fault == 'nan_scan': node.scan.ranges[0] = float('nan')
    if fault == 'partial_scan': node.scan.angle_increment = .001
    if fault in ('teleop', 'publisher_lost', 'duplicate_recognizer'):
        def publishers(topic):
            values = original(topic)
            if fault == 'teleop' and topic == '/cmd_vel':
                return values+[NS(node_name='teleop_twist_keyboard')]
            if topic == '/detected_gesture':
                if fault == 'publisher_lost': return []
                if fault == 'duplicate_recognizer': return values*2
            return values
        monkeypatch.setattr(node, 'get_publishers_info_by_topic', publishers)
    assert not node.safe()


def test_ros_packet_duplicate_cannot_renew_watchdog_or_reselect(controller):
    import json
    import time
    from std_msgs.msg import String
    node, _ = controller
    data = dict(version=1, session='test', sequence=1, stamp=time.time(),
                gesture='Victory', confidence=.9, event=True, hand_present=True, camera_ok=True)
    node.gesture(String(data=json.dumps(data)))
    assert node.behavior.state == 'PENDING_CONFIRMATION'
    seen = node.seen['signal']
    data['gesture'] = 'Pointing_Up'
    node.gesture(String(data=json.dumps(data)))
    assert node.seen['signal'] == seen and node.behavior.selected == 'Victory'


def test_backup_warning_defers_to_upstream_and_detects_measured_stop(controller):
    import time
    node, commands = controller
    node.behavior = executing('Pointing_Up')
    node.behavior.index = 1  # Actual reverse segment of the four-cycle plan.
    node.backup_limit = True
    node.tick()
    assert node.behavior.state == 'EXECUTING'
    assert commands[-1][0] < 0  # Warning is not the upstream zero-buffer limit.
    node.progress_time = time.monotonic()-1.1
    node.tick()
    assert node.behavior.state == 'STOPPED'
    assert commands[-1] == (0., 0.)


@pytest.mark.parametrize('moving,expected', [(True, 'SETTLING'), (False, 'IDLE')])
def test_final_completion_requires_measured_stop(controller, moving, expected):
    import time
    node, commands = controller
    node.behavior.state = 'SETTLING'
    node.settling_started = time.monotonic()-.5
    node.stationary_since = time.monotonic()-.4
    node.velocity = (.06 if moving else 0., 0.)
    node.tick()
    assert node.behavior.state == expected and commands[-1] == (0., 0.)


def test_greeting_completes_left_right_then_returns_to_heading():
    b = executing('Wave')
    yaw, now = 0., .2
    assert [angle for _, angle in b.plan] == [.35, -.7, .35]
    for _, angle in b.plan.copy():
        b.tick(now, (0., 0., yaw), True)
        steps = round(abs(angle)/.05)
        for _ in range(steps):
            yaw += math.copysign(.05, angle)
            now += .2
            b.tick(now, (0., 0., yaw), True)
    assert b.state == 'IDLE' and abs(yaw) < 1e-8 and b.index == 3


def test_launch_preflight_rejects_missing_camera_before_simulation(monkeypatch):
    import importlib.util
    from launch import LaunchContext
    path = Path(__file__).resolve().parents[1]/'launch/assignment6.launch.py'
    spec = importlib.util.spec_from_file_location('assignment6_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations['webcam_device'] = '987654'
    with pytest.raises(RuntimeError, match='Real webcam unavailable'):
        module.preflight_webcam(context)


@pytest.mark.parametrize('docked', [True, False])
def test_shutdown_zero_is_published_without_idle_interference(controller, monkeypatch, docked):
    from turtlebot4.robot_behavior_node import RobotBehavior
    node, _ = controller
    messages = []
    monkeypatch.setattr(node.publisher, 'publish', messages.append)
    node.docked = docked
    RobotBehavior.command(node, 0., 0.)
    assert messages == []  # Idle commands must not interfere with undocking.
    RobotBehavior.command(node, 0., 0., force=True)
    assert len(messages) == 1 and messages[0].linear.x == messages[0].angular.z == 0.


def test_own_motion_is_followed_by_bounded_repeated_stop(controller, monkeypatch):
    from turtlebot4.robot_behavior_node import RobotBehavior
    node, _ = controller
    messages = []
    monkeypatch.setattr(node.publisher, 'publish', messages.append)
    RobotBehavior.command(node, .06, 0.)
    RobotBehavior.command(node, 0., 0.)
    RobotBehavior.command(node, 0., 0.)
    assert len(messages) == 3
    node.stop_until = -math.inf
    RobotBehavior.command(node, 0., 0.)
    assert len(messages) == 3


def test_wave_survives_canned_classifier_flicker_with_real_open_hand_landmarks():
    f = GestureFilter(smoothing=1., stable_frames=1)
    xs = [.3, .35, .4, .5, .6, .5, .4, .3, .4, .5, .6, .5, .4, .3]
    results = [f.update('None', .1, x, i*.08, open_hand=True, hand_confidence=.95)
               for i, x in enumerate(xs)]
    assert any(label == 'Wave' and event for label, event in results)


def test_wave_accepts_small_natural_sweep_with_brief_landmark_dropout():
    f = GestureFilter()
    labels = []
    for i in range(45):
        now = i*.05
        if i in (12, 26):
            labels.append(f.update('None', 0., None, now)[0])
        else:
            x = .5+.065*math.sin(2*math.pi*1.6*now)
            labels.append(f.update('None', .1, x, now, open_hand=True, hand_confidence=.95)[0])
    assert 'Wave' in labels


def test_closed_hand_clears_wave_history_immediately():
    f = GestureFilter()
    for i in range(10):
        f.update('Open_Palm', .95, .5+i*.01, i*.05)
    assert f.samples
    f.update('Closed_Fist', .95, .6, .5)
    assert not f.samples and f.wave_progress() == 0


def test_wave_rejects_closed_hand_movement_and_unconfident_landmarks():
    for open_hand, confidence in ((False, .95), (True, .1)):
        f = GestureFilter(smoothing=1., stable_frames=1)
        for i in range(40):
            label, _ = f.update('None', .1, .5+.2*math.sin(i), i*.03,
                                open_hand=open_hand, hand_confidence=confidence)
            assert label != 'Wave'


def test_geometrically_open_hand_stops_execution_despite_wave_label(controller):
    import json
    import time
    from std_msgs.msg import String
    node, commands = controller
    node.session = 'test'
    node.behavior = executing('Victory')
    packet = dict(version=1, session='test', sequence=1, stamp=time.time(), gesture='Wave',
                  confidence=.9, event=True, camera_ok=True, hand_present=True, open_hand=True)
    node.gesture(String(data=json.dumps(packet)))
    assert node.behavior.state == 'STOPPED' and commands[-1] == (0., 0.)


def test_gazebo_process_detection_is_scoped_by_session(tmp_path):
    from turtlebot4.simulation_processes import gazebo_servers, SESSION_ENV
    for pid, name, session in ((10, 'ign gazebo server', 'ours'), (20, 'ign gazebo server', 'other'), (30, 'python3', 'ours')):
        path = tmp_path/str(pid)
        path.mkdir()
        (path/'cmdline').write_bytes(name.encode()+b'\0')
        (path/'environ').write_bytes(f'{SESSION_ENV}={session}'.encode()+b'\0')
    assert sorted(gazebo_servers(proc=tmp_path)) == [10, 20]
    assert gazebo_servers(proc=tmp_path, session='ours') == [10]


def test_lightweight_world_has_real_collisions_and_no_network_assets():
    import xml.etree.ElementTree as ET
    root = Path(__file__).resolve().parents[1]
    world = ET.parse(root/'worlds/assignment6_open.sdf').getroot().find('world')
    assert world.attrib['name'] == 'assignment6_open'
    assert len(world.findall('.//collision')) == 5
    assert not world.findall('.//uri')
