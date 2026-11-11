"""Synthetic classifiers/sensors: offline tests, never live acceptance evidence."""
import hashlib
import json
import math
from pathlib import Path
import time
from types import SimpleNamespace as NS
import pytest
from turtlebot4.emotion_logic import LABELS, EmotionFilter, primary_face
from turtlebot4.emotion_behavior_logic import EmotionBehavior, EmotionLimits


def stable(f, label, start=0., **kwargs):
    return [f.update(label, .9, start+i*.15, **kwargs) for i in range(4)][-1]


@pytest.mark.parametrize('label', LABELS)
def test_supported_labels_and_stability(label):
    f = EmotionFilter()
    stable(f, 'Neutral')
    r = stable(f, label, 1.)
    assert r['label'] == label
    assert r['event'] == (label != 'Neutral')  # sustained Neutral is suppressed too


@pytest.mark.parametrize('label,score', [('Invalid', .9), ('Happy', .1), ('Happy', math.nan), ('Disgust', .9)])
def test_unqualified_outputs_never_trigger(label, score):
    f = EmotionFilter()
    stable(f, 'Neutral')
    assert not any(f.update(label, score, 1+i*.15)['event'] for i in range(10))


def test_one_frame_cannot_trigger_and_neutral_required():
    f = EmotionFilter()
    assert not stable(f, 'Happy')['event']
    stable(f, 'Neutral', 1.)
    assert not f.update('Happy', .9, 2.)['event']
    assert any(f.update('Happy', .9, 2.1+i*.15)['event'] for i in range(4))
    assert not stable(f, 'Sad', 3.)['event']


def test_persistent_expression_cooldown_and_idle_neutral_rearm():
    f = EmotionFilter(cooldown=5.)
    stable(f, 'Neutral')
    assert stable(f, 'Happy', 1.)['event']
    assert not any(f.update('Happy', .9, 2+i*.1)['event'] for i in range(100))
    stable(f, 'Neutral', 12., idle=False)
    assert not stable(f, 'Sad', 13.)['event']
    stable(f, 'Neutral', 14.)
    assert stable(f, 'Sad', 15.)['event']
    assert stable(f, 'Neutral', 15.5)['event']  # stop ignores cooldown
    assert not stable(f, 'Happy', 16.)['event']


def test_no_face_timeout_disconnect_and_ambiguity():
    f = EmotionFilter()
    stable(f, 'Neutral')
    assert f.update('None', 0., .6)['control_ok']
    assert not f.update('None', 0., 1.3)['control_ok']
    assert not f.update('Happy', .9, 2., camera_ok=False)['control_ok']
    assert not f.update('Happy', .9, 3., primary=False)['control_ok']


def test_face_selection_never_changes_identity_in_multiple_scene():
    face = dict(box=[10, 10, 50, 60], emotions=dict(happy=.9, disgust=.1))
    assert primary_face([face])[:2] == ('Happy', .9)
    assert primary_face([]) is None
    assert primary_face([face, face]) is None
    assert primary_face([dict(box=face['box'], emotions=dict(disgust=.9, happy=.1))]) is None


def executing(label):
    b = EmotionBehavior()
    b.expression('Neutral', True, 0., True)
    b.expression(label, True, .1, True)
    assert b.state == 'EXECUTING'
    return b


@pytest.mark.parametrize('label,v,w', [('Happy', 1, 1), ('Sad', 1, -1), ('Angry', -1, 0), ('Surprise', 0, 1), ('Fear', 1, 0)])
def test_motion_mapping(label, v, w):
    b = executing(label)
    command = b.tick(.2, (0., 0., 0.), True)
    assert (int(math.copysign(1, command[0])) if command[0] else 0) == v
    assert (int(math.copysign(1, command[1])) if command[1] else 0) == w


@pytest.mark.parametrize('label', ['Fear', 'Angry', 'Happy', 'Sad', 'Surprise'])
def test_measured_completion_and_final_zero(label):
    b = executing(label)
    b.tick(.2, (0., 0., 0.), True)
    for i in range(1, 361):
        if label == 'Surprise':
            angle = 2*math.pi*i/360
            pose = (0., 0., math.atan2(math.sin(angle), math.cos(angle)))
        elif label in ('Happy', 'Sad'):
            angle = .5*i/360
            sign = 1 if label == 'Happy' else -1
            pose = (.5*math.sin(angle), sign*.5*(1-math.cos(angle)), sign*angle)
        else:
            pose = ((1 if label == 'Fear' else -1)*.10*i/360, 0., 0.)
        b.tick(.2+i*.07, pose, True)
    assert b.state == 'IDLE' and 'verified' in b.reason
    assert b.tick(30., pose, True) == (0., 0.)
    b.expression(label, True, 31., True)
    assert b.state == 'IDLE'  # controller itself also requires Neutral


@pytest.mark.parametrize('fault', ['timeout', 'unsafe', 'jump', 'neutral', 'shutdown'])
def test_every_motion_bounded_and_interruptible(fault):
    b = executing('Surprise')
    b.tick(.2, (0., 0., 3.1), True)
    b.tick(.3, (0., 0., -3.1), True)
    assert 0 < b.yaw_sum < .1
    if fault == 'neutral': b.expression('Neutral', True, .4, False)
    if fault == 'shutdown': b.abort('ROS shutdown')
    result = b.tick(50. if fault == 'timeout' else .5,
                    (5., 0., 0.) if fault == 'jump' else (0., 0., -3.1), fault != 'unsafe')
    assert result == (0., 0.) and b.state != 'EXECUTING'


@pytest.mark.parametrize('key,value', [('speed', 1.), ('distance', .5), ('segment_timeout', math.inf), ('angular_speed', .1)])
def test_unsafe_configuration(key, value):
    with pytest.raises(ValueError): EmotionLimits(**{key: value})


@pytest.fixture
def controller(monkeypatch):
    import rclpy
    from sensor_msgs.msg import LaserScan
    from turtlebot4.emotion_behavior_node import EmotionBehaviorNode
    rclpy.init()
    n = EmotionBehaviorNode()
    n.control_ok, n.docked, n.hazard = True, False, False
    n.pose, n.velocity, n.sim = (0., 0., 0.), (0., 0.), 10.
    n.seen = {k: time.monotonic() for k in ('odom', 'scan', 'hazard', 'dock', 'clock', 'signal')}
    scan = LaserScan()
    scan.angle_increment, scan.range_min, scan.range_max = 2*math.pi/639, .05, 10.
    scan.ranges = [5.]*640
    n.scan = scan
    def publishers(topic):
        name = {'/cmd_vel': n.get_name(), '/diffdrive_controller/cmd_vel_unstamped': 'motion_control',
                '/detected_emotion': 'emotion_recognition_node', '/assignment8/state': n.get_name()}[topic]
        return [NS(node_name=name, node_namespace='/')]
    monkeypatch.setattr(n, 'get_publishers_info_by_topic', publishers)
    monkeypatch.setattr(n, 'get_subscriptions_info_by_topic', lambda _: [NS(node_name='motion_control')])
    monkeypatch.setattr(n, 'get_node_names_and_namespaces', lambda: [])
    commands = []
    monkeypatch.setattr(n, 'command', lambda v, w, **kwargs: commands.append((v, w)))
    yield n, commands
    n.destroy_node()
    rclpy.shutdown()


@pytest.mark.parametrize('fault', ['odom', 'scan', 'hazard', 'dock', 'clock', 'signal', 'conflict', 'docked', 'obstacle', 'rear_unknown', 'nav2'])
def test_sensor_and_graph_safety(controller, monkeypatch, fault):
    n, commands = controller
    assert n.safe()
    n.behavior = executing('Angry')
    if fault in n.seen: n.seen[fault] -= 4
    elif fault == 'docked': n.docked = True
    elif fault == 'obstacle': n.scan.ranges[0] = .1
    elif fault == 'rear_unknown': n.scan.ranges[320] = math.nan
    elif fault == 'nav2': monkeypatch.setattr(n, 'get_node_names_and_namespaces', lambda: [('controller_server', '/')])
    else: monkeypatch.setattr(n, 'get_publishers_info_by_topic', lambda _: [NS(node_name='teleop', node_namespace='/')])
    assert not n.safe()
    n.tick()
    assert commands[-1] == (0., 0.) and n.behavior.state == 'STOPPED'


def packet(n, sequence, **changes):
    from std_msgs.msg import String
    d = dict(version=1, session='test', sequence=sequence, stamp=time.time(), label='Neutral',
             confidence=.9, event=True, control_ok=True, camera_ok=True, primary_face=True)
    d.update(changes)
    n.gesture(String(data=json.dumps(d)))


def test_protocol_exactly_once_neutral_cancellation(controller):
    n, commands = controller
    packet(n, 1)
    packet(n, 2, label='Happy')
    assert n.behavior.state == 'EXECUTING'
    packet(n, 2, label='Neutral')  # replay rejected
    assert n.behavior.state == 'EXECUTING'
    packet(n, 3)
    assert n.behavior.state == 'SETTLING' and commands[-1] == (0., 0.)


@pytest.mark.parametrize('changes', [dict(camera_ok=False), dict(primary_face=False, control_ok=False), dict(label='Disgust'), dict(stamp=0.), dict(confidence=.1), dict(control_ok=False)])
def test_invalid_or_lost_camera_stops(controller, changes):
    n, commands = controller
    packet(n, 1)
    packet(n, 2, label='Surprise')
    packet(n, 3, **changes)
    assert n.behavior.state == 'STOPPED' and commands[-1] == (0., 0.)


def test_stationary_confirmation(controller):
    n, commands = controller
    n.behavior.state = 'SETTLING'
    n.settling_started = time.monotonic()
    n.stationary_since = time.monotonic()-.4
    n.tick()
    assert n.behavior.state == 'IDLE' and 'stationary' in n.behavior.reason
    assert commands[-1] == (0., 0.)


def test_world_and_previous_entry_points_preserved():
    root = Path(__file__).resolve().parents[1]
    assert hashlib.sha256((root/'worlds/assignment6_open.sdf').read_bytes()).hexdigest() == 'b6a9bcb9f3de768b3f750b4c27cf8e2a7e9cf9b307b56054726bbae325461761'
    for name in ('simulation', 'turtlebot4', 'assignment5', 'assignment6', 'assignment7', 'assignment8'):
        assert (root/'launch'/f'{name}.launch.py').is_file()
    setup = (root/'setup.py').read_text()
    for name in ('start_slam', 'start_navigation', 'waypoint_navigator', 'gesture_recognition_node', 'robot_behavior_node', 'speech_recognition_node', 'voice_navigation_node', 'emotion_recognition_node', 'emotion_behavior_node'):
        assert name+' = ' in setup


def test_real_ros_expression_subscription_and_status_publication(controller):
    """Real DDS messages with synthetic classifier packet and mocked sensors."""
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String
    n, _ = controller
    peer = rclpy.create_node('assignment8_offline_protocol_test')
    publisher = peer.create_publisher(String, '/detected_emotion', 1)
    states = []
    peer.create_subscription(String, '/assignment8/state', lambda m: states.append(json.loads(m.data)), 1)
    executor = SingleThreadedExecutor()
    executor.add_node(n)
    executor.add_node(peer)
    deadline = time.monotonic()+3
    sequence = 0
    try:
        while time.monotonic() < deadline and (n.session != 'dds-test' or not states):
            sequence += 1
            publisher.publish(String(data=json.dumps(dict(version=1, session='dds-test', sequence=sequence,
                stamp=time.time(), label='Neutral', confidence=.9, event=True,
                control_ok=True, camera_ok=True, primary_face=True))))
            n.publish_state()
            executor.spin_once(timeout_sec=.05)
        assert n.session == 'dds-test' and n.sequence > 0
        assert states and 'state' in states[-1] and 'ready' in states[-1]
    finally:
        executor.shutdown()
        peer.destroy_node()


def test_fer_numpy_box_is_supported():
    import numpy as np
    result = primary_face([dict(box=np.array([10, 10, 50, 60]), emotions={'happy': .9})])
    assert result[0] == 'Happy'
    assert result[2] is not None and len(result[2]) == 4


def test_low_confidence_visible_face_is_not_a_missing_face_or_motion_event():
    f = EmotionFilter()
    stable(f, 'Neutral')
    for i in range(20):
        result = f.update('Sad', .3, 1+i*.15)
        assert not result['event'] and result['control_ok']
    # Retain neutral rearm while a visible face transitions through uncertain
    # expressions; confidence and stability still gate every motion event.
    assert stable(f, 'Sad', 5.)['event']
