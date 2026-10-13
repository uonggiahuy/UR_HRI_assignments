"""M4.2 uncertainty, timing and coordinate regressions using native ROS messages."""
import math
from pathlib import Path
import importlib.util

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Path as RosPath, Odometry
from sensor_msgs.msg import LaserScan
import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

from turtlebot4.initial_pose import make_pose
from turtlebot4.localization_health import covariance_health, pose_time_health, transform_pose
from turtlebot4.navigation_diagnostics import (cell_index, path_costmap_overlap,
    classify_path_rejection, validate_costmap_freshness)


def pose(variance=.01, heading_variance=.01):
    msg = make_pose(0, 0, 0, Time(sec=10))
    msg.pose.covariance[0] = msg.pose.covariance[7] = variance
    msg.pose.covariance[35] = heading_variance
    return msg


@pytest.mark.parametrize('values', [(4.19021, 2.50007, .474), (.090001, .01, .01), (.01, .01, .069)])
def test_excessive_covariance_rejected_even_with_connected_tf(values):
    msg = pose()
    for i, v in zip((0, 7, 35), values):
        msg.pose.covariance[i] = v
    assert not covariance_health(msg)['acceptable']


def test_correlated_position_major_axis_and_invalid_psd():
    msg = pose(.08)
    msg.pose.covariance[1] = msg.pose.covariance[6] = .07
    assert not covariance_health(msg)['acceptable']
    msg.pose.covariance[1] = .20
    assert not covariance_health(msg)['positive_semidefinite']


def support(stamp=20):
    scan, odom = LaserScan(), Odometry()
    scan.header.stamp = odom.header.stamp = Time(sec=stamp)
    return scan, odom


def test_stationary_retained_pose_requires_current_support_and_agreement():
    scan, odom = support()
    assert pose_time_health(pose(), 20., scan, odom, (0, 0, 0), True)['acceptable']
    assert not pose_time_health(pose(), 20., scan, odom, (0, 0, 0))['acceptable']
    assert not pose_time_health(pose(), 20., scan, odom, (1, 0, 0), True)['acceptable']
    odom.twist.twist.linear.x = .1
    assert not pose_time_health(pose(), 20., scan, odom, (0, 0, 0), True)['acceptable']


@pytest.mark.parametrize('now,sensor_stamp', [(9., 9), (20., 17), (20., 21)])
def test_inconsistent_timestamps_rejected(now, sensor_stamp):
    scan, odom = support(sensor_stamp)
    assert not pose_time_health(pose(), now, scan, odom, (0, 0, 0), True)['acceptable']


def test_correct_world_to_map_rotation_translation_and_heading():
    assert transform_pose((2., 1., math.pi/4), (5., -3., math.pi/2)) == pytest.approx((4., -1., 3*math.pi/4))
    assert transform_pose((4., -1., 3*math.pi/4), (3., 5., -math.pi/2)) == pytest.approx((2., 1., math.pi/4))


def grid():
    m = Costmap(); m.header.frame_id = 'odom'; m.header.stamp.sec = 20
    m.metadata.size_x = m.metadata.size_y = 4; m.metadata.resolution = .5
    m.metadata.origin.position.x = 10.; m.metadata.origin.position.y = -2.
    m.metadata.origin.orientation.w = 1.; m.data = [0]*16
    return m


def test_costmap_origin_and_rotated_grid():
    m = grid()
    assert cell_index(10.75, -1.25, m) == 5
    m.metadata.origin.orientation.z = math.sin(math.pi/4)
    m.metadata.origin.orientation.w = math.cos(math.pi/4)
    assert cell_index(9.25, -1.25, m) == 5


def test_global_path_local_grid_overlap_requires_explicit_transform():
    m = grid(); path = RosPath(); path.header.frame_id = 'map'
    for x in (.25, .75, 5.):
        p = PoseStamped(); p.pose.position.x = x; p.pose.position.y = .25; path.poses.append(p)
    tf = TransformStamped(); tf.header.frame_id = 'odom'; tf.child_frame_id = 'map'
    tf.transform.translation.x = 10.; tf.transform.translation.y = -2.; tf.transform.rotation.w = 1.
    overlap = path_costmap_overlap(path, m, tf)
    assert (overlap['inside'], overlap['outside'], overlap['free']) == (2, 1, 2)
    m.data[0] = 254; m.data[1] = 255
    overlap = path_costmap_overlap(path, m, tf)
    assert classify_path_rejection(overlap) == 'OVERLAPPING_POINTS_BLOCKED_OR_UNKNOWN'
    assert classify_path_rejection(overlap, fresh=False) == 'STALE_COSTMAP_OR_TF'
    tf.child_frame_id = 'world'
    with pytest.raises(ValueError, match='frames'):
        path_costmap_overlap(path, m, tf)


@pytest.mark.parametrize('now', [24., 16.])
def test_stale_or_future_costmap_rejected(now):
    with pytest.raises(ValueError, match='stale'):
        validate_costmap_freshness(grid(), now)


def test_empty_and_nonoverlapping_plans_are_distinct():
    assert classify_path_rejection({'points':0}) == 'EMPTY_PLAN'
    assert classify_path_rejection({'points':3, 'inside':0}) == 'NO_GEOMETRIC_OVERLAP'


def test_rviz_qos_navigation_panel_and_unknown_protection():
    root = Path(__file__).resolve().parents[1]
    rviz = yaml.safe_load((root/'rviz/simulation.rviz').read_text())
    assert any(p['Class'] == 'nav2_rviz_plugins/Navigation 2' for p in rviz['Panels'])
    navigation = yaml.safe_load((root/'rviz/navigation.rviz').read_text())
    assert navigation['Visualization Manager']['Global Options']['Fixed Frame'] == 'map'
    navigation['Visualization Manager']['Global Options']['Fixed Frame'] = 'odom'
    assert navigation == rviz
    vm = rviz['Visualization Manager']
    assert any(t['Class']=='nav2_rviz_plugins/GoalTool' for t in vm['Tools'])
    topic = next(d['Topic'] for d in vm['Displays'] if d['Name']=='Map')
    assert topic['Durability Policy']=='Transient Local' and topic['Reliability Policy']=='Reliable'
    config = yaml.safe_load((root/'config/nav2.yaml').read_text())
    assert config['planner_server']['ros__parameters']['GridBased']['allow_unknown'] is False
    assert config['global_costmap']['global_costmap']['ros__parameters']['track_unknown_space'] is True


def test_persistent_base_exposes_map_frame_without_changing_m1_default():
    root = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location('persistent_base', root/'launch/turtlebot4.launch.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    description = module.generate_launch_description()
    arg = next(a for a in description.entities if isinstance(a, DeclareLaunchArgument) and a.name=='rviz_fixed_frame')
    assert ''.join(s.perform(LaunchContext()) for s in arg.default_value)=='odom'
    assert 'map' in arg.choices


def test_ground_truth_refuses_unknown_registration_and_wrong_frame():
    from turtlebot4.localization_health import aligned_truth, validate_registration
    with pytest.raises(ValueError, match='independently verified'):
        validate_registration(None)
    registration = {'verified':True, 'world_frame':'warehouse', 'map_frame':'map',
        'translation':[5.,-3.], 'yaw':math.pi/2, 'reference_uncertainty_m':.05,
        'world_points':[[0.,0.],[1.,0.],[0.,1.]], 'map_points':[[5.,-3.],[5.,-2.],[4.,-3.]]}
    msg = Odometry();msg.header.frame_id = 'warehouse';msg.pose.pose.orientation.w = 1.
    assert aligned_truth(msg, registration) == pytest.approx((5.,-3.,math.pi/2))
    msg.header.frame_id = 'odom'
    with pytest.raises(ValueError, match='frame'):
        aligned_truth(msg, registration)
    registration['map_points'][2] = [7.,-3.]
    with pytest.raises(ValueError, match='residual'):
        validate_registration(registration)


def test_native_transient_delivery_does_not_make_old_pose_fresh():
    """Transport on isolated test topic, never publish a fabricated /amcl_pose."""
    import time
    import uuid
    import json
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.qos import QoSProfile, DurabilityPolicy
    rclpy.init()
    node = rclpy.create_node('m4_2_transport_' + uuid.uuid4().hex[:8])
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    topic = '/m4_2_test/sample_' + uuid.uuid4().hex
    received=[]
    try:
        pub = node.create_publisher(PoseWithCovarianceStamped, topic, qos)
        pub.publish(pose())
        node.create_subscription(PoseWithCovarianceStamped, topic, received.append, qos)
        end = time.monotonic()+3
        while not received and time.monotonic()<end:
            rclpy.spin_once(node, timeout_sec=.05)
        assert received
        scan, odom = support()
        timing = pose_time_health(received[0], 20., scan, odom, (0,0,0))
        assert not timing['acceptable'] and timing['age_sec']==10.
        json.dumps(covariance_health(received[0]), allow_nan=False)
    finally:
        node.destroy_node();rclpy.shutdown()
