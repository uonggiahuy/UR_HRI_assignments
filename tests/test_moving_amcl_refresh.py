"""Regression for live AMCL age 2.013 s at oscillating 0.05263 rad/s."""
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty

from turtlebot4.localization_health import pose_time_health
from turtlebot4.voice_navigation_node import VoiceProbe


@pytest.mark.parametrize('age,angular,available,stationary,count', [
    (.9, .05263157884, True, False, 0),
    (1.1, .05263157884, True, False, 1),
    (1.1, 0., True, True, 1),
    (1.1, 0., True, False, 1),
    (1.1, .05263157884, False, False, 0),
])
def test_request_real_measurement_before_age_deadline_once_per_estimate(age, angular, available, stationary, count):
    pose=PoseWithCovarianceStamped(); pose.header.stamp.sec=100
    odom=Odometry(); odom.twist.twist.angular.z=angular
    calls=[]
    client=SimpleNamespace(service_is_ready=lambda:available,
                           call_async=lambda req:calls.append(req) or SimpleNamespace(done=lambda:True))
    probe=SimpleNamespace(messages={'pose':pose,'odom':odom},clock_samples=[100+age],
                          odom_observations=[(0.,stationary),(3.,True)],
                          service_clients={(Empty,'/request_nomotion_update'):client})
    VoiceProbe.maintain_active_pose(probe)
    VoiceProbe.maintain_active_pose(probe)
    assert len(calls)==count
    assert pose.header.stamp.sec==100  # Never re-stamp retained data.


def test_failed_refresh_still_rejects_recorded_moving_pose_age():
    pose=PoseWithCovarianceStamped();pose.header.stamp.sec=100
    scan=LaserScan();odom=Odometry()
    scan.header.stamp.sec=odom.header.stamp.sec=102
    odom.twist.twist.angular.z=.05263157884
    assert not pose_time_health(pose,102.013,scan,odom,(0.,0.,0.),False)['acceptable']
