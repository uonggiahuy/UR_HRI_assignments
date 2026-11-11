"""Finite odometry-driven expression motions; no camera or ROS dependencies."""
from dataclasses import dataclass
import math
from turtlebot4.behavior_logic import Behavior
from turtlebot4.emotion_logic import LABELS


@dataclass(frozen=True)
class EmotionLimits:
    speed: float = .06
    angular_speed: float = .25
    distance: float = .10
    radius: float = .50
    curve_distance: float = .25
    rotation_target: float = 2*math.pi
    segment_timeout: float = 40.
    clearance: float = .40
    distance_tolerance: float = .005
    angle_tolerance: float = .025

    def __post_init__(self):
        if not all(math.isfinite(v) and v > 0 for v in vars(self).values()):
            raise ValueError('Motion limits must be finite and positive')
        if (not .02 <= self.speed <= .10 or not .20 <= self.angular_speed <= .4
                or not .02 <= self.distance <= .12 or not .35 <= self.radius <= 1.
                or not .1 <= self.curve_distance <= .4
                or abs(self.rotation_target-2*math.pi) > .01
                or not 35 <= self.segment_timeout <= 60 or not .35 <= self.clearance <= 1.
                or self.distance_tolerance > .01 or self.angle_tolerance > .05):
            raise ValueError('Outside conservative simulation envelope')


class EmotionBehavior(Behavior):
    def __init__(self, limits=EmotionLimits()):
        super().__init__(limits)
        self.armed = False

    def abort(self, reason, idle=False):
        super().abort(reason, idle)
        self.armed = False

    def expression(self, label, event, now, ready):
        if label not in LABELS:
            self.abort('Unsupported expression')
        elif label == 'Neutral' and event:
            if self.state != 'SETTLING':
                self.abort('Neutral stop', idle=True)
                self.armed = ready
        elif event and self.armed and ready and self.state == 'IDLE':
            self.armed = False
            self.selected, self.state, self.reason = label, 'EXECUTING', 'Accepted once'
            self.origin = None
            self.segment_started = now

    def clearance(self):
        # Entire swept disk, including reverse, must be visible to 360° LiDAR.
        extent = self.limits.curve_distance if self.selected in ('Happy', 'Sad') else self.limits.distance if self.selected in ('Fear', 'Angry') else 0.
        return self.limits.clearance + extent

    def tick(self, now, pose, safe):
        if self.state != 'EXECUTING':
            return 0., 0.
        if not safe or pose is None or not all(math.isfinite(x) for x in pose):
            self.abort('Safety watchdog/obstacle')
            return 0., 0.
        if self.origin is None:
            self.origin = self.last_pose = pose
            self.travel = self.yaw_sum = 0.
        x, y, yaw = pose
        px, py, previous_yaw = self.last_pose
        step = math.hypot(x-px, y-py)
        dyaw = math.atan2(math.sin(yaw-previous_yaw), math.cos(yaw-previous_yaw))
        if step > .1 or abs(dyaw) > .3:
            self.abort('Odometry discontinuity')
            return 0., 0.
        self.travel += step
        self.yaw_sum += dyaw
        self.last_pose = pose
        l = self.limits
        if now-self.segment_started > l.segment_timeout:
            self.abort('Motion timeout; completion unverified')
            return 0., 0.
        label = self.selected
        ox, oy, oyaw = self.origin
        along = (x-ox)*math.cos(oyaw)+(y-oy)*math.sin(oyaw)
        lateral = -(x-ox)*math.sin(oyaw)+(y-oy)*math.cos(oyaw)
        complete = False
        if label == 'Surprise':
            if math.hypot(x-ox, y-oy) > .05:
                self.abort('Rotation drift')
                return 0., 0.
            complete = self.yaw_sum >= l.rotation_target-l.angle_tolerance
            command = (0., l.angular_speed)
        elif label in ('Fear', 'Angry'):
            direction = 1 if label == 'Fear' else -1
            if abs(lateral) > .04 or abs(self.yaw_sum) > .15:
                self.abort('Linear path deviation')
                return 0., 0.
            complete = along*direction >= l.distance-l.distance_tolerance
            command = (direction*l.speed, 0.)
        else:
            direction = 1 if label == 'Happy' else -1
            if self.yaw_sum*direction < -.08 or self.travel > l.curve_distance+.04:
                self.abort('Curve direction/path mismatch')
                return 0., 0.
            complete = (self.travel >= l.curve_distance-l.distance_tolerance
                        and self.yaw_sum*direction >= l.curve_distance/l.radius-.08)
            command = (l.speed, direction*l.speed/l.radius)
        if complete:
            self.abort('Completed; odometry verified', idle=True)
            return 0., 0.
        return command
