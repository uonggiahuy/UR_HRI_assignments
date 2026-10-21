"""Deterministic confirmation FSM and odometry-verified bounded motion."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Limits:
    speed: float = .06
    angular_speed: float = .25
    distance: float = .12
    radius: float = .35
    greeting_angle: float = .35
    selection_timeout: float = 8.
    segment_timeout: float = 65.
    position_tolerance: float = .06

    def __post_init__(self):
        values = vars(self)
        if not all(math.isfinite(x) and x > 0 for x in values.values()):
            raise ValueError('Limits must be finite and positive')
        if (self.speed > .12 or self.angular_speed > .5 or self.distance > .15
                or not .25 <= self.radius <= 1. or self.greeting_angle > .7
                or self.speed/self.radius > .5 or self.position_tolerance > .1
                or self.segment_timeout > 120 or self.selection_timeout > 30):
            raise ValueError('Motion exceeds conservative simulation envelope')


class Behavior:
    def __init__(self, limits=Limits()):
        self.limits = limits
        self.state, self.selected, self.reason = 'IDLE', None, 'Waiting for selection'
        self.deadline = 0.
        self.index = 0
        self.plan = []
        self.origin = None
        self.last_pose = None
        self.yaw_sum = self.travel = 0.
        self.segment_started = 0.

    def abort(self, reason, idle=False):
        self.state = 'IDLE' if idle else 'STOPPED'
        self.selected, self.plan, self.origin = None, [], None
        self.reason = reason

    def gesture(self, label, event, now, ready):
        # Safety gestures bypass debouncing and confirmation.
        if label == 'Open_Palm':
            self.abort('STOP: Open_Palm')
            return
        if label == 'Closed_Fist':
            self.abort('CANCEL: Closed_Fist', idle=True)
            return
        if self.state == 'PENDING_CONFIRMATION' and now > self.deadline:
            self.abort('Selection expired', idle=True)
        if not event or self.state == 'EXECUTING':
            return
        if label in ('Wave', 'Pointing_Up', 'Victory'):
            self.selected, self.state = label, 'PENDING_CONFIRMATION'
            self.deadline = now + self.limits.selection_timeout
            self.reason = 'WAITING FOR THUMBS UP'
        elif label == 'Thumb_Up' and self.state == 'PENDING_CONFIRMATION':
            if not ready:
                self.abort('Readiness/clearance check failed')
                return
            l = self.limits
            self.plan = ([(l.distance, 0.), (-l.distance, 0.)] * 4 if self.selected == 'Pointing_Up'
                         else [(0., l.greeting_angle), (0., -2*l.greeting_angle), (0., l.greeting_angle)]
                         if self.selected == 'Wave' else [(2*math.pi*l.radius, 2*math.pi)])
            self.index, self.origin = 0, None
            self.state, self.reason = 'EXECUTING', 'Confirmed once'

    def clearance(self):
        # Full circumscribed swept region: conservative, independent of scan yaw.
        extent = 2*self.limits.radius if self.selected == 'Victory' else self.limits.distance if self.selected == 'Pointing_Up' else 0.
        return .4 + extent

    def tick(self, now, pose, safe):
        if self.state == 'PENDING_CONFIRMATION' and now > self.deadline:
            self.abort('Selection expired', idle=True)
        if self.state != 'EXECUTING':
            return 0., 0.
        if not safe or pose is None or not all(math.isfinite(v) for v in pose):
            self.abort('Safety watchdog/obstacle')
            return 0., 0.
        if self.origin is None:
            self.origin = self.last_pose = pose
            self.yaw_sum = self.travel = 0.
            self.segment_started = now
        x, y, yaw = pose
        px, py, pyaw = self.last_pose
        step = math.hypot(x-px, y-py)
        dyaw = math.atan2(math.sin(yaw-pyaw), math.cos(yaw-pyaw))
        if step > .1 or abs(dyaw) > .3:
            self.abort('Odometry discontinuity')
            return 0., 0.
        self.travel += step
        self.yaw_sum += dyaw
        self.last_pose = pose
        if now - self.segment_started > self.limits.segment_timeout:
            self.abort('Motion timeout; completion unverified')
            return 0., 0.
        distance, angle = self.plan[self.index]
        ox, oy, oyaw = self.origin
        progress = (x-ox)*math.cos(oyaw) + (y-oy)*math.sin(oyaw)
        if self.selected == 'Victory':
            complete = self.yaw_sum >= angle - .025
            if complete and (math.hypot(x-ox, y-oy) > self.limits.position_tolerance
                             or self.travel < .85*distance):
                self.abort('Circle odometry failed closure/path-length check')
                return 0., 0.
        elif distance:
            if abs((x-ox)*math.sin(oyaw) - (y-oy)*math.cos(oyaw)) > .05:
                self.abort('Excessive cross-track deviation')
                return 0., 0.
            complete = progress * math.copysign(1, distance) >= abs(distance)-.005
        else:
            complete = self.yaw_sum * math.copysign(1, angle) >= abs(angle)-.015
        if complete:
            self.index += 1
            self.origin = None
            if self.index == len(self.plan):
                self.abort('Completed; odometry verified', idle=True)
            return 0., 0.
        if self.selected == 'Victory':
            return self.limits.speed, self.limits.speed/self.limits.radius
        return (math.copysign(self.limits.speed, distance), 0.) if distance else (0., math.copysign(self.limits.angular_speed, angle))
