"""Simulation-only gesture controller with independent wall-clock watchdogs."""
import json
import math
import signal
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from irobot_create_msgs.msg import DockStatus, HazardDetection, HazardDetectionVector

from turtlebot4.behavior_logic import Behavior, Limits
from turtlebot4.gesture_logic import LABELS


class RobotBehavior(Node):
    def __init__(self):
        super().__init__('robot_behavior_node')
        self.o = {k: self.declare_parameter(k, v).value for k, v in dict(
            cmd_topic='/cmd_vel', signal_timeout=.7, sensor_timeout=1.,
            speed=.06, angular_speed=.25, distance=.12, radius=.35,
            greeting_angle=.35, selection_timeout=8., segment_timeout=65.,
            position_tolerance=.06).items()}
        if not .1 <= self.o['signal_timeout'] <= 2. or not .1 <= self.o['sensor_timeout'] <= 2.:
            raise ValueError('Watchdog timeout outside .1–2 seconds')
        self.behavior = Behavior(Limits(**{k: self.o[k] for k in Limits.__dataclass_fields__}))
        self.seen = {}
        self.pose = None
        self.velocity = (0., 0.)
        self.sim = None
        self.docked = True
        self.hazard = True
        self.backup_limit = False
        self.scan = None
        self.control_ok = False
        self.readiness_reason = 'Waiting for sensors and webcam'
        self.session, self.sequence = None, -1
        self.last_state = None
        self.was_moving = False
        self.stop_until = -math.inf
        self.progress_pose = None
        self.progress_time = 0.
        self.settling_started = self.stationary_since = None
        self.publisher = self.create_publisher(Twist, self.o['cmd_topic'], qos_profile_sensor_data)
        self.status = self.create_publisher(String, '/assignment6/state', 1)
        self.create_subscription(String, '/detected_gesture', self.gesture, 1)
        for kind, topic, callback in [
            (Odometry, '/odom', self.odom), (LaserScan, '/scan', self.lidar),
            (ClockMessage, '/clock', self.clock), (DockStatus, '/dock_status', self.dock),
            (HazardDetectionVector, '/hazard_detection', self.hazards),
        ]:
            self.create_subscription(kind, topic, callback, qos_profile_sensor_data)
        # A ROS-time timer freezes when Gazebo pauses; this timer must not.
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.05, self.tick, clock=self.wall_clock)
        self.create_timer(.2, self.publish_state, clock=self.wall_clock)
        self.get_logger().info('Simulation controller: waiting for live webcam, undock and safe scan')

    def stamp(self, msg):
        stamp = msg.header.stamp
        return stamp.sec + stamp.nanosec/1e9

    def fresh_message(self, name, msg):
        if self.sim is not None and -.1 <= self.sim-self.stamp(msg) <= self.o['sensor_timeout']:
            self.seen[name] = time.monotonic()
            return True
        return False

    def clock(self, msg):
        value = msg.clock.sec + msg.clock.nanosec/1e9
        if self.sim is not None and value < self.sim:
            self.behavior.abort('Simulation clock reset')
            self.seen.clear()
        if self.sim is None or value > self.sim:
            self.seen['clock'] = time.monotonic()
        self.sim = value

    def odom(self, msg):
        if not self.fresh_message('odom', msg):
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = (p.x, p.y, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in values) or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) > .05:
            self.seen.pop('odom', None)
            return
        self.pose = (p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        self.velocity = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def lidar(self, msg):
        if self.fresh_message('scan', msg):
            self.scan = msg

    def dock(self, msg):
        if self.fresh_message('dock', msg):
            self.docked = msg.is_docked

    def hazards(self, msg):
        if self.fresh_message('hazard', msg):
            # Humble emits BACKUP_LIMIT at 5 cm remaining, before enforcing
            # zero reverse velocity at 0 cm. Let motion_control enforce its
            # buffer; the measured-progress watchdog below detects a stop.
            self.backup_limit = any(d.type == HazardDetection.BACKUP_LIMIT for d in msg.detections)
            self.hazard = any(d.type != HazardDetection.BACKUP_LIMIT for d in msg.detections)
            if self.hazard:
                self.behavior.abort('Robot hazard: '+str([d.type for d in msg.detections]))
                self.command(0., 0.)

    def safe(self):
        now = time.monotonic()
        if not self.control_ok:
            self.readiness_reason = 'No live hand control signal'
            return False
        if self.docked:
            self.readiness_reason = 'Docked: complete /undock first'
            return False
        if self.hazard or self.pose is None:
            self.readiness_reason = 'Robot hazard or missing odometry'
            return False
        stale = [k for k in ('odom', 'scan', 'hazard', 'dock', 'clock') if now-self.seen.get(k, -math.inf) > self.o['sensor_timeout']]
        if stale:
            self.readiness_reason = 'Stale inputs: '+', '.join(stale)
            return False
        if now-self.seen.get('signal', -math.inf) > self.o['signal_timeout']:
            self.readiness_reason = 'Gesture publisher/frame timeout'
            return False
        scan = self.scan
        if scan is None or len(scan.ranges) < 32 or abs(scan.angle_increment)*(len(scan.ranges)-1) < 2*math.pi-.05:
            self.readiness_reason = 'Full 360 degree LiDAR required'
            return False
        bound = self.behavior.clearance()
        # Unknown readings fail closed. +inf means no obstacle up to range_max.
        if not math.isfinite(scan.range_max) or scan.range_max < bound:
            self.readiness_reason = 'LiDAR range insufficient'
            return False
        if any(math.isnan(x) or x < scan.range_min or x <= bound for x in scan.ranges):
            self.readiness_reason = f'Clearance too small/unknown: need {bound:.2f} m'
            return False
        publishers = self.get_publishers_info_by_topic(self.o['cmd_topic'])
        own = [p for p in publishers if p.node_name == self.get_name() and p.node_namespace == '/']
        if len(own) != 1 or any(p.node_name not in (self.get_name(), 'cmd_vel_bridge') for p in publishers):
            self.readiness_reason = 'Conflicting /cmd_vel publishers'
            return False
        downstream = self.get_publishers_info_by_topic('/diffdrive_controller/cmd_vel_unstamped')
        if any(p.node_name != 'motion_control' for p in downstream):
            self.readiness_reason = 'Conflicting downstream velocity publisher'
            return False
        if len(self.get_publishers_info_by_topic('/detected_gesture')) != 1:
            self.readiness_reason = 'Need exactly one gesture publisher'
            return False
        ready = any(p.node_name == 'motion_control' for p in self.get_subscriptions_info_by_topic(self.o['cmd_topic']))
        self.readiness_reason = 'Ready' if ready else 'Robot motion_control subscriber missing'
        return ready

    def gesture(self, msg):
        try:
            data = json.loads(msg.data)
            if (data['version'] != 1 or type(data['sequence']) is not int
                    or not isinstance(data['session'], str) or not data['session']
                    or not math.isfinite(data['stamp']) or not -.1 <= time.time()-data['stamp'] <= self.o['signal_timeout']
                    or data['gesture'] not in (*LABELS, 'None')
                    or type(data['event']) is not bool or type(data['hand_present']) is not bool
                    or type(data['camera_ok']) is not bool
                    or not math.isfinite(data['confidence']) or not 0 <= data['confidence'] <= 1):
                raise ValueError('Malformed/stale control packet')
            if data['session'] != self.session:
                self.behavior.abort('Perception session changed')
                self.session, self.sequence = data['session'], -1
            if data['sequence'] <= self.sequence:
                return
            self.sequence = data['sequence']
            self.seen['signal'] = time.monotonic()
            self.control_ok = data['camera_ok'] and data['hand_present']
            label = data['gesture']
            # Raw open hand always wins over temporal Wave during execution.
            if self.behavior.state == 'EXECUTING' and (data.get('raw_gesture') == 'Open_Palm' or data.get('open_hand') is True):
                label = 'Open_Palm'
            if not self.control_ok:
                if self.behavior.state == 'EXECUTING':
                    self.behavior.abort('Webcam/hand signal lost')
            else:
                ready = self.safe() and max(abs(v) for v in self.velocity) < .02
                if self.behavior.state != 'SETTLING' or label in ('Open_Palm', 'Closed_Fist'):
                    self.behavior.gesture(label, data['event'], time.monotonic(), ready)
            if self.behavior.state != 'EXECUTING':
                self.command(0., 0.)
        except (ValueError, KeyError, TypeError, OverflowError):
            self.control_ok = False
            self.behavior.abort('Invalid control message')
            self.command(0., 0.)

    def command(self, linear, angular, force=False):
        # Do not preempt Create 3's /undock action with idle Twist messages.
        # An active gesture command is always stopped, even if docking changes.
        if not linear and not angular and not self.was_moving and time.monotonic() > self.stop_until and not force:
            return
        msg = Twist()
        msg.linear.x, msg.angular.z = float(linear), float(angular)
        self.publisher.publish(msg)
        if linear or angular:
            self.stop_until = time.monotonic()+.5
        self.was_moving = bool(linear or angular)

    def tick(self):
        now = time.monotonic()
        safe = self.safe()
        b = self.behavior
        if b.state == 'SETTLING':
            if not safe or now-self.settling_started > 3.:
                b.abort('Final stationary verification failed')
            elif max(abs(v) for v in self.velocity) < .01:
                if self.stationary_since is None:
                    self.stationary_since = now
                if now-self.stationary_since >= .3:
                    b.abort('Completed; odometry and stationary stop verified', idle=True)
            else:
                self.stationary_since = None
            self.command(0., 0.)
            return
        was_executing = b.state == 'EXECUTING'
        v, w = b.tick(now, self.pose, safe)
        if v or w:
            if self.progress_pose is None:
                self.progress_pose, self.progress_time = self.pose, now
            px, py, yaw = self.progress_pose
            angle = self.pose[2]-yaw
            if (math.hypot(self.pose[0]-px, self.pose[1]-py) >= .002
                    or abs(math.atan2(math.sin(angle), math.cos(angle))) >= .01):
                self.progress_pose, self.progress_time = self.pose, now
            elif now-self.progress_time > 1.:
                b.abort('No measured progress; upstream safety limit or stalled motion')
                v, w = 0., 0.
        else:
            self.progress_pose = None
        if was_executing and b.state == 'IDLE' and b.reason == 'Completed; odometry verified':
            b.state, b.reason = 'SETTLING', 'Checking measured stationary stop'
            self.settling_started, self.stationary_since = now, None
        self.command(v, w)

    def publish_state(self):
        b = self.behavior
        data = dict(state=b.state, selected=b.selected, reason=b.reason,
                    segment=b.index, ready=self.safe(), readiness_reason=self.readiness_reason,
                    backup_warning=self.backup_limit,
                    velocity=self.velocity)
        self.status.publish(String(data=json.dumps(data)))
        signature = (b.state, b.selected, b.reason)
        if signature != self.last_state:
            self.get_logger().info(json.dumps(data))
            self.last_state = signature


def main(args=None):
    # Keep the context alive through the final zero command on SIGINT.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    previous_term = signal.signal(signal.SIGTERM, interrupt)
    node = None
    try:
        node = RobotBehavior()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.behavior.abort('ROS shutdown')
            if rclpy.ok():
                for _ in range(3):
                    node.command(0., 0., force=True)
                node.publish_state()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGTERM, previous_term)


if __name__ == '__main__':
    main()
