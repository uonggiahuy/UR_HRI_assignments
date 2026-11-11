"""Expression protocol adapter over Assignment 6's independent safety controller."""
import json
import math
import signal
import time
import rclpy
from rclpy.signals import SignalHandlerOptions
from turtlebot4.robot_behavior_node import RobotBehavior
from turtlebot4.emotion_behavior_logic import EmotionBehavior, EmotionLimits
from turtlebot4.emotion_logic import LABELS


class EmotionBehaviorNode(RobotBehavior):
    NODE_NAME = 'emotion_behavior_node'
    SIGNAL_TOPIC = '/detected_emotion'
    STATE_TOPIC = '/assignment8/state'
    EXTRA_DEFAULTS = dict(distance=.10, radius=.50, segment_timeout=40., curve_distance=.25, rotation_target=2*math.pi, clearance=.40,
                          distance_tolerance=.005, angle_tolerance=.025,
                          confidence_threshold=.65, stop_tolerance=.01,
                          settling_duration=.3, settling_timeout=3.)

    def make_behavior(self):
        limits = {k: self.o[k] for k in EmotionLimits.__dataclass_fields__}
        if (not 0 < self.o['confidence_threshold'] <= 1
                or not .001 <= self.o['stop_tolerance'] <= .01
                or not .3 <= self.o['settling_duration'] <= 1.
                or not 2. <= self.o['settling_timeout'] <= 5.):
            raise ValueError('Invalid confidence threshold')
        return EmotionBehavior(EmotionLimits(**limits))

    def safe(self):
        if not super().safe():
            if self.readiness_reason == 'No live hand control signal':
                self.readiness_reason = 'No fresh validated facial-expression signal'
            elif self.readiness_reason == 'Need exactly one gesture publisher':
                self.readiness_reason = 'Need exactly one expression publisher'
            return False
        if (not all(math.isfinite(v) for v in self.velocity)
                or self.scan is None
                or not all(math.isfinite(v) for v in (self.scan.angle_min, self.scan.angle_max,
                           self.scan.angle_increment, self.scan.range_min, self.scan.range_max))
                or self.scan.angle_increment == 0 or self.scan.range_min < 0
                or self.scan.range_max <= self.scan.range_min):
            self.readiness_reason = 'Invalid sensor numeric values'
            return False
        if len(self.get_publishers_info_by_topic('/assignment8/state')) != 1:
            self.readiness_reason = 'Duplicate Assignment 8 controller'
            return False
        if any(name in ('robot_behavior_node', 'voice_navigation_node', 'waypoint_navigator',
                        'controller_server') for name, _ in self.get_node_names_and_namespaces()):
            self.readiness_reason = 'Another assignment/navigation controller is active'
            return False
        return True

    def gesture(self, msg):
        try:
            d = json.loads(msg.data)
            if (d['version'] != 1 or type(d['sequence']) is not int or d['sequence'] < 1
                    or not isinstance(d['session'], str) or not d['session']
                    or not math.isfinite(d['stamp'])
                    or not -.1 <= time.time()-d['stamp'] <= self.o['signal_timeout']
                    or d['label'] not in (*LABELS, 'None')
                    or any(type(d[k]) is not bool for k in ('event', 'control_ok', 'camera_ok', 'primary_face'))
                    or not math.isfinite(d['confidence']) or not 0 <= d['confidence'] <= 1):
                raise ValueError('Malformed/stale expression packet')
            if d['session'] != self.session:
                self.behavior.abort('Perception session changed')
                self.session, self.sequence = d['session'], -1
            if d['sequence'] <= self.sequence:
                return
            self.sequence = d['sequence']
            self.seen['signal'] = time.monotonic()
            self.control_ok = d['control_ok'] and d['camera_ok']
            if not d['primary_face'] and d['event']:
                raise ValueError('Movement event requires an unambiguous primary face')
            if not self.control_ok:
                self.behavior.abort('Camera/face lost or ambiguous')
            elif d['event']:
                if d['label'] == 'None' or d['confidence'] < self.o['confidence_threshold']:
                    raise ValueError('Unqualified expression event')
                ready = self.safe() and all(math.isfinite(v) and abs(v) < .02 for v in self.velocity)
                was_executing = self.behavior.state == 'EXECUTING'
                self.behavior.expression(d['label'], True, time.monotonic(), ready)
                if was_executing and d['label'] == 'Neutral':
                    self.behavior.state = 'SETTLING'
                    self.settling_started, self.stationary_since = time.monotonic(), None
            if self.behavior.state != 'EXECUTING':
                self.command(0., 0.)
        except (ValueError, KeyError, TypeError, OverflowError):
            self.control_ok = False
            self.behavior.abort('Invalid expression packet')
            self.command(0., 0.)

    def tick(self):
        neutral_cancel = (self.behavior.state == 'SETTLING'
                          and self.behavior.reason == 'Neutral stop')
        super().tick()
        if neutral_cancel and self.behavior.state == 'IDLE':
            self.behavior.reason = 'Neutral canceled; stationary stop verified'

    def command(self, linear, angular, force=False):
        # Ramp starts/changes; cancellation always sends zero immediately.
        now = time.monotonic()
        previous = getattr(self, '_last_command', (0., 0.))
        dt = min(.1, max(0., now-getattr(self, '_command_time', now-.05)))
        if linear or angular:
            linear = max(previous[0]-.10*dt, min(previous[0]+.10*dt, linear))
            angular = max(previous[1]-.50*dt, min(previous[1]+.50*dt, angular))
        self._command_time, self._last_command = now, (linear, angular)
        super().command(linear, angular, force)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    previous = signal.signal(signal.SIGTERM, interrupt)
    node = None
    try:
        node = EmotionBehaviorNode()
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
        signal.signal(signal.SIGTERM, previous)


if __name__ == '__main__':
    main()
