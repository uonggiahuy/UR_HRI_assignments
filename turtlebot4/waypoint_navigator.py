"""Bounded, fail-closed three-goal Nav2 mission with concurrent detector evidence."""
from collections import deque
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import signal
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from irobot_create_msgs.msg import DockStatus, HazardDetectionVector
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rcl_interfaces.msg import Log
import rclpy
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Empty
import numpy as np
from scipy.ndimage import distance_transform_edt
import yaml

from turtlebot4.diagnostics import seconds
from turtlebot4.localization_health import covariance_health
from turtlebot4.localization_runner import resolve_saved_map_path
from turtlebot4.navigation_diagnostics import (
    NavigationProbe, cell_index, validate_costmap_freshness, validate_path, wait_for_readiness,
)


def load_goals(path, map_path):
    config = yaml.safe_load(Path(path).read_text())
    goals = config.get('goals', [])
    if config.get('frame_id') != 'map' or len(goals) != 3:
        raise ValueError('Require exactly three map-frame goals')
    if len({item['name'] for item in goals}) != 3:
        raise ValueError('Goal names must be distinct')
    for goal in goals:
        if not all(isinstance(goal.get(key), (int, float)) and
                   not isinstance(goal[key], bool) and math.isfinite(goal[key])
                   for key in ('x', 'y', 'yaw')):
            raise ValueError('Goal coordinates must be finite numbers')
    for i, goal in enumerate(goals):
        for other in goals[:i]:
            if math.hypot(goal['x'] - other['x'], goal['y'] - other['y']) < .75:
                raise ValueError('Goal positions must be at least 0.75 m apart')
    if not .55 <= config.get('minimum_clearance', 0) <= 2.:
        raise ValueError('Goal clearance must be at least 0.55 m')
    meta = yaml.safe_load(Path(map_path).read_text())
    image = Path(map_path).parent / meta['image']
    hashes = {key: hashlib.sha256(file.read_bytes()).hexdigest()
              for key, file in [('yaml', Path(map_path)), ('pgm', image)]}
    if hashes != config.get('map_sha256'):
        raise ValueError('Goal configuration was measured on a different map')
    return config


def validate_goal_cells(config, grid, costmap):
    clearance = distance_transform_edt(
        np.asarray(grid.data).reshape(grid.info.height, grid.info.width) == 0) * grid.info.resolution
    margins = []
    for item in config['goals']:
        index = cell_index(item['x'], item['y'], grid)
        margin = float(clearance.flat[index])
        if grid.data[index] != 0 or margin < config['minimum_clearance']:
            raise ValueError(f"Goal {item['name']} is not clear known Free space")
        if costmap.data[cell_index(item['x'], item['y'], costmap)] >= 253:
            raise ValueError(f"Goal {item['name']} is blocked in the live costmap")
        margins.append(margin)
    return margins


def moving_sample(event, observations):
    if not observations:
        return None
    sample = min(observations, key=lambda item: abs(item[0] - event['stamp']))
    if abs(sample[0] - event['stamp']) > .15:
        return None
    return sample if abs(sample[1]) > .02 or abs(sample[2]) > .05 else None


class WaypointNavigator(NavigationProbe):
    def __init__(self):
        super().__init__()
        share = Path(get_package_share_directory('turtlebot4'))
        default_report = Path(os.environ.get('XDG_STATE_HOME', Path.home() / '.local/state')) / 'turtlebot4/assignment5/latest.json'
        self.goals_path = self.declare_parameter('goals_file', str(share / 'config/goals.yaml')).value
        self.map_path = resolve_saved_map_path(self.declare_parameter('map_file', '').value or None)
        self.ready_timeout = self.declare_parameter('ready_timeout', 60.).value
        self.goal_timeout = self.declare_parameter('goal_timeout', 120.).value
        self.report_path = Path(self.declare_parameter('report_file', str(default_report)).value).expanduser()
        if not 5 <= self.ready_timeout <= 180 or not 10 <= self.goal_timeout <= 300:
            raise ValueError('Readiness/goal timeout outside supported bounds')
        self.config = load_goals(self.goals_path, self.map_path)
        self.report = dict(status='RUNNING', goals=[], detections_during_motion=[],
                           map_sha256=self.config['map_sha256'], physical_accuracy='NOT VERIFIED')
        self.active_goal = None
        self.pending_goal = None
        self.planning_goal = None
        self.planning_pending = None
        self.odom_history = deque(maxlen=300)
        self.dock = None
        self.dock_received = 0.
        self.hazards = []
        self.hazard_received = 0.
        self.contacts = 0
        self.recoveries = 0
        self.dwb_failures = 0
        self.image_received = 0.
        self.image_stamp = 0.
        self.image_count = 0
        self.goal_name = None
        self.last_feedback = 0.
        self.events = self.create_publisher(String, '/assignment5/mission/status', 10)
        self.zero = self.create_publisher(Twist, '/cmd_vel', qos_profile_sensor_data)
        self.planner = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.create_subscription(DockStatus, '/dock_status', self.dock_cb, qos_profile_sensor_data)
        self.create_subscription(HazardDetectionVector, '/hazard_detection', self.hazard_cb, qos_profile_sensor_data)
        self.create_subscription(Contacts, '/bumper_contact', self.contact_cb, qos_profile_sensor_data)
        self.create_subscription(Image, '/assignment5/yolo/image_annotated', self.image_cb, qos_profile_sensor_data)
        self.create_subscription(String, '/assignment5/yolo/detections', self.detection_cb, 10)
        self.create_subscription(Log, '/rosout', self.log_cb, 100)

    def record(self, key, msg):
        super().record(key, msg)
        if key == 'odom' and hasattr(self, 'odom_history'):
            t = msg.twist.twist
            self.odom_history.append((seconds(msg.header.stamp), t.linear.x, t.angular.z))

    def dock_cb(self, msg):
        self.dock, self.dock_received = msg.is_docked, time.monotonic()

    def hazard_cb(self, msg):
        self.hazards = [item.type for item in msg.detections]
        self.hazard_received = time.monotonic()
        if self.goal_name:
            self.report.setdefault('hazard_types', []).extend(self.hazards)

    def contact_cb(self, msg):
        if self.goal_name:
            self.contacts += len(msg.contacts)

    def image_cb(self, msg):
        if msg.encoding != 'bgr8' or not msg.width or not msg.height or not msg.data:
            return
        self.image_received, self.image_stamp = time.monotonic(), seconds(msg.header.stamp)
        self.image_count += 1

    def detection_cb(self, msg):
        event = json.loads(msg.data)
        expected = {label for goal in self.config['goals'] for label in goal.get('expected_classes', [])}
        if self.goal_name and any(d['class'] in expected for d in event['detections']):
            motion = moving_sample(event, self.odom_history)
            if motion:
                event.update(goal=self.goal_name, odom_stamp=motion[0], linear=motion[1], angular=motion[2])
                self.report['detections_during_motion'].append(event)

    def log_cb(self, msg):
        if self.goal_name and msg.name.endswith('controller_server') and 'No valid trajectories' in msg.msg:
            self.dwb_failures += 1

    def status(self, state, **values):
        event = dict(state=state, goal=self.goal_name, stamp=self.clock_samples[-1] if self.clock_samples else 0., **values)
        self.events.publish(String(data=json.dumps(event)))
        self.get_logger().info(json.dumps(event))

    def await_future(self, future, timeout, guard=False):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.03)
            if guard:
                self.motion_guard()
        if not future.done() or future.result() is None:
            raise TimeoutError('Action response/result deadline exceeded')
        return future.result()

    def pose(self, item):
        result = PoseStamped()
        result.header.frame_id = 'map'
        result.header.stamp = self.messages['clock'].clock
        result.pose.position.x, result.pose.position.y = float(item['x']), float(item['y'])
        result.pose.orientation.z = math.sin(item['yaw'] / 2)
        result.pose.orientation.w = math.cos(item['yaw'] / 2)
        return result

    def check_publishers(self):
        allowed = {'cmd_vel_bridge', 'controller_server', 'velocity_smoother', 'behavior_server', self.get_name()}
        extra = [p.node_name for p in self.get_publishers_info_by_topic('/cmd_vel') if p.node_name not in allowed]
        if extra or 'turtlebot4_teleop' in self.get_node_names():
            raise RuntimeError(f'Competing velocity publisher/teleop: {extra}')

    def robot_and_detector_ready(self):
        self.check_publishers()
        if self.dock is not False or time.monotonic() - self.dock_received > 2.:
            raise RuntimeError('Robot must be verifiably undocked')
        if time.monotonic() - self.hazard_received > 2. or any(h in (1, 2, 3, 4) for h in self.hazards):
            raise RuntimeError('Robot hazards absent, stale or unsafe')
        if self.image_count < 2 or time.monotonic() - self.image_received > 2.:
            raise RuntimeError('Waiting for two current YOLO annotated frames')
        if not -.1 <= self.clock_samples[-1] - self.image_stamp <= 1.:
            raise RuntimeError('Annotated image measurement timestamp is stale')

    def motion_guard(self):
        now = self.clock_samples[-1]
        self.robot_and_detector_ready()
        for key in ('clock', 'scan', 'odom'):
            if time.monotonic() - self.received.get(key, 0.) > 1.:
                raise RuntimeError(f'{key} reception stalled')
        for key in ('scan', 'odom'):
            if not -.1 <= now - seconds(self.messages[key].header.stamp) <= 1.:
                raise RuntimeError(f'{key} measurement stale')
        transform = self.buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        if abs(now - seconds(transform.header.stamp)) > .5:
            raise RuntimeError('Moving localization TF is stale')
        if not covariance_health(self.messages['pose'])['acceptable']:
            raise RuntimeError('AMCL uncertainty exceeded the existing readiness bounds')
        for key in ('local', 'global'):
            validate_costmap_freshness(self.messages[key], now)
            footprint = key + '_footprint'
            if (time.monotonic() - self.received.get(footprint, 0.) > 3. or
                    abs(now - seconds(self.messages[footprint].header.stamp)) > 3.):
                raise RuntimeError(f'{key} footprint stale')
        if self.contacts or self.recoveries > 1 or self.dwb_failures > 1:
            raise RuntimeError('Collision or repeated controller/recovery failure')

    def plan(self, target, start=None):
        if not self.planner.wait_for_server(timeout_sec=3.):
            raise RuntimeError('ComputePathToPose unavailable')
        request = ComputePathToPose.Goal(goal=target, planner_id='GridBased')
        if start is not None:
            request.start, request.use_start = start, True
        self.planning_pending = self.planner.send_goal_async(request)
        self.planning_goal = self.await_future(self.planning_pending, 5.)
        if not self.planning_goal.accepted:
            raise RuntimeError('Global plan rejected')
        result = self.await_future(self.planning_goal.get_result_async(), 10.)
        self.planning_goal = self.planning_pending = None
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f'Global planning failed with status {result.status}')
        return validate_path(result.result.path, self.messages['map'], self.messages['global'])

    def feedback(self, msg):
        f = msg.feedback
        self.recoveries = max(self.recoveries, f.number_of_recoveries)
        if time.monotonic() - self.last_feedback >= 2.:
            p = f.current_pose.pose.position
            self.status('EXECUTING', position=[p.x, p.y], distance_remaining=f.distance_remaining,
                        elapsed=seconds(f.navigation_time), recoveries=f.number_of_recoveries)
            self.last_feedback = time.monotonic()

    def wait_stopped(self, timeout=5.):
        deadline = time.monotonic() + timeout
        beginning = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.03)
            msg = self.messages.get('odom')
            if msg is None or time.monotonic() - self.received['odom'] > 1.:
                beginning = None
                continue
            twist = msg.twist.twist
            if abs(twist.linear.x) < .01 and abs(twist.angular.z) < .02:
                beginning = beginning or time.monotonic()
                if time.monotonic() - beginning >= 1.:
                    return [twist.linear.x, twist.angular.z]
            else:
                beginning = None
        raise RuntimeError('Stopped odometry not verified for one second')

    def run(self):
        wait_for_readiness(self, self.map_path, self.ready_timeout, navigation=True)
        deadline = time.monotonic() + self.ready_timeout
        last_error = None
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.05)
            try:
                self.robot_and_detector_ready()
                break
            except RuntimeError as exc:
                last_error = exc
        else:
            raise TimeoutError(str(last_error))
        self.report['goal_clearance_m'] = validate_goal_cells(self.config, self.messages['map'], self.messages['global'])
        self.report['preflight_paths'] = []
        previous = None
        for item in self.config['goals']:
            pose = self.pose(item)
            self.report['preflight_paths'].append(self.plan(pose, previous))
            previous = pose
        self.status('READY', detector_frames=self.image_count)
        for item in self.config['goals']:
            wait_for_readiness(self, self.map_path, min(20., self.ready_timeout), navigation=True)
            self.robot_and_detector_ready()
            validate_goal_cells(self.config, self.messages['map'], self.messages['global'])
            planned = self.plan(self.pose(item))
            self.goal_name = item['name']
            self.recoveries, self.dwb_failures = 0, 0
            self.status('REQUESTED', target=[item['x'], item['y'], item['yaw']])
            self.pending_goal = self.action.send_goal_async(
                NavigateToPose.Goal(pose=self.pose(item)), feedback_callback=self.feedback)
            self.active_goal = self.await_future(self.pending_goal, 5.)
            if not self.active_goal.accepted:
                raise RuntimeError(f'Goal {self.goal_name} rejected')
            self.status('ACCEPTED')
            started = time.monotonic()
            result = self.await_future(self.active_goal.get_result_async(), self.goal_timeout, guard=True)
            if result.status != GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError(f'Goal {self.goal_name} failed, action status={result.status}')
            self.active_goal = self.pending_goal = None
            stopped = self.wait_stopped()
            # AMCL updates after a motion threshold; request one real scan while
            # stopped so a final sub-threshold displacement is not stale at handoff.
            self.request(Empty, '/request_nomotion_update', Empty.Request(), time.monotonic() + 3.)
            self.motion_guard()
            self.report['goals'].append(dict(name=self.goal_name, target=[item['x'], item['y'], item['yaw']],
                action_status=result.status, result='SUCCEEDED', seconds=time.monotonic() - started,
                recoveries=self.recoveries, preflight_path=planned, stopped_velocity=stopped))
            self.status('SUCCEEDED', recoveries=self.recoveries)
            self.save_report()
            self.goal_name = None
        self.report['stopped_velocity'] = self.wait_stopped()
        self.report['bumper_contacts'] = self.contacts
        self.report['status'] = 'PASS' if self.report['detections_during_motion'] else 'PARTIAL'
        self.status('MISSION_COMPLETE', result=self.report['status'],
                    motion_detection_frames=len(self.report['detections_during_motion']))
        self.save_report()
        return 0 if self.report['status'] == 'PASS' else 2

    def cancel_and_stop(self):
        # A response timeout can leave an accepted goal pending. Resolve it
        # before cancellation rather than silently abandoning server ownership.
        errors = []
        for pending, handle in [(self.pending_goal, self.active_goal),
                                (self.planning_pending, self.planning_goal)]:
            try:
                if handle is None and pending is not None:
                    handle = self.await_future(pending, 3.)
                if handle is not None and handle.accepted:
                    self.await_future(handle.cancel_goal_async(), 3.)
                    result = self.await_future(handle.get_result_async(), 5.)
                    if result.status not in (GoalStatus.STATUS_SUCCEEDED,
                                             GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED):
                        raise RuntimeError(f'Unexpected terminal action status: {result.status}')
                    self.report.setdefault('cleanup_action_results', []).append(result.status)
            except Exception as exc:
                self.get_logger().error(f'Cancellation could not be confirmed: {exc}')
                errors.append(str(exc))
        deadline = time.monotonic() + .5
        while rclpy.ok() and time.monotonic() < deadline:
            self.zero.publish(Twist())
            rclpy.spin_once(self, timeout_sec=.05)
        self.wait_stopped()
        self.report['shutdown_stopped'] = True
        self.save_report()
        if errors:
            raise RuntimeError('Cancellation not verified: ' + '; '.join(errors))

    def save_report(self):
        self.report_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.report_path.with_suffix('.tmp')
        temporary.write_text(json.dumps(self.report, indent=2) + '\n')
        temporary.replace(self.report_path)


def interrupt(_signum, _frame):
    raise KeyboardInterrupt()


def main(args=None):
    # Explicit ActionClients avoid Humble BasicNavigator's unbounded wait loops.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    node = None
    code = 1
    try:
        node = WaypointNavigator()
        code = node.run()
    except (Exception, KeyboardInterrupt) as exc:
        print(f'MISSION FAIL: {type(exc).__name__}: {exc}', file=sys.stderr)
        if node is not None:
            node.report.update(status='FAIL', error=f'{type(exc).__name__}: {exc}')
            node.status('FAILED', reason=str(exc))
            node.save_report()
        code = 1
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node is not None:
            try:
                node.cancel_and_stop()
            except (Exception, KeyboardInterrupt) as exc:
                node.get_logger().error(f'Stop verification failed: {exc}')
                node.report.update(status='FAIL', stop_error=str(exc))
                node.save_report()
                code = 1
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
