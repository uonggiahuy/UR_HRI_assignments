#!/usr/bin/env python3
"""M4.3 simulator-only evidence capture: one real A-to-B Nav2 goal.

Requires a safely undocked, localized Standard robot in the real saved warehouse
map, with Nav2 active. This script DOES execute simulation navigation actions.
It never publishes teleop velocity, fake TF or robot poses. Use only in simulation.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import time
import uuid

import numpy as np
from scipy.ndimage import distance_transform_edt
from geometry_msgs.msg import PoseStamped, Twist
from irobot_create_msgs.msg import DockStatus, HazardDetectionVector
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from rcl_interfaces.msg import Log
from action_msgs.srv import CancelGoal
from unique_identifier_msgs.msg import UUID

from turtlebot4.localization_health import (aligned_truth, validate_registration, validate_reference_for_map,
    require_reference_scope, covariance_health)
from turtlebot4.navigation_diagnostics import log_severity, path_overlap_at_costmap_time, NavigationProbe, cell_index, costmap_stamp, validate_path, validate_costmap_freshness, wait_for_readiness
from turtlebot4.localization_runner import resolve_saved_map_path


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def angle_error(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def destination_centers(path, origin):
    """Resolve exactly one B; A is measured immediately before execution."""
    if path is None:
        return [('A-to-B', (origin[0] - 1.5, origin[1]))]
    centers = [(item['name'], tuple(item['center'])) for item in json.loads(path.read_text())]
    if len(centers) != 1 or any(len(center) != 2 or not all(math.isfinite(v) for v in center) for _, center in centers):
        raise ValueError('Require exactly one finite two-dimensional destination B')
    return centers


def physical_arrival_accuracy(position_error, yaw_error, tolerances, registration):
    position_bound = position_error + registration['reference_uncertainty_m']
    yaw_bound = yaw_error + registration.get('reference_yaw_uncertainty_rad', 0.)
    return {'position_residual_m': position_error, 'position_error_bound_m': position_bound,
            'yaw_residual_rad': yaw_error, 'yaw_error_bound_rad': yaw_bound,
            'within_configured_tolerance_verified':
                position_bound <= tolerances['general_goal_checker.xy_goal_tolerance'] and
                yaw_bound <= tolerances['general_goal_checker.yaw_goal_tolerance']}


class GoalEvidence(NavigationProbe):
    def __init__(self, registration):
        validate_registration(registration)
        super().__init__()
        self.registration = registration
        self.truth = None
        self.docked = None
        self.hazards = []
        self.current_hazards = []
        self.commands = []
        self.paths = []
        self.bumper_contacts = 0
        self.observations = []
        self.path_errors = []
        self.controller_logs = []
        self.recovery_count = 0
        self.last_observation = 0.0
        self.pending_path_samples = []
        self.recording = False
        self.create_subscription(Odometry, '/sim_ground_truth_pose', self.truth_cb, qos_profile_sensor_data)
        self.create_subscription(DockStatus, '/dock_status', self.dock_cb, qos_profile_sensor_data)
        self.create_subscription(HazardDetectionVector, '/hazard_detection', self.hazard_cb, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.command_cb, qos_profile_sensor_data)
        self.create_subscription(Contacts, '/bumper_contact', self.contact_cb, qos_profile_sensor_data)
        self.create_subscription(Log, '/rosout', self.log_cb, 100)
        self.planner = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.active_goal = None

    def truth_cb(self, msg):
        self.truth = msg
        if self.recording and time.monotonic()-self.last_observation >= .1:
            self.observations.append(self.snapshot())
            self.last_observation = time.monotonic()

    def log_cb(self, msg):
        if self.recording and log_severity(msg.level) >= log_severity(Log.WARN) and any(name in msg.name for name in ('controller', 'dwb', 'behavior', 'bt_navigator')):
            self.controller_logs.append({'name': msg.name, 'level': log_severity(msg.level), 'message': msg.msg,
                                         'stamp': msg.stamp.sec + msg.stamp.nanosec/1e9})

    def feedback_cb(self, msg):
        self.recovery_count = max(self.recovery_count, msg.feedback.number_of_recoveries)

    def dock_cb(self, msg):
        self.docked = msg.is_docked

    def hazard_cb(self, msg):
        self.current_hazards = [hazard.type for hazard in msg.detections]
        if self.recording:
            self.hazards.extend(hazard.type for hazard in msg.detections)

    def contact_cb(self, msg):
        if self.recording:
            self.bumper_contacts += len(msg.contacts)

    def command_cb(self, msg):
        if self.recording:
            self.commands.append([msg.linear.x, msg.angular.z])

    def record(self, key, msg):
        super().record(key, msg)
        if key == 'path' and self.recording:
            try:
                stats = validate_path(msg, self.messages['map'], self.messages['global'])
                local = self.messages.get('local')
                if local is not None:
                    self.pending_path_samples.append((stats, msg, local, time.monotonic()))
                stats['points'] = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
                self.paths.append(stats)
            except ValueError as exc:
                self.path_errors.append(str(exc))

    def resolve_pending_paths(self):
        remaining = []
        for stats, path, local, received in self.pending_path_samples:
            overlap = path_overlap_at_costmap_time(self.buffer, path, local)
            if overlap is not None:
                stats['local_overlap'] = overlap
            elif time.monotonic()-received > 1.0:
                raise RuntimeError('Time-aligned path/costmap TF unavailable for one second')
            else:
                remaining.append((stats,path,local,received))
        self.pending_path_samples = remaining

    def snapshot(self):
        transform = self.buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        if self.recording:
            stamp = transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9
            if abs(self.clock_samples[-1] - stamp) > 0.5:
                raise RuntimeError(f'Moving localization TF is stale: age={self.clock_samples[-1] - stamp:.3f}s')
        tf = transform.transform
        truth = self.truth.pose.pose
        pose = [tf.translation.x, tf.translation.y, yaw(tf.rotation)]
        actual = aligned_truth(self.truth, self.registration)
        if abs(self.clock_samples[-1] - (self.truth.header.stamp.sec + self.truth.header.stamp.nanosec/1e9)) > .5:
            raise RuntimeError('Ground-truth measurement is stale')
        return {'time': self.clock_samples[-1], 'pose': pose, 'truth': actual,
                'monotonic': time.monotonic(),
                'tf_age': self.clock_samples[-1] - (transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9),
                'costmap_ages': {key: self.clock_samples[-1] - costmap_stamp(self.messages[key])
                                 for key in ('global', 'local') if key in self.messages},
                'footprint_ages': {key: self.clock_samples[-1] -
                                   (self.messages[key + '_footprint'].header.stamp.sec +
                                    self.messages[key + '_footprint'].header.stamp.nanosec / 1e9)
                                   for key in ('global', 'local') if key + '_footprint' in self.messages},
                'covariance': covariance_health(self.messages['pose']),
                'truth_world': [truth.position.x, truth.position.y, yaw(truth.orientation)],
                'localization_error_m': math.dist(pose[:2], actual[:2]),
                'localization_yaw_error': angle_error(pose[2], actual[2])}

    def await_future(self, future, timeout):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise TimeoutError('Action response timed out')
        return future.result()

    def pose_goal(self, x, y, heading):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.messages['clock'].clock
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(heading / 2), math.cos(heading / 2)
        return pose

    def plan(self, pose):
        goal = ComputePathToPose.Goal()
        goal.goal, goal.planner_id = pose, 'GridBased'
        handle = self.await_future(self.planner.send_goal_async(goal), 5)
        if not handle.accepted:
            return None, 0
        result = self.await_future(handle.get_result_async(), 10)
        return result.result.path, result.status

    def select_goal(self, desired):
        grid, costmap = self.messages['map'], self.messages['global']
        data = np.array(grid.data).reshape(grid.info.height, grid.info.width)
        clearance = distance_transform_edt(data == 0) * grid.info.resolution
        candidates = []
        # Search only the requested 2m square, rather than starving TF callbacks
        # while visiting every Free cell of the warehouse.
        origin, res = grid.info.origin.position, grid.info.resolution
        x0 = max(0, math.floor((desired[0]-1-origin.x)/res))
        x1 = min(grid.info.width, math.ceil((desired[0]+1-origin.x)/res))
        y0 = max(0, math.floor((desired[1]-1-origin.y)/res))
        y1 = min(grid.info.height, math.ceil((desired[1]+1-origin.y)/res))
        for ry, rx in zip(*np.where(clearance[y0:y1,x0:x1] >= 0.55)):
            iy, ix = int(ry)+y0, int(rx)+x0
            x = grid.info.origin.position.x + (ix + 0.5) * grid.info.resolution
            y = grid.info.origin.position.y + (iy + 0.5) * grid.info.resolution
            if math.dist((x, y), desired) > 1.0:
                continue
            try:
                require_reference_scope(x, y, self.registration)
            except ValueError:
                continue
            if costmap.data[cell_index(x, y, costmap)] > 100:
                continue
            candidates.append((math.dist((x, y), desired), x, y, clearance[iy, ix]))
        for _, x, y, margin in sorted(candidates)[:12]:
            start = self.snapshot()['pose']
            pose = self.pose_goal(x, y, math.atan2(y - start[1], x - start[0]))
            plan, status = self.plan(pose)
            if status == 4 and plan and len(plan.poses) > 1:
                stats = validate_path(plan, grid, costmap)
                for point in plan.poses:
                    require_reference_scope(point.pose.position.x, point.pose.position.y, self.registration)
                stats['points'] = [[p.pose.position.x,p.pose.position.y] for p in plan.poses]
                stats['direct_distance_m'] = math.dist(start[:2], (x,y))
                stats['detour_ratio'] = stats['length_m']/stats['direct_distance_m']
                samples = max(2, math.ceil(stats['direct_distance_m']/(grid.info.resolution/2)))
                direct = [(start[0]+(x-start[0])*t, start[1]+(y-start[1])*t) for t in np.linspace(0,1,samples)]
                stats['direct_nonfree_cells'] = sum(grid.data[cell_index(px,py,grid)] != 0 for px,py in direct)
                stats['direct_blocked_costmap_cells'] = sum(costmap.data[cell_index(px,py,costmap)] >= 253 for px,py in direct)
                if stats['direct_nonfree_cells'] == 0 and stats['direct_blocked_costmap_cells'] == 0 and stats['detour_ratio'] > 1.3:
                    raise RuntimeError('Unexplained detour on an unobstructed direct route')
                if stats['length_m'] > 1.0:
                    return pose, float(margin), stats
        raise RuntimeError(f'No safe reachable mapped goal near {desired}')

    def unsafe_plan_checks(self):
        grid, costmap = self.messages['map'], self.messages['global']
        results = []
        for value, name in ((-1, 'unknown'), (100, 'occupied')):
            indexes = [i for i, cell in enumerate(grid.data) if cell == value]
            for index in indexes:
                x = grid.info.origin.position.x + (index % grid.info.width + 0.5) * grid.info.resolution
                y = grid.info.origin.position.y + (index // grid.info.width + 0.5) * grid.info.resolution
                raw = costmap.data[cell_index(x, y, costmap)]
                if raw != (255 if value == -1 else 254):
                    continue
                plan, status = self.plan(self.pose_goal(x, y, 0.0))
                assert status != 4 and (plan is None or not plan.poses)
                results.append({'type': name, 'goal': [x, y], 'map_value': value,
                                'costmap_value': raw, 'planner_action_status': status,
                                'path_poses': len(plan.poses) if plan else 0})
                break
            else:
                raise RuntimeError(f'No actual {name} test cell found')
        return results

    def wait_current_observations(self, timeout=5.0):
        deadline = time.monotonic()+timeout
        while time.monotonic()<deadline:
            rclpy.spin_once(self,timeout_sec=.01)
            tf = self.buffer.lookup_transform('map','base_link',rclpy.time.Time())
            age = self.clock_samples[-1]-(tf.header.stamp.sec+tf.header.stamp.nanosec/1e9)
            if abs(age)<.10 and time.monotonic()-self.received.get('clock',0)<.10:
                return
        raise RuntimeError('Recorder TF/clock did not catch up before sending a goal')

    def execute(self, name, desired, timeout):
        assert self.docked is False and self.truth is not None
        assert not any(value in (1, 2, 3, 4) for value in self.current_hazards)
        nodes = self.get_node_names()
        assert 'turtlebot4_teleop' not in nodes
        pose, margin, planned = self.select_goal(desired)
        self.wait_current_observations()
        start = self.snapshot()
        assert start['localization_error_m'] + self.registration['reference_uncertainty_m'] <= 0.3
        assert start['localization_yaw_error'] + self.registration.get('reference_yaw_uncertainty_rad', 0) <= 0.25
        self.paths, self.commands, self.hazards, self.observations, self.path_errors = [], [], [], [], []
        self.bumper_contacts = 0
        self.controller_logs = []
        self.recovery_count = 0
        self.last_observation = 0.0
        self.pending_path_samples = []
        self.recording = True
        goal = NavigateToPose.Goal(pose=pose)
        goal_id = UUID(uuid=list(uuid.uuid4().bytes))
        future = None
        self.trial_start = start
        self.trial_target = [pose.pose.position.x,pose.pose.position.y,yaw(pose.pose.orientation)]
        try:
            handle = self.await_future(self.action.send_goal_async(goal, feedback_callback=self.feedback_cb,
                                                                  goal_uuid=goal_id), 5)
            assert handle.accepted
            self.active_goal = handle
            future = handle.get_result_async()
            deadline = time.monotonic() + timeout
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                self.resolve_pending_paths()
                for key in ('local', 'global'):
                    validate_costmap_freshness(self.messages[key], self.clock_samples[-1])
                    footprint = self.messages[key + '_footprint']
                    stamp = footprint.header.stamp.sec + footprint.header.stamp.nanosec / 1e9
                    if (abs(self.clock_samples[-1] - stamp) > 3.0 or
                            time.monotonic() - self.received[key + '_footprint'] > 3.0):
                        raise RuntimeError(f'{key} footprint stopped advancing')
                if any(value in (1, 2, 3, 4) for value in self.hazards) or self.path_errors or self.bumper_contacts:
                    raise RuntimeError('Collision hazard or unsafe path: ' + str(self.path_errors))
                if not covariance_health(self.messages['pose'])['acceptable']:
                    raise RuntimeError('Moving AMCL uncertainty exceeded readiness bound')
                if self.observations and self.observations[-1]['localization_error_m'] + self.registration['reference_uncertainty_m'] > 0.3:
                    raise RuntimeError('Localization drift exceeded safety bound')
                if self.recovery_count > 1:
                    raise RuntimeError('Repeated navigation recoveries')
                dwb_failures = sum('No valid trajectories' in entry['message'] for entry in self.controller_logs)
                if dwb_failures > 1:
                    raise RuntimeError('Repeated DWB trajectory failures')
            if not future.done():
                raise TimeoutError(f'Goal {name} exceeded {timeout}s')
            result = future.result()
            assert result.status == 4, f'Navigation result {result.status}'
            end = time.monotonic() + 2
            while time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.05)
            self.resolve_pending_paths()
            final = self.snapshot()
            target = [pose.pose.position.x, pose.pose.position.y, yaw(pose.pose.orientation)]
            distance = math.dist(final['pose'][:2], target[:2])
            rotation = angle_error(final['pose'][2], target[2])
            params = self.parameters('controller_server', ['general_goal_checker.xy_goal_tolerance',
                                      'general_goal_checker.yaw_goal_tolerance'], time.monotonic() + 5)
            assert distance <= params['general_goal_checker.xy_goal_tolerance']
            assert rotation <= params['general_goal_checker.yaw_goal_tolerance']
            assert math.dist(start['truth'][:2], final['truth'][:2]) > 0.8
            assert self.paths and any(abs(c[0]) > 0.02 for c in self.commands)
            twist = self.messages['odom'].twist.twist
            assert abs(twist.linear.x) < 0.01 and abs(twist.angular.z) < 0.02
            assert not any(value in (1, 2, 3, 4) for value in self.hazards)
            physical = physical_arrival_accuracy(math.dist(final['truth'][:2], target[:2]),
                                                angle_error(final['truth'][2], target[2]), params, self.registration)
            return {'name': name, 'status': 'PASS' if physical['within_configured_tolerance_verified'] else 'NOT VERIFIED',
                    'nav2_execution_pass': True, 'physical_arrival': physical, 'action_accepted': handle.accepted,
                    'action_result': result.status, 'start': start, 'goal': target, 'final': final,
                    'recovery_count': self.recovery_count, 'controller_warnings': self.controller_logs,
                    'physical_yaw_goal_error': angle_error(final['truth'][2], target[2]),
                    'physical_position_goal_error': math.dist(final['truth'][:2], target[:2]),
                    'reference_uncertainty_m': self.registration['reference_uncertainty_m'],
                    'position_goal_error': distance, 'yaw_goal_error': rotation,
                    'configured_tolerances': params, 'goal_clearance_m': margin,
                    'preflight_plan': planned, 'observed_paths': self.paths,
                    'command_samples': len(self.commands),
                    'max_command': [max(abs(c[0]) for c in self.commands), max(abs(c[1]) for c in self.commands)],
                    'hazard_types': sorted(set(self.hazards)), 'bumper_contacts': self.bumper_contacts,
                    'motion_observations': self.observations,
                    'stopped_velocity': [twist.linear.x, twist.angular.z],
                    'truth_displacement_m': math.dist(start['truth'][:2], final['truth'][:2])}
        finally:
            self.recording = False
            if future is None or not future.done():
                request = CancelGoal.Request()
                request.goal_info.goal_id = goal_id
                self.request(CancelGoal, self.topic('navigate_to_pose/_action/cancel_goal'), request, time.monotonic()+5)
                if future is not None:
                    self.await_future(future, 10)
            self.active_goal = None
            self.recording = False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--registration', type=Path, required=True,
                        help='Reviewed world-to-map landmark registration JSON; no identity assumption')
    parser.add_argument('--goals', type=Path, help='JSON list with exactly one named destination B in validated map space')
    parser.add_argument('--goal-timeout', type=float, default=100.0)
    args = parser.parse_args()
    if not math.isfinite(args.goal_timeout) or not 10 <= args.goal_timeout <= 180:
        parser.error('goal-timeout must be finite and between 10 and 180 seconds')
    registration = json.loads(args.registration.read_text())
    validate_reference_for_map(registration, resolve_saved_map_path())
    rclpy.init()
    node = GoalEvidence(registration)
    results = {'status': 'RUNNING', 'goals': []}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    try:
        results['readiness'] = wait_for_readiness(node, resolve_saved_map_path(), 30, navigation=True)
        assert node.planner.wait_for_server(timeout_sec=5)
        results['unsafe_goal_checks'] = node.unsafe_plan_checks()
        node.wait_current_observations()
        origin = node.snapshot()['pose']
        results['selection_origin'] = origin
        centers = destination_centers(args.goals, origin)
        for name, desired in centers:
            result = node.execute(name, desired, args.goal_timeout)
            results['goals'].append(result)
            args.output.write_text(json.dumps(results, indent=2) + '\n')
            print(f"Goal {name}: SUCCEEDED, final error {result['position_goal_error']:.3f} m", flush=True)
        results['status'] = 'PASS' if all(g['status'] == 'PASS' for g in results['goals']) else 'NOT VERIFIED'
    except BaseException as exc:
        results['status'] = 'FAIL'
        results['error'] = str(exc)
        results['interrupted_trial'] = {'start': getattr(node,'trial_start',None),
            'goal': getattr(node,'trial_target',None), 'motion_observations': node.observations,
            'command_samples': node.commands, 'paths': node.paths, 'warnings': node.controller_logs,
            'hazards': node.hazards, 'bumper_contacts': node.bumper_contacts}
        raise
    finally:
        args.output.write_text(json.dumps(results, indent=2) + '\n')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
