#!/usr/bin/env python3
"""M4 simulator-only evidence capture: plan checks and three real Nav2 goals.

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

from turtlebot4.navigation_diagnostics import NavigationProbe, cell_index, validate_path, validate_costmap, wait_for_readiness
from turtlebot4.localization_runner import resolve_saved_map_path


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def angle_error(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


class GoalEvidence(NavigationProbe):
    def __init__(self):
        super().__init__()
        self.truth = None
        self.docked = None
        self.hazards = []
        self.commands = []
        self.paths = []
        self.bumper_contacts = 0
        self.observations = []
        self.path_errors = []
        self.recording = False
        self.create_subscription(Odometry, '/sim_ground_truth_pose', self.truth_cb, qos_profile_sensor_data)
        self.create_subscription(DockStatus, '/dock_status', self.dock_cb, qos_profile_sensor_data)
        self.create_subscription(HazardDetectionVector, '/hazard_detection', self.hazard_cb, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.command_cb, qos_profile_sensor_data)
        self.create_subscription(Contacts, '/bumper_contact', self.contact_cb, qos_profile_sensor_data)
        self.planner = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.active_goal = None

    def truth_cb(self, msg):
        self.truth = msg
        if self.recording:
            self.observations.append(self.snapshot())

    def dock_cb(self, msg):
        self.docked = msg.is_docked

    def hazard_cb(self, msg):
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
                stats['points'] = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
                self.paths.append(stats)
            except ValueError as exc:
                self.path_errors.append(str(exc))

    def snapshot(self):
        transform = self.buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        if self.recording:
            stamp = transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9
            if abs(self.clock_samples[-1] - stamp) > 0.5:
                raise RuntimeError('Moving localization TF is stale')
        tf = transform.transform
        truth = self.truth.pose.pose
        pose = [tf.translation.x, tf.translation.y, yaw(tf.rotation)]
        actual = [truth.position.x, truth.position.y, yaw(truth.orientation)]
        return {'time': self.clock_samples[-1], 'pose': pose, 'truth': actual,
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
        for iy, ix in zip(*np.where(clearance >= 0.55)):
            x = grid.info.origin.position.x + (ix + 0.5) * grid.info.resolution
            y = grid.info.origin.position.y + (iy + 0.5) * grid.info.resolution
            if math.dist((x, y), desired) > 1.0:
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

    def execute(self, name, desired, timeout):
        assert self.docked is False and self.truth is not None
        nodes = self.get_node_names()
        assert 'turtlebot4_teleop' not in nodes
        pose, margin, planned = self.select_goal(desired)
        start = self.snapshot()
        assert start['localization_error_m'] < 0.2 and start['localization_yaw_error'] < 0.25
        self.paths, self.commands, self.hazards, self.observations, self.path_errors = [], [], [], [], []
        self.bumper_contacts = 0
        self.recording = True
        goal = NavigateToPose.Goal(pose=pose)
        handle = self.await_future(self.action.send_goal_async(goal), 5)
        assert handle.accepted
        self.active_goal = handle
        future = handle.get_result_async()
        deadline = time.monotonic() + timeout
        try:
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                for key in ('local', 'global'):
                    validate_costmap(self.messages[key], self.clock_samples[-1])
                if any(value in (1, 2, 3, 4) for value in self.hazards) or self.path_errors or self.bumper_contacts:
                    raise RuntimeError('Collision hazard or unsafe path: ' + str(self.path_errors))
                if self.observations and self.observations[-1]['localization_error_m'] > 0.3:
                    raise RuntimeError('Localization drift exceeded safety bound')
            if not future.done():
                raise TimeoutError(f'Goal {name} exceeded {timeout}s')
            result = future.result()
            assert result.status == 4, f'Navigation result {result.status}'
            end = time.monotonic() + 2
            while time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.05)
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
            return {'name': name, 'status': 'PASS', 'action_accepted': handle.accepted,
                    'action_result': result.status, 'start': start, 'goal': target, 'final': final,
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
            if not future.done():
                self.await_future(handle.cancel_goal_async(), 5)
                self.await_future(future, 10)
            self.active_goal = None
            self.recording = False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--goal-timeout', type=float, default=100.0)
    args = parser.parse_args()
    if not math.isfinite(args.goal_timeout) or not 10 <= args.goal_timeout <= 180:
        parser.error('goal-timeout must be finite and between 10 and 180 seconds')
    rclpy.init()
    node = GoalEvidence()
    results = {'status': 'RUNNING', 'goals': []}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    try:
        results['readiness'] = wait_for_readiness(node, resolve_saved_map_path(), 30, navigation=True)
        assert node.planner.wait_for_server(timeout_sec=5)
        results['unsafe_goal_checks'] = node.unsafe_plan_checks()
        origin = node.snapshot()['pose']
        results['selection_origin'] = origin
        centers = [('A', (origin[0] - 1.5, origin[1])),
                   ('B', (origin[0] - 1.5, origin[1] - 1.5)),
                   ('C', (origin[0], origin[1] - 1.5))]
        for name, desired in centers:
            result = node.execute(name, desired, args.goal_timeout)
            results['goals'].append(result)
            args.output.write_text(json.dumps(results, indent=2) + '\n')
            print(f"Goal {name}: SUCCEEDED, final error {result['position_goal_error']:.3f} m", flush=True)
        results['status'] = 'PASS'
    except BaseException as exc:
        results['status'] = 'FAIL'
        results['error'] = str(exc)
        raise
    finally:
        args.output.write_text(json.dumps(results, indent=2) + '\n')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
