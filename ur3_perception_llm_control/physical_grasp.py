"""Verified Gazebo Fortress detachable joints for Assignment 03 cubes.

Gazebo poses are read only to check grasp proximity. They are never planning
targets or perception input. All cube motion is produced by Gazebo physics.
"""

from __future__ import annotations

import json
import math
import subprocess
import time

from geometry_msgs.msg import Pose
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformListener

from ur3_perception_llm_control.gripper import CLOSED_POSITION, FINGER_JOINTS


CUBES = ("red_cube", "yellow_cube", "blue_cube", "green_cube", "purple_cube")


def model_pose(object_name: str, timeout: float = 3.0) -> Pose | None:
    """Read-only Gazebo pose oracle for proximity and M7 diagnostics."""
    try:
        result = subprocess.run(
            ["ign", "topic", "-e", "-t", "/world/empty/pose/info", "--json-output", "-n", "1"],
            capture_output=True, text=True, timeout=timeout, check=False,
        )
        if result.returncode:
            return None
        message = json.loads(result.stdout)
        for item in message.get("pose", []):
            if item.get("name") == object_name:
                position = item.get("position", {})
                orientation = item.get("orientation", {})
                pose = Pose()
                pose.position.x = float(position.get("x", 0.0))
                pose.position.y = float(position.get("y", 0.0))
                pose.position.z = float(position.get("z", 0.0))
                pose.orientation.x = float(orientation.get("x", 0.0))
                pose.orientation.y = float(orientation.get("y", 0.0))
                pose.orientation.z = float(orientation.get("z", 0.0))
                pose.orientation.w = float(orientation.get("w", 1.0))
                return pose
    except (OSError, subprocess.TimeoutExpired, ValueError, json.JSONDecodeError):
        pass
    return None


class PhysicalGraspManager:
    """Own and verify one physical fixed joint at a time.

    Fortress initially creates each declared joint. ``wait_until_ready``
    detaches all five before the robot can move. State events are required for
    every transition; a missing event is a failure, never assumed success.
    """

    def __init__(self, node: Node) -> None:
        self._node = node
        self._state: dict[str, str | None] = {name: None for name in CUBES}
        self._revision: dict[str, int] = {name: 0 for name in CUBES}
        self._publishers = {}
        self._subscriptions = []
        for name in CUBES:
            base = f"/m7/grasp/{name}"
            self._publishers[name] = (
                node.create_publisher(Empty, f"{base}/attach", 10),
                node.create_publisher(Empty, f"{base}/detach", 10),
            )
            self._subscriptions.append(node.create_subscription(
                String, f"{base}/state",
                lambda message, cube=name: self._on_state(cube, message), 10,
            ))
        self._joints: dict[str, float] = {}
        self._subscriptions.append(node.create_subscription(
            JointState, "/joint_states", self._on_joints, 10,
        ))
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, node)
        self._ready = False

    def _on_state(self, name: str, message: String) -> None:
        if message.data in ("attached", "detached"):
            self._state[name] = message.data
            self._revision[name] += 1

    def _on_joints(self, message: JointState) -> None:
        self._joints.update(zip(message.name, message.position))

    def wait_until_ready(self, timeout: float = 45.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if all(pub[1].get_subscription_count() for pub in self._publishers.values()):
                break
            rclpy.spin_once(self._node, timeout_sec=0.1)
        else:
            return False
        # The plugin starts attached. Explicitly clear all joints before HOME.
        for name in CUBES:
            if not self._transition(name, "detached", deadline - time.monotonic()):
                self._node.get_logger().error(f"Cannot establish detached baseline: {name}")
                return False
        self._ready = True
        return True

    def is_attached(self, object_name: str) -> bool:
        return self._ready and self._state.get(object_name) == "attached"

    def attach(self, object_name: str, timeout: float = 8.0) -> bool:
        if not self._ready or object_name not in CUBES:
            return False
        if any(state == "attached" for state in self._state.values()):
            return False
        if self._state[object_name] != "detached" or not self._closed() or not self._near(object_name):
            return False
        return self._transition(object_name, "attached", timeout)

    def release(self, object_name: str, timeout: float = 8.0) -> bool:
        if not self.is_attached(object_name):
            return False
        return self._transition(object_name, "detached", timeout)

    def _closed(self) -> bool:
        return all(
            joint in self._joints
            and abs(self._joints[joint] - CLOSED_POSITION) <= 0.003
            for joint in FINGER_JOINTS
        )

    def _near(self, object_name: str) -> bool:
        cube = model_pose(object_name)
        if cube is None:
            return False
        try:
            transform = self._tf.lookup_transform(
                "world", "gripper_tcp", rclpy.time.Time(),
                timeout=Duration(seconds=2.0),
            )
        except Exception as exc:
            self._node.get_logger().error(f"Grasp TF unavailable: {exc}")
            return False
        tcp = transform.transform.translation
        xy = math.hypot(cube.position.x - tcp.x, cube.position.y - tcp.y)
        z = abs(cube.position.z - tcp.z)
        self._node.get_logger().info(f"Grasp proximity {object_name}: XY={xy:.4f} m Z={z:.4f} m")
        return xy <= 0.015 and z <= 0.020

    def _transition(self, object_name: str, wanted: str, timeout: float) -> bool:
        if timeout <= 0:
            return False
        before = self._revision[object_name]
        attach, detach = self._publishers[object_name]
        deadline = time.monotonic() + timeout
        next_publish = 0.0
        while time.monotonic() < deadline:
            if time.monotonic() >= next_publish:
                (attach if wanted == "attached" else detach).publish(Empty())
                next_publish = time.monotonic() + 0.5
            rclpy.spin_once(self._node, timeout_sec=min(0.1, deadline - time.monotonic()))
            if self._revision[object_name] > before:
                return self._state[object_name] == wanted
        return False

    def destroy(self) -> None:
        for publishers in self._publishers.values():
            for publisher in publishers:
                self._node.destroy_publisher(publisher)
        for subscription in self._subscriptions:
            self._node.destroy_subscription(subscription)
