"""Assignment-local visual/physics synchronization for an attached Gazebo cube.

MoveIt remains the source of truth for collision attachment.  Gazebo Fortress
does not expose an entity attachment service in this workcell, so while a cube
is attached this class repeatedly writes that entity's world pose through the
installed Fortress ``ign service /world/empty/set_pose`` endpoint. Releasing
stops those writes after first setting the requested world pose, allowing
normal Gazebo physics to resume.
"""

from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
from copy import deepcopy
import json
import subprocess
import time

from geometry_msgs.msg import Pose, PoseStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener


class GazeboAttachmentSynchronizer:
    """Keep one Gazebo model aligned with a MoveIt attachment transform."""

    def __init__(
        self,
        node: Node,
        *,
        world_frame: str = "world",
        service_name: str = "/world/empty/set_pose",
        period: float = 0.04,
    ) -> None:
        self._node = node
        self._world_frame = world_frame
        self._service_name = service_name
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)
        self._object_name: str | None = None
        self._attachment_link: str | None = None
        self._relative_pose: Pose | None = None
        self._worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="gazebo_pose")
        self._pending: Future[bool] | None = None
        self._timer = node.create_timer(period, self._synchronize)

    def wait_until_ready(self, timeout: float = 30.0) -> bool:
        """Return whether the installed Gazebo pose service is available."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            query = subprocess.run(
                ["ign", "service", "-l"], capture_output=True, text=True, check=False
            )
            if query.returncode == 0 and self._service_name in query.stdout.splitlines():
                return True
            rclpy.spin_once(self._node, timeout_sec=0.2)
        return False

    def attach(self, object_name: str, attachment_link: str, relative_pose: Pose) -> bool:
        """Begin following ``attachment_link`` with the exact MoveIt offset."""
        if not object_name or not attachment_link:
            return False
        self._object_name = object_name
        self._attachment_link = attachment_link
        self._relative_pose = deepcopy(relative_pose)
        # A newly created listener may not have received /tf_static yet.  The
        # timer retries from the same authoritative transform without changing
        # the stored offset.
        self.synchronize_once()
        return True

    def release(self, world_pose: PoseStamped) -> bool:
        """Set the release pose once, then return the model to normal physics."""
        if self._object_name is None or world_pose.header.frame_id != self._world_frame:
            return False
        success = self.set_world_pose(self._object_name, world_pose)
        self._object_name = None
        self._attachment_link = None
        self._relative_pose = None
        return success

    def set_world_pose(self, object_name: str, world_pose: PoseStamped) -> bool:
        """Write one explicit simulated-world pose without changing MoveIt state."""
        if world_pose.header.frame_id != self._world_frame:
            return False
        return self._set_pose_blocking(object_name, world_pose.pose)

    def stop(self) -> None:
        """Stop following without changing the model's current Gazebo pose."""
        self._object_name = None
        self._attachment_link = None
        self._relative_pose = None

    def model_pose(self, object_name: str, timeout: float = 2.0) -> Pose | None:
        """Read a Gazebo model pose from the installed Fortress pose topic."""
        result = subprocess.run(
            [
                "ign", "topic", "-e", "-t", "/world/empty/pose/info",
                "--json-output", "-n", "1",
            ],
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
        if result.returncode != 0:
            return None
        try:
            message = json.loads(result.stdout)
        except json.JSONDecodeError:
            return None
        for item in message.get("pose", []):
            if item.get("name") == object_name:
                pose = Pose()
                position = item.get("position", {})
                orientation = item.get("orientation", {})
                pose.position.x = float(position.get("x", 0.0))
                pose.position.y = float(position.get("y", 0.0))
                pose.position.z = float(position.get("z", 0.0))
                pose.orientation.x = float(orientation.get("x", 0.0))
                pose.orientation.y = float(orientation.get("y", 0.0))
                pose.orientation.z = float(orientation.get("z", 0.0))
                pose.orientation.w = float(orientation.get("w", 1.0))
                return pose
        return None

    def synchronize_once(self) -> bool:
        """Request one asynchronous Gazebo pose update from the current TF."""
        if (
            self._object_name is None
            or self._attachment_link is None
            or self._relative_pose is None
            or self._pending is not None
        ):
            return False
        try:
            transform = self._tf_buffer.lookup_transform(
                self._world_frame,
                self._attachment_link,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0),
            )
            return self._set_pose(
                self._object_name, do_transform_pose(self._relative_pose, transform)
            )
        except Exception as exc:
            self._node.get_logger().error(
                f"Cannot synchronize Gazebo {self._object_name}: {exc}"
            )
            return False

    def destroy(self) -> None:
        self._timer.cancel()
        self._worker.shutdown(wait=True, cancel_futures=True)

    def _synchronize(self) -> None:
        if self._pending is not None and self._pending.done():
            try:
                if not self._pending.result():
                    self._node.get_logger().error("Gazebo rejected attached-object pose")
            except Exception as exc:
                self._node.get_logger().error(f"Gazebo pose request failed: {exc}")
            self._pending = None
        if self._object_name is not None:
            self.synchronize_once()

    def _set_pose(self, object_name: str, pose: Pose) -> bool:
        self._pending = self._worker.submit(self._set_pose_blocking, object_name, pose)
        return True

    def _set_pose_blocking(self, object_name: str, pose: Pose) -> bool:
        """Call Fortress's installed entity-pose service without a ROS bridge."""
        request = (
            f'name: "{object_name}" '
            f'position {{ x: {pose.position.x:.9g} y: {pose.position.y:.9g} z: {pose.position.z:.9g} }} '
            f'orientation {{ x: {pose.orientation.x:.9g} y: {pose.orientation.y:.9g} '
            f'z: {pose.orientation.z:.9g} w: {pose.orientation.w:.9g} }}'
        )
        result = subprocess.run(
            [
                "ign", "service", "-s", self._service_name, "-r", request,
                "--reqtype", "ignition.msgs.Pose", "--reptype", "ignition.msgs.Boolean",
                "--timeout", "1000",
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode != 0 or "data: true" not in result.stdout:
            self._node.get_logger().error(
                f"Gazebo set-pose failed for {object_name}: {result.stderr.strip() or result.stdout.strip()}"
            )
            return False
        return True
