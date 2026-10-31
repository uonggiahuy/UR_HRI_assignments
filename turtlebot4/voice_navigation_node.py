"""Foreground, single-mission Nav2 executor. Never publishes velocity commands."""

import json
import math
from pathlib import Path
import queue
import threading
import time

from action_msgs.msg import GoalStatus, GoalStatusArray
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from irobot_create_msgs.msg import DockStatus, HazardDetectionVector
from nav2_msgs.action import ComputePathToPose, NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Empty

from turtlebot4.diagnostics import seconds
from turtlebot4.localization_health import covariance_health, pose_time_health, yaw
from turtlebot4.map_segmentation import load_regions
from turtlebot4.navigation_diagnostics import (
    NavigationProbe,
    cell_index,
    validate_costmap_freshness,
    validate_path,
    wait_for_readiness,
)
from turtlebot4.voice_command_parser import TranscriptGate, VoiceCommandParser


def goal_pose(item, stamp):
    pose = PoseStamped()
    pose.header.frame_id, pose.header.stamp = "map", stamp
    pose.pose.position.x, pose.pose.position.y = float(item["x"]), float(item["y"])
    pose.pose.orientation.z, pose.pose.orientation.w = math.sin(
        item["yaw"] / 2
    ), math.cos(item["yaw"] / 2)
    return pose


def validate_live_goal(item, costmap, clearance=0.22):
    """Check every intersecting cell of a conservative 0.22 m robot disk."""
    if costmap.header.frame_id != "map":
        raise ValueError("Global costmap frame must be map")
    meta = costmap.metadata
    resolution = meta.resolution
    if resolution <= 0:
        raise ValueError("Invalid costmap resolution")
    center = cell_index(item["x"], item["y"], costmap)
    cx, cy = center % meta.size_x, center // meta.size_x
    steps = math.ceil(clearance / resolution) + 1
    angle = yaw(meta.origin.orientation)
    c, s = math.cos(angle), math.sin(angle)
    for ix in range(cx - steps, cx + steps + 1):
        for iy in range(cy - steps, cy + steps + 1):
            lx, ly = (ix + 0.5) * resolution, (iy + 0.5) * resolution
            x = meta.origin.position.x + c * lx - s * ly
            y = meta.origin.position.y + s * lx + c * ly
            if math.hypot(
                x - item["x"], y - item["y"]
            ) > clearance + resolution / math.sqrt(2):
                continue
            if not (0 <= ix < meta.size_x and 0 <= iy < meta.size_y):
                raise ValueError("Goal footprint outside costmap")
            if costmap.data[iy * meta.size_x + ix] >= 253:
                raise ValueError("Goal footprint blocked in live costmap")


def validate_loaded_map(grid, message):
    import numpy as np

    expected = np.where(grid.free, 0, np.where(grid.occupied, 100, -1))
    actual = np.asarray(message.data).reshape(message.info.height, message.info.width)
    if actual.shape != expected.shape or not np.array_equal(
        actual, np.flipud(expected)
    ):
        raise ValueError("Live occupancy map differs from segmented source map")


class CancelReadiness:
    def __init__(self, cancel, closing):
        self.cancel, self.closing = cancel, closing
        self.returncode = "cancellation requested"

    def poll(self):
        return 1 if self.cancel.is_set() or self.closing.is_set() else None


class MissionStopped(RuntimeError):
    pass


class MissionExecutor:
    """Action protocol separated from ROS adapters for failure/order testing."""

    def __init__(self, adapter):
        self.adapter = adapter

    def run(self, ids):
        for ident in ids:
            self.adapter.validate(ident)
            self.adapter.plan(ident)
            self.adapter.navigate(ident)


class VoiceProbe(NavigationProbe):
    def __init__(self):
        super().__init__()
        self.docked, self.dock_received, self.hazards, self.hazard_received = (
            None,
            0.0,
            [],
            0.0,
        )
        self.navigation_status = []
        self.owned_goals = []
        self.create_subscription(
            GoalStatusArray, "/navigate_to_pose/_action/status", self.action_status, 10
        )
        self.planner = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.create_subscription(
            DockStatus, "/dock_status", self.dock_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            HazardDetectionVector,
            "/hazard_detection",
            self.hazard_cb,
            qos_profile_sensor_data,
        )

    def action_status(self, msg):
        self.navigation_status = msg.status_list

    def record(self, key, msg):
        super().record(key, msg)
        if key == "clock":
            del self.clock_samples[:-200]
        if key in self.costmap_samples:
            del self.costmap_samples[key][:-100]

    def dock_cb(self, msg):
        self.docked, self.dock_received = msg.is_docked, time.monotonic()

    def hazard_cb(self, msg):
        self.hazards, self.hazard_received = [
            d.type for d in msg.detections
        ], time.monotonic()

    def guard(self):
        now = time.monotonic()
        if self.docked is not False or now - self.dock_received > 2.0:
            raise RuntimeError(
                "Robot not verifiably undocked; /dock_status absent or stale"
            )
        if now - self.hazard_received > 2.0 or any(
            h in (1, 2, 3, 4) for h in self.hazards
        ):
            raise RuntimeError("Robot hazard data absent, stale or unsafe")
        if any(
            s.status in (1, 2, 3)
            and bytes(s.goal_info.goal_id.uuid) not in self.owned_goals
            for s in self.navigation_status
        ):
            raise RuntimeError("Another NavigateToPose mission is active")
        names = self.get_node_names()
        if any(
            n in names
            for n in ("waypoint_navigator", "robot_behavior_node", "turtlebot4_teleop")
        ):
            raise RuntimeError("Competing waypoint/gesture/teleop controller")
        allowed = {
            "cmd_vel_bridge",
            "controller_server",
            "velocity_smoother",
            "behavior_server",
        }
        if any(
            p.node_name not in allowed
            for p in self.get_publishers_info_by_topic("/cmd_vel")
        ):
            raise RuntimeError("Competing velocity publisher")
        for key in ("clock", "scan", "odom", "pose", "global", "local"):
            limit = 3.0 if key in ("pose", "global", "local") else 1.0
            if key not in self.messages or now - self.received.get(key, 0.0) > limit:
                # AMCL may be sparse at rest; the inherited timestamp policy below
                # permits stationary estimates while advancing TF remains required.
                if key != "pose":
                    raise RuntimeError(f"Stale {key}")
        sim = self.clock_samples[-1] if self.clock_samples else 0.0
        if (
            len(self.clock_samples) < 5
            or self.clock_samples[-1] <= self.clock_samples[-5]
        ):
            raise RuntimeError("Simulation clock not advancing")
        for key in ("scan", "odom"):
            if abs(sim - seconds(self.messages[key].header.stamp)) > 1.0:
                raise RuntimeError(f"Stale {key} measurement")
        pose = self.messages.get("pose")
        if pose is None or not covariance_health(pose)["acceptable"]:
            raise RuntimeError("Localization uncertain or uninitialized")
        for parent, child in [
            ("map", "odom"),
            ("odom", "base_link"),
            ("map", "base_link"),
        ]:
            tf = self.buffer.lookup_transform(parent, child, rclpy.time.Time())
            if abs(sim - seconds(tf.header.stamp)) > 0.6:
                raise RuntimeError("Localization transform stale")
        tf = self.buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        observations = self.odom_observations
        stationary = (
            len(observations) > 1
            and observations[-1][0] - observations[0][0] >= 2.0
            and all(stopped for _, stopped in observations)
        )
        timing = pose_time_health(
            pose,
            sim,
            self.messages["scan"],
            self.messages["odom"],
            (
                tf.transform.translation.x,
                tf.transform.translation.y,
                yaw(tf.transform.rotation),
            ),
            stationary,
        )
        if not timing["acceptable"]:
            raise RuntimeError("AMCL timestamp inconsistent with current localization")
        for key in ("global", "local"):
            validate_costmap_freshness(self.messages[key], sim)


class VoiceNavigationNode(Node):
    def __init__(self):
        super().__init__("voice_navigation_node")
        share = Path(get_package_share_directory("turtlebot4"))
        self.regions_path = self.declare_parameter(
            "regions_file", str(share / "maps/regions.yaml")
        ).value
        self.map_path = self.declare_parameter(
            "map_file", str(share / "maps/map.yaml")
        ).value
        self.timeout = float(self.declare_parameter("navigation_timeout", 180.0).value)
        if not 10 <= self.timeout <= 600:
            raise ValueError("navigation_timeout must be 10..600 seconds")
        self.config, self.grid = load_regions(self.regions_path, self.map_path)
        self.regions = {r["id"]: r for r in self.config["regions"]}
        self.parser, self.gate = VoiceCommandParser(), TranscriptGate()
        self.cancel = threading.Event()
        self.closing = threading.Event()
        self.lock = threading.Lock()
        self.busy, self.current, self.command, self.ids = False, "", "", ()
        self.jobs = queue.Queue(maxsize=1)
        self.events = self.create_publisher(String, "/assignment7/state", 10)
        self.create_subscription(String, "/recognized_speech", self.recognized, 10)
        self.probe = VoiceProbe()
        self.goal_handle = self.pending = None
        self.worker = threading.Thread(target=self.work, name="assignment7-mission")
        self.worker.start()
        self.status("IDLE")

    def status(self, state, **values):
        event = dict(
            state=state,
            recognized_command=self.command,
            parsed_region_ids=self.ids,
            current_destination=self.current,
            **values,
        )
        self.events.publish(String(data=json.dumps(event, ensure_ascii=False)))
        self.get_logger().info(json.dumps(event, ensure_ascii=False))

    def recognized(self, msg):
        command = self.parser.parse(msg.data)
        if command.cancel:
            self.cancel.set()
            if not self.busy:
                self.status("CANCELED", reason="Không có nhiệm vụ đang chạy")
            return
        with self.lock:
            if self.closing.is_set():
                self.status("FAILED", reason="Executor closed; restart required")
                return
            if self.busy:
                self.status(
                    "RECOGNIZED", reason="Đang chạy nhiệm vụ; lệnh mới bị từ chối"
                )
                return
            if not self.gate.accept(msg.data, time.monotonic()):
                return
            self.command, self.ids = msg.data, command.regions
            self.status("RECOGNIZED")
            if command.error:
                self.status("FAILED", reason=command.error)
                return
            self.cancel.clear()
            self.busy = True
            self.jobs.put_nowait(command.regions)

    def check_cancel(self):
        if self.cancel.is_set() or self.closing.is_set():
            raise MissionStopped("Đã yêu cầu hủy nhiệm vụ")

    def await_future(self, future, timeout, guard=False, interruptible=True):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                raise TimeoutError("Action response/result deadline exceeded")
            if interruptible:
                self.check_cancel()
            rclpy.spin_once(self.probe, timeout_sec=0.03)
            if guard:
                self.probe.guard()
        result = future.result()
        if result is None:
            raise RuntimeError("Empty action result")
        return result

    def resolve_pending(self):
        if self.pending is not None:
            handle = self.await_future(self.pending, 8.0, interruptible=False)
            self.pending = None
            if handle.accepted:
                self.goal_handle = handle

    def cancel_active(self):
        # An in-flight send may accept after cancellation; resolve it before
        # clearing ownership, then require a terminal result before any new mission.
        self.resolve_pending()
        if self.goal_handle is not None:
            self.await_future(
                self.goal_handle.cancel_goal_async(), 5.0, interruptible=False
            )
            result = self.await_future(
                self.goal_handle.get_result_async(), 8.0, interruptible=False
            )
            if result.status not in (
                GoalStatus.STATUS_CANCELED,
                GoalStatus.STATUS_ABORTED,
                GoalStatus.STATUS_SUCCEEDED,
            ):
                raise RuntimeError("Cancellation did not reach terminal state")
            self.goal_handle = None

    def refresh_pose(self):
        """Request real sensor processing before transitioning out of rest."""
        previous = seconds(self.probe.messages["pose"].header.stamp)
        self.probe.request(
            Empty, "/request_nomotion_update", Empty.Request(), time.monotonic() + 3.0
        )
        deadline = time.monotonic() + 3.0
        while seconds(self.probe.messages["pose"].header.stamp) <= previous:
            self.check_cancel()
            if time.monotonic() > deadline:
                raise TimeoutError("AMCL did not process a fresh scan")
            rclpy.spin_once(self.probe, timeout_sec=0.03)

    def wait_stopped(self):
        deadline = time.monotonic() + 5.0
        started = time.monotonic()
        while time.monotonic() < deadline:
            self.check_cancel()
            rclpy.spin_once(self.probe, timeout_sec=0.03)
            samples = [s for s in self.probe.odom_observations if s[0] >= started]
            if (
                len(samples) > 1
                and samples[-1][0] - samples[0][0] >= 0.5
                and all(s[1] for s in samples)
            ):
                return
            if samples and not samples[-1][1]:
                started = samples[-1][0]
        raise TimeoutError("No measured stationary odometry after action")

    def validate(self, ident):
        self.current = ident
        self.status("VALIDATING")
        self.check_cancel()
        # Re-read hashes on each leg: stale config/map cannot silently continue.
        load_regions(self.regions_path, self.map_path)
        r = self.regions[ident]
        if r["validation"] != "VALID" or not r["approach_goal"]:
            raise ValueError(f'Region {ident} unavailable: {r["reason"]}')
        self.grid.validate_goal(r["approach_goal"])
        self.probe.lifecycle(
            [
                "map_server",
                "amcl",
                "planner_server",
                "controller_server",
                "bt_navigator",
            ],
            time.monotonic() + 8.0,
        )
        self.refresh_pose()
        self.probe.guard()
        validate_loaded_map(self.grid, self.probe.messages["map"])
        grid = self.probe.messages["global"]
        validate_live_goal(r["approach_goal"], grid)

    def send(self, client, goal, timeout):
        if not client.server_is_ready():
            raise RuntimeError("Action server unavailable")
        self.pending = client.send_goal_async(goal, feedback_callback=self.feedback)
        self.resolve_pending()
        self.check_cancel()
        if self.goal_handle is None:
            raise RuntimeError("Action goal rejected")
        if client is self.probe.action:
            self.probe.owned_goals.append(bytes(self.goal_handle.goal_id.uuid))
            del self.probe.owned_goals[:-8]
        result = self.await_future(
            self.goal_handle.get_result_async(), timeout, guard=True
        )
        self.goal_handle = None
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"Action failed with status {result.status}")
        return result.result

    def plan(self, ident):
        goal = ComputePathToPose.Goal()
        goal.goal = goal_pose(
            self.regions[ident]["approach_goal"], self.probe.messages["clock"].clock
        )
        goal.use_start = False
        goal.planner_id = "GridBased"
        result = self.send(self.probe.planner, goal, 20.0)
        validate_path(
            result.path, self.probe.messages["map"], self.probe.messages["global"]
        )
        self.status("VALIDATING", nav2_path="VALID")

    def navigate(self, ident):
        # Recheck after planning; no competing goal can be submitted by this worker.
        self.validate(ident)
        goal = NavigateToPose.Goal()
        goal.pose = goal_pose(
            self.regions[ident]["approach_goal"], self.probe.messages["clock"].clock
        )
        self.status("NAVIGATING")
        self.send(self.probe.action, goal, self.timeout)
        self.wait_stopped()
        self.refresh_pose()
        self.probe.guard()
        transform = self.probe.buffer.lookup_transform(
            "map", "base_link", rclpy.time.Time()
        )
        self.status(
            "SUCCEEDED",
            result=f"Reached {ident}",
            stopped_odometry=True,
            final_map_pose=[
                transform.transform.translation.x,
                transform.transform.translation.y,
            ],
        )

    def feedback(self, msg):
        now = time.monotonic()
        if now - getattr(self, "last_feedback", 0.0) > 1.0 and hasattr(
            msg.feedback, "distance_remaining"
        ):
            self.last_feedback = now
            self.status(
                "NAVIGATING", distance_remaining=msg.feedback.distance_remaining
            )

    def work(self):
        try:
            while not self.closing.is_set():
                try:
                    ids = self.jobs.get(timeout=0.1)
                except queue.Empty:
                    continue
                try:
                    self.status("VALIDATING")
                    # Retained AMCL output can lag the final sub-threshold stop.
                    deadline = time.monotonic() + 5.0
                    while (
                        "pose" not in self.probe.messages
                        and time.monotonic() < deadline
                    ):
                        self.check_cancel()
                        rclpy.spin_once(self.probe, timeout_sec=0.03)
                    if "pose" in self.probe.messages:
                        self.refresh_pose()
                    # Readiness runs outside subscription callbacks, on its own probe.
                    wait_for_readiness(
                        self.probe,
                        self.map_path,
                        timeout=30.0,
                        navigation=True,
                        process=CancelReadiness(self.cancel, self.closing),
                    )
                    self.check_cancel()
                    MissionExecutor(self).run(ids)
                    self.current = ""
                    self.status("SUCCEEDED", result="Mission completed")
                except Exception as exc:
                    try:
                        self.cancel_active()
                    except Exception as cancel_error:
                        # Fail closed: uncertainty about ownership forbids future goals.
                        self.closing.set()
                        exc = RuntimeError(
                            f"{exc}; cancel unconfirmed: {cancel_error}; restart required"
                        )
                    self.status(
                        (
                            "CANCELED"
                            if isinstance(exc, MissionStopped) or self.cancel.is_set()
                            else "FAILED"
                        ),
                        reason=str(exc),
                    )
                finally:
                    with self.lock:
                        self.busy = False
        finally:
            try:
                self.cancel_active()
            finally:
                self.probe.destroy_node()

    def stop(self):
        self.cancel.set()
        self.closing.set()
        self.worker.join(timeout=50.0)
        if self.worker.is_alive():
            self.get_logger().error("Mission worker did not stop within deadline")


def main(args=None):
    from rclpy.signals import SignalHandlerOptions
    import signal

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = VoiceNavigationNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    stop = threading.Event()
    previous = {
        sig: signal.signal(sig, lambda *_: stop.set())
        for sig in (signal.SIGINT, signal.SIGTERM)
    }
    try:
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, executor=executor, timeout_sec=0.1)
    finally:
        node.stop()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
