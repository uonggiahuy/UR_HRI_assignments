"""Foreground, single-mission Nav2 executor. Never publishes velocity commands."""

import json
import math
from collections import Counter, deque
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
from turtlebot4.localization_health import covariance_health, pose_time_health, yaw, MAX_POSE_AGE_SEC
from turtlebot4.map_segmentation import load_regions
from turtlebot4.navigation_diagnostics import (
    NavigationProbe,
    cell_index,
    validate_costmap_freshness,
    validate_path,
    wait_for_readiness,
)
from turtlebot4.voice_command_parser import TranscriptGate, VoiceCommandParser


CALLBACK_SERVICE_BUDGET_SEC = 0.05
CALLBACK_WAIT_SEC = 0.005
MAX_CALLBACK_SERVICE_SPINS = 256


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
    def __init__(self, wall_stall_timeout=2.5):
        # Live Gazebo pauses reached 2.047 s with almost unchanged sim stamps.
        # Measurement age remains limited to 1 sim second; this bounds a stall
        # even when simulation time stops or advances extremely slowly.
        if not math.isfinite(wall_stall_timeout) or not 1.0 <= wall_stall_timeout <= 5.0:
            raise ValueError("sensor_wall_stall_timeout must be 1..5 seconds")
        self.wall_stall_timeout = wall_stall_timeout
        self.callback_counts = Counter()
        self.scan_intervals = deque(maxlen=128)
        self.service_diagnostics = {}
        self.last_guard_wall = None
        self.last_clock_progress_wall = None
        self.previous_clock_stamp = None
        self.clock_consistent = True
        super().__init__(independent_tf=True)
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

    def maintain_active_pose(self):
        """Keep real AMCL measurements current while an action may resume motion.

        AMCL gates updates on net displacement. Oscillation or a Nav2 wait can
        leave an old pose before motion resumes. Request actual scan processing
        once per old estimate, asynchronously, without extending its deadline.
        """
        pose = self.messages.get("pose")
        if pose is None or not self.clock_samples:
            return
        stamp = seconds(pose.header.stamp)
        if self.clock_samples[-1] - stamp < MAX_POSE_AGE_SEC / 2:
            return
        pending = getattr(self, "moving_pose_update", None)
        if pending is not None and not pending.done():
            return
        if stamp == getattr(self, "moving_pose_requested_stamp", None):
            return
        client = self.service_clients.get((Empty, "/request_nomotion_update"))
        if client is None or not client.service_is_ready():
            return  # The unchanged guard terminates if no fresh estimate arrives.
        self.moving_pose_requested_stamp = stamp
        self.moving_pose_update = client.call_async(Empty.Request())

    def record(self, key, msg):
        previous = self.received.get(key)
        super().record(key, msg)
        self.callback_counts[key] += 1
        if key == "scan" and previous is not None:
            self.scan_intervals.append(self.received[key] - previous)
        if key == "clock":
            stamp = self.clock_samples[-1]
            if self.previous_clock_stamp is None or stamp > self.previous_clock_stamp:
                self.last_clock_progress_wall = self.received[key]
            if self.previous_clock_stamp is not None:
                self.clock_consistent &= stamp >= self.previous_clock_stamp
            self.previous_clock_stamp = stamp
            del self.clock_samples[:-200]
        if key in self.costmap_samples:
            del self.costmap_samples[key][:-100]

    def service_callbacks(self):
        """Give queued sensors a bounded turn before inspecting freshness.

        spin_once executes just one callback: after a worker delay it can deliver
        a fresh clock while scan/odom are still queued. Sensor caches and mission
        checks stay on this worker; TF has an independent, thread-safe Buffer.
        """
        started = time.monotonic()
        previous = self.service_diagnostics.get("finished_wall", started)
        executor = rclpy.get_global_executor()
        added = executor.add_node(self)
        calls = 0
        try:
            # Start the servicing budget after the first callback: an OS/worker
            # delay inside that wait must not consume the whole recovery turn.
            rclpy.spin_once(self, executor=executor, timeout_sec=CALLBACK_WAIT_SEC)
            calls += 1
            deadline = time.monotonic() + CALLBACK_SERVICE_BUDGET_SEC
            while calls < MAX_CALLBACK_SERVICE_SPINS and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=CALLBACK_WAIT_SEC)
                calls += 1
        finally:
            if added:
                executor.remove_node(self)
        self.service_diagnostics = {
            "spin_calls": calls,
            "duration_wall": time.monotonic() - started,
            "gap_wall": started - previous,
            "finished_wall": time.monotonic(),
        }

    def timing_diagnostics(self, now=None):
        now = time.monotonic() if now is None else now
        sim = self.clock_samples[-1] if self.clock_samples else None
        stamps = {
            key: seconds(self.messages[key].header.stamp)
            if key in self.messages else None for key in ("scan", "odom")
        }
        return {
            "wall": now,
            "wall_stall_timeout": self.wall_stall_timeout,
            "callback_age_wall": {key: now - self.received[key]
                                  if key in self.received else None
                                  for key in ("scan", "clock", "odom", "global", "local")},
            "message_stamp": stamps,
            "simulation_clock": sim,
            "clock_progress_age_wall": now - self.last_clock_progress_wall
            if self.last_clock_progress_wall is not None else None,
            "clock_consistent": self.clock_consistent,
            "scan_age_sim": sim - stamps["scan"]
            if sim is not None and stamps["scan"] is not None else None,
            "scan_interval_wall_last": self.scan_intervals[-1]
            if self.scan_intervals else None,
            "scan_interval_wall_max": max(self.scan_intervals, default=None),
            "callback_counts": dict(self.callback_counts),
            "executor_service": dict(self.service_diagnostics),
            "guard_interval_wall": now - self.last_guard_wall
            if self.last_guard_wall is not None else None,
        }

    def stale(self, reason, timing):
        self.get_logger().error(json.dumps({
            "reason": reason, "timing": timing
        }))
        raise RuntimeError(reason)

    def dock_cb(self, msg):
        self.docked, self.dock_received = msg.is_docked, time.monotonic()

    def hazard_cb(self, msg):
        self.hazards, self.hazard_received = [
            d.type for d in msg.detections
        ], time.monotonic()

    def guard(self):
        now = time.monotonic()
        timing = self.timing_diagnostics(now)
        self.last_guard_wall = now
        status_limit = max(2.0, self.wall_stall_timeout)
        if self.docked is not False or now - self.dock_received > status_limit:
            raise RuntimeError(
                "Robot not verifiably undocked; /dock_status absent or stale"
            )
        if now - self.hazard_received > status_limit or any(
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
        for key in ("clock", "scan", "odom", "global", "local"):
            # Costmaps age in ROS time below. Several short Gazebo pauses can
            # span one grid publication without aging the grid in simulation.
            # The independent clock/scan/odom wall guards still bound stalls.
            wall_stale = (key in ("clock", "scan", "odom")
                          and now - self.received.get(key, 0.0) > self.wall_stall_timeout)
            if key not in self.messages or wall_stale:
                self.stale(f"Stale {key}", timing)
        sim = self.clock_samples[-1] if self.clock_samples else 0.0
        if (
            len(self.clock_samples) < 5
            or not self.clock_consistent
            or self.last_clock_progress_wall is None
            or now - self.last_clock_progress_wall > self.wall_stall_timeout
        ):
            self.stale("Simulation clock not advancing", timing)
        for key in ("scan", "odom"):
            if abs(sim - seconds(self.messages[key].header.stamp)) > 1.0:
                self.stale(f"Stale {key} measurement", timing)
        pose = self.messages.get("pose")
        covariance = covariance_health(pose) if pose is not None else {"acceptable": False}
        timing["localization_covariance"] = covariance
        if not covariance["acceptable"]:
            self.stale("Localization uncertain or uninitialized", timing)
        for parent, child in [
            ("map", "odom"),
            ("odom", "base_link"),
            ("map", "base_link"),
        ]:
            tf = self.buffer.lookup_transform(parent, child, rclpy.time.Time())
            if abs(sim - seconds(tf.header.stamp)) > 0.6:
                timing["tf"] = {"parent": parent, "child": child,
                                "stamp": seconds(tf.header.stamp),
                                "age_sim": sim - seconds(tf.header.stamp)}
                self.stale("Localization transform stale", timing)
        tf = self.buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        observations = self.odom_observations
        stationary = (
            len(observations) > 1
            and observations[-1][0] - observations[0][0] >= 2.0
            and all(stopped for _, stopped in observations)
        )
        pose_timing = pose_time_health(
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
        if not pose_timing["acceptable"]:
            timing["localization_timestamp"] = pose_timing
            timing["stationary_observed"] = stationary
            twist = self.messages["odom"].twist.twist
            timing["odom_velocity"] = [twist.linear.x, twist.linear.y, twist.angular.z]
            self.stale("AMCL timestamp inconsistent with current localization", timing)
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
        self.probe = VoiceProbe(float(self.declare_parameter(
            "sensor_wall_stall_timeout", 2.5
        ).value))
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
            self.probe.service_callbacks()
            if guard:
                self.probe.maintain_active_pose()
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
                    self.probe.service_callbacks()
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
