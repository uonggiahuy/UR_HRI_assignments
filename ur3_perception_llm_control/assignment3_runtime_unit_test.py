"""Focused persistent-runtime camera and command-boundary regressions."""

from __future__ import annotations

from collections import deque
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from ur3_perception_llm_control import assignment3_runtime as runtime_module
from ur3_perception_llm_control.assignment3_runtime import Assignment3Runtime
from ur3_perception_llm_control.gripper import FINGER_JOINTS, OPEN_POSITION
from ur3_perception_llm_control.moveit_interface import ARM_JOINTS
from ur3_perception_llm_control.perception_scene_unit_test import A, snapshot
from ur3_perception_llm_control.perception_state import (
    PerceptionStateError, PerceptionStateStatus, WorkcellGeometry,
)
from ur3_perception_llm_control.perception_state_unit_test import detections
from ur3_perception_llm_control.task_validator import TaskStatus
from ur3_perception_llm_control.world_state import BLOCKS


ROOT = Path(__file__).resolve().parents[1]


class Assignment3RuntimeUnitTest(unittest.TestCase):
    def setUp(self):
        self.runtime = Assignment3Runtime.__new__(Assignment3Runtime)
        self.runtime._geometry = WorkcellGeometry.from_scene_file(ROOT / "config/scene.yaml")
        self.runtime._perception = SimpleNamespace(frames=2, max_xy_spread_m=0.01)
        self.runtime._options = SimpleNamespace(camera_timeout=30.0)
        self.runtime._last_stamp = 9.0
        self.runtime._node = SimpleNamespace(samples=deque(maxlen=2),
                                             timestamps=deque(maxlen=2), failures=[])
        self.runtime._now_sec = lambda: 13.0

    def test_stale_window_is_skipped_until_fresh_window_arrives(self):
        stamps = iter((10.0, 10.1, 11.0, 12.5))
        seen_last_stamps = []

        # Use real detections and stability evaluation while controlling only ROS delivery.
        def deliver(_node, timeout_sec):
            self.assertEqual(timeout_sec, 0.1)
            seen_last_stamps.append(self.runtime._last_stamp)
            stamp = next(stamps)
            self.runtime._node.samples.append(detections(A, stamp))
            self.runtime._node.timestamps.append(stamp)

        with patch.object(runtime_module.rclpy, "ok", return_value=True), \
                patch.object(runtime_module.rclpy, "spin_once", side_effect=deliver):
            accepted = self.runtime._snapshot_source()

        self.assertEqual(accepted.observation_timestamp_sec, 12.5)
        self.assertEqual(seen_last_stamps, [9.0, 9.0, 9.0, 9.0])
        self.assertEqual(self.runtime._last_stamp, 12.5)

    def test_unsupported_command_has_no_perception_or_scene_access(self):
        self.runtime._snapshot_source = Mock()
        self.runtime._skills = Mock()
        self.runtime._physical = Mock()
        self.runtime._synchronizer = Mock()
        self.runtime._planner = Mock()
        self.runtime._node.samples.append(object())
        self.runtime._node.timestamps.append(9.0)

        with patch("builtins.print") as printed:
            self.assertFalse(self.runtime.execute_command("pick the purple cube in zone B"))

        printed.assert_called_once_with(
            "TASK FAILED: unsupported or low-level request; no LLM call or motion", flush=True)
        self.runtime._snapshot_source.assert_not_called()
        self.runtime._synchronizer.apply_snapshot.assert_not_called()
        self.runtime._planner.assert_not_called()
        self.runtime._skills.clear_temporary_position.assert_not_called()
        self.assertEqual(len(self.runtime._node.samples), 1)

    def test_future_frame_is_not_accepted_or_recorded(self):
        self.runtime._node.samples.extend((detections(A, 14.0), detections(A, 14.1)))
        self.runtime._node.timestamps.extend((14.0, 14.1))
        with patch.object(runtime_module.rclpy, "ok", side_effect=(True, False)), \
                patch.object(runtime_module.rclpy, "spin_once"):
            with self.assertRaisesRegex(RuntimeError, "No stable fresh RGB snapshot"):
                self.runtime._snapshot_source()
        self.assertEqual(self.runtime._last_stamp, 9.0)

    def test_unexpected_perception_error_propagates(self):
        self.runtime._node.samples.extend((detections(A, 12.4), detections(A, 12.5)))
        self.runtime._node.timestamps.extend((12.4, 12.5))
        error = PerceptionStateError(PerceptionStateStatus.INVALID, "bad camera data")
        with patch.object(runtime_module.rclpy, "ok", return_value=True), \
                patch.object(runtime_module.rclpy, "spin_once"), \
                patch.object(runtime_module.PerceptionSnapshot, "require_fresh",
                             side_effect=error):
            with self.assertRaises(PerceptionStateError) as caught:
                self.runtime._snapshot_source()
        self.assertIs(caught.exception, error)
        self.assertEqual(self.runtime._last_stamp, 9.0)

    def test_sequential_valid_transactions_clear_window_and_complete(self):
        self.runtime._skills = Mock(reserved_temporary=None)
        self.runtime._physical = Mock()
        self.runtime._physical.ready_for_pick.return_value = True
        self.runtime._synchronizer = Mock()
        self.runtime._synchronizer.apply_snapshot.return_value = SimpleNamespace(
            attached_ids=[], authoritative_xyz={name: (0, 0, 0) for name in BLOCKS},
            max_sync_error_m=0.0)
        self.runtime._planner = Mock()
        home = {name: 0.0 for name in ARM_JOINTS}
        self.runtime._motion = {"home": home}
        self.runtime._interface = Mock()
        self.runtime._interface.current_joint_positions.return_value = dict(
            home, **{name: OPEN_POSITION for name in FINGER_JOINTS})
        initial = snapshot(A, self.runtime._geometry, 12.5)
        red_final = snapshot(dict(A, red_cube=(0.0, 0.24)), self.runtime._geometry, 12.6)
        purple_final = snapshot(dict(A, purple_cube=(0.0, 0.24)), self.runtime._geometry, 12.7)
        snapshots = iter((initial, red_final, initial, purple_final))

        def acquire():
            self.assertEqual(len(self.runtime._node.samples), 0)
            self.assertEqual(len(self.runtime._node.timestamps), 0)
            return next(snapshots)

        self.runtime._snapshot_source = Mock(side_effect=acquire)

        def completed(command, _initial, _geometry, state, *_args):
            goal = runtime_module.extract_placement_goal(command)
            state.record_pick_success(goal.object_name)
            state.record_place_success(goal.object_name, goal.destination_zone)
            return "validated", SimpleNamespace(status=TaskStatus.SUCCESS, completed_steps=3)

        with patch.object(runtime_module, "plan_and_execute", side_effect=completed) as plan, \
                patch("builtins.print"):
            for command in ("put the red cube in zone B", "put the purple cube in zone B"):
                self.runtime._node.samples.append(object())
                self.runtime._node.timestamps.append(9.0)
                self.assertTrue(self.runtime.execute_command(command))
                self.assertEqual(len(self.runtime._node.samples), 0)
                self.assertEqual(len(self.runtime._node.timestamps), 0)

        self.assertEqual(plan.call_count, 2)
        self.assertEqual(self.runtime._snapshot_source.call_count, 4)
        self.assertEqual(self.runtime._synchronizer.apply_snapshot.call_count, 4)


if __name__ == "__main__":
    unittest.main()
