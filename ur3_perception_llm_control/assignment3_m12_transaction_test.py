"""M12 execution-boundary regressions; no ROS graph or robot motion."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
import unittest

from ur3_perception_llm_control.assignment3_m12_transaction import (
    plan_and_execute, verify_placement_observation,
)
from ur3_perception_llm_control.perception_scene_unit_test import A, snapshot
from ur3_perception_llm_control.perception_state import WorkcellGeometry
from ur3_perception_llm_control.scene_aware_planner import SceneAwareLLMPlanner
from ur3_perception_llm_control.skill_executor import ExecutionResult
from ur3_perception_llm_control.task_validator import TaskStatus
from ur3_perception_llm_control.world_state import WorldState


ROOT = Path(__file__).resolve().parents[1]
OCCUPIED = ('{"plan":[{"skill":"pick","object":"blue_cube"},'
            '{"skill":"place_temp","object":"blue_cube"},'
            '{"skill":"pick","object":"red_cube"},'
            '{"skill":"place","object":"red_cube","zone":"zone_b"},'
            '{"skill":"home"}]}')
GREEN = ('{"plan":[{"skill":"pick","object":"green_cube"},'
         '{"skill":"place","object":"green_cube","zone":"zone_a"},'
         '{"skill":"home"}]}')


class Client:
    def __init__(self, candidate):
        self.candidate = candidate
        self.calls = 0
        self.chat = SimpleNamespace(completions=self)

    def create(self, **_):
        self.calls += 1
        return SimpleNamespace(choices=[SimpleNamespace(message=SimpleNamespace(
            content=self.candidate))])


class Executor:
    def __init__(self):
        self.calls = []

    def execute(self, validation):
        self.calls.append(validation)
        return ExecutionResult(TaskStatus.SUCCESS, "TASK SUCCESS", len(validation.steps))


class M12TransactionTest(unittest.TestCase):
    def setUp(self):
        self.geometry = WorkcellGeometry.from_scene_file(ROOT / "config/scene.yaml")
        self.initial = snapshot(dict(A, blue_cube=(0.0, 0.24)), self.geometry)
        self.state = WorldState.from_perception_snapshot(self.initial, 10.1, self.geometry)
        self.executor = Executor()
        self.before_calls = []

    def run_candidate(self, candidate, *, command="Put the red cube in Zone B.",
                      current=None):
        client = Client(candidate)
        planner = SceneAwareLLMPlanner.from_environment(
            environ={"NINEROUTER_API_KEY": "test-only", "NINEROUTER_MODEL": "test-model"},
            client_factory=lambda **_: client)
        original_plan = planner.plan

        def recorded_plan(*args):
            result = original_plan(*args)
            self.planned_validation = result.validation
            return result

        planner.plan = recorded_plan
        observed = self.initial if current is None else current
        try:
            result = plan_and_execute(
                command, self.initial, self.geometry, self.state, planner, self.executor,
                lambda: observed, lambda: 10.1, self.before_calls.append)
            return result, client
        except RuntimeError:
            return None, client

    def test_occupied_raw_candidate_is_exact_execution_plan(self):
        result, client = self.run_candidate(OCCUPIED)
        self.assertEqual(result[0], OCCUPIED)
        self.assertEqual(client.calls, 1)
        self.assertEqual(self.executor.calls[0].steps[1],
                         {"skill": "place_temp", "object": "blue_cube"})
        self.assertIs(self.executor.calls[0], self.planned_validation)
        self.assertEqual(len(self.executor.calls), 1)
        self.assertEqual(len(self.before_calls), 1)

    def test_failures_never_call_executor_or_fallback(self):
        for candidate in ("{", '{"plan":[{"skill":"pick","object":"red_cube","x":0.1}]}',
                          '{"plan":[{"skill":"home"}]}'):
            with self.subTest(candidate=candidate):
                self.assertIsNone(self.run_candidate(candidate)[0])
                self.assertEqual(self.executor.calls, [])
                self.assertEqual(self.before_calls, [])

    def test_stale_camera_rejected_before_execution(self):
        changed = snapshot(dict(A, blue_cube=(0.0, 0.24), red_cube=(-0.19, 0.5)),
                           self.geometry)
        self.assertIsNone(self.run_candidate(OCCUPIED, current=changed)[0])
        self.assertEqual(self.executor.calls, [])
        self.assertEqual(self.before_calls, [])

    def test_low_level_request_makes_no_llm_call(self):
        result, client = self.run_candidate(OCCUPIED, command="Move joint 2 to 30 degrees.")
        self.assertIsNone(result)
        self.assertEqual(client.calls, 0)
        self.assertEqual(self.executor.calls, [])

    def test_five_block_direct_placement(self):
        self.initial = snapshot(A, self.geometry)
        self.state = WorldState.from_perception_snapshot(self.initial, 10.1, self.geometry)
        result, _ = self.run_candidate(GREEN, command="Put the green cube in Zone A.")
        self.assertEqual(result[1].status, TaskStatus.SUCCESS)
        self.assertEqual(len(self.executor.calls[0].steps), 3)

    def test_post_place_mismatch_stops_task(self):
        expected = self.state.copy()
        expected.record_pick_success("blue_cube")
        expected.record_temp_place_success("blue_cube")
        valid = snapshot(A, self.geometry)
        verify_placement_observation(valid, expected, True)
        for observed, physical_ready in ((self.initial, True), (valid, False), (None, True)):
            with self.assertRaisesRegex(RuntimeError, "post-place"):
                verify_placement_observation(observed, expected, physical_ready)


if __name__ == "__main__":
    unittest.main()
