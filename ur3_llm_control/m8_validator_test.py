"""Static M8 validator and fail-closed executor tests; no ROS graph or LLM."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import yaml

from ur3_llm_control.skill_executor import SkillExecutor
from ur3_llm_control.task_validator import TaskStatus, TaskValidator
from ur3_llm_control.world_state import WorldState


SCENE = {
    "objects": {
        name: {"pose": {"x": 0.0, "y": 0.0, "z": 0.0}}
        for name in ("red_cube", "yellow_cube", "blue_cube")
    }
}
VALID_PLAN = {
    "plan": [
        {"skill": "pick", "object": "red_cube"},
        {"skill": "place", "object": "red_cube", "zone": "zone_b"},
        {"skill": "home"},
    ]
}


class FakeRobotSkills:
    def __init__(self, results=None) -> None:
        self.calls: list[tuple] = []
        self.results = list(results or ["SUCCESS"] * 3)

    def _result(self, call: tuple) -> str:
        self.calls.append(call)
        return self.results.pop(0)

    def home(self) -> str:
        return self._result(("home",))

    def pick(self, object_name: str) -> str:
        return self._result(("pick", object_name))

    def place(self, object_name: str, zone_name: str) -> str:
        return self._result(("place", object_name, zone_name))


class M8ValidatorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls._temporary_directory = tempfile.TemporaryDirectory()
        cls.scene_path = Path(cls._temporary_directory.name) / "scene.yaml"
        cls.scene_path.write_text(yaml.safe_dump(SCENE), encoding="utf-8")

    @classmethod
    def tearDownClass(cls) -> None:
        cls._temporary_directory.cleanup()

    def setUp(self) -> None:
        self.state = WorldState.from_scene_file(self.scene_path)
        self.validator = TaskValidator()

    def _validate(self, plan) -> object:
        return self.validator.validate(json.dumps(plan), self.state)

    def test_valid_plan_and_successful_state_updates(self) -> None:
        validation = self._validate(VALID_PLAN)
        self.assertTrue(validation.accepted)
        skills = FakeRobotSkills()
        result = SkillExecutor(skills, self.state).execute(validation)
        self.assertEqual(result.status, TaskStatus.SUCCESS)
        self.assertEqual(skills.calls, [
            ("pick", "red_cube"), ("place", "red_cube", "zone_b"), ("home",),
        ])
        self.assertIsNone(self.state.held_object)
        self.assertEqual(self.state.object_locations["red_cube"], "zone_b")
        self.assertEqual(self.state.zone_occupancy["zone_b"], "red_cube")

    def test_unknown_skill_executes_zero_actions(self) -> None:
        validation = self._validate({"plan": [{"skill": "dance"}]})
        self.assertEqual(validation.status, TaskStatus.INVALID_SKILL)
        skills = FakeRobotSkills()
        result = SkillExecutor(skills, self.state).execute(validation)
        self.assertEqual(result.completed_steps, 0)
        self.assertEqual(skills.calls, [])

    def test_invalid_object(self) -> None:
        self.assertEqual(self._validate({"plan": [{"skill": "pick", "object": "green_cube"}]}).status,
                         TaskStatus.INVALID_OBJECT)

    def test_invalid_zone(self) -> None:
        plan = {"plan": [{"skill": "pick", "object": "red_cube"},
                          {"skill": "place", "object": "red_cube", "zone": "bin"}]}
        self.assertEqual(self._validate(plan).status, TaskStatus.INVALID_ZONE)

    def test_malformed_and_schema_errors(self) -> None:
        self.assertEqual(self.validator.validate("{", self.state).status, TaskStatus.INVALID_PLAN)
        self.assertEqual(self._validate({"plan": [{"skill": "pick"}]}).status, TaskStatus.INVALID_PLAN)
        self.assertEqual(self._validate({"plan": [{"skill": "home", "extra": True}]}).status,
                         TaskStatus.INVALID_PLAN)

    def test_low_level_command_is_rejected(self) -> None:
        plan = {"plan": [{"skill": "joint_trajectory", "joints": [0, 0, 0, 0, 0, 0]}]}
        self.assertEqual(self._validate(plan).status, TaskStatus.INVALID_SKILL)

    def test_place_without_holding(self) -> None:
        plan = {"plan": [{"skill": "place", "object": "red_cube", "zone": "zone_b"}]}
        self.assertEqual(self._validate(plan).status, TaskStatus.OBJECT_NOT_HELD)

    def test_occupied_zone(self) -> None:
        self.state.zone_occupancy["zone_b"] = "blue_cube"
        self.assertEqual(self._validate(VALID_PLAN).status, TaskStatus.ZONE_OCCUPIED)

    def test_executor_stops_and_does_not_update_failed_action(self) -> None:
        validation = self._validate(VALID_PLAN)
        skills = FakeRobotSkills(["SUCCESS", "FAILED", "SUCCESS"])
        result = SkillExecutor(skills, self.state).execute(validation)
        self.assertEqual(result.status, TaskStatus.SKILL_FAILED)
        self.assertEqual(skills.calls, [("pick", "red_cube"), ("place", "red_cube", "zone_b")])
        self.assertEqual(self.state.held_object, "red_cube")
        self.assertEqual(self.state.object_locations["red_cube"], "held")
        self.assertIsNone(self.state.zone_occupancy["zone_b"])

    def test_stale_accepted_plan_executes_zero_actions(self) -> None:
        validation = self._validate(VALID_PLAN)
        self.state.record_pick_success("yellow_cube")
        skills = FakeRobotSkills()
        result = SkillExecutor(skills, self.state).execute(validation)
        self.assertEqual(result.status, TaskStatus.INVALID_PLAN)
        self.assertEqual(skills.calls, [])


def main() -> None:
    result = unittest.main(module=__name__, exit=False).result
    if not result.wasSuccessful():
        raise SystemExit(1)


if __name__ == "__main__":
    main()
