"""Static M12 orchestration tests; no ROS graph, 9Router, or robot motion."""

from __future__ import annotations

import json
import unittest

from ur3_llm_control.llm_planner import PlannerResult, PlannerStatus
from ur3_llm_control.m12_runtime import M12Runtime, RuntimeStatus, arrangement_steps
from ur3_llm_control.skill_executor import SkillExecutor
from ur3_llm_control.student_task import get_assignment_mapping, get_object_zone_mapping, resolve_student_task
from ur3_llm_control.task_validator import TaskStatus, TaskValidator
from ur3_llm_control.world_state import OBJECTS, TABLE, ZONES, WorldState


def _world() -> WorldState:
    return WorldState(None, {name: TABLE for name in OBJECTS}, {zone: None for zone in ZONES})


class FakeSkills:
    def __init__(self) -> None:
        self.calls: list[tuple] = []

    def home(self):
        self.calls.append(("home",))
        return "SUCCESS"

    def pick(self, object_name: str):
        self.calls.append(("pick", object_name))
        return "SUCCESS"

    def place(self, object_name: str, zone_name: str):
        self.calls.append(("place", object_name, zone_name))
        return "SUCCESS"


class FakePlanner:
    def __init__(self, response: dict | None = None) -> None:
        self.response = response
        self.calls: list[tuple[str, str | None]] = []

    def plan(self, request: str, world_state: WorldState, planning_context: str | None = None) -> PlannerResult:
        self.calls.append((request, planning_context))
        if self.response is None:
            return PlannerResult(PlannerStatus.API_ERROR, "simulated 9Router failure")
        candidate = json.dumps(self.response)
        validation = TaskValidator().validate(candidate, world_state)
        status = PlannerStatus.SUCCESS if validation.accepted else PlannerStatus.VALIDATION_FAILED
        return PlannerResult(status, "ok" if validation.accepted else validation.message, validation, candidate)


class M12RuntimeTest(unittest.TestCase):
    def test_all_m9_variants_keep_mapping_with_deterministic_safe_order(self) -> None:
        for variant in range(6):
            with self.subTest(variant=variant):
                student_id = f"0000{variant:02d}"
                mapping = get_assignment_mapping(student_id)
                steps = arrangement_steps(_world(), mapping)
                self.assertEqual(steps[-1], {"skill": "home"})
                transfers = [step for step in steps if step["skill"] in ("pick", "place")]
                self.assertEqual(len(transfers), 6)
                picks = [step["object"] for step in steps if step["skill"] == "pick"]
                places = {step["object"]: step["zone"] for step in steps if step["skill"] == "place"}
                self.assertEqual(picks, ["yellow_cube", "red_cube", "blue_cube"])
                self.assertEqual(set(picks), OBJECTS)
                self.assertEqual(places, get_object_zone_mapping(student_id))
                self.assertTrue(all(step["skill"] in ("home", "pick", "place") for step in steps))

    def test_runtime_student_id_override_remains_authoritative(self) -> None:
        resolution = resolve_student_task("12345600")
        self.assertEqual(resolution.variant, 0)
        self.assertEqual(resolution.mapping, get_assignment_mapping("12345600"))

    def test_standard_plan_calls_only_validated_public_skills(self) -> None:
        state, skills = _world(), FakeSkills()
        planner = FakePlanner({"plan": [{"skill": "pick", "object": "red_cube"},
                                         {"skill": "place", "object": "red_cube", "zone": "zone_b"},
                                         {"skill": "home"}]})
        result = M12Runtime(planner, SkillExecutor(skills, state), state).run("Đặt khối đỏ vào vùng B rồi về home.")
        self.assertEqual(result.status, RuntimeStatus.SUCCESS)
        self.assertEqual(skills.calls, [("pick", "red_cube"), ("place", "red_cube", "zone_b"), ("home",)])
        self.assertEqual(state.object_locations["red_cube"], "zone_b")
        self.assertEqual(state.zone_occupancy["zone_b"], "red_cube")

    def test_planner_failure_cannot_execute_or_reuse_a_previous_plan(self) -> None:
        state, skills = _world(), FakeSkills()
        accepted = {"plan": [{"skill": "home"}]}
        runtime = M12Runtime(FakePlanner(accepted), SkillExecutor(skills, state), state)
        self.assertEqual(runtime.run("go home").status, RuntimeStatus.SUCCESS)
        # Replace the planner response after a successful execution.  The
        # second failure must cause no additional RobotSkills invocation.
        runtime._planner.response = None  # test-only controlled fake
        failure = runtime.run("Move joint 2 to 30 degrees.")
        self.assertEqual(failure.status, RuntimeStatus.PLANNER_FAILED)
        self.assertEqual(skills.calls, [("home",)])
        self.assertEqual(state.revision, 0)

    def test_python_resolves_advanced_mapping_and_llm_must_match_it(self) -> None:
        state, skills = _world(), FakeSkills()
        expected = arrangement_steps(state, {
            "zone_a": "red_cube", "zone_b": "yellow_cube", "zone_c": "blue_cube",
        })
        planner = FakePlanner({"plan": list(expected)})
        result = M12Runtime(planner, SkillExecutor(skills, state), state).run(
            "Arrange all objects according to my student ID.", "0000"
        )
        self.assertEqual(result.status, RuntimeStatus.SUCCESS)
        self.assertEqual(result.student_variant, 0)
        self.assertEqual(result.mapping, {"zone_a": "red_cube", "zone_b": "yellow_cube", "zone_c": "blue_cube"})
        self.assertIn('"object":"red_cube","zone":"zone_a"', planner.calls[0][1])
        self.assertIsNone(state.held_object)
        self.assertEqual(state.zone_occupancy, {"zone_a": "red_cube", "zone_b": "yellow_cube", "zone_c": "blue_cube"})

    def test_occupied_or_zone_resident_object_fails_before_planner_or_motion(self) -> None:
        state, skills = _world(), FakeSkills()
        state.object_locations["red_cube"] = "zone_b"
        state.zone_occupancy["zone_b"] = "red_cube"
        planner = FakePlanner({"plan": [{"skill": "home"}]})
        result = M12Runtime(planner, SkillExecutor(skills, state), state).run(
            "Arrange all objects according to my student ID.", "0000"
        )
        self.assertEqual(result.status, RuntimeStatus.ARRANGEMENT_UNSAFE)
        self.assertEqual(planner.calls, [])
        self.assertEqual(skills.calls, [])
        self.assertEqual(state.object_locations["red_cube"], "zone_b")

    def test_mismatched_llm_assignment_is_rejected_before_execution(self) -> None:
        state, skills = _world(), FakeSkills()
        planner = FakePlanner({"plan": [{"skill": "home"}]})
        result = M12Runtime(planner, SkillExecutor(skills, state), state).run(
            "Arrange all objects according to my student ID.", "0000"
        )
        self.assertEqual(result.status, RuntimeStatus.PLANNER_FAILED)
        self.assertEqual(skills.calls, [])


if __name__ == "__main__":
    unittest.main()
