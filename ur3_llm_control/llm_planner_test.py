"""M11 static and authenticated live robustness checks for LLM-to-M8 planning."""

from __future__ import annotations

import argparse
from types import SimpleNamespace
import unittest

from ur3_llm_control.llm_planner import (
    LLMPlanner,
    PlannerConfigurationError,
    PlannerStatus,
    resolve_llm_config,
)
from ur3_llm_control.world_state import OBJECTS, TABLE, ZONES, WorldState


RED_TO_B = (
    {"skill": "pick", "object": "red_cube"},
    {"skill": "place", "object": "red_cube", "zone": "zone_b"},
)
BLUE_TO_A = (
    {"skill": "pick", "object": "blue_cube"},
    {"skill": "place", "object": "blue_cube", "zone": "zone_a"},
)


class FakeCompletions:
    def __init__(self, contents: list[str]) -> None:
        self.contents = list(contents)
        self.calls: list[dict] = []

    def create(self, **kwargs):
        self.calls.append(kwargs)
        content = self.contents.pop(0)
        return SimpleNamespace(choices=[SimpleNamespace(message=SimpleNamespace(content=content))])


class FakeClient:
    def __init__(self, contents: list[str]) -> None:
        self.chat = SimpleNamespace(completions=FakeCompletions(contents))


def _fresh_world_state() -> WorldState:
    return WorldState(
        held_object=None,
        object_locations={object_name: TABLE for object_name in OBJECTS},
        zone_occupancy={zone_name: None for zone_name in ZONES},
    )


def _test_config() -> dict[str, str]:
    return {
        "NINEROUTER_API_KEY": "test-key-not-a-secret",
        "NINEROUTER_MODEL": "test-model",
    }


class LLMPlannerTest(unittest.TestCase):
    def _planner(self, contents: list[str]) -> tuple[LLMPlanner, FakeClient]:
        client = FakeClient(contents)
        planner = LLMPlanner.from_environment(
            environ=_test_config(),
            client_factory=lambda **_kwargs: client,
        )
        return planner, client

    def test_vietnamese_plan_is_validated_and_uses_configured_model(self) -> None:
        planner, client = self._planner([
            '{"plan":[{"skill":"pick","object":"red_cube"},'
            '{"skill":"place","object":"red_cube","zone":"zone_b"},{"skill":"home"}]}'
        ])
        result = planner.plan("Đặt khối đỏ vào vùng B rồi về home.", _fresh_world_state())
        self.assertTrue(result.accepted)
        self.assertEqual(result.validation.steps, (
            {"skill": "pick", "object": "red_cube"},
            {"skill": "place", "object": "red_cube", "zone": "zone_b"},
            {"skill": "home"},
        ))
        self.assertEqual(client.chat.completions.calls[0]["model"], "test-model")

    def test_prompt_requires_multilingual_fail_closed_behavior(self) -> None:
        planner, _ = self._planner(['{"plan":[{"skill":"home"}]}'])
        prompt = planner._system_prompt
        for required_text in (
            "Vietnamese, English, or mixed-language",
            "đỏ/red",
            "vàng/yellow",
            "xanh dương/blue",
            "Do not infer a missing object, zone, referent",
            '{"plan":[]}',
        ):
            self.assertIn(required_text, prompt)

    def test_english_plan_is_validated(self) -> None:
        planner, _ = self._planner([
            '{"plan":[{"skill":"pick","object":"blue_cube"},'
            '{"skill":"place","object":"blue_cube","zone":"zone_a"}]}'
        ])
        result = planner.plan("Move the blue cube to zone A.", _fresh_world_state())
        self.assertTrue(result.accepted)
        self.assertEqual(result.validation.steps[-1], {
            "skill": "place", "object": "blue_cube", "zone": "zone_a"
        })

    def test_home_plan_is_validated(self) -> None:
        planner, _ = self._planner(['{"plan":[{"skill":"home"}]}'])
        result = planner.plan("Về vị trí home.", _fresh_world_state())
        self.assertTrue(result.accepted)
        self.assertEqual(result.validation.steps, ({"skill": "home"},))

    def test_malformed_and_low_level_candidates_are_rejected(self) -> None:
        planner, _ = self._planner([
            "not JSON",
            '{"plan":[{"skill":"joint_trajectory","joints":[0,0,0,0,0,0]}]}',
        ])
        malformed = planner.plan("anything", _fresh_world_state())
        unsafe = planner.plan("anything", _fresh_world_state())
        self.assertEqual(malformed.status, PlannerStatus.MALFORMED_RESPONSE)
        self.assertEqual(unsafe.status, PlannerStatus.VALIDATION_FAILED)
        self.assertFalse(unsafe.validation.accepted)

    def test_missing_credentials_fail_before_client_or_request(self) -> None:
        factory_calls = []
        with self.assertRaises(PlannerConfigurationError):
            LLMPlanner.from_environment(
                environ={}, client_factory=lambda **kwargs: factory_calls.append(kwargs)
            )
        self.assertEqual(factory_calls, [])

    def test_failed_second_response_never_reuses_first_plan(self) -> None:
        planner, _ = self._planner(['{"plan":[{"skill":"home"}]}', "{"])
        first = planner.plan("home", _fresh_world_state())
        second = planner.plan("home", _fresh_world_state())
        self.assertTrue(first.accepted)
        self.assertEqual(second.status, PlannerStatus.MALFORMED_RESPONSE)
        self.assertIsNone(second.candidate_json)
        self.assertIsNone(second.validation)


def _run_live() -> None:
    """Run M11 planner-only checks against authenticated 9Router; never execute."""
    planner = LLMPlanner.from_environment()
    valid_cases = (
        # Vietnamese paraphrases (five equivalent red-to-zone-B requests).
        ("Đặt khối đỏ vào vùng B.", RED_TO_B),
        ("Cho khối màu đỏ sang khu B.", RED_TO_B),
        ("Gắp cục đỏ rồi bỏ vào zone B.", RED_TO_B),
        ("Đưa red cube đến vùng B.", RED_TO_B),
        ("Chuyển khối đỏ qua B.", RED_TO_B),
        # English paraphrases (five equivalent blue-to-zone-A requests).
        ("Put the blue cube in zone A.", BLUE_TO_A),
        ("Move the blue block to area A.", BLUE_TO_A),
        ("Pick up the blue cube and place it in zone A.", BLUE_TO_A),
        ("Transfer the blue cube to zone A.", BLUE_TO_A),
        ("Take the blue block over to zone A.", BLUE_TO_A),
        # Mixed language also covers the remaining allowed color/zone pairings.
        ("Đặt red cube vào zone C.", (
            {"skill": "pick", "object": "red_cube"},
            {"skill": "place", "object": "red_cube", "zone": "zone_c"},
        )),
        ("Move khối vàng to zone B.", (
            {"skill": "pick", "object": "yellow_cube"},
            {"skill": "place", "object": "yellow_cube", "zone": "zone_b"},
        )),
        # Explicit home phrasings.
        ("về home", ({"skill": "home"},)),
        ("về vị trí ban đầu", ({"skill": "home"},)),
        ("trở về home", ({"skill": "home"},)),
        ("go home", ({"skill": "home"},)),
        # Multi-step ordering.
        ("Đặt khối đỏ vào vùng B rồi về home.", (*RED_TO_B, {"skill": "home"})),
        ("Move the blue cube to zone A, then go home.", (*BLUE_TO_A, {"skill": "home"})),
    )
    rejected_cases = (
        "Đặt khối này vào vùng B.",
        "Move it there.",
        "Gắp một khối.",
        "Move joint 2 to 30 degrees.",
        "Move the gripper to x=0.2 y=0.3.",
        "Send this trajectory ...",
        "Pick the green cube.",
    )
    for request, expected_steps in valid_cases:
        result = planner.plan(request, _fresh_world_state())
        if not result.accepted or result.validation.steps != expected_steps:
            raise RuntimeError(f"live planner validation failed for request: {request}")
    for request in rejected_cases:
        result = planner.plan(request, _fresh_world_state())
        if result.accepted:
            raise RuntimeError(f"unsafe request was accepted: {request}")
        if result.validation is not None and result.validation.accepted:
            raise RuntimeError(f"unsafe request produced an accepted validation: {request}")
    print("M11 live planner robustness tests passed; no robot actions were executed.")


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--live", action="store_true")
    parsed, remaining = parser.parse_known_args(args)
    if parsed.live:
        _run_live()
        return
    result = unittest.main(module=__name__, argv=[__name__, *remaining], exit=False).result
    if not result.wasSuccessful():
        raise SystemExit(1)


if __name__ == "__main__":
    main()
