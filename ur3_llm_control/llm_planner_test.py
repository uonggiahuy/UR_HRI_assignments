"""M10 static and optional live tests for the LLM-to-M8 validation boundary."""

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
    """Make the requested M10 calls only when explicitly invoked with --live."""
    planner = LLMPlanner.from_environment()
    cases = (
        ("Đặt khối đỏ vào vùng B rồi về home.", (
            {"skill": "pick", "object": "red_cube"},
            {"skill": "place", "object": "red_cube", "zone": "zone_b"},
            {"skill": "home"},
        )),
        ("Move the blue cube to zone A.", (
            {"skill": "pick", "object": "blue_cube"},
            {"skill": "place", "object": "blue_cube", "zone": "zone_a"},
        )),
        ("Về vị trí home.", ({"skill": "home"},)),
    )
    for request, expected_steps in cases:
        result = planner.plan(request, _fresh_world_state())
        if not result.accepted or result.validation.steps != expected_steps:
            raise RuntimeError(f"live planner validation failed for request: {request}")
    print("M10 live planner tests passed; no robot actions were executed.")


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
