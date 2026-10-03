"""Assignment 03 RGB-symbolic, planning-only LLM boundary."""

from __future__ import annotations

import re
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Mapping

from ur3_perception_llm_control.llm_planner import (
    LLMPlanner, PlannerResult, PlannerStatus, _package_file, _read_text, resolve_llm_config,
)
from ur3_perception_llm_control.perception_state import PerceptionSnapshot, WorkcellGeometry
from ur3_perception_llm_control.task_validator import TaskValidator
from ur3_perception_llm_control.world_state import BLOCKS, ZONES, WorldState


ZONE_ORDER = ("zone_a", "zone_b", "zone_c")
_COLORS = ("red", "yellow", "blue", "green", "purple")
_LOW_LEVEL = re.compile(
    r"\b(joint|angle|degree|coordinate|trajectory|velocity|acceleration|controller|"
    r"gripper|motor|camera|gazebo|moveit|pose|orientation|pixel|ignore|instruction)\b"
    r"|\b[xyz]\s*=", re.IGNORECASE,
)
_PLACEMENT = re.compile(r"\b(put|move|place|transfer)\b", re.IGNORECASE)


@dataclass(frozen=True)
class PlacementGoal:
    object_name: str
    destination_zone: str


def extract_placement_goal(command: str) -> PlacementGoal | None:
    """Accept one explicit English placement target and one destination."""
    if not isinstance(command, str) or not _PLACEMENT.search(command) or _LOW_LEVEL.search(command):
        return None
    normalized = command.casefold().replace("_", " ")
    mentioned_colors = [color for color in _COLORS if re.search(rf"\b{color}\b", normalized)]
    objects = [f"{color}_cube" for color in mentioned_colors
               if re.search(rf"\b{color}\s+(?:cube|block)\b", normalized)]
    zones = [f"zone_{letter}" for letter in "abc"
             if re.search(rf"\b(?:zone|area)\s+{letter}\b", normalized)]
    if len(mentioned_colors) != 1 or len(objects) != 1 or len(zones) != 1:
        return None
    return PlacementGoal(objects[0], zones[0])


def build_scene_context(state: WorldState) -> str:
    """Serialize only symbolic task state in explicit, repeatable order."""
    if (set(state.object_locations) != set(BLOCKS) or set(state.zone_occupancy) != ZONES
            or state.held_object is not None):
        raise ValueError("complete unheld five-block state required")
    lines = ["Objects:"]
    lines.extend(f"  {name}: {state.object_locations[name]}" for name in BLOCKS)
    lines.append("Zones:")
    lines.extend(f"  {zone}: {state.zone_occupancy[zone] or 'empty'}" for zone in ZONE_ORDER)
    lines.append("Held object: none")
    return "\n".join(lines)


def expected_goal_steps(goal: PlacementGoal, state: WorldState) -> tuple[dict[str, str], ...]:
    """Bound the supported task to the minimal safe public-skill sequence."""
    if state.object_locations[goal.object_name] == goal.destination_zone:
        return ({"skill": "home"},)
    steps: list[dict[str, str]] = []
    blocker = state.zone_occupancy[goal.destination_zone]
    if blocker is not None:
        steps.extend(({"skill": "pick", "object": blocker},
                      {"skill": "place_temp", "object": blocker}))
    steps.extend(({"skill": "pick", "object": goal.object_name},
                  {"skill": "place", "object": goal.object_name,
                   "zone": goal.destination_zone}, {"skill": "home"}))
    return tuple(steps)


def simulate_goal(steps: tuple[dict[str, str], ...], state: WorldState,
                  goal: PlacementGoal) -> tuple[bool, WorldState]:
    """Pure symbolic simulation after TaskValidator has checked semantics."""
    simulated = state.copy()
    for step in steps:
        if step["skill"] == "pick":
            simulated.record_pick_success(step["object"])
        elif step["skill"] == "place_temp":
            simulated.record_temp_place_success(step["object"])
        elif step["skill"] == "place":
            simulated.record_place_success(step["object"], step["zone"])
    satisfied = (simulated.object_locations[goal.object_name] == goal.destination_zone
                 and simulated.zone_occupancy[goal.destination_zone] == goal.object_name
                 and simulated.held_object is None)
    return satisfied, simulated


class SceneAwareLLMPlanner(LLMPlanner):
    """Generate and verify one Assignment 03 plan from a fresh RGB snapshot."""

    @classmethod
    def from_environment(cls, environ: Mapping[str, str] | None = None,
                         config_path: str | Path | None = None,
                         prompt_path: str | Path | None = None,
                         client_factory: Callable[..., Any] | None = None) -> "SceneAwareLLMPlanner":
        prompt = prompt_path or _package_file("prompt", "assignment03_system_prompt.txt")
        environment = dict(os.environ if environ is None else environ)
        # Assignment 03 also accepts the OpenAI-compatible names supplied by
        # 9Router sessions; the legacy planner keeps its existing configuration.
        environment.setdefault("NINEROUTER_BASE_URL", environment.get("OPENAI_BASE_URL", ""))
        environment.setdefault("NINEROUTER_API_KEY", environment.get("OPENAI_API_KEY", ""))
        environment.setdefault("NINEROUTER_MODEL", "oc/muse-spark-1.2-contributor-free")
        return cls(resolve_llm_config(environment, config_path),
                   _read_text(prompt, "Assignment 03 prompt"), client_factory)

    def plan(self, request: str, snapshot: PerceptionSnapshot,
             now_sec: float, geometry: WorkcellGeometry) -> PlannerResult:
        try:
            state = WorldState.from_perception_snapshot(snapshot, now_sec, geometry)
            context = build_scene_context(state)
        except (AttributeError, KeyError, TypeError, ValueError) as error:
            return PlannerResult(PlannerStatus.INVALID_REQUEST, f"invalid perception: {error}")
        goal = extract_placement_goal(request)
        if goal is None:
            return PlannerResult(PlannerStatus.INVALID_REQUEST, "unsupported or ambiguous placement request")
        candidate_result = self._request_candidate(request, context, allow_markdown=False)
        if candidate_result.status != PlannerStatus.SUCCESS:
            return candidate_result
        candidate = candidate_result.candidate_json
        assert candidate is not None
        validation = TaskValidator(assignment03=True).validate(candidate, state)
        if not validation.accepted:
            return PlannerResult(PlannerStatus.VALIDATION_FAILED, validation.message,
                                 validation, candidate)
        satisfied, _ = simulate_goal(validation.steps, state, goal)
        if not satisfied or validation.steps != expected_goal_steps(goal, state):
            return PlannerResult(PlannerStatus.VALIDATION_FAILED,
                                 "candidate does not achieve the requested minimal placement goal",
                                 candidate_json=candidate)
        return PlannerResult(PlannerStatus.SUCCESS, "Assignment 03 goal validated",
                             validation, candidate)
