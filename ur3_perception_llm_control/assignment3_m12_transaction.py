"""One Assignment 03 LLM planning transaction and its motion eligibility gate."""

from __future__ import annotations

from typing import Callable

from ur3_perception_llm_control.perception_state import PerceptionSnapshot, WorkcellGeometry
from ur3_perception_llm_control.scene_aware_planner import (
    SceneAwareLLMPlanner, expected_goal_steps, extract_placement_goal, simulate_goal,
)
from ur3_perception_llm_control.skill_executor import ExecutionResult, SkillExecutor
from ur3_perception_llm_control.task_validator import TaskValidator
from ur3_perception_llm_control.world_state import WorldState


def verify_placement_observation(snapshot: PerceptionSnapshot | None,
                                 state: WorldState, physical_ready: bool) -> None:
    """Stop the task if a completed placement differs from the expected prefix."""
    if (snapshot is None or dict(snapshot.object_locations) != state.object_locations
            or dict(snapshot.zone_occupancy) != state.zone_occupancy
            or snapshot.held_object is not None or not physical_ready):
        raise RuntimeError("post-place camera/physical mismatch")


def plan_and_execute(
    command: str,
    initial: PerceptionSnapshot,
    geometry: WorkcellGeometry,
    state: WorldState,
    planner: SceneAwareLLMPlanner,
    executor: SkillExecutor,
    snapshot_source: Callable[[], PerceptionSnapshot],
    now_sec: Callable[[], float],
    before_execute: Callable[[PerceptionSnapshot], None],
    on_validated: Callable[[str], None] | None = None,
) -> tuple[str, ExecutionResult]:
    """Execute exactly the fresh candidate, after a second camera and scene check."""
    planned = planner.plan(command, initial, now_sec(), geometry)
    if not planned.accepted or planned.candidate_json is None or planned.validation is None:
        raise RuntimeError(f"LLM plan rejected: {planned.status.value}: {planned.message}")
    candidate = planned.candidate_json
    validation = planned.validation
    # Recheck the raw response at the execution boundary, including its goal.
    checked = TaskValidator(assignment03=True).validate(candidate, state)
    goal = extract_placement_goal(command)
    if (goal is None or not checked.accepted or checked != validation
            or checked.steps != expected_goal_steps(goal, state)
            or not simulate_goal(checked.steps, state, goal)[0]):
        raise RuntimeError("validated candidate failed execution-boundary goal check")
    if checked.world_revision != state.revision or checked.world_signature != state.signature():
        raise RuntimeError("STALE_PLAN: world revision or signature changed")
    current = snapshot_source()
    if not state.matches_perception_snapshot(current, now_sec()):
        raise RuntimeError("STALE_PLAN: camera layout changed before execution")
    before_execute(current)
    if on_validated is not None:
        on_validated(candidate)
    return candidate, executor.execute(validation)
