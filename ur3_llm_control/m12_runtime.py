"""M12 fail-closed natural-language command orchestration.

This module is the only layer that connects the planner to execution.  It
does not add robot motion primitives: an accepted M8 validation is passed
unchanged to ``SkillExecutor``, which is the sole caller of ``RobotSkills``.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from enum import Enum
from typing import Protocol

from ur3_llm_control.llm_planner import PlannerResult
from ur3_llm_control.skill_executor import ExecutionResult, SkillExecutor
from ur3_llm_control.student_task import StudentConfigurationError, StudentIdError, resolve_student_task
from ur3_llm_control.task_validator import TaskStatus
from ur3_llm_control.world_state import OBJECTS, TABLE, ZONES, WorldState


class RuntimeStatus(str, Enum):
    SUCCESS = "SUCCESS"
    PLANNER_FAILED = "PLANNER_FAILED"
    EXECUTION_FAILED = "EXECUTION_FAILED"
    ARRANGEMENT_UNSAFE = "ARRANGEMENT_UNSAFE"
    STUDENT_ID_ERROR = "STUDENT_ID_ERROR"


@dataclass(frozen=True)
class RuntimeResult:
    status: RuntimeStatus
    message: str
    planner: PlannerResult | None = None
    execution: ExecutionResult | None = None
    plan: tuple[dict[str, str], ...] = ()
    student_variant: int | None = None
    mapping: dict[str, str] | None = None


class PlannerProtocol(Protocol):
    def plan(self, request: str, world_state: WorldState, planning_context: str | None = None) -> PlannerResult: ...


class ArrangementError(ValueError):
    """Raised before motion when the public-skill state machine cannot rearrange safely."""


# The centre-row yellow source is vulnerable once another cube occupies the
# front-row zone_b.  Clear it before placing any assignment target; red and
# blue then retain a stable deterministic order for every M9 permutation.
TRANSFER_ORDER = ("yellow_cube", "red_cube", "blue_cube")


def is_student_arrangement_request(command: str) -> bool:
    """Recognize only an unambiguous supported high-level arrangement request."""
    normalized = " ".join(command.casefold().split())
    return normalized in {
        "arrange all objects according to my student id.",
        "arrange all objects according to my student id",
        "arrange all objects according to my student-id.",
        "arrange all objects according to my student-id",
    }


def arrangement_steps(world_state: WorldState, zone_to_object: dict[str, str]) -> tuple[dict[str, str], ...]:
    """Build the minimum safe plan from deterministic M9 assignment data.

    Existing correctly placed cubes are skipped.  M7/M8 deliberately support
    picking only a table object and placing only into an empty zone.  Therefore
    an occupied wrong zone is rejected before any motion rather than being
    overwritten or relocated with invented coordinates.
    """
    if set(zone_to_object) != ZONES or set(zone_to_object.values()) != OBJECTS:
        raise ArrangementError("M9 mapping is not a complete object/zone permutation")
    if world_state.held_object is not None:
        raise ArrangementError("cannot arrange while an object is held")

    object_to_zone = {object_name: zone for zone, object_name in zone_to_object.items()}
    steps: list[dict[str, str]] = []
    for object_name in TRANSFER_ORDER:
        target_zone = object_to_zone[object_name]
        location = world_state.object_locations[object_name]
        if location == target_zone:
            continue
        occupant = world_state.zone_occupancy[target_zone]
        if occupant is not None and occupant != object_name:
            raise ArrangementError(
                f"target {target_zone} contains {occupant}; safe relocation is not a public M7/M8 skill"
            )
        if location != TABLE:
            raise ArrangementError(
                f"{object_name} is at {location}; public M7/M8 skills do not pick from placement zones"
            )
        steps.extend((
            {"skill": "pick", "object": object_name},
            {"skill": "place", "object": object_name, "zone": target_zone},
        ))
    steps.append({"skill": "home"})
    return tuple(steps)


def arrangement_context(steps: tuple[dict[str, str], ...]) -> str:
    """Give the LLM an already-resolved, non-sensitive assignment constraint."""
    return (
        "Trusted deterministic application context: the student-ID mapping was already "
        "computed in Python. Do not calculate an ID, modulo, mapping, coordinates, or "
        "low-level command. Return exactly this allowed public-skill JSON plan and nothing else: "
        + json.dumps({"plan": list(steps)}, separators=(",", ":"))
    )


class M12Runtime:
    """One-session orchestrator retaining only authoritative WorldState."""

    def __init__(self, planner: PlannerProtocol, executor: SkillExecutor, world_state: WorldState) -> None:
        self._planner = planner
        self._executor = executor
        self.world_state = world_state

    def run(self, command: str, runtime_student_id: str | None = None) -> RuntimeResult:
        if is_student_arrangement_request(command):
            return self._run_arrangement(command, runtime_student_id)
        return self._run_standard(command)

    def _run_standard(self, command: str) -> RuntimeResult:
        planner_result = self._planner.plan(command, self.world_state)
        if not planner_result.accepted:
            return RuntimeResult(RuntimeStatus.PLANNER_FAILED, planner_result.message, planner=planner_result)
        return self._execute(planner_result)

    def _run_arrangement(self, command: str, runtime_student_id: str | None) -> RuntimeResult:
        try:
            resolution = resolve_student_task(runtime_student_id)
            steps = arrangement_steps(self.world_state, resolution.mapping)
        except (StudentConfigurationError, StudentIdError) as error:
            return RuntimeResult(RuntimeStatus.STUDENT_ID_ERROR, str(error))
        except ArrangementError as error:
            return RuntimeResult(RuntimeStatus.ARRANGEMENT_UNSAFE, str(error))

        planner_result = self._planner.plan(command, self.world_state, arrangement_context(steps))
        if not planner_result.accepted:
            return RuntimeResult(
                RuntimeStatus.PLANNER_FAILED, planner_result.message, planner=planner_result,
                student_variant=resolution.variant, mapping=resolution.mapping,
            )
        # The Python-derived goals are authoritative.  This prevents the LLM
        # from selecting a different student mapping while still allowing it
        # to be the NL-to-public-skills planner.
        if planner_result.validation.steps != steps:
            return RuntimeResult(
                RuntimeStatus.PLANNER_FAILED, "LLM plan does not match Python-resolved assignment",
                planner=planner_result, student_variant=resolution.variant, mapping=resolution.mapping,
            )
        return self._execute(planner_result, resolution.variant, resolution.mapping)

    def _execute(
        self, planner_result: PlannerResult, variant: int | None = None, mapping: dict[str, str] | None = None,
    ) -> RuntimeResult:
        assert planner_result.validation is not None
        execution = self._executor.execute(planner_result.validation)
        if execution.status != TaskStatus.SUCCESS:
            return RuntimeResult(
                RuntimeStatus.EXECUTION_FAILED, execution.message, planner_result, execution,
                planner_result.validation.steps, variant, mapping,
            )
        return RuntimeResult(
            RuntimeStatus.SUCCESS, execution.message, planner_result, execution,
            planner_result.validation.steps, variant, mapping,
        )
