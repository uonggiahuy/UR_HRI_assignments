"""Fail-closed execution of an accepted M8 plan through RobotSkills only."""

from __future__ import annotations

from dataclasses import dataclass
import logging
from typing import Protocol

from ur3_perception_llm_control.task_validator import TaskStatus, ValidationResult
from ur3_perception_llm_control.world_state import TABLE, WorldState


LOGGER = logging.getLogger(__name__)


class RobotSkillsProtocol(Protocol):
    def home(self): ...
    def pick(self, object_name: str): ...
    def place(self, object_name: str, zone_name: str): ...


@dataclass(frozen=True)
class ExecutionResult:
    status: TaskStatus
    message: str
    completed_steps: int


class SkillExecutor:
    """Call only ``home``, ``pick``, and ``place`` on the supplied RobotSkills."""

    def __init__(self, skills: RobotSkillsProtocol, world_state: WorldState) -> None:
        self._skills = skills
        self._world_state = world_state

    def execute(self, validation: ValidationResult) -> ExecutionResult:
        if not validation.accepted:
            return ExecutionResult(validation.status, validation.message, 0)
        if validation.world_revision != self._world_state.revision:
            return ExecutionResult(TaskStatus.INVALID_PLAN, "stale plan rejected", 0)

        total_steps = len(validation.steps)
        LOGGER.info("VALIDATED PLAN LENGTH = %d", total_steps)
        for index, step in enumerate(validation.steps, start=1):
            LOGGER.info("START step %d/%d: %s", index, total_steps, step)
            precondition = self._runtime_precondition(step)
            if precondition is not None:
                LOGGER.info("STOP step %d/%d: %s", index, total_steps, precondition.value)
                return ExecutionResult(precondition, f"step {index} semantic precondition failed", index - 1)
            result = self._call_skill(step)
            if self._status_value(result) != TaskStatus.SUCCESS.value:
                LOGGER.info("RESULT step %d/%d: %s; STOP", index, total_steps, self._status_value(result))
                return ExecutionResult(TaskStatus.SKILL_FAILED, f"step {index} skill failed", index - 1)
            LOGGER.info("RESULT step %d/%d: SUCCESS; CONTINUE", index, total_steps)
            self._record_success(step)
        return ExecutionResult(TaskStatus.SUCCESS, "TASK SUCCESS", len(validation.steps))

    def _runtime_precondition(self, step: dict[str, str]) -> TaskStatus | None:
        if step["skill"] == "pick":
            if self._world_state.held_object is not None:
                return TaskStatus.ALREADY_HOLDING_OBJECT
            if self._world_state.object_locations[step["object"]] != TABLE:
                return TaskStatus.INVALID_PLAN
        elif step["skill"] == "place":
            if self._world_state.held_object != step["object"]:
                return TaskStatus.OBJECT_NOT_HELD
            if self._world_state.zone_occupancy[step["zone"]] is not None:
                return TaskStatus.ZONE_OCCUPIED
        return None

    def _call_skill(self, step: dict[str, str]):
        if step["skill"] == "home":
            return self._skills.home()
        if step["skill"] == "pick":
            return self._skills.pick(step["object"])
        return self._skills.place(step["object"], step["zone"])

    def _record_success(self, step: dict[str, str]) -> None:
        if step["skill"] == "pick":
            self._world_state.record_pick_success(step["object"])
        elif step["skill"] == "place":
            self._world_state.record_place_success(step["object"], step["zone"])

    @staticmethod
    def _status_value(result: object) -> str:
        return str(getattr(result, "value", result))
