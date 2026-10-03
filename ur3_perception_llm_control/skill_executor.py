"""Fail-closed execution of an accepted M8 plan through RobotSkills only."""

from __future__ import annotations

from dataclasses import dataclass
import logging
from typing import Callable, Protocol

from ur3_perception_llm_control.perception_state import PerceptionSnapshot

from ur3_perception_llm_control.task_validator import TaskStatus, ValidationResult
from ur3_perception_llm_control.world_state import TABLE, WorldState


LOGGER = logging.getLogger(__name__)


class RobotSkillsProtocol(Protocol):
    def home(self): ...
    def pick(self, object_name: str): ...
    def place(self, object_name: str, zone_name: str): ...
    def place_temp(self, object_name: str): ...


@dataclass(frozen=True)
class ExecutionResult:
    status: TaskStatus
    message: str
    completed_steps: int


class SkillExecutor:
    """Dispatch validated skills and commit task state only after success."""

    def __init__(self, skills: RobotSkillsProtocol, world_state: WorldState, *,
                 snapshot_source: Callable[[], PerceptionSnapshot] | None = None,
                 now_sec: Callable[[], float] | None = None,
                 on_step_success: Callable[[int, dict[str, str]], None] | None = None) -> None:
        self._skills = skills
        self._world_state = world_state
        self._snapshot_source = snapshot_source
        self._now_sec = now_sec
        self._on_step_success = on_step_success

    def execute(self, validation: ValidationResult) -> ExecutionResult:
        if not validation.accepted:
            return ExecutionResult(validation.status, validation.message, 0)
        if (validation.world_revision != self._world_state.revision
                or validation.world_signature != self._world_state.signature()):
            return ExecutionResult(TaskStatus.INVALID_PLAN, "stale plan rejected", 0)

        if self._world_state.perception_timestamp_sec is not None:
            if self._snapshot_source is None or self._now_sec is None:
                return ExecutionResult(TaskStatus.INVALID_PLAN, "camera guard unavailable", 0)
            try:
                snapshot = self._snapshot_source()
                if not self._world_state.matches_perception_snapshot(snapshot, self._now_sec()):
                    return ExecutionResult(TaskStatus.INVALID_PLAN, "camera layout changed since validation", 0)
            except (RuntimeError, ValueError, TypeError):
                return ExecutionResult(TaskStatus.INVALID_PLAN, "fresh camera guard failed", 0)
            temporary = [step["object"] for step in validation.steps
                         if step["skill"] == "place_temp"]
            if len(temporary) > 1:
                return ExecutionResult(TaskStatus.INVALID_PLAN, "one temporary reservation per task", 0)
            if temporary:
                prepare = getattr(self._skills, "reserve_temporary_position", None)
                try:
                    prepared = prepare is not None and self._status_value(
                        prepare(temporary[0], snapshot)) == TaskStatus.SUCCESS.value
                except (RuntimeError, ValueError, TypeError):
                    prepared = False
                if not prepared:
                    return ExecutionResult(TaskStatus.SKILL_FAILED, "temporary slot unavailable", 0)

        total_steps = len(validation.steps)
        LOGGER.info("VALIDATED PLAN LENGTH = %d", total_steps)
        try:
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
                if self._on_step_success is not None:
                    self._on_step_success(index, step)
            return ExecutionResult(TaskStatus.SUCCESS, "TASK SUCCESS", len(validation.steps))
        finally:
            clear = getattr(self._skills, "clear_temporary_position", None)
            if clear is not None:
                clear()

    def _runtime_precondition(self, step: dict[str, str]) -> TaskStatus | None:
        if step["skill"] == "pick":
            if self._world_state.held_object is not None:
                return TaskStatus.ALREADY_HOLDING_OBJECT
            if (self._world_state.perception_timestamp_sec is None
                    and self._world_state.object_locations[step["object"]] != TABLE):
                return TaskStatus.INVALID_PLAN
        elif step["skill"] in ("place", "place_temp"):
            if self._world_state.held_object != step["object"]:
                return TaskStatus.OBJECT_NOT_HELD
            if (step["skill"] == "place"
                    and self._world_state.zone_occupancy[step["zone"]] is not None):
                return TaskStatus.ZONE_OCCUPIED
        return None

    def _call_skill(self, step: dict[str, str]):
        if step["skill"] == "home":
            return self._skills.home()
        if step["skill"] == "pick":
            return self._skills.pick(step["object"])
        if step["skill"] == "place_temp":
            return self._skills.place_temp(step["object"])
        return self._skills.place(step["object"], step["zone"])

    def _record_success(self, step: dict[str, str]) -> None:
        if step["skill"] == "pick":
            self._world_state.record_pick_success(step["object"])
        elif step["skill"] == "place":
            self._world_state.record_place_success(step["object"], step["zone"])
        elif step["skill"] == "place_temp":
            self._world_state.record_temp_place_success(step["object"])

    @staticmethod
    def _status_value(result: object) -> str:
        return str(getattr(result, "value", result))
