"""Strict JSON plan schema and semantic validation for M8."""

from __future__ import annotations

import json
from dataclasses import dataclass
from enum import Enum
from typing import Any

from ur3_perception_llm_control.world_state import BLOCKS, LEGACY_STUDENT_OBJECTS, TABLE, ZONES, WorldState


class TaskStatus(str, Enum):
    SUCCESS = "SUCCESS"
    FAILED = "FAILED"
    INVALID_PLAN = "INVALID_PLAN"
    INVALID_SKILL = "INVALID_SKILL"
    INVALID_OBJECT = "INVALID_OBJECT"
    INVALID_ZONE = "INVALID_ZONE"
    OBJECT_NOT_HELD = "OBJECT_NOT_HELD"
    ALREADY_HOLDING_OBJECT = "ALREADY_HOLDING_OBJECT"
    ZONE_OCCUPIED = "ZONE_OCCUPIED"
    SKILL_FAILED = "SKILL_FAILED"


@dataclass(frozen=True)
class ValidationResult:
    status: TaskStatus
    message: str
    steps: tuple[dict[str, str], ...] = ()
    world_revision: int | None = None
    world_signature: tuple | None = None

    @property
    def accepted(self) -> bool:
        return self.status == TaskStatus.SUCCESS


class TaskValidator:
    """Accept only whitelisted high-level plans and reject unsafe semantics."""

    _SCHEMAS = {
        "home": frozenset(("skill",)),
        "pick": frozenset(("skill", "object")),
        "place": frozenset(("skill", "object", "zone")),
    }

    def __init__(self, *, assignment03: bool = False) -> None:
        self.assignment03 = assignment03

    def validate(self, plan_input: str | bytes | dict[str, Any], world_state: WorldState) -> ValidationResult:
        if self.assignment03 and world_state.perception_timestamp_sec is None:
            return self._reject(TaskStatus.INVALID_PLAN, "Assignment 03 requires camera-derived task state")
        document = self._decode(plan_input)
        if document is None:
            return self._reject(TaskStatus.INVALID_PLAN, "plan must be valid JSON object")
        if set(document) != {"plan"}:
            return self._reject(TaskStatus.INVALID_PLAN, "top-level object must contain exactly plan")
        raw_steps = document.get("plan")
        if not isinstance(raw_steps, list):
            return self._reject(TaskStatus.INVALID_PLAN, "plan must be a list")
        if not raw_steps:
            return self._reject(TaskStatus.INVALID_PLAN, "plan must not be empty")

        simulated = world_state.copy()
        steps: list[dict[str, str]] = []
        for index, raw_step in enumerate(raw_steps, start=1):
            checked = self._validate_step(raw_step, index)
            if isinstance(checked, ValidationResult):
                return checked
            semantic_error = self._validate_semantics(checked, simulated, index)
            if semantic_error is not None:
                return semantic_error
            self._simulate(checked, simulated)
            steps.append(checked)
        return ValidationResult(TaskStatus.SUCCESS, "PLAN ACCEPTED", tuple(steps),
                                world_state.revision, world_state.signature())

    @staticmethod
    def _decode(plan_input: str | bytes | dict[str, Any]) -> dict[str, Any] | None:
        if isinstance(plan_input, dict):
            return plan_input
        if not isinstance(plan_input, (str, bytes)):
            return None
        try:
            decoded = json.loads(plan_input)
        except (TypeError, ValueError, json.JSONDecodeError):
            return None
        return decoded if isinstance(decoded, dict) else None

    def _validate_step(self, raw_step: Any, index: int) -> dict[str, str] | ValidationResult:
        if not isinstance(raw_step, dict) or not raw_step:
            return self._reject(TaskStatus.INVALID_PLAN, f"step {index} must be a non-empty object")
        skill = raw_step.get("skill")
        schemas = self._SCHEMAS if not self.assignment03 else {
            **self._SCHEMAS, "place_temp": frozenset(("skill", "object")),
        }
        if not isinstance(skill, str) or skill not in schemas:
            return self._reject(TaskStatus.INVALID_SKILL, f"step {index} has unsupported skill")
        if set(raw_step) != schemas[skill]:
            return self._reject(TaskStatus.INVALID_PLAN, f"step {index} has incorrect arguments for {skill}")
        if skill in ("pick", "place", "place_temp"):
            object_name = raw_step["object"]
            allowed = BLOCKS if self.assignment03 else LEGACY_STUDENT_OBJECTS
            if not isinstance(object_name, str) or object_name not in allowed:
                return self._reject(TaskStatus.INVALID_OBJECT, f"step {index} has invalid object")
        if skill == "place":
            zone_name = raw_step["zone"]
            if not isinstance(zone_name, str) or zone_name not in ZONES:
                return self._reject(TaskStatus.INVALID_ZONE, f"step {index} has invalid zone")
        return {key: value for key, value in raw_step.items()}

    def _validate_semantics(
        self, step: dict[str, str], state: WorldState, index: int
    ) -> ValidationResult | None:
        skill = step["skill"]
        if skill == "pick":
            if state.held_object is not None:
                return self._reject(TaskStatus.ALREADY_HOLDING_OBJECT, f"step {index}: already holding {state.held_object}")
            if (not self.assignment03 and state.object_locations[step["object"]] != TABLE):
                return self._reject(TaskStatus.INVALID_PLAN, f"step {index}: object is not available on table")
        elif skill in ("place", "place_temp"):
            if state.held_object != step["object"]:
                return self._reject(TaskStatus.OBJECT_NOT_HELD, f"step {index}: object is not held")
            if skill == "place" and state.zone_occupancy[step["zone"]] is not None:
                return self._reject(TaskStatus.ZONE_OCCUPIED, f"step {index}: zone is occupied")
        return None

    @staticmethod
    def _simulate(step: dict[str, str], state: WorldState) -> None:
        if step["skill"] == "pick":
            state.record_pick_success(step["object"])
        elif step["skill"] == "place":
            state.record_place_success(step["object"], step["zone"])
        elif step["skill"] == "place_temp":
            state.record_temp_place_success(step["object"])

    @staticmethod
    def _reject(status: TaskStatus, message: str) -> ValidationResult:
        return ValidationResult(status, message)
