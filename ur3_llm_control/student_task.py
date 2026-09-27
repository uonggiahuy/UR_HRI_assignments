"""Deterministic, local student-ID task personalization for M9.

This module deliberately has no ROS, LLM, or robot-control dependencies.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

import yaml


ZONE_TO_OBJECT_BY_VARIANT: tuple[dict[str, str], ...] = (
    {"zone_a": "red_cube", "zone_b": "yellow_cube", "zone_c": "blue_cube"},
    {"zone_a": "red_cube", "zone_b": "blue_cube", "zone_c": "yellow_cube"},
    {"zone_a": "yellow_cube", "zone_b": "red_cube", "zone_c": "blue_cube"},
    {"zone_a": "yellow_cube", "zone_b": "blue_cube", "zone_c": "red_cube"},
    {"zone_a": "blue_cube", "zone_b": "red_cube", "zone_c": "yellow_cube"},
    {"zone_a": "blue_cube", "zone_b": "yellow_cube", "zone_c": "red_cube"},
)


class StudentIdError(ValueError):
    """Raised when a supplied student ID cannot safely determine a variant."""


class StudentConfigurationError(ValueError):
    """Raised when no usable student ID can be resolved from configuration."""


@dataclass(frozen=True)
class ParsedStudentId:
    """A validated ID, preserving the original string and its final two digits."""

    student_id: str
    xx: int


@dataclass(frozen=True)
class StudentTaskResolution:
    """Fully local result used by later task-planning code."""

    student_id: str
    xx: int
    variant: int
    mapping: dict[str, str]
    student_name: str | None = None

    def object_to_zone_mapping(self) -> dict[str, str]:
        """Return the inverse mapping for deterministic later planning."""
        return get_object_zone_mapping(self.student_id)


def parse_student_id(student_id: str) -> ParsedStudentId:
    """Validate a numeric string ID and obtain its final two digits (XX)."""
    if not isinstance(student_id, str):
        raise StudentIdError("student_id must be a string")
    if not student_id:
        raise StudentIdError("student_id must not be empty")
    if len(student_id) < 2:
        raise StudentIdError("student_id must contain at least two digits")
    if not student_id.isascii() or not student_id.isdecimal():
        raise StudentIdError("student_id must contain only ASCII digits")
    return ParsedStudentId(student_id=student_id, xx=int(student_id[-2:]))


def compute_variant(student_id: str) -> int:
    """Compute the assignment variant P from the validated student ID."""
    return parse_student_id(student_id).xx % len(ZONE_TO_OBJECT_BY_VARIANT)


def get_assignment_mapping(student_id: str) -> dict[str, str]:
    """Return a fresh zone-to-object mapping for the ID's deterministic P value."""
    return dict(ZONE_TO_OBJECT_BY_VARIANT[compute_variant(student_id)])


def get_object_zone_mapping(student_id: str) -> dict[str, str]:
    """Return a fresh object-to-zone inverse mapping for later planning."""
    return {object_name: zone_name for zone_name, object_name in get_assignment_mapping(student_id).items()}


def resolve_student_task(
    runtime_student_id: str | None = None,
    config_path: str | Path | None = None,
) -> StudentTaskResolution:
    """Resolve runtime ID first, otherwise use ``student_config.yaml`` fallback.

    A provided runtime ID, including a malformed one, is never silently replaced
    by configuration. ``student_name`` is retained solely as metadata.
    """
    if runtime_student_id is not None:
        parsed = parse_student_id(runtime_student_id)
        student_name = _read_student_name(config_path) if config_path is not None else None
    else:
        config = _read_config(config_path or _default_config_path())
        configured_id = config.get("student_id")
        try:
            parsed = parse_student_id(configured_id)
        except StudentIdError as error:
            raise StudentConfigurationError(f"student_id configuration error: {error}") from error
        configured_name = config.get("student_name")
        student_name = configured_name if isinstance(configured_name, str) else None

    variant = parsed.xx % len(ZONE_TO_OBJECT_BY_VARIANT)
    return StudentTaskResolution(
        student_id=parsed.student_id,
        xx=parsed.xx,
        variant=variant,
        mapping=dict(ZONE_TO_OBJECT_BY_VARIANT[variant]),
        student_name=student_name,
    )


def _read_student_name(config_path: str | Path) -> str | None:
    """Best-effort metadata read; a runtime ID must not require configuration."""
    try:
        config = _read_config(config_path)
    except StudentConfigurationError:
        return None
    student_name = config.get("student_name")
    return student_name if isinstance(student_name, str) else None


def _default_config_path() -> Path:
    """Find the package config both from source and after ament installation."""
    source_config = Path(__file__).resolve().parent.parent / "config" / "student_config.yaml"
    if source_config.is_file():
        return source_config
    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError as error:
        raise StudentConfigurationError("default student configuration is unavailable") from error
    return Path(get_package_share_directory("ur3_llm_control")) / "config" / "student_config.yaml"


def _read_config(config_path: str | Path) -> Mapping[str, object]:
    path = Path(config_path)
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except OSError as error:
        raise StudentConfigurationError(f"cannot read student configuration {path}: {error}") from error
    except yaml.YAMLError as error:
        raise StudentConfigurationError(f"invalid student configuration {path}: {error}") from error
    if not isinstance(document, dict):
        raise StudentConfigurationError("student configuration must be a YAML mapping")
    return document
