"""Fail-closed M10 natural-language to validated M8 plan boundary."""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Callable, Mapping

import yaml

from ur3_llm_control.task_validator import TaskValidator, ValidationResult
from ur3_llm_control.world_state import WorldState


class PlannerStatus(str, Enum):
    SUCCESS = "SUCCESS"
    CONFIGURATION_ERROR = "CONFIGURATION_ERROR"
    INVALID_REQUEST = "INVALID_REQUEST"
    CLIENT_ERROR = "CLIENT_ERROR"
    API_ERROR = "API_ERROR"
    MALFORMED_RESPONSE = "MALFORMED_RESPONSE"
    VALIDATION_FAILED = "VALIDATION_FAILED"


@dataclass(frozen=True)
class LLMConfig:
    base_url: str
    api_key: str
    model: str
    timeout_sec: float
    temperature: float


@dataclass(frozen=True)
class PlannerResult:
    status: PlannerStatus
    message: str
    validation: ValidationResult | None = None
    candidate_json: str | None = None

    @property
    def accepted(self) -> bool:
        return self.status == PlannerStatus.SUCCESS and self.validation is not None and self.validation.accepted


class PlannerConfigurationError(ValueError):
    """Raised before a client or network request can be created."""


class LLMPlanner:
    """Use 9Router only to produce an M8-validated plan; never execute it."""

    def __init__(
        self,
        config: LLMConfig,
        system_prompt: str,
        client_factory: Callable[..., Any] | None = None,
    ) -> None:
        self._config = config
        self._system_prompt = system_prompt
        self._client_factory = client_factory or _openai_client_factory
        self._client: Any | None = None

    @classmethod
    def from_environment(
        cls,
        environ: Mapping[str, str] | None = None,
        config_path: str | Path | None = None,
        prompt_path: str | Path | None = None,
        client_factory: Callable[..., Any] | None = None,
    ) -> "LLMPlanner":
        return cls(
            config=resolve_llm_config(environ=environ, config_path=config_path),
            system_prompt=_read_text(prompt_path or _default_prompt_path(), "planner system prompt"),
            client_factory=client_factory,
        )

    def plan(self, request: str, world_state: WorldState) -> PlannerResult:
        """Return a new validated plan or a failure; no previous plan is retained."""
        if not isinstance(request, str) or not request.strip():
            return PlannerResult(PlannerStatus.INVALID_REQUEST, "natural-language request must be non-empty")

        try:
            client = self._client or self._client_factory(
                base_url=self._config.base_url,
                api_key=self._config.api_key,
            )
            self._client = client
        except Exception:
            return PlannerResult(PlannerStatus.CLIENT_ERROR, "9Router client is unavailable")

        try:
            response = client.chat.completions.create(
                model=self._config.model,
                messages=(
                    {"role": "system", "content": self._system_prompt},
                    {"role": "user", "content": request},
                ),
                temperature=self._config.temperature,
                timeout=self._config.timeout_sec,
            )
        except Exception:
            return PlannerResult(PlannerStatus.API_ERROR, "9Router planner request failed")

        candidate = _extract_json_candidate(response)
        if candidate is None:
            return PlannerResult(PlannerStatus.MALFORMED_RESPONSE, "9Router response did not contain one JSON object")

        validation = TaskValidator().validate(candidate, world_state)
        if not validation.accepted:
            return PlannerResult(
                PlannerStatus.VALIDATION_FAILED,
                f"LLM plan rejected: {validation.message}",
                validation=validation,
                candidate_json=candidate,
            )
        return PlannerResult(
            PlannerStatus.SUCCESS,
            "LLM plan accepted by M8 validator",
            validation=validation,
            candidate_json=candidate,
        )


def resolve_llm_config(
    environ: Mapping[str, str] | None = None,
    config_path: str | Path | None = None,
) -> LLMConfig:
    """Resolve non-secret defaults and require an environment key and model."""
    environment = os.environ if environ is None else environ
    document = _read_yaml(config_path or _default_config_path())
    base_url_env = _required_text(document, "base_url_env")
    api_key_env = _required_text(document, "api_key_env")
    model_env = _required_text(document, "model_env")
    base_url = environment.get(base_url_env) or _required_text(document, "default_base_url")
    api_key = environment.get(api_key_env)
    model = environment.get(model_env)
    if not api_key or not api_key.strip():
        raise PlannerConfigurationError(f"missing required environment variable {api_key_env}")
    if not model or not model.strip():
        raise PlannerConfigurationError(f"missing required environment variable {model_env}")
    timeout_sec = _positive_number(document, "timeout_sec")
    temperature = _nonnegative_number(document, "temperature")
    return LLMConfig(base_url=base_url, api_key=api_key, model=model, timeout_sec=timeout_sec, temperature=temperature)


def _extract_json_candidate(response: Any) -> str | None:
    """Accept one raw JSON object, optionally as a single JSON Markdown fence."""
    try:
        content = response.choices[0].message.content
    except (AttributeError, IndexError, TypeError):
        return None
    if not isinstance(content, str):
        return None
    candidate = content.strip()
    if candidate.startswith("```"):
        lines = candidate.splitlines()
        if len(lines) < 3 or lines[0].strip().lower() not in ("```", "```json") or lines[-1].strip() != "```":
            return None
        candidate = "\n".join(lines[1:-1]).strip()
    try:
        decoded = json.loads(candidate)
    except (TypeError, ValueError, json.JSONDecodeError):
        return None
    return candidate if isinstance(decoded, dict) else None


def _openai_client_factory(**kwargs: Any) -> Any:
    try:
        from openai import OpenAI
    except ImportError as error:
        raise RuntimeError("OpenAI SDK is not installed") from error
    return OpenAI(**kwargs)


def _default_config_path() -> Path:
    return _package_file("config", "llm.yaml")


def _default_prompt_path() -> Path:
    return _package_file("prompt", "planner_system_prompt.txt")


def _package_file(directory: str, filename: str) -> Path:
    source_file = Path(__file__).resolve().parent.parent / directory / filename
    if source_file.is_file():
        return source_file
    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError as error:
        raise PlannerConfigurationError(f"cannot find package {directory}/{filename}") from error
    return Path(get_package_share_directory("ur3_llm_control")) / directory / filename


def _read_yaml(path: str | Path) -> Mapping[str, Any]:
    try:
        document = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise PlannerConfigurationError(f"cannot read LLM configuration: {error}") from error
    if not isinstance(document, dict):
        raise PlannerConfigurationError("LLM configuration must be a YAML mapping")
    return document


def _read_text(path: str | Path, label: str) -> str:
    try:
        text = Path(path).read_text(encoding="utf-8")
    except OSError as error:
        raise PlannerConfigurationError(f"cannot read {label}: {error}") from error
    if not text.strip():
        raise PlannerConfigurationError(f"{label} must not be empty")
    return text


def _required_text(document: Mapping[str, Any], key: str) -> str:
    value = document.get(key)
    if not isinstance(value, str) or not value.strip():
        raise PlannerConfigurationError(f"LLM configuration requires non-empty {key}")
    return value


def _positive_number(document: Mapping[str, Any], key: str) -> float:
    value = document.get(key)
    if not isinstance(value, (int, float)) or isinstance(value, bool) or value <= 0:
        raise PlannerConfigurationError(f"LLM configuration requires positive {key}")
    return float(value)


def _nonnegative_number(document: Mapping[str, Any], key: str) -> float:
    value = document.get(key)
    if not isinstance(value, (int, float)) or isinstance(value, bool) or value < 0:
        raise PlannerConfigurationError(f"LLM configuration requires non-negative {key}")
    return float(value)
