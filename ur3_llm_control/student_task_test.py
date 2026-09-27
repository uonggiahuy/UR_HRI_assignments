"""Static M9 tests for deterministic student-ID personalization; no ROS or LLM."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import yaml

from ur3_llm_control.student_task import (
    StudentConfigurationError,
    StudentIdError,
    compute_variant,
    get_assignment_mapping,
    get_object_zone_mapping,
    parse_student_id,
    resolve_student_task,
)


EXPECTED_MAPPINGS = (
    {"zone_a": "red_cube", "zone_b": "yellow_cube", "zone_c": "blue_cube"},
    {"zone_a": "red_cube", "zone_b": "blue_cube", "zone_c": "yellow_cube"},
    {"zone_a": "yellow_cube", "zone_b": "red_cube", "zone_c": "blue_cube"},
    {"zone_a": "yellow_cube", "zone_b": "blue_cube", "zone_c": "red_cube"},
    {"zone_a": "blue_cube", "zone_b": "red_cube", "zone_c": "yellow_cube"},
    {"zone_a": "blue_cube", "zone_b": "yellow_cube", "zone_c": "red_cube"},
)


class StudentTaskTest(unittest.TestCase):
    def _config_path(self, contents: dict[str, object]) -> Path:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / "student_config.yaml"
        path.write_text(yaml.safe_dump(contents), encoding="utf-8")
        return path

    def test_all_six_variants_with_synthetic_ids(self) -> None:
        for variant, expected in enumerate(EXPECTED_MAPPINGS):
            # The leading zeros prove the ID is handled as a string, not an int.
            synthetic_id = f"0000{variant:02d}"
            with self.subTest(variant=variant, student_id=synthetic_id):
                self.assertEqual(compute_variant(synthetic_id), variant)
                self.assertEqual(get_assignment_mapping(synthetic_id), expected)

    def test_runtime_override_wins_over_config(self) -> None:
        config_path = self._config_path({"student_id": "000000", "student_name": "Config Name"})
        resolved = resolve_student_task("000005", config_path)
        self.assertEqual(resolved.student_id, "000005")
        self.assertEqual(resolved.variant, 5)
        self.assertEqual(resolved.mapping, EXPECTED_MAPPINGS[5])
        self.assertEqual(resolved.student_name, "Config Name")

    def test_config_fallback(self) -> None:
        config_path = self._config_path({"student_id": "000004", "student_name": "Metadata Only"})
        resolved = resolve_student_task(config_path=config_path)
        self.assertEqual(resolved.student_id, "000004")
        self.assertEqual(resolved.xx, 4)
        self.assertEqual(resolved.variant, 4)
        self.assertEqual(resolved.mapping, EXPECTED_MAPPINGS[4])
        self.assertEqual(resolved.student_name, "Metadata Only")

    def test_malformed_ids_are_rejected(self) -> None:
        for value in ("", "7", "12A4", "12 34", 1234, None):
            with self.subTest(value=value):
                with self.assertRaises(StudentIdError):
                    parse_student_id(value)  # type: ignore[arg-type]

    def test_missing_or_unusable_config_id_is_rejected(self) -> None:
        for config in ({}, {"student_id": ""}, {"student_id": "x1"}):
            with self.subTest(config=config):
                with self.assertRaises(StudentConfigurationError):
                    resolve_student_task(config_path=self._config_path(config))

    def test_repeated_result_is_deterministic_and_invertible(self) -> None:
        first = resolve_student_task("001234")
        second = resolve_student_task("001234")
        self.assertEqual(first, second)
        self.assertEqual(first.mapping, get_assignment_mapping("001234"))
        self.assertEqual(get_object_zone_mapping("001234"), {
            object_name: zone_name for zone_name, object_name in first.mapping.items()
        })


def main() -> None:
    result = unittest.main(module=__name__, exit=False).result
    if not result.wasSuccessful():
        raise SystemExit(1)


if __name__ == "__main__":
    main()
