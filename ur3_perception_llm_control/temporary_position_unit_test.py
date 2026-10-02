"""M9 geometry, fail-closed state, and deterministic ranking tests."""

from dataclasses import replace
from pathlib import Path
import unittest

import yaml

from ur3_perception_llm_control.perception_scene_unit_test import A, B, snapshot
from ur3_perception_llm_control.perception_state import WorkcellGeometry
from ur3_perception_llm_control.temporary_position import (
    TemporaryPositionError, TemporaryPositionPlanner,
)
from ur3_perception_llm_control.workcell_scene import load_scene
from ur3_perception_llm_control.world_state import BLOCKS


ROOT = Path(__file__).resolve().parents[1]


class TemporaryPositionUnitTest(unittest.TestCase):
    def setUp(self):
        self.scene = load_scene(str(ROOT / "config/scene.yaml"))
        self.geometry = WorkcellGeometry.from_scene_file(ROOT / "config/scene.yaml")
        self.settings = yaml.safe_load((ROOT / "config/temporary_position.yaml").read_text())
        self.planner = TemporaryPositionPlanner(self.scene, self.geometry, self.settings)
        self.observed = snapshot(A, self.geometry)

    def test_empty_regions_yield_deterministic_table_candidate(self):
        selected, stats = self.planner.find_temporary_position(
            "red_cube", self.observed, 10.1, lambda candidate: True)
        repeated, repeated_stats = self.planner.find_temporary_position(
            "red_cube", self.observed, 10.1, lambda candidate: True)
        self.assertEqual(selected, repeated)
        self.assertEqual(stats, repeated_stats)
        self.assertGreater(stats.generated, 0)
        self.assertGreater(stats.geometric_valid, 0)
        self.assertEqual(stats.moveit_feasible, 1)
        self.assertEqual(self.planner.candidate_rule("red_cube", (selected.x, selected.y),
                                                     self.observed)[0], None)
        self.assertAlmostEqual(selected.z, 0.3225)

    def test_zone_center_and_footprint_crossing_are_rejected(self):
        for xy in ((-0.12, 0.24), (-0.12, 0.30)):
            self.assertEqual(self.planner.candidate_rule("red_cube", xy, self.observed)[0],
                             "zone")

    def test_cube_overlap_and_clearance_are_rejected(self):
        self.assertEqual(self.planner.candidate_rule("red_cube", A["yellow_cube"],
                                                     self.observed)[0], "cube")
        self.assertEqual(self.planner.candidate_rule("red_cube", (0.059, 0.33),
                                                     self.observed)[0], "cube")

    def test_table_footprint_and_fixed_obstacle_are_rejected(self):
        self.assertEqual(self.planner.candidate_rule("red_cube", (-0.29, 0.40),
                                                     self.observed)[0], "table")
        altered = dict(self.scene)
        altered["pedestal"] = dict(self.scene["pedestal"])
        altered["pedestal"]["pose"] = dict(self.scene["pedestal"]["pose"], y=0.38)
        planner = TemporaryPositionPlanner(altered, self.geometry, self.settings)
        self.assertEqual(planner.candidate_rule("red_cube", (0.15, 0.40),
                                                self.observed)[0], "fixed")

    def test_own_old_footprint_is_excluded(self):
        self.assertIsNone(self.planner.candidate_rule("red_cube", A["red_cube"],
                                                       self.observed)[0])

    def test_stale_incomplete_ambiguous_and_conflicting_state_rejected_before_feasibility(self):
        invalid = (
            (self.observed, 12.0),
            (replace(self.observed, object_world_xy={"red_cube": A["red_cube"]}), 10.1),
            (replace(self.observed, object_world_xy=dict(self.observed.object_world_xy,
                                                         red_cube=(-0.12, 0.29))), 10.1),
            (replace(self.observed, zone_occupancy={"zone_a": "red_cube"}), 10.1),
        )
        for observed, now in invalid:
            with self.subTest(observed=observed, now=now):
                called = []
                with self.assertRaises(TemporaryPositionError):
                    self.planner.find_temporary_position(
                        "red_cube", observed, now, lambda candidate: called.append(candidate))
                self.assertFalse(called)

    def test_no_feasible_candidate_is_explicit(self):
        with self.assertRaises(TemporaryPositionError) as raised:
            self.planner.find_temporary_position("red_cube", self.observed, 10.1,
                                                  lambda candidate: False)
        self.assertEqual(raised.exception.code, "NO_TEMPORARY_POSITION")
        self.assertEqual(raised.exception.statistics.moveit_infeasible,
                         raised.exception.statistics.geometric_valid)

    def test_changed_layout_changes_choice(self):
        first, _ = self.planner.find_temporary_position("red_cube", self.observed,
                                                         10.1, lambda candidate: True)
        second, _ = self.planner.find_temporary_position("red_cube", snapshot(B, self.geometry),
                                                          10.1, lambda candidate: True)
        self.assertNotEqual(first.candidate_id, second.candidate_id)

    def test_occupied_zone_blocker_pattern(self):
        positions = dict(A, blue_cube=(0.0, 0.24), green_cube=(-0.12, 0.46),
                         purple_cube=(0.12, 0.46))
        observed = snapshot(positions, self.geometry)
        self.assertEqual(observed.zone_occupancy["zone_b"], "blue_cube")
        first, stats = self.planner.find_temporary_position(
            "blue_cube", observed, 10.1, lambda candidate: True)
        second, _ = self.planner.find_temporary_position(
            "blue_cube", observed, 10.1, lambda candidate: True)
        self.assertEqual(first, second)
        self.assertGreater(stats.geometric_valid, 0)
        self.assertIsNone(self.planner.candidate_rule("blue_cube", (first.x, first.y),
                                                      observed)[0])
        for name in BLOCKS:
            if name != "blue_cube":
                self.assertNotEqual((first.x, first.y), observed.object_world_xy[name])


if __name__ == "__main__":
    unittest.main()
