"""M5 symbolic containment, consistency, freshness, and legacy compatibility."""

from __future__ import annotations

from dataclasses import replace
from math import nan
from pathlib import Path
import unittest

from ur3_perception_llm_control.cube_detector import ObservedBlock
from ur3_perception_llm_control.perception_state import (
    DEFAULT_MAX_AGE_SEC, ZONE_CLEARANCE_M, PerceptionSnapshot,
    PerceptionStateError, PerceptionStateStatus, WorkcellGeometry, classify_location,
)
from ur3_perception_llm_control.world_state import BLOCKS, LEGACY_STUDENT_OBJECTS, WorldState


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "config/scene.yaml"
LAYOUT_B = ROOT / "config/scene_m4_layout_b.yaml"
A = {
    "red_cube": (-0.12, 0.33), "yellow_cube": (0.0, 0.33),
    "blue_cube": (0.12, 0.33), "green_cube": (-0.12, 0.46),
    "purple_cube": (0.12, 0.46),
}
B = {
    "red_cube": (-0.19, 0.50), "yellow_cube": (0.0, 0.48),
    "blue_cube": (0.19, 0.50), "green_cube": (0.12, 0.24),
    "purple_cube": (0.0, 0.24),
}


def detections(positions, stamp=10.0):
    return {
        name: ObservedBlock(name, (320.0, 240.0), xy, 500, 1.0, stamp, (310, 230, 23, 23))
        for name, xy in positions.items()
    }


class PerceptionStateTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.geometry = WorkcellGeometry.from_scene_file(SCENE)

    def test_all_five_table_and_all_three_empty_zones(self):
        snapshot = PerceptionSnapshot.from_detections(detections(A), self.geometry)
        self.assertEqual(set(snapshot.object_locations), set(BLOCKS))
        self.assertEqual(set(snapshot.object_world_xy), set(BLOCKS))
        self.assertEqual(set(snapshot.zone_occupancy), {"zone_a", "zone_b", "zone_c"})
        self.assertTrue(all(value == "table" for value in snapshot.object_locations.values()))
        self.assertTrue(all(value is None for value in snapshot.zone_occupancy.values()))
        self.assertIsNone(snapshot.held_object)
        with self.assertRaises(TypeError):
            snapshot.zone_occupancy["zone_a"] = "red_cube"

    def test_layout_b_occupied_zones_and_spawn_pose_independence(self):
        other_geometry = WorkcellGeometry.from_scene_file(LAYOUT_B)
        self.assertEqual(dict(self.geometry.zones), dict(other_geometry.zones))
        self.assertEqual(dict(self.geometry.cube_sizes), dict(other_geometry.cube_sizes))
        snapshot = PerceptionSnapshot.from_detections(detections(B), self.geometry)
        self.assertEqual(dict(snapshot.object_locations), {
            "red_cube": "table", "yellow_cube": "table", "blue_cube": "table",
            "green_cube": "zone_c", "purple_cube": "zone_b",
        })
        self.assertEqual(dict(snapshot.zone_occupancy), {
            "zone_a": None, "zone_b": "purple_cube", "zone_c": "green_cube",
        })
        self.assertEqual(dict(PerceptionSnapshot.from_detections(detections(B), other_geometry).object_locations),
                         dict(snapshot.object_locations))

    def test_footprint_containment_and_boundary_ambiguity(self):
        self.assertEqual(self.geometry.cube_sizes["red_cube"], (0.045, 0.045))
        self.assertEqual(self.geometry.zones["zone_a"].size_xy, (0.08, 0.08))
        self.assertEqual(self.geometry.clearance_m, ZONE_CLEARANCE_M)
        self.assertEqual(classify_location("red_cube", (-0.12, 0.24), self.geometry), "zone_a")
        self.assertEqual(classify_location("red_cube", (-0.12, 0.33), self.geometry), "table")
        # 30 mm offset leaves a 7.5 mm cube overhang beyond the zone edge.
        with self.assertRaises(PerceptionStateError) as caught:
            classify_location("red_cube", (-0.12, 0.27), self.geometry)
        self.assertEqual(caught.exception.status, PerceptionStateStatus.AMBIGUOUS)
        # Even near the exact-fit limit, the 3 mm observation margin fails closed.
        with self.assertRaises(PerceptionStateError):
            classify_location("red_cube", (-0.12, 0.255), self.geometry)

    def test_missing_nonfinite_mixed_frame_and_conflict_rejected(self):
        sample = detections(A)
        del sample["blue_cube"]
        with self.assertRaises(PerceptionStateError) as caught:
            PerceptionSnapshot.from_detections(sample, self.geometry)
        self.assertEqual(caught.exception.status, PerceptionStateStatus.INCOMPLETE)
        sample = detections(A)
        sample["blue_cube"] = replace(sample["blue_cube"], world_xy=(nan, 0.3))
        with self.assertRaises(PerceptionStateError) as caught:
            PerceptionSnapshot.from_detections(sample, self.geometry)
        self.assertEqual(caught.exception.status, PerceptionStateStatus.INVALID)
        sample = detections(A)
        sample["blue_cube"] = replace(sample["blue_cube"], timestamp_sec=9.9)
        with self.assertRaises(PerceptionStateError) as caught:
            PerceptionSnapshot.from_detections(sample, self.geometry)
        self.assertEqual(caught.exception.status, PerceptionStateStatus.INVALID)
        positions = dict(B, red_cube=(0.0, 0.24))
        with self.assertRaises(PerceptionStateError) as caught:
            PerceptionSnapshot.from_detections(detections(positions), self.geometry)
        self.assertEqual(caught.exception.status, PerceptionStateStatus.OCCUPANCY_CONFLICT)

    def test_freshness_and_stale_rejection(self):
        snapshot = PerceptionSnapshot.from_detections(detections(A), self.geometry)
        self.assertEqual(snapshot.observation_timestamp_sec, 10.0)
        self.assertEqual(DEFAULT_MAX_AGE_SEC, 1.0)
        self.assertTrue(snapshot.is_fresh(10.1))
        self.assertTrue(snapshot.is_fresh(11.0))
        self.assertFalse(snapshot.is_fresh(11.001))
        self.assertFalse(snapshot.is_fresh(9.9))
        with self.assertRaises(PerceptionStateError) as caught:
            snapshot.require_fresh(11.001)
        self.assertEqual(caught.exception.status, PerceptionStateStatus.STALE)

    def test_assignment_02_world_state_and_mssv_domain_unchanged(self):
        legacy = WorldState.from_scene_file(SCENE)
        self.assertEqual(set(legacy.object_locations), set(BLOCKS))
        self.assertTrue(all(value == "table" for value in legacy.object_locations.values()))
        self.assertEqual(LEGACY_STUDENT_OBJECTS, {"red_cube", "yellow_cube", "blue_cube"})
        legacy.record_pick_success("red_cube")
        legacy.record_place_success("red_cube", "zone_b")
        self.assertEqual(legacy.zone_occupancy["zone_b"], "red_cube")
        self.assertEqual(legacy.revision, 2)


if __name__ == "__main__":
    unittest.main()
