"""M6 pure/fake-client tests for coherent, verified MoveIt scene updates."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import replace
from pathlib import Path
import unittest

from moveit_msgs.msg import AttachedCollisionObject

from ur3_perception_llm_control.cube_detector import ObservedBlock
from ur3_perception_llm_control.perception_scene import (
    PerceptionPlanningSceneSynchronizer, SceneSyncError,
)
from ur3_perception_llm_control.perception_state import PerceptionSnapshot, WorkcellGeometry
from ur3_perception_llm_control.planning_scene import planning_scene_diff, scene_matches
from ur3_perception_llm_control.workcell_scene import load_scene
from ur3_perception_llm_control.world_state import BLOCKS, ZONES


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "config/scene.yaml"
A = {"red_cube": (-0.12, 0.33), "yellow_cube": (0.0, 0.33),
     "blue_cube": (0.12, 0.33), "green_cube": (-0.12, 0.46), "purple_cube": (0.12, 0.46)}
B = {"red_cube": (-0.19, 0.50), "yellow_cube": (0.0, 0.48),
     "blue_cube": (0.19, 0.50), "green_cube": (0.12, 0.24), "purple_cube": (0.0, 0.24)}


def snapshot(positions, geometry, stamp=10.0):
    observations = {
        name: ObservedBlock(name, (320.0, 240.0), xy, 500, 1.0, stamp, (310, 230, 23, 23))
        for name, xy in positions.items()
    }
    return PerceptionSnapshot.from_detections(observations, geometry)


class FakeManager:
    def __init__(self, scene):
        self.scene = planning_scene_diff(scene)
        self.applied = []
        self.get_calls = 0
        self.response_success = True
        self.inject_duplicate = False
        self.inject_pose_error = False

    def get(self):
        self.get_calls += 1
        return deepcopy(self.scene)

    def apply_scene_diff(self, diff):
        self.applied.append(deepcopy(diff))
        for object_ in diff.world.collision_objects:
            self.scene.world.collision_objects = [item for item in self.scene.world.collision_objects
                                                  if item.id != object_.id]
            self.scene.world.collision_objects.append(deepcopy(object_))
        if self.inject_duplicate:
            self.scene.world.collision_objects.append(deepcopy(diff.world.collision_objects[0]))
        if self.inject_pose_error:
            next(item for item in self.scene.world.collision_objects
                 if item.id == "red_cube").pose.position.x += 0.002
        return self.response_success


class PerceptionSceneTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scene = load_scene(str(SCENE))
        cls.geometry = WorkcellGeometry.from_scene_file(SCENE)

    def setup_sync(self):
        manager = FakeManager(self.scene)
        return manager, PerceptionPlanningSceneSynchronizer(manager, self.scene, self.geometry)

    def test_five_world_boxes_use_camera_xy_and_table_support_z(self):
        manager, sync = self.setup_sync()
        observed = snapshot(A, self.geometry)
        report = sync.apply_snapshot(observed, 10.1)
        self.assertEqual(len(manager.applied), 1)
        self.assertEqual(len(manager.applied[0].world.collision_objects), 5)
        self.assertEqual(set(report.authoritative_xyz), set(BLOCKS))
        for name in BLOCKS:
            self.assertEqual(report.requested_xyz[name][:2], A[name])
            self.assertAlmostEqual(report.requested_xyz[name][2], 0.3225)
            self.assertEqual(report.requested_xyz[name], report.authoritative_xyz[name])
            self.assertEqual(report.sync_errors_m[name], 0.0)
        self.assertEqual(report.max_sync_error_m, 0.0)
        self.assertFalse(report.attached_ids)

    def test_zone_support_z_dimensions_and_spawn_xy_independence(self):
        manager, sync = self.setup_sync()
        report = sync.apply_snapshot(snapshot(B, self.geometry), 10.1)
        self.assertEqual(report.requested_xyz["green_cube"], (0.12, 0.24, 0.3245))
        self.assertEqual(report.requested_xyz["purple_cube"], (0.0, 0.24, 0.3245))
        for name in ("red_cube", "yellow_cube", "blue_cube"):
            self.assertAlmostEqual(report.requested_xyz[name][2], 0.3225)
        self.assertNotEqual(report.requested_xyz["green_cube"][:2],
                            (self.scene["objects"]["green_cube"]["pose"]["x"],
                             self.scene["objects"]["green_cube"]["pose"]["y"]))
        for item in manager.scene.world.collision_objects:
            if item.id in BLOCKS:
                self.assertEqual(list(item.primitives[0].dimensions), [0.045, 0.045, 0.045])
                self.assertEqual((item.pose.orientation.x, item.pose.orientation.y,
                                  item.pose.orientation.z, item.pose.orientation.w), (0.0, 0.0, 0.0, 1.0))

    def test_stale_and_invalid_snapshot_cause_zero_mutation(self):
        manager, sync = self.setup_sync()
        valid = snapshot(A, self.geometry)
        before = deepcopy(manager.scene)
        with self.assertRaises(SceneSyncError):
            sync.apply_snapshot(valid, 12.0)
        with self.assertRaises(SceneSyncError):
            sync.apply_snapshot(replace(valid, object_world_xy={"red_cube": A["red_cube"]}), 10.1)
        with self.assertRaises(SceneSyncError):
            sync.apply_snapshot(replace(valid, object_locations=dict(valid.object_locations, red_cube="zone_a")), 10.1)
        self.assertEqual(manager.applied, [])
        self.assertEqual(manager.get_calls, 0)
        self.assertEqual(manager.scene, before)

    def test_attached_cube_excluded_from_world_diff(self):
        manager, sync = self.setup_sync()
        attached = AttachedCollisionObject()
        attached.link_name = "gripper_tcp"
        attached.object.id = "red_cube"
        manager.scene.robot_state.attached_collision_objects.append(attached)
        manager.scene.world.collision_objects = [item for item in manager.scene.world.collision_objects
                                                 if item.id != "red_cube"]
        report = sync.apply_snapshot(snapshot(A, self.geometry), 10.1)
        self.assertEqual(report.attached_ids, {"red_cube"})
        self.assertEqual({item.id for item in manager.applied[0].world.collision_objects}, set(BLOCKS)-{"red_cube"})
        self.assertNotIn("red_cube", {item.id for item in manager.scene.world.collision_objects})

    def test_preexisting_world_plus_attached_rejected_without_mutation(self):
        manager, sync = self.setup_sync()
        attached = AttachedCollisionObject()
        attached.object.id = "red_cube"
        manager.scene.robot_state.attached_collision_objects.append(attached)
        with self.assertRaises(SceneSyncError):
            sync.apply_snapshot(snapshot(A, self.geometry), 10.1)
        self.assertEqual(manager.applied, [])

    def test_duplicate_world_object_rejected_by_authoritative_verification(self):
        manager, sync = self.setup_sync()
        manager.inject_duplicate = True
        with self.assertRaisesRegex(SceneSyncError, "exactly one WORLD"):
            sync.apply_snapshot(snapshot(A, self.geometry), 10.1)
        self.assertEqual(len(manager.applied), 1)

    def test_authoritative_pose_over_one_mm_rejected(self):
        manager, sync = self.setup_sync()
        manager.inject_pose_error = True
        with self.assertRaisesRegex(SceneSyncError, "pose error"):
            sync.apply_snapshot(snapshot(A, self.geometry), 10.1)

    def test_unexpected_attachment_rejected_before_mutation(self):
        manager, sync = self.setup_sync()
        attached = AttachedCollisionObject()
        attached.object.id = "other_object"
        manager.scene.robot_state.attached_collision_objects.append(attached)
        with self.assertRaisesRegex(SceneSyncError, "non-cube attachment"):
            sync.apply_snapshot(snapshot(A, self.geometry), 10.1)
        self.assertEqual(manager.applied, [])

    def test_service_boolean_is_not_the_verification_oracle(self):
        manager, sync = self.setup_sync()
        manager.response_success = False
        report = sync.apply_snapshot(snapshot(A, self.geometry), 10.1)
        self.assertFalse(report.service_accepted)
        self.assertEqual(report.max_sync_error_m, 0.0)

    def test_static_geometry_zones_and_legacy_scene_api_preserved(self):
        manager, sync = self.setup_sync()
        initial = deepcopy(manager.scene)
        sync.apply_snapshot(snapshot(B, self.geometry), 10.1)
        for name in ("robot_pedestal", "manipulation_table"):
            old = next(item for item in initial.world.collision_objects if item.id == name)
            new = next(item for item in manager.scene.world.collision_objects if item.id == name)
            self.assertEqual(old, new)
        self.assertFalse({item.id for item in manager.scene.world.collision_objects} & set(ZONES))
        legacy = planning_scene_diff(self.scene)
        self.assertTrue(scene_matches(legacy, planning_scene_diff(self.scene))[0])
        self.assertAlmostEqual(next(item for item in legacy.world.collision_objects
                                    if item.id == "green_cube").pose.position.y, 0.46)


if __name__ == "__main__":
    unittest.main()
