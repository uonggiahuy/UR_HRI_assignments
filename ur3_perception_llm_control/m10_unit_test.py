"""Assignment 03 public plan semantics and fail-closed execution tests."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import yaml

from ur3_perception_llm_control.perception_scene_unit_test import A, snapshot
from ur3_perception_llm_control.perception_scene_unit_test import FakeManager
from ur3_perception_llm_control.perception_state import WorkcellGeometry
from ur3_perception_llm_control.perception_scene import PerceptionPlanningSceneSynchronizer
from ur3_perception_llm_control.motion_primitives import ManipulationMotionPrimitives
from ur3_perception_llm_control.moveit_interface import MotionResult
from ur3_perception_llm_control.robot_skills import RobotSkills, SkillStatus
from ur3_perception_llm_control.skill_executor import SkillExecutor
from ur3_perception_llm_control.task_validator import TaskStatus, TaskValidator
from ur3_perception_llm_control.world_state import BLOCKS, TABLE, WorldState
from ur3_perception_llm_control.workcell_scene import load_scene


ROOT = Path(__file__).resolve().parents[1]
PLAN = {"plan": [
    {"skill": "pick", "object": "blue_cube"},
    {"skill": "place_temp", "object": "blue_cube"},
    {"skill": "pick", "object": "red_cube"},
    {"skill": "place", "object": "red_cube", "zone": "zone_b"},
    {"skill": "home"},
]}


class M10UnitTest(unittest.TestCase):
    def setUp(self):
        self.geometry = WorkcellGeometry.from_scene_file(ROOT / "config/scene.yaml")
        self.positions = dict(A, blue_cube=(0.0, 0.24))
        self.observed = snapshot(self.positions, self.geometry)
        self.state = WorldState.from_perception_snapshot(self.observed, 10.1, self.geometry)
        self.validator = TaskValidator(assignment03=True)

    def test_camera_adapter_and_all_five_objects(self):
        self.assertEqual(self.state.zone_occupancy["zone_b"], "blue_cube")
        self.assertEqual(set(self.state.object_locations), set(BLOCKS))
        for name in BLOCKS:
            self.assertTrue(self.validator.validate({"plan": [{"skill": "pick", "object": name}]},
                                                    self.state).accepted)
        with self.assertRaises(ValueError):
            WorldState.from_perception_snapshot(self.observed, 12.0, self.geometry)
        with self.assertRaises(ValueError):
            WorldState.from_perception_snapshot(
                replace(self.observed, object_world_xy={"red_cube": (0, 0)}), 10.1,
                self.geometry)

    def test_schema_is_exact_and_rejects_low_level_fields(self):
        self.assertTrue(self.validator.validate(PLAN, self.state).accepted)
        for extra in ({"x": 0.1}, {"y": 0.1}, {"z": 0.1}, {"zone": "zone_a"},
                      {"trajectory": []}, {"candidate_id": 2}):
            with self.subTest(extra=extra):
                step = dict(PLAN["plan"][1], **extra)
                self.assertEqual(self.validator.validate(
                    {"plan": [PLAN["plan"][0], step]}, self.state).status,
                    TaskStatus.INVALID_PLAN)
        self.assertEqual(self.validator.validate(
            {"plan": [{"skill": "move_joint", "joint_position": 0}]}, self.state).status,
            TaskStatus.INVALID_SKILL)
        self.assertEqual(self.validator.validate(
            {"plan": [{"skill": "pick", "object": "unknown"}]}, self.state).status,
            TaskStatus.INVALID_OBJECT)

    def test_occupied_direct_place_rejected_but_blocker_plan_accepted(self):
        direct = {"plan": [
            {"skill": "pick", "object": "red_cube"},
            {"skill": "place", "object": "red_cube", "zone": "zone_b"},
            {"skill": "home"},
        ]}
        self.assertEqual(self.validator.validate(direct, self.state).status,
                         TaskStatus.ZONE_OCCUPIED)
        self.assertTrue(self.validator.validate(PLAN, self.state).accepted)
        simulated = self.state.copy()
        simulated.record_pick_success("blue_cube")
        self.assertIsNone(simulated.zone_occupancy["zone_b"])
        self.assertEqual(simulated.held_object, "blue_cube")
        simulated.record_temp_place_success("blue_cube")
        self.assertEqual(simulated.object_locations["blue_cube"], TABLE)
        self.assertIsNone(simulated.held_object)
        simulated.record_pick_success("red_cube")
        simulated.record_place_success("red_cube", "zone_b")
        self.assertEqual(simulated.zone_occupancy["zone_b"], "red_cube")
        self.assertEqual(self.state.zone_occupancy["zone_b"], "blue_cube")

    def test_wrong_object_and_holding_preconditions(self):
        for steps, wanted in (
            ([PLAN["plan"][0], {"skill": "pick", "object": "red_cube"}],
             TaskStatus.ALREADY_HOLDING_OBJECT),
            ([PLAN["plan"][0], {"skill": "place_temp", "object": "red_cube"}],
             TaskStatus.OBJECT_NOT_HELD),
            ([PLAN["plan"][0], {"skill": "place", "object": "red_cube", "zone": "zone_a"}],
             TaskStatus.OBJECT_NOT_HELD),
        ):
            self.assertEqual(self.validator.validate({"plan": steps}, self.state).status, wanted)

    def _skills(self, results=None):
        outcomes = iter(results or ["SUCCESS"] * len(PLAN["plan"]))
        skills = Mock()
        for method in ("pick", "place_temp", "place", "home"):
            getattr(skills, method).side_effect = lambda *args: next(outcomes)
        skills.reserve_temporary_position.return_value = "SUCCESS"
        return skills

    def _executor(self, skills, observed=None):
        return SkillExecutor(skills, self.state,
                             snapshot_source=lambda: observed or self.observed,
                             now_sec=lambda: 10.1)

    def test_executor_preflight_dispatch_and_success_only_state(self):
        skills = self._skills()
        validation = self.validator.validate(PLAN, self.state)
        result = self._executor(skills).execute(validation)
        self.assertEqual(result.status, TaskStatus.SUCCESS)
        skills.reserve_temporary_position.assert_called_once_with("blue_cube", self.observed)
        skills.place_temp.assert_called_once_with("blue_cube")
        skills.clear_temporary_position.assert_called_once()
        self.assertEqual(self.state.object_locations["blue_cube"], TABLE)
        self.assertEqual(self.state.zone_occupancy["zone_b"], "red_cube")
        self.assertEqual(self.state.revision, 4)

    def test_executor_stops_on_failure_and_clears_reservation(self):
        skills = self._skills(["SUCCESS", "FAILED"])
        result = self._executor(skills).execute(self.validator.validate(PLAN, self.state))
        self.assertEqual(result.status, TaskStatus.SKILL_FAILED)
        self.assertEqual(result.completed_steps, 1)
        self.assertEqual(self.state.held_object, "blue_cube")
        self.assertEqual(self.state.object_locations["blue_cube"], "held")
        skills.place.assert_not_called()
        skills.clear_temporary_position.assert_called_once()

    def test_stale_revision_and_new_occupied_snapshot_rejected_before_motion(self):
        validation = self.validator.validate(PLAN, self.state)
        self.state.revision += 1
        skills = self._skills()
        self.assertEqual(self._executor(skills).execute(validation).status, TaskStatus.INVALID_PLAN)
        skills.pick.assert_not_called()

        empty = snapshot(dict(self.positions, blue_cube=(0.12, 0.33)), self.geometry)
        empty_state = WorldState.from_perception_snapshot(empty, 10.1, self.geometry)
        empty_validation = self.validator.validate(
            {"plan": [{"skill": "pick", "object": "red_cube"},
                      {"skill": "place", "object": "red_cube", "zone": "zone_b"}]},
            empty_state)
        self.assertTrue(empty_validation.accepted)
        result = SkillExecutor(skills, empty_state, snapshot_source=lambda: self.observed,
                               now_sec=lambda: 10.1).execute(empty_validation)
        self.assertEqual(result.status, TaskStatus.INVALID_PLAN)
        skills.pick.assert_not_called()
        self.state.revision -= 1
        changed = snapshot(dict(self.positions, blue_cube=(0.12, 0.33)), self.geometry)
        self.assertEqual(self._executor(skills, changed).execute(validation).status, TaskStatus.INVALID_PLAN)
        skills.reserve_temporary_position.assert_not_called()
        skills.pick.assert_not_called()
        self.state.zone_occupancy["zone_b"] = None
        self.assertEqual(self._executor(skills).execute(validation).status, TaskStatus.INVALID_PLAN)
        skills.pick.assert_not_called()

    def test_camera_verification_failure_rejects_physical_place(self):
        scene = load_scene(str(ROOT / "config/scene.yaml"))
        motion = yaml.safe_load((ROOT / "config/robot_motion.yaml").read_text())
        manager = FakeManager(scene)
        red = next(item for item in manager.scene.world.collision_objects if item.id == "red_cube")
        manager.scene.world.collision_objects.remove(red)
        manager.scene.robot_state.attached_collision_objects.append(
            SimpleNamespace(object=SimpleNamespace(id="red_cube")))
        manager.clear_grasp_contact = Mock(return_value=True)
        manager.detach_object = Mock(side_effect=lambda name, pose: (
            manager.scene.robot_state.attached_collision_objects.clear(),
            manager.scene.world.collision_objects.append(red), True)[-1])
        interface = Mock(planning_frame="base_link")
        interface.move_to_pose.return_value = MotionResult.SUCCESS
        interface.move_to_joint_configuration.return_value = MotionResult.SUCCESS
        held = [True]
        physical = Mock()
        physical.is_attached.side_effect = lambda name: held[0]
        physical.release.side_effect = lambda name: held.__setitem__(0, False) or True
        synchronizer = PerceptionPlanningSceneSynchronizer(manager, scene, self.geometry)
        skills = RobotSkills(
            interface=interface, gripper=Mock(),
            primitives=ManipulationMotionPrimitives(interface, scene, motion),
            scene_manager=manager, physical_grasp=physical,
            snapshot_source=lambda: snapshot(A, self.geometry),
            scene_synchronizer=synchronizer, now_sec=lambda: 10.1,
            home_configuration=motion["home"], require_place_verification=True,
        )
        skills._last_observation_sec = 9.0
        self.assertEqual(skills.place("red_cube", "zone_a"), SkillStatus.FAILED)
        self.assertIsNone(skills.last_verified_snapshot)
        self.assertFalse(held[0])

    def test_legacy_domain_stays_three_cube_only(self):
        legacy = WorldState.from_scene_file(ROOT / "config/scene.yaml")
        self.assertEqual(TaskValidator().validate(PLAN, legacy).status, TaskStatus.INVALID_SKILL)
        self.assertEqual(TaskValidator().validate(
            {"plan": [{"skill": "pick", "object": "green_cube"}]}, legacy).status,
            TaskStatus.INVALID_OBJECT)


if __name__ == "__main__":
    unittest.main()
