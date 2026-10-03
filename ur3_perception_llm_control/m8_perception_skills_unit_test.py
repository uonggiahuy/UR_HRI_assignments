"""M8 camera target, freshness, and visual-zone support regression tests."""

from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from ur3_perception_llm_control.motion_primitives import ManipulationMotionPrimitives
from ur3_perception_llm_control.moveit_interface import MotionResult
from ur3_perception_llm_control.perception_scene import PerceptionPlanningSceneSynchronizer
from ur3_perception_llm_control.perception_scene_unit_test import A, B, FakeManager, snapshot
from ur3_perception_llm_control.perception_state import WorkcellGeometry
from ur3_perception_llm_control.robot_skills import RobotSkills, SkillStatus
from ur3_perception_llm_control.workcell_scene import load_scene
import yaml


ROOT = Path(__file__).resolve().parents[1]


class M8PerceptionSkillsTest(unittest.TestCase):
    def setUp(self):
        self.scene = load_scene(str(ROOT / "config/scene.yaml"))
        self.motion = yaml.safe_load((ROOT / "config/robot_motion.yaml").read_text())
        self.geometry = WorkcellGeometry.from_scene_file(ROOT / "config/scene.yaml")
        self.manager = FakeManager(self.scene)
        self.manager.allow_grasp_contact = Mock(return_value=False)
        self.interface = Mock(planning_frame="base_link")
        self.interface.move_to_joint_configuration.return_value = MotionResult.SUCCESS
        self.interface.move_to_pose.return_value = MotionResult.SUCCESS
        self.interface.move_straight_to_pose.return_value = MotionResult.SUCCESS
        self.primitives = ManipulationMotionPrimitives(self.interface, self.scene, self.motion)
        self.gripper = Mock()
        self.physical = Mock()
        self.sync = PerceptionPlanningSceneSynchronizer(self.manager, self.scene, self.geometry)

    def skills_for(self, observed, now=10.1):
        return RobotSkills(
            interface=self.interface, gripper=self.gripper, primitives=self.primitives,
            scene_manager=self.manager, physical_grasp=self.physical,
            snapshot_source=lambda: observed, scene_synchronizer=self.sync,
            now_sec=lambda: now, home_configuration=self.motion["home"],
        )

    def test_changed_camera_xy_changes_pick_target_and_scene_together(self):
        targets = []
        for positions in (A, B):
            observed = snapshot(positions, self.geometry)
            skills = self.skills_for(observed)
            self.assertEqual(skills.pick("red_cube"), SkillStatus.FAILED)
            self.assertEqual(len(self.manager.applied), len(targets) + 1)
            target = self.interface.move_to_pose.call_args.args[0]
            world_x = target.pose.position.x + self.scene["robot"]["mount_pose"]["x"]
            world_y = target.pose.position.y + self.scene["robot"]["mount_pose"]["y"]
            self.assertAlmostEqual(world_x, positions["red_cube"][0])
            self.assertAlmostEqual(world_y, positions["red_cube"][1])
            self.assertEqual(self.manager.applied[-1].world.collision_objects[0].pose.position.x,
                             positions["red_cube"][0])
            targets.append((world_x, world_y))
        self.assertNotEqual(targets[0], targets[1])
        self.assertNotEqual(targets[1], (self.scene["objects"]["red_cube"]["pose"]["x"],
                                          self.scene["objects"]["red_cube"]["pose"]["y"]))

    def test_stale_incomplete_and_ambiguous_snapshots_cause_zero_motion(self):
        valid = snapshot(A, self.geometry)
        invalid = (
            (valid, 12.0),
            (replace(valid, object_world_xy={"red_cube": A["red_cube"]}), 10.1),
            (replace(valid, object_locations=dict(valid.object_locations, red_cube="zone_a")), 10.1),
            (replace(valid, object_world_xy=dict(valid.object_world_xy,
                                                 red_cube=(-0.12, 0.29))), 10.1),
        )
        for observed, now in invalid:
            with self.subTest(now=now, observed=observed):
                self.assertEqual(self.skills_for(observed, now).pick("red_cube"), SkillStatus.FAILED)
                self.interface.move_to_joint_configuration.assert_not_called()
                self.interface.move_to_pose.assert_not_called()
                self.gripper.open.assert_not_called()
                self.assertFalse(self.manager.applied)

    def test_zone_placement_uses_fixed_xy_and_tabletop_height(self):
        table = self.scene["table"]
        height = table["pose"]["z"] + table["size"]["z"] / 2
        cube_half = self.scene["objects"]["red_cube"]["size"]["z"] / 2
        for zone_name in ("zone_a", "zone_b", "zone_c"):
            placement = self.primitives.placement_world_pose("red_cube", zone_name)
            zone = self.scene["zones"][zone_name]
            self.assertAlmostEqual(placement.pose.position.x, zone["pose"]["x"])
            self.assertAlmostEqual(placement.pose.position.y, zone["pose"]["y"])
            self.assertAlmostEqual(placement.pose.position.z, height + cube_half)
            self.primitives.descend_to_placement("red_cube", zone_name)
            target = self.interface.move_straight_to_pose.call_args.args[0]
            self.assertAlmostEqual(target.pose.position.z + self.scene["robot"]["mount_pose"]["z"],
                                   height + cube_half + self.motion["manipulation"]["grasp_z_offset"])

    def test_zone_repick_descent_keeps_the_approach_xy_and_uses_cartesian_motion(self):
        """A settled zone cube must descend vertically beside table neighbors."""
        observed = snapshot(dict(A, red_cube=(0.002, 0.252)), self.geometry)
        center = self.primitives.target_world_pose("red_cube")
        center.pose.position.x = observed.object_world_xy["red_cube"][0]
        center.pose.position.y = observed.object_world_xy["red_cube"][1]
        approach, descent = self.primitives.placement_planning_poses(center)
        self.primitives.descend_to_world_pose(center)
        target = self.interface.move_straight_to_pose.call_args.args[0]
        self.assertAlmostEqual(target.pose.position.x, descent.pose.position.x)
        self.assertAlmostEqual(target.pose.position.y, descent.pose.position.y)
        self.assertLess(target.pose.position.z, approach.pose.position.z)
        self.interface.move_to_pose.assert_not_called()

    def test_authoritative_scene_mismatch_blocks_motion(self):
        self.manager.inject_duplicate = True
        self.assertEqual(self.skills_for(snapshot(A, self.geometry)).pick("red_cube"),
                         SkillStatus.FAILED)
        self.interface.move_to_joint_configuration.assert_not_called()
        self.interface.move_to_pose.assert_not_called()

    def test_physical_release_refreshes_world_from_later_camera_frame(self):
        red = next(item for item in self.manager.scene.world.collision_objects
                   if item.id == "red_cube")
        self.manager.scene.world.collision_objects.remove(red)
        self.manager.scene.robot_state.attached_collision_objects.append(
            SimpleNamespace(object=SimpleNamespace(id="red_cube")))
        self.manager.clear_grasp_contact = Mock(return_value=True)
        held = [True]
        events = []
        self.interface.move_to_pose.side_effect = lambda pose: events.append("motion") or MotionResult.SUCCESS
        self.interface.move_straight_to_pose.side_effect = (
            lambda pose: events.append("motion") or MotionResult.SUCCESS
        )
        self.gripper.open.side_effect = lambda: events.append("open")
        self.physical.is_attached.side_effect = lambda name: held[0]

        def detach(name, pose):
            self.manager.scene.robot_state.attached_collision_objects.clear()
            red.pose = pose.pose
            self.manager.scene.world.collision_objects.append(red)
            return True

        self.manager.detach_object = Mock(side_effect=detach)
        def release(name):
            events.append("release")
            held[0] = False
            return True

        self.physical.release.side_effect = release
        observed = snapshot(B, self.geometry)
        skills = self.skills_for(observed)
        skills._last_observation_sec = 9.0
        self.assertEqual(skills.place("red_cube", "zone_a"), SkillStatus.SUCCESS)
        red_now = next(item for item in self.manager.scene.world.collision_objects
                       if item.id == "red_cube")
        self.assertAlmostEqual(red_now.pose.position.x, B["red_cube"][0])
        self.assertAlmostEqual(red_now.pose.position.y, B["red_cube"][1])
        self.assertFalse(held[0])
        self.assertEqual(len(self.manager.applied), 1)
        self.interface.move_to_joint_configuration.assert_called_once()
        self.assertEqual(events[-3:], ["release", "motion", "open"])


if __name__ == "__main__":
    unittest.main()
