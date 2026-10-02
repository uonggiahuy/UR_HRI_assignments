"""M7 fail-closed grasp and dual-lifecycle unit tests."""

import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from geometry_msgs.msg import Pose, PoseStamped

from ur3_perception_llm_control.gripper import CLOSED_POSITION, FINGER_JOINTS
from ur3_perception_llm_control.physical_grasp import CUBES, PhysicalGraspManager
from ur3_perception_llm_control.robot_skills import RobotSkills, SkillStatus


class PhysicalGraspUnitTest(unittest.TestCase):
    def setUp(self):
        self.grasp = PhysicalGraspManager.__new__(PhysicalGraspManager)
        self.grasp._ready = True
        self.grasp._state = {name: "detached" for name in CUBES}
        self.grasp._closed = Mock(return_value=True)
        self.grasp._near = Mock(return_value=True)
        self.grasp._transition = Mock(side_effect=self._transition)

    def _transition(self, name, state, timeout):
        self.grasp._state[name] = state
        return True

    def test_invalid_object_rejected(self):
        self.assertFalse(self.grasp.attach("table"))
        self.grasp._transition.assert_not_called()

    def test_closed_and_near_required(self):
        self.grasp._closed.return_value = False
        self.assertFalse(self.grasp.attach("red_cube"))
        self.grasp._closed.return_value = True
        self.grasp._near.return_value = False
        self.assertFalse(self.grasp.attach("red_cube"))
        self.grasp._transition.assert_not_called()

    def test_attach_and_release_verified_state(self):
        self.assertTrue(self.grasp.attach("red_cube"))
        self.assertTrue(self.grasp.is_attached("red_cube"))
        self.assertTrue(self.grasp.release("red_cube"))
        self.assertFalse(self.grasp.is_attached("red_cube"))

    def test_double_attach_and_second_cube_rejected(self):
        self.assertTrue(self.grasp.attach("red_cube"))
        self.assertFalse(self.grasp.attach("red_cube"))
        self.assertFalse(self.grasp.attach("blue_cube"))

    def test_release_without_attachment_rejected(self):
        self.assertFalse(self.grasp.release("red_cube"))
        self.grasp._transition.assert_not_called()

    def test_initial_reset_is_required_before_pick(self):
        self.assertTrue(self.grasp.ready_for_pick())
        self.grasp._ready = False
        self.assertFalse(self.grasp.ready_for_pick())
        self.grasp._ready = True
        self.grasp._state["purple_cube"] = "attached"
        self.assertFalse(self.grasp.ready_for_pick())

    def test_measured_fingers_must_both_be_closed(self):
        original = PhysicalGraspManager._closed.__get__(self.grasp)
        self.grasp._joints = {FINGER_JOINTS[0]: CLOSED_POSITION}
        self.assertFalse(original())
        self.grasp._joints[FINGER_JOINTS[1]] = CLOSED_POSITION
        self.assertTrue(original())
        self.grasp._joints[FINGER_JOINTS[1]] += 0.004
        self.assertFalse(original())


class RobotSkillsPhysicalUnitTest(unittest.TestCase):
    def setUp(self):
        self.skills = RobotSkills.__new__(RobotSkills)
        self.physical = Mock()
        self.physical.attach.return_value = True
        self.physical.is_attached.return_value = True
        self.scene = Mock()
        object_msg = SimpleNamespace(
            id="red_cube", header=SimpleNamespace(frame_id="world"),
            pose=Pose(),
        )
        self.scene.get.return_value = SimpleNamespace(
            world=SimpleNamespace(collision_objects=[object_msg]),
            robot_state=SimpleNamespace(attached_collision_objects=[]),
        )
        self.scene.attach_object.return_value = False
        self.skills._physical_grasp = self.physical
        self.skills._scene_manager = self.scene
        self.skills._interface = Mock()
        self.skills._primitives = Mock()

    def test_moveit_attach_failure_rolls_back_physics_without_retreat(self):
        status = self.skills._physical_pick_attachment("red_cube", PoseStamped())
        self.assertEqual(status, SkillStatus.FAILED)
        self.physical.release.assert_called_once_with("red_cube")
        self.skills._primitives.retreat.assert_not_called()
        self.skills._interface.stop.assert_called()

    def test_physical_attach_failure_does_not_attach_moveit_or_retreat(self):
        self.physical.attach.return_value = False
        self.physical.is_attached.return_value = False
        status = self.skills._physical_pick_attachment("red_cube", PoseStamped())
        self.assertEqual(status, SkillStatus.FAILED)
        self.scene.attach_object.assert_not_called()
        self.skills._primitives.retreat.assert_not_called()

    def test_physical_path_never_calls_legacy_follower(self):
        self.skills._gazebo_sync = Mock()
        self.physical.attach.return_value = False
        self.physical.is_attached.return_value = False
        self.skills._physical_pick_attachment("red_cube", PoseStamped())
        self.skills._gazebo_sync.attach.assert_not_called()
        self.skills._gazebo_sync.set_world_pose.assert_not_called()

    def test_world_attached_duplicate_is_rejected(self):
        self.scene.get.return_value.robot_state.attached_collision_objects = [
            SimpleNamespace(object=SimpleNamespace(id="red_cube"))
        ]
        self.assertEqual(
            self.skills._physical_pick_attachment("red_cube", PoseStamped()), SkillStatus.FAILED
        )
        self.physical.attach.assert_not_called()


if __name__ == "__main__":
    unittest.main()
