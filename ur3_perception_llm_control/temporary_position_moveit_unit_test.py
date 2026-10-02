"""M9 plan-only local scene and no-execution checks."""

from pathlib import Path
import unittest
from unittest.mock import Mock

import yaml
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject, PlanningScene, RobotState, RobotTrajectory

from ur3_perception_llm_control.motion_primitives import ManipulationMotionPrimitives
from ur3_perception_llm_control.moveit_interface import ARM_JOINTS, MotionResult, MoveItArmInterface
from ur3_perception_llm_control.perception_scene import PerceptionPlanningSceneSynchronizer
from ur3_perception_llm_control.perception_scene_unit_test import A, FakeManager, snapshot
from ur3_perception_llm_control.perception_state import WorkcellGeometry
from ur3_perception_llm_control.temporary_position import TemporaryPositionPlanner
from ur3_perception_llm_control.temporary_position_moveit import MoveItTemporaryFeasibility
from ur3_perception_llm_control.workcell_scene import load_scene


ROOT = Path(__file__).resolve().parents[1]


class M9MoveItUnitTest(unittest.TestCase):
    def test_local_attached_cube_diff_never_mutates_authoritative_world(self):
        scene = load_scene(str(ROOT / "config/scene.yaml"))
        motion = yaml.safe_load((ROOT / "config/robot_motion.yaml").read_text())
        settings = yaml.safe_load((ROOT / "config/temporary_position.yaml").read_text())
        geometry = WorkcellGeometry.from_scene_file(ROOT / "config/scene.yaml")
        observed = snapshot(A, geometry)
        manager = FakeManager(scene)
        report = PerceptionPlanningSceneSynchronizer(manager, scene, geometry).apply_snapshot(observed, 10.1)
        interface = Mock(planning_frame="base_link")
        interface.plan_pose_sequence.return_value = MotionResult.SUCCESS
        primitives = ManipulationMotionPrimitives(interface, scene, motion)
        checker = MoveItTemporaryFeasibility(interface, primitives, scene, motion,
                                             observed, report, "red_cube")
        candidate = TemporaryPositionPlanner(scene, geometry, settings).ranked_candidates(
            "red_cube", observed, 10.1)[0][0]
        self.assertTrue(checker(candidate))
        local = interface.plan_pose_sequence.call_args.kwargs["scene_diff"]
        self.assertEqual([(item.id, item.operation) for item in local.world.collision_objects],
                         [("red_cube", CollisionObject.REMOVE)])
        self.assertEqual([item.object.id for item in local.robot_state.attached_collision_objects],
                         ["red_cube"])
        self.assertEqual(local.robot_state.attached_collision_objects[0].link_name, "gripper_tcp")
        self.assertEqual(interface.plan_pose_sequence.call_args.kwargs["start_joint_positions"],
                         motion["home"])
        poses = interface.plan_pose_sequence.call_args.args[0]
        self.assertEqual(len(poses), 2)
        self.assertAlmostEqual(poses[0].pose.position.z - poses[1].pose.position.z,
                               motion["manipulation"]["approach_clearance"])
        interface.move_to_pose.assert_not_called()
        self.assertEqual(len(manager.applied), 1)
        self.assertFalse(manager.scene.robot_state.attached_collision_objects)

    def test_pose_sequence_is_plan_only(self):
        interface = MoveItArmInterface.__new__(MoveItArmInterface)
        interface.wait_until_ready = Mock(return_value=True)
        interface._current_robot_state = Mock(return_value=RobotState())
        interface._state_with_arm_positions = Mock(side_effect=lambda state, joints: state)
        interface._state_is_valid = Mock(return_value=True)
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = list(ARM_JOINTS)
        from trajectory_msgs.msg import JointTrajectoryPoint
        point = JointTrajectoryPoint()
        point.positions = [0.0] * len(ARM_JOINTS)
        trajectory.joint_trajectory.points = [point]
        interface._plan_pose_target = Mock(return_value=(MotionResult.SUCCESS, trajectory))
        interface._execute = Mock()
        result = interface.plan_pose_sequence(
            [PoseStamped(), PoseStamped()], start_joint_positions={name: 0.0 for name in ARM_JOINTS},
            scene_diff=PlanningScene(),
        )
        self.assertEqual(result, MotionResult.SUCCESS)
        self.assertEqual(interface._plan_pose_target.call_count, 2)
        interface._execute.assert_not_called()


if __name__ == "__main__":
    unittest.main()
