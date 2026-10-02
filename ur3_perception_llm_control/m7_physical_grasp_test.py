"""Single-cube M7 live diagnostic; Gazebo poses are read-only test oracles."""

from __future__ import annotations

import argparse
import math
import time

import rclpy

from ur3_perception_llm_control.cube_detection_test import FrameCollector
from ur3_perception_llm_control.cube_detector import CubeDetector, PerceptionConfig, evaluate_stability
from ur3_perception_llm_control.gripper import ParallelJawGripper
from ur3_perception_llm_control.motion_primitives import ManipulationMotionPrimitives
from ur3_perception_llm_control.moveit_interface import ARM_JOINTS, MotionResult, MoveItArmInterface
from ur3_perception_llm_control.perception_scene import PerceptionPlanningSceneSynchronizer
from ur3_perception_llm_control.perception_state import PerceptionSnapshot, WorkcellGeometry
from ur3_perception_llm_control.physical_grasp import PhysicalGraspManager, model_pose
from ur3_perception_llm_control.planar_mapper import Calibration, CubeTopMapper, PlanarMapper
from ur3_perception_llm_control.planning_scene import PlanningSceneManager
from ur3_perception_llm_control.robot_skills import RobotSkills, SkillStatus
from ur3_perception_llm_control.robot_skills_test import _package_file, _require_controllers, _yaml
from ur3_perception_llm_control.workcell_scene import load_scene


OBJECT = "red_cube"
ZONE = "zone_a"


def _require(label: str, condition: bool) -> None:
    if not condition:
        raise RuntimeError(f"M7 {label} failed; stopping motion")
    print(f"M7 {label}: PASS", flush=True)


def _pose() -> tuple[float, float, float]:
    pose = model_pose(OBJECT)
    if pose is None:
        raise RuntimeError("Gazebo red_cube read-only pose unavailable")
    return pose.position.x, pose.position.y, pose.position.z


def _scene_state(manager: PlanningSceneManager, expected: str) -> bool:
    scene = manager.get()
    if scene is None:
        return False
    world = {item.id for item in scene.world.collision_objects}
    attached = {item.object.id for item in scene.robot_state.attached_collision_objects}
    return (OBJECT in world, OBJECT in attached) == (
        (True, False) if expected == "WORLD" else (False, True)
    )


def _hold(node, manager: PlanningSceneManager, physical: PhysicalGraspManager, seconds: float) -> None:
    initial = _pose()
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if not physical.is_attached(OBJECT):
            raise RuntimeError("M7 physical joint detached during hold; stopping motion")
    final = _pose()
    _require("hold height", final[2] > 0.36)
    _require("hold drift", math.dist(initial, final) < 0.015)
    _require("MoveIt attached during hold", _scene_state(manager, "ATTACHED"))
    print(f"M7 hold {seconds:.1f}s: cube {initial} -> {final}", flush=True)


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description="Camera-driven physical red-cube pick and release")
    parser.add_argument("--scene", default=_package_file("config/scene.yaml"))
    options, ros_args = parser.parse_known_args(args)
    scene_file = options.scene
    config = PerceptionConfig.from_file(_package_file("config/perception.yaml"))
    calibration = Calibration.from_file(_package_file("config/camera_calibration.yaml"))
    detector = CubeDetector(config, CubeTopMapper(PlanarMapper(calibration),
                                                  config.camera_xyz, config.cube_top_z_m))
    geometry = WorkcellGeometry.from_scene_file(scene_file)
    rclpy.init(args=ros_args)
    node = FrameCollector(detector, config.frames, node_name="m8_perception_pick_test", use_sim_time=True)
    interface = gripper = manager = physical = None
    try:
        scene = load_scene(scene_file)
        motion = _yaml("config/robot_motion.yaml")
        settings = motion["manipulation"]
        interface = MoveItArmInterface(
            node, planning_group=str(motion["planning_group"]),
            planning_frame=str(motion["planning_frame"]),
            planning_tip=str(motion["planning_tip"]),
            application_tip=str(motion["application_tip"]),
            tool0_to_tcp_z=float(motion["tool0_to_tcp_z"]),
            velocity_scaling=float(motion["velocity_scaling"]),
            acceleration_scaling=float(motion["acceleration_scaling"]),
            planning_time=float(motion["planning_time"]),
            planning_attempts=int(motion["planning_attempts"]),
        )
        manager = PlanningSceneManager(
            node, scene, attachment_link=str(settings["attachment_link"]),
            touch_links=tuple(str(link) for link in settings["touch_links"]),
        )
        gripper = ParallelJawGripper(node)
        physical = PhysicalGraspManager(node)
        synchronizer = PerceptionPlanningSceneSynchronizer(manager, scene, geometry)
        last_stamp = [-1.0]
        last_snapshot = [None]

        def snapshot_source() -> PerceptionSnapshot:
            deadline = time.monotonic() + 25.0
            while rclpy.ok() and time.monotonic() < deadline:
                if (len(node.samples) == config.frames and node.timestamps[-1] > last_stamp[0]
                        and evaluate_stability(list(node.samples), config).stable):
                    result = PerceptionSnapshot.from_detections(node.samples[-1], geometry)
                    result.require_fresh(node.get_clock().now().nanoseconds / 1e9)
                    last_stamp[0] = result.observation_timestamp_sec
                    last_snapshot[0] = result
                    print(f"M8 RGB pick snapshot: {OBJECT} XY={result.object_world_xy[OBJECT]}", flush=True)
                    return result
                rclpy.spin_once(node, timeout_sec=0.1)
            raise RuntimeError(f"No stable fresh RGB snapshot; failures={node.failures[-3:]}")

        home = {name: float(value) for name, value in motion["home"].items()}
        _require("HOME configuration", set(home) == set(ARM_JOINTS))
        primitives = ManipulationMotionPrimitives(interface, scene, motion)
        skills = RobotSkills(
            interface=interface, gripper=gripper, primitives=primitives,
            scene_manager=manager, physical_grasp=physical,
            snapshot_source=snapshot_source, scene_synchronizer=synchronizer,
            now_sec=lambda: node.get_clock().now().nanoseconds / 1e9,
            home_configuration=home,
        )
        _require("MoveIt ready", interface.wait_until_ready())
        _require_controllers(node)
        _require("all initial physical joints detached", physical.wait_until_ready())
        initial = _pose()
        _require("pick red_cube", skills.pick(OBJECT) == SkillStatus.SUCCESS)
        observed_xy = last_snapshot[0].object_world_xy[OBJECT]
        pick_z = synchronizer.table_top_z + synchronizer.cubes[OBJECT].size[2] / 2
        print(f"M8 pick target world XYZ=({observed_xy[0]:.6f},{observed_xy[1]:.6f},{pick_z:.6f}); "
              f"Gazebo pre-pick oracle XYZ={initial}; "
              f"camera-oracle XY error={math.hypot(observed_xy[0]-initial[0], observed_xy[1]-initial[1])*1000:.3f} mm",
              flush=True)
        _require("dual attachment", physical.is_attached(OBJECT) and _scene_state(manager, "ATTACHED"))
        lifted = _pose()
        _require("physical lift", lifted[2] > initial[2] + 0.06)
        _hold(node, manager, physical, 3.2)
        _require("lateral MoveIt motion", primitives.move_above(ZONE) == MotionResult.SUCCESS)
        lateral = _pose()
        _require("lateral physical transport", abs(lateral[1] - lifted[1]) > 0.045)
        _hold(node, manager, physical, 1.0)
        place_status = skills.place(OBJECT, ZONE)
        print(f"M8 place status={place_status} physical_attached={physical.is_attached(OBJECT)} "
              f"scene_WORLD={_scene_state(manager, 'WORLD')} "
              f"scene_ATTACHED={_scene_state(manager, 'ATTACHED')} "
              f"Gazebo_oracle={_pose()}", flush=True)
        _require("place and release", place_status == SkillStatus.SUCCESS)
        print(f"M8 post-release RGB XY={last_snapshot[0].object_world_xy[OBJECT]}; "
              f"location={last_snapshot[0].object_locations[OBJECT]}", flush=True)
        _require("dual release", not physical.is_attached(OBJECT) and _scene_state(manager, "WORLD"))
        release_pose = _pose()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        settled = _pose()
        support_z = synchronizer.table_top_z + synchronizer.cubes[OBJECT].size[2] / 2
        _require("physical settling", abs(settled[2] - support_z) < 0.018)
        _require("independent after retreat", math.dist(release_pose, settled) < 0.025)
        _require_controllers(node)
        print(f"M7 PASS: initial={initial}, lift={lifted}, lateral={lateral}, "
              f"release={release_pose}, settled={settled}", flush=True)
    finally:
        if physical is not None:
            physical.destroy()
        if gripper is not None:
            gripper.destroy()
        if manager is not None:
            manager.destroy()
        if interface is not None:
            interface.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
