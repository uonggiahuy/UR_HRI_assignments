"""M9 live RGB-to-MoveIt temporary-slot diagnostic; never executes motion."""

from __future__ import annotations

import argparse
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
import yaml

from ur3_perception_llm_control.cube_detection_test import FrameCollector
from ur3_perception_llm_control.cube_detector import CubeDetector, PerceptionConfig, evaluate_stability
from ur3_perception_llm_control.motion_primitives import ManipulationMotionPrimitives
from ur3_perception_llm_control.moveit_interface import MoveItArmInterface
from ur3_perception_llm_control.perception_scene import PerceptionPlanningSceneSynchronizer
from ur3_perception_llm_control.perception_state import PerceptionSnapshot, WorkcellGeometry
from ur3_perception_llm_control.physical_grasp import PhysicalGraspManager
from ur3_perception_llm_control.planar_mapper import Calibration, CubeTopMapper, PlanarMapper
from ur3_perception_llm_control.planning_scene import PlanningSceneManager
from ur3_perception_llm_control.temporary_position import TemporaryPositionPlanner
from ur3_perception_llm_control.temporary_position_moveit import MoveItTemporaryFeasibility
from ur3_perception_llm_control.workcell_scene import load_scene
from ur3_perception_llm_control.world_state import BLOCKS


def _yaml(path: Path) -> dict:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"{path} must be a mapping")
    return data


def run(options: argparse.Namespace) -> None:
    scene = load_scene(str(options.scene))
    motion = _yaml(options.motion)
    settings = _yaml(options.temporary)
    geometry = WorkcellGeometry.from_scene_file(options.scene)
    camera = PerceptionConfig.from_file(options.perception)
    calibration = Calibration.from_file(options.calibration)
    detector = CubeDetector(camera, CubeTopMapper(PlanarMapper(calibration),
                                                  camera.camera_xyz, camera.cube_top_z_m))
    rclpy.init()
    node = FrameCollector(detector, camera.frames, node_name="temporary_position_test",
                          use_sim_time=True)
    manager = interface = physical = None
    try:
        manipulation = motion["manipulation"]
        manager = PlanningSceneManager(
            node, scene, attachment_link=str(manipulation["attachment_link"]),
            touch_links=tuple(str(link) for link in manipulation["touch_links"]),
        )
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
        physical = PhysicalGraspManager(node)
        if not interface.wait_until_ready():
            raise RuntimeError("MoveIt/controller/joint state unavailable")
        if not physical.wait_until_ready() or not physical.ready_for_pick():
            raise RuntimeError("initial physical joints did not reach detached baseline")
        deadline = time.monotonic() + options.timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if (len(node.samples) == camera.frames
                    and evaluate_stability(list(node.samples), camera).stable):
                break
            rclpy.spin_once(node, timeout_sec=0.1)
        else:
            raise RuntimeError(f"no stable complete RGB sample; failures={node.failures[-3:]}")
        snapshot = PerceptionSnapshot.from_detections(node.samples[-1], geometry)
        now = node.get_clock().now().nanoseconds / 1e9
        snapshot.require_fresh(now)
        before_joints = interface.current_joint_positions()
        if before_joints is None:
            raise RuntimeError("joint feedback unavailable before feasibility checks")
        synchronizer = PerceptionPlanningSceneSynchronizer(manager, scene, geometry)
        report = synchronizer.apply_snapshot(snapshot, now)
        checker = MoveItTemporaryFeasibility(
            interface, ManipulationMotionPrimitives(interface, scene, motion),
            scene, motion, snapshot, report, options.object,
        )
        planner = TemporaryPositionPlanner(scene, geometry, settings)
        selected, stats = planner.find_temporary_position(
            options.object, snapshot, now, checker,
        )
        after_report = synchronizer.verify_snapshot(snapshot, attached_ids=frozenset())
        after_joints = interface.current_joint_positions()
        if after_joints is None or any(abs(before_joints[name] - after_joints[name]) > 1e-4
                                        for name in before_joints):
            raise RuntimeError("robot joint state changed during plan-only diagnostic")
        if after_report.attached_ids:
            raise RuntimeError("plan-only check mutated the authoritative attachment state")
        print(f"RGB frames={camera.frames}; snapshot_age={now-snapshot.observation_timestamp_sec:.3f}s; "
              f"scene_sync_error={report.max_sync_error_m*1000:.3f}mm")
        for name in BLOCKS:
            print(f"observed {name}: {snapshot.object_locations[name]} "
                  f"XY={snapshot.object_world_xy[name]}")
        print(f"generated={stats.generated} rejected_table={stats.rejected_table} "
              f"rejected_zone={stats.rejected_zone} rejected_cube={stats.rejected_cube} "
              f"rejected_fixed={stats.rejected_fixed} geometric_valid={stats.geometric_valid} "
              f"moveit_infeasible={stats.moveit_infeasible} moveit_feasible={stats.moveit_feasible}")
        print(f"selected={selected.candidate_id} XYZ=({selected.x:.6f},"
              f"{selected.y:.6f},{selected.z:.6f}) "
              f"min_clearance={selected.minimum_clearance_m*1000:.3f}mm")
        print("M9 LIVE PASS: camera-derived, authoritative scene verified, "
              "plan-only attached-cube feasibility, zero robot motion")
    finally:
        if physical is not None:
            physical.destroy()
        if interface is not None:
            interface.destroy()
        if manager is not None:
            manager.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv: list[str] | None = None) -> None:
    share = Path(get_package_share_directory("ur3_perception_llm_control"))
    parser = argparse.ArgumentParser(description="M9 plan-only temporary tabletop slot diagnostic")
    parser.add_argument("--scene", type=Path, default=share / "config/scene.yaml")
    parser.add_argument("--motion", type=Path, default=share / "config/robot_motion.yaml")
    parser.add_argument("--temporary", type=Path, default=share / "config/temporary_position.yaml")
    parser.add_argument("--perception", type=Path, default=share / "config/perception.yaml")
    parser.add_argument("--calibration", type=Path, default=share / "config/camera_calibration.yaml")
    parser.add_argument("--object", choices=BLOCKS, default="red_cube")
    parser.add_argument("--timeout", type=float, default=30.0)
    options = parser.parse_args(argv)
    if options.timeout <= 0:
        parser.error("timeout must be positive")
    run(options)


if __name__ == "__main__":
    main()
