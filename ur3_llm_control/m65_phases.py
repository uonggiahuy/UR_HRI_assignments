"""Short, stateful M6.5 live-validation phases for the existing M6 primitives."""

from __future__ import annotations

import argparse
import math
import os
import time

from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import PlanningSceneComponents
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_geometry_msgs import do_transform_pose
import yaml

from ur3_llm_control.gazebo_sync import GazeboAttachmentSynchronizer
from ur3_llm_control.gripper import ParallelJawGripper
from ur3_llm_control.motion_primitives import ManipulationMotionPrimitives
from ur3_llm_control.moveit_interface import ARM_JOINTS, MotionResult, MoveItArmInterface
from ur3_llm_control.planning_scene import PlanningSceneManager
from ur3_llm_control.workcell_scene import load_scene


OBJECT = "red_cube"
OTHER_OBJECTS = ("yellow_cube", "blue_cube")
TIMEOUT = 10.0
POSE_TOLERANCE = 0.012


def _package_file(relative_path: str) -> str:
    return os.path.join(get_package_share_directory("ur3_llm_control"), relative_path)


def _yaml(relative_path: str) -> dict:
    with open(_package_file(relative_path), encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _same_pose(left: Pose, right: Pose, tolerance: float = 1e-5) -> bool:
    return all(
        math.isclose(a, b, abs_tol=tolerance)
        for a, b in zip(
            (left.position.x, left.position.y, left.position.z,
             left.orientation.x, left.orientation.y, left.orientation.z, left.orientation.w),
            (right.position.x, right.position.y, right.position.z,
             right.orientation.x, right.orientation.y, right.orientation.z, right.orientation.w),
        )
    )


def _require(label: str, result: MotionResult) -> None:
    if result != MotionResult.SUCCESS:
        raise RuntimeError(f"{label}: {result.value}")


def _controllers(node: Node) -> None:
    client = node.create_client(ListControllers, "/controller_manager/list_controllers")
    try:
        if not client.wait_for_service(timeout_sec=TIMEOUT):
            raise RuntimeError("controller manager unavailable")
        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=TIMEOUT)
        response = future.result() if future.done() else None
        states = {} if response is None else {item.name: item.state for item in response.controller}
        missing = [
            name for name in (
                "joint_trajectory_controller", "gripper_controller", "joint_state_broadcaster"
            ) if states.get(name) != "active"
        ]
        if missing:
            raise RuntimeError("inactive controllers: " + ", ".join(missing))
    finally:
        node.destroy_client(client)


def _scene_state(manager: PlanningSceneManager, expected: str):
    scene = manager.get(TIMEOUT)
    if scene is None:
        raise RuntimeError("/get_planning_scene unavailable")
    world = {item.id: item for item in scene.world.collision_objects}
    attached = {item.object.id: item for item in scene.robot_state.attached_collision_objects}
    if expected == "WORLD":
        if OBJECT not in world or OBJECT in attached:
            raise RuntimeError(f"{OBJECT} is not WORLD-only")
    else:
        if OBJECT in world or OBJECT not in attached:
            raise RuntimeError(f"{OBJECT} is not ATTACHED-only")
    if any(name not in world or name in attached for name in OTHER_OBJECTS):
        raise RuntimeError("non-target cubes did not remain WORLD-only")
    return scene, world, attached


def _assert_no_broad_acm(
    scene, manager: PlanningSceneManager, *, allow_target_touch: bool = False
) -> None:
    names = list(scene.allowed_collision_matrix.entry_names)
    values = scene.allowed_collision_matrix.entry_values
    for row_index, row in enumerate(values):
        for column_index, enabled in enumerate(row.enabled):
            if not enabled:
                continue
            pair = {names[row_index], names[column_index]}
            if OBJECT not in pair:
                continue  # Stock robot self-collision allowances are unchanged.
            other = next(iter(pair - {OBJECT}), OBJECT)
            if not allow_target_touch or other not in manager.touch_links:
                raise RuntimeError(f"unexpected {OBJECT} ACM allowance: {sorted(pair)}")


def _context(node: Node):
    scene = load_scene(_package_file("config/scene.yaml"))
    motion = _yaml("config/robot_motion.yaml")
    settings = motion["manipulation"]
    manager = PlanningSceneManager(
        node, scene, attachment_link=str(settings["attachment_link"]),
        touch_links=tuple(str(link) for link in settings["touch_links"]),
    )
    interface = MoveItArmInterface(
        node, planning_group=str(motion["planning_group"]),
        planning_frame=str(motion["planning_frame"]), planning_tip=str(motion["planning_tip"]),
        application_tip=str(motion["application_tip"]),
        tool0_to_tcp_z=float(motion["tool0_to_tcp_z"]),
        velocity_scaling=float(motion["velocity_scaling"]),
        acceleration_scaling=float(motion["acceleration_scaling"]),
        planning_time=float(motion["planning_time"]), planning_attempts=int(motion["planning_attempts"]),
    )
    if not interface.wait_until_ready():
        raise RuntimeError("MoveIt not ready")
    return scene, motion, manager, interface, ManipulationMotionPrimitives(interface, scene, motion)


def _wait_for_gazebo_pose(
    node: Node, sync: GazeboAttachmentSynchronizer, expected: Pose, timeout: float = 5.0
) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        actual = sync.model_pose(OBJECT)
        if actual is not None and _same_pose(actual, expected, POSE_TOLERANCE):
            return
        rclpy.spin_once(node, timeout_sec=0.1)
    raise RuntimeError("Gazebo cube pose did not converge to the gripper transform")


def _world_from_link(node: Node, manager: PlanningSceneManager, link_name: str):
    deadline = time.monotonic() + TIMEOUT
    while time.monotonic() < deadline:
        try:
            return manager._tf_buffer.lookup_transform(
                "world", link_name, rclpy.time.Time(), timeout=Duration(seconds=0.1)
            )
        except Exception:
            rclpy.spin_once(node, timeout_sec=0.1)
    raise RuntimeError(f"TF world -> {link_name} was unavailable")


def _phase_a(node: Node) -> None:
    scene, motion, manager, interface, primitives = _context(node)
    gripper = ParallelJawGripper(node)
    try:
        _controllers(node)
        _scene_state(manager, "WORLD")
        home = {name: float(value) for name, value in motion["home"].items()}
        if set(home) != set(ARM_JOINTS):
            raise RuntimeError("HOME is incomplete")
        _require("HOME", interface.move_to_joint_configuration(home))
        gripper.open()
        _require("move_above", primitives.move_above(OBJECT))
        if not manager.allow_grasp_contact(OBJECT):
            raise RuntimeError("target-only grasp contact allowance failed")
        _require("descend", primitives.descend(OBJECT))
        _assert_no_broad_acm(manager.get(TIMEOUT), manager, allow_target_touch=True)
        node.get_logger().info("M6.5 Phase A PASS")
    finally:
        gripper.destroy(); interface.destroy(); manager.destroy()


def _phase_b(node: Node) -> None:
    _, _, manager, interface, primitives = _context(node)
    gripper = ParallelJawGripper(node)
    sync = GazeboAttachmentSynchronizer(node)
    try:
        _controllers(node)
        initial = manager.get(TIMEOUT)
        if initial is None:
            raise RuntimeError("/get_planning_scene unavailable")
        initial_world = {item.id: item for item in initial.world.collision_objects}
        initial_attached = {
            item.object.id: item for item in initial.robot_state.attached_collision_objects
        }
        expected_world = next(
            item for item in manager._expected.world.collision_objects if item.id == OBJECT
        )
        world_pose = initial_world.get(OBJECT, expected_world).pose
        if OBJECT not in initial_attached:
            if OBJECT not in initial_world:
                raise RuntimeError("red_cube is in neither WORLD nor ATTACHED state")
            gripper.close()
            if not manager.attach_object(OBJECT):
                raise RuntimeError("attach_object did not create an authoritative attachment")
        if not manager.clear_grasp_contact(OBJECT):
            raise RuntimeError("could not clear pre-grasp target contact")
        scene, _, attached = _scene_state(manager, "ATTACHED")
        item = attached[OBJECT]
        if item.link_name != manager.attachment_link:
            raise RuntimeError("incorrect attachment link")
        if set(item.touch_links) != set(manager.touch_links):
            raise RuntimeError("incorrect touch links")
        transform = _world_from_link(node, manager, item.link_name)
        reconstructed = do_transform_pose(item.object.pose, transform)
        if not _same_pose(reconstructed, world_pose, tolerance=0.002):
            raise RuntimeError(
                "T_tcp_cube does not reconstruct T_world_cube: "
                f"expected=({world_pose.position.x:.4f}, {world_pose.position.y:.4f}, {world_pose.position.z:.4f}) "
                f"actual=({reconstructed.position.x:.4f}, {reconstructed.position.y:.4f}, {reconstructed.position.z:.4f})"
            )
        _assert_no_broad_acm(scene, manager)
        if not sync.attach(OBJECT, item.link_name, item.object.pose):
            raise RuntimeError("Gazebo synchronization could not start")
        _require("retreat", primitives.retreat(OBJECT))
        relative = manager.attached_pose(OBJECT)
        if relative is None or not _same_pose(relative, item.object.pose):
            raise RuntimeError("attached grasp offset changed during retreat")
        transform = _world_from_link(node, manager, item.link_name)
        _wait_for_gazebo_pose(node, sync, do_transform_pose(relative, transform))
        node.get_logger().info("M6.5 Phase B PASS")
    finally:
        sync.destroy(); gripper.destroy(); interface.destroy(); manager.destroy()


def _phase_c_move(node: Node) -> None:
    _, _, manager, interface, primitives = _context(node)
    sync = GazeboAttachmentSynchronizer(node)
    try:
        _controllers(node); _, _, attached = _scene_state(manager, "ATTACHED")
        item = attached[OBJECT]
        if not sync.attach(OBJECT, item.link_name, item.object.pose):
            raise RuntimeError("Gazebo synchronization could not start")
        _require("move_above release", primitives.move_above(OBJECT))
        _require("descend release", primitives.descend(OBJECT))
        transform = _world_from_link(node, manager, item.link_name)
        _wait_for_gazebo_pose(node, sync, do_transform_pose(item.object.pose, transform))
        node.get_logger().info("M6.5 Phase C move PASS")
    finally:
        sync.destroy(); interface.destroy(); manager.destroy()


def _phase_c_finish(node: Node) -> None:
    _, motion, manager, interface, primitives = _context(node)
    gripper = ParallelJawGripper(node)
    sync = GazeboAttachmentSynchronizer(node)
    try:
        _controllers(node)
        scene, _, attached = _scene_state(manager, "ATTACHED")
        item = attached[OBJECT]
        if not sync.attach(OBJECT, item.link_name, item.object.pose):
            raise RuntimeError("Gazebo synchronization could not start")
        transform = _world_from_link(node, manager, item.link_name)
        expected_gazebo_pose = do_transform_pose(item.object.pose, transform)
        _wait_for_gazebo_pose(node, sync, expected_gazebo_pose)
        release_pose = primitives.target_world_pose(OBJECT)
        if not sync.release(release_pose):
            raise RuntimeError("Gazebo release pose update failed")
        if not manager.detach_object(OBJECT, release_pose):
            raise RuntimeError("detach_object failed")
        scene, world, attached = _scene_state(manager, "WORLD")
        if attached:
            raise RuntimeError("AttachedCollisionObject remains after detach")
        _assert_no_broad_acm(scene, manager, allow_target_touch=True)
        _wait_for_gazebo_pose(node, sync, release_pose.pose)
        gripper.open()
        _require("retreat", primitives.retreat(OBJECT))
        # After the fingers have cleared, write the same release pose once
        # more so the dynamic model has no residual contact impulse.
        if not sync.set_world_pose(OBJECT, release_pose):
            raise RuntimeError("Gazebo final release pose update failed")
        _wait_for_gazebo_pose(node, sync, release_pose.pose)
        if not manager.clear_grasp_contact(OBJECT):
            raise RuntimeError("could not clear post-release target contact")
        _assert_no_broad_acm(manager.get(TIMEOUT), manager)
        home = {name: float(value) for name, value in motion["home"].items()}
        _require("HOME", interface.move_to_joint_configuration(home))
        _controllers(node)
        node.get_logger().info("M6.5 Phase C finish PASS")
    finally:
        sync.destroy(); gripper.destroy(); interface.destroy(); manager.destroy()


def _phase_c_recover(node: Node) -> None:
    """Finish a released-cube withdrawal if Phase C was interrupted mid-retreat."""
    _, motion, manager, interface, primitives = _context(node)
    try:
        _controllers(node); _scene_state(manager, "WORLD")
        if not manager.allow_grasp_contact(OBJECT):
            raise RuntimeError("could not restore target-only withdrawal contact")
        _require("retreat", primitives.retreat(OBJECT))
        if not manager.clear_grasp_contact(OBJECT):
            raise RuntimeError("could not clear post-release target contact")
        home = {name: float(value) for name, value in motion["home"].items()}
        _require("HOME", interface.move_to_joint_configuration(home))
        _controllers(node)
        node.get_logger().info("M6.5 Phase C recovery PASS")
    finally:
        interface.destroy(); manager.destroy()


def _phase_verify(node: Node) -> None:
    """Read-only final M6.5 state and controller verification."""
    _, _, manager, interface, _ = _context(node)
    sync = GazeboAttachmentSynchronizer(node)
    try:
        _controllers(node)
        scene, world, attached = _scene_state(manager, "WORLD")
        if attached:
            raise RuntimeError("AttachedCollisionObject remains after M6.5")
        _assert_no_broad_acm(scene, manager)
        expected = world[OBJECT].pose
        _wait_for_gazebo_pose(node, sync, expected)
        node.get_logger().info("M6.5 final read-only verification PASS")
    finally:
        sync.destroy(); interface.destroy(); manager.destroy()


def main(args=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("phase", choices=("a", "b", "c_move", "c_finish", "c_recover", "verify"))
    phase = parser.parse_args(args=args).phase
    rclpy.init(args=args)
    node = Node(f"m65_phase_{phase}")
    try:
        {"a": _phase_a, "b": _phase_b, "c_move": _phase_c_move, "c_finish": _phase_c_finish, "c_recover": _phase_c_recover, "verify": _phase_verify}[phase](node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
