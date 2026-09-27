# Assignment 02 — Current Project State

- **Completed through**: M6
- **Next milestone**: M7
- **Branch**: `assignments_2`

## M6 Manipulation Primitives

`ManipulationMotionPrimitives` is the small reusable M6 API. It accepts an
object or zone name from `config/scene.yaml` and plans only through the existing
`MoveItArmInterface` using `gripper_tcp` application poses:

- `move_above(target)` applies `manipulation.approach_clearance`.
- `descend(target)` applies `manipulation.grasp_z_offset`.
- `retreat()` applies `manipulation.retreat_clearance` for the most recent
  target.

Target coordinates remain in `scene.yaml`; configurable orientation, offsets,
attachment link, and touch links are in `config/robot_motion.yaml`. The active
model uses planning group `ur_manipulator`, planning tip `tool0`, and application
tip `gripper_tcp`.

## Planning Scene Attachment Lifecycle

`PlanningSceneManager.attach_object(object_name)` reads the authoritative WORLD
object with `/get_planning_scene`, transforms its current pose into the actual
`gripper_tcp` link frame, then applies one diff containing WORLD `REMOVE` and an
`AttachedCollisionObject` `ADD`. The attachment uses the narrowly scoped
gripper touch links `gripper_base`, `left_finger_link`, and `right_finger_link`.
The target cube/touch-link ACM exception is enabled only during physical
contact before attachment and the immediate post-release withdrawal, then
cleared before HOME; no global or non-target collision permission is changed.

`detach_object(object_name, world_pose)` applies one inverse diff: attached
`REMOVE` plus YAML-geometry WORLD `ADD` at the supplied `world` pose. Thus a
cube is never intentionally present in both lifecycle states, and its geometry
continues to come from `scene.yaml`.

`motion_primitives_test` is the one-cube M6 integration executable. It runs
HOME, gripper open, approach, descend, close, attach, retreat, safe return,
placement descend, detach, open, retreat, and HOME; it checks the authoritative
scene as WORLD → ATTACHED → WORLD and confirms all arm, gripper, and
joint-state controllers remain active.

## M6.5 Live Validation and Gazebo Synchronization

M6.5 was live-validated in the UR3e Fortress workcell with `IGN_IP=127.0.0.1`
and `ROS_LOCALHOST_ONLY=1`. The test was split into short phases because the
execution environment limits a standalone command to about 30 seconds. The
authoritative scene was verified as `red_cube` WORLD-only before attachment,
ATTACHED-only during the collision-aware retreat, and WORLD-only with no
`AttachedCollisionObject` after detach. `joint_trajectory_controller`,
`gripper_controller`, and `joint_state_broadcaster` remained active.

`GazeboAttachmentSynchronizer` is assignment-local and does not alter MoveIt
collision ownership. It captures the MoveIt attachment offset
`T_tcp_cube = inverse(T_world_tcp) * T_world_cube` and writes only the
simulated Fortress model pose while attached through `/world/empty/set_pose`.
On release it writes the requested world pose; after the fingers retreat it
writes that pose once more to remove residual contact impulse before normal
Gazebo physics resumes. No vendor package, global physics setting, or broad
collision allowance is modified.
