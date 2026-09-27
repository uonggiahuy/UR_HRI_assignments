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
It does not modify the global Allowed Collision Matrix.

`detach_object(object_name, world_pose)` applies one inverse diff: attached
`REMOVE` plus YAML-geometry WORLD `ADD` at the supplied `world` pose. Thus a
cube is never intentionally present in both lifecycle states, and its geometry
continues to come from `scene.yaml`.

`motion_primitives_test` is the one-cube M6 integration executable. It runs
HOME, gripper open, approach, descend, close, attach, retreat, safe return,
placement descend, detach, open, retreat, and HOME; it checks the authoritative
scene as WORLD → ATTACHED → WORLD and confirms both controllers remain active.

## Runtime Limitation

The M6 mechanism is a hybrid grasp: gripper command plus MoveIt Planning Scene
attachment. MoveIt's attached collision object follows the robot for planning
and RViz. Gazebo physics object following is not synchronized by M6; no
friction-only or unsafe teleportation workaround is added. A later demo may add
the smallest assignment-local Gazebo synchronization if it is required.
