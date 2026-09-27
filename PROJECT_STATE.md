# Assignment 02 — Project State

## Status

- **Completed through**: M5 (MoveIt Planning Scene)
- **Next milestone**: M6
- **Branch**: `assignments_2`

## Robot and Runtime

- ROS 2 Humble in the `ros2_workspace` environment
- UR3e by default (`ur_type` remains configurable for UR3)
- Gazebo Fortress / Ignition 6
- MoveIt planning group `ur_manipulator`, planning frame `base_link`, planning
  tip `tool0`, application tip `gripper_tcp`
- Assignment-local M3 parallel-jaw gripper and fixed `gripper_tcp` remain in
  the MoveIt robot model
- Active motion controllers: `joint_trajectory_controller` and
  `gripper_controller`

## Planning Scene Architecture and API

- `config/scene.yaml` is the single source of truth for Gazebo and MoveIt
  workcell names, dimensions, poses, and colors.
- `ur3_llm_control/planning_scene.py` converts collidable YAML box models to
  `moveit_msgs/CollisionObject` messages in the YAML `world` frame.
- `PlanningSceneManager` applies one scene diff through
  `/apply_planning_scene` and reads the authoritative scene through
  `/get_planning_scene`.
- The manager verifies returned object IDs, box dimensions, frames, and poses
  against the generated YAML-backed scene at startup.
- `moveit.launch.py` starts the manager with the selected `scene_config`.

## Collision Objects Currently Managed

- `robot_pedestal`
- `manipulation_table`
- `red_cube`
- `yellow_cube`
- `blue_cube`

`zone_a`, `zone_b`, and `zone_c` remain Gazebo visual/semantic markers and are
not MoveIt collision objects.

## Verified M5 Behavior

- A safe `gripper_tcp` target plans and executes with the Planning Scene active.
- A target derived from `scene.yaml` and placed inside the table is rejected by
  collision-aware MoveIt IK without trajectory execution or arm motion.
- RViz displays the pedestal, table, and three colored cubes in the Planning
  Scene.

## Current Limitation

M5 represents cube poses from their initial values in `scene.yaml`; it does not
synchronize later Gazebo physics motion or implement attach/detach or world-state
updates. Those behaviors remain outside M5.
