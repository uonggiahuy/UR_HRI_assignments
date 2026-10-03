# Assignment 03 — UR3e Perception-Driven LLM Control

ROS 2 Humble package for a simulated UR3e workcell. Assignment-specific code remains in this package; Universal Robots repositories are upstream infrastructure.

## Architecture and safety

Natural-language request -> 9Router scene-aware LLM -> untrusted JSON -> TaskValidator, goal check, stale-world check -> SkillExecutor -> RobotSkills -> MoveIt 2/gripper -> Gazebo DetachableJoint -> RGB verification.

The five-block workcell has red, yellow, blue, green, and purple cubes plus Zones A, B, and C. OpenCV processes an overhead RGB camera. A calibrated homography maps cube-top image centers to XY; complete fresh detections become immutable symbolic WorldState snapshots and synchronize the authoritative MoveIt WORLD collision scene.

Gazebo Fortress DetachableJoint makes the physical gripper/cube constraint. MoveIt verifies WORLD -> ATTACHED -> WORLD; Gazebo keeps the cube dynamic and collidable, then detaches it on release. Active Assignment 03 transport performs no direct Gazebo object-pose writes.

Public LLM skills are only home(), pick(object), place(object, zone), and place_temp(object). The LLM receives symbolic state only: no coordinates, poses, joint values, trajectories, controllers, or temporary-position IDs. Every candidate must pass exact-schema validation, goal simulation, and a fresh RGB stale-world guard before SkillExecutor can run it.

For an occupied destination, Python chooses a collision-checked temporary position internally:

    pick(blue_cube) -> place_temp(blue_cube) -> pick(red_cube)
    -> place(red_cube, zone_b) -> home()

## Build and launch

    cd /root/ros2_ws
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install --packages-select ur3_perception_llm_control
    source install/setup.bash

Terminal 1 — start the workcell once:

    ros2 launch ur3_perception_llm_control workcell.launch.py

For the deterministic occupied-zone sequence used for acceptance:

    ros2 launch ur3_perception_llm_control workcell.launch.py \
      gazebo_gui:=false launch_rviz:=false \
      scene_config:=/root/ros2_ws/src/ur3_perception_llm_control/config/scene_m12_blocker.yaml

Terminal 2 — source the same ROS environment and set these environment variable names privately; never commit values:

    NINEROUTER_BASE_URL
    NINEROUTER_API_KEY
    NINEROUTER_MODEL

    ros2 run ur3_perception_llm_control assignment3_runtime \
      --scene /root/ros2_ws/src/ur3_perception_llm_control/config/scene_m12_blocker.yaml

The runtime initializes ROS, RGB collection, MoveIt, gripper, physical grasp,
planning-scene synchronizer, robot skills, and the 9Router planner once. Each
Command prompt creates a new RGB snapshot, WorldState, validator result,
stale-world check, and SkillExecutor transaction; prior LLM plans and temporary
reservations are discarded after every command.

    Assignment 03 ready.
    Command> Put the red cube in Zone B.
    TASK SUCCESS
    Command> Put the green cube in Zone B.
    TASK SUCCESS
    Command> Put the purple cube in Zone A.
    TASK SUCCESS
    Command> quit

Use exit, quit, or Ctrl-D for clean shutdown. Unsupported low-level requests
are rejected before an LLM request or robot motion and return to Command>. The
runtime returns to Command> after a failure only if MoveIt and physical grasp
state are verified safe; otherwise it terminates fail-closed.

The one-shot M12 executable remains diagnostic/acceptance tooling:

    ros2 run ur3_perception_llm_control m12_scene_aware_execution \
      'Put the red cube in Zone B.' \
      --scene /root/ros2_ws/src/ur3_perception_llm_control/config/scene_m12_blocker.yaml

Expected final state: red_cube=zone_b, blue_cube=table, held_object=none, five MoveIt WORLD objects, zero ATTACHED, detached grasp, open gripper, and HOME.

## Validation

    cd /root/ros2_ws
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    python3 -m unittest discover \
      -s src/ur3_perception_llm_control/ur3_perception_llm_control \
      -p '*_test.py'

Known non-blocking messages: Humble combined attach-diff warning with authoritative scene verification, missing Octomap 3D sensor configuration, Gazebo plugin/update-period warnings, and shutdown-only RViz/move_group Ctrl-C faults. A same-world DetachableJoint diagnostic can leave no fresh baseline event; use a clean workcell for normal execution.
