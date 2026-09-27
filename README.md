# ur3_llm_control

Assignment 02 — UR3/UR3e Control using LLM + Skill-Based Planning.

M5 adds an assignment-local MoveIt Planning Scene manager while retaining the
M4 motion interface and M3 workcell and parallel-jaw gripper. The launch
composes the official
`ur_simulation_gz/ur_sim_control.launch.py` launch file with assignment-local
pedestal, table, cube, and placement-zone entities. An assignment-local Xacro
wrapper invokes the vendor UR macro with the mounting pose from `scene.yaml`;
the vendor simulation stack and robot geometry are not copied or modified.

The gripper mounts directly to `tool0`, extends along tool0 `+Z`, and closes
symmetrically along tool0 `+/-X` using two explicit prismatic joints. Its clear
opening spans 0--80 mm by joint limits; `close()` and `open()` command 47 mm and
75 mm respectively to preserve limit margins while comfortably clearing each
45 mm cube. The
fixed `gripper_tcp` frame is translated `(0, 0, 0.080) m` from `tool0` with no
rotation and lies at the intended grasp center between the fingers.

## M5 Planning Scene

`planning_scene.py` reads the same `config/scene.yaml` used by Gazebo, converts
the pedestal, table, and three cubes to MoveIt box collision objects, and
applies them through `/apply_planning_scene`. It verifies names, dimensions,
frames, and poses against `/get_planning_scene` at startup. Placement zones are
intentionally omitted because they are visual/semantic markers rather than
obstacles.

The acceptance executable also verifies that MoveIt's robot description still
contains the gripper and `gripper_tcp`, executes one safe collision-checked
target, and confirms that a YAML-derived target inside the table is rejected
without arm motion:

```bash
ros2 run ur3_llm_control planning_scene_test
```

## M4 arm motion

`MoveItArmInterface` accepts joint targets and `gripper_tcp` poses in
`base_link`. Pose requests are converted through the fixed 80 mm TCP offset,
checked with collision-aware IK and state validity, planned by MoveIt, and only
then sent to MoveIt's `ExecuteTrajectory` action. The interface never publishes
hand-written arm trajectories. It reports explicit planning, execution,
invalid-target, cancellation, and readiness results and exposes `stop()`.

`config/robot_motion.yaml` defines a collision-checked, bent-elbow HOME posture,
5% velocity/acceleration scaling, and the small Cartesian acceptance target.
The bent elbow avoids the singular straight-arm stock `up` posture. M4 targets
only the six arm joints, and the acceptance test verifies both gripper joints
remain unchanged.

## M2.5 workcell

`config/scene.yaml` is the canonical source for the robot mounting transform
and all workcell dimensions, initial poses, masses, and colors. Values use metres, radians, and kilograms.
Poses are XYZ/RPY in Gazebo's `world` frame, and pose Z means the geometric
center of the box.

- One static, collidable `robot_pedestal`: 0.25 × 0.25 × 0.30 m, centered at
  `(0, 0, 0.15)`. Its top and the UR `base_link` mounting plane are both at
  `z=0.30 m`.
- One static, collidable `manipulation_table`: 0.60 × 0.40 × 0.30 m, centered
  at `(0, 0.38, 0.15)`, with tabletop height 0.30 m. The robot is centered on
  the near table edge. The 0.055 m pedestal-to-table gap and approximately
  0.116 m UR base-mesh-to-table clearance avoid intersection.
- Three dynamic 0.045 m cubes (`red_cube`, `yellow_cube`, `blue_cube`), each
  with 0.08 kg mass, box collision geometry, and solid-box inertia. They spawn
  1 mm above the tabletop and settle under physics.
- Three static, visual-only 0.080 × 0.080 × 0.002 m placement markers
  (`zone_a`, `zone_b`, `zone_c`). They have no collision geometry and therefore
  are semantic targets rather than obstacles.

The cubes and zones occupy separate rows and are spaced 0.12 m apart. This
leaves 0.075 m clear between adjacent 0.045 m cubes. Their straight-line
distances from `base_link` are 0.331–0.352 m for cubes and 0.240–0.268 m for
zones, within the UR3e's nominal 0.50 m reach as a geometric sanity check.
Collision-aware preliminary M2.5 checks also succeeded for `tool0` poses
0.10 m above all six targets with a vertical-down orientation. M3 itself does
not command the UR arm.

## Build

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ur3_llm_control
source install/setup.bash
```

## Launch the workcell

The verified local runtime currently requires loopback discovery variables.
They are intentionally not embedded in source code:

```bash
export IGN_IP=127.0.0.1
export ROS_LOCALHOST_ONLY=1
ros2 launch ur3_llm_control workcell.launch.py
```

After launch, run the completion-aware gripper acceptance sequence:

```bash
ros2 run ur3_llm_control gripper_test
```

Run the M4 motion and invalid-target acceptance sequence in a second terminal:

```bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 run ur3_llm_control moveit_test
```

The M4 sequence is current state -> HOME -> nearby `gripper_tcp` pose -> HOME,
followed by an unreachable target that must fail without moving the arm.

Headless launch:

```bash
ros2 launch ur3_llm_control workcell.launch.py gazebo_gui:=false
```

Select the UR3 instead of the default UR3e:

```bash
ros2 launch ur3_llm_control workcell.launch.py ur_type:=ur3
```

The M1 node-only launch remains available as `llm_robot.launch.py`; it is not
part of the M2 workcell launch.

## Scope boundary

M5 does not implement attachment, detachment, Gazebo-to-MoveIt object-state
updates, robot skills, pick/place, validation/execution orchestration, student
mapping, or LLM/9Router execution. Those belong to later milestones.
