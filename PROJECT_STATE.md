# Assignment 02 — Project State

## Status
- **Current Milestone**: Completed through M4
- **Next Milestone**: M5 (MoveIt Planning Scene)

## Robot & Environment
- **Robot Model**: UR3e (configurable `ur_type`)
- **ROS Distro**: ROS 2 Humble
- **Simulation**: Gazebo Fortress / Ignition 6 (`IGN_IP=127.0.0.1`, `ROS_LOCALHOST_ONLY=1`)

## Workcell Geometry
- **Pedestal**: Center `(0, 0, 0.15)`, size `(0.25, 0.25, 0.30)`
- **Robot Mounting**: `(0, 0, 0.30)` atop pedestal
- **Table**: Center `(0, 0.38, 0.15)`, size `(0.60, 0.40, 0.30)`, tabletop at `z=0.30`
- **Objects**: `red_cube`, `yellow_cube`, `blue_cube` (cube size `0.045 m`)
- **Drop Zones**: `zone_a`, `zone_b`, `zone_c` (visual-only, no collision)

## Gripper Configuration
- **Type**: Assignment-local parallel-jaw gripper
- **TCP Link**: `gripper_tcp` (`tool0 -> gripper_tcp` with translation `(0, 0, 0.080)` and identity rotation)
- **Finger Joints**: `left_finger_joint`, `right_finger_joint`
- **Stroke Limits**: Mechanical max `80 mm`, commanded open `75 mm`, commanded closed `4 mm`
- **Controller**: `gripper_controller`

## MoveIt 2 Motion Planning
- **Implementation**: Pure `rclpy` + `moveit_msgs` (no `moveit_py`, `pymoveit2`, or `moveit_commander`)
- **Planning Group**: `ur_manipulator`
- **Base Frame**: `base_link`
- **MoveIt Planning Tip**: `tool0`
- **Application Target**: `gripper_tcp` (offset converted in `moveit_interface.py`)

## HOME Configuration
- **Joint Values (rad)**:
  - `shoulder_pan_joint`: `0.0`
  - `shoulder_lift_joint`: `-1.5708`
  - `elbow_joint`: `1.5708`
  - `wrist_1_joint`: `-1.5708`
  - `wrist_2_joint`: `-1.5708`
  - `wrist_3_joint`: `0.0`
- **Scaling**: Velocity scaling `0.05`, acceleration scaling `0.05`

## Active Controllers
- `joint_state_broadcaster`
- `joint_trajectory_controller`
- `gripper_controller`

## 9Router LLM Interface
- **Endpoint**: `http://127.0.0.1:20128/v1`
- **Environment Variables**:
  - `NINEROUTER_BASE_URL`
  - `NINEROUTER_API_KEY` (never commit keys)
  - `NINEROUTER_MODEL`

## Vendor Repositories Policy
Upstream vendor packages must not be modified:
- `Universal_Robots_ROS2_Description`
- `Universal_Robots_ROS2_Driver`
- `Universal_Robots_ROS2_GZ_Simulation`
