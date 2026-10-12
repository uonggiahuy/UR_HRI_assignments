# M2 Engineering Report — Persistent TurtleBot 4 Simulation and SLAM Mapping

Date: 2026-10-12 (UTC). Baseline: `2b94daea67ffc99fda32dd8ac647f353f12c813c`, branch `assignments_4`.

---

## 1. Executive Summary

Milestone M2 implements a persistent base simulation architecture alongside independent SLAM mapping, safe keyboard teleoperation, robust atomic map saving, comprehensive live mapping diagnostics, and unified visualization.

Key achievements:
- **Persistent Base Simulation:** Exactly one Gazebo Fortress instance and one TurtleBot 4 model run continuously under `launch/turtlebot4.launch.py`. When switching modes or terminating SLAM, the simulator and RViz remain active without restarts.
- **Independent SLAM Toolbox:** `ros2 run turtlebot4 start_slam` launches synchronous SLAM Toolbox in the foreground with clean signal forwarding (SIGINT/Ctrl+C), verified `use_sim_time=true`, and strict mutual exclusion checks (rejecting duplicate SLAM instances and conflicting AMCL stacks).
- **Controlled Teleoperation:** `ros2 run turtlebot4 teleop` provides keyboard navigation with QoS matching `/cmd_vel` (`BEST_EFFORT`, `VOLATILE`, depth 5), bounded speed ramping, and guaranteed zero-velocity stopping upon exit.
- **Atomic Map Saver:** `ros2 run turtlebot4 save_map` captures and validates the live OccupancyGrid, invokes Nav2 `map_saver_cli` to an isolated temporary directory, verifies header and data integrity, and atomically writes `maps/map.yaml` and `maps/map.pgm`. Blank or unmapped occupancy grids are strictly rejected.
- **Mapping Diagnostics:** `ros2 run turtlebot4 check_mapping` continuously verifies node exclusivity, sensor health, `map -> odom -> base_link` TF connectivity, and occupancy cell statistics.
- **Testing & Stability:** All 37 M1 baseline assertions continue to pass; 15 new M2 automated assertions were added, totaling **52 passing automated tests** with 0 failures and 0 regressions.
- **Live Warehouse Exploration:** Robot explored the warehouse environment, expanded the mapped area from 32,771 to 80,090 free cells, achieved loop closure, safely stopped, and saved real map files (`491 x 447` cells, resolution `0.05 m`).

---

## 2. Final M2 Architecture

The system decouples the persistent physics/sensor backend from transient navigation stacks:

```text
+-----------------------------------------------------------------------------------------+
|                                    PERSISTENT BASE                                      |
|                 ros2 launch turtlebot4 turtlebot4.launch.py rviz:=true                  |
|                                                                                         |
|  [Ignition Gazebo Fortress]                                                             |
|    - Physics & World: warehouse.sdf                                                     |
|    - TurtleBot 4 Standard + Create 3 Base                                               |
|    - Clock Bridge: /clock (333 Hz)                                                      |
|    - LiDAR Bridge: /scan (62.5 Hz, sensor_msgs/msg/LaserScan)                           |
|    - DiffDrive Controller: /odom (62 Hz, nav_msgs/msg/Odometry)                         |
|    - Motion Control: /cmd_vel (best-effort subscriber)                                  |
|    - TF static & dynamic publishers: odom -> base_link -> links                         |
|  [RViz2]                                                                                |
|    - Fixed frame: odom (persists seamlessly before and during SLAM)                      |
|    - Displays: Grid, RobotModel, LaserScan, TF, Map (/map)                              |
+-----------------------------------------------------------------------------------------+
                                          |
                +-------------------------+-------------------------+
                |                                                   |
                v                                                   v
+---------------------------------------+         +---------------------------------------+
|          TRANSIENT MAPPING            |         |          TELEOPERATION                |
|     ros2 run turtlebot4 start_slam    |         |       ros2 run turtlebot4 teleop      |
|                                       |         |                                       |
|  - Pre-checks: simulator clock/LiDAR  |         |  - Undock check & trigger action      |
|  - Exclusivity: rejects AMCL & dupes  |         |  - Safe keyboard velocity mapping     |
|  - sync_slam_toolbox_node             |         |  - QoS: BEST_EFFORT, VOLATILE, d=5    |
|  - Broadcasts: map -> odom (50 Hz)    |         |  - Guaranteed zero-velocity on exit   |
|  - Publishes: /map (OccupancyGrid)    |         +---------------------------------------+
+---------------------------------------+
                |
                v
+---------------------------------------+
|          MAP VERIFICATION & SAVE      |
|     ros2 run turtlebot4 save_map      |
|     ros2 run turtlebot4 check_mapping |
|                                       |
|  - Validates OccupancyGrid data       |
|  - Saves via nav2_map_server cli      |
|  - Verifies YAML image & PGM header   |
|  - Atomic write to maps/map.{yaml,pgm}|
+---------------------------------------+
```

---

## 3. Changed and Implemented Files

| File | Status | Description |
|---|---|---|
| `launch/turtlebot4.launch.py` | Added | Persistent base launch entrypoint reusing `simulation_launch.py`. |
| `launch/README.md` | Modified | Documented persistent base and legacy simulator launch files. |
| `turtlebot4/slam_runner.py` | Added | Foreground SLAM lifecycle manager, simulator pre-check, and argument builder. |
| `turtlebot4/teleop.py` | Added | Safe keyboard teleoperation, velocity ramping, QoS compliance, undock trigger. |
| `turtlebot4/map_saver.py` | Added | Atomic map saving, OccupancyGrid validation, PGM header parsing, error rollback. |
| `turtlebot4/mapping_diagnostics.py` | Added | Comprehensive SLAM health, sensor readiness, TF chain, and statistics diagnostics. |
| `rviz/simulation.rviz` | Modified | Added `rviz_default_plugins/Map` display while retaining `odom` fixed frame. |
| `rviz/README.md` | Modified | Documented map visualization behavior and recovery across stack transitions. |
| `package.xml` | Modified | Registered runtime dependencies (`geometry_msgs`, `turtlebot4_navigation`, `slam_toolbox`, `nav2_map_server`). |
| `setup.py` | Modified | Registered console scripts (`start_slam`, `teleop`, `save_map`, `check_mapping`). |
| `maps/map.yaml` | Generated | Verified map metadata from real live exploration. |
| `maps/map.pgm` | Generated | 219,492-byte occupancy image from real live exploration. |
| `test/test_m2.py` | Added | 15 offline automated unit test assertions for M2. |
| `README.md` | Modified | Complete practical Vietnamese terminal-by-terminal user guide. |
| `docs/evidence/m2/` | Added | Real runtime measurement logs, JSON reports, and odometry evidence. |

---

## 4. Runtime Verification and ROS Interfaces

Measured on the running system during live execution:

| Topic | Observed Type | Observed Publisher | QoS Profile | Measured Rate |
|---|---|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | `/clock_bridge` | Reliable, Volatile, Depth 10 | 333.3 Hz |
| `/scan` | `sensor_msgs/msg/LaserScan` | `/lidar_bridge` | Reliable, Volatile, Depth 10 | 62.5 Hz |
| `/odom` | `nav_msgs/msg/Odometry` | `/diffdrive_controller` | Reliable, Volatile, Depth 1 | 62.0 Hz |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `/turtlebot4_teleop` | Best-effort, Volatile, Depth 5 | 10–20 Hz (on command) |
| `/map` | `nav_msgs/msg/OccupancyGrid` | `/slam_toolbox` | Reliable, Transient Local, Depth 1 | 2.0 Hz (update rate) |
| `/tf` | `tf2_msgs/msg/TFMessage` | `/robot_state_publisher`, `/slam_toolbox`, `/diffdrive_controller` | Reliable, Volatile, Depth 100 | > 100 Hz |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | `/robot_state_publisher`, `/rplidar_stf`, etc. | Reliable, Transient Local, Depth 1 | 5 retained msgs |

Action servers verified:
- `/undock`: `irobot_create_msgs/action/Undock` served by `/motion_control` (Accepted, Succeeded, `is_docked=false`).

---

## 5. Actual SLAM and TF Observations

During active SLAM Toolbox mapping, the TF tree was verified via `tf2_echo` and `check_mapping`:
- **Transform `map -> odom`:** Broadcaster: `slam_toolbox` (unique authority).
  - Translation: `[-0.000, -0.000, 0.000]`
  - Rotation: Quaternion `[0.000, 0.000, -0.000, 1.000]`
- **Transform `odom -> base_link`:** Broadcaster: `/diffdrive_controller`.
- **LiDAR frame chain:** `map -> odom -> base_link -> shell_link -> rplidar_link -> turtlebot4/rplidar_link/rplidar`.
- **Mutual Exclusivity:** Verified that AMCL was not running, preventing dual-authority transform conflicts.

---

## 6. Mapping and Exploration Results

The live warehouse exploration completed a closed loop across major aisles:

| Metric | Initial State (at spawn) | Final State (after exploration & loop closure) |
|---|---|---|
| Map Dimensions | 446 x 301 cells | **491 x 447 cells** |
| Physical Coverage | 22.30m x 15.05m | **24.55m x 22.35m** |
| Grid Resolution | 0.05 m/cell | 0.05 m/cell |
| Free Cells | 32,771 (24.4%) | **80,090 (36.5%)** |
| Occupied Cells | 212 (0.2%) | **1,938 (0.9%)** |
| Unknown Cells | 101,263 (75.4%) | **137,449 (62.6%)** |
| Robot Final Position | `(0.0, 0.0)` | `x=-2.23m, y=-1.51m, yaw=2.53 rad` |
| Final Velocity | `(0.0, 0.0)` | `(0.0, 0.0)` (stationary hold verified) |

---

## 7. Map Saving and Reload Verification

The map saving process was executed via `ros2 run turtlebot4 save_map`:
1. Live `/map` validated: 80,090 free cells observed, non-empty condition satisfied.
2. Nav2 `map_saver_cli` written to `/tmp/tb4_map_save_...` prefix.
3. Temporary `map.yaml` and `map.pgm` passed programmatic validation:
   - `resolution: 0.05`
   - `origin: [-13.4, -12.6, 0]`
   - `image: map.pgm` (relative reference)
   - PGM magic `P5`, dimensions `491 x 447`, size `219,492 bytes`.
4. Atomically moved to destination:
   - `/root/ros2_ws/src/turtlebot4/maps/map.yaml`
   - `/root/ros2_ws/src/turtlebot4/maps/map.pgm`
5. Verified independent reloadability via `validate_saved_map_files`.

---

## 8. Automated Test Counts and Results

Ran standard `colcon test`:
```bash
colcon build --symlink-install --packages-select turtlebot4
colcon test --packages-select turtlebot4
colcon test-result --verbose
```
**Results:** `Summary: 52 tests, 0 errors, 0 failures, 0 skipped`.

Breakdown:
- `test_m1.py`: 37 assertions (all PASS)
- `test_m2.py`: 15 assertions (all PASS)
  - `test_persistent_base_launch_syntax_and_loadability`: PASS
  - `test_slam_runner_timeout_validation`: PASS
  - `test_slam_runner_rejects_duplicate_slam`: PASS
  - `test_slam_runner_rejects_conflicting_amcl_localization`: PASS
  - `test_slam_runner_build_launch_cmd`: PASS
  - `test_teleop_twist_calculation`: PASS
  - `test_teleop_speed_adjustment_and_clamping`: PASS
  - `test_teleop_cmd_vel_qos_matches_robot_subscriber`: PASS
  - `test_occupancy_grid_validation_accepts_valid_grid`: PASS
  - `test_occupancy_grid_validation_rejects_empty_or_fabricated_grid`: PASS
  - `test_pgm_header_parsing`: PASS
  - `test_pgm_header_parsing_rejects_invalid_format`: PASS
  - `test_saved_map_files_validation`: PASS
  - `test_saved_map_files_validation_rejects_missing_image`: PASS
  - `test_rviz_simulation_config_has_all_required_displays`: PASS

---

## 9. Regression Test Results

Preserved baseline tests:
- Launch syntax, composition, and single world/spawn: PASS
- Argument validation and namespace normalization: PASS
- Measured/default joint-state merging: PASS
- Descendant PID tracking and safe process signals: PASS
- Zero regressions observed across all M1 deliverables.

---

## 10. Acceptance PASS/FAIL Matrix

| Acceptance Criterion | Verification Method | Status |
|---|---|---|
| Exactly one persistent Gazebo instance | Process inspection and launch lifecycle probe | **PASS** |
| Exactly one TurtleBot 4 spawned | `/robot_description`, joint and TF inspection | **PASS** |
| Persistent base launch entrypoint (`turtlebot4.launch.py`) | AST parse, launch test, live bringup | **PASS** |
| SLAM runs independently (`start_slam`) | Dedicated runner, subprocess ownership, live run | **PASS** |
| `use_sim_time=true` enforced in SLAM | Parameter query & launch argument verification | **PASS** |
| Clean foreground shutdown of SLAM via SIGINT/Ctrl+C | Live signal test, process table audit | **PASS** |
| Simulator remains running after SLAM shutdown | Live process table & `check_simulation` audit | **PASS** |
| Duplicate SLAM / AMCL collision rejected | Automated offline tests & node name queries | **PASS** |
| Keyboard teleoperation with safe stop (`teleop`) | Offline twist math tests, safe stop verification | **PASS** |
| Safe undock procedure documented and verified | Live `/undock` action goal: SUCCEEDED | **PASS** |
| Real warehouse exploration and loop closure | 45s trajectory executed, mapped cells increased | **PASS** |
| Atomic map saver (`save_map`) | Live capture, temporary directory validation | **PASS** |
| `map.yaml` and `map.pgm` created and consistent | YAML parser & PGM header verification | **PASS** |
| Rejection of blank/empty/fabricated maps | Automated unit tests with 100% unknown grids | **PASS** |
| Persistent RViz config supporting Map display | Resource verification with `Fixed Frame: odom` | **PASS** |
| Mapping diagnostics tool (`check_mapping`) | Live execution, JSON output, automated check | **PASS** |
| All 52 automated tests pass | `colcon test` & `pytest` execution | **PASS** |
| Vietnamese README tutorial complete | Fully authored, verified terminal commands | **PASS** |
| Git repository isolation on `assignments_4` | `git status`, branch check, clean worktree | **PASS** |

---

## 11. Evidence Artifacts

All runtime artifacts are stored in `docs/evidence/m2/`:
- `sim_check.json`: Initial simulator diagnostics output.
- `initial_mapping_diagnostics.json`: Initial SLAM mapping diagnostics JSON report.
- `exploration_run.log`: Timestamped log of the warehouse exploration sequence.
- `odom_after_undock.txt`: Odometry snapshot following the undock action.
- `odom_moving.txt`: Odometry snapshot during corridor exploration.
- `odom_final_stopped.txt`: Odometry snapshot confirming stationary robot state.
- `sim_check_after_slam_stop.json`: Simulator diagnostics proving Gazebo persists after SLAM exit.
- `live_topics.txt`: Listing of all active ROS topics during simulation.
- `map.yaml` & `map.pgm`: Verified map files generated from exploration.

---

## 12. Git Commit SHA and Push Status

- Implementation commit: `05bd606` (`05bd606114ebfa24e8a4d467812f842828b4d081`)
- Commit message: `feat(assignment4): implement persistent simulation and SLAM mapping`
- Branch: `assignments_4`
- Remote: `origin/assignments_4` (pushed successfully, verified via `git status`: `Your branch is up to date with 'origin/assignments_4'`)
- Worktree state: clean, zero unstaged files, zero untracked files.

---

## 13. Remaining Limitations

1. **Undock Heading Drift:** The Create 3 undock routine backs out and turns slightly based on internal dock odometry, resulting in an initial non-zero yaw relative to the dock frame. This is standard behavior for the iRobot Create 3 platform.
2. **LiDAR Message Drops on Heavy CPU Load:** Under intensive multi-core usage during initial simulator startup, the ROS Gazebo parameter bridge occasionally logs queue-full warnings for laser scans until the simulator time step stabilizes.

---

## 14. Recommendations for Milestone M3 (AMCL Localization)

1. **Map Loading:** Use `nav2_map_server` to load the verified `maps/map.yaml` created in M2.
2. **Initial Pose Estimate:** In the continuous demonstration, the robot's current pose is approximately `x=-2.23, y=-1.51, yaw=2.53`. Set the 2D Pose Estimate in RViz2 or via `/initialpose` topic accordingly.
3. **Exclusive Authority:** Ensure SLAM Toolbox remains stopped so AMCL is the sole active broadcaster of `map -> odom`.
4. **Lifecycle Management:** Verify that `map_server` and `amcl` transition to `ACTIVE` state via Nav2 lifecycle manager before issuing navigation goals.
