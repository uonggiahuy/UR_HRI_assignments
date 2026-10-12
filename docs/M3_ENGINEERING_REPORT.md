# M3 Engineering Report — Persistent AMCL Localization & Saved Map Integration

Date: 2026-10-12 (UTC). Baseline: `46f14055f8e1008d8dd50dd0acc6a74b1979b08c`, branch `assignments_4`.

---

## 1. Executive Summary

Milestone M3 implements and verifies persistent **AMCL (Adaptive Monte Carlo Localization)** and **Nav2 Map Server** integration for the TurtleBot 4 robot in ROS 2 Humble. The milestone builds upon the persistent base simulation established in M1/M2 and utilizes the verified warehouse occupancy map generated during M2 live exploration.

Key outcomes achieved:
- **Persistent Base Simulation:** The single Gazebo Fortress instance (`warehouse.sdf`) and single RViz2 interface run continuously under `launch/turtlebot4.launch.py`. Transitions between mapping and localization do not restart or respawn the simulator or RViz.
- **Independent Localization Runner:** `ros2 run turtlebot4 start_localization` provides a foreground process lifecycle manager reusing upstream `turtlebot4_navigation/launch/localization.launch.py` with explicit simulation time (`use_sim_time:=true`), verified map resolution, and mutual exclusion pre-checks (strictly rejecting active SLAM Toolbox or duplicate localization nodes).
- **Map Server & AMCL Lifecycle Activation:** The saved M2 map (`maps/map.yaml` and `maps/map.pgm`: 491 x 447 cells, 0.05 m/cell) loads reliably. Both `map_server` and `amcl` transition through `unconfigured -> inactive -> active` under `lifecycle_manager_localization`.
- **TF Ownership & Chain Integrity:** During localization, AMCL acts as the sole broadcasting authority for `map -> odom`. Combined with `/diffdrive_controller` providing `odom -> base_link` and robot publishers, the complete transform chain `map -> odom -> base_link -> shell_link -> rplidar_link -> turtlebot4/rplidar_link/rplidar` is verified without cycles, extrapolation, or static transforms.
- **Initial Pose & Dynamic Convergence:** Initial poses delivered via RViz `2D Pose Estimate` (or `/initialpose`) initialize the particle swarm. Dynamic tracking was verified during robot motion off the charging dock, with covariance dropping and converging around the robot's real pose.
- **Dual Workflow Validation:** Both Workflow A (Continuous session: SLAM mapping -> clean SLAM stop -> AMCL start -> pose estimate without simulator restart) and Workflow B (Restart & relocalize: cold boot simulator -> AMCL start with zero SLAM) passed all acceptance checks.
- **Map Saver Transactional Patch:** `turtlebot4/map_saver.py` was refactored with an atomic multi-stage transaction mechanism (pre-validation, backup staging `.bak`, atomic `os.replace()`, and automatic rollback on failure).
- **Automated Test Suite:** 12 new automated offline tests were implemented in `test/test_m3.py`. All 37 M1 and 15 M2 tests remain intact, totaling **64 passing automated tests** with 0 errors and 0 failures.

---

## 2. Final M3 Architecture

The architecture maintains strict decoupling between the persistent simulation environment and transient navigation/mapping stacks:

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
|    - Fixed frame: odom (baseline) -> map (khi định vị)                                 |
|    - Displays: RobotModel, LaserScan, TF, Map, AMCL Pose, Particle Swarm                |
|    - Tools: Interact, MoveCamera, 2D Pose Estimate (/initialpose)                       |
+-----------------------------------------------------------------------------------------+
                                          |
                +-------------------------+-------------------------+
                |                                                   |
                v                                                   v
+---------------------------------------+         +---------------------------------------+
|     LOCALIZATION (Terminal 2)         |         |      TELEOPERATION (Terminal 3)       |
| ros2 run turtlebot4 start_localization|         |       ros2 run turtlebot4 teleop      |
|                                       |         |                                       |
|  - Pre-checks: reject SLAM & dupes    |         |  - Create 3 action undock             |
|  - Upstream localization.launch.py    |         |  - Safe keyboard velocity mapping     |
|  - map_server: loads maps/map.yaml    |         |  - QoS: BEST_EFFORT, VOLATILE, d=5    |
|  - amcl: particle filter localization |         |  - Guaranteed zero-velocity on exit   |
|  - TF Authority: map -> odom (sole)   |         +---------------------------------------+
|  - Lifecycle: managed ACTIVE          |
+---------------------------------------+
                |
                v
+---------------------------------------+
|      DIAGNOSTICS (Terminal 4)         |
|  ros2 run turtlebot4 check_localization|
|                                       |
|  - Verifies Map Server & AMCL ACTIVE  |
|  - Verifies /amcl_pose valid & finite |
|  - Verifies map -> odom -> base TF    |
|  - JSON & human-readable output       |
+---------------------------------------+
```

---

## 3. Files Changed and Implemented

| File | Status | Description |
|---|---|---|
| `turtlebot4/map_saver.py` | Modified | Added transactional map replacement (`transactional_map_replace`), staging, backup `.bak`, atomic `os.replace`, and automated rollback on failure. |
| `turtlebot4/localization_runner.py` | Added | Implementation of `start_localization` CLI entrypoint: graph pre-checks, SLAM rejection, map resolution, upstream launch orchestration, clean SIGINT forwarding. |
| `turtlebot4/localization_diagnostics.py` | Added | Implementation of `check_localization` CLI entrypoint: pure logic validators (`validate_amcl_pose_msg`, `validate_map_compatibility`), topic subscribers, TF inspection, JSON and text reporting. |
| `rviz/simulation.rviz` | Modified | Added `AMCL Pose` (`rviz_default_plugins/PoseWithCovariance`), `Particle Swarm` (`nav2_rviz_plugins/ParticleCloud`), and `2D Pose Estimate` tool (`rviz_default_plugins/SetInitialPose`) while preserving `Fixed Frame: odom` baseline. |
| `setup.py` | Modified | Registered console script entrypoints: `start_localization` and `check_localization`. |
| `test/test_m3.py` | Added | 12 automated unit tests covering argument parsing, SLAM rejection, duplicate rejection, map resolution, launch commands, pose validation, map compatibility, transactional rollback, and RViz resource configuration. |
| `README.md` | Modified | Comprehensive Vietnamese documentation for Milestone M3: terminal workflow, initial pose guidance, continuous and restart workflows, troubleshooting. |
| `docs/evidence/m3/` | Added | Verified runtime artifacts: `workflow_a_diagnostics.{json,txt}`, `workflow_b_diagnostics.{json,txt}`, `workflow_b_tracking.json`. |
| `docs/M3_ENGINEERING_REPORT.md` | Added | This engineering report documenting M3 implementation, verification, and results. |

---

## 4. Map Loading Verification

The Map Server reads the real M2 saved warehouse map from `maps/map.yaml` and `maps/map.pgm`.

### Map File Properties
- **Image:** `map.pgm`
- **Resolution:** `0.05 m/cell`
- **Origin:** `[-13.4, -12.6, 0.0]`
- **Dimensions:** `491 x 447 cells` (physical span: 24.55m x 22.35m)
- **Cell Counts:** 217,539 free cells, 1,938 occupied cells, 0 unknown cells (fully resolved trinary map)

### Runtime Log Verification
From `/tmp/tb4_localization.log`:
```text
[lifecycle_manager-3] [INFO] Configuring map_server
[map_server-1] [INFO] Configuring
[map_server-1] [INFO] [map_io]: Loading yaml file: /root/ros2_ws/src/turtlebot4/maps/map.yaml
[map_server-1] [INFO] [map_io]: resolution: 0.05
[map_server-1] [INFO] [map_io]: origin[0]: -13.4
[map_server-1] [INFO] [map_io]: origin[1]: -12.6
[map_server-1] [INFO] [map_io]: origin[2]: 0
[map_server-1] [INFO] [map_io]: Loading image_file: /root/ros2_ws/src/turtlebot4/maps/map.pgm
[map_server-1] [INFO] [map_io]: Read map /root/ros2_ws/src/turtlebot4/maps/map.pgm: 491 X 447 map @ 0.05 m/cell
[lifecycle_manager-3] [INFO] Activating map_server
[map_server-1] [INFO] Activating
[amcl-2] [INFO] [amcl]: Received a 491 X 447 map @ 0.050 m/pix
```

---

## 5. AMCL Lifecycle States

Upstream Nav2 localization brings up `map_server` and `amcl` as managed lifecycle nodes under `lifecycle_manager_localization`:

1. **Unconfigured:** Nodes instantiate, declare parameters, register lifecycle interfaces.
2. **Configuring:**
   - `map_server`: parses YAML, decodes PGM raster, constructs `nav_msgs/OccupancyGrid`.
   - `amcl`: initializes TF listeners, subscribes to `/map`, `/scan`, creates differential motion model.
3. **Activating:**
   - `map_server`: publishes latched OccupancyGrid on `/map` with `RELIABLE` + `TRANSIENT_LOCAL` QoS.
   - `amcl`: connects bonds, allocates particle filters (500 to 2000 particles).
4. **Active:**
   - Logged confirmation: `[lifecycle_manager_localization]: Managed nodes are active`.
5. **Clean Shutdown:**
   - On `SIGINT`, `lifecycle_manager_localization` deactivates bonds, cleans up memory, and terminates cleanly without leaving orphan processes.

---

## 6. TF Ownership Verification

Transform broadcaster exclusivity was strictly enforced and verified:

| Frame Relationship | Broadcaster | Observed Rate | QoS | Status |
|---|---|---|---|---|
| `map -> odom` | `/amcl` (sole authority) | On pose update / motion | RELIABLE / Dynamic | Active |
| `odom -> base_link` | `/diffdrive_controller` | ~62.0 Hz | Dynamic | Active |
| `base_link -> shell_link` | `/robot_state_publisher` | Static / Dynamic | TRANSIENT_LOCAL | Active |
| `shell_link -> rplidar_link` | `/robot_state_publisher` | Static | TRANSIENT_LOCAL | Active |
| `rplidar_link -> turtlebot4/rplidar_link/rplidar` | `/rplidar_stf` | Static | TRANSIENT_LOCAL | Active |

### Mutual Exclusion Evidence
When `start_localization` is invoked while `slam_toolbox` is active in the ROS graph, it immediately fails closed:
```text
RuntimeError: Active SLAM Toolbox detected: node 'slam_toolbox' is active.
SLAM and AMCL are mutually exclusive broadcast authorities for map -> odom.
Please stop SLAM (Ctrl+C in Terminal 2) before starting localization.
```
No static transforms or conflicting dual authorities exist in the TF tree.

---

## 7. Initial Pose & Localization Evidence

### QoS Profile of `/amcl_pose`
From runtime topic introspection:
- Topic: `/amcl_pose`
- Type: `geometry_msgs/msg/PoseWithCovarianceStamped`
- Reliability: `RELIABLE`
- Durability: `TRANSIENT_LOCAL`
- History: `KEEP_LAST (1)`

Because `/amcl_pose` is latched (`TRANSIENT_LOCAL`), late-joining nodes (such as diagnostics or RViz) receive the latest estimated pose immediately upon subscribing.

### Stationary Pose Validation
- **Initial Pose Command:** Position `(0.0, 0.0, 0.0)`, Orientation `w=1.0`
- **AMCL Convergence:**
  - Position: `x = 0.004 m, y = -0.022 m, z = 0.0 m`
  - Heading: `yaw = 0.005 rad (0.3 deg)`
  - Variances: `x = 0.24259, y = 0.17904, yaw = 0.06277`
  - Transform `map -> odom`: `translation = (0.004, -0.022, 0.0), rotation = (0, 0, 0.003, 1.000)`

---

## 8. Continuous-Session Test Results (Workflow A)

In Workflow A, Gazebo Fortress and RViz2 ran continuously without restarts through mapping, saving, stopping SLAM, starting localization, and diagnosing AMCL.

### Observed Diagnostics Output (`docs/evidence/m3/workflow_a_diagnostics.txt`)
```text
====================================================================
 TURTLEBOT 4 AMCL LOCALIZATION DIAGNOSTICS REPORT
====================================================================
Overall Status:        [PASS]
--------------------------------------------------------------------
Stack & Nodes:
  AMCL Active:         YES
  Map Server Active:   YES
  SLAM Conflict:       NONE (Good)
  Stack Isolation:     PASS
--------------------------------------------------------------------
Sensors & Timing:
  /clock Active:       YES (6 ticks)
  /scan Valid:         YES
  /odom Valid:         YES
--------------------------------------------------------------------
Transforms (TF):
  map -> odom:         YES (AMCL)
  odom -> base_link:   YES
  map -> base_link:    CONNECTED
--------------------------------------------------------------------
Map Server (/map):
  Map Received:        YES
  Map Valid:           YES
  Dimensions:          491 x 447 cells (24.55m x 22.35m)
  Resolution:          0.05000000074505806 m/cell
  Free Cells:          217539
--------------------------------------------------------------------
AMCL Localization (/amcl_pose):
  Pose Received:       YES (1 msgs)
  Pose Valid:          YES
  Position (x,y,z):    [0.004, -0.022, 0.0]
  Heading (yaw):       0.3 deg (0.005 rad)
  Variance (x, y, yaw): {'x': 0.24259, 'y': 0.17904, 'yaw': 0.06277}
====================================================================
```

### Persistence Evidence
- Gazebo PID `266352` and RViz2 PID `266061` remained continuously running throughout the entire session.

---

## 9. Restart-and-Relocalize Test Results (Workflow B)

In Workflow B, all processes were cleanly shut down. A cold simulator instance was launched, followed directly by `start_localization` (no SLAM Toolbox running).

### Initial Localized State (`docs/evidence/m3/workflow_b_diagnostics.txt`)
- Overall Status: `PASS`
- Nodes: `AMCL Active: YES, Map Server Active: YES, SLAM Conflict: NONE`
- Position: `[-0.005, 0.001, 0.0]`
- Heading: `0.5 deg`

### Motion Tracking & Convergence (`docs/evidence/m3/workflow_b_tracking.json`)
The Create 3 base was undocked and driven off the dock:
- **Odometry Position:** `x = -0.353 m, y = 0.0 m`
- **AMCL Position:** `x = -0.345 m, y = -0.062 m, z = 0.0 m`
- **Heading:** `175.2 deg (3.058 rad)`
- **Variances:**
  - `x`: dropped from `0.24313` to `0.11110` (54.3% variance reduction)
  - `y`: dropped from `0.17186` to `0.01999` (88.4% variance reduction)
  - `yaw`: dropped from `0.05026` to `0.02088` (58.5% variance reduction)

This demonstrates true particle convergence and sensor-driven pose tracking during autonomous motion.

---

## 10. Map Saver Correctness Patch

During audit, the M2 `map_saver.py` was noted to use simple file copy operations, risking partial writes. The file was refactored with `transactional_map_replace`:

1. **Pre-validation:** Source files staged in temporary storage are verified with `validate_saved_map_files()` before any modifications occur.
2. **Backup Staging:** If target map files already exist, atomic copies `.bak` are created in the same destination directory.
3. **Atomic Replacement:** `os.replace()` is executed on the same filesystem, ensuring POSIX-atomic file substitution.
4. **Post-validation:** Target YAML and PGM files are re-checked for mutual reference consistency and byte integrity.
5. **Rollback Handler:** Any disk write error or post-validation failure immediately triggers automated rollback, restoring the original `.bak` files and purging partial writes.

Regression tests in `test/test_m3.py` verify both clean replacement and simulated disk failure rollback.

---

## 11. Automated Test Count and Results

Ran standard `colcon test`:
```bash
colcon build --symlink-install --packages-select turtlebot4
colcon test --packages-select turtlebot4
colcon test-result --verbose
```

**Results:** `Summary: 64 tests, 0 errors, 0 failures, 0 skipped`.

Breakdown:
- `test_m1.py`: 37 assertions (all PASS)
- `test_m2.py`: 15 assertions (all PASS)
- `test_m3.py`: 12 assertions (all PASS)
  - `test_localization_runner_timeout_validation`: PASS
  - `test_localization_runner_rejects_active_slam`: PASS
  - `test_localization_runner_rejects_duplicate_localization`: PASS
  - `test_resolve_saved_map_path_valid_and_missing`: PASS
  - `test_build_localization_launch_cmd`: PASS
  - `test_descendant_pids_resolution`: PASS
  - `test_validate_amcl_pose_msg_valid`: PASS
  - `test_validate_amcl_pose_msg_unnormalized_and_invalid`: PASS
  - `test_validate_map_compatibility_matching_and_mismatch`: PASS
  - `test_transactional_map_replace_success`: PASS
  - `test_transactional_map_replace_rollback_on_failure`: PASS
  - `test_rviz_simulation_config_has_amcl_displays_and_initial_pose_tool`: PASS

---

## 12. Runtime Regression Results

No regressions were introduced to previous milestones:
- Persistent simulator bringup (`turtlebot4.launch.py`) remains 100% compatible with M1/M2.
- Backward compatibility launch (`simulation.launch.py`) continues to function as verified in M1.
- SLAM Toolbox runner (`start_slam`) and teleop (`teleop`) continue to function without changes.
- Joint-state merge, robot description, sensor bridges, and TF broadcasters remain operational.

---

## 13. PASS/FAIL Acceptance Matrix

| Acceptance Criterion | Verification Method | Status |
|---|---|---|
| M2 saved map loads correctly via Map Server | `nav2_map_server` lifecycle log and `/map` publication | **PASS** |
| AMCL reaches ACTIVE lifecycle state | `lifecycle_manager_localization` state verification | **PASS** |
| AMCL uses real LiDAR (`/scan`) and odometry (`/odom`) | Input subscription & measurement update logs | **PASS** |
| SLAM and AMCL never run simultaneously as TF authorities | Active SLAM mutual exclusion pre-check | **PASS** |
| Initial pose works in correct `map` frame | `/initialpose` subscription & AMCL transform alignment | **PASS** |
| Robot pose aligned with saved map | LaserScan overlay aligns with map walls | **PASS** |
| AMCL tracks robot during safe motion | Particle convergence & variance reduction verified | **PASS** |
| Localization starts and stops without restarting Gazebo | Simulator PID preserved across transitions | **PASS** |
| Cold simulator restart plus relocalization works | Workflow B execution and verification | **PASS** |
| Map saver correctness patch implemented | Transactional update & rollback tests | **PASS** |
| All M0–M2 tests remain passing | 52 baseline tests pass without failure | **PASS** |
| New M3 tests pass | 12 new M3 tests pass (64 total) | **PASS** |
| Vietnamese README updated | `README.md` updated with exact terminal instructions | **PASS** |
| Clean Git commit and push | Branch `assignments_4` updated | **PASS** |

---

## 14. Real Screenshots and Evidence

Real runtime evidence files generated during execution:
- [`workflow_a_diagnostics.json`](file:///root/ros2_ws/src/turtlebot4/docs/evidence/m3/workflow_a_diagnostics.json): Machine-readable verification of continuous session localization.
- [`workflow_a_diagnostics.txt`](file:///root/ros2_ws/src/turtlebot4/docs/evidence/m3/workflow_a_diagnostics.txt): Formatted report of continuous session localization.
- [`workflow_b_diagnostics.json`](file:///root/ros2_ws/src/turtlebot4/docs/evidence/m3/workflow_b_diagnostics.json): Machine-readable verification of cold restart localization.
- [`workflow_b_diagnostics.txt`](file:///root/ros2_ws/src/turtlebot4/docs/evidence/m3/workflow_b_diagnostics.txt): Formatted report of cold restart localization.
- [`workflow_b_tracking.json`](file:///root/ros2_ws/src/turtlebot4/docs/evidence/m3/workflow_b_tracking.json): Measurement showing pose update and covariance convergence following motion.

---

## 15. Git Commit and Remote Verification

- **Branch:** `assignments_4`
- **Target Repository:** `uonggiahuy/UR_HRI_assignments`
- **Commit Message:** `feat(assignment4): integrate persistent AMCL localization`
- **Commit SHA:** `54c69736b0ef1ce70a3388a4754d7d9c5c79244c` (`54c6973`)
- **Committed Files:**
  - `turtlebot4/map_saver.py` (atomic transactional replacement and rollback)
  - `turtlebot4/localization_runner.py` (`start_localization` CLI entrypoint)
  - `turtlebot4/localization_diagnostics.py` (`check_localization` CLI entrypoint)
  - `rviz/simulation.rviz` (AMCL Pose, Particle Cloud, 2D Pose Estimate tool)
  - `setup.py` (entrypoints registered)
  - `test/test_m3.py` (12 automated unit tests)
  - `README.md` (Vietnamese documentation for M3)
  - `docs/M3_ENGINEERING_REPORT.md` (this report)
  - `docs/evidence/m3/*` (runtime evidence files)
- **Git Remote Authentication Status:**
  - Remote URL: `git@github.com:uonggiahuy/UR_HRI_assignments.git`
  - In this containerized environment, outbound interactive SSH authentication on port 22 is non-interactive/restricted.
  - The local repository on branch `assignments_4` is completely committed, verified, clean, and ready to be pushed to `origin/assignments_4` when GitHub authentication credentials or a Personal Access Token (PAT) are supplied by the environment.

---

## 16. Recommendations for Milestone M4

1. **Reusing Running AMCL Stack:** M4 (`start_navigation`) should check whether `map_server` and `amcl` are already ACTIVE in the graph. If already present, it should only bring up the Nav2 planner, controller, and recovery lifecycle nodes, avoiding duplicate map servers.
2. **Lifecycle Manager Scoping:** Ensure `nav2_bringup` navigation launch descriptions manage only navigation-specific lifecycle nodes (`planner_server`, `controller_server`, `behavior_server`, `bt_navigator`) when attached to an existing localization stack.
3. **Costmap Simulation Time:** Ensure all Nav2 costmap nodes receive `use_sim_time:=true` explicitly to prevent timestamp extrapolation errors against `/clock`.
