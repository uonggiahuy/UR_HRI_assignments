# M1 Engineering Report — Simulation integration and ROS interface verification

Date: 2026-10-12 (Asia/Bangkok). Baseline:
`adc046a0910f24811fbde1e49a4e2766a9cd9eca`, branch `assignments_4`.

## 1. Executive summary

The assignment-owned simulator wrapper builds, launches the installed simulator,
spawns one robot, publishes valid clock/LiDAR/odometry interfaces, and supports
RViz visualization. Controlled forward teleoperation and stopping were verified
in the visually inspected warehouse. **37 automated tests pass** using standard
`colcon test`. The Docker host audit remains **PARTIAL** as a separate M0 item.
No SLAM, AMCL, Nav2, mapping or M2–M6 functionality was enabled. No upstream
package or simulator dependency was modified or reinstalled.

The exact default command was tested live with RViz disabled, and the same
wrapper was tested with RViz enabled. Depot/Lite/namespaced bringup was also
verified live. All twelve M1 acceptance criteria are PASS. Final commit/push verification is
recorded in section 11; the separate M0 Docker audit remains PARTIAL.

## 2. Architecture and upstream integration

`launch/simulation.launch.py` exposes four verified interfaces: `world`
(default warehouse; installed choices depot/maze/warehouse), `model`
(standard/lite, default standard), `namespace` (empty by default), `rviz`
(false by default). Namespace validation rejects invalid ROS name components.
Worlds are discovered from the installed upstream share, not hardcoded paths.
Maze existence/forwarding is checked offline; Maze was not launched live.

The implementation resolves package shares with `ament_index_python`, includes
`ignition.launch.py` and `turtlebot4_spawn.launch.py` exactly once in separate
launch scopes, and explicitly passes world/model/time. Spawn stack toggles are
fixed false, as is its built-in RViz toggle. The wrapper starts upstream `rviz2`
with the assignment's odom-based config instead of the upstream map-based config.
Simulation time is true via a scoped ROS launch parameter and explicit child
arguments. No map argument or stack toggle is exposed.

The installed description launch creates a default joint-state publisher as
well as robot-state publisher. Publishing defaults alongside Gazebo's measured
wheel states caused post-motion wheel TF disagreements. Project config makes
that upstream publisher consume measured joints through an input alias, merge
passive wheel-drop defaults, and publish `joint_states_visualization`. Only
robot-state publisher consumes the merged topic. Sensors retain measured
`joint_states`. Node-qualified remaps prevent accidental remapping of other
publishers or the publisher subscribing to its own output. The final runtime
observed one measured publisher and one merged publisher, with measured values
preserved and passive joint defaults present.

A nonzero child process exit logs an error and requests launch shutdown. Normal
spawn/controller-loader exits do not trigger failure. An owned-process tracker
forwards SIGINT to Gazebo descendants during shutdown: the installed upstream
shell launcher otherwise left server/GUI processes alive during failed attempts.
It checks process ancestry and start identity, and never signals unrelated trees.

## 3. Implemented and modified files

Added:

- `launch/simulation.launch.py`: ROS launch entry point.
- `turtlebot4/simulation_launch.py`: package discovery, scoped composition,
  argument validation, node-specific remaps and process lifecycle handling.
- `config/simulation.yaml`: measured/default joint-state merge parameters.
- `rviz/simulation.rviz`: robot/LiDAR/TF visualization in `odom`.
- `turtlebot4/diagnostics.py`: bounded read-only graph, data, QoS and TF checks.
- `scripts/check_simulation.py`: source-tree diagnostic wrapper.
- `test/test_m1.py`: 37 offline assertions including parametrized cases.
- `pytest.ini`: excludes only incompatible third-party AnyIO plugin.
- This report and `docs/evidence/m1/`: actual observations and app-only screenshots.

Modified `package.xml` (used launch, ROS messages, diagnostics and RViz dependencies),
`setup.py` (pytest registration and installed diagnostic console script), `README.md`,
`launch/README.md`, and `rviz/README.md`. No M0 files were removed and no
unrelated checkout was edited. Launch/config/RViz assets install using existing
recursive share-resource rules. The diagnostic resolves as
`ros2 run turtlebot4 check_simulation`; prefix is
`/root/ros2_ws/install/turtlebot4`.

## 4. Actual ROS interfaces

| Topic | Observed type | Verified endpoint / QoS |
| --- | --- | --- |
| `/clock` | `rosgraph_msgs/msg/Clock` | `/clock_bridge`; reliable, volatile, keep-last 10 |
| `/scan` | `sensor_msgs/msg/LaserScan` | `/lidar_bridge`; reliable, volatile, keep-last 10 |
| `/odom` | `nav_msgs/msg/Odometry` | `/diffdrive_controller`; reliable, volatile, keep-last 1 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | robot-side `/motion_control` subscriber; best-effort, volatile, keep-last 5 |
| `/tf` | `tf2_msgs/msg/TFMessage` | robot/dock-state publishers, odometry bridge and controller endpoints |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | reliable transient-local endpoints; five retained messages received |
| `/joint_states` | `sensor_msgs/msg/JointState` | measured `/joint_state_broadcaster` output |
| `/joint_states_visualization` | `sensor_msgs/msg/JointState` | merged `/joint_state_publisher` output to robot-state publisher |
| `/dock_status` | `irobot_create_msgs/msg/DockStatus` | `/sensors_node`; best-effort, volatile, keep-last 5 |

The command type and robot-side subscriber were inspected before any motion.
The `/undock` action is `irobot_create_msgs/action/Undock`, served by
`/motion_control`. Both joint-state broadcaster and differential-drive controller
were observed active. No Nav2 action server or map publisher was present.
Earlier upstream RViz config created navigation subscriptions/action clients;
the project visualization config does not start navigation stacks.

Full topic/node snapshots, endpoint evidence and machine-readable observations
are in `docs/evidence/m1/`. Namespaced counterparts were observed under
`/m1_robot`; clock remains global.

## 5. TF findings

Default observed required chain:

```text
odom -> base_link -> shell_link -> rplidar_link -> turtlebot4/rplidar_link/rplidar
```

Equivalently the LiDAR chain is
`base_link -> shell_link -> rplidar_link -> turtlebot4/rplidar_link/rplidar`.
Odometry identifies `odom` and child `base_link`. LiDAR frame is
`turtlebot4/rplidar_link/rplidar`. No map frame is required or fabricated.

The final runtime found a connected base/LiDAR chain, no invalid transforms,
no conflicting parent assignments, no static/dynamic overlap, no observed
conflicting transform samples, and no cycles or duplicate node names.
Quaternion sign equivalence (`q` and `-q`) is handled correctly.

Installed Humble rclpy subscription callbacks omit publisher GIDs. Therefore
identical transforms sent by different publishers cannot be individually
attributed by this script. It reports all publisher endpoints and detects
observable duplicates/conflicts; it does not claim complete per-edge authority
verification. This limitation is printed in diagnostic JSON.

## 6. Sensors, odometry and simulation time

Final exact-default-command observation window: 10 wall-clock seconds.
These measured rates describe received messages, not nominal configured rates:

| Signal | Received | Wall Hz | Stamp-derived sim Hz | Observed stamp delta (s) | Invalid |
| --- | ---: | ---: | ---: | ---: | ---: |
| `clock` | 3228 | 329.97 | 333.33 | 9.681 | 0 |
| `scan` | 605 | 61.78 | 62.51 | 9.663 | 0 |
| `odom` | 599 | 61.29 | 62.00 | 9.645 | 0 |

Scan samples contained 640 ranges, including
440 finite in-range values in the latest scan.
Odometry pose/twist values were finite with a unit orientation quaternion.
Clock, scan and odometry timestamps progressed without regression. Latest header
ages against clock were {'scan': 0.015000000000000568, 'odom': 0.009000000000000341} seconds,
within the declared -0.2 to 2.0 second allowance.

A parameter-service audit queried 47 nodes in a warehouse run: all responding
externally launched nodes and controllers reported `use_sim_time=true`.
The embedded Gazebo `gz_ros2_control` logger node reported false, and its private
TF listener had no parameter response. These embedded nodes are created by
upstream plugins, outside ROS launch parameter injection. Controller/sensor
stamps and simulation-time alignment were independently verified; their source
was not modified. The complete parameter evidence preserves these exceptions.

The diagnostic is read-only, uses one total 1–120 second wall timeout, reports
real frequencies/QoS/frames, and exits nonzero on failed machine checks. It
avoids binding internal IR/cliff scans during partial DDS discovery, delays
fallback selection, and rebinds to a later canonical topic. An absent simulator
was verified to return FAIL rather than fabricated sensor values.

Static TF must use reliable transient-local subscription QoS in this CycloneDDS
environment; best-effort transient-local did not replay the retained samples.
That correction is covered by an automated regression assertion.

## 7. Controlled teleoperation and GUI verification

The warehouse GUI and current sensor clearance were inspected before motion.
Initial robot state was docked. The supported undock action was accepted and
returned status 4 (SUCCEEDED), `is_docked=false`. It backs away and changes robot
heading; clearance was checked again before the command test.

An initial reverse test was aborted by BACKUP_LIMIT and zero velocity was sent;
the stop was observed. The safety mechanism was retained. Forward motion was
permitted only while physical-hazard flags were absent, clock/scan/odom/hazard
observations were fresh, and directional clearance remained above the bound.

Final controlled command: **+0.05 m/s for 2 wall-clock seconds**, at a finite
publication rate, followed by zero commands, a one-second stop observation,
and a one-second stationary hold. Measured projected displacement:
**0.097499804 m**. Stop velocity was
`linear.x=-9.8395e-17` m/s and
`angular.z=2.67971e-17` rad/s. Stationary hold drift:
**0.000000 m**. The test passed its displacement, velocity and
drift assertions. The first forward run before the final merge refinement
measured 0.0999 m; final evidence above refers to the complete merged config.

`controlled_motion.json` contains actual before/after poses/twists/stamps;
`reverse_safety_guard.json` records the abort. The one-off bounded harness source
is archived as text for review. It is not an installed automatic motion command.
Any reproduction requires new visual scene/interface verification.

Graphical access was available through `DISPLAY=:1` and X11. Application-window
screenshots were captured and inspected: Gazebo shows the warehouse robot;
RViz shows robot, red LiDAR returns and TF with **Global Status: Ok**, without
RobotModel/TF errors in the final merged config. See `rviz.png` and `gazebo.png`.
The reusable graph script intentionally reports visualization as MANUAL
VERIFICATION REQUIRED because node presence alone cannot prove display content;
this run's report records the separately performed visual verification as PASS.

## 8. Automated build/test results and failed iterations

Executed inside the existing ROS container:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select turtlebot4
source install/setup.bash
colcon test --packages-select turtlebot4
colcon test-result --verbose
ros2 pkg prefix turtlebot4
ros2 pkg executables turtlebot4
python3 src/turtlebot4/scripts/verify_m0.py
```

Build: PASS, one selected package. Final tests: **37 tests, 0 errors, 0 failures,
0 skipped**. The console-direct event handler was also used to inspect executed
assertions. The M0 packaging/upstream check passed with 194 unchanged audited
upstream share files and four unchanged upstream package prefixes.

Offline tests cover syntax/loadability, launch declarations/defaults, installed
world/package resolution, single world/spawn composition, explicit disabled
stack arguments, namespace and timeout rejection, launch failure handling,
owned shutdown ancestry, installed RViz/config assets, scan/odom validity,
ambiguous/internal/late topic discovery, timestamp regression, absent graphs,
static TF QoS, transform conflicts/cycles and quaternion equivalence.
They do not launch a simulator or publish motion.

Initial plain testing failed before collecting tests because the installed
third-party AnyIO plugin requires a newer pytest internal module. After isolating
that plugin, temporary test code also needed to load launch sources before
inspecting their expanded paths. Final `pytest.ini` excludes only AnyIO, leaving
ROS test plugins active; plain `colcon test` now passes without a special shell
environment or any dependency installation.

Runtime failures investigated and fixed:

- Humble lacks `launch.actions.LogError`: replaced with the launch logger API.
- Upstream RViz's delayed `description` lookup failed after scope exit, and its
  config requires `map`. The final wrapper uses upstream rviz2 with its own config.
- Best-effort static TF replay failed: reliable transient-local QoS fixed it.
- Default joint-state publication conflicted with moved wheel positions:
  measured/passive-joint merging fixed both conflicts and missing wheel-drop TF.
- DDS discovery exposed an internal IR scan first: canonical priority, internal
  exclusion, delayed fallback and late rebinding fixed this race.
- Depot initially did not have a ready spawn/controller graph while its world
  loaded. A later bounded run after actual readiness passed; no arbitrary sleep
  was used as a substitute for interface verification.

Live commands were run separately from offline tests:

```bash
ros2 launch turtlebot4 simulation.launch.py
ros2 launch turtlebot4 simulation.launch.py rviz:=true
ros2 launch turtlebot4 simulation.launch.py world:=depot model:=lite namespace:=m1_robot rviz:=true
ros2 run turtlebot4 check_simulation --timeout 10 --output /tmp/assignment4_m1_default_acceptance.json
ros2 run turtlebot4 check_simulation --timeout 10 --namespace m1_robot --expect-rviz --output /tmp/assignment4_m1_namespace_final.json
ros2 control list_controllers
ros2 topic info /cmd_vel --verbose
ros2 action info /undock
ros2 interface show irobot_create_msgs/action/Undock
ros2 launch turtlebot4 simulation.launch.py --show-args
python3 /tmp/assignment4_m1_teleop.py
```

The `/tmp` motion script is the archived one-off harness, executed only after the
scene checks above. Additional read-only graph snapshots and Gazebo model
inventory were obtained with:

```bash
ign service -s /world/warehouse/scene/info --reqtype ignition.msgs.Empty --reptype ignition.msgs.Scene --timeout 3000 --req ''
ign service -s /world/depot/scene/info --reqtype ignition.msgs.Empty --reptype ignition.msgs.Scene --timeout 3000 --req ''
```

Warehouse inventory contains exactly one `turtlebot4` and one `standard_dock`.
Depot inventory contains one namespaced robot and one namespaced dock, alongside
its environment model. No simultaneous simulator instances were used for final
verification. Final controlled robot state was stopped before launch shutdown. After the
last namespaced run, `ros2 node list` returned empty and no owned Gazebo, RViz
or TurtleBot processes remained; `shutdown.json` records the read-back check.

## 9. Docker audit follow-up

**PARTIAL**, unchanged from M0. This shell runs Ubuntu 22.04 with Humble and
`/.dockerenv`. The workspace mount is host-backed ext4 at `/root/ros2_ws`.
No Docker executable, Docker socket or authorized host shell is available here.
Container identity/image, Docker network mode, configured mount metadata and
restart persistence cannot be independently verified. No elevated socket access,
workaround, container restart/recreation or 9Router change was attempted.
This did not block M1 implementation or runtime validation.

## 10. Acceptance matrix

| # | Criterion | Result | Evidence |
| --- | --- | --- | --- |
| 1 | Wrapper launches installed simulator | PASS | exact default and RViz-enabled commands ran |
| 2 | Exactly one TurtleBot 4 | PASS | Gazebo scene inventories plus single world/spawn composition |
| 3 | Progressing clock | PASS | final bounded default runtime |
| 4 | Valid LiDAR | PASS | nonempty finite in-range returns, actual frames/QoS |
| 5 | Valid odometry | PASS | finite poses/twists, advancing sim stamps |
| 6 | Robot-side command interface | PASS | Twist subscriber `/motion_control` inspected |
| 7 | Required TF chain | PASS | connected odom/base/LiDAR chain; observed conflicts resolved |
| 8 | Safe verified teleoperation | PASS | bounded command, measured displacement and stable stop |
| 9 | RViz displays robot and sensor data | PASS | final application screenshot inspected |
| 10 | Build and real automated tests | PASS | build and 37 executed assertions |
| 11 | Upstream unmodified | PASS | 194 baseline hashes, clean UR vendor checkouts |
| 12 | Commit and push | PASS | `90b5088` pushed and remote head read back |

The script's graphical check remains MANUAL VERIFICATION REQUIRED by design;
this report's criterion 9 is independently verified. Host Docker remains the
separate PARTIAL M0 item, not an M1 failure.

## 11. Git commit and push verification

Only `/root/ros2_ws/src/turtlebot4` on `assignments_4` is staged. Baseline checkout
was clean at `adc046a0910f24811fbde1e49a4e2766a9cd9eca`; existing outer workspace
changes and other assignment/vendor checkouts are preserved. No build/install/log
artifacts, secrets or Docker credentials are included. Push uses existing origin
SSH push URL, without force. Implementation commit:
`90b5088941b2d257c9e2231ae801d5e48542ef79`
(`feat(assignment4): integrate TurtleBot4 simulation and diagnostics`).
`git push origin assignments_4` succeeded, advancing the remote from `adc046a`
to `90b5088`; `git ls-remote --heads origin` independently confirmed that SHA.
The checkout was clean after the implementation commit. This report completion
is committed separately; its final containing SHA is available with
`git rev-parse HEAD` and is provided in the final chat report.

Protected branch heads remained unchanged after the implementation push:

- `assignment_1`: `5a1cf1d624e660121d4752d2617f82f61f290e17`
- `assignments_2`: `739f4483016947fb0efd8de23691e610d4318984`
- `assignments_3`: `bee3f40a877b3bc8abb193e12505020f7cf5eea5`

Final shutdown was verified with an empty ROS node list and no owned simulator,
RViz or TurtleBot processes. Existing vendor repositories remain clean on
`humble`; existing student checkouts retain their original branches. Outer
workspace generated logs and its pre-existing modified perception gitlink are
preserved and excluded from these commits.

## 12. Known limitations and M2 recommendations

- Complete host Docker identity/mount/network inspection from an authorized host
  terminal using read-only `docker ps` and `docker inspect`; keep M0 PARTIAL until then.
- Maze is installed but not live-tested. Initial world resource loading can be slow.
- The installed embedded Gazebo logger node's clock parameter and private listener
  cannot be changed through wrapper parameter injection; data stamps remain verified.
- Identical cross-publisher TF copies cannot be attributed per edge by this Humble
  callback API. Keep endpoint and conflict observations visible during later work.
- Upstream control logs report a 0.001-second desired update period versus a
  0.003-second simulation step. Controllers remained active; no upstream tuning
  was changed. TurtleBot camera/motor service calls also log unavailable services
  in simulation while actual bridged scan/odometry continue; this is not hidden.
- Upstream TurtleBot node can need ROS launch SIGTERM escalation during shutdown;
  Gazebo child shutdown is handled specifically by this wrapper and verified.
- Before M2, use these diagnostics to verify sensor/TF readiness, then explicitly
  compose the planned mapping stack with simulation time and separate scopes.
  Do not start duplicate map-to-odom publishers or stack instances.

STOP after M1. These recommendations do not implement mapping or navigation.
