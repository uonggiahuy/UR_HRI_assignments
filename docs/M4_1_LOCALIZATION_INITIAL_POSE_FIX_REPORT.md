# M4.1 — AMCL initial pose and RViz map frame

Date: 2026-10-13 (Asia/Bangkok). Branch: `assignments_4`.
Baseline: `e3f1bd14b42d3e35b264a000df99821f3057b76f`.

**PASS for the bounded localization fix/replay:** current 603 × 1002 map,
real lifecycle ACTIVE, service and topic initialization, fresh pose/TF,
check_localization PASS, plausible ground-truth pose, observed RViz map workflow,
150 automated tests. The user's exact earlier GUI lock and full manual mapping
session were not independently reproduced. No M5/M6 or new navigation goals.

## Read-only investigation and root cause

The shell is the existing Ubuntu 22.04.5 container environment, with ROS Humble,
CycloneDDS and DISPLAY=:1. Docker host management is unavailable here; no host
ROS installation, container or 9Router change was needed. Initial process and
fresh graph inspection found no simulator, SLAM, AMCL or RViz running. All runtime
sessions described below were therefore created and owned by this investigation.
The assignment checkout was on `assignments_4`, initially modified only in
`maps/map.yaml` and `maps/map.pgm`; those user changes are preserved and excluded
from both commits. Outer workspace/build artifacts are not staged.

Inspected AGENTS.md, TurtleBot platform/localization/simulation/SLAM/debugging/demo
skills, ROS general/debugging skills, M0/M1/upstream and M3/M3.5/M4/recovery reports,
README, assignment runner/diagnostics/RViz/launch code, installed TurtleBot
localization launch/YAML and Nav2 localization include.

1. **Confirmed wrong-frame initialization.** Actual RViz `2D Pose Estimate`
   published `/initialpose` with `frame_id=odom`, simulation timestamp 75.525 s,
   complete covariance and position approximately (-0.052,-0.047). AMCL received
   it but logged `Ignoring initial pose in frame "odom"; initial poses must be in
   the global frame, "map"`. `/amcl_pose` and map TF remained absent. RViz uses its
   Fixed Frame for the initial pose tool; keeping the simulator config's `odom`
   default while trying localization does not initialize AMCL.
2. **Confirmed retained-map visualization failure.** Map Server published its
   retained map while RViz used `odom` and no map transform existed. RViz's
   MessageFilter later discarded the map at 7.554 s as earlier than its transform
   cache. Switching the editable field to `map` did not replay the discarded
   message. Unticking/ticking the existing Map display re-received it and produced
   `Map OK` and `Transform Ok`, with visible scan and map. QoS was already correct.
3. **Fixed Frame editing itself is not locked in this reproduction.** Double
   clicking the value cell, typing `map` and pressing Enter succeeds even before
   AMCL publishes TF. Until initialization, RobotModel/LaserScan transform errors
   are expected. Camera previously targets `base_link`; the assignment config now
   targets `<Fixed Frame>` so a map can be viewed/selected without map-to-base TF.
   A fresh updated-config run visibly rendered the map before AMCL initialization.
   A user-specific dropdown/window-manager problem remains unverified.
4. **Verifier timing edge independently reproduced and corrected.** A GUI map
   request at 219.846 s produces a *new* AMCL publication stamped 219.843 s, the
   scan acquisition time. A strict request-time comparison initially timed out
   despite AMCL logging `Setting pose`. The final verifier uses a VOLATILE pose
   subscription (no retained old output), requires a new publication, allows only
   0.25 s acquisition/transport skew and checks pose age, new TF broadcasts and
   progressing sensors. The failed first GUI-verifier attempt is retained in
   `gui_initialization.txt`; the successful retry is separately recorded.

Actual `/amcl/get_state` and `/map_server/get_state` returned id=3, label=active.
AMCL subscribes to scan/map/initialpose and serves `/set_initial_pose` with
`nav2_msgs/srv/SetInitialPose`. RViz/AMCL initialpose endpoints are both RELIABLE,
VOLATILE, geometry_msgs/PoseWithCovarianceStamped. AMCL pose is RELIABLE,
TRANSIENT_LOCAL; late diagnostic subscriptions are compatible. RViz, AMCL and
Map Server use simulation time; clock and sensor stamps advance. No SLAM conflict
or alternate upstream RViz config was found. `/proc` maps verify that AMCL and
RViz load the existing unmodified cache TF2 0.25.24 library. Some requests logged
a 9 ms odometry extrapolation warning, followed by `Setting pose` and real output:
that warning did not reject initialization and is not the wrong-frame cause.

Matching primary source:
[Nav2 AMCL 1.1.20](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_amcl/src/amcl_node.cpp).
Real logs/endpoints and final native mappings are under [evidence/m4_1](evidence/m4_1).

## Minimal correction and changed files

- `turtlebot4/initial_pose.py`, `setup.py`: new `set_initial_pose` CLI. Requires
  explicit finite map x/y/yaw-deg, uses current ROS sim time, normalized quaternion,
  36 covariance entries (x/y variance 0.25 m²; yaw variance (15°)²). Uses actual
  ACTIVE lifecycle services, verifies matching map contents and AMCL frames/time,
  rejects concurrent SLAM/duplicate AMCL, prefers discovered SetInitialPose.
  Auto topic fallback occurs only when that service is absent and the correct
  AMCL subscriber is discovered. Explicit topic mode is available and tested.
  Service timeout never silently falls back. One overall 1..120 s deadline bounds
  requests and output verification; pending futures/clients and node are cleaned.
  An empty service response or publish alone cannot print PASS.
- `turtlebot4/localization_diagnostics.py`: GetState-based lifecycle status,
  clock progress, fresh dynamic TF, correct map pose frame, explicit CLI failure
  guidance and namespace-aware node/TF listener context. Retained stationary
  AMCL pose is allowed only alongside current sensor/TF evidence.
- `rviz/simulation.rviz`: change only camera Target Frame to `<Fixed Frame>`;
  preserve `Fixed Frame: odom`, all M1 displays and existing map/AMCL/particle
  displays/tools. Runtime frame selection and retained-map refresh are documented.
- `test/test_m3_5.py`: retain all 19 tests; derive independent expected class
  counts from canonical raw PGM bytes instead of old M2 constants. Preserve real
  faulty-threshold rejection, ordered layout checks, saver/reload and byte-level
  rollback assertions. Convert YAML integer origin coordinates to ROS floats.
- `test/test_m4_1.py`: 34 additional cases covering CLI, yaw/quaternion,
  covariance, frame/time, service availability, verified fallback, deadlines,
  pending cleanup, lifecycle, fresh output/TF, scan-time skew, SLAM exclusion,
  RViz compatibility and a real bounded rclpy service-unavailable transport test.
- Vietnamese `README.md`, this report and curated runtime evidence: terminal
  initialization, final SLAM pose reading, separate restart steps and RViz recovery.

No upstream/installed source, TF2 overlay, navigation safety parameters, user
map, UR code, skill, secret or workspace build/install/log directory is committed.

## Map preservation

Both initial and final file hashes are identical:

- PGM: `915c88f67974aa4e160f2679cc17c538efceddc1e2ed2f9eb0cf05000b3a4955`
- YAML: `b1be9283ae2c966e54ce6000133263b62fcf7daa2a84cce61c94b210622bf8c9`

Actual Map Server/diagnostics: 603 × 1002, 0.05 m/cell, origin [-15,-25.1,0],
505,728 Free / 15,141 Occupied / 83,337 Unknown, thresholds .196/.65, trinary.
It matches the saved image's ordered occupancy signature. No map saver was run
against the user's source map. Regression saver/reload tests write temporary
files only. Handoff tests **reference** the existing map without regenerating it.

## Commands and real runtime verification

Every test terminal sources:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source "${XDG_CACHE_HOME:-$HOME/.cache}/turtlebot4/tf2-0.25.24/install/setup.bash"
```

Owned base and AMCL sessions used bounded foreground commands:

```bash
timeout --signal=INT --kill-after=20s 1800s ros2 launch turtlebot4 turtlebot4.launch.py rviz:=true
timeout --signal=INT --kill-after=15s 1500s ros2 run turtlebot4 start_localization --timeout 30
ros2 node info /amcl
ros2 topic info /initialpose --verbose
ros2 service call /amcl/get_state lifecycle_msgs/srv/GetState '{}'
ros2 service call /map_server/get_state lifecycle_msgs/srv/GetState '{}'
```

Before initialization: active nodes and valid new map/sensors, repeated AMCL
initial-pose warnings, no pose/TF. A real GUI click in `odom` reproduces rejection.
After **independent ground-truth confirmation** of this newly created warehouse
spawn, the following test-only coordinates were appropriate:

```bash
ros2 run turtlebot4 set_initial_pose --x 0 --y 0 --yaw-deg 0 --timeout 20
ros2 run turtlebot4 set_initial_pose --x 0 --y 0 --yaw-deg 0 --interface topic --timeout 20
ros2 run turtlebot4 check_localization --timeout 15 --json
timeout 5 ros2 topic echo /amcl_pose --once --qos-durability transient_local
timeout 5 ros2 run tf2_ros tf2_echo map base_link
```

These zero coordinates are **not** a general handoff default. The README requires
user-supplied current map coordinates. Both service/topic paths independently
produce fresh AMCL publications and scan-driven TF. The installed service returns
no success field, so the CLI verifies effects instead. Observed first service
pose [0.011,-0.076], final restart stationary ground-truth error 0.0383 m.

Bounded motion validation (`observe_localization.py --motion --output ...`):
actual Undock SUCCEEDED (4), robot-side cmd_vel endpoint and LiDAR clearance
checked, path cells verified Free, then 0.08 m/s for four seconds. No navigation
stack or goals; final repeated zero command and stopped odometry verified.
Maximum sampled motion localization error 0.1982 m, yaw error 0.0525 rad;
after-stop error 0.2021 m. Recorded hazard type 0 (BACKUP_LIMIT) was not bypassed;
no prohibited hazard was recorded. These are this trial's measurements, not a
universal accuracy guarantee.

Continuous-session handoff: the same Gazebo/RViz processes remained alive while
AMCL was stopped, a short SLAM session ran, a real stationary map-to-base pose
was read, SLAM stopped and AMCL restarted with the **unchanged** saved map.
Captured pose (-0.664745,-0.061588,-168.832819°) at sim time 518.937 s; explicit
CLI input from that read produced [-0.672,-0.138] and -170.8°. Diagnostics PASS;
stationary ground-truth error 0.1403 m / 0.1671 rad. A live attempt to initialize
while SLAM was present correctly exited 1 before sending a pose.

Evidence JSON retains uniformly spaced samples and the maxima/count from the
full observations; `evidence_stride` documents the reduction. The reusable
observer limits stored samples while checking every callback.

No automatic pose snapshot replay is implemented. The saved occupancy pair does
not encode the final pose or simulator identity. Reliable automatic replay would
need an owned persistent session identity and continuous no-motion monitoring
through the transition; a clock value or reused PID alone would not establish
those facts. The manual workflow reads the real SLAM map pose before shutdown,
checks the corresponding map/session/stationary conditions and explicitly inputs
it. Unknown conditions require a new manual estimate. A cold restart never loads
that recorded pose. The JSON capture here is test evidence, not replay state.

Separate cold restart: the owned base was stopped while testing a fresh RViz
config, its AMCL/standalone RViz were stopped, then a new one-base/one-RViz session
was started (600 s bound), followed by AMCL (400 s bound) and fresh spawn truth
confirmation. Before/after diagnostics are recorded separately. The updated
config rendered the retained map with Fixed Frame `map` before AMCL initialization.
Real RViz map-frame initialization was accepted; the final verifier retry confirms
new pose and fresh TF. Service and topic were then retested, and diagnostics PASS.
Stopping the first launch-owned RViz triggered the existing child-exit shutdown
policy for that **owned** base; the temporary standalone RViz is not a product
workaround and was stopped before restart. No user process was signalled.

RViz observed: editable `map` value, updated camera target, Map/RobotModel/TF/
LaserScan/AMCL Pose/Particle Cloud displayed together after initialization; scan
visually follows the nearby mapped wall. Screenshot:
[rviz_map_initialized.png](evidence/m4_1/rviz_map_initialized.png).
Costmap warnings are expected with Nav2 deliberately absent. A user-specific GUI
lock, full warehouse scan alignment/coverage and video are NOT VERIFIED. No new
navigation goals or full Nav2 startup are claimed; existing M4 regressions pass.

## Build/tests and limitations

```bash
# Initial baseline (against the user's newer map):
timeout 150 python3 -m pytest src/turtlebot4/test -q
# Result: 112 passed / 4 failed; all four failures tied to old map constants/types.
colcon build --symlink-install --packages-select turtlebot4
source install/setup.bash
timeout 150 colcon test --packages-select turtlebot4 --event-handlers console_direct+
colcon test-result --test-result-base build/turtlebot4 --verbose
```

Final: **150 tests, 0 errors, 0 failures, 0 skipped**; all 116 prior cases retained
and 34 new cases. Real Nav2 map loading/saving/reloading and native TF2 callback
regression run as part of that suite. Build succeeds. Git diff whitespace check
passes. Namespace selection is covered offline; a namespaced simulator is not
replayed here. Automatic service-absent fallback selection is tested offline;
real topic transport was tested explicitly against installed AMCL with its service
also present. Fresh CLI initialization is not a global localization accuracy check:
operators must still confirm scan/map alignment and current pose before Nav2.

## Acceptance matrix

| Criterion | Status / evidence |
| --- | --- |
| Preserve/load user's 603 × 1002 map | PASS — identical hashes and actual grid |
| Real Map Server/AMCL lifecycle ACTIVE | PASS — GetState, final diagnostics |
| Initial pose accepted via installed interfaces | PASS — logs, CLI service/topic, GUI map request |
| New AMCL pose and fresh connected TF | PASS — request-effect checks and final TF |
| check_localization PASS | PASS — direct, handoff and cold restart |
| Plausible stationary/moving pose | PASS within 0.30 m trial bound — real ground truth |
| RViz editable map frame and coherent nearby view | PASS in replay — screenshot and real GUI request |
| User's exact prior GUI lock | NOT REPRODUCED — manual steps documented |
| All prior tests/new regressions | PASS — 116 + 34 = 150 |
| Full repeat of user's manually collected map/video | NOT VERIFIED — short SLAM reference-map handoff only |
| Automatic snapshot handoff | Not implemented; explicit manual current-pose input |
| New Nav2 runtime/goals | Not run; outside required localization correction |

## Git

Verified correction is committed with the requested message and pushed only to
`origin/assignments_4` when authentication succeeds. Final implementation SHA,
remote verification and owned-process cleanup are appended after those actions.
User map changes remain uncommitted by design. No force push.

Cleanup: SIGINT was sent only to the recorded owned localization runner and base
launch. Their children stopped; a fresh no-daemon graph is empty. The upstream
TurtleBot node needed launch-managed SIGTERM escalation, as in M4 Recovery.
No simulator/SLAM/AMCL/RViz test process remains.

Verified implementation commit: `bf78625199c5abac52e022f9f6c1843040e7e26c`.
Message: `fix(assignment4): restore AMCL initial pose and RViz map frame workflow`.
`git push origin HEAD:assignments_4` succeeded (`e3f1bd1..bf78625`). Independent
`git ls-remote origin refs/heads/assignments_4` returned the identical full SHA.
After the push, status contains only the original user changes to the two map
files. This Git-result entry is a separate documentation-only follow-up; its SHA
is available with `git log -1 --format=%H -- docs/M4_1_LOCALIZATION_INITIAL_POSE_FIX_REPORT.md`.
No application code changed after the final 150-test run.
