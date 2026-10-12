# M4 — Nav2 Autonomous Navigation and Three-Goal Verification

Date: 2026-10-12 (Asia/Bangkok). Branch: `assignments_4`.
Baseline: `4d885b575693329f7148d746ccdc174af1284d91`.
M3.5 ancestor: `ebd889b5c7a8cafbed79f85101130b15af98aed5`.

**M4 acceptance: FAIL. Software build/regressions PASS; three successful autonomous
navigation goals NOT VERIFIED (0/3).** Navigation orchestration, safety parameters,
diagnostics and RViz support are implemented. Real execution exposed stale native
costmap/TF state and lifecycle-service timeouts. Attempts were cancelled and the
final runner fails readiness rather than declaring this stack safe for motion.
No M5/M6 implementation was performed.

## Architecture and installed upstream integration

One assignment base launch owns Gazebo Fortress, one Standard robot, bridges,
robot TF and one RViz. Existing `start_localization` owns Map Server, AMCL and
`lifecycle_manager_localization`. New `start_navigation` owns a separate foreground
navigation launch; it does not include simulation, spawn, SLAM, localization or
RViz. AMCL remains the intended `map -> odom` authority.

Inspected the installed Humble launch sources:

- `turtlebot4_navigation/launch/nav2.launch.py`: accepts namespace, use_sim_time,
  params_file; remaps both costmap scan topics and explicitly includes
  `nav2_bringup/launch/navigation_launch.py` with composition disabled.
- `navigation_launch.py`: complete managed list is controller_server,
  smoother_server, planner_server, behavior_server, bt_navigator,
  waypoint_follower and velocity_smoother. The runner discovers this list from
  installed source and compares the real manager's `node_names`; it does not
  replace upstream lifecycle management with an incomplete fixed list.
- `turtlebot4_navigation/config/nav2.yaml`: copied into the assignment with all
  original settings preserved except the two planner safety changes below.

All ROS execution occurred in the existing Ubuntu 22.04/Humble container shell.
No host ROS packages, installed upstream sources, UR code, 9Router configuration
or other branches were changed. Initial RMW was `rmw_cyclonedds_cpp`; middleware
comparisons used process-scoped `rmw_fastrtps_cpp`, without modifying global setup.
Neither middleware comparison establishes a reliable accepted navigation session.

## Source/configuration changes

| File | Purpose |
| --- | --- |
| `turtlebot4/navigation_runner.py` | Foreground navigation-only owner; map/config checks, bounded prerequisites, duplicate/SLAM/teleop rejection, explicit launch forwarding, readiness, signal handling and bounded owned-descendant cleanup with PID start-time checks. |
| `turtlebot4/navigation_diagnostics.py` | Shared real graph, lifecycle and parameter clients; advancing clock, fresh sensors/TF, plausible stable AMCL, matching real map, free robot cell, costmap update/footprint checks, action and command interfaces, interpolated path checks, JSON/text CLI. Reuses service clients across polls. |
| `config/nav2.yaml` | Assignment-owned upstream parameter configuration, with `GridBased.allow_unknown: false` and `GridBased.tolerance: 0.0`. |
| `rviz/simulation.rviz` | Adds Global Path, Global/Local Costmap displays and installed `nav2_rviz_plugins/GoalTool`. Preserves odom initial fixed frame and all prior displays/tools. |
| `setup.py` | Registers `start_navigation` and `check_navigation`; existing resource discovery installs the new YAML. |
| `package.xml` | Declares Nav2, lifecycle/parameter/TF, RViz and verification-helper dependencies. |
| `scripts/verify_navigation.py` | Simulator-only action verifier; actual-map goal selection, clearance, real planner failure checks, path/controller observations, localization/bumper/hazard monitoring, bounded cancellation, pose tolerances and stop checks. No manual Twist publication during goals. |
| `test/test_m4.py` | 26 M4 cases, including actual ROS probe construction and real cleanup of a descendant in a separate process group. |
| `README.md` | Vietnamese terminal workflow, conditional RViz goal tutorial, video checklist and explicit failed acceptance status. |
| `docs/evidence/m4/` | Genuine logs, JSON measurements, probes, failed-goal evidence and screenshots. |

`maps/map.yaml` and `maps/map.pgm`, M1–M3.5 source/tests and previous reports remain
unchanged. PGM SHA-256 remains
`9bdc7c7095f17db56a5112e9c0bae857d9d8f29e9db757e9b4313410761b2dbc`.

## Unknown-space safety and actual parameter audit

The saved map still publishes 80,090 Free, 1,938 Occupied and 137,449 Unknown cells,
491 × 447 at 0.05 m/cell. Its M3.5 metadata remains trinary/non-negated with free
threshold 0.196 and occupied threshold 0.65.

The assignment configuration changes only:

| Parameter | Upstream | Assignment |
| --- | --- | --- |
| `planner_server.GridBased.allow_unknown` | true | **false** |
| `planner_server.GridBased.tolerance` | 0.5 m | **0.0 m** |

The second change prevents substituting a nearby reachable point for an invalid
requested goal. Inspected matching
[Navfn 1.1.20 implementation](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_navfn_planner/src/navfn_planner.cpp)
for the tolerance search. Controller goal tolerances remain 0.25 m / 0.25 rad.

Real parameter-service responses in
[initial_parameter_audit.json](evidence/m4/initial_parameter_audit.json) confirmed:

- Planner `nav2_navfn_planner/NavfnPlanner`, allow_unknown false, tolerance 0.0.
- Global track_unknown_space true; static/obstacle/inflation layers;
  static_layer.map_subscribe_transient_local true; obstacle scan topic `scan`.
- Local static/voxel/inflation layers; voxel scan topic `scan`.
- Both costmaps robot_radius 0.175 m and inflation_radius 0.45 m.
- Controller `dwb_core::DWBLocalPlanner`, actual goal tolerances 0.25/0.25.
- Planner, controller and both costmap `use_sim_time` values true; the upstream
  launch also receives `use_sim_time:=true` for its parameter rewrite.

The upstream radius is retained, and the installed Create 3 base collision
cylinder has radius 0.164 m (`irobot_create_description/urdf/create3.urdf.xacro`).
This uses the official TurtleBot navigation footprint; a new full mesh-envelope
calibration was not performed.

Measured raw costmaps initially had:

| Map | Size | Unknown (255) | Lethal (254) | Free (0) | Inflated |
| --- | --- | ---: | ---: | ---: | ---: |
| Global | 491 × 447, 0.05 m | 132,603 | 1,989 | 68,468 | 16,417 |
| Local | 50 × 50, 0.06 m | 0 | 10 | 2,333 | 157 |

Layered costmap counts need not equal the saved map: inflation and real sensor
marking/clearing alter costs. Actual unknown and occupied test goals were sent
only to `ComputePathToPose`, without controller execution:

| Requested cell | Coordinates (m, rounded) | Saved class | Raw cost | Result |
| --- | --- | --- | ---: | --- |
| Unknown | (-13.375, -12.575) | -1 | 255 | ABORTED (6), empty path |
| Occupied | (-7.875, -12.575) | 100 | 254 | ABORTED (6), empty path |

These are real planner failures, not mocked goal labels. A free-space plan was
also generated before the first accepted navigation attempt. The verifier checks
all path segments at half-cell spacing against both the original map and raw
costmap; it rejects any unknown/occupied saved-map cell or cost >=253. No unknown
cells were relabelled to obtain a plan. Reliable sustained motion remains unproven.

## AMCL moving-localization regression

Before navigation attempts, a separate bounded simulation probe checked ground
truth, current map/TF and scan clearance, sent the real `Undock` action and checked
its SUCCEEDED status (4) plus `is_docked=false`. It then checked a short forward
segment in saved-map Free cells, drove at 0.08 m/s for four seconds, sent repeated
zero commands, and checked the observed odometry stop. This manual command phase
ended before navigation goal execution.

Latest actual measurements in
[moving_localization.json](evidence/m4/moving_localization.json), from the
all-Fast-DDS comparison:

| Stage | AMCL/TF estimate x,y (m) | Ground truth x,y (m) | Position error |
| --- | --- | --- | ---: |
| Before undock | (0.00645, -0.05552) | (0.00505, -0.00076) | 0.05478 m |
| Undocked | (-0.30597, -0.05208) | (-0.34564, 0.01534) | 0.07822 m |
| After bounded segment and stop | (-0.63698, -0.06343) | (-0.66555, 0.01812) | 0.08641 m |

Maximum error during commanded-segment samples was 0.08342 m; maximum sampled yaw
error was 0.01733 rad. Final stopped linear velocity was approximately
`4.2e-16 m/s`. Observed hazard type 0 is BACKUP_LIMIT, which was not overridden.
No BUMP/CLIFF/STALL/WHEEL_DROP condition was bypassed. Earlier CycloneDDS bounded
tracking also passed (maximum segment error approximately 0.0882 m).

This is a **PASS for the bounded pre-navigation localization regression**, not
proof that localization remains reliable during the later failing Nav2 attempts.

## Navigation lifecycle, path/control observations and failure

An initial live navigation bringup reached ACTIVE for all seven upstream managed
nodes and both costmap lifecycle nodes. `NavigateToPose` was available; real
parameter services and command interfaces were observed. Gazebo, RViz, AMCL and
Map Server were reused without starting duplicate localization. The initial
probe reported PASS, but its wall-time receipt check was insufficient to identify
old retained costmap data; that readiness result is superseded by the stricter
final implementation.

Two CycloneDDS goal-A attempts were accepted and produced real planner/controller
activity. Neither reached verified SUCCEEDED. The controller repeatedly reported:

```text
Transform data too old when converting from map to odom
Data time: 159s 735000000ns, Transform time: 127s 882000000ns
```

Progress checks failed. The action verifier cancelled when observed localization
error exceeded 0.30 m; native navigation logs confirm cancellation and controller
stop. Later checks rejected attempts from an occupied robot map cell.

An independent footprint probe exposed stale native costmap state:

| Footprint | Footprint center | Published stamp | Live TF position | Live TF stamp |
| --- | --- | ---: | --- | ---: |
| Local (odom) | (-0.66653, -0.06258) | 126.903 s | (-5.97081, 7.13133) | 561.807 s |
| Global (map) | (-0.66021, -0.09081) | 126.276 s | (-4.98253, 8.06785) | 561.807 s |

See [stale_footprints.json](evidence/m4/stale_footprints.json). Fresh `/clock`, scan,
odometry and external TF were insufficient to establish healthy internal Nav2
costmap updates. The underlying native TF/costmap/service stall is **not resolved**
and its precise cause has not been independently isolated. Do not attribute it
to middleware alone based on these comparisons.

Final readiness now checks advancing costmap update timestamps (not just new
receipt time), footprint timestamps and agreement with current TF. It also
rejects a robot outside original mapped Free space, observes localization/TF
progress, retires readiness subscriptions before the foreground wait, and reuses
service clients rather than recreating reply endpoints in each poll.

A mixed Fast-DDS/CycloneDDS trial failed on Map Server service completion. An
all-Fast-DDS session passed the bounded moving-localization regression but the
navigation runner ultimately failed its 45 s gate:

```text
Readiness failed after 45.0s: Service timed out: /lifecycle_manager_navigation/get_parameters
```

It shut down owned navigation components. No goals were sent in that failed
comparison. Middleware changes were limited to these processes and were not
adopted as a recommended/global fix.

## Three-goal results and collision evidence

**Successful autonomous goals: 0/3.**

| Goal | Actual attempted start/target | Result | Final tolerance / safe arrival |
| --- | --- | --- | --- |
| A, first trial | Start approx. (-0.68,-0.08), target (-2.025,-0.025) | Accepted; cancelled after localization drift | NOT VERIFIED |
| A, fresh CycloneDDS trial | Start approx. (-0.66,-0.09), target (-2.175,-0.075) | Accepted; cancelled after stale TF/drift | NOT VERIFIED |
| B | No autonomous execution | NOT VERIFIED | NOT VERIFIED |
| C | No autonomous execution | NOT VERIFIED | NOT VERIFIED |

Start/target values come from actual logs and the saved selection origin; final
arrival poses were not captured as successful results. Do not interpret the later
independent occupied-cell measurement as a successful final goal pose. The failed
attempt data retain FAIL status and no successful-goal records.

No teleop node or manual Twist publisher from the verification code was active
during the attempted goals. Planning/control activity alone does not prove safe
navigation. Bounded tracking stopped successfully; three autonomous stops after
arrival and collision-free completion are **NOT VERIFIED**. The final verifier
adds bumper contacts, hazard checks, advancing costmaps and live TF monitoring,
and bounded goal cancellation. Its new full three-goal success path was not
executed after readiness remained unsafe.

## Exact executed commands and regression results

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon list
git -C src/turtlebot4 branch --show-current
git -C src/turtlebot4 status --short
ros2 launch turtlebot4_navigation nav2.launch.py --show-args

# Original baseline, then final package build/test:
timeout 150 python3 -m pytest src/turtlebot4/test -q
timeout 90 colcon build --symlink-install --packages-select turtlebot4
timeout 150 colcon test --packages-select turtlebot4 --event-handlers console_direct+
colcon test-result --test-result-base build/turtlebot4 --verbose

# Live sessions (separate foreground command owners):
timeout --signal=INT --kill-after=20s 1800s \
  ros2 launch turtlebot4 turtlebot4.launch.py rviz:=true
timeout --signal=INT --kill-after=15s 1500s \
  ros2 run turtlebot4 start_localization --timeout 30
timeout 50 python3 src/turtlebot4/docs/evidence/m3_5/localization_probe.py
timeout 75 python3 /tmp/m4_motion.py
# Preserved executable source: docs/evidence/m4/moving_localization_probe.py

timeout --signal=INT --kill-after=20s 1200s \
  ros2 run turtlebot4 start_navigation --timeout 60
timeout 45 ros2 run turtlebot4 check_navigation --timeout 30 --json
timeout --signal=INT --kill-after=15s 370s python3 \
  src/turtlebot4/scripts/verify_navigation.py \
  --output src/turtlebot4/docs/evidence/m4/three_goals.json
# Goal attempts failed/cancelled; no SUCCEEDED claim.

# Independent stale-footprint capture:
timeout 10 python3 /tmp/m4_footprint_check.py
# Preserved source: docs/evidence/m4/footprint_probe.py

# Comparison used export RMW_IMPLEMENTATION=rmw_fastrtps_cpp in only its shells.
# Final isolated navigation command in that session:
timeout --signal=INT --kill-after=20s 230s \
  ros2 run turtlebot4 start_navigation --timeout 45

# Final shutdown / read-only diagnostics without any live stack:
ros2 node list
timeout 8 python3 -m turtlebot4.navigation_diagnostics --timeout 3 --json
git -C src/turtlebot4 diff --check
sha256sum src/turtlebot4/maps/map.pgm
```

Initial baseline: 83 PASS. Final build: PASS. Final package suite:
**109 tests, 0 errors, 0 failures, 0 skipped** (83 unchanged prior cases + 26 M4).
Intermediate editing/constructor defects were caught and corrected before the
final passing run. The suite includes real Nav2 Map Server regression integrations,
a real ROS probe construction and actual isolated process/descendant shutdown;
unit mocks do not establish autonomous motion success.

After explicit shutdown of owned wrappers/base, `ros2 node list` was empty and
process inspection found no remaining owned Nav2 or Gazebo server/GUI processes.
The final standalone diagnostic correctly returned FAIL for missing AMCL.
Some native planner/controller cleanup needed signal escalation during failed
trials. The runner now records descendant start times and can clean separate
child process groups without signalling unrelated processes.

## Screenshots and evidence

[RViz screenshot](evidence/m4/rviz_failed_trial.png) and
[Gazebo screenshot](evidence/m4/gazebo_failed_trial.png) are actual application
window captures from a **failed CycloneDDS trial**, not generated mockups or
proof of successful goals. RViz loaded the added costmaps/path displays and Nav2
Goal tool. No successful three-goal video exists.

Log files have only trailing whitespace normalized for Git; events and measured
values are retained. Other evidence includes initial parameter audit, stale footprint measurements,
failed action/navigation logs, moving-localization JSON and executable probes,
Fast-DDS readiness failure, final build/tests and empty shutdown graph. Original
M0–M3.5 evidence remains unchanged.

## Acceptance matrix

| Criterion | Status | Evidence / limitation |
| --- | --- | --- |
| Persistent base, one robot and one RViz per session | PASS | Actual simulator launch and application captures |
| Navigation-only upstream include; reuse AMCL/Map Server | PASS | Source inspection and initial live graph |
| Initial full lifecycle ACTIVE/action availability | PASS | Actual initial service/graph audit; not sustained-motion proof |
| Corrected map preserved and used | PASS | Actual class counts, unchanged SHA and 83 legacy tests |
| Runtime planner/unknown safety parameters | PASS | Actual parameter responses and failed unknown/occupied planner actions |
| Known-free path planning | PASS | Actual preflight plan and accepted attempt |
| Costmap/TF freshness during motion | FAIL | Stale native footprints/transforms; unresolved blocker |
| Final readiness fails closed on unsafe/incomplete state | PASS | Final bounded failure and rejection checks |
| Bounded moving AMCL regression before navigation | PASS | Real undock, motion, ground truth and final stop |
| Reliable localization throughout Nav2 goal execution | FAIL | Drift exceeded verifier's safety bound; cancellation |
| Three distinct autonomous SUCCEEDED goals | NOT VERIFIED | 0/3; B/C never executed |
| Goal arrival tolerances and stops | NOT VERIFIED | No successful arrival |
| Collision-free autonomous completion | NOT VERIFIED | No accepted three-goal run |
| No manual teleop during attempted goals | PASS | Verifier sends actions only; teleop absent |
| RViz goal tool and displays load | PASS | Real window capture; click-driven successful goal not verified |
| Build and all old/new tests | PASS | 109 tests, zero failures/errors/skips |
| Cleanup of owned processes | PASS | Empty final graph/process audit; real descendant-cleanup test |
| Vietnamese README reflects limitations | PASS | Conditional workflow and explicit failed M4 status |
| M4 overall acceptance | **FAIL** | Required autonomous success gate unmet |

## Remaining issues and M5 recommendations

Resolve the native TF/costmap update stall and service timeouts before further
navigation. Isolate lifecycle/parameter queries, subscriber lifetimes, clocks,
executor/DDS behavior and costmap update threading with a minimal reproducible
upstream launch. Middleware comparisons did not prove a fix. Keep the freshness
checks, original map, unknown-space restrictions, footprint and collision layers.

Then repeat pre-navigation moving localization and all three real goals, recording
start/goal/final poses, actions, paths, commands, bumper/hazard data and observed
stops. Repeat the continuous-session and restart scenarios and record a real RViz
video. Namespaced navigation, full mesh footprint calibration and return-to-SLAM
handoff were not newly verified in M4. Do not proceed to M5/M6 before M4 passes.

## Git result

- Implementation commit: `eb4b42dc064034e01e1a4664714003e297e39dce`.
- Message: `feat(assignment4): integrate Nav2 autonomous navigation`.
- Push to `origin/assignments_4` succeeded without force:
  `4d885b5..eb4b42d HEAD -> assignments_4`.
- Independent `git ls-remote origin refs/heads/assignments_4` returned
  `eb4b42dc064034e01e1a4664714003e297e39dce`, matching the local implementation commit.
- This report is committed separately after that verified implementation push;
  its SHA can be obtained with
  `git log -1 --format=%H -- docs/M4_ENGINEERING_REPORT.md`.
- Only assignment-owned changes/evidence were staged. Workspace build/install/log,
  UR repositories, skills, AGENTS.md and unrelated dirty files were not committed.
- The commit/push verifies delivery of the tested software and honest failure
  evidence. It **does not change M4's failed autonomous-navigation acceptance**.
