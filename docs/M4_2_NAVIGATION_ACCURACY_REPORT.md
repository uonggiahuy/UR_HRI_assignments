# M4.2 — Navigation accuracy, AMCL convergence and costmap consistency

Date: 2026-10-13 (Asia/Bangkok). Branch: `assignments_4`.

**Overall acceptance: FAIL / incomplete.** Verified diagnostic corrections reject
uncertain localization; the RViz startup correction and regression results are
reported below. The new warehouse map is preserved exactly. AMCL remains above
the readiness uncertainty limit after bounded motion. No new autonomous goals
were sent. Physical accuracy, current-map DWB performance and three-goal execution
are NOT VERIFIED. M5/M6 were not implemented.

## Original symptoms and investigation boundaries

User observations: inaccurate Gazebo/RViz correspondence, circuitous motion,
PathDist/GoalDist rejection, 419 invalid trajectories, controller patience aborts,
multiple recoveries, missing static RViz map and an unsafe localization PASS.
These are separate issues; their common cause has not been established.

Initial inspection found Ubuntu 22.04.5 inside the existing container, ROS Humble,
CycloneDDS, DISPLAY=:1, assignment overlay and no running simulator/AMCL/Nav2/RViz.
Docker host tooling is unavailable. No container, middleware, 9Router, UR or vendor
configuration changed. Source map files and the Navigation 2 panel addition were
already modified by the user. They were preserved; the panel is now explicitly
covered by regression tests. All experimental launchers were owned by this task.

Read AGENTS.md, project TurtleBot platform/localization/Nav2/simulation/SLAM/
debugging/demo and ROS general/debugging skills, M0/M1/upstream/M3.5/M4/recovery/
M4.1 reports, runners, diagnostics, initial-pose CLI, navigation verifier, Nav2
YAML, RViz and Vietnamese README. Installed upstream launch lists and AMCL
parameters were inspected. Existing TF2 0.25.24 cache overlay remains unchanged;
AMCL/RViz library mappings are recorded in `evidence/m4_2/runtime_libraries.txt`.
The M4.1 overlay findings are historical; current command environments explicitly
source the pinned overlay.

## Root-cause evidence and before/after

| Issue | Finding | Correction / remaining work |
| --- | --- | --- |
| Misleading localization PASS | Confirmed: finite/nonnegative covariance was sufficient; no maximum uncertainty or scan/odom header-age check. Actual original code reports PASS with x variance 0.22851 m² after initialization (`before_initialized.json`). | Shared covariance/timestamp policy; final live uncertain state fails both diagnostics. |
| Contradictory diagnostics | Confirmed: navigation used an independent `max(variance) > 0.5` rule. | Both use the same eigenvalue and yaw limits, and the same retained-pose timestamp policy. |
| Raw world/map subtraction | Confirmed in the existing verifier/old observer; these assumed identity. | Verifier requires independently validated registration and checks ground-truth frame/time. New measurements explicitly label the candidate registration unverified. |
| RViz retained map | M4.1 established MessageFilter loss in `odom` before map TF. Current replay confirms retained-map trouble; QoS already matches Map Server. | Start the persistent navigation view in `map`; default M1 view remains `odom`. No map republishing or fabricated TF. |
| Uncertain AMCL | Reproduced after initialization, undock and short movement. x uncertainty remains high while TF/sensors advance. | Navigation fails closed. Cause of residual uncertainty needs further measured scan/odom/landmark work; no unjustified AMCL tuning. |
| Circuitous plan / DWB / recoveries | User logs establish symptoms, but no live failure with corresponding path/costmap coordinates was safely reproduced here. | NOT VERIFIED. No changes to DWB critics, local window size, inflation, radius or tolerances. Added geometric diagnostic support; no speculative root-cause claim. |

## AMCL covariance and readiness

The user's variances 4.19021 / 2.50007 m² and 0.474 rad² correspond to standard
deviations about 2.05 / 1.58 m and 39.4°. Connected TF cannot make that state safe.
New regression cases reject it.

Shared acceptance policy in `turtlebot4/localization_health.py`:

- finite, symmetric, positive-semidefinite planar x/y/yaw covariance;
- largest XY covariance eigenvalue ≤ 0.09 m² (major-axis sigma ≤ 0.30 m);
- yaw variance ≤ (15°)² = 0.0685389 rad²;
- scan/odom measurement age ≤ 1 s, at most 0.10 s into the future;
- advancing simulation clock and current supporting sensor reception;
- advancing, fresh dynamic TF and at least two seconds of observation;
- old retained AMCL output needs stationary observations and agreement with current
  TF within 0.15 m / 0.15 rad; moving output older than two seconds is rejected.

These are conservative **necessary readiness limits**, not experimentally proven
physical accuracy guarantees. The live trial does not meet them. No uncertainty
was rewritten or shrunk. Initial-pose covariance remains M4.1's genuine seed
uncertainty. A PASS from `set_initial_pose` verifies request effects, not convergence.

`check_localization` preserves `overall_pass` but reports explicit `checks`:
`localization_stack_active`, `map_valid`, `tf_connected`, `sensor_freshness_pass`,
`amcl_pose_available`, `amcl_covariance_acceptable`, `localization_converged`,
`ground_truth_accuracy_verified`, `navigation_safe`. Ground-truth accuracy remains
false without separate registered-reference evidence. `navigation_safe` means
these readiness gates passed; it does not replace scan/map review or physical
accuracy validation. `check_navigation` and the runner use the same uncertainty
and timestamp functions.

## Experiment A — synchronized localization and controlled motion

A new owned warehouse/Standard simulator was initialized from independently
identified local map geometry, using a **coarse experimental seed** (0.148,0.016,
-0.0265°), derived from the candidate shelf registration below. This is not an
operator default or a verified exact map pose. The held-out local scan agreed
with mapped obstacles, making a bounded initialization/observation experiment
reasonable; it does not validate global accuracy.

Real `set_initial_pose` service initialization produced [0.140,-0.057], yaw 0.2°.
Original diagnostics returned PASS at x variance 0.22851, y 0.18126, yaw 0.05867.
The original runner started Nav2 without sending any goal. That owned Nav2 stack
was stopped; after rebuilding the correction, the runner rejects uncertain
localization before launching Nav2 (`navigation_guard_fixed.txt`).

Undock returned success. A separate controlled experiment required robot-side
cmd_vel, undocked state, real scan clearance > 0.25 m, mapped swept center-path
clearance > 0.40 m, fresh sensors/TF and no prohibited hazards. It published
0.08 m/s for four seconds, then repeated zero commands and checked stopped odometry.
No teleop ran during any Nav2 goal (there were no goals).

| Measurement | Before short movement | After stop |
| --- | --- | --- |
| World position | (-0.35261,-0.000001) | (-0.67228,0.00214) |
| TF map pose | (-0.25404,-0.13342) | (-0.57293,-0.14390) |
| Major position sigma | 0.37544 m | 0.38153 m |
| Yaw sigma | 0.17086 rad | 0.20358 rad |
| TF age | 0.006 s | 0.003 s |
| Scan age | 0.018 s | 0.012 s |
| Odom age | 0.006 s | 0.003 s |
| Candidate-aligned position residual | 0.15733 m | 0.16929 m |
| Candidate-aligned heading residual | 0.06004 rad | 0.03446 rad |

Ground-truth and TF/scan samples are timestamped in `motion.json`; sampling is
about 10 Hz with all control-loop safety checks maintained. The last reported
AMCL pose is acquired before the final movement ends; TF extrapolates via actual
odom. Pose receipt time is not substituted for acquisition time.

**Physical localization accuracy: NOT VERIFIED.** The residuals in the table use
an exploratory transform whose validation failed; they are sensitivity measurements,
not certified error values. World displacement is about 0.320 m; final linear /
angular odometry is approximately zero. No bumper contacts or prohibited hazard
were recorded. Hazard type 0 (BACKUP_LIMIT) was observed and not bypassed. This
bounded trial does not establish collision-free three-goal execution.

## Gazebo-to-map registration methodology and limits

Map origin [-15,-25.1,0] is grid metadata, not registration evidence. The user
confirmed the original SLAM frame history and surveyed map landmarks are unknown.

Independently inspected installed `warehouse.sdf` and cached Fuel `shelf_big`
`model.sdf`. Three large shelves have world poses x=-8.5,-1.5,6.5 and y=-13;
the link has x offset -0.5 and a 2.1 × 18 m collision box. Their world rectangle
corners are derived from those actual SDF definitions, not AMCL. Corresponding
isolated occupancy rectangles are identifiable in the current map image.
`register.py` extracts their occupied bounds and fits one rigid SE(2) transform.

Fit: translation (0.147669,0.015896) m; yaw -0.0004617 rad; RMS residual 0.15560 m;
maximum corner residual 0.19123 m. Some edges include outlier/ghost occupied cells,
so bounding extrema are deliberately not treated as exact surveyed landmarks.
The three observed rectangle widths exceed the collision box width by about
0.20–0.35 m. This may reflect map artifacts, extrema selection or sensor-visible
geometry; the current evidence does not isolate accumulated drift.

Held-out default-spawn LiDAR uses the real static sensor transform, including
its 90° yaw and -0.04 m x offset. Against occupied map cells, candidate-registration
median endpoint residual is 0 m and 90th percentile 0.005 m. All sampled endpoints
are within 0.15 m. Identity alone also has good local scan agreement (median 0 m,
p90 0.05 m). Local scan agreement therefore cannot uniquely prove identity or
precise global registration. No static world/map TF was published.

The candidate exceeds the 0.15 m reference-uncertainty contract and has
`verified:false`. It is rejected by the execution verifier. A stronger reference
needs independently identified exact wall/shelf corners, robust edge fits,
spatially distributed held-out landmarks and uncertainty accounting. Do not simply
flip that flag or change the map origin. No remapping recommendation is justified
until distortion is separated from alignment and measurement geometry.

## Experiment B — saved map assessment

Actual current map: 603 × 1002, resolution 0.05 m, trinary, thresholds .196/.65;
505,728 Free, 15,141 Occupied, 83,337 Unknown (604,206 cells). Map Server reloads
the same ordered occupancy signature. Unknown semantics remain protected.

Four-connected Free components: 294; largest contains 505,124 Free cells
(99.88% of Free). Small isolated components are not treated as reachable corridors.
The occupancy plot shows straight principal walls and long shelf rectangles, plus
small speckles/thick or doubled edges and irregular coverage near the southern
boundary. Those visual features are investigation leads, not proof of SLAM drift.
The default local spawn and controlled movement center path are in sufficiently
clear mapped Free space. Full warehouse obstacle/inflation passage assessment is
NOT VERIFIED. Original files were never saved/replaced/reverted by this task.

## Experiments C/D — global plan, local costmap and DWB

**Runtime execution blocked by localization readiness.** No new Navfn plan or
NavigateToPose goal was requested after the corrected checks failed. Consequently
path length/direct distance, sampled inflation/obstacle clearance, global-grid
origin, time-varying local overlap, critic scores, missed controller deadlines,
recovery counts and commanded-versus-actual autonomous motion are NOT VERIFIED.
The user-observed DWB messages alone do not identify mechanisms A–F.

Installed configuration remains: global `map`, local `odom`, rolling 3 × 3 m,
robot radius 0.175 m, inflation radius 0.45 m, static/voxel/inflation local layers.
These YAML values are not claimed to be new runtime parameter evidence. Earlier
native startup configured the standard stack, but did not provide a complete
coordinate snapshot. No window enlargement, critic disabling, unknown-space
conversion or goal-tolerance relaxation was made.

Diagnostics now query actual global/local frames, local size and rolling-window
parameters, compare grid frame with the parameter, ensure the robot lies inside
the grid and the rolling window is centered, and expose raw grid origin/frame/stamp.
For an observed path they transform map-frame vertices explicitly into local-grid
coordinates and count inside/blocked/unknown/free samples. Classification separates
stale data, empty plan, no geometric overlap and overlapping blocked points; an
overlapping plan still requires critic/motion evidence. This is diagnostic support,
not a demonstrated DWB repair. Regression tests use native ROS grid/path/transform
messages with translated and rotated coordinate systems.

## Experiment E — RViz retained-map behavior

Map Server ACTIVE and `/map` type `nav_msgs/msg/OccupancyGrid`, Reliable /
Transient Local / Keep Last 1 were observed. RViz matches that QoS. The source
map display already had `Use Timestamp: false`, which affects rendering but does
not bypass the upstream MessageFilter delivery gate.

Upstream [Humble MapDisplay](https://github.com/ros2/rviz/blob/humble/rviz_default_plugins/src/rviz_default_plugins/displays/map/map_display.cpp)
and installed headers were inspected. M4.1 demonstrated that a retained map
received in `odom` before map TF can be discarded and changing Fixed Frame later
need not replay it. No QoS change is warranted.

The persistent base has optional `rviz_fixed_frame:=map`, selecting
`rviz/navigation.rviz`. It differs from `simulation.rviz` only in Fixed Frame;
all displays, Navigation 2 panel and Nav2 Goal tool persist. Default M1 behavior
still uses the odom view. A preliminary `rviz2 -f map` experiment hid the editable
Fixed Frame field; the final implementation uses the config variant instead.
That preliminary experiment is not the shipped fix.

Final replay loaded the map view before Map Server startup, initialized AMCL,
and rendered the actual map without unticking/ticking Map or changing Fixed Frame.
`final_rviz.png` shows map, aligned local scan, robot/particles and the Navigation 2
panel/tools with Global Status OK. The panel reports Unknown while its Nav2 lifecycle
manager is absent; this is not a claim of navigation ACTIVE. `final_rviz_status.png`
also shows the editable Fixed Frame value `map`.

Final live view/restart observations are recorded in `final_base.log`,
`final_localization_launch.log`, `final_initialization.txt`, and `final_rviz.png`.
Map rendering must be checked after initialization; missing robot/scan TF before
initialization remains an honest warning. No fake map transform is inserted.
Arbitrary later GUI changes back to odom and again to map are not repaired by this
startup option; if the user deliberately changes frames and loses a retained map,
the M4.1 refresh procedure may still be needed.

## Exact corrections and regression coverage

- `localization_health.py`: shared covariance/measurement-time policy, SE(2)
  composition, reviewed registration validation and frame-aware ground truth.
- `localization_diagnostics.py`: explicit readiness checks, header and receipt
  freshness, advancing TF, stationary support and observation stability.
- `navigation_diagnostics.py`: same covariance/time policy, rolling-window
  geometry checks, costmap coordinates and transformed path overlap classification.
- `scripts/verify_navigation.py`: requires registration before motion, compares
  aligned fresh ground truth and records physical final-target residual separately.
- `launch/turtlebot4.launch.py`, `simulation_launch.py`, `navigation.rviz`: optional
  map-startup view; original simulation view and launch remain compatible.
- `simulation.rviz`: retains the user's existing Navigation 2 panel addition.
- `test_m4_2.py`: 18 cases covering high covariance with available TF, covariance
  correlation/PSD, stationary retained messages, timestamps, registration rejection,
  SE(2), rotated grid origins, local overlap, costmap age, rejection classification,
  RViz/panel/tools, unknown protection, launch option and native DDS retained delivery.
- `test_m2.py`: extends its exact argument-set assertion for the optional view
  argument; all original test cases/assertions otherwise retained.
- Vietnamese README: documents current-map failure, readiness policy, verified
  launch/diagnostic workflow, registration gate, terminal ownership and video limits.

All original 150 cases remain. Final results and command outputs are in
`evidence/m4_2/tests.txt`, `colcon_tests.txt`, `colcon_results.txt`, `build.txt`.

## Goal A/B/C and final acceptance matrix

| Property | Result |
| --- | --- |
| Map hashes unchanged; user map excluded from Git | PASS |
| Original 150 tests + 18 new tests | PASS — 168 tests |
| Targeted package build | PASS |
| AMCL / Map Server ACTIVE | PASS in owned experiments |
| Connected/fresh TF and sensors | PASS in bounded localization observations |
| High covariance never yields navigation-ready PASS | PASS for tested states; both live diagnostics reject uncertainty |
| AMCL convergence in current-map replay | FAIL — major sigma about 0.38 m |
| Validated world/map reference within 0.15 m | FAIL — candidate max residual about 0.19 m |
| Accurate physical localization / final target | NOT VERIFIED |
| Retained static map and navigation GUI | PASS for map-startup replay after initialization; panel/tools visible |
| Global/local grid consistency under autonomous motion | NOT VERIFIED |
| Safe valid DWB trajectories / resolved recoveries | NOT VERIFIED |
| Goal A: short straight | NOT RUN — localization gate failed |
| Goal B: turn | NOT RUN — localization gate failed |
| Goal C: legitimate obstacle traversal | NOT RUN — localization gate failed |
| Goal accepted / SUCCEEDED | 0 / 0 new goals |
| Collision-free autonomous three-goal execution | NOT VERIFIED |
| Bounded controlled move and final stop | PASS — 0 recorded bumper contacts, verified zero odometry |
| Complete end-to-end/restart navigation regression | NOT VERIFIED |
| No upstream, UR, TF2 or safety-parameter change | PASS |

## Map preservation

Initial/final SHA-256 (both remain original uncommitted user changes):

- YAML: `b1be9283ae2c966e54ce6000133263b62fcf7daa2a84cce61c94b210622bf8c9`
- PGM: `915c88f67974aa4e160f2679cc17c538efceddc1e2ed2f9eb0cf05000b3a4955`

Evidence retains a derived occupancy plot and scan points, not staged copies of
the map files. No map saver command was run against the user's map. Regression
map-server/saver tests operate in temporary locations.

## Reproduction and remaining blockers

All ROS commands run in this Humble container:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source /root/.cache/turtlebot4/tf2-0.25.24/install/setup.bash
colcon build --symlink-install --packages-select turtlebot4
# Re-source assignment then TF2 after a build.
source install/setup.bash
source /root/.cache/turtlebot4/tf2-0.25.24/install/setup.bash
colcon test --packages-select turtlebot4 --event-handlers console_direct+
colcon test-result --test-result-base build/turtlebot4 --verbose
```

Owned foreground experiment commands (bounds may be shortened):

```bash
# Terminal 1; one base/RViz, no pre-existing user processes:
timeout --signal=INT --kill-after=20s 300s ros2 launch turtlebot4 turtlebot4.launch.py rviz:=true rviz_fixed_frame:=map
# Terminal 2; SLAM absent:
timeout --signal=INT --kill-after=15s 200s ros2 run turtlebot4 start_localization --timeout 30
# Terminal 4, after independently determining a COARSE current map pose:
ros2 run turtlebot4 set_initial_pose --x X --y Y --yaw-deg ANGLE --timeout 20
ros2 run turtlebot4 check_localization --timeout 6 --json
# Only if localization and independent scan/map assessment pass:
ros2 run turtlebot4 start_navigation --timeout 45
ros2 run turtlebot4 check_navigation --timeout 10 --json
```

Evidence research scripts use `/tmp/m4_2`; to repeat read-only map/registration
analysis copy `map_scan_audit.py` and `register.py` there, source the environment,
run the first while an owned simulator/map server is active, then the second.
The scan script writes `scan_map.npz`; the registration script reads that file and
the unchanged source map. `observe_motion.py --output FILE` observes only;
`--motion` explicitly publishes a bounded command and needs a safely undocked
simulated robot. It is an experimental evidence tool, not a general operator
command. Do not run it to force readiness or use its candidate residual as accuracy.

Next required evidence: robust independently checked wall/corner correspondences,
scan agreement across distinct regions, AMCL observability/convergence under
bounded controlled movement, then planner and DWB coordinate snapshots followed
by the three requested goals. Until those gates pass, this report remains M4.2 FAIL.
No random goals, fabricated covariance/TF or unmeasured parameter tuning were used.

## Git and owned-process cleanup

Only verified assignment-owned corrections and truthful evidence are eligible for
commit. Maps are explicitly excluded. Implementation commit/push verification and
final owned-process cleanup are recorded below after those actions.

Final software validation: targeted build PASS; pytest **168 passed**; colcon
**168 tests, 0 errors, 0 failures, 0 skipped**. Final map hashes match the initial
values. The incomplete runtime acceptance above remains unchanged.

Owned-process cleanup: bounded foreground sessions stopped (normal timeout/launch
cleanup); final no-daemon ROS graph is empty and no Gazebo, RViz, AMCL or Nav2
process remains. No user-owned process was signalled.

Evidence formatting: trailing whitespace was removed from copied text logs;
no runtime errors were removed. The final bounded base shutdown records a Gazebo
GUI teardown stack trace and Ruby ESRCH while children exit. Final process/graph
checks are empty, but this is not a claim of error-free upstream shutdown. The
AMCL/RViz library audit excludes the inspecting shell's own command text.

Verified implementation commit: `8ebcd1ab60094914b666c3672e53bea7801138c1`
(`fix(assignment4): reject uncertain localization and retain RViz map`).
Overall M4.2 remains FAIL; this commit contains software corrections and truthful
incomplete acceptance evidence, not an accepted navigation demonstration.

Push verification: `git push origin HEAD:assignments_4` succeeded without force,
advancing remote `5f2f6b5..fe16bb1`. Independent
`git ls-remote origin refs/heads/assignments_4` returned
`fe16bb14f337285e21e56e6d240ce3a9ce813b6e`, identical to the pushed local HEAD.
Only the user's two original modified map files remain unstaged. This Git-result
entry is a documentation-only follow-up; obtain its containing SHA with
`git log -1 --format=%H -- docs/M4_2_NAVIGATION_ACCURACY_REPORT.md`.
No source code changed after the final 168-test run.
