# M4 Recovery — TF2 lock inversion and observer starvation

Date: 2026-10-12 (Asia/Bangkok). Branch: `assignments_4`.
Baseline: `39122e1` (M4 implementation ancestor `eb4b42d`).

**Navigation runtime recovery: PASS. Three distinct real NavigateToPose goals
SUCCEEDED, with the configured Nav2 pose tolerances, bounded localization error,
fresh TF/footprints, no recorded collision/hazard and verified stops.**
Physical target accuracy of 0.25 m is **not** established for all goals: goal C's
Gazebo ground truth is 0.434 m from its target despite an AMCL-frame error of
0.240 m. No M5/M6 work, teleports, fabricated TF, map edits or safety overrides.

## Root causes and isolation

1. **Native TF freeze: confirmed TF2 0.25.23 ABBA deadlock.** Controller thread
   14 waits inside `BufferCore::addTransformableRequest`, called by the costmap
   LaserScan message filter. Thread 13 waits inside the Buffer transform-ready
   callback, called by `testTransformableRequests` on the TF listener. The
   matching release holds `transformable_requests_mutex_` while calling that
   callback; `Buffer::waitForTransform` holds `timer_to_request_map_mutex_`
   before requesting the first mutex. The opposite order blocks both threads.
   See [native stack trace](evidence/m4_recovery/debug_nav.log), particularly
   threads 13/14. The update thread is sleeping normally in `mapUpdateLoop`,
   not stuck waiting for the costmap mutex. External TF and simulator sensors
   continue advancing. This is an application locking failure, not evidence
   against CycloneDDS, Fast DDS, Gazebo or RViz.
2. **“Frozen costmaps”: distinguish TF state from publication.** In this
   reproduction footprints keep arriving with the same old TF-derived stamp,
   while raw costmap publication headers advance. New receipt times cannot
   prove healthy geometry. In Nav2 1.1.20 `prepareCostmap()` does not populate
   `metadata.update_time`; its zero value cannot measure topic update progress.
   The `GetCostmap` service fills that field separately. The diagnostic now uses
   a nonzero metadata stamp when available, otherwise the publication header,
   AND still requires advancing/fresh footprints consistent with live TF.
3. **Lifecycle timeout: reproduced diagnostic misattribution.** The original
   probe reports `/lifecycle_manager_navigation/get_parameters` timing out,
   while independent requests throughout the same trial all succeed. A direct
   call from NavigationProbe also succeeds. At overall deadline exhaustion,
   its last zero-budget service attempt overwrites the actual readiness error.
   The corrected loop preserves the last substantive failure, avoids issuing
   expired-budget requests and removes/cancels timed-out pending futures.
   Historical mixed-middleware timeouts are not explained by this result.
4. **Verifier TF starvation: independently reproduced.** After the native fix,
   the first goal is safely cancelled by the Python verifier's stale-TF check.
   Its loop counts the entire 219,477-cell grid between single executor steps.
   A stationary comparison measured 507 callbacks / 3 s and TF age 0.597 s with
   that guard, versus 2,992 callbacks and age 0.015 s with a constant-time guard.
   See [comparison](evidence/m4_recovery/executor_isolation.json). The motion
   guard still checks dimensions, data length, resolution and timestamp; full
   occupancy counts remain in readiness, and full path collision checks remain.
   Native and external TF were healthy during the verifier failure.

Release sources inspected, with installed package versions and checksums in
[dependency audit](evidence/m4_recovery/dependency_audit.txt):

- [TF2 0.25.23 BufferCore](https://github.com/ros2/geometry2/blob/0.25.23/tf2/src/buffer_core.cpp),
  [Buffer mutex order](https://github.com/ros2/geometry2/blob/0.25.23/tf2_ros/src/buffer.cpp).
- [Unmodified TF2 0.25.24 fix](https://github.com/ros2/geometry2/blob/404b7224d623d614f18fa9738dbf1716403d857e/tf2/src/buffer_core.cpp),
  [release changelog, upstream #990/#982](https://github.com/ros2/geometry2/blob/0.25.24/tf2/CHANGELOG.rst).
- [Nav2 1.1.20 costmap publisher](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/src/costmap_2d_publisher.cpp).

### Controlled experiments

All critical nodes use `rmw_cyclonedds_cpp`, ROS 2 Humble, Ubuntu 22.04.5 in the
existing container. Docker host management is unavailable in this shell and was
unnecessary. The real saved warehouse map and Standard robot were reused.

| Experiment | Observation / result |
| --- | --- |
| A: simulator + AMCL | First 60 s without Nav2: advancing clock, scan, odom and both dynamic TF edges; active Map Server/AMCL; stationary ground-truth error 0.0304 m. |
| B: native Nav2, no assignment probe or RViz | TF freeze within seconds. At 115 s observation clock is 252.312 s, footprints remain 140.343 s. Native lifecycle/parameter services remain responsive. |
| C: enable assignment probe | Same frozen native stamps; all 138 independent service requests succeed. Probe reports the misleading lifecycle-manager timeout at its 45 s deadline. |
| D: RViz light / normal | Both 30 s comparisons retain the already reproduced native freeze; no new service failures. RViz is unnecessary to trigger it. |
| Fixed native Nav2, same simulator/AMCL | 120 s: advancing footprints, external TF and raw headers; no service failures; max service latency 0.00621 s. |
| Fixed assignment runner + normal RViz | Foreground runner reaches READY using the persistent cache overlay; 120 s observation has zero service errors (max 0.00627 s latency), advancing footprints and TF, including normal RViz displays. |

Timing caveat: A's file records 90 s, including the last roughly 25 s after native
Nav2 startup; use its first 60 s for the AMCL-only comparison. B records 120 s;
C starts during its last roughly 20 s, so use B's first 90 s for the probe-free
comparison. The freeze predates C. D adds displays to the same stalled session;
it does not claim independent fresh launches for each display setting. These
boundaries are visible in monotonic/simulated timestamps and launch logs.
Middleware comparison was unnecessary once the native locking failure was
isolated. There were no intentional mixed-middleware critical nodes.

Process snapshots capture CPU, RSS, process and thread counts. The base had
roughly 1,039 threads across the container; Gazebo server/GUI used about 103%/55%
CPU in the initial snapshot, with about 600/911 MB RSS. No cgroup CPU throttling
was reported, and memory was available. Clock/wall progression remained near
1:1. These observations do not establish a universal resource performance bound.

## Correction and reproducibility

`/opt/ros/humble` still contains TF2 0.25.23; APT's candidate is the same version.
The correction builds **unmodified upstream TF2 0.25.24** at pinned commit
`404b7224d623d614f18fa9738dbf1716403d857e` outside the workspace and vendor trees.
Only package `tf2` is built. Installed `tf2_ros`, Nav2, TurtleBot, simulator and
middleware packages are not patched. The initial native comparison used
`/tmp/m4_tf2_install`; the durable runner replay uses the same source built in
`${XDG_CACHE_HOME:-$HOME/.cache}/turtlebot4/tf2-0.25.24`. Runtime `/proc` mappings
confirmed the controller/planner loaded the overlay library.

Changed assignment files:

- `scripts/prepare_tf2_runtime.sh`: bounded fetch, exact revision/clean-source
  checks, bounded separate dependency build and explicit shell sourcing command.
- `turtlebot4/navigation_diagnostics.py`: compatible timestamp selection,
  constant-time motion guard, deadline/error distinction and pending cleanup.
- `scripts/verify_navigation.py`: use the constant-time guard; preserve the
  0.5 s moving TF limit, monitor footprint freshness, record TF/costmap/footprint
  ages and monotonic timestamps with motion observations.
- `test/test_m4_recovery.py`, `test/tf2_callback_reentry.cpp`: seven regressions,
  including compiling and executing the actually loaded native TF2 library.
- `package.xml`: direct native TF2 test dependency.
- Vietnamese README, M4 report recovery section, this report and real evidence.

`config/nav2.yaml` is unchanged, including `GridBased.allow_unknown: false`,
zero planner tolerance, unknown-space tracking, collision layers, robot radius,
controller speeds and goal tolerances. The map PGM SHA-256 remains
`9bdc7c7095f17db56a5112e9c0bae857d9d8f29e9db757e9b4313410761b2dbc`.

Run in the configured ROS container:

```bash
cd /root/ros2_ws
bash src/turtlebot4/scripts/prepare_tf2_runtime.sh
# Every new terminal, before launching its owned process:
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
source "${XDG_CACHE_HOME:-$HOME/.cache}/turtlebot4/tf2-0.25.24/install/setup.bash"
```

The overlay must be sourced **before process creation**. Sourcing a new terminal
cannot change libraries already loaded into an old process. Generated dependency
source/build/install/log files are outside the assignment checkout and uncommitted.

## First gate, motion, unknown-space safety and goals

Before goals, native healthy observation lasted 120 wall seconds, with clock
565.389 → 684.873 s. Local footprint stamps advanced 565.536 → 684.810 s;
global stamps advanced 566.892 → 684.375 s. All actual lifecycle requests returned
ACTIVE, all requested sim-time parameters were true, and services were responsive.
The corrected readiness verified the NavigateToPose server, runtime unknown-space
parameters, command interfaces, map contents and live footprint/TF consistency.

AMCL only publishes a new retained `/amcl_pose` when its filter updates; a
stationary retained pose timestamp is not a continuously published heartbeat.
Active scan-driven `map -> odom` rebroadcasts and composed TF remain fresh.
Moving localization was separately measured against real ground truth.
Undock SUCCEEDED (status 4), dock status became false, and a bounded 0.08 m/s
segment stopped. Maximum localization error was 0.1158 m and yaw error 0.0501 rad;
observed hazard type 0 is BACKUP_LIMIT, not bypassed. That manual phase ended and
its publisher was destroyed before any navigation action.

The successful verifier used actual planner actions to Unknown/Occupied cells:
both were ABORTED (6), with empty paths and raw costs 255/254 respectively. Goals
were chosen in original mapped Free space with >=0.55 m clearance; all published
path segments were checked at half-cell spacing against saved-map and costmap
collision/unknown cells. No teleop node/publisher competed during execution.

| Goal | Target (x,y m) | Final AMCL/TF (x,y m) | Action | Pose error m / rad | Ground-truth target distance m |
| --- | --- | --- | --- | --- | --- |
| A | (-2.175,-0.075) | (-1.949,-0.014) | SUCCEEDED (4) | 0.2344 / 0.1315 | 0.1272 |
| B | (-2.475,-1.575) | (-2.390,-1.356) | SUCCEEDED (4) | 0.2347 / 0.1305 | 0.2307 |
| C | (-0.625,-1.125) | (-0.863,-1.158) | SUCCEEDED (4) | 0.2399 / 0.0133 | 0.4343 |

Configured Nav2 tolerance is 0.25 m / 0.25 rad in the localization frame. Actual
start/target/final poses, full paths, action results, commands and sampled timing
are in [three_goals.json](evidence/m4_recovery/three_goals.json);
[summary.json](evidence/m4_recovery/summary.json) preserves compact measurements.

| Goal | Max localization error m | Max TF age s | Max global/local raw publication age s | Max global/local footprint age s | Actual displacement m |
| --- | ---: | ---: | --- | --- | ---: |
| A | 0.1713 | 0.141 | 1.989 / 0.597 | 1.011 / 0.213 | 1.317 |
| B | 0.2245 | 0.186 | 1.989 / 0.600 | 1.014 / 0.213 | 1.462 |
| C | 0.2285 | 0.186 | 1.989 / 0.600 | 1.008 / 0.216 | 1.542 |

Each goal recorded real nonzero controller commands, no prohibited hazard, zero
bumper contacts and stopped odometry (linear/angular magnitudes below 3e-15).
The 0.30 m localization cancellation bound was never exceeded. Contact/hazard
observations establish no recorded collision during this run, not a universal
collision guarantee. Native log entries independently report all three successes.

## Tests, shutdown and experiment limitations

Baseline: **109 passed**. Final package build succeeds; **116 tests, zero errors,
failures or skips**. The deterministic callback-reentry test hangs and is killed
by its three-second bound on installed 0.25.23, then passes on overlay 0.25.24.
Five diagnostic regressions and one constant-time guard regression also pass.
The native test requires the existing C++ compiler and Humble headers; no mock
pretends the native deadlock is fixed.

Executed validation commands:

```bash
timeout 150 python3 -m pytest src/turtlebot4/test -q  # initial 109 baseline
# Negative native test, with original /opt/ros environment: expected FAIL
timeout 50 python3 -m pytest src/turtlebot4/test/test_m4_recovery.py -q -k native_tf2
# With the fixed dependency overlay sourced:
timeout 90 colcon build --symlink-install --packages-select turtlebot4
source install/setup.bash
timeout 150 colcon test --packages-select turtlebot4 --event-handlers console_direct+
colcon test-result --test-result-base build/turtlebot4 --verbose

# Owned foreground sessions (base, localization, native comparison, then runner):
timeout --signal=INT --kill-after=20s 3600s ros2 launch turtlebot4 turtlebot4.launch.py rviz:=false
timeout --signal=INT --kill-after=15s 3300s ros2 run turtlebot4 start_localization --timeout 30
# Fresh spawn estimate only after ground-truth confirmation:
timeout 45 python3 src/turtlebot4/docs/evidence/m3_5/localization_probe.py
# Native B and fixed-native sessions use this command under different TF2 overlays:
ros2 launch turtlebot4_navigation nav2.launch.py use_sim_time:=true params_file:=/root/ros2_ws/src/turtlebot4/config/nav2.yaml
# Bounded observations (duration selected per experiment):
timeout 140 python3 /tmp/m4_recovery_observe.py --duration 120 --nav --output src/turtlebot4/docs/evidence/m4_recovery/fixed_native.json
# The two lines below run from /root/ros2_ws:
timeout 75 python3 /tmp/m4_recovery_motion.py
timeout --signal=INT --kill-after=15s 370s python3 src/turtlebot4/scripts/verify_navigation.py --output src/turtlebot4/docs/evidence/m4_recovery/three_goals.json
# Persistent cache-overlay replay:
timeout --signal=INT --kill-after=20s 240s ros2 run turtlebot4 start_navigation --timeout 45
```

Evidence scripts reproduce measurements; their default absolute output paths
reflect this verified workspace. The debug launch is an experimental copy under
`/tmp`, adding a controller GDB prefix and scan remaps only; it is not the product
navigation launch. GDB was installed for thread inspection. Attaching to existing
processes was blocked; launching the controller as GDB's child worked. That
instrumented trial intentionally kills its owned controller after the stack
capture and is excluded from acceptance. The middleware and vendor binaries
were not replaced globally.

A verifier starvation attempt was cancelled safely before the successful run.
The first runner replay rejected two identically named independent observers;
renaming the experimental observers allowed the subsequent replay. Both failures
are retained, not discarded. A/B/C did not use manual velocity commands.
No new screenshots/video, full SLAM handoff replay or second simulator restart
acceptance are claimed. Initial fresh-spawn localization is not a substitute for
that separate restart demonstration.

## Final matrix

| Criterion | Result | Scope |
| --- | --- | --- |
| Native TF2 root cause isolated | PASS | Native thread trace, matching source and deterministic before/after regression |
| Sustained healthy native TF/costmap geometry | PASS | 120 s fixed native observation, healthy motion samples |
| Lifecycle services / ACTIVE / sim time | PASS | Independent bounded replies and corrected readiness |
| Runtime unknown-space and collision safety | PASS | Actual parameters, raw unknown cells and rejected unsafe planner goals |
| Moving AMCL localization safety bound | PASS | Max 0.2285 m during goals, below existing 0.30 m cancellation limit |
| Three distinct autonomous successes and stops | PASS | Real status 4, observed paths, commands, displacement and stops |
| Configured Nav2 pose tolerances | PASS | All within 0.25 m / 0.25 rad in AMCL frame |
| Physical target distance <=0.25 m for all goals | FAIL | Goal C ground-truth distance 0.4343 m; localization accuracy limits remain |
| Recorded collision-free completion | PASS | No bumper contacts/prohibited hazards in this run |
| All prior tests + focused native regression | PASS | 116 total, all 109 existing preserved |
| Foreground runner with durable dependency | PASS | READY and owned-stack replay |
| Owned process shutdown | PASS | Clean fixed Nav2 shutdown, then empty fresh graph and process audit |
| New complete restart/video demonstration | NOT VERIFIED | Not replayed; earlier evidence remains historical |
| M4 navigation runtime recovery | **PASS** | Three real successes with healthy timing and bounded localization |

If an acceptance rubric requires **physical** 0.25 m target accuracy (rather than
Nav2's configured localization-frame goal checker), that stricter criterion is
**FAIL**, and this report must not be used to claim it passed. Additional map/
localization accuracy work is outside this TF/costmap recovery correction.

## Git and remaining limitations

Implementation commit and remote verification are recorded after committing the
verified changes below. Only `assignments_4` and assignment-owned files are staged.
No build/install/log directories, external dependency code, secrets, UR work,
skills or other branches are included. No force push.

Cleanup: the bounded runner stopped all owned Nav2 children cleanly, leaving
localization/base alive. Explicit SIGINT then stopped localization and base. The upstream TurtleBot
node required the launch system's bounded SIGTERM escalation; no orphan remained.
The final graph/process audit is in `shutdown_nodes.txt` and
`shutdown_processes.txt`; the fresh `ros2 node list --no-daemon --spin-time 3` graph is empty and no owned
Gazebo/Nav2/AMCL processes remain. The initial daemon query retained stale node
names, preserved separately in `shutdown_nodes_cached.txt`.

Verified implementation commit: `3bf26cc671d5c673f15ecbb480e2325161ea15d4`.
Message: `fix(assignment4): recover Nav2 TF and costmap runtime stability`.
`git push origin HEAD:assignments_4` succeeded (`39122e1..3bf26cc`).
Independent `git ls-remote origin refs/heads/assignments_4` returned the same full
SHA. This final Git-result documentation follows in a separate commit, whose SHA
is available with `git log -1 --format=%H -- docs/M4_RECOVERY_REPORT.md`.
Trailing blank lines in two new endpoint captures were normalized in that
follow-up; no measured values changed. No source changes followed final tests.
