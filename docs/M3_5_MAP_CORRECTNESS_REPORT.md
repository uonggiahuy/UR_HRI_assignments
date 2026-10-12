# M3.5 — Critical Map Correctness Patch

Date: 2026-10-12 (Asia/Bangkok). **PASS for map correctness, real Nav2
save/reload, package regression tests and stationary AMCL localization.**
Branch: `assignments_4`. Audited checkout baseline:
`9e1dafd47aaa5b57613f74563639cc8a4dd209d5` (see `environment.txt` for authoritative
full SHA); M3 implementation ancestor: `54c69736b0ef1ce70a3388a4754d7d9c5c79244c`.
No M4 implementation or robot velocity commands were performed.

## Root cause and independent reproduction

The original real warehouse PGM has 491 × 447 = 219,477 cells, with exactly
80,090 pixels at 254, 1,938 at 0 and 137,449 at 205. It is not a fully explored
map. M3's assertion that zero unknown cells meant a “fully resolved trinary map”
was incorrect; its recorded Map Server free count included every unknown cell.
Historical M2/M3 reports and evidence remain unchanged.

Installed `ros-humble-nav2-map-server` is
`1.1.20-1jammy.20260908.000810`. Inspected installed `map_io.hpp`, `libmap_io.so`
interface and executable behavior, together with the matching upstream
[Nav2 1.1.20 map_io.cpp](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_map_server/src/map_io.cpp).
The implementation `.cpp` is not shipped in `/opt/ros/humble`; the release source
was downloaded read-only to `/tmp/m35_map_io.cpp`, with checksum in the evidence.
The installed executable, rather than an emulated classifier, determined the
runtime acceptance results.

For non-negated trinary loading, Nav2 computes `occ = 1 - pixel/255` and tests
`occ > occupied_thresh`, then `occ < free_thresh`; otherwise the cell is Unknown.
Its saver encodes unknown/intermediate cells as pixel 205, free cells as 254 and
occupied cells as 0. Pixel 205 gives `occ = 50/255 ≈ 0.1960784314`, below the old
`free_thresh: 0.25`; the loader therefore publishes those cells as `0` (Free).
The saver writes its save thresholds into YAML, so its upstream 0.25 default
reproduces the defect on the next load. Changing mode or raster data is unnecessary.

Before editing, an actual namespaced Map Server was configured and activated
through lifecycle services and its retained OccupancyGrid was counted. The new
integration test repeats this with a temporary faulty YAML referencing a byte
copy of the original PGM, then loads the corrected repository YAML.

| Source / actual runtime stage | Free (`0`) | Occupied (`100`) | Unknown (`-1`) |
| --- | ---: | ---: | ---: |
| Original PGM pixels, intended classes | 80,090 | 1,938 | 137,449 |
| Actual Map Server with old YAML (`free_thresh: 0.25`) | 217,539 | 1,938 | 0 |
| Actual Map Server with corrected YAML (`free_thresh: 0.196`) | 80,090 | 1,938 | 137,449 |
| Actual Nav2 saver → new files → actual Map Server reload | 80,090 | 1,938 | 137,449 |
| Real warehouse simulation, active AMCL/Map Server | 80,090 | 1,938 | 137,449 |

The round-trip test compares **every cell in order**, dimensions and origin.
It saves only into a temporary test directory; it never replaces the warehouse
map or generates fabricated occupancy data.

## Source changes

| File | Correction |
| --- | --- |
| `maps/map.yaml` | Change only free threshold to 0.196; retain trinary, negate 0, occupied threshold 0.65, image, origin and resolution. |
| `turtlebot4/map_saver.py` | Named safe defaults used by function and CLI; explicit upstream trinary saving; reject thresholds that misclassify 254/0/205 before capture; validate finite metadata, matching image, 8-bit PGM and loading semantics; provide class counts and ordered occupancy digest. Temporary saving cleanup now also covers subprocess timeouts. |
| `turtlebot4/localization_diagnostics.py` | Compare real loaded classes and ordered occupancy digest against the saved PGM, rejecting count-preserving pixel swaps too; follow the YAML image name; display all three occupancy counts. |
| `package.xml` | Declare `lifecycle_msgs` test dependency for actual lifecycle integration tests. |
| `test/test_m3_5.py` | 19 regression cases, including installed Nav2 executables and rollback injection. |
| `README.md` | Vietnamese threshold explanation, corrected occupancy example, verification command, test count and historical M3 correction. |
| `docs/evidence/m3_5/` | Real runtime logs, executable probe, occupancy summaries, lifecycle/TF/ground-truth observations, build/test logs and shutdown evidence. |
| `docs/M3_5_MAP_CORRECTNESS_REPORT.md` | This audit and verification report. |

`localization_runner.py` needs no edit: its existing call to
`validate_saved_map_files()` now rejects unsafe map semantics before starting
localization. Transaction staging, backup, two `os.replace` calls and rollback
are preserved. Added failure tests exercise staging, second-file replacement and
post-validation failures, with and without an existing destination, asserting
byte-for-byte restoration and no leftover hidden staging/backup files.

Original `maps/map.pgm` SHA-256 before and after:
`9bdc7c7095f17db56a5112e9c0bae857d9d8f29e9db757e9b4313410761b2dbc`.
No PGM changes, vendor/upstream changes, UR changes, skill changes, other-branch
changes or 9Router changes are included.

## Exact commands and automated results

Executed in the existing Ubuntu 22.04.5 container shell with ROS 2 Humble,
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`; no host ROS installation or Docker
reconfiguration. Docker's authoritative container name is unavailable from this
shell (no Docker executable/socket); environment evidence establishes the actual
ROS installation and workspace used.

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon list
git -C src/turtlebot4 branch --show-current
git -C src/turtlebot4 status --short
ros2 pkg prefix nav2_map_server
ros2 node list
ros2 topic list -t
ros2 run nav2_map_server map_saver_cli --help

# Initial independent reproduction, before the source patch:
timeout 50 python3 /tmp/m35_probe.py
# Output: {"0": 217539, "100": 1938}

# Existing tests immediately after the source patch:
timeout 90 python3 -m pytest src/turtlebot4/test -q
# 64 passed (before new tests were added).

# Initial full regression + integration suite:
timeout 150 python3 -m pytest src/turtlebot4/test -q --basetemp=/tmp/m35_pytest
# 82 passed; subsequently added one timeout cleanup case.

# Final build and final suite:
timeout 90 colcon build --symlink-install --packages-select turtlebot4
source install/setup.bash
timeout 150 colcon test --packages-select turtlebot4 \
  --event-handlers console_direct+ --pytest-args -s
colcon test-result --test-result-base build/turtlebot4 --verbose
git -C src/turtlebot4 diff --check
sha256sum src/turtlebot4/maps/map.pgm
```

Final result: **83 tests, 0 errors, 0 failures, 0 skipped**. Build succeeded.
64 unchanged M1–M3 tests plus 19 M3.5 cases passed. The final suite includes
real namespaced Map Server configuration/activation/GetState checks, retained
OccupancyGrid capture, and real `map_saver_cli` saving and reloading. These are
not mocked integrations. Unit-only cases explicitly inject failures to verify
rollback and temporary-directory cleanup.

To reproduce the before/after and real saver round-trip without Gazebo:

```bash
timeout 150 python3 -m pytest src/turtlebot4/test/test_m3_5.py -v -s
```

The tests require the installed ROS/Nav2 dependencies, launch bounded private
Map Servers, stop their own process groups and never start a simulator.

## Actual AMCL simulation verification

Executed commands (long-running commands owned separate command sessions):

```bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
# Persistent base; fresh warehouse, Standard robot, RViz disabled for this audit:
timeout --signal=INT --kill-after=20s 600s \
  ros2 launch turtlebot4 turtlebot4.launch.py rviz:=false
# Independent localization, no SLAM:
timeout --signal=INT --kill-after=15s 360s \
  ros2 run turtlebot4 start_localization --timeout 30

# Separate read-only observations:
timeout 12 ros2 lifecycle get /amcl
timeout 12 ros2 lifecycle get /map_server
timeout 12 ros2 topic echo /sim_ground_truth_pose --once --field pose.pose
timeout 12 ros2 topic echo /odom --once --field pose.pose

# Bounded independent probe; publishes only the verified spawn initial estimate:
timeout 50 python3 src/turtlebot4/docs/evidence/m3_5/localization_probe.py
# Assignment diagnostics after initialization:
timeout 35 ros2 run turtlebot4 check_localization --timeout 15 --json
```

Before publishing `/initialpose`, the independent probe confirmed real Gazebo
ground truth within 3 cm and 0.03 rad of the documented `(0,0,0)` spawn. It used a
current simulated timestamp and ordinary AMCL initial-estimate covariance.
No fake TF, teleportation, fabricated localization pose or velocity commands
were used. It observed ten seconds of actual sensor/TF traffic afterward.

Actual measured results in [localization.json](evidence/m3_5/localization.json):

- Lifecycle services: `map_server: active`, `amcl: active` (ID 3).
  This independently checks lifecycle; the existing CLI's “active” node labels
  by themselves only establish node presence.
- Map classes exactly match the PGM, including all 137,449 Unknown cells.
- AMCL estimate: `x=-0.006 m`, `y=-0.030 m`, `yaw=0.003 rad` (rounded).
- Gazebo ground truth: `x=0.000516 m`, `y≈0 m`, `yaw≈0.000006 rad`.
- Position error: `0.030399 m`; yaw error: `0.002714 rad`.
- 3,340 clock samples, progressing from simulated 169.512 to 179.529 s.
- 626 real LiDAR samples and 622 odometry samples, with progressing timestamps.
- 625 fresh `map -> odom` samples, stamps advancing from 170.521 to 180.505 s;
  the ~1 s future offset is AMCL's configured transform tolerance.
- TF lookup succeeds for `map -> odom`, `odom -> base_link` and `map -> base_link`;
  composed transform timestamp 179.526 s matches the progressing clock.
- No SLAM node; graph shows one AMCL instance. `/tf` publishers are AMCL and
  the normal robot/odometry/dock publishers, with no manually inserted map TF.
- Assignment `check_localization` returns `overall_pass: true` and the corrected
  class counts, matching the independent probe.

Localization wrapper PID `354444` received SIGINT, stopped its owned upstream
stack cleanly, and left the base simulator running. Base launch PID `352085`
then received SIGINT and completed its normal cleanup. Final `ros2 node list`
is empty; no owned Map Server, AMCL or Ignition processes remained. See logs and
[shutdown_nodes.txt](evidence/m3_5/shutdown_nodes.txt).

## Acceptance matrix and remaining limitations

| Criterion | Result | Evidence |
| --- | --- | --- |
| Reproduce unknown-to-free defect on installed Map Server | PASS | Before executable capture and repeated integration assertion |
| Original warehouse PGM preserved | PASS | Same SHA-256 and pixel histogram |
| Corrected loader preserves all classes and cell locations | PASS | Actual Map Server counts and digest comparison |
| Safe future saving defaults and real reloading | PASS | Actual Nav2 saver round-trip, every cell equal |
| Unsafe YAML/thresholds rejected | PASS | Regression tests and runner's existing validation path |
| Transaction rollback and timeout cleanup | PASS | Failure injection at three boundaries, absent/existing destinations, timeout case |
| Existing behavior/test baseline preserved | PASS | All 64 unchanged M1–M3 tests |
| Build / final test suite | PASS | 83 tests, zero errors/failures/skips |
| AMCL lifecycle and fresh complete TF chain | PASS | GetState services, live sensor/TF probe and ground truth |
| Clean stack stop with simulator persistence, then base stop | PASS | Real shutdown logs and empty final graph |
| Moving AMCL convergence with corrected map | NOT VERIFIED | Stationary audit; no motion requested for this check |
| Complete SLAM-to-AMCL continuous handoff replay | NOT VERIFIED | M3.5 used the preserved saved map with a fresh simulator |
| RViz alignment and full warehouse coverage | NOT VERIFIED | No new visual/manual coverage audit; unknown area intentionally remains |
| Nav2 autonomous goals / M4 | NOT VERIFIED | Outside M3.5 scope; no implementation |

Trinary saving intentionally quantizes intermediate occupancy probabilities;
the exact preservation test applies to this real `-1/0/100` warehouse map.
The supported validation path is 8-bit binary PGM with trinary semantics.
Two atomic renames do not make the pair crash-atomic: concurrent readers and
power-loss recovery remain outside the existing rollback guarantee. Rollback
requires restoration I/O to remain available; persistent disk failure cannot be
proven recoverable by a simulated one-shot failure. None of these limitations
justify changing Unknown cells to Free.

The simulator logged existing optional camera/LiDAR-stop service-unavailable
messages and a transient AMCL startup scan-cache warning. LiDAR, odometry,
lifecycle and TF checks subsequently passed; upstream packages were not patched.

## Git result

- Verified implementation commit: `ebd889b5c7a8cafbed79f85101130b15af98aed5`.
- Exact message: `fix(assignment4): preserve unknown occupancy cells on map reload`.
- `git push origin HEAD:assignments_4` succeeded without force:
  `54c6973..ebd889b HEAD -> assignments_4`.
- Independent `git ls-remote origin refs/heads/assignments_4` returned
  `ebd889b5c7a8cafbed79f85101130b15af98aed5`, matching the local implementation commit.
- This report is delivered in a separate documentation commit after the verified
  implementation. Its SHA is discoverable with
  `git log -1 --format=%H -- docs/M3_5_MAP_CORRECTNESS_REPORT.md`.
- Only assignment-owned source, YAML metadata, tests, README and evidence are
  committed. Workspace build/install/log directories and unrelated dirty
  workspace files were not staged. M3.5 stops here; M4 is untouched.

