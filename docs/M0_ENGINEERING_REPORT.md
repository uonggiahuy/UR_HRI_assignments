# M0 Engineering Report — Assignment 4

Audit date: 2026-10-12 (Asia/Bangkok). **Software foundation passes; overall M0
acceptance remains partial because authoritative Docker identity and restart
persistence cannot be verified from this shell.** No M1–M6 implementation or
robot motion was performed. No packages were installed or reinstalled.

## 1. Environment and package audit

- Actual workspace: `/root/ros2_ws`; assignment repository:
  `/root/ros2_ws/src/turtlebot4`.
- Current execution environment: Ubuntu 22.04.5 LTS, `/opt/ros/humble/setup.bash`
  present, `ROS_DISTRO=humble`, `ROS_VERSION=2`, initial
  `AMENT_PREFIX_PATH=/opt/ros/humble`. ROS commands ran here inside the container.
- `/.dockerenv` and Docker overlay root establish container execution. Hostname:
  `CHOCONADYNEubuntu`. Expected container name `ros2_workspace` is supplied by
  workspace instructions, but cannot be independently confirmed here.
- `findmnt -T /root/ros2_ws` reports
  `/dev/nvme0n1p4[/choconadyne/ros2_ws]`, ext4, read/write. The workspace is
  backed by a host filesystem mount rather than the root overlay. Persistence
  across restarts is strongly indicated, but Docker's configured mounts and
  an actual restart were not inspected or tested.
- No `docker` executable or `/var/run/docker.sock` is available. No host control
  path was found. Docker container inventory, image, network mode and configured
  mounts remain unverified. No container or 9Router was reconfigured.
- Before creation, `ros2 pkg prefix turtlebot4` returned `Package not found`,
  and `colcon list` contained no `turtlebot4`: no initial name collision.
- `turtlebot4_ignition_bringup`, `turtlebot4_navigation`, `slam_toolbox`, and
  `nav2_bringup` are all discoverable at `/opt/ros/humble`, both before and after
  the build. Full initial package inventory: `audit/ros_packages.txt`.
- Existing UR description/driver/Gazebo repositories are clean on `humble`.
  `ur3_llm_control` and `ur_drawing` are clean on `assignments_2`;
  `ur3_perception_llm_control` is clean on `assignments_3`. The outer workspace
  repository is on `assignments_2` and initially reports the perception checkout
  as modified relative to its recorded gitlink. That pre-existing state is preserved.
- Colcon creates build/install/log artifacts in the outer workspace and updates
  its tracked log symlinks. These generated changes are not committed or pushed.

## 2. Final project structure

```text
turtlebot4/
├── .gitignore
├── LICENSE
├── README.md
├── package.xml
├── setup.py
├── setup.cfg
├── resource/turtlebot4
├── turtlebot4/__init__.py
├── launch/README.md
├── config/README.md
├── maps/README.md
├── rviz/README.md
├── scripts/verify_m0.py
└── docs/
    ├── M0_ENGINEERING_REPORT.md
    ├── UPSTREAM_COMPATIBILITY.md
    └── audit/
        ├── ros_packages.txt
        ├── upstream_sha256.json
        ├── inherited_files_removed.txt
        └── *.launch.py.args.txt (five files)
```

`ament_python` provides the index marker, manifest, importable module and resource
installation. All future files beneath launch/config/maps/rviz are installed
recursively with their relative directories preserved. Their README placeholders
are already verified in the installed share tree. No runnable node or launch is
needed at M0. Only `ament_python` is declared as a ROS build dependency; future
runtime dependencies will be declared when used. The validation script is a
source-tree developer check using the existing Humble development tools.

## 3. Files created, modified and removed

All files in the structure above are new Assignment 4 content. Matching filenames
from Assignment 3 were removed in a separate cleanup commit before replacement.
Cleanup removes 93 tracked inherited files only from this new checkout. Exact
paths: `audit/inherited_files_removed.txt`; removal diff is in cleanup commit
`962edb1`. This includes UR3 Python nodes/tests, launch/config/RViz, URDF/SRDF,
worlds, prompts and Assignment 3 packaging/docs. Existing Assignment 3 files at
`src/ur3_perception_llm_control` remain untouched. No upstream files were changed.

## 4. Commands and validation results

Initial audit executed `pwd`, `find . -maxdepth 2 -type f | sort | head -200`,
read `AGENTS.md` and `skills/ros2-general/SKILL.md`, checked ROS setup,
`colcon list`, `git status --short`, repository branches/remotes/status,
`findmnt -T /root/ros2_ws`, `findmnt -T /`, `/etc/os-release` and `/.dockerenv`.
`ros2 pkg list` and `ros2 pkg prefix` checked all required packages and collisions.

The following were run in the current ROS container:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
colcon list --packages-select turtlebot4
colcon build --symlink-install --packages-select turtlebot4
source install/setup.bash
ros2 pkg prefix turtlebot4
ros2 pkg executables turtlebot4
python3 src/turtlebot4/scripts/verify_m0.py
colcon test --packages-select turtlebot4
colcon test-result --verbose
```

Results: package selected as `ros.ament_python`; build succeeds (one package).
Prefix is `/root/ros2_ws/install/turtlebot4`. Executable list is empty by design.
Verification passes: manifest parsing, Python import, installed prefix, seven
installed assets, four unchanged upstream prefixes and 194 SHA-256 comparisons.
Colcon test succeeds with **0 tests, 0 errors, 0 failures, 0 skipped**; its
unittest summary appears on stderr but reports `OK`. This is packaging validation,
not SLAM/navigation runtime testing.

```bash
cd /root/ros2_ws/src/turtlebot4
python3 setup.py check
git diff --check
```

Both succeed. The first metadata check warned about a missing URL; the repository
URL was added, the check then passed without warnings, and the package rebuilt.

Each command below passed, and its output is saved under `audit/`:

```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py --show-args
ros2 launch turtlebot4_ignition_bringup turtlebot4_spawn.launch.py --show-args
ros2 launch turtlebot4_navigation slam.launch.py --show-args
ros2 launch turtlebot4_navigation localization.launch.py --show-args
ros2 launch turtlebot4_navigation nav2.launch.py --show-args
```

All five installed source files were also read. Simulation was not relaunched;
the user's working simulator was preserved.

## 5. Acceptance criteria

| Criterion | Result | Evidence |
| --- | --- | --- |
| Independent package at requested location | PASS | `src/turtlebot4`, ament_python |
| No package-name collision | PASS | initial index/list audit; unique colcon package |
| Docker/workspace paths verified | FAIL (partially verified) | workspace and host-backed mount verified; Docker identity/configured mounts/restart persistence unavailable |
| Builds and discoverable through ROS 2 | PASS | successful build and workspace prefix |
| Required upstream dependencies available | PASS | four package prefixes under `/opt/ros/humble` |
| No upstream source modified | PASS | 194 upstream share-file hashes unchanged; UR checkouts clean |
| Git branch isolation verified | PASS | separate clone on `assignments_4`; existing branches unchanged |
| Committed and pushed to `assignments_4` | PASS | foundation commit `18c67a8` pushed; remote head verified |

## 6. Git isolation, commits and push

`git ls-remote --heads https://github.com/uonggiahuy/UR_HRI_assignments.git`
confirmed `assignments_4` and `assignments_3` initially share
`bee3f40a877b3bc8abb193e12505020f7cf5eea5`.
A separate single-branch clone checked out `assignments_4` at the requested path.
Cleanup commit: `962edb1` (`chore(assignment4): remove inherited Assignment 3 package`).
Foundation commit: `18c67a8edd12862062c42ffc7abfc474374b89b5`
(`feat(assignment4): initialize TurtleBot4 ROS2 package`).
The initial HTTPS push failed because no HTTPS credentials were configured.
The clone's origin fetch URL remains HTTPS; origin push URL was set to the
SSH URL already used by existing assignment checkouts:
`git@github.com:uonggiahuy/UR_HRI_assignments.git`.
`git push -u origin assignments_4` succeeded through SSH, advancing the remote
from `bee3f40` to `18c67a8` without force. Remote branch head was independently
read back, and all three protected remote heads below were unchanged.
This finalized report is committed separately after the foundation push.
To obtain its containing commit SHA use `git rev-parse HEAD` from this checkout;
the final chat report also records it.
Only this checkout is staged; outer workspace artifacts are excluded.

Protected remote heads recorded before work:

- `assignment_1`: `5a1cf1d624e660121d4752d2617f82f61f290e17`
- `assignments_2`: `739f4483016947fb0efd8de23691e610d4318984`
- `assignments_3`: `bee3f40a877b3bc8abb193e12505020f7cf5eea5`

## 7. Upstream integration issues

See `UPSTREAM_COMPATIBILITY.md` for package locations, direct arguments/defaults,
exact CLI outputs and source forwarding findings. Nested `--show-args` output
is not a forwarding guarantee. Simulator top-level forwarding omits stack
switches, map and explicit model forwarding. Spawn passes only namespace/time
explicitly into the three stacks. SLAM/localization use `params`, Nav2 uses
`params_file`; shared launch configurations require careful scoping. Nav2's
opaque include is not expanded by `--show-args`. Navigation launches default to
wall time, so simulation wrappers must explicitly set simulation time.

## 8. Recommendations for M1

1. Complete read-only host checks with `docker ps -a` and
   `docker inspect ros2_workspace` (name, image, network mode and mounts), then
   verify the workspace mapping against the host. Do not restart a running
   simulator just for this audit without coordinating its interruption.
2. Validate simulator-only bringup, chosen robot model, namespace, `/clock`,
   actual scan/odometry topics, TF and lifecycle readiness before adding stacks.
3. Compose upstream launch files with explicit scoped arguments; retain project
   code/config in this package. Add only dependencies used by that implementation.
4. Keep later SLAM and AMCL modes exclusive and avoid duplicate stack bringup.

Stop at M0: these recommendations are not implemented.
