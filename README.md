# Assignment 4 — TurtleBot 4

M0 provides an independent ROS 2 Humble `ament_python` package named
`turtlebot4`. It contains packaging and audit documentation only. SLAM,
localization, navigation, motion commands and simulator wrappers are deferred.
The local package is distinct from `turtlebot4_navigation` and
`turtlebot4_ignition_bringup`; no upstream repositories are vendored.

## Build (inside the ROS Docker container)

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select turtlebot4
source install/setup.bash
ros2 pkg prefix turtlebot4
ros2 pkg executables turtlebot4
```

The expected prefix is `/root/ros2_ws/install/turtlebot4`. There are no
executables in M0. If the workspace is mounted elsewhere, use its actual path.
Launch/config/map/RViz resources, including nested files, install beneath
`share/turtlebot4`. Add dependencies when corresponding code uses them; M0
requires only `ament_python` and Python setuptools. The installed simulator,
TurtleBot navigation, SLAM Toolbox and Nav2 are audited environment prerequisites.

## Documentation

- [M0 Engineering Report](docs/M0_ENGINEERING_REPORT.md)
- [Installed upstream compatibility audit](docs/UPSTREAM_COMPATIBILITY.md)
- `docs/audit/`: package inventory, exact launch argument output, and upstream hashes.

## Later integration

Compose installed upstream launches with explicit, scoped arguments. Keep the
simulator's built-in `slam`, `localization`, and `nav2` toggles disabled if separate
wrappers start those stacks. Mapping and AMCL localization must not both own
`map -> odom`. Set `use_sim_time:=true` consistently. Inspect the actual graph,
scan topics, TF tree and lifecycle state before planning or commanding motion.
See the compatibility audit for installed launch interfaces and forwarding caveats.

M0 stops here; no M1–M6 behavior is implemented.
