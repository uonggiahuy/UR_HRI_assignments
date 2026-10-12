# Assignment 4 — TurtleBot 4 simulation

M1 integrates the installed TurtleBot 4 Ignition simulator with read-only runtime
diagnostics. SLAM, AMCL, mapping and Nav2 are disabled; no M2–M6 behavior is provided.

## Environment and architecture

Run ROS commands inside the existing Ubuntu 22.04 / ROS 2 Humble container.
The verified workspace is `/root/ros2_ws`, backed by a host filesystem mount.
Host Docker identity/configured mounts remain a separate partial M0 audit item.
Do not install ROS on the Ubuntu 24.04 host or reinstall the working simulator.

The wrapper resolves upstream shares through `ament_index_python` and includes
`ignition.launch.py` once and `turtlebot4_spawn.launch.py` once in separate scopes.
It explicitly supplies world, model and simulation time, disables the three
navigation stacks, and propagates nonzero child exits through visible errors and
launch shutdown. It forwards shutdown to its own Gazebo descendants because the
installed upstream shell launcher does not reliably forward signals to them.

The installed description launch publishes default joint positions. A ROS
node-qualified remap sends `joint_state_publisher` output to
`joint_states_visualization`. Project config makes it consume measured wheel
states through a separate input alias and retain defaults only for passive
wheel-drop joints. Robot state publisher consumes that merged stream; sensors
still consume Gazebo's measured `joint_states`. This prevents conflicting wheel
positions while keeping the full robot model connected.

RViz uses the installed `rviz2` executable and project-owned
`rviz/simulation.rviz`: fixed frame `odom`, transient-local robot description,
best-effort scan and TF. The upstream RViz launch hardcodes a navigation config
with fixed frame `map`, which is unsuitable while localization is disabled.

Required installed packages include `turtlebot4_ignition_bringup` and its TurtleBot,
iRobot, Gazebo bridge/control dependencies, `rviz2`, `rclpy`, `launch`,
`launch_ros`, `ament_index_python`, `rosgraph_msgs`, `sensor_msgs`, `nav_msgs`
and `tf2_msgs`. No upstream source is copied or modified.

## Build and automated tests

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select turtlebot4
source install/setup.bash
colcon test --packages-select turtlebot4
colcon test-result --verbose
```

This container has an incompatible third-party AnyIO pytest plugin. Package
`pytest.ini` excludes only that plugin, which these synchronous tests do not
use. The standard `colcon test` command therefore runs the real assertions
without changing system packages or suppressing test failures.

Launch/config/map/RViz resources install recursively beneath `share/turtlebot4`.
The diagnostic is installed as `ros2 run turtlebot4 check_simulation`;
`scripts/check_simulation.py` is also available from the source tree.

## Launch (simulation only)

```bash
ros2 launch turtlebot4 simulation.launch.py
ros2 launch turtlebot4 simulation.launch.py rviz:=true
ros2 launch turtlebot4 simulation.launch.py --show-args
ros2 launch turtlebot4 simulation.launch.py world:=depot model:=lite namespace:=m1_robot rviz:=true
```

Public arguments: `world` (default `warehouse`, installed SDF worlds), `model`
(`standard`/`lite`, default `standard`), `namespace` (default empty), and `rviz`
(`true`/`false`, default `false`). Installed worlds are `warehouse`, `depot`,
and `maze`. Warehouse/Standard and Depot/Lite/namespaced combinations were
verified live. Maze is installed and its forwarding is tested, but was not
launched live. World resources may need time to load before spawn/bridges become ready.
Simulation time is fixed true. No stack toggles, map files or arbitrary Gazebo
command arguments are exposed. Start only one wrapper instance at a time.

## Read-only runtime verification

In a second sourced container terminal:

```bash
ros2 run turtlebot4 check_simulation --timeout 20 --expect-rviz --output /tmp/simulation_check.json
ros2 run turtlebot4 check_simulation --timeout 20 --namespace m1_robot --expect-rviz
ros2 topic list -t
ros2 topic info /scan --verbose
ros2 topic info /odom --verbose
ros2 topic info /cmd_vel --verbose
ros2 topic info /tf_static --verbose
ros2 control list_controllers
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link turtlebot4/rplidar_link/rplidar
```

Substitute the runtime namespace/frames from the diagnostic for namespaced runs.
The diagnostic has a total 1–120 second wall-clock bound. It discovers scan and
odometry by type, rejects ambiguous matches, records actual QoS/frame names and
received frequencies, checks finite data, advancing clock/stamps, simulation-time
alignment, TF topology/conflicts and duplicate node names. It never publishes
velocity, calls an action or changes simulator parameters. Exit code is 1 for a
failed check, 0 when machine-checkable checks pass; visual content is always
reported `MANUAL VERIFICATION REQUIRED` by this read-only tool.

`observed_wall_hz` describes delivery during observation; `observed_sim_hz` uses
message timestamps and may differ when Gazebo runs slower than wall time. A
paused simulator fails the clock progression check. Static TF uses reliable
transient-local QoS to receive retained transforms. Installed Humble rclpy lacks
publisher GIDs in subscription callbacks, so the tool detects duplicate children,
multiple parents, static/dynamic overlap, conflicting transform samples and cycles,
but cannot attribute identical copies from different publishers to a specific edge.
No `map -> odom` transform is required.

## Controlled teleoperation (simulation only)

Inspect the GUI, LiDAR clearance, hazard flags and command subscribers before
motion. In this installation `/cmd_vel` is `geometry_msgs/msg/Twist`, consumed
by `/motion_control` with best-effort/volatile QoS. The robot starts docked.
Use the supported action after confirming the undock path is clear:

```bash
ros2 action info /undock
ros2 interface show irobot_create_msgs/action/Undock
# Only after verifying a safe simulated scene:
timeout --signal=INT 30s ros2 action send_goal /undock irobot_create_msgs/action/Undock '{}'
```

If that observation times out, cancel/stop the action and verify stopped state
before any further test. The timeout ends the CLI, not necessarily the action.
Use the Gazebo HMI Stop control or publish zero velocity as appropriate.
The robot may turn while undocking. Reinspect current heading and clearance.
The upstream BACKUP_LIMIT flag prevents reversing after undocking; retain that
safety setting and choose a short forward test only when its path is clear.
Physical hazards (bump, cliff, stall, wheel drop, proximity) require stopping.

A finite forward command test, after those checks:

```bash
timeout --signal=INT 5s ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.05}}' --rate 10 --times 20
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{}' --rate 10 --times 5
ros2 topic echo /odom --once
```

Always send zero commands on completion or interruption and confirm odometry
velocity and pose are stable. For namespaced robots, prefix all topic/action
names. The Gazebo GUI also provides upstream teleop controls; reduce its default
maximum speeds before use. Do not use unlimited-duration command publication.
M1 observed a bounded 2-second forward command, expected displacement and a
stable stop; see the engineering report for exact evidence.

## Troubleshooting and shutdown

- Missing package: source Humble, build this package, then source `install/setup.bash`.
- No clock/scan/odom: check launch errors and wait for actual world/spawn/controller
  readiness; rerun diagnostics with an appropriate bounded observation window.
  Depot's first load was slower than Warehouse in this environment.
- RViz requires working DISPLAY/OpenGL access. Without it, launch with `rviz:=false`
  and record graphical verification as manual. The simulator GUI also needs display access.
- No motion while docked: inspect `/dock_status` and `/undock`; never override safety
  flags to make a test pass. BACKUP_LIMIT is directional and blocks reversing.
- Wheel TF conflict: verify `/joint_states` is published only by the simulation
  broadcaster and `/joint_states_visualization` merges its measured wheel positions with
  passive-joint defaults. Check the publisher's `source_list` and remaps.
- Upstream logs warn about controller update period versus Gazebo step size. The
  active controllers and odometry were verified; upstream settings remain intact.
- Child exit: read the first launch error. Unexpected nonzero exits cause visible
  wrapper shutdown; normal spawn/controller-loader exits do not stop simulation.

Before termination, stop teleoperation, send zero velocity and confirm the robot
is stopped. Press Ctrl+C in the wrapper terminal and wait for launch shutdown.
The wrapper signals its own Gazebo descendant processes and lets ROS launch stop
its child nodes. Check `ros2 node list` and `ps -eo pid,args` afterward; do not
start another simulator until the previous one is gone. Never use broad `pkill`
or terminate unrelated user processes. The installed TurtleBot node can require
ROS launch's SIGTERM escalation; this was observed and is documented in the report.

## Reports

- [M0 engineering report](docs/M0_ENGINEERING_REPORT.md)
- [Installed upstream compatibility audit](docs/UPSTREAM_COMPATIBILITY.md)
- [M1 engineering report](docs/M1_ENGINEERING_REPORT.md)

Stop after M1. Mapping, localization, Nav2 and autonomous navigation are deferred.
