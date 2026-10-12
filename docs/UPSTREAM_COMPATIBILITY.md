# Installed Humble upstream compatibility audit

Audit date: 2026-10-12. All four required upstream package prefixes are
`/opt/ros/humble`. Package share roots are `/opt/ros/humble/share/<package>`.
This audit reads installed sources; it does not rely on a remote default branch.
Exact `ros2 launch <package> <file> --show-args` output is saved in `audit/`.

## Launch interfaces

Files below are relative to the corresponding package share root.
“Direct” means declared in that launch file; additional arguments in CLI output
can originate from included launches and must not be mistaken for explicit forwarding.

| Package / file | Direct arguments and defaults |
| --- | --- |
| `turtlebot4_ignition_bringup/launch/turtlebot4_ignition.launch.py` | `namespace=''`, `rviz=false`, `world=warehouse`, `model=standard` (standard/lite), `x=0.0`, `y=0.0`, `z=0.0`, `yaw=0.0` |
| `turtlebot4_ignition_bringup/launch/turtlebot4_spawn.launch.py` | `rviz=false`, `use_sim_time=true`, `model=standard` (standard/lite), `namespace=''`, `localization=false`, `slam=false`, `nav2=false`, `x=0.0`, `y=0.0`, `z=0.0`, `yaw=0.0`, `param_file=<ignition share>/config/turtlebot4_node.yaml` |
| `turtlebot4_navigation/launch/slam.launch.py` | `use_sim_time=false`, `sync=true`, `namespace=''`, `params=<navigation share>/config/slam.yaml` |
| `turtlebot4_navigation/launch/localization.launch.py` | `use_sim_time=false`, `namespace=''`, `params=<navigation share>/config/localization.yaml`, `map=<navigation share>/maps/warehouse.yaml` |
| `turtlebot4_navigation/launch/nav2.launch.py` | `use_sim_time=false`, `params_file=<navigation share>/config/nav2.yaml`, `namespace=''` |

## Source findings

- Simulator top-level explicitly passes only `world` to `ignition.launch.py`,
  and `namespace`, `rviz`, `x`, `y`, `z`, `yaw` to the spawn launch. Its declared
  `model` is not explicitly passed to spawn. `slam`, `nav2`, `localization`,
  `map` and `use_sim_time` are not explicitly forwarded there.
- CLI launch configurations can be visible through unscoped includes. This
  explains the nested arguments in top-level `--show-args`; it is not evidence
  of a dedicated forwarding contract. Runtime behavior was not tested in M0.
- Spawn declares three stack toggles and conditionally includes their launches.
  It explicitly passes only `namespace` and `use_sim_time` into each stack.
  It does not explicitly pass the map or stack parameter filenames.
- SLAM selects synchronous or asynchronous SLAM Toolbox, rewrites YAML under
  the namespace, and remaps TF, scan and map topics. Its `params` argument has
  an inaccurate upstream description (“Robot namespace”); the source shows it
  is a parameter filename.
- Localization explicitly maps its `params` to Nav2's `params_file`, along with
  `map`, `namespace`, and `use_sim_time`. It includes
  `nav2_bringup/launch/localization_launch.py`.
- Nav2 uses an `OpaqueFunction` to include
  `nav2_bringup/launch/navigation_launch.py`. It explicitly passes simulation
  time, parameter file and a normalized namespace, forces composition off,
  and remaps both costmap scan topics. `--show-args` does not expand the
  opaque runtime include; only its three direct arguments are shown.
- Shared names (`params`, `params_file`, `namespace`) may interact across
  includes. A future wrapper should scope stack includes and pass arguments
  explicitly rather than depend on inherited launch configurations.

## Integration strategy for later milestones

M1 should first validate simulation alone and discover `/clock`, actual scan
and odometry topics, TF, model and namespace. No names are assumed from this audit.
Compose installed simulator/spawn launches; avoid copying their implementation.
For subsequent mapping, include `slam.launch.py` with explicit `params`, `sync`,
`namespace`, and `use_sim_time=true`. For saved-map navigation, include
`localization.launch.py` with explicit `map` and `params`, plus `nav2.launch.py`
with explicit `params_file`. Keep simulator stack toggles false when those
stacks are launched separately, and keep SLAM and AMCL mutually exclusive.
Resolve project resources from the package share directory and preserve map
YAML/image pairs. Verify lifecycle activation and a single `map -> odom`
publisher before autonomous navigation. M0 implements none of these wrappers.
