# M1 observation evidence

These are measured M1 observations, not simulator inputs. All ROS observations
were made in the existing Humble container. Screenshots contain only the Gazebo
and RViz application windows. `default_runtime.json` is the final diagnostic
run using the exact default launch command; screenshots and motion evidence
are from the same wrapper with RViz enabled.

`controlled_motion_test.py.txt` archives the exact one-off bounded harness used
after visually verifying the warehouse and current LiDAR clearance. It is not
an installed executable. **Do not execute it automatically or without rechecking
that the live graph belongs only to a safe simulation.** It includes bounded
undock handling, fresh sensor/clock checks, directional clearance, physical-hazard
checks, a finite command window, zero commands in cleanup and stop verification.
The numeric hazard value 0 is upstream BACKUP_LIMIT, allowed only for forward
motion; other hazards abort. The earlier reverse attempt was stopped by this
safety flag and is recorded separately.

`simulation_time_parameters.json` records an embedded Gazebo control logger node
with `use_sim_time=false` and a private TF listener with no parameter response.
All queried externally launched nodes and controllers reported true. Sensor/odom
stamp alignment with simulation time was independently verified.

Historical failed iterations are described in the engineering report. These
files describe the verified implementation unless explicitly named as a guard
or absent-simulator case. Frequencies and poses must not be reused as constants.
