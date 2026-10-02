# Assignment 03 — Current Project State

- Branch: `assignments_3`
- ROS package: `ur3_perception_llm_control`
- Inherited baseline: final accepted Assignment 02 implementation.
- M0 audit: **PASS**
- M0.5 package rename: **PASS**
- M1 five-block workcell: **PASS**
- M2 fixed overhead RGB camera and RViz zone markers: **PASS**
- M3 tabletop homography at world z = 0.300 m: **PASS**; fixed-reference mean 0.194 mm, max 0.220 mm.
- M4 five-cube RGB detection and XY localization: **PASS**. `config/perception.yaml` stores five measured HSV classes and geometry/stability limits. OpenCV segments and filters one square top-face component per color; missing or ambiguous components fail closed. The component centroid represents the cube top-face center. `CubeTopMapper` intersects the camera ray from the accepted tabletop homography with known cube-top z = 0.345 m, leaving M3 unchanged.
- M4 live validation: two distinct layouts, eight consecutive complete RGB frames each at 10.00 Hz, maximum XY spread 0.000 mm. Green on zone C and purple on zone B remained detectable without zone masks or visual changes. Test-only Gazebo settled poses yielded layout A mean/max 2.048/2.561 mm and layout B mean/max 1.447/1.804 mm; all five cubes passed in each layout. One `move_group`, active joint-state/arm/gripper controllers, and the RViz zone-marker subscription were observed.
- Runtime perception uses RGB, OpenCV, and fixed camera geometry only. No depth and no runtime movable-object Gazebo pose input. Saved Gazebo pose samples were used only in the M4 diagnostic as an error oracle. No camera-derived symbolic WorldState or zone occupancy is implemented yet.
- Existing shutdown behavior: Ctrl-C can produce RViz and `move_group` exit code -11 after otherwise normal operation.
- Next milestone: **M5 camera-derived symbolic WorldState and zone occupancy**.
