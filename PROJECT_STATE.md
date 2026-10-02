# Assignment 03 — Current Project State

- **Baseline inherited**: Final accepted Assignment 02 implementation through M12.
- **M0 audit**: PASS. The worktree started clean on `assignments_3`; recent history and the package contents document the accepted M12 baseline.
- **M0.5 package rename**: PASS. Python package, ROS package metadata, resource marker, entry points, runtime references, launch/xacro lookups, and documentation use the renamed package.
- **M1 five-block workcell**: PASS. The canonical Gazebo scene contains dynamic, collidable red, yellow, blue, green, and purple cubes. Green and purple spawn at (-0.12, 0.46, 0.3235) and (0.12, 0.46, 0.3235) m. All five settled on the table in the live workcell; the planning scene loaded them, one `move_group` started, and the joint-state, arm, and gripper controllers became active.
- **M2 fixed RGB camera and RViz zones**: PASS. A static overhead RGB camera at world pose (0, 0.38, 1.25) m and RPY (0, π/2, 0) observes all five cubes, three zones, and free table area. It publishes 640×480 RGB images at approximately 10 Hz on `/camera/image_raw`, with `/camera/camera_info`; both headers use `overhead_rgb_camera/camera_link/rgb`. `camera_test` received and converted a real frame through `cv_bridge` and OpenCV. RViz loads an assignment-local config subscribing to three scene-derived `/workcell/zone_markers` markers; zones remain visual-only and absent from MoveIt collision geometry. No depth camera or object detection exists. Ctrl-C still causes `move_group` and RViz exit code -11 during shutdown, after normal live operation.
- **M3 tabletop homography**: PASS. `config/camera_calibration.yaml` stores four noncollinear fixed tabletop-corner pixel/world correspondences on the `world` plane `z = 0.300 m`; OpenCV fits the image→world homography and its inverse. Image origin is top-left; +u maps toward world -y and +v toward world -x. The live RGB/CameraInfo diagnostic checked three fixed zone-center holdouts: mean XY error 0.194 mm, maximum 0.220 mm, with negligible interior round-trip error. There is no depth or object detection, and no runtime Gazebo object-pose input.
- **Legacy MSSV compatibility**: The P0..P5 mapping and public Assignment 02 pick/place domain retain only red, yellow, and blue. `WorldState` tracks all five physical blocks; the Basic red-to-zone-B plan remains valid.
- **Perception direction**: Fixed overhead RGB camera + OpenCV + planar homography. No RGB-D camera.
- **Current package**: `ur3_perception_llm_control`
- **Current branch**: `assignments_3`
- **Next milestone**: M4 OpenCV five-cube detection and localization.

Cube perception, occupied-zone handling, temporary placement, and physical-grasp changes have not been implemented yet.
