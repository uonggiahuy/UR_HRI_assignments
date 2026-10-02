# Assignment 03 — Current Project State

- Branch: `assignments_3`; ROS package: `ur3_perception_llm_control`; inherited baseline: final accepted Assignment 02.
- Milestones: M0 **PASS**, M0.5 **PASS**, M1 **PASS**, M2 **PASS**, M3 **PASS**, M4 **PASS**, M5 **PASS**, M6 **PASS**.
- Runtime perception authority: RGB image → OpenCV five-cube detection → calibrated cube-top XY → fresh, complete `PerceptionSnapshot` → one MoveIt Planning Scene diff for the movable WORLD cubes. No depth or runtime Gazebo movable-pose input.
- Fixed scene geometry supplies the table/zone support heights and cube dimensions. Cube center Z is support top plus half its height: 0.32250 m on the table, 0.32450 m on a zone. Collision-box orientation is deterministic identity because each block is an equal-edge cube; RGB does not measure orientation or Z.
- Attached safety: authoritative MoveIt attachments are inspected before updating; attached cube IDs are excluded from WORLD updates, and WORLD+ATTACHED duplicates or unexpected attachments are rejected. The returned authoritative scene must contain exactly one WORLD box for each non-attached cube, the expected dimensions/pose within 1 mm, unchanged static table/pedestal, and no zone collision objects.
- Live Layout A: five WORLD-only cubes at camera-derived XY, all at table support Z; maximum requested-to-authoritative MoveIt error 0.000 mm.
- Live Layout B: red/yellow/blue at table support Z; green in zone C and purple in zone B at zone support Z; maximum requested-to-authoritative MoveIt error 0.000 mm. Both runs used eight stable RGB frames, one `move_group`, active controllers, and the existing RViz zone-marker publisher/subscriber.
- Legacy Assignment 02 Planning Scene and M12 APIs remain available. No robot motion, new grasp implementation, LLM integration, temporary-position planning, or occupied-zone execution was added.
- Existing shutdown behavior: Ctrl-C can produce RViz and `move_group` exit code -11 after normal operation.
- Next milestone: **M7 physical Gazebo grasp without object set-pose following**.
