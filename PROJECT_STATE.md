# Assignment 03 — Final Project State

- Branch: assignments_3; package: ur3_perception_llm_control.
- Milestones: M0, M0.5, M1–M12 PASS; M13 PASS.
- Static/unit regression: 118/118 passed.
- Package build passed: colcon build --symlink-install --packages-select ur3_perception_llm_control.
- Diff integrity passed: git diff --check.

## Final occupied-zone showcase

A clean headless workcell using config/scene_m12_blocker.yaml ran the real 9Router plan:

    pick(blue_cube)
    place_temp(blue_cube)
    pick(red_cube)
    place(red_cube, zone_b)
    home()

Initial RGB had blue_cube=zone_b. TaskValidator, goal check, and stale-world check passed. Blue was physically released to a temporary table position, camera verified Zone B empty, then red was physically released in Zone B and verified by RGB.

- Final camera: red_cube=zone_b, blue_cube=table, other cubes=table, zone_b=red_cube, held_object=none.
- Final MoveIt: 5 WORLD, 0 ATTACHED, 0.000 mm synchronization error.
- Final physical grasp: detached; gripper open; HOME maximum joint error 0.000948 rad.

## M13 audits

- Active Assignment 03 uses PhysicalGraspManager and physical placement methods, not legacy GazeboAttachmentSynchronizer. Active transport performs zero direct Gazebo pose writes; legacy Assignment 02 diagnostics are unreachable from m12_scene_aware_execution.
- Public skills are exactly home, pick, place, and place_temp. LLM candidates pass exact-schema validation, goal simulation, and fresh camera/state-signature checks before SkillExecutor.
- Credential-pattern scan found no tracked API keys, tokens, credentials, dotenv files, or secret files. Runtime values remain only in NINEROUTER_BASE_URL, NINEROUTER_API_KEY, and NINEROUTER_MODEL.
- All three upstream Universal Robots repositories were clean; no vendor source was modified. No stale local package-name references were found.
- README is finalized for architecture, safety boundaries, environment variable names, build, launch, and showcase workflow.

## Known non-blocking warnings

- Humble MoveIt combined attach-diff warning despite verified authoritative state.
- Octomap missing-3D-sensor message.
- Gazebo plugin name/update-period warnings.
- RViz/move_group Ctrl-C shutdown faults only.
- A prior same-world DetachableJoint diagnostic can lack a new baseline event. Normal clean launch/showcase is reliable; use a clean workcell for that lifecycle.

Submission readiness: READY. M13 remains accepted; persistent interactive
runtime work is in progress and has not replaced the M13 result.
