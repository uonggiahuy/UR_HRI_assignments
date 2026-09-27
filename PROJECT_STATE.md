# Assignment 02 — Current Project State

- **Completed through**: M7
- **Next milestone**: M8
- **Branch**: `assignments_2`

## M7 High-Level Robot Skills

`ur3_llm_control.robot_skills.RobotSkills` is the public, fail-closed layer
over the validated M4--M6 interfaces. It contains no LLM functionality and
does not duplicate MoveIt, gripper, Planning Scene, or Gazebo synchronization
implementation.

Public API:

- `home() -> SkillStatus`: plans and executes the validated `home` joint
  configuration in `config/robot_motion.yaml`.
- `pick(object_name) -> SkillStatus`: accepts only `red_cube`, `yellow_cube`,
  or `blue_cube`; executes open, approach, descend, close, Planning Scene
  attach, Gazebo attachment synchronization, and retreat.
- `place(object_name, zone_name) -> SkillStatus`: accepts those objects and
  only `zone_a`, `zone_b`, or `zone_c`; refreshes the authoritative attachment,
  moves above the YAML zone target and descends to the YAML-derived top-of-zone
  cube pose, releases/detaches there, opens, retreats, performs the final
  Gazebo pose write, and clears the temporary target-only contact exception.

Statuses are `SUCCESS`, `FAILED`, `INVALID_OBJECT`, `INVALID_ZONE`,
`PLANNING_FAILED`, and `EXECUTION_FAILED`. Every arm call consumes a new M4
plan/execution result; the skill returns on its first failed step and never
continues with a stale result.

`robot_skills_test` is the direct M7 integration executable for:
`home()`, `pick(red_cube)`, `place(red_cube, zone_b)`, and `home()`. It checks
the MoveIt WORLD -> ATTACHED -> WORLD lifecycle, final zone pose, Gazebo final
pose, unchanged non-target cubes, and active controllers.
