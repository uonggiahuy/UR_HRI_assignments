# Assignment 02 — Current Project State

- **Completed through**: M11
- **Next milestone**: M12
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

## M8 World State, Validation, and Execution

M8 adds no LLM or natural-language interface. It accepts only structured JSON
plans and calls the existing M7 `RobotSkills` public API.

Public APIs:

- `WorldState.from_scene_file(scene_file) -> WorldState` reads the configured
  cube poses from `config/scene.yaml`, initializes all cubes as `table`
  objects, and tracks `held_object`, `object_locations`, and `zone_occupancy`.
  Its `record_pick_success()` and `record_place_success()` transitions are
  called only after an M7 skill reports `SUCCESS`.
- `TaskValidator.validate(plan_input, world_state) -> ValidationResult`
  accepts a JSON string/bytes or dictionary. It requires the exact top-level
  `{"plan"}` schema, exact per-skill arguments, and only `home`, `pick`, and
  `place` over the configured M8 objects/zones. It simulates the supplied
  `WorldState` snapshot to reject obvious semantic errors before execution.
- `SkillExecutor(skills, world_state).execute(validation) -> ExecutionResult`
  accepts only an accepted validation result for the unchanged state revision,
  calls only `RobotSkills.home()`, `.pick()`, and `.place()`, stops at the first
  failing skill, and applies a world-state transition only on success.

`m8_validator_test` is a static, no-ROS/no-LLM executable covering strict
schema checks, low-level command rejection, semantic rejection, stop-on-failure,
state-update-after-success-only, and stale-plan rejection.

`m8_plan_test` is the live hard-coded M8 JSON integration executable. It
validates and executes pick(red_cube), place(red_cube, zone_b), home(), then
checks final MoveIt, Gazebo, WorldState, and controller state. It requires the
already-running M7 simulation/MoveIt stack.

`RobotSkills.pick()` first enters the configured collision-checked `home`
configuration internally. This is a task-internal transit, not an additional
LLM/M8 plan step: a fresh workcell starts with a straight elbow, for which the
approach planner can generate an invalid upper-arm/gripper collision. The M8
plan therefore remains exactly `pick`, `place`, `home` and is valid from a
clean workcell.

`m8_transition_diagnostic` compares the startup and HOME transitions using
the existing M7 `pick` skill. Live validation confirmed startup pick planning
fails from the stock straight-elbow posture, while HOME-to-pick completes the
Planning Scene `WORLD -> ATTACHED` transition. The full M8 live plan then
completed `WORLD -> ATTACHED -> WORLD`, with the Gazebo cube at `zone_b` and
all controllers active.

## M9 Student-ID Personalization

`ur3_llm_control.student_task` provides local, deterministic assignment
personalization without ROS, LLM, 9Router, MoveIt, or robot execution.

- `parse_student_id(student_id)` accepts only a non-empty ASCII digit string
  with at least two digits, preserving leading zeros, and returns the final
  two digits as `XX`.
- `compute_variant(student_id)` deterministically computes `P = XX mod 6`.
- `get_assignment_mapping(student_id)` returns the specified zone-to-object
  mapping. `get_object_zone_mapping(student_id)` provides its inverse for
  later planning.
- `resolve_student_task(runtime_student_id=None, config_path=None)` resolves
  identity with strict priority: a provided runtime ID, then
  `config/student_config.yaml`, then an explicit configuration error. A bad
  runtime ID never falls back silently. `student_name` is metadata only and
  never influences `P` or the mapping.

`student_task_test` is a static no-ROS/no-LLM executable covering all six
variants, runtime-over-config precedence, configuration fallback, malformed
and missing IDs, repeatability, and the inverse mapping.

## M10 9Router LLM Planner Integration

`ur3_llm_control.llm_planner.LLMPlanner` is the isolated natural-language
planning boundary. `LLMPlanner.from_environment()` reads the non-secret
endpoint fallback, timeout, and temperature from `config/llm.yaml`; it requires
`NINEROUTER_API_KEY` and `NINEROUTER_MODEL` from the environment, with no
defaults. It uses the OpenAI-compatible Chat Completions API at the configured
9Router endpoint and never logs credentials.

`plan(request, world_state) -> PlannerResult` sends the constrained system
prompt, accepts only one JSON-object response (raw or a single JSON fence), and
passes it unchanged to M8 `TaskValidator`. It returns an explicit status for
configuration, client/API, malformed-response, and validation failures; it
does not retain or reuse a prior plan. It has no dependency on RobotSkills,
MoveIt, gripper, controllers, SkillExecutor, or robot execution.

`llm_planner_test` covers the Vietnamese red-to-zone-B request, English
blue-to-zone-A request, home request, malformed/low-level rejection, missing
credential fail-closed behavior, configured-model use, and stale-plan
non-reuse. Its `--live` mode performs the three authenticated planner checks
against 9Router and validates them with M8 only; it performs no robot motion.

## M11 Natural-Language Robustness (implementation pending live verification)

M11 keeps the M10 boundary unchanged: user text is sent to `LLMPlanner`, its
single JSON candidate is passed unchanged to M8 `TaskValidator`, and no
executor, robot skill, MoveIt, gripper, controller, or robot motion component
is imported or called.

The concise planner prompt now explicitly normalizes Vietnamese, English, and
mixed color/object/zone wording into the existing public IDs, preserves an
explicit `home` request only when present, and instructs the model to emit an
empty plan for ambiguous, incomplete, unknown, or low-level requests. M8 then
rejects that candidate under its existing strict schema and semantic rules.

`llm_planner_test --live` is the authenticated 9Router robustness suite: five
Vietnamese red-to-zone-B paraphrases, five English blue-to-zone-A paraphrases,
two mixed-language transfers, Vietnamese and English home phrasings, two
multi-step transfer-plus-home requests, and ambiguous/unsafe rejection cases.
Every request starts from a fresh M8 `WorldState`; only accepted M8 semantic
plans are compared, so a failed response cannot reuse a previous plan. This is
planner-only validation and executes no robot action. Completion remains
pending until the required authenticated 9Router environment is provided and
the suite passes.
