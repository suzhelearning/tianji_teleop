# Model-State OTG + Cartesian Velocity Servo + Velocity QP Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Execute inline with test-driven development. Do not use subagents for this workspace.

**Goal:** Add an explicitly selectable model-state-only execution mode for the existing Cartesian OTG + Cartesian Velocity Servo + Velocity QP path without changing the existing QP formulation or legacy default behavior.

**Architecture:** Keep the existing `CartesianReferenceGenerator`, `cartesianReferenceServoTwist`, and 13-variable velocity QP unchanged. Add one controller-level state-source policy: legacy mode continues to let actual-reference tracking error scale/freeze commands and reject invalid actual joint positions; model-state-only mode computes and advances exclusively from `q_ref`, while actual state remains diagnostic-only. Expose the policy through optional YAML and viewer CLI flags so both modes can run from the same executable and benchmark assets.

**Tech Stack:** C++20, Eigen, yaml-cpp, MuJoCo, qpOASES, GoogleTest, CMake/CTest.

## Global Constraints

- Preserve the existing branch and dirty worktree.
- Do not change the velocity QP decision variable, objective, equality, joint bounds, solver, arm-angle null-space refinement, or outward constraint.
- Default behavior must remain legacy-compatible.
- Do not commit until the user explicitly requests a commit.
- Production changes require a failing regression test first.

---

### Task 1: Configurable model-state-only policy

**Files:**
- Modify: `include/tianji_qp_ik/config.hpp`
- Modify: `src/config.cpp`
- Modify: `tests/test_config.cpp`

**Interfaces:**
- Produces: `ControllerConfig::model_state_only` with default `false`.
- Consumes YAML: optional `controller.model_state_only` boolean.

- [ ] Add a config test asserting the existing PICO config defaults to `false` when the key is absent and a temporary YAML with `model_state_only: true` parses as `true`.
- [ ] Run the focused config test and verify it fails because the field does not exist.
- [ ] Add the boolean field and optional YAML parser.
- [ ] Re-run the focused config tests and verify they pass.

### Task 2: Velocity controller state-source gate

**Files:**
- Modify: `src/controller.cpp`
- Modify: `tests/test_hierarchical_controller.cpp`

**Interfaces:**
- Consumes: `QpIkConfig::controller.model_state_only`.
- Behavior in model-state-only mode:
  - `reference_scale = 1.0` regardless of `q_ref-q_actual`.
  - `reference_frozen = false` regardless of `q_ref-q_actual`.
  - actual-position validation does not reject the model command.
  - kinematics, Jacobian, bounds, QP, and integration continue using `q_ref` exactly as before.

- [ ] Add a regression test that creates a large artificial actual/reference mismatch, proves legacy mode freezes, and proves model-state-only mode continues accepting and advancing the model reference.
- [ ] Run the focused test and verify it fails because model-state-only mode is not implemented.
- [ ] Gate only actual-feedback scaling/freezing and actual-position rejection in `DualArmController::step`.
- [ ] Re-run controller tests and verify they pass.

### Task 3: Viewer A/B selection

**Files:**
- Modify: `apps/run_qp_ik_viewer.cpp`
- Modify: `tests/test_pico_teleop_session.cpp`
- Modify: `tests/test_pico_viewer_integration.py`

**Interfaces:**
- Produces CLI flags:
  - `--model-state-only` sets `config.controller.model_state_only = true`.
  - `--actual-feedback-control` sets it to `false`.
- Produces one startup line: `control_state_source=model_reference` or `control_state_source=actual_feedback_guarded`.

- [ ] Add parser/integration tests for both flags, mutual last-flag override behavior, help text, and startup reporting.
- [ ] Run focused viewer tests and verify they fail because the flags are unknown.
- [ ] Add the optional override to `Options`, parse both no-value flags before the value-required branch, apply it after loading YAML, and print the selected state source.
- [ ] Re-run viewer tests and verify they pass.

### Task 4: Fair-comparison commands and documentation

**Files:**
- Modify: `docs/otg_cartesian_velocity_servo_velocity_qp_model_state_route_v1.md`

**Interfaces:**
- Uses the same `config/qp_ik_pico_teleop.yaml` for both runs and changes only the CLI state-source flag, preventing copied YAML profiles from drifting.
- Keeps every existing config file unchanged for legacy comparison.

- [ ] Add exact live and offline A/B commands to the technical route document.
- [ ] Verify both commands report the intended `control_state_source` while loading the same YAML path.

### Task 5: Regression and algorithm comparison

**Files:**
- No production files.
- Output only under `benchmark_results/`; do not stage large artifacts.

**Interfaces:**
- Runs the existing deterministic Cartesian benchmark for Direct Velocity QP, OTG + Velocity QP, and OTG + Acceleration QP.
- Runs the current PICO trace validation/replay tools if available without modifying their source format.

- [ ] Build the project with the existing build directory.
- [ ] Run focused config, controller, viewer, OTG, QP, arm-angle, and outward tests.
- [ ] Run complete CTest and report the exact pass/fail count.
- [ ] Run deterministic circle comparison with the current `pico_outward` profile.
- [ ] Confirm legacy benchmark metrics have not regressed beyond deterministic numerical tolerance.
- [ ] Run a short headless PICO/model-state-only startup smoke test and verify startup reports `model_reference`.
- [ ] Review `git diff --check`, list only files changed by this task, and leave everything uncommitted.
