# PICO Interactive Arm Calibration Implementation Plan

> **For agentic workers:** Execute inline in this session. Do not commit; preserve existing user changes and recordings.

**Goal:** Provide one interactive PICO calibration entrypoint supporting arbitrary single-item calibration and ordered new-operator calibration, while porting the palm-to-wrist calibrator.

**Architecture:** Keep existing TCP and arm-geometry solvers unchanged. Add the wrist calibrator and a thin menu/orchestration layer in `scripts/calibrate_pico_arm.sh`; the menu validates side-specific artifacts before each dependent operation and atomically activates only accepted geometry candidates.

**Tech Stack:** Bash, Python 3.11, ROS 2 Humble, `rclpy`, NumPy, PyYAML, Pixi, pytest, colcon.

## Global Constraints

- Default `ROS_DOMAIN_ID=120` and `ROS_LOCALHOST_ONLY=1`; explicit environment overrides remain honored.
- TCP, wrist pivot, and arm geometry artifacts are side-specific; never mirror left to right.
- `wrist` requires valid same-side TCP; `geometry` requires valid same-side TCP and wrist pivot.
- `all` runs exactly TCP → wrist → geometry and stops on first failure.
- Existing `recordings/` are read-only for activation; no deletion or rewriting of recordings.
- No exoskeleton repository changes and no commit.

### Task 1: Port wrist pivot solver and tests

**Files:**
- Create: `src/pico_bridge/scripts/pico_palm_wrist_calibrator.py`
- Create: `src/pico_bridge/test/test_pico_palm_wrist_calibrator.py`
- Modify: `src/pico_bridge/CMakeLists.txt`

**Steps:**

- [x] Copy the approved calibrator and its yaw/full mathematical tests from the sibling `pico_project` worktree, preserving `wrist_to_palm` schema and `p_palm = p_wrist + R_palm*r` semantics.
- [x] Install the executable as `pico_palm_wrist_calibrator` and register its pytest file.
- [x] Run `pixi run python -m pytest -q src/pico_bridge/test/test_pico_palm_wrist_calibrator.py` and require all tests pass.

### Task 2: Add artifact validators and operation dispatcher

**Files:**
- Modify: `scripts/calibrate_pico_arm.sh`
- Test: `src/pico_bridge/test/test_pico_calibration_menu.py`

**Interfaces:**
- `./scripts/calibrate_pico_arm.sh <left|right> <tcp|wrist|geometry|all>`
- `./scripts/calibrate_pico_arm.sh status`
- no-argument interactive menu using stdin/stdout.

**Steps:**

- [x] Add failing tests for argument parsing, dependency rejection, and output paths.
- [x] Make `valid_tcp`, `valid_wrist`, and `valid_geometry` validate YAML type, side, validity, and required fields.
- [x] Dispatch `tcp` to `ros2 run pico_bridge pico_palm_tcp_calibrator`.
- [x] Dispatch `wrist` to `ros2 run pico_bridge pico_palm_wrist_calibrator`, after TCP validation.
- [x] Dispatch `geometry` to existing `pico_arm_geometry_pixi.sh`, after TCP and wrist validation, and atomically activate accepted v3 candidate.
- [x] Make each operation return nonzero on failure and preserve prior artifacts.
- [x] Run menu tests and Bash syntax checks.

### Task 3: Add interactive menus and ordered all mode

**Files:**
- Modify: `scripts/calibrate_pico_arm.sh`
- Test: `src/pico_bridge/test/test_pico_calibration_menu.py`

**Steps:**

- [x] Add no-argument menu choices: single item, new operator complete adaptation, status, exit.
- [x] Add side and operation prompts for single-item mode.
- [x] Add side prompt for `all`, calling the three dispatchers in strict TCP → wrist → geometry order.
- [x] Stop `all` immediately if any step fails; do not run later operations.
- [x] Implement read-only `status` output for both sides and three artifacts.
- [x] Run piped-input menu tests proving arbitrary single selection and ordered all mode.

### Task 4: Build and regression gate

**Files:** None beyond Tasks 1–3.

**Steps:**

- [x] Run `bash -n scripts/calibrate_pico_arm.sh scripts/start_pico_driver.sh scripts/start_pico_m0.sh`.
- [x] Run focused menu and wrist tests.
- [x] Run `pixi run colcon test --packages-select pico_bridge --event-handlers console_cohesion+`.
- [x] Run `pixi run colcon test-result --test-result-base build/pico_bridge --verbose` and require 0 errors/failures.
- [x] Run `git diff --check` and report changed files without committing.
