# Tianji PICO Default M0 Viewer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the Tianji PICO one-command launcher start the PICO M0 MuJoCo skeleton viewer by default.

**Architecture:** Keep the existing three-window tmux topology and pass the existing `--viewer` option through the `m0` window command. Preserve the current M0 child-process isolation and keep the receiving DLS/Spark robot viewer outside this launcher.

**Tech Stack:** Bash, tmux, Pixi, pytest, Markdown

## Global Constraints

- The default viewer is the PICO M0 skeleton viewer for `/pico/smpl_palm_corrected_ik` with raw SMPL overlay.
- Do not start `TJ_arm_control_DLS_IK` or `tianji_qp_ik_viewer`.
- Viewer failure must not terminate the M0 skeleton filter.
- Work directly on the current `main` branch as explicitly requested by the user.

---

### Task 1: Default M0 Viewer Startup

**Files:**
- Modify: `src/pico_bridge/test/test_pico_m0_scripts.py`
- Modify: `scripts/start_tianji_pico_teleop.sh`
- Modify: `README.md`
- Modify: `README.zh-CN.md`

**Interfaces:**
- Consumes: `start_pico_m0.sh --viewer`
- Produces: one-command launcher behavior that opens the M0 MuJoCo skeleton viewer by default

- [ ] **Step 1: Write the failing launcher contract assertion**

Add this assertion to `test_tianji_tmux_launcher_contract_starts_only_the_pico_side`:

```python
assert "start_pico_m0.sh --viewer" in source
```

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
pixi run pytest -q src/pico_bridge/test/test_pico_m0_scripts.py::test_tianji_tmux_launcher_contract_starts_only_the_pico_side
```

Expected: FAIL because the current M0 command does not contain `--viewer`.

- [ ] **Step 3: Implement the minimal launcher change**

Change the launcher command construction to:

```bash
m0_inner="cd $repo_quoted && $ros_environment && exec ./scripts/start_pico_m0.sh --viewer"
```

- [ ] **Step 4: Document the default viewer behavior**

Update the one-command startup sections of both README files to state that the PICO M0 skeleton viewer opens by default and that the receiving DLS/Spark robot viewer remains excluded.

- [ ] **Step 5: Verify GREEN and shell syntax**

Run:

```bash
pixi run pytest -q src/pico_bridge/test/test_pico_m0_scripts.py
bash -n scripts/start_tianji_pico_teleop.sh
git diff --check
```

Expected: all tests pass, Bash reports no syntax error, and the diff check is clean.

- [ ] **Step 6: Commit the implementation**

```bash
git add scripts/start_tianji_pico_teleop.sh src/pico_bridge/test/test_pico_m0_scripts.py README.md README.zh-CN.md
git commit -m "feat: open PICO M0 viewer by default"
```
