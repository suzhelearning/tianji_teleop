# Post-Calibration Teleoperation README Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make both quick-start READMEs direct users from completed arm calibration to either PICO-only validation or the one-command Tianji PICO teleoperation pipeline.

**Architecture:** Restructure only the post-calibration paragraphs in the existing quick-start sections. Keep detailed tmux, bridge, and DLS/Spark receiver information in the later teleoperation section.

**Tech Stack:** Markdown, shell command examples

## Global Constraints

- Update `README.md` and `README.zh-CN.md` symmetrically.
- Preserve the calibration order `TCP -> wrist pivot -> arm geometry` / `TCP → 掌心到手腕 → 上臂/前臂骨长`.
- Use `./scripts/start_pico_m0.sh --viewer` only for PICO-only validation.
- Use `./scripts/start_tianji_pico_teleop.sh` and `./scripts/stop_tianji_pico_teleop.sh` for Tianji teleoperation.
- State that the launcher starts driver, M0 Viewer, and TJVR bridge and cleans historical instances.

---

### Task 1: Update Post-Calibration Quick Start

**Files:**
- Modify: `README.md`
- Modify: `README.zh-CN.md`

**Interfaces:**
- Consumes: existing calibration and launcher scripts
- Produces: bilingual post-calibration command guidance

- [ ] **Step 1: Rewrite the Chinese post-calibration paragraph**

After `./scripts/calibrate_pico_arm.sh status`, label `./scripts/start_pico_m0.sh --viewer` as PICO-only validation. Add a Tianji teleoperation paragraph requiring both sides to complete `TCP → 掌心到手腕 → 上臂/前臂骨长`, followed by:

```bash
./scripts/start_tianji_pico_teleop.sh
```

Then state that the launcher starts driver, M0 Viewer, and TJVR bridge, cleans historical instances, and stops with:

```bash
./scripts/stop_tianji_pico_teleop.sh
```

- [ ] **Step 2: Mirror the English guidance**

Use the same structure and commands with `TCP -> wrist pivot -> arm geometry`, explicitly distinguishing PICO-only validation from Tianji teleoperation.

- [ ] **Step 3: Verify bilingual command coverage and Markdown hygiene**

Run a Python assertion that both README quick-start prefixes contain `start_pico_m0.sh --viewer`, `start_tianji_pico_teleop.sh`, and `stop_tianji_pico_teleop.sh`, then run `git diff --check`.

- [ ] **Step 4: Commit**

Commit both README files with message `docs: clarify post-calibration teleop startup`.
