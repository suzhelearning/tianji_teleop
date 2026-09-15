# Headroom-Focused README Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the root README a concise operational guide for `spark_upper_qpoases_headroom_feedforward_velocity_qp` and move historical algorithm material to `docs/legacy_algorithms.md`.

**Architecture:** Rewrite README around the current no-argument PICO Viewer path rather than editing historical sections in place. Preserve useful legacy commands, formulas, and links in one explicitly non-default document, then validate paths, links, line count, and algorithm focus.

**Tech Stack:** Markdown, Bash command examples, CMake/Pixi project commands.

## Global Constraints

- README target length is 150–200 lines.
- The only recommended algorithm is `spark_upper_qpoases_headroom_feedforward_velocity_qp`.
- Current live and TJVR replay commands remain directly copyable.
- Historical algorithms remain accessible from `docs/legacy_algorithms.md`.
- No source, configuration, algorithm parameter, benchmark data, or test behavior changes.

---

### Task 1: Create the historical algorithm reference

**Files:**
- Create: `docs/legacy_algorithms.md`

**Interfaces:**
- Consumes: Historical sections of the current README.
- Produces: One non-default reference linked from the new README.

- [ ] **Step 1: Add scope and navigation**

State that the document covers historical regression and A/B paths, that the current recommended path lives in `../README.md`, and that none of its commands are the main default.

- [ ] **Step 2: Preserve historical commands and concepts**

Include hierarchical QP/DLS, Cartesian OTG velocity/acceleration, historical SPARK variants, benchmark commands, QP equations and constraints, full legacy Viewer controls, and verification links.

- [ ] **Step 3: Check required legacy topics**

Run:

```bash
rg -n 'hierarchical_qp|nullspace_dls|cartesian_otg|spark_guided_velocity_qp|spark_direct_velocity_qp' docs/legacy_algorithms.md
```

Expected: every historical topic is present and clearly marked non-default.

### Task 2: Rewrite README around the Headroom main path

**Files:**
- Modify: `README.md`

**Interfaces:**
- Consumes: Current Viewer defaults and the validated TJVR trace.
- Produces: A 150–200 line operational README.

- [ ] **Step 1: Write focused overview, build, and live startup**

Use the Headroom algorithm in the title and opening. Include Pixi build/test, PICO_tracker startup, no-argument Viewer startup, and a default-values table.

- [ ] **Step 2: Explain only the current algorithm**

Include the approved data-flow diagram and concise explanations of SPARK IK, feedforward, Headroom reduction, continuity weights, settled-hold, hard limits, and model-reference operation.

- [ ] **Step 3: Preserve explicit startup and replay**

Include the complete explicit Viewer command and the two-terminal `output_continuity_retest.tjvr` replay commands with `/tmp` telemetry paths, 9442 frames, and 106.887 seconds.

- [ ] **Step 4: Add concise operations, safety, verification, and legacy link**

Keep only daily PICO/plot/exit controls, MuJoCo-only safety caveats, `ctest` command, expected no-argument startup summary, and a link to `docs/legacy_algorithms.md`.

### Task 3: Validate and commit the documentation migration

**Files:**
- Verify: `README.md`
- Verify: `docs/legacy_algorithms.md`

**Interfaces:**
- Consumes: Tasks 1–2.
- Produces: Evidence that the docs are focused, complete, and navigable.

- [ ] **Step 1: Validate focus and size**

Run:

```bash
wc -l README.md
rg -n 'spark_upper_qpoases_headroom_feedforward_velocity_qp' README.md
rg -n '^## ' README.md
```

Expected: README has 150–200 lines, the main algorithm appears in defaults, explicit startup, and replay, and headings follow the approved structure.

- [ ] **Step 2: Validate paths and links**

Run `test -e` for the PICO config, fast model, TJVR trace, replay tool, legacy doc, and every local Markdown link target referenced by the two edited documents.

- [ ] **Step 3: Validate formatting and scope**

Run:

```bash
git diff --check
git diff --stat
git status --short
```

Expected: only README, legacy doc, design doc, and plan doc are involved; benchmark directories remain unstaged.

- [ ] **Step 4: Commit**

```bash
git add README.md docs/legacy_algorithms.md
git commit -m "docs: focus README on PICO Headroom QP"
```
