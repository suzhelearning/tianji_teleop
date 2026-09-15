# Pre-Push Review Blockers Fix Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the PICO Headroom README honest and reproducible without committing TJVR data, and isolate Viewer tests that share UDP port 15000.

**Architecture:** The existing README CMake assertion becomes a small documentation contract for required recording, directory, and replay statements. CTest resource locking serializes only Viewer profile tests that can bind the default PICO endpoint while leaving unrelated tests parallel.

**Tech Stack:** Markdown, CMake/CTest, Git

## Global Constraints

- Do not add or stage `output_continuity_retest.tjvr` or any other file under `benchmark_results/`.
- Preserve `spark_upper_qpoases_headroom_feedforward_velocity_qp`, `velocity`, `model_reference`, and `models/marvin_m6_qp_pico_fast.xml` as the main replay identity.
- State that the replay trace is a local prerequisite generated with `--pico-record` or supplied by the tester.
- Use CTest `RESOURCE_LOCK pico_udp_15000` for tests that can bind the default endpoint.

---

### Task 1: Enforce and document the local-trace replay contract

**Files:**
- Modify: `tests/assert_readme_portable.cmake`
- Modify: `README.md`

**Interfaces:**
- Consumes: root README text through `README_FILE`.
- Produces: a focused CTest contract requiring recording, local trace validation, output-directory creation, model, and state-source documentation.

- [ ] **Step 1: Add required README snippets to the existing test**

Append this CMake code to `tests/assert_readme_portable.cmake`:

```cmake
function(require_readme_text required_text)
  string(FIND "${README_CONTENT}" "${required_text}" required_offset)
  if(required_offset EQUAL -1)
    message(FATAL_ERROR "README is missing required text: ${required_text}")
  endif()
endfunction()

require_readme_text("mkdir -p benchmark_results/pico_live/traces")
require_readme_text("--pico-record benchmark_results/pico_live/traces/output_continuity_retest.tjvr")
require_readme_text("TRACE_FILE=benchmark_results/pico_live/traces/output_continuity_retest.tjvr")
require_readme_text("test -f \"$TRACE_FILE\"")
require_readme_text("--input \"$TRACE_FILE\"")
require_readme_text("回放模型：`models/marvin_m6_qp_pico_fast.xml`")
require_readme_text("回放状态源：`model_reference`")
```

- [ ] **Step 2: Run the focused test to verify RED**

Run:

```bash
ctest --test-dir build -R '^test_readme_portable_paths$' --output-on-failure
```

Expected: FAIL with `README is missing required text`.

- [ ] **Step 3: Implement the README contract**

In the explicit launch section, add:

```bash
mkdir -p benchmark_results/pico_live/traces
```

and record the trace with:

```bash
--pico-record benchmark_results/pico_live/traces/output_continuity_retest.tjvr
```

In the replay section, state that the trace is local and not distributed, then use:

```bash
TRACE_FILE=benchmark_results/pico_live/traces/output_continuity_retest.tjvr
test -f "$TRACE_FILE"
```

Pass `--input "$TRACE_FILE"` and include these exact identity lines:

```markdown
- 回放模型：`models/marvin_m6_qp_pico_fast.xml`
- 回放状态源：`model_reference`
```

- [ ] **Step 4: Run the focused test to verify GREEN**

Run:

```bash
ctest --test-dir build -R '^test_readme_portable_paths$' --output-on-failure
```

Expected: `100% tests passed, 0 tests failed out of 1`.

### Task 2: Isolate the default PICO UDP endpoint in CTest

**Files:**
- Modify: `CMakeLists.txt`
- Test: generated CTest metadata in `build/`

**Interfaces:**
- Consumes: CTest resource name `pico_udp_15000`.
- Produces: exclusive scheduling for the three Viewer profile tests that may bind `127.0.0.1:15000`.

- [ ] **Step 1: Verify the resource lock is absent**

Run:

```bash
ctest --test-dir build --show-only=json-v1 | python3 -c '
import json, sys
tests = {item["name"]: item for item in json.load(sys.stdin)["tests"]}
names = ["test_viewer_default_profile", "test_viewer_explicit_profile", "test_viewer_pico_profile"]
assert all(any(p["name"] == "RESOURCE_LOCK" and p["value"] == ["pico_udp_15000"]
               for p in tests[name].get("properties", [])) for name in names)
'
```

Expected: FAIL with `AssertionError`.

- [ ] **Step 2: Add the shared resource lock**

Set this property on all three tests while retaining existing properties:

```cmake
RESOURCE_LOCK pico_udp_15000
```

- [ ] **Step 3: Reconfigure and verify the resource lock**

Run:

```bash
pixi run configure
ctest --test-dir build --show-only=json-v1 | python3 -c '
import json, sys
tests = {item["name"]: item for item in json.load(sys.stdin)["tests"]}
names = ["test_viewer_default_profile", "test_viewer_explicit_profile", "test_viewer_pico_profile"]
assert all(any(p["name"] == "RESOURCE_LOCK" and p["value"] == ["pico_udp_15000"]
               for p in tests[name].get("properties", [])) for name in names)
print("RESOURCE_LOCK_VERIFIED=3/3")
'
```

Expected: `RESOURCE_LOCK_VERIFIED=3/3`.

- [ ] **Step 4: Exercise parallel scheduling**

Run:

```bash
ctest --test-dir build -R '^test_viewer_(default|explicit|pico)_profile$' \
  -j 3 --repeat until-fail:5 --output-on-failure
```

Expected: all three tests pass five repetitions without concurrent use of `pico_udp_15000`.

### Task 3: Verify, review, commit, and push

**Files:**
- Modify: `README.md`
- Modify: `CMakeLists.txt`
- Modify: `tests/assert_readme_portable.cmake`

**Interfaces:**
- Consumes: completed documentation and CTest fixes.
- Produces: reviewed fast-forward update of `origin/main` without benchmark data.

- [ ] **Step 1: Verify the exact diff and benchmark exclusion**

```bash
git diff --check
git diff -- README.md CMakeLists.txt tests/assert_readme_portable.cmake
git status --short
git diff --name-only | grep '^benchmark_results/' && exit 1 || true
```

- [ ] **Step 2: Run the complete suite in bounded groups**

```bash
ctest --test-dir build -I 1,40 --output-on-failure
ctest --test-dir build -I 41,81 --output-on-failure
```

Expected: both groups report 100% passed, totaling 81 tests.

- [ ] **Step 3: Commit only implementation files**

```bash
git add README.md CMakeLists.txt tests/assert_readme_portable.cmake
git commit -m "fix: resolve PICO README review blockers"
```

- [ ] **Step 4: Repeat Standards and Spec reviews**

Review `git diff origin/main...HEAD` against repository standards and
`docs/superpowers/specs/2026-08-18-review-blockers-fix-design.md`. Expected: no push-blocking findings.

- [ ] **Step 5: Push the fast-forward main update**

```bash
git fetch origin main
git merge-base --is-ancestor origin/main HEAD
git push origin main:main
```

Expected: remote `main` advances to the verified local HEAD.
