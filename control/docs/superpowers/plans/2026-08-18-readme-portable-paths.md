# README Portable Paths Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove machine-specific absolute paths from the root README and prevent them from returning.

**Architecture:** A focused CMake script reads only the root README and rejects Unix or macOS user-home prefixes. README commands run from the repository root, use explicit external-repository placeholders, and store replay telemetry under the existing repository benchmark directory.

**Tech Stack:** Markdown, CMake/CTest, Git

## Global Constraints

- Do not assume the checkout location or relative layout of `TJ_arm_control`, `PICO_tracker`, and `vr_data`.
- Do not stage or commit untracked `benchmark_results/` runtime data.
- Keep the current PICO Headroom algorithm, model, control level, and command-line options unchanged.
- The root README must not contain `/home/` or `/Users/` user-directory paths.

---

### Task 1: Add the README portability regression test

**Files:**
- Create: `tests/assert_readme_portable.cmake`
- Modify: `CMakeLists.txt`
- Test: `tests/assert_readme_portable.cmake`

**Interfaces:**
- Consumes: CMake variable `README_FILE`, containing the absolute path to the root README.
- Produces: CTest `test_readme_portable_paths`, which succeeds only when the README contains no `/home/` or `/Users/` prefix.

- [ ] **Step 1: Write the failing CMake check**

```cmake
if(NOT DEFINED README_FILE)
  message(FATAL_ERROR "README_FILE is required")
endif()

file(READ "${README_FILE}" README_CONTENT)
foreach(FORBIDDEN_PREFIX IN ITEMS "/home/" "/Users/")
  string(FIND "${README_CONTENT}" "${FORBIDDEN_PREFIX}" PREFIX_OFFSET)
  if(NOT PREFIX_OFFSET EQUAL -1)
    message(FATAL_ERROR
      "README contains machine-specific path prefix: ${FORBIDDEN_PREFIX}")
  endif()
endforeach()
```

Register it in the existing `if(BUILD_TESTING)` section:

```cmake
add_test(
  NAME test_readme_portable_paths
  COMMAND ${CMAKE_COMMAND}
    -DREADME_FILE=${CMAKE_CURRENT_SOURCE_DIR}/README.md
    -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/assert_readme_portable.cmake
)
```

- [ ] **Step 2: Configure and run the test to verify RED**

Run:

```bash
cmake -S . -B build
ctest --test-dir build -R '^test_readme_portable_paths$' --output-on-failure
```

Expected: FAIL with `README contains machine-specific path prefix: /home/`.

### Task 2: Make all root README commands portable

**Files:**
- Modify: `README.md`
- Test: `tests/assert_readme_portable.cmake`

**Interfaces:**
- Consumes: the repository-root execution convention and external paths supplied by the reader.
- Produces: portable build, PICO startup, Viewer startup, and TJVR replay examples.

- [ ] **Step 1: Replace repository-local absolute paths**

Remove each `cd /home/zj/current_robotics/TJ_arm/TJ_arm_control` command and state once that the corresponding commands run from the `TJ_arm_control` repository root.

- [ ] **Step 2: Replace external tool paths**

Use these exact forms:

```bash
cd <PICO_tracker目录>
python3 <vr_data目录>/tools/replay_pico_udp_trace.py \
```

- [ ] **Step 3: Move replay telemetry into the repository**

Use these exact commands and output paths:

```bash
mkdir -p benchmark_results/pico_live
--telemetry benchmark_results/pico_live/pico_headroom_main_replay.csv
--joint-telemetry benchmark_results/pico_live/pico_headroom_main_replay_joints.csv
```

- [ ] **Step 4: Run the focused test to verify GREEN**

Run:

```bash
ctest --test-dir build -R '^test_readme_portable_paths$' --output-on-failure
```

Expected: `100% tests passed, 0 tests failed out of 1`.

### Task 3: Verify and commit the implementation

**Files:**
- Modify: `README.md`
- Modify: `CMakeLists.txt`
- Create: `tests/assert_readme_portable.cmake`

**Interfaces:**
- Consumes: the completed README and CTest registration.
- Produces: one reviewed implementation commit with no benchmark runtime data.

- [ ] **Step 1: Check Markdown and CMake diffs**

Run:

```bash
git diff --check
rg -n '/home/|/Users/' README.md
git diff -- README.md CMakeLists.txt tests/assert_readme_portable.cmake
```

Expected: `git diff --check` succeeds, `rg` returns no matches, and the diff contains only the intended documentation and test changes.

- [ ] **Step 2: Run the complete test suite**

Run:

```bash
ctest --test-dir build --output-on-failure
```

Expected: all configured tests pass, including `test_readme_portable_paths`.

- [ ] **Step 3: Commit only implementation files**

```bash
git add README.md CMakeLists.txt tests/assert_readme_portable.cmake
git commit -m "docs: make README paths portable"
```
