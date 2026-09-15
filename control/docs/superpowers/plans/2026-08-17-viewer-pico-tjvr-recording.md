# Viewer Native PICO TJVR Recording Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `--pico-record PATH.tjvr` so a live Viewer run records the exact validated TJVR UDP input stream for deterministic replay.

**Architecture:** A dedicated `PicoTraceRecorder` owns an exclusive streaming file on the PICO receiver thread. `PicoUdpReceiver` appends original packet bytes after protocol/CRC validation and before stream gating, while the Viewer only configures the path and reports final status.

**Tech Stack:** C++17, POSIX file APIs, UDP sockets, GoogleTest, Python 3 integration tests, CMake/CTest, existing TJVR v1 container and TJVR v4 packet protocol.

## Global Constraints

- Keep the 200 Hz control thread free of trace file I/O.
- Record protocol-valid packets before stream-gate rejection.
- Preserve packet bytes, source timestamps, tracking epochs, sequence values, and CRC exactly.
- Produce the existing little-endian `TJVT` version-1 container.
- Refuse existing destinations and never overwrite them.
- Runtime recording failure must not stop UDP reception or robot control.
- `--pico-record` requires `--pico-teleop`; omission preserves current behavior.
- Do not change the strong-output-damping configuration in this implementation.

---

### Task 1: Streaming TJVR recorder

**Files:**
- Create: `include/tianji_qp_ik/pico_trace_recorder.hpp`
- Create: `src/pico_trace_recorder.cpp`
- Create: `tests/test_pico_trace_recorder.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: destination path, original packet bytes, packet size, and monotonic receive timestamp.
- Produces: `PicoTraceRecorder`, `PicoTraceRecorderStats`, `PicoTraceRecorderState`, and `picoTraceRecorderStateName()`.

- [x] **Step 1: Add failing recorder tests.** Define tests that construct `PicoTraceRecorder(path)`, append two 656-byte packets at `1'000'000'000` and `1'011'000'000 ns`, call `finish()`, and require header magic `TJVT`, version 1, packet size 656, count 2, relative times 0 and 11 ms, and byte-identical packet payloads. Add tests for existing-file refusal, mixed packet-size failure without throwing from `append()`, and idempotent `finish()`.
- [x] **Step 2: Verify RED.** Run `cmake -S . -B build && cmake --build build -j12 --target test_pico_trace_recorder`; expect compilation failure because `pico_trace_recorder.hpp` and the class do not exist.
- [x] **Step 3: Implement the minimal recorder.** Use `open(..., O_CREAT | O_EXCL)` to reserve the path. On the first append write little-endian header `{TJVT, 1, packet_size, 0}`, then append `<q relative_ns>` and raw bytes. On `finish()`, seek to byte 8, write the final count, flush, and close. Treat supported packet sizes as 160, 208, 400, and 656; a mixed size or I/O error changes state to failed and makes later appends no-ops. A no-packet finish removes the empty reserved file and reports `no_valid_packets`.
- [x] **Step 4: Verify GREEN.** Build and run `./build/test_pico_trace_recorder`; require all recorder tests to pass.

### Task 2: Record at the receiver validation boundary

**Files:**
- Modify: `include/tianji_qp_ik/pico_udp_receiver.hpp`
- Modify: `src/pico_udp_receiver.cpp`
- Modify: `tests/test_pico_udp_receiver.cpp`

**Interfaces:**
- Consumes: `PicoUdpReceiverOptions::record_path` and `PicoTraceRecorder::append()`.
- Produces: recording state/count through `PicoReceiverStats`.

- [x] **Step 1: Add failing receiver tests.** Configure a temporary recording path, send one valid packet, one CRC-invalid packet, one protocol-malformed datagram, and one protocol-valid jump-rejected packet. After `stop()`, require the trace count to be 2 and its packets to equal the valid accepted and valid jump-rejected datagrams; require malformed/CRC-invalid packets to be absent. Assert receiver stats report finalized recording and count 2.
- [x] **Step 2: Verify RED.** Build `test_pico_udp_receiver`; expect failure because receiver options/stats have no recording interface.
- [x] **Step 3: Implement receiver wiring.** Add optional `record_path` as the final options member, construct the recorder before socket startup, and append immediately after successful decode and monotonic receive-time assignment but before `gate.evaluate()`. Finalize after joining the receiver thread in `stop()`. Convert recording exceptions at startup into Viewer-visible startup errors; runtime append failures remain recorder status only.
- [x] **Step 4: Verify GREEN.** Run `./build/test_pico_udp_receiver`; require all UDP and recording-boundary tests to pass.

### Task 3: Viewer CLI and deterministic end-to-end replay

**Files:**
- Modify: `apps/run_qp_ik_viewer.cpp`
- Modify: `tests/test_pico_viewer_integration.py`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: CLI option `--pico-record PATH.tjvr`.
- Produces: startup line `pico_record_path=PATH`, final summary fields `pico_record_state` and `pico_recorded_packets`, and a trace readable by the existing Python tools.

- [x] **Step 1: Add failing integration coverage.** Extend the Python integration test to run a headless PICO Viewer with a temporary `--pico-record` path, send known v4 packets including one stream-gate jump rejection, then parse the binary trace and require unmodified packet bytes, original source timestamps, packet size 656, monotonically increasing relative receive times, and a count including the valid gate-rejected packet. Add CLI failure checks for missing `--pico-teleop` and an existing output path.
- [x] **Step 2: Verify RED.** Run the new integration case and require failure because the CLI rejects `--pico-record` as unknown.
- [x] **Step 3: Implement CLI wiring/reporting.** Add the option to `Options`, help text, parser, preflight validation, receiver options, startup output, and headless/final summaries. Ensure all normal and exceptional shutdown paths call receiver `stop()` before reading final recording stats.
- [x] **Step 4: Verify GREEN.** Run the focused Viewer integration case and validate the generated trace with `/home/zj/current_robotics/TJ_arm/vr_data/tools/tjvr_format.py`.

### Task 4: Full verification and handoff

**Files:**
- Modify: `docs/superpowers/plans/2026-08-17-viewer-pico-tjvr-recording.md`

**Interfaces:**
- Consumes: all implementation and test outputs.
- Produces: verified live recording and replay commands.

- [x] **Step 1: Run build and all tests.** Run `cmake --build build -j12` and all 76+ CTest cases in bounded batches with `--output-on-failure`; require zero failures.
- [x] **Step 2: Run an actual record/replay smoke test.** Record synthetic v4 packets to a fresh `/tmp` trace, validate with `tjvr_format.py --validate-only` through `replay_pico_udp_trace.py`, and replay into a second headless Viewer with zero control failures.
- [x] **Step 3: Check repository integrity.** Run `git diff --check`, confirm no Viewer/replay process remains, and inspect the final diff for unrelated changes.
- [x] **Step 4: Update this checklist and report exact commands, test counts, trace status, and commit state to the user.**
