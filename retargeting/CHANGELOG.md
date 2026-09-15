# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Added the local Tianji simulation bridge `example/tj_wuji2_hand_bridge.py`: side-labeled ROS 2 landmarks are retargeted with MANUS/Hand2 configs and encoded as TJH2 UDP packets in the viewer's named joint order. Missing hands are not refreshed; sender sequence numbers remain increasing across bridge restarts on the same host boot.
- Added an optional local `real_robot` execution boundary for both Marvin arms and both Hand2 devices. It consumes the existing controller's final references, defaults to no-device dry-run, and requires explicit interactive authorization, measured-pose alignment and fresh feedback before motor enable. Vendor I/O is isolated from the existing IK/retargeting algorithms.
- Upgraded internal joint-command output to TJRC v2 (468 bytes) with independent left/right targets and freshness; v1 is rejected. `hands` selects both hands and `all` selects arms plus both hands. Device identities are verified per side and SDK lifetime is shared without allowing one device's cleanup to release its peer.
- Added the workspace `home.sh` / `real_robot/return_home.py` entry for standalone, guarded dual-arm HOME without PICO/Manus or any Wuji hand connection. It reuses configured HOME and motion limits, refuses an occupied teleop endpoint, supports a no-device dry-run, and stops/releases arms on completion, interruption or failure.
- Added workspace `visualize_data.sh` and a dependency-free browser UI backed by the collection Python environment for read-only HDF5 episode playback: recorded-time camera frames, 54 named joint observations/plots, sampling statistics, partial-file labels, and confined loopback-only dataset access without hardware connections.

### Changed

- Consolidated application entry points under root `tianji`, native build dependencies under root `pixi.toml`, and Python dependencies under `pyproject.toml`. Collection/compression/viewer share portable `dataset/` defaults and `TIANJI_DATASET`; historical data and calibration revisions are not rewritten.
- Moved arm control and tracking into `control/` and `tracking/`, vendored licensed Ruckig sources for offline CMake, and replaced three duplicated bitwise CRC implementations with a shared constexpr table. Controller arm-input construction is shared and duplicate Pinocchio FK traversal removed.
- Internalized hand retargeting, calibration commands, and populated model/simulator assets under `retargeting/`; Python imports now use `retargeting.wuji_retargeting` and the root installation. Upstream licenses remain with the assets; no runtime submodule initialization is required.
- Manus launch uses the root `.venv` and retains sourced ROS 2 paths, while preserving paired calibration files and the simulation-only TJH2 safety boundary.
- Cached each retargeter's Euler rotation matrix between frames, recomputing on value changes (including in-place tuning edits) without changing coordinate transforms or filtering.
- Moved adaptive geometry loss and analytical gradients into the mandatory C++17/pybind11 `_native` kernel. Borrowed contiguous arrays and stack-sized finger geometry replace per-iteration NumPy temporaries and Python loops; Pinocchio, NLopt, calibration, regularization, and joint safety remain in their existing implementations.

- Switched default collection to uncompressed, single-frame-chunked RGB HDF5 in `TianjiData_3cam_raw`, retaining legacy JPEG reading. Added `compress_data.sh` for fixed JPEG Q50 copies in an independent dataset with source preservation, resumable validated publication and identity checks. The viewer now supports raw RGB through lossless PNG transport and existing JPEG streams.
- Fixed live preview frame loss by replacing desktop-loop `poll_for_frames` with independent blocking camera readers. Preview retains only each camera's latest owned image and reports separate two-second receive (RX) and submitted-image (UI) rates instead of a lifetime average.
- Added `bash visualize_oneline.sh` for camera-only desktop preview of configured RealSense streams side by side, with serials and measured receive rates, stale-stream detection, and camera release on exit. It neither records datasets nor connects to robot devices.
- Enabled the installed right-wrist RealSense D405 (`409122273692`) alongside top and left-wrist cameras. Collection and viewer defaults now use `TianjiData_3cam`, preserving the old two-camera dataset contract; the existing per-camera acquisition, HDF5 and playback paths handle all three streams.
- Increased the real executor's shared target/feedback position-bound margin from 0.0005 rad to 0.01 rad for all arm and hand joints. Configured/model limits, speed limits and tracking thresholds are unchanged.
- Increased shared hardware feedback freshness from 150 ms to 300 ms consistently across real-executor configuration, Marvin/Hand2 driver health checks, and collection state-progress defaults. Controller-command and per-hand input freshness limits remain unchanged; stale streams still stop the session rather than trigger automatic reconnection.
- The Tianji bridge now publishes bilateral latest retargeting results at 100 Hz using TJH2 v2 (364 bytes), with publication time and independent original receive timestamps per hand. The controller linearly samples a 10 ms delayed timeline at its 200 Hz real-mode rate, holds endpoints without extrapolation, and preserves independent source expiry despite repeated publications. Rebuild the controller and restart both bridge and executor; v1 packets are rejected.
- Removed runtime following-error shutdown for both Hand2 devices; arm tracking protection, hand feedback/input freshness, SDK health, position/speed bounds, and startup alignment/arrival checks remain active.
- Increased only the PICO bridge-to-command freshness limit from 150 ms to 300 ms. Device feedback, executor packet, and per-hand source freshness limits remain unchanged; future timestamps are still rejected.
- Standalone `home.sh` now explicitly adopts healthy already-enabled dual arms, holds their measured pose before returning HOME, and stops/disables them on completion or after takeover failure. Idle-arm enable remains supported; mixed enable states and occupied command ports remain rejected, normal teleop still refuses takeover, and Wuji hands remain untouched.

### Fixed

- Fixed dataset playback flashing black/loading between JPEG frames: keep the previous eligible decoded image until replacement is ready, accept delayed frames during forward playback, and report the displayed frame's actual age. Seek revisions and episode generations reject stale completions; playback elapsed time cannot become negative at startup.
- Fixed Marvin enable guards rejecting the vendor's normal `101` position-mode transition. Accept it only within the driver's own bounded two-second enable window, retain freshness/error/bounds checks, and require both arms in state `1` before motion. Idle cleanup still requires state `0`; timeout diagnostics now retain actual states even when the feedback health detail is empty.
- Fixed `--pico-teleop` silently overriding the configured target timeout with 50 ms. Runtime now honors `cartesian_servo.target_timeout_seconds`; integration coverage checks actual stale transitions against each profile rather than a hardcoded threshold.
- Constrained the Pinocchio 3.8 wheel's URDFDOM and TinyXML2 dependencies to compatible ABI versions in `requirements.txt`.
- Fixed the local Manus input adapter rejecting real Metaglove Pro skeletons whose thumb IP is labeled `Distal` rather than the SDK header's documented `Intermediate`; exactly one IP candidate and a valid parent chain are still required.
- Derived the Tianji bridge's canonical-to-model wrist rotation from Hand2 neutral landmarks, replacing inherited hand-one Euler angles and segment scaling. Added neutral-pose, individual-finger, rigid-motion, and named-joint regressions.
- Removed the local Manus adapter's legacy Y reflection so SDK right-handed coordinates retain palmward flexion through Hand2 wrist normalization. Added full SDK-record-to-retarget direction regressions for all five fingers on both hands.

## [2026.8.17]

### Changed

- Updated the `wuji-description` hand model submodule to `v2026.8.3` and pointed the Wuji Hand 2 example configs (`adaptive_analytical_wuji_glove_wuji_hand_2_{left,right}.yaml`) at the new `hand2/hand2_beta1/body/` asset layout. Retuned `mediapipe_rotation` for the updated model orientation (left `x = 180`, right `y = 180`, both `z = -90`). No change to IK or simulation behavior.

## [2026.8.3]

### Changed

- Retuned the `mediapipe_rotation` bias in the Wuji Glove example configs (`adaptive_analytical_wuji_glove_{left,right}.yaml` and `adaptive_analytical_wuji_glove_wuji_hand_2_{left,right}.yaml`) for the new per-serial calibration. The manual `wrist_offset_cm` / `thumb_offset_cm` stay disabled because the SDK applies a per-serial calibrated URDF.

### Fixed

- Fixed `teleop_real.py` crashing at startup with `AttributeError` on the Wuji Hand 2 (network) backend after the Wuji SDK 2026.7.1 resource-API redesign. Control behavior is unchanged. Requires Wuji SDK ≥ 2026.7.1 and matching firmware.

## [2026.6.27]

### Changed

- Standardized hardware naming across the docs and example configs to `Wuji Hand 2` / `Wuji Hand` / `Wuji Glove`, fixing inconsistent model references. The Wuji Glove example configs are now `adaptive_analytical_wuji_glove_wuji_hand_2_{left,right}.yaml`.

## [2026.6.15]

### Added

- Added Wuji Hand 2 (network-connected) support to `teleop_real.py`. Select the hand model with `--hand-model {wuji_hand,wuji_hand_2}` (inferred from the config when omitted). The new network hand also accepts `--wuji-hand-2-ip` (auto-discovers when omitted), `--kp`, `--kd`, and `--current-limit`
- Added config-driven hand model selection for retargeting. Point the optimizer at any hand via `optimizer.urdf_path` (IK) and `optimizer.mjcf_path` (simulation). `optimizer.link_naming` maps logical link roles (palm, fingertip, PIP, DIP, MCP) onto the URDF's actual link names, so anatomically named hands work without code changes. Joint commands are remapped by name across the viewer, simulation, and hardware paths, so a URDF that declares fingers in a different order still drives the right ones. Ships right- and left-hand Wuji Hand 2 configs. The default Wuji Hand path is unchanged when no override is set
- Documented Docker usage in the README. No official Dockerfile is shipped. The Wuji SDK reads per-device assets from `~/.wuji`, which must be mounted into the container when using Wuji Glove or real hardware (simulation and replay do not need it)

### Fixed

- Fixed the adaptive analytical optimizer hardcoding finger joint indices, which could silently apply hyperextension and DIP/PIP coupling constraints to the wrong joints on custom hand URDFs. Indices are now resolved from the kinematic chain, with a clear error at load time if the expected finger links are missing. Behavior on the default hand is unchanged
- Fixed `pip install .` silently producing an `UNKNOWN` package with no dependencies on systems with older setuptools. On Ubuntu 22.04 also run `pip install -U pip` for the fix to take effect — see the Ubuntu note in the README

## [2026.6.10]

### Fixed

- Fixed the `wuji-description` hand model submodule being absent from the v2026.05.26 release. A fresh `git clone --recurse-submodules` now correctly fetches the URDF/MJCF assets, so `teleop_sim`, `teleop_real`, and `tuning_tool` work out of the box.

## [2026.05.26]

### Added

- Added Retargeting Parameter Tuning Guide covering tuning_tool usage, skeleton color legend, a full parameter quick-reference table, and the recommended tuning order

### Changed

- Swapped the hand description submodule from `wuji_hand_description` to `wuji-description`. The URDF and MJCF assets now live under `wuji-description/hand/body/`, and the package no longer depends on the `mujoco-sim` submodule for hand model files.

## [2026.05.23]

### Changed

- Tuned the Wuji Glove left/right example configs for better fit: refined per-finger segment scaling, adjusted MediaPipe rotation calibration, and disabled the wrist/thumb offsets in favor of the SDK's built-in per-device calibration

## [2026.05.18]

### Added

- Added Custom Input Device Integration Guide describing how to integrate a custom hand input device with a small wrapper class
- Added `Custom Input Devices` section to README linking to the new integration guide
- Added Wuji Glove input device (`wuji_glove_device.py`) with SDK-driven 21 MediaPipe keypoint output
- Added Wuji Glove example configs for left and right hands
- Added offset calibration tool (`calibrate_offset.py`) for neutral-pose wrist/thumb offsets
- Added `--input wuji_glove` support in `example/teleop_sim.py` and `example/teleop_real.py`
- Added Wuji Glove live mode to `example/tuning_tool.py`
- Added optimizer extensions for thumb PIP skipping, anti-hyperextension, DIP/PIP coupling, optional URDF override, and retarget keypoint offsets
- Added `wuji-sdk>=0.10.0` dependency and optional `tuning` extra

### Changed

- Updated docs to recommend Wuji Glove as the live input path while keeping other input sources available

## [2026.04.27] - 2026-04-27

### Added

- Added a key-vector retargeting optimizer with example configs for Apple Vision Pro and video
- Added ZED camera support as a real-time hand input
- Added interactive parameter tuning visualizer with three-layer skeleton comparison, HUD, and fingertip highlighting. Supports hot-reload and playback

## [0.2.0] - 2026-04-07

### Added

- Added MP4 video input device with MediaPipe hand tracking
- Added Intel RealSense camera input device with MediaPipe hand tracking
- Added an abstract base class for input devices
- Added a video/RealSense configuration preset
- Added `--video`, `--realsense`, `--show-video` CLI flags to teleop scripts
- Added optional `realsense` and `video` install extras

## [0.1.1] - 2026-02-02

### Changed

- Upgraded to v0.2.1 hand description

### Fixed

- Removed invalid MDX syntax in appendix documentation

## [0.1.0] - 2026-01-21

Initial public release.

### Added

- Added core retargeting system with a unified retargeter interface
- Added adaptive analytical optimizer with Huber loss and hand-written analytical gradients
- Added low-pass filtering for smooth motion output
- Added Apple Vision Pro hand tracking support
- Added MediaPipe recording playback support
- Added input data recording functionality
- Added MuJoCo simulation example
- Added real hardware control example
- Added YAML-based configuration system with per-finger scaling and pinch thresholds

[Unreleased]: https://github.com/wuji-technology/wuji-retargeting/compare/v2026.8.17...HEAD
[2026.8.17]: https://github.com/wuji-technology/wuji-retargeting/compare/v2026.8.3...v2026.8.17
[2026.8.3]: https://github.com/wuji-technology/wuji-retargeting/compare/v2026.6.27...v2026.8.3
[2026.6.27]: https://github.com/wuji-technology/wuji-retargeting/compare/v2026.6.15...v2026.6.27
[2026.6.15]: https://github.com/wuji-technology/wuji-retargeting/compare/v2026.6.10...v2026.6.15
[2026.6.10]: https://github.com/wuji-technology/wuji-retargeting/compare/v2026.05.26...v2026.6.10
[2026.05.26]: https://github.com/wuji-technology/wuji-retargeting/compare/v2026.05.23...v2026.05.26
[2026.05.23]: https://github.com/wuji-technology/wuji-retargeting/compare/v2026.05.18...v2026.05.23
[2026.05.18]: https://github.com/wuji-technology/wuji-retargeting/compare/v2026.04.27...v2026.05.18
[2026.04.27]: https://github.com/wuji-technology/wuji-retargeting/compare/v0.2.0...v2026.04.27
[0.2.0]: https://github.com/wuji-technology/wuji-retargeting/compare/v0.1.1...v0.2.0
[0.1.1]: https://github.com/wuji-technology/wuji-retargeting/compare/v0.1.0...v0.1.1
[0.1.0]: https://github.com/wuji-technology/wuji-retargeting/releases/tag/v0.1.0
