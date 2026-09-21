# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Environment

This is the monorepo's ROS 2 Humble workspace. Use the root `.venv` for
all Python commands and a compatible ROS SDK. Do not create a separate
environment in tracking. See `docs/install-guide.md` for system dependencies,
the `ROS_SETUP` override and SDK/Python ABI requirements.

```bash
source scripts/environment.sh   # from tracking, after building
```

Tracking launchers source the SDK and `install/local_setup.bash` automatically.

## Common commands

```bash
# From tracking/: compile internal source using the root environment.
bash scripts/build.sh --all
colcon test --base-paths src --event-handlers console_direct+

# Single C++ test (pico_bridge has a gtest)
colcon test --packages-select pico_bridge --ctest-args -R PicoFrame --event-handlers console_direct+

# Run (three terminals; each must source scripts/environment.sh)
adb forward tcp:9999 tcp:9999
ros2 launch pico_bridge start_pico_bridge.launch.py          # host:=… port:=… optional
ros2 launch fisheye_camera fisheye_camera.launch.py          # optional
ros2 run data_collector data_collector_node \
    --config src/data_collector/config/collect_config.yaml   # keys: s=start, d=stop+save, q=quit

# Smoke test without a real PICO
python3 src/pico_bridge/scripts/mock_pico_server.py --fps 30 --duration 60
```

## Architecture

Three ROS2 packages under `src/`, connected only by topics. No shared library — each package is independently buildable.

```
PICO 4U ──adb forward tcp:9999──► pico_bridge ──┐
                                                 │
USB fisheye /dev/video*  ──────► fisheye_camera ─┼──► data_collector ──► HDF5 + MP4 + CSV
                                                 │
                       (ROS2 topics)             ┘
```

### `pico_bridge` (C++, rclcpp)
TCP client that connects to `PicoStreamingServer` on the headset. Single background thread runs `recv_loop`, decodes the 14-byte little-endian frame header (`[0xAB | type:1 | ts_ms:i64 | payload_len:u32]`), and republishes:

| Wire type | Topic | Msg |
|---|---|---|
| `0x01` / `0x02` | `/pico/cam_{left,right}/compressed` | `sensor_msgs/CompressedImage` (jpeg) |
| `0x03` / `0x04` / `0x05` | `/pico/pose/{left_hand,right_hand,head}` | `geometry_msgs/PoseStamped` |
| `0x10` / `0x11` | `/pico/ble/{left,right}` | `pico_bridge/BleFrame` (custom) |
| `0x20`–`0x37` | `/pico/smpl_raw` | `geometry_msgs/PoseArray` (24 APK poses) |

`pico_smpl_ground` subscribes to `/pico/smpl_raw` and `/pico/world_reset`.
After A plus 30 stable foot frames it publishes canonical `/pico/smpl` in
`pico_ground`, with the initial foot soles at `Z=0`.

- Protocol constants and parsers live in a **header-only** file: `src/pico_bridge/include/pico_bridge/pico_frame.hpp` (`FRAME_MAGIC=0xAB`, `HEADER_SIZE=14`, `MAX_PAYLOAD_BYTES=10 MiB`). Unit tests (`test/test_pico_frame.cpp`) exercise it.
- Pose payload = 7×float32 little-endian `[pos.xyz | quat.xyzw]`. BLE payload = `[esp32_ts:u32 | sensor_bytes]`.
- Wire `ts_ms` is stuffed into `header.stamp` as `sec = ts_ms/1000, nanosec = (ts_ms%1000)*1e6`. **This is a PICO device-local boot time, not ROS/epoch time** — do not compare against `rclcpp::Clock::now()`.
- The `0x06` world-reset frame is parsed and published as `/pico/world_reset`; it resets ground, foot-IMU, and Odin session calibration state.
- On bad magic / oversized payload / disconnect, the node tears down the socket and reconnects after 2 s.
- Full wire spec: `docs/PICO_Streaming_Guide.md`. Original Python reference: `reference/receiver.py`.

### `fisheye_camera` (C++, rclcpp)
Spawns one thread per camera; each opens a V4L2 MJPEG stream and publishes the **raw JPEG mmap buffer** directly as `sensor_msgs/CompressedImage` (zero-copy passthrough — no decode, no re-encode). Reliability is `BEST_EFFORT`, depth 1.

- Cameras configured as strings `"device:name[:key=val...]"` (e.g. `/dev/video0:right_hand_fisheye:auto_exposure=1`) in `config/fisheye_cameras.yaml`. Topic becomes `/<name>/image/compressed`.
- Supported inline keys: `auto_exposure`, `auto_wb`, `exposure`, `gain`, `wb_temperature`. `rotate` is parsed but **ignored** (incompatible with zero-copy mmap); `record=true` is also a no-op (recording is handled by `data_collector`).
- Reconnect logic: 30 consecutive grab failures → close + retry open up to 5× at 3 s intervals.

### `data_collector` (Python, rclpy)
Config-driven recorder. Subscribes to topics listed in `config/collect_config.yaml`, buffers messages by wall-clock receive time, and at `collection_frequency` Hz picks the buffer entry closest to the current wall-clock time (via `bisect_left`) from **every** configured dataset. If any buffer is empty, or the nearest sample is older than `max_age`, collection aborts.

Key design points when editing this package:
- Each dataset entry in YAML points to a `processor` class (Python import path); the node imports it dynamically. To add a new stream, write a `DataProcessor` subclass in `data_collector/data_processor.py` and reference it from YAML — no changes to `data_collector_node.py`.
- Processors return a dict tagged with `kind` (`image` / `pose` / `ble`). `save_data()` creates one HDF5 group per dataset shaped by `kind`:
  - `image`: variable-length `jpeg` dataset + `ts_ms` + an MP4 written via `cv2.VideoWriter` (one mp4 per image stream, sized from the first decoded frame).
  - `pose`: `pos [N,3] f64`, `quat_xyzw [N,4] f64`, `ts_ms [N] i64`.
  - `ble`: vlen `data` + `esp32_ts [N] u32` + `ts_ms [N] i64`. The `raw` field is the full wire payload (esp32_ts prefix reconstructed).
- **Synchronization is wall-clock based, not PICO-header based.** This is intentional (see `create_callback` comment in `data_collector_node.py:108-119`) because the PICO `ts_ms` is device boot time and cannot be aligned to ROS wall time. `ts_ms` is still preserved per-frame inside each processor's output.
- Output layout per session: `<data_dir>/<YYYYmmdd_HHMMSS>/data.hdf5 + *.mp4 + timestamps.csv`.
- Keyboard loop runs on main thread via `termios` raw mode; `rclpy.spin` runs on a daemon thread. Don't move spinning onto the main thread without restructuring the control loop.
- **`keyboard_controller` / `data_collector.launch.py` are currently dead paths.** `keyboard_controller.py` calls `start_collect` / `stop_collect` Trigger services that `data_collector_node.py` never creates — the only working control path is `ros2 run data_collector data_collector_node --config …` with its built-in keyboard loop. Add the services to the node before relying on the launch file.

## Pitfalls

- Do **not** try to correlate `header.stamp` across pico_bridge and other sources — it's PICO boot-relative. Use wall-clock receive time (the existing pattern) or publish your own synchronized clock.
- Fisheye camera rotation is not implementable in the current zero-copy pipeline; the `rotate` key is accepted only to silence legacy configs.
- The `mock_pico_server.py` script is the only way to exercise `pico_bridge` without a headset — prefer it over stubbing in code.
- `reference/visualizer.py` is a standalone (non-ROS) live viewer for the PICO stream — camera windows + 3D pose trajectory. Useful for eyeballing the raw TCP stream when debugging; needs only `adb forward` + matplotlib/scipy/opencv, no build.
- Root dependency pins remain load-bearing: `setuptools <80` for ament Python editable builds and `cmake <4`. `scripts/build.sh` selects root `.venv` Python explicitly. Never mix Python ABIs between `.venv` and the ROS SDK.
- `PICO_adb.zip` and `PICO_adb/` are vendored artifacts unrelated to the ROS workspace — don't include them in builds.
- Design docs live in `docs/superpowers/specs/` and implementation plans in `docs/superpowers/plans/`; check them before making protocol-level changes.
