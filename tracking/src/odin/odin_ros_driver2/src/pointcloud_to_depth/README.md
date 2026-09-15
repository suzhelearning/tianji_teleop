# pointcloud2depth 模块 / Module

> 中英双语 / Bilingual (Chinese + English)

## 1. 模块定位 / Overview

**中文**：
`pointcloud2depth` 把 dToF 雷达发出的稀疏点云 (`PointCloud2`，单位米) 与 Odin1 鱼眼 RGB 相机的去畸变图像在时间上同步后，按相机 0 的内参 / 外参把每个 3D 点投影到一张深度图上，再经过膨胀、Sobel 边缘抑制、最近邻上采样得到与 RGB 同分辨率的稠密深度图，并发布数据 (`depth`) 和两张可视化图 (`depth_color`、`depth_overlay`)。

**English**:
`pointcloud2depth` consumes the sparse dToF point cloud (`PointCloud2`, in meters) together with the time-synchronized undistorted RGB image from camera 0, projects each 3D point onto a low-resolution canvas using camera 0's intrinsics + extrinsics, then applies dilation, Sobel edge suppression and nearest-neighbor upsampling to obtain a dense depth map at the full RGB resolution. It publishes the data product (`depth`) and two visualizations (`depth_color`, `depth_overlay`).

> 设计原则 / Design principle: **不依赖任何 PCL / Eigen / ROS 类型**的纯 C 风格核心算法（见 `pointcloud2depth_node.hpp`），方便将来下沉到固件 / 嵌入式 SDK。ROS 节点只负责消息收发 + cv_bridge + OpenCV 后处理。
>
> The projection core (`pointcloud2depth_node.hpp`) is **PCL/Eigen/ROS-free** — only plain C-style types — so it can be reused on the device or in a non-ROS SDK. The ROS node wraps it with cv_bridge + OpenCV post-processing only.

---

## 2. 文件结构 / File Layout

```
src/pointcloud2depth/
├── pointcloud2depth_node.cpp     # ROS1/ROS2 双兼容节点 / dual-ROS node entry
├── pointcloud2depth_node.hpp       # 投影核心算法（纯 C 风格）/ projection core
└── README.md                     # 本文档 / this document
```

依赖的共享头 / Shared headers (still under `include/utility/`):

- `yaml_config_loader.hpp` — 解析 `calib_<SN>.yaml`
- `ros_compat.hpp`         — ROS1/ROS2 兼容宏

---

## 3. 输入 / Inputs

| Topic | Type | 说明 / Description |
|-------|------|--------------------|
| `/{prefix}/{model}/device{N}/cloud/raw` | `sensor_msgs/PointCloud2` | dToF 原始点云，约 1~5 万点 / Raw dToF cloud, ~10k–50k points |
| `/{prefix}/{model}/device{N}/camera0/undistort` | `sensor_msgs/Image` (bgr8) | 鱼眼相机去畸变后的 RGB 图 / Undistorted RGB image |
| `/{prefix}/driver/device_online` | `std_msgs/String` (JSON) | 设备发现广播（**TRANSIENT_LOCAL**，延后加入也能收到）/ Device discovery broadcast (TL) |
| `/{prefix}/driver/resolution_change` | `std_msgs/String` (JSON) | 当前 RGB 分辨率（**TRANSIENT_LOCAL**）/ Current RGB resolution (TL) |

两个同步源使用 `message_filters::ApproximateTime`，slop 默认 100 ms。
The two streams are time-synchronized via `message_filters::ApproximateTime`, default slop 100 ms.

### 服务 / Services

| Service | Type | 用途 / Purpose |
|---------|------|----------------|
| `/{prefix}/{model}/device{N}/get_calibration` | `odin_ros_driver_rev1/GetCalibration` | 拉取 `calib_<SN>.yaml` 内容（首次启动用）/ Fetch `calib_<SN>.yaml` once at startup |

---

## 4. 输出 / Outputs

主题前缀 / Topic prefix: `/{topic_prefix}/{model}/device{N}/{output_topic_suffix}/`，默认 `output_topic_suffix = "pointcloud2depth"`。

| Topic | Encoding | 内容 / Content |
|-------|----------|----------------|
| `.../pointcloud2depth/depth`         | `32FC1` (m)  | 深度数据产品（全分辨率，单位米）/ Depth data product, full-res, meters |
| `.../pointcloud2depth/depth_color`   | `bgr8`       | JET 色彩可视化（无效像素为纯黑）/ JET visualization (black = invalid) |
| `.../pointcloud2depth/depth_overlay` | `bgr8`      | JET 与 RGB 叠加（验证对齐）/ JET blended onto RGB (alignment check) |

---

## 5. 参数 / Parameters

> 所有参数都通过 launch 文件暴露，下表为节点内默认值。
> All parameters are exposed via the launch file; the table shows the in-node defaults.

| 参数 / Param | 类型 / Type | 默认 / Default | 含义 / Meaning |
|-------------|-------------|----------------|----------------|
| `topic_prefix`         | string | `"manifold"` | 主驱动话题前缀 / Driver topic prefix base |
| `scale`                | double | `7.0`        | 投影画布下采样倍数，低分辨率 = `img / scale` / Projection canvas downscale factor |
| `z_min`                | double | `0.1`        | 有效深度下限 (m) / Minimum valid depth |
| `z_max`                | double | `50.0`       | 有效深度上限 (m) / Maximum valid depth |
| `dilate_radius`        | int    | `1`          | 投影点 3×3 膨胀半径（0 = 关闭）/ Dilation radius for projected pixels |
| `use_z_buffer`         | bool   | `false`      | true=z-buffer（取近）；false=只写空像素（贴合文档默认）/ true: keep nearer; false: write-if-empty (matches doc spec) |
| `edge_grad_threshold`  | double | `0.75`       | Sobel 梯度阈值 (m)，超过则擦除像素，0 = 关闭 / Sobel gradient threshold; 0 disables edge suppression |
| `vis_depth_min`        | double | `0.1`        | JET 色图归一化下限 (m)；< 0 → 逐帧自适应 / JET min; < 0 = per-frame auto |
| `vis_depth_max`        | double | `5.0`        | JET 色图归一化上限 (m) / JET max |
| `cloud_topic_suffix`   | string | `"cloud/raw"`         | 点云话题后缀 / Cloud topic suffix |
| `image_topic_suffix`   | string | `"camera0/undistort"` | 图像话题后缀 / Image topic suffix |
| `output_topic_suffix`  | string | `"pointcloud2depth"`  | 输出话题分组名 / Output namespace |
| `sync_queue_size`      | int    | `10`         | 同步器队列长度 / Synchronizer queue size |
| `sync_slop_sec`        | double | `0.1`        | 同步最大时间差 (s) / Max sync slop |

---

## 6. 处理流程 / Pipeline

```
                        ┌─────────────────────┐
   /cloud/raw  ────────►│ ApproximateTime     │   (slop = 100 ms)
                        │   Synchronizer      │
   /camera0/undistort ─►│                     │
                        └──────────┬──────────┘
                                   ▼
   ┌────────────────────────────────────────────────┐
   │ 1. PointCloud2 → float xyz[]                   │   (no copy of color/intensity)
   │ 2. depth_core::pointcloud_to_depth             │   (low-res canvas, plain types)
   │      - 透视投影: u,v = K·Tcl·P                  │
   │      - 仅保留 z ∈ [z_min, z_max] 的点           │
   │      - 3×3 膨胀填洞 (dilate_radius)             │
   │      - z-buffer 可选 (use_z_buffer)            │
   │ 3. cv::resize INTER_NEAREST → 全分辨率深度图    │
   │ 4. Sobel 边缘抑制 (edge_grad_threshold > 0)    │
   │ 5. JET 色图（cv::applyColorMap, SIMD）         │
   │ 6. addWeighted 与 RGB 叠加                     │
   └────────────────┬───────────────────────────────┘
                    ▼
                 publish ×3
```

```
                        ┌─────────────────────┐
   /cloud/raw  ────────►│ ApproximateTime     │   (slop = 100 ms)
                        │   Synchronizer      │
   /camera0/undistort ─►│                     │
                        └──────────┬──────────┘
                                   ▼
   ┌────────────────────────────────────────────────┐
   │ 1. PointCloud2 -> float xyz[]                  │
   │ 2. depth_core::pointcloud_to_depth (low-res)   │
   │      - Perspective project: u,v = K * Tcl * P  │
   │      - Keep points with z in [z_min, z_max]    │
   │      - 3x3 dilation (dilate_radius) to fill    │
   │      - Optional z-buffer (use_z_buffer)        │
   │ 3. cv::resize INTER_NEAREST -> full-res depth  │
   │ 4. Sobel edge suppression on full-res depth    │
   │ 5. JET colormap (cv::applyColorMap, SIMD)      │
   │ 6. addWeighted blend with RGB                  │
   └────────────────┬───────────────────────────────┘
                    ▼
                publish x 3
```

---

## 7. 启动 / Running

**和主驱动一起启动（推荐） / Launched together with the driver (recommended)**：

```bash
source install/setup.bash
ros2 launch odin_ros_driver_rev1 driver.launch.py
# or ROS1:
roslaunch odin_ros_driver_rev1 driver.launch
```

**单独运行 / Stand-alone**：

```bash
# ROS2
ros2 run odin_ros_driver_rev1 pointcloud2depth_node

# ROS1
rosrun odin_ros_driver_rev1 pointcloud2depth_node
```

主驱动的 `device_online` 和 `resolution_change` topic 都是 **TRANSIENT_LOCAL**，所以本节点**先启动或后启动**都能正确拿到设备前缀和当前分辨率。
The driver's `device_online` and `resolution_change` topics are **TRANSIENT_LOCAL**, so this node works correctly **regardless of launch order**.

---

## 8. 性能 / Performance

**首帧 latency 自动打印一次 / The first-frame latency is logged once**:

```
[pointcloud2depth_node]: First-frame pipeline latency: 39.112 ms (n_pts=49152, 1280x1088)
```

测量范围：同步回调入口 → 三张图都 publish 完成（含 cv_bridge 序列化）。
Measured from the synchronized callback entry to the moment all three publishes return (includes cv_bridge serialization).

| 阶段 / Stage                         | 大约耗时 / Approx cost | 备注 / Notes |
|-------------------------------------|-----------------------:|---------------|
| PointCloud2 解包 / unpack            | < 1 ms                | 49k 点 xyz 提取 |
| `pointcloud_to_depth` (低分辨率)     | < 1 ms                | 183×155 画布 / Low-res canvas |
| `cv::resize` NN 到 1280×1088         | ~2-3 ms               | 全分辨率上采样 / Full-res upsample |
| Sobel 边缘抑制（全分辨率）/ on full-res | ~10-15 ms            | 主要 CPU 开销 / Main CPU cost |
| `makeDepthJet` (向量化)              | ~1-2 ms               | minMaxLoc + convertTo + applyColorMap |
| `blendJetOnImage` (addWeighted)      | ~1 ms                 | SIMD |
| 3× publish (cv_bridge + DDS)         | ~5-10 ms              | 每张 4-5 MB / 4-5 MB per image |
| **合计 / Total**                     | **~25-40 ms**         |               |

10 Hz / 14.5 Hz 设备帧率下都还有充足预算 (100 ms / 69 ms)。
Plenty of headroom at the device rates of 10 Hz / 14.5 Hz (budgets 100 ms / 69 ms).

### 调优开关 / Tuning knobs

- **降帧率压力**：`scale` 调大（如 9 或 10） → 投影画布变小、Sobel 之外的代价基本不变（Sobel 仍跑在全分辨率）。
- **去掉中间可视化**：如果不需要 `depth_color`，可以在 launch 里 remap 掉，但当前节点总会发布（未来可加 `enable_depth_color` 参数）。
- **更激进的边缘抑制**：把 `edge_grad_threshold` 调小（如 0.4），断层处擦除更彻底；调大或设为 0 关闭。
- **Lower frame-rate pressure**: increase `scale` (e.g. 9 or 10) → smaller projection canvas; Sobel still runs on full-res so the rest of the cost is largely unchanged.
- **Skip a visualization**: `depth_color` is always published; future work can add an `enable_depth_color` flag. For now remap it away in the launch file.
- **Aggressive edge suppression**: lower `edge_grad_threshold` (e.g. 0.4) for a more thorough wipeout of streaks; raise it or set to 0 to disable entirely.

---

## 9. 实现注意 / Implementation Notes

### 9.1 Late-join 安全网 / Late-join safety net

启动顺序可能是「先 pointcloud2depth，后主驱动」、「同时启动」、或「主驱动出了图再启动 pointcloud2depth」。前两种情形依赖 `TRANSIENT_LOCAL` 的 `resolution_change` 回放。第三种情形里，节点同步回调里**直接拿同步图像的 `cols/rows` 作为权威分辨率**，与内部状态不一致时立刻重建 `Kcl`，避免因 `resolution_change` 错过而卡在 YAML 默认分辨率。

Launch order may be: pointcloud2depth-first, simultaneous, or driver-first-then-pointcloud2depth. The first two rely on TRANSIENT_LOCAL replay of `resolution_change`. The third case is covered by the safety net inside the synchronized callback: it treats the **incoming image's `cols/rows` as the ground truth** and rebuilds `Kcl` on mismatch, so the node is never stuck on YAML defaults even if `resolution_change` is missed.

### 9.2 Sobel 必须在全分辨率上做 / Sobel must run on full-res

历史上尝试过把 Sobel 移到 upsample 之前以省 ~49× 的计算量，但 NN-upsample 会把低分辨率的 1-2 px 边缘 mask 复制成 `scale × scale` 像素块（在 1280×1088 上 = 14 px），视觉上深度断层处的「黑色裂缝」会明显变宽。当前实现选择保留全分辨率 Sobel 以保证视觉效果。
We have tried moving Sobel before the NN-upsample to save ~49× CPU. However NN-upsample turns a 1–2 px low-res edge mask into a `scale × scale` block on full-res (14 px at scale=7), which visibly widens the "black gaps" along depth discontinuities. The current implementation keeps the Sobel on full-res to preserve visual fidelity.

### 9.3 QoS / 持久性

- `cloud/raw` / `camera0/undistort`：**VOLATILE** (默认的 sensor data QoS)
- `device_online` / `resolution_change`：**TRANSIENT_LOCAL**（与主驱动 publisher 对齐）

注意 `resolution_change` 的发布端在主驱动里有**两个**（HotplugManager + 每设备的 `OnImagePacket`），都已统一为 TL，否则会出现 `incompatible QoS ... DURABILITY_QOS_POLICY` 警告。
There are **two** `resolution_change` publishers on the driver side (HotplugManager + per-device `OnImagePacket`). Both must be TRANSIENT_LOCAL to avoid `incompatible QoS ... DURABILITY_QOS_POLICY` warnings.

---

## 10. 后续可做的工作 / Future Work

- 把 `pointcloud2depth_node.hpp` 下沉到设备端 SDK，host 端只接收已渲染好的低分辨率深度图。
- 发布线程化：把 cv_bridge + DDS 推送放到后台线程，主线程立刻返回，单帧 latency 可压到 ~5 ms。
- 跳过 `depth_pub_`（5.5 MB/帧的 32FC1 是带宽大头），按需启用。
- 暴露 `enable_depth_color` / `enable_depth_overlay` 开关，节省不需要可视化场景下的发布开销。

- Sink `pointcloud2depth_node.hpp` into the device-side SDK so the host only consumes a pre-rendered low-res depth map.
- Threaded publish: move cv_bridge + DDS submit to a background thread; main thread returns immediately, target latency ~5 ms.
- Skip `depth_pub_` (5.5 MB/frame 32FC1 dominates bandwidth) when not needed.
- Expose `enable_depth_color` / `enable_depth_overlay` to save publish cost when visualizations are unused.

---

## 11. 历史 / Changelog

| 日期 / Date | 改动 / Change |
|------------|---------------|
| 2026-05-30 | 模块从 `src/depth_pipeline_verify_node.cpp` 重命名为 `pointcloud2depth_node`，移入独立子目录 `src/pointcloud2depth/`，topic 后缀从 `depth_verify` 改为 `pointcloud2depth`。/ Renamed and moved into its own subfolder; topic suffix changed from `depth_verify` to `pointcloud2depth`. |
| 2026-05-30 | `makeDepthJet` / `blendJetOnImage` 改为 OpenCV 向量化实现（无视觉差异）；首帧 latency 一次性日志加入。/ Vectorized JET and blend (visually identical); one-shot first-frame latency log added. |
| 2026-05-30 | `resolution_change` publisher / subscriber 全部对齐到 `TRANSIENT_LOCAL`，消除 QoS 警告并支持任意启动顺序。/ All `resolution_change` pubs/subs aligned to TRANSIENT_LOCAL. |
| 2026-05-30 | 重构为多设备架构：`Pointcloud2DepthNode` 仅作管理器，每个设备一份 `DeviceWorker`，由 `device_online` / `device_offline` 驱动生灭；`resolution_change` 按 SN 分发。`depth_pipeline_core.hpp` 合并进 `pointcloud2depth_node.hpp`，旧文件 `src/depth_pipeline_verify_node.cpp` 已删除。/ Refactored to multi-device architecture: `Pointcloud2DepthNode` is now a manager that spawns/destroys one `DeviceWorker` per device on `device_online`/`device_offline`; `resolution_change` is dispatched by SN. `depth_pipeline_core.hpp` merged into `pointcloud2depth_node.hpp`; the legacy `src/depth_pipeline_verify_node.cpp` has been removed. |
