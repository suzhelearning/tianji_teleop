# PICO Streaming Server 使用文档

## 概述

`PicoStreamingServer` 通过 USB 有线（adb forward）将 PICO 4U 的以下数据实时流式传输到 PC：

- 双目 RGB 相机视频流（左/右，640×480，MJPEG）
- 左右手柄 + 头显 6DOF 位姿（位置 + 四元数）
- 蓝牙设备数据（灵巧手 / UMI 夹爪磁编码器）

---

## 快速开始

### 1. Unity 端配置

1. 在 Unity Inspector 中**禁用** `PicoDualCameraRecorderWithBluetooth` 组件
2. **启用** `PicoStreamingServer` 组件
3. 配置参数：

| 参数 | 说明 | 默认值 |
|------|------|--------|
| TCP Port | 监听端口 | 9999 |
| Record Width/Height | 相机分辨率 | 640×480 |
| Record FPS | 目标帧率 | 30fps |
| Jpg Quality | JPEG 压缩质量（1-100） | 60 |
| Max Queue Size | 发送队列最大帧数 | 30 |
| Enable Bluetooth | 是否启用蓝牙 | false |
| Ble Mode | Hand（灵巧手）或 UMI（夹爪） | Hand |
| Hand Left/Right Address | 灵巧手 BLE MAC 地址 | 空 |
| UMI Left/Right Address | 夹爪 BLE MAC 地址 | 空 |

4. Build APK 并安装到 PICO 4U

### 2. 建立 adb 连接

PICO 通过 USB 连接 PC，在 PC 终端执行：

```bash
# 建立端口转发
adb forward tcp:9999 tcp:9999

# 验证转发是否成功
adb forward --list
# 应输出：<serial> tcp:9999 tcp:9999
```

### 3. 运行 Python 工具

```bash
# 安装依赖
pip install opencv-python numpy matplotlib scipy

# 数据接收 + 终端打印（相机/位姿/BLE）
python receiver.py

# 6DOF 实时 3D 可视化
python visualizer.py
```

### 4. 重置世界坐标系

在 PICO 头显中**按右手柄 A 键**，将当前头显位置设为世界原点，坐标轴水平对齐（Z 轴严格向上）。按下后 PC 端可视化界面的世界原点坐标轴会同步更新。

---

## 状态面板（HMD 内 UI）

脚本运行后，右手柄旁会出现一个 World Space 悬浮面板，实时显示各模块连接状态。

```
BLE L:OK  R:X
Data L:OK  R:X
PC: 等待连接
```

| 字段 | 说明 |
|------|------|
| `BLE L` / `BLE R` | 左/右 ESP32 蓝牙连接状态：`OK`=已连接，`..`=连接中，`X`=未连接 |
| `Data L` / `Data R` | 左/右 BLE 数据接收状态：`OK`=2秒内收到数据，`X`=无数据 |
| `PC` | TCP 客户端连接状态：`已连接` / `等待连接` |

> 面板跟随右手柄偏移 (-10cm, +2cm)，每 0.5 秒刷新一次。

---

## 数据协议

### TCP 帧格式

每个数据帧的二进制格式：

```
[0xAB][Type:1B][TimestampMs:8B][PayloadLen:4B][Payload:N B]
```

| 字段 | 大小 | 说明 |
|------|------|------|
| Magic | 1 字节 | 固定 `0xAB`，用于帧同步 |
| Type | 1 字节 | 数据类型（见下表） |
| TimestampMs | 8 字节 | 相对录制开始的毫秒时间戳，小端序 int64 |
| PayloadLen | 4 字节 | Payload 长度（字节），小端序 uint32 |
| Payload | N 字节 | 数据内容（见各类型说明） |

### 帧类型

| Type | 值 | Payload 内容 |
|------|----|-------------|
| 左相机 | `0x01` | JPEG 图像数据 |
| 右相机 | `0x02` | JPEG 图像数据 |
| 左手柄 6DOF | `0x03` | 7×float32 = 28 字节（见下） |
| 右手柄 6DOF | `0x04` | 7×float32 = 28 字节 |
| 头显 6DOF | `0x05` | 7×float32 = 28 字节 |
| 世界坐标系重置 | `0x06` | 4 字节，小端序 float，当前 Yaw 角（度） |
| 左/右手柄按键 | `0x07` / `0x08` | 控制器状态原始字节 |
| A 键录制开关 | `0x09` | 1 字节，非 0 表示开始录制 |
| BLE 设备1（左） | `0x10` | 见 BLE Payload 格式 |
| BLE 设备2（右） | `0x11` | 见 BLE Payload 格式 |
| 全身关节 | `0x20`–`0x37` | `0x20 + BodyTrackerRole`，每关节 7×float32 |

全身关节按以下固定顺序组成 24 关节 SMPL body pose：

```text
Pelvis, LEFT_HIP, RIGHT_HIP, SPINE1, LEFT_KNEE, RIGHT_KNEE,
SPINE2, LEFT_ANKLE, RIGHT_ANKLE, SPINE3, LEFT_FOOT, RIGHT_FOOT,
NECK, LEFT_COLLAR, RIGHT_COLLAR, HEAD, LEFT_SHOULDER,
RIGHT_SHOULDER, LEFT_ELBOW, RIGHT_ELBOW, LEFT_WRIST,
RIGHT_WRIST, LEFT_HAND, RIGHT_HAND
```

`pico_bridge` 收齐同一时间戳的 24 个关节后，将 APK 原始头部原点数据发布到
`/pico/smpl_raw`（`geometry_msgs/PoseArray`）。按 A 后，地面归零节点等待 30 帧
稳定双脚数据，把脚底设为 `Z=0`，再以 `pico_ground` 坐标发布标准话题
`/pico/smpl`。采集文件保存标准话题为 `/states/smpl/pose`，shape 为
`(N, 24, 7)`，最后一维是 `[x,y,z,qx,qy,qz,qw]`。

### 6DOF Payload 格式（28 字节）

```
[pos.x:4B][pos.y:4B][pos.z:4B][rot.x:4B][rot.y:4B][rot.z:4B][rot.w:4B]
```

所有 float 均为小端序（little-endian）。

**坐标系定义（右手系，按 A 键重置后生效）：**

| 轴 | 方向 | 说明 |
|----|------|------|
| X | 前 | 按 A 时头显水平朝向 |
| Y | 左 | |
| Z | 上 | 严格对齐重力反方向 |

> 重置时只保留 Yaw（水平偏转角），Pitch/Roll 归零，使坐标平面始终水平。未按 A 键前输出 Unity 世界坐标（左手系，Y 向上）。

**四元数格式：** `[qx, qy, qz, qw]`，scipy `Rotation.from_quat()` 可直接使用。

### 世界重置帧（0x06）

```
[yaw:4B 小端序 float]
```

PC 端收到此帧时，可视化界面的世界原点坐标轴按该 yaw 角更新朝向。

### BLE Payload 格式（TCP 侧）

```
[esp32_ts:4B][sensor_data:N B]
```

| 字段 | 大小 | 说明 |
|------|------|------|
| esp32_ts | 4 字节，小端序 uint32 | ESP32 内部时间戳，单位毫秒 |
| sensor_data | N 字节 | 传感器原始数据，格式由 ESP32 固件决定 |

---

## BLE 通信协议（ESP32 ↔ PICO）

### 概述

ESP32 与 PICO 之间通过 BLE UART（Nordic UART Service, NUS）通信，PICO 侧仅接收数据，不向 ESP32 发送任何指令。

**BLE 服务 UUID：**

| UUID | 用途 |
|------|------|
| `6E400001-B5B3-F393-E0A9-E50E24DCCA9E` | NUS Service |
| `6E400002-B5B3-F393-E0A9-E50E24DCCA9E` | RX Characteristic（VR→ESP32，Write，当前未使用） |
| `6E400003-B5B3-F393-E0A9-E50E24DCCA9E` | TX Characteristic（ESP32→VR，Notify） |

### ESP32 → PICO 数据包格式（0x0B）

```
[0xAA][0xBB][0xCC][0x0B][Len][esp32_ts:4B][data:Len-4 B][0xDD][0xEE]
```

| 字段 | 大小 | 说明 |
|------|------|------|
| Header | 3 字节 | 固定 `0xAA 0xBB 0xCC` |
| FuncCode | 1 字节 | 固定 `0x0B`（数据推送） |
| Len | 1 字节 | 有效载荷总长度（含 4B 时间戳） |
| esp32_ts | 4 字节，小端序 uint32 | ESP32 内部毫秒时间戳 |
| data | Len-4 字节 | 传感器原始数据 |
| Tail | 2 字节 | 固定 `0xDD 0xEE` |

**完整包长度** = Len + 7 字节

**PICO 侧校验：**
1. `raw[0..2] == 0xAA 0xBB 0xCC`，`raw[3] == 0x0B`
2. `raw.Length == Len + 7`
3. `raw[raw.Length-2] == 0xDD`，`raw[raw.Length-1] == 0xEE`

---

## 可视化工具（visualizer.py）

### 启动

```bash
pip install matplotlib scipy numpy
python visualizer.py
```

### 界面说明

左侧为 3D 视图（RViz 风格深色背景），右侧为实时数据面板。

| 元素 | 说明 |
|------|------|
| 蓝色轨迹 + 圆点 | 左手柄位置历史轨迹（最近 200 帧） |
| 红色轨迹 + 圆点 | 右手柄 |
| 绿色轨迹 + 圆点 | 头显 |
| RGB 三轴箭头 | 各设备当前朝向（X=红/前，Y=绿/左，Z=蓝/上） |
| 白色圆点 + 坐标轴 | 世界原点（按 A 键设定，大尺寸 RGB 三轴） |
| 灰色网格 | 水平地面参考，每 60 帧跟随设备中心移动 |

### 快捷键

| 键 | 功能 |
|----|------|
| `C` | 清除所有设备的轨迹历史 |
| `R` | 以当前设备位置为中心重置视图（±0.8m 固定尺度） |
| `Q` / `Esc` | 退出 |
| 左键拖动 | 旋转视角，同时显示旋转轴圆环（蓝=Z轴/水平，绿=Y轴/俯仰） |

### 坐标尺度

视图范围固定，不自动缩放，避免画面抖动。初始范围：

```
X: [-0.8, 0.8] m
Y: [-0.8, 0.8] m
Z: [-0.1, 1.6] m
```

按 `R` 可将中心重置到当前设备位置。

---

## Python 接收端开发

### 基础接收框架

```python
import socket
import struct

HEADER_SIZE = 14  # 1+1+8+4

def recv_exact(sock, n):
    buf = b''
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("Connection closed")
        buf += chunk
    return buf

def receive_frames(host='localhost', port=9999):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host, port))
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    while True:
        header = recv_exact(sock, HEADER_SIZE)
        magic, frame_type = struct.unpack('BB', header[:2])
        if magic != 0xAB:
            continue

        ts_ms = struct.unpack('<q', header[2:10])[0]
        payload_len = struct.unpack('<I', header[10:14])[0]
        payload = recv_exact(sock, payload_len)

        yield frame_type, ts_ms, payload
```

### 解析各类型数据

```python
import numpy as np
import cv2
from scipy.spatial.transform import Rotation

def parse_jpeg(payload):
    return cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)

def parse_pose(payload):
    """返回 {'pos': [x,y,z], 'rot': [qx,qy,qz,qw]}"""
    f = struct.unpack('<7f', payload[:28])
    return {'pos': np.array(f[:3]), 'rot': np.array(f[3:])}

def parse_world_reset(payload):
    """返回 yaw 角（度），世界坐标系重置时收到"""
    return struct.unpack('<f', payload[:4])[0]

def parse_ble(payload):
    esp32_ts = struct.unpack('<I', payload[:4])[0]
    return esp32_ts, payload[4:]

def quat_to_euler(rot_array):
    """rot_array = [qx,qy,qz,qw]，返回欧拉角 [roll,pitch,yaw]（度）"""
    return Rotation.from_quat(rot_array).as_euler('xyz', degrees=True)

# 使用示例
for frame_type, ts_ms, payload in receive_frames():
    if frame_type == 0x01:
        img_left = parse_jpeg(payload)
    elif frame_type == 0x02:
        img_right = parse_jpeg(payload)
    elif frame_type == 0x03:
        pose = parse_pose(payload)
        euler = quat_to_euler(pose['rot'])
        print(f"[{ts_ms}ms] Left  pos={pose['pos']}  euler={euler}")
    elif frame_type == 0x04:
        pose = parse_pose(payload)
    elif frame_type == 0x05:
        pose = parse_pose(payload)
    elif frame_type == 0x06:
        yaw = parse_world_reset(payload)
        print(f"[{ts_ms}ms] World reset  yaw={yaw:.1f}°")
    elif 0x20 <= frame_type <= 0x37:
        joint_index = frame_type - 0x20
        body_pose[joint_index] = parse_pose(payload)
    elif frame_type in (0x10, 0x11):
        dev = "Left" if frame_type == 0x10 else "Right"
        esp_ts, data = parse_ble(payload)
        print(f"[{ts_ms}ms] BLE {dev} esp_ts={esp_ts}ms data={data.hex()}")
```

### 多线程接收（推荐用于实时处理）

```python
import threading

class PicoReceiver:
    def __init__(self, host='localhost', port=9999):
        self.host = host
        self.port = port
        self.latest = {}
        self.lock = threading.Lock()
        self._running = False

    def start(self):
        self._running = True
        threading.Thread(target=self._recv_loop, daemon=True).start()

    def get(self, frame_type):
        with self.lock:
            return self.latest.get(frame_type)

    def _recv_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((self.host, self.port))
        while self._running:
            try:
                header = recv_exact(sock, 14)
                magic, ftype = struct.unpack('BB', header[:2])
                if magic != 0xAB: continue
                ts = struct.unpack('<q', header[2:10])[0]
                plen = struct.unpack('<I', header[10:14])[0]
                payload = recv_exact(sock, plen)
                with self.lock:
                    self.latest[ftype] = (ts, payload)
            except Exception as e:
                print(f"Recv error: {e}")
                break
```

---

## C++ 接收端开发

```cpp
#include <cstdint>
#include <cstring>
#include <vector>

#pragma pack(push, 1)
struct FrameHeader {
    uint8_t  magic;        // 0xAB
    uint8_t  type;
    int64_t  ts_ms;        // 小端序
    uint32_t payload_len;  // 小端序
};
#pragma pack(pop)

struct Pose6DOF {
    float pos[3];   // x(前), y(左), z(上)
    float rot[4];   // qx, qy, qz, qw
};

bool read_frame(int sock, uint8_t& type, int64_t& ts_ms,
                std::vector<uint8_t>& payload) {
    FrameHeader hdr;
    if (recv(sock, &hdr, sizeof(hdr), MSG_WAITALL) != sizeof(hdr)) return false;
    if (hdr.magic != 0xAB) return false;
    type  = hdr.type;
    ts_ms = le64toh(hdr.ts_ms);
    uint32_t len = le32toh(hdr.payload_len);
    payload.resize(len);
    return recv(sock, payload.data(), len, MSG_WAITALL) == (ssize_t)len;
}

Pose6DOF parse_pose(const std::vector<uint8_t>& payload) {
    Pose6DOF pose;
    memcpy(&pose, payload.data(), sizeof(Pose6DOF));
    return pose;
}

void parse_ble(const std::vector<uint8_t>& payload,
               uint32_t& esp32_ts, const uint8_t*& data, size_t& data_len) {
    memcpy(&esp32_ts, payload.data(), 4);
    esp32_ts = le32toh(esp32_ts);
    data     = payload.data() + 4;
    data_len = payload.size() - 4;
}
```

---

## 常见问题

### Q: `adb forward` 后 Python 连接立刻断开

1. 确认 PICO 上的 APK 已运行
2. `adb devices` 确认设备已连接
3. 重新执行 `adb forward tcp:9999 tcp:9999`
4. 查看 Unity logcat：`adb logcat -s Unity`

### Q: 状态面板显示 `PC: 等待连接` 但 Python 已运行

- 确认执行了 `adb forward tcp:9999 tcp:9999`
- 确认连接的是 `localhost:9999`
- 检查防火墙是否阻止了本地回环连接

### Q: 视频流延迟高

- 降低 `Jpg Quality`（建议 40-60）
- 降低分辨率（如 320×240）
- 确保 USB 3.0 连接

### Q: 6DOF 坐标系混乱 / 欧拉角方向不对

- 必须先按右手柄 A 键重置世界坐标系再使用
- 重置后坐标系：X 前、Y 左、Z 上（右手系，水平对齐）
- 旋转四元数与位置使用相同坐标系，可直接用 scipy `Rotation.from_quat([qx,qy,qz,qw])`

### Q: 可视化界面坐标轴一直抖动

- 视图范围已固定，不自动缩放
- 如果设备走出视野，按 `R` 键以当前位置为中心重置视图

### Q: 状态面板显示 `BLE L:X R:X`

- 确认 Inspector 中 `Enable Bluetooth = true`
- 确认 MAC 地址格式正确（`XX:XX:XX:XX:XX:XX`，大写）
- 查看 logcat：`adb logcat -s Unity | grep BLE`

### Q: BLE 连接显示 `OK` 但 `Data L/R` 显示 `X`

- ESP32 固件未按协议发送 `0x0B` 包
- 检查包头 `0xAA 0xBB 0xCC 0x0B`、包尾 `0xDD 0xEE` 及 `Len` 字段是否正确

---

## 数据流示意图

```
ESP32 (BLE)              PICO 4U                            PC
┌──────────────┐         ┌───────────────────────┐          ┌────────────────────────────┐
│ 0x0B 数据包  │─Notify─►│  BLEHelper.java        │          │  receiver.py               │
│ [AA BB CC 0B │         │  PicoStreamingServer.cs│          │  visualizer.py             │
│  Len ts data │         │                        │          │                            │
│  DD EE]      │         │  BLE ───────────────── ┼──0x10/11─┼──► parse_ble()             │
└──────────────┘         │  相机 SDK ─────────────┼──0x01/02─┼──► parse_jpeg()  cv2.show  │
                         │  XR Input (6DOF) ──────┼──0x03~05─┼──► parse_pose()  3D 可视化 │
                         │  A 键重置坐标系 ────────┼──0x06────┼──► 世界原点更新             │
                         │                        │          │                            │
                         │  TCP Server :9999      │◄─adb─────┤  TCP Client :9999          │
                         └───────────────────────┘  forward  └────────────────────────────┘
```
