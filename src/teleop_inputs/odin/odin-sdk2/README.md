# Odin SDK

与 Odin 雷达通信的 C++ 静态库，提供设备发现、连接管理、数据接收、指令发送和安全文件传输功能。

## 功能特性

- **设备发现**：广播发现局域网内的 Odin 设备
- **多设备连接**：支持同时连接多台雷达
- **数据接收**：点云、图像、IMU、里程计等多种数据类型
- **指令发送**：模式切换、固件升级、文件传输等
- **SLAM/Odom 同步**：支持按帧号同步 SLAM 点云和里程计数据
- **HTTPS 安全传输**：基于 mTLS 的加密文件传输（固件升级、标定文件等）
- **热插拔监听**：支持设备热插拔事件监听

## 系统要求

- Ubuntu 20.04 / 22.04
- C++17 编译器

## 依赖安装

```bash
# 编译工具
sudo apt install build-essential cmake

# SDK 依赖库
sudo apt install libeigen3-dev libssl-dev
```

## 编译与安装

```bash
./build.sh
```

默认安装到 `build/install/` 目录：
- `build/install/lib/libodin_sdk.a` - 静态库
- `build/install/include/odin_lidar_api.h` - API 头文件
- `build/install/include/odin_lidar_def.h` - 数据类型定义

自定义安装路径：
```bash
./build.sh /your/custom/path
# 或
INSTALL_PREFIX=/your/custom/path ./build.sh
```

## 目录结构

```
sdk_api/
├── include/                    # 公开头文件
│   ├── odin_lidar_api.h        # SDK API 接口定义
│   └── odin_lidar_def.h        # 数据类型定义
├── sdk/                        # SDK 核心实现
│   ├── src/                    # SDK 主实现
│   ├── device/                 # 设备抽象层
│   ├── discovery/              # 设备发现模块
│   ├── protocol/               # 通信协议
│   ├── transport/              # 传输层
│   ├── file_transfer/          # 文件传输模块 (HTTPS)
│   ├── hotplugListener/        # 热插拔监听
│   ├── utils/                  # 工具库
│   │   └── crypto/             # 加密模块 (TLS/签名)
│   └── logger/                 # 日志模块
├── sample/                     # 示例程序
│   ├── hotplug/                # 热插拔监听示例
│   ├── getCalibration/         # 获取标定文件示例
│   ├── sendCalibration/        # 发送标定文件示例
│   ├── upgrade/                # 固件升级示例 (HTTPS OTA)
│   └── https_client/           # HTTPS 客户端示例
├── 3rdparty/                   # 第三方库
│   ├── FastCRC/                # CRC 校验库
│   └── mongoose/               # HTTP/HTTPS 网络库
├── docs/                       # 内部文档
├── build.sh                    # 编译脚本
└── CMakeLists.txt              # CMake 配置
```

## SDK 使用流程

```
┌─────────────────────────────────────────────────────────────────────┐
│                          Application                                 │
└─────────────────────────────────────────────────────────────────────┘
                                  │
        ┌─────────────────────────┴─────────────────────────┐
        ▼                                                   ▼
┌───────────────┐                                   ┌───────────────┐
│ DiscoverDevices│ (主动发现)                        │ HotplugListener│ (热插拔监听)
└───────┬───────┘                                   └───────┬───────┘
        │                                                   │
        └─────────────────────────┬─────────────────────────┘
                                  ▼
                    ┌─────────────────────────┐
                    │    ConnectDevice()      │
                    │  获取设备句柄 (Handle)   │
                    └───────────┬─────────────┘
                                │
        ┌───────────────────────┼───────────────────────┐
        ▼                       ▼                       ▼
┌───────────────┐     ┌─────────────────┐     ┌─────────────────┐
│ RegisterXxxCb │     │ SendSetModeCmd  │     │ SendFileToDevice│
│ 注册数据回调   │     │ 设置工作模式     │     │ 文件传输 (HTTPS)│
└───────────────┘     └────────┬────────┘     └─────────────────┘
                               │
                               ▼
                    ┌─────────────────────┐
                    │ kNormal / kStandby  │
                    │   开启/关闭数据流    │
                    └─────────┬───────────┘
                              │
                              ▼
            ┌─────────────────┼─────────────────┐
            ▼                 ▼                 ▼
    ┌─────────────┐   ┌─────────────┐   ┌─────────────┐
    │ PointCloud  │   │ Image/SLAM  │   │  IMU/Odom   │
    │   Callback  │   │  Callback   │   │  Callback   │
    └─────────────┘   └─────────────┘   └─────────────┘
```

## 架构说明

```
┌─────────────────────────────────────────────────────────────────────┐
│                          Application                                 │
├─────────────────────────────────────────────────────────────────────┤
│                        odin_lidar_api.h                              │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌─────────┐  ┌────────┐  │
│  │ Discover │  │ Hotplug  │  │ Connect  │  │Callback │  │ HTTPS  │  │
│  └──────────┘  └──────────┘  └──────────┘  └─────────┘  └────────┘  │
├─────────────────────────────────────────────────────────────────────┤
│                          OdinSdkImpl                                 │
│  ┌───────────────────────────────────────────────────────────────┐  │
│  │                    Device Instance (Handle)                    │  │
│  │  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌───────────────┐  │  │
│  │  │ Protocol │  │ Transport│  │FrameAsm  │  │ FileTransfer  │  │  │
│  │  │ 协议解析  │  │ UDP通信   │  │帧组装/CRC │  │ HTTPS/Crypto │  │  │
│  │  └──────────┘  └──────────┘  └──────────┘  └───────────────┘  │  │
│  └───────────────────────────────────────────────────────────────┘  │
├─────────────────────────────────────────────────────────────────────┤
│                    UDP Socket / HTTPS (mTLS)                         │
└─────────────────────────────────────────────────────────────────────┘
                                  │
                                  ▼
                         ┌───────────────┐
                         │  Odin Device  │
                         └───────────────┘
```

## 示例程序

### 热插拔监听
```bash
./build/odin_hotplug
```
- 自动监听设备上下线事件
- 设备连接后自动开始数据流

### 固件升级 (OTA)
```bash
./build/odin_upgrade <firmware_path> [--device <ip>] [--timeout <ms>]
```
- `firmware_path`：固件文件路径 (.bin)
- `--device`：可选，指定设备 IP
- `--timeout`：可选，发现超时

### 获取标定文件
```bash
./build/odin_get_calibration [--device <ip>] [--output <path>]
```

### 发送标定文件
```bash
./build/odin_send_calibration <calibration_file> [--device <ip>]
```

## API 文档

详见头文件 `include/odin_lidar_api.h` 中的注释说明。

## 数据类型

SDK 支持以下数据类型（定义在 `odin_lidar_def.h`）：

| 类型 | 枚举值 | 说明 |
|------|--------|------|
| `kPointCloud` | 0 | 原始点云数据 |
| `kSlamPointCloud` | 1 | SLAM 点云数据 |
| `kJpegImage` | 2 | 第二路 MJPEG 图像 |
| `kImu` | 3 | IMU 数据 |
| `kOdom` | 4 | 里程计数据 |
| `kFile` | 5 | 文件传输 |
| `kGrayJpegImage` | 6 | 灰度 JPEG 图像 |

## 安全特性

- **mTLS 双向认证**：客户端和设备端相互验证证书
- **文件签名**：固件和重要文件使用 Ed25519 签名验证
- **加密传输**：所有文件传输通过 HTTPS 加密
