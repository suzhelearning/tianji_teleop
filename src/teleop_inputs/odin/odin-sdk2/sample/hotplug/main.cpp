/**
 * @file main.cpp
 * @brief Multi-device hotplug demo with frame rate statistics
 *
 * This sample demonstrates:
 * - Hotplug detection (device arrival/removal)
 * - Multi-device support
 * - Sensor capability query
 * - Stream control (StartStream/CloseStream)
 * - Frame rate statistics for each data type
 * - Automatic resource cleanup on device removal
 * - Re-connection when device comes back online
 */

#include "odin_lidar_api.h"
#include "logger.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace odin::sdk;

// =============================================================================
// Configuration
// =============================================================================

constexpr int kFpsUpdateIntervalMs = 1000;  // FPS update interval

// =============================================================================
// Data Structures
// =============================================================================

struct FrameStats {
  std::atomic<uint64_t> frame_count{0};
  std::chrono::steady_clock::time_point last_update;
  double fps{0.0};

  FrameStats() : last_update(std::chrono::steady_clock::now()) {}

  void Reset() {
    frame_count.store(0);
    last_update = std::chrono::steady_clock::now();
    fps = 0.0;
  }
};

struct DeviceContext {
  DiscoveredDevice device;
  OdinDeviceHandle handle{kInvalidDeviceHandle};
  bool connected{false};

  // Frame statistics for each data type
  FrameStats raw_stats;
  FrameStats slam_stats;
  FrameStats image_stats;
  FrameStats imu_stats;
  FrameStats odom_stats;
  FrameStats image2_stats;

  std::chrono::steady_clock::time_point connect_time;
};

// =============================================================================
// Global State
// =============================================================================

std::atomic<bool> g_running{true};
std::mutex g_devices_mutex;
std::map<std::string, std::unique_ptr<DeviceContext>> g_devices;  // key = SN

void SignalHandler(int) { g_running.store(false); }

// =============================================================================
// Utility Functions
// =============================================================================

std::string GetTimestamp() {
  auto now = std::chrono::system_clock::now();
  auto time = std::chrono::system_clock::to_time_t(now);
  std::tm tm_info;
  localtime_r(&time, &tm_info);
  char buffer[32];
  strftime(buffer, sizeof(buffer), "%H:%M:%S", &tm_info);
  return buffer;
}

void UpdateFps(FrameStats& stats) {
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - stats.last_update);
  if (elapsed.count() >= kFpsUpdateIntervalMs) {
    uint64_t frames = stats.frame_count.exchange(0);
    stats.fps = static_cast<double>(frames) * 1000.0 / elapsed.count();
    stats.last_update = now;
  }
}

DeviceContext* FindDeviceByHandle(OdinDeviceHandle handle) {
  for (auto& kv : g_devices) {
    if (kv.second->handle == handle) {
      return kv.second.get();
    }
  }
  return nullptr;
}

// =============================================================================
// Callbacks
// =============================================================================

void PointCloudCallback(const OdinPointCloudPacket& packet, void*) {
  std::lock_guard<std::mutex> lock(g_devices_mutex);
  if (auto* ctx = FindDeviceByHandle(packet.device)) {
    ctx->raw_stats.frame_count++;
  }
}

void SlamCallback(const OdinSlamPacket& packet, void*) {
  std::lock_guard<std::mutex> lock(g_devices_mutex);
  if (auto* ctx = FindDeviceByHandle(packet.device)) {
    ctx->slam_stats.frame_count++;
  }
}

void ImageCallback(const OdinImagePacket& packet, void*) {
  std::lock_guard<std::mutex> lock(g_devices_mutex);
  if (auto* ctx = FindDeviceByHandle(packet.device)) {
    ctx->image_stats.frame_count++;
  }
}

void ImageCallback2(const OdinImagePacket& packet, void*) {
  std::lock_guard<std::mutex> lock(g_devices_mutex);
  if (auto* ctx = FindDeviceByHandle(packet.device)) {
    ctx->image2_stats.frame_count++;
  }
}

void ImuCallback(const OdinImuPacket& packet, void*) {
  std::lock_guard<std::mutex> lock(g_devices_mutex);
  if (auto* ctx = FindDeviceByHandle(packet.device)) {
    ctx->imu_stats.frame_count++;
  }
}

void OdomCallback(const OdinOdomPacket& packet, OdomSourceType odom_type, void*) {
  (void)odom_type;  // Unused
  std::lock_guard<std::mutex> lock(g_devices_mutex);
  if (auto* ctx = FindDeviceByHandle(packet.device)) {
    ctx->odom_stats.frame_count++;
  }
}

// =============================================================================
// Device Management
// =============================================================================

// Helper function to convert channel to string
const char* ChannelToString(OdinDataChannel ch) {
  switch (ch) {
    case OdinDataChannel::kRawPoint:  return "RawPoint";
    case OdinDataChannel::kSlamPoint: return "SlamPoint";
    case OdinDataChannel::kImage0:    return "Image0";
    case OdinDataChannel::kImage1:    return "Image1";
    case OdinDataChannel::kImu:       return "IMU";
    case OdinDataChannel::kOdom:      return "Odom";
    default: return "Unknown";
  }
}

// Helper function to convert format to string
const char* FormatToString(OdinDataFormat fmt) {
  switch (fmt) {
    case OdinDataFormat::kImageMjpeg:       return "MJPEG";
    case OdinDataFormat::kImageYuyv:        return "YUYV";
    case OdinDataFormat::kImageNv12:        return "NV12";
    case OdinDataFormat::kImageNv21:        return "NV21";
    case OdinDataFormat::kImageRgb24:       return "RGB24";
    case OdinDataFormat::kRawPointXyzic:    return "XYZIC";
    case OdinDataFormat::kRawPointXyz:      return "XYZ";
    case OdinDataFormat::kSlamPointXyzrgba: return "XYZRGBA";
    case OdinDataFormat::kSlamPointXyz:     return "XYZ";
    case OdinDataFormat::kImu6Axis:         return "6-Axis";
    case OdinDataFormat::kImu9Axis:         return "9-Axis";
    case OdinDataFormat::kOdomStandard:     return "Standard";
    default: return "Unknown";
  }
}

void PrintSensorCapabilities(const std::vector<OdinSensorCapability>& capabilities) {
  std::cout << "\n  ╔════════════════════════════════════════════════════════╗\n";
  std::cout << "  ║              SENSOR CAPABILITIES                       ║\n";
  std::cout << "  ╠════════════════════════════════════════════════════════╣\n";

  for (const auto& cap : capabilities) {
    std::cout << "  ║  Channel: " << std::left << std::setw(12) << ChannelToString(cap.channel);
    std::cout << "                              ║\n";
    std::cout << "  ║  ┌──────────────────────────────────────────────────┐ ║\n";
    std::cout << "  ║  │  Mode   Width  Height   FPS   Format            │ ║\n";
    std::cout << "  ║  ├──────────────────────────────────────────────────┤ ║\n";

    int mode_idx = 0;
    for (const auto& cfg : cap.modes) {
      std::cout << "  ║  │  [" << std::setw(2) << mode_idx++ << "]  ";
      if (cfg.width > 0) {
        std::cout << std::setw(5) << cfg.width << "  ";
      } else {
        std::cout << "  -    ";
      }
      if (cfg.height > 0) {
        std::cout << std::setw(5) << cfg.height << "  ";
      } else {
        std::cout << "  -    ";
      }
      if (cfg.fps > 0) {
        std::cout << std::setw(4) << cfg.fps << "  ";
      } else {
        std::cout << " -   ";
      }
      std::cout << std::left << std::setw(16) << FormatToString(cfg.format);
      std::cout << " │ ║\n";
    }

    if (cap.modes.empty()) {
      std::cout << "  ║  │  (No configurations available)                   │ ║\n";
    }

    std::cout << "  ║  └──────────────────────────────────────────────────┘ ║\n";
  }

  if (capabilities.empty()) {
    std::cout << "  ║  (No sensor capabilities reported by device)          ║\n";
  }

  std::cout << "  ╚════════════════════════════════════════════════════════╝\n\n";
}

void ConnectAndRegister(DeviceContext* ctx) {
  std::cout << "[" << GetTimestamp() << "] Connecting to device: " << ctx->device.network.ip_address
            << " (SN: " << ctx->device.sn << ")" << std::endl;

  ctx->handle = ConnectDevice(ctx->device);
  if (ctx->handle == kInvalidDeviceHandle) {
    std::cerr << "[" << GetTimestamp() << "] Failed to connect to device: " << ctx->device.sn
              << std::endl;
    return;
  }

  ctx->connected = true;
  ctx->connect_time = std::chrono::steady_clock::now();

  // Register all callbacks
  RegisterPointCloudCallback(ctx->handle, PointCloudCallback, nullptr);
  RegisterSlamCallback(ctx->handle, SlamCallback, nullptr);
  RegisterImageCallback(ctx->handle, ImageCallback, nullptr);
  RegisterImageCallback2(ctx->handle, ImageCallback2, nullptr);
  RegisterImuCallback(ctx->handle, ImuCallback, nullptr);
  RegisterOdomCallback(ctx->handle, OdomCallback, nullptr);

  // Get and print firmware version
  std::string version;
  if (GetFirmwareVersion(ctx->handle, version) == OdinResult::kOk) {
    std::cout << "\n";
    std::cout << "========================================\n";
    std::cout << "  FIRMWARE VERSION: " << version << "\n";
    std::cout << "========================================\n";
    std::cout << "\n";
  }

  // Query and print sensor capabilities
  std::cout << "[" << GetTimestamp() << "] Querying sensor capabilities..." << std::endl;
  std::vector<OdinSensorCapability> capabilities;
  auto cap_result = GetSensorCapability(ctx->handle, capabilities, {}, 2000);
  if (cap_result == OdinResult::kOk) {
    PrintSensorCapabilities(capabilities);
  } else {
    std::cerr << "[" << GetTimestamp() << "] Failed to query capabilities, result="
              << static_cast<int>(cap_result) << std::endl;
  }

  // Set to standby mode first for stream configuration
  if (SetOperatingMode(ctx->handle, OdinOperatingMode::kStandby) != OdinResult::kOk) {
    std::cerr << "[" << GetTimestamp() << "] Failed to set STANDBY mode" << std::endl;
  }

  // Start all data streams with UDP transport
  std::cout << "[" << GetTimestamp() << "] Starting data streams..." << std::endl;

  auto start_stream = [&](OdinDataChannel channel, const char* name) {
    auto result = StartStream(ctx->handle, channel, OdinTransportMode::kUdp);
    if (result == OdinResult::kOk) {
      std::cout << "  [OK] " << name << " stream started" << std::endl;
    } else {
      std::cerr << "  [FAIL] " << name << " stream failed, result="
                << static_cast<int>(result) << std::endl;
    }
  };

  start_stream(OdinDataChannel::kRawPoint, "RawPoint");
  start_stream(OdinDataChannel::kSlamPoint, "SlamPoint");
  start_stream(OdinDataChannel::kImage0, "Image0");
  start_stream(OdinDataChannel::kImage1, "Image1");
  start_stream(OdinDataChannel::kImu, "IMU");
  start_stream(OdinDataChannel::kOdom, "Odom");

  // Set to normal mode to start streaming
  if (SetOperatingMode(ctx->handle, OdinOperatingMode::kNormal) == OdinResult::kOk) {
    std::cout << "[" << GetTimestamp() << "] Device " << ctx->device.sn
              << " set to NORMAL mode, streaming started" << std::endl;
  } else {
    std::cerr << "[" << GetTimestamp()
              << "] Failed to set NORMAL mode for device: " << ctx->device.sn << std::endl;
  }
}

void DisconnectAndRelease(DeviceContext* ctx) {
  if (!ctx->connected) return;

  std::cout << "[" << GetTimestamp() << "] Disconnecting device: " << ctx->device.network.ip_address
            << " (SN: " << ctx->device.sn << ")" << std::endl;

  if (ctx->handle != kInvalidDeviceHandle) {
    // Set to standby mode before closing streams
    SetOperatingMode(ctx->handle, OdinOperatingMode::kStandby);

    // Close all data streams
    CloseStream(ctx->handle, OdinDataChannel::kRawPoint);
    CloseStream(ctx->handle, OdinDataChannel::kSlamPoint);
    CloseStream(ctx->handle, OdinDataChannel::kImage0);
    CloseStream(ctx->handle, OdinDataChannel::kImage1);
    CloseStream(ctx->handle, OdinDataChannel::kImu);
    CloseStream(ctx->handle, OdinDataChannel::kOdom);

    DisconnectDevice(ctx->handle);
    ctx->handle = kInvalidDeviceHandle;
  }

  // Reset all stats
  ctx->raw_stats.Reset();
  ctx->slam_stats.Reset();
  ctx->image_stats.Reset();
  ctx->imu_stats.Reset();
  ctx->odom_stats.Reset();
  ctx->image2_stats.Reset();

  ctx->connected = false;

  std::cout << "[" << GetTimestamp() << "] Device " << ctx->device.sn << " resources released"
            << std::endl;
}

// =============================================================================
// Hotplug Event Handlers
// =============================================================================

void OnDeviceAttach(const DiscoveredDevice& device) {
  std::string key = device.sn.empty() ? device.network.ip_address : device.sn;

  std::lock_guard<std::mutex> lock(g_devices_mutex);

  std::cout << "\n[" << GetTimestamp() << "] === DEVICE ARRIVED ===" << std::endl;
  std::cout << "  IP: " << device.network.ip_address << std::endl;
  std::cout << "  SN: " << device.sn << std::endl;
  std::cout << "  Model: " << device.model << std::endl;
  std::cout << "  Firmware: " << device.firmware_version << std::endl;

  if (g_devices.find(key) != g_devices.end()) {
    // Device already exists (shouldn't happen since we erase on removal)
    return;
  }
  auto ctx = std::unique_ptr<DeviceContext>(new DeviceContext());
  ctx->device = device;
  ConnectAndRegister(ctx.get());
  g_devices[key] = std::move(ctx);

  std::cout << "[" << GetTimestamp() << "] Total devices online: " << g_devices.size() << std::endl;
}

void OnDeviceDetach(const DiscoveredDevice& device) {
  std::string key = device.sn.empty() ? device.network.ip_address : device.sn;

  std::cout << "\n[" << GetTimestamp() << "] === DEVICE REMOVED ===" << std::endl;
  std::cout << "  IP: " << device.network.ip_address << std::endl;
  std::cout << "  SN: " << device.sn << std::endl;

  // Move device out of map first, then disconnect outside lock to avoid deadlock
  // (callbacks hold g_devices_mutex, Stop() waits for callback thread to exit)
  std::unique_ptr<DeviceContext> ctx_to_disconnect;
  size_t remaining_devices = 0;
  {
    std::lock_guard<std::mutex> lock(g_devices_mutex);
    auto it = g_devices.find(key);
    if (it != g_devices.end()) {
      ctx_to_disconnect = std::move(it->second);
      g_devices.erase(it);
    }
    remaining_devices = g_devices.size();
  }
  // Now disconnect without holding the lock
  if (ctx_to_disconnect) {
    DisconnectAndRelease(ctx_to_disconnect.get());
  }

  std::cout << "[" << GetTimestamp() << "] Total devices online: " << remaining_devices
            << std::endl;
}

// =============================================================================
// Statistics Display
// =============================================================================

void PrintStatistics() {
  std::lock_guard<std::mutex> lock(g_devices_mutex);

  if (g_devices.empty()) {
    return;
  }

  // Update FPS for all devices
  for (auto& kv : g_devices) {
    auto* ctx = kv.second.get();
    if (!ctx->connected) continue;

    UpdateFps(ctx->raw_stats);
    UpdateFps(ctx->slam_stats);
    UpdateFps(ctx->image_stats);
    UpdateFps(ctx->imu_stats);
    UpdateFps(ctx->odom_stats);
    UpdateFps(ctx->image2_stats);
  }

  // Print header
  std::cout << "\n========== Frame Rate Statistics [" << GetTimestamp() << "] ==========\n";

  for (auto& kv : g_devices) {
    auto* ctx = kv.second.get();
    if (!ctx->connected) continue;

    auto uptime = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now() - ctx->connect_time);

    std::cout << std::fixed << std::setprecision(1);
    std::cout << "Device: " << ctx->device.sn << " (" << ctx->device.network.ip_address << ")\n";
    std::cout << "  Uptime: " << uptime.count() << "s\n";
    std::cout << "  RAW:    " << std::setw(6) << ctx->raw_stats.fps << " fps | ";
    std::cout << "SLAM:   " << std::setw(6) << ctx->slam_stats.fps << " fps | ";
    std::cout << "IMAGE:  " << std::setw(6) << ctx->image_stats.fps << " fps\n";
    std::cout << "  IMU:    " << std::setw(6) << ctx->imu_stats.fps << " fps | ";
    std::cout << "ODOM:   " << std::setw(6) << ctx->odom_stats.fps << " fps | ";
    std::cout << "IMAGE2: " << std::setw(6) << ctx->image2_stats.fps << " fps\n";
  }

  std::cout << "============================================================\n";
}

// =============================================================================
// Main
// =============================================================================

void PrintUsage(const char* exe) {
  std::cout << "Usage: " << exe << "\n"
            << "\n"
            << "Multi-device hotplug demo with frame rate statistics.\n"
            << "The program will automatically detect devices as they come online/offline.\n"
            << "\n"
            << "Press Ctrl+C to exit.\n";
}

int main(int argc, char* argv[]) {
  if (argc > 1 && (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help")) {
    PrintUsage(argv[0]);
    return 0;
  }

  // Setup signal handler
  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);

  // Initialize logger
  log_config(LOG_LEVEL_INFO, printf);

  std::cout << "\n";
  std::cout << "========================================\n";
  std::cout << "  SDK VERSION: " << GetSdkVersion() << "\n";
  std::cout << "========================================\n";
  std::cout << "\n";
  std::cout << "Multi-Device Hotplug Demo\n";
  std::cout << "Starting hotplug listener...\n";
  std::cout << "Waiting for devices to come online...\n";
  std::cout << "Press Ctrl+C to exit.\n\n";

  // Start hotplug listener
  HotplugCallbacks callbacks;
  callbacks.on_attach = OnDeviceAttach;
  callbacks.on_detach = OnDeviceDetach;
  if (!StartHotplugListener(callbacks, true)) {
    std::cerr << "Failed to start hotplug listener!" << std::endl;
    return 1;
  }

  // Main loop - print statistics periodically
  auto last_print = std::chrono::steady_clock::now();
  while (g_running.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_print);
    if (elapsed.count() >= 2) {  // Print every 2 seconds
      PrintStatistics();
      last_print = now;
    }
  }

  std::cout << "\nShutting down...\n";

  // Stop hotplug listener
  StopHotplugListener();

  // Disconnect all devices - collect first, then disconnect outside lock to avoid deadlock
  // (callbacks hold g_devices_mutex, Stop() waits for callback thread to exit)
  std::vector<std::unique_ptr<DeviceContext>> devices_to_disconnect;
  {
    std::lock_guard<std::mutex> lock(g_devices_mutex);
    for (auto& kv : g_devices) {
      devices_to_disconnect.push_back(std::move(kv.second));
    }
    g_devices.clear();
  }
  // Now disconnect without holding the lock
  for (auto& ctx : devices_to_disconnect) {
    DisconnectAndRelease(ctx.get());
  }

  std::cout << "Cleanup complete. Goodbye!\n";
  return 0;
}
