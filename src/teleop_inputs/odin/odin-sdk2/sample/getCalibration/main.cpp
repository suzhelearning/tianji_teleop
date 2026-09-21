#include "odin_lidar_api.h"
#include "logger.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

using namespace odin::sdk;

namespace {

constexpr int kDefaultDiscoveryTimeoutMs = 3000;

std::atomic<bool> g_running{true};
std::atomic<bool> g_transfer_done{false};
std::atomic<bool> g_transfer_success{false};

void SignalHandler(int) { g_running.store(false); }

void ProgressCallback(OdinDeviceHandle device, float progress, void* client_data) {
  (void)device;
  (void)client_data;
  // progress is already 0-100
  std::cout << "\rProgress: " << std::fixed << std::setprecision(1) << progress << "%"
            << std::flush;
  if (progress >= 100.0f) {
    std::cout << "\n";
    g_transfer_done.store(true);
    g_transfer_success.store(true);
  }
}

void PrintUsage(const char* program_name) {
  std::cout << "Usage: " << program_name << " <save_path> [options]\n"
            << "\n"
            << "Get calibration file from ODIN device.\n"
            << "\n"
            << "Arguments:\n"
            << "  <save_path>       Path to save the calibration file\n"
            << "\n"
            << "Options:\n"
            << "  --device <ip>     Device IP address (default: first discovered)\n"
            << "  --timeout <ms>    Discovery timeout in ms (default: 3000)\n"
            << "  --help            Show this help message\n"
            << "\n"
            << "Examples:\n"
            << "  " << program_name << " ./calibration.bin\n"
            << "  " << program_name << " ./calib.bin --device 192.168.1.200\n";
}

}  // namespace

int main(int argc, char* argv[]) {
  setvbuf(stdout, NULL, _IOLBF, 0);  // use line buffer
  log_config(LOG_LEVEL_DEBUG, printf);
  if (argc < 2) {
    PrintUsage(argv[0]);
    return 1;
  }

  std::string save_path;
  std::string device_ip;
  int discovery_timeout_ms = kDefaultDiscoveryTimeoutMs;

  // Parse arguments
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      PrintUsage(argv[0]);
      return 0;
    } else if (arg == "--device" && i + 1 < argc) {
      device_ip = argv[++i];
    } else if (arg == "--timeout" && i + 1 < argc) {
      discovery_timeout_ms = std::atoi(argv[++i]);
    } else if (arg[0] != '-' && save_path.empty()) {
      save_path = arg;
    }
  }

  if (save_path.empty()) {
    std::cerr << "Error: save_path is required.\n";
    PrintUsage(argv[0]);
    return 1;
  }

  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);

  std::cout << "========================================\n";
  std::cout << "ODIN Get Calibration File Tool\n";
  std::cout << "========================================\n";
  std::cout << "Save path: " << save_path << "\n";

  std::cout << "\n";
  std::cout << "========================================\n";
  std::cout << "  SDK VERSION: " << GetSdkVersion() << "\n";
  std::cout << "========================================\n";
  std::cout << "\n";

  // Discover devices
  std::cout << "Discovering devices (timeout: " << discovery_timeout_ms << "ms)...\n";
  std::vector<DiscoveredDevice> devices;
  if (!DiscoverDevices(devices, discovery_timeout_ms)) {
    std::cerr << "Device discovery failed.\n";
    return 1;
  }

  if (devices.empty()) {
    std::cerr << "No devices found.\n";
    return 1;
  }

  std::cout << "Found " << devices.size() << " device(s):\n";
  for (size_t i = 0; i < devices.size(); ++i) {
    std::cout << "  [" << i << "] " << devices[i].network.ip_address << " (SN: " << devices[i].sn << ")\n";
  }

  // Select device
  DiscoveredDevice* target_device = nullptr;
  if (!device_ip.empty()) {
    for (auto& dev : devices) {
      if (dev.network.ip_address == device_ip) {
        target_device = &dev;
        break;
      }
    }
    if (!target_device) {
      std::cerr << "Device with IP " << device_ip << " not found.\n";
      return 1;
    }
  } else {
    target_device = &devices[0];
  }

  std::cout << "Using device: " << target_device->network.ip_address << " (SN: " << target_device->sn << ")\n";

  // Connect to device (ports auto-generated)
  OdinDeviceHandle handle = ConnectDevice(*target_device);
  if (handle == kInvalidDeviceHandle) {
    std::cerr << "Failed to connect to device.\n";
    return 1;
  }
  std::cout << "Connected to device.\n";

  // Get and print firmware version
  std::string version;
  if (GetFirmwareVersion(handle, version) == OdinResult::kOk) {
    std::cout << "\n";
    std::cout << "========================================\n";
    std::cout << "  FIRMWARE VERSION: " << version << "\n";
    std::cout << "========================================\n";
    std::cout << "\n";
  }

  // Set device to standby mode for file transfer
  std::cout << "Setting device to standby mode...\n";
  if (SetOperatingMode(handle, OdinOperatingMode::kStandby) != OdinResult::kOk) {
    std::cerr << "Warning: Failed to set device mode.\n";
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Get calibration file
  std::cout << "Requesting calibration file...\n";

  OdinResult result =
      GetDeviceFile(handle, ProgressCallback, OdinFileType::kCalibrationFile, save_path);
  std::cout << "finish Requesting calibration file...\n";
  if (result != OdinResult::kOk) {
    std::cerr << "Failed to get calibration file. Error code: " << static_cast<int>(result) << "\n";
    DisconnectDevice(handle);
    return 1;
  }

  // Wait for transfer to complete
  auto start_time = std::chrono::steady_clock::now();
  constexpr int kTransferTimeoutSec = 60;
  while (!g_transfer_done.load() && g_running.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() > kTransferTimeoutSec) {
      std::cerr << "\nTransfer timeout.\n";
      break;
    }
  }

  // Cleanup
  DisconnectDevice(handle);

  if (g_transfer_success.load()) {
    std::cout << "\n========================================\n";
    std::cout << "Calibration file saved to: " << save_path << "\n";
    std::cout << "========================================\n";
    return 0;
  } else {
    std::cerr << "Failed to complete file transfer.\n";
    return 1;
  }
}
