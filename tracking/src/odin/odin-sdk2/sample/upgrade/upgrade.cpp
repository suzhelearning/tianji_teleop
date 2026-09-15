/**
 * @file upgrade.cpp
 * @brief Simplified ODIN OTA Tool using SDK UpgradeFirmware API
 *
 * This demo demonstrates the simplified firmware upgrade process:
 * 1. Discover device (or use specified IP)
 * 2. Connect to device
 * 3. Call UpgradeFirmware() - SDK handles everything internally
 * 4. Display progress bar (0-100%)
 * 5. Done!
 */

#include "odin_lidar_api.h"
#include "logger.h"

#include <atomic>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

using namespace odin::sdk;

// =============================================================================
// Global State
// =============================================================================

static std::atomic<bool> g_interrupted{false};
static OdinDeviceHandle g_active_handle = kInvalidDeviceHandle;

static void signal_handler(int /*sig*/) {
  g_interrupted.store(true);
}

static void install_signal_handlers() {
  struct sigaction sa {};
  sa.sa_handler = signal_handler;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;
  sigaction(SIGINT, &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);
}

// =============================================================================
// Utility Functions
// =============================================================================

namespace {

constexpr int kDefaultDiscoveryTimeoutMs = 3000;

void print_usage(const char *exe) {
  std::cout << "Usage: " << exe << " <firmware_path> [options]\n"
            << "\n"
            << "ODIN OTA Tool (Simplified)\n"
            << "\n"
            << "Auto-discovers device via UDP, connects, and upgrades firmware.\n"
            << "SDK handles all internal details (HTTP/HTTPS, state machine, etc.)\n"
            << "\n"
            << "Arguments:\n"
            << "  <firmware_path>   Path to firmware file (.bin)\n"
            << "\n"
            << "Options:\n"
            << "  --ip <addr>       Device IP (skip discovery if specified)\n"
            << "  --timeout <ms>    Discovery timeout (default: 3000)\n"
            << "  --help            Show this help\n"
            << "\n"
            << "Examples:\n"
            << "  " << exe << " ./firmware.bin\n"
            << "  " << exe << " ./firmware.bin --ip 192.168.1.251\n";
}

/**
 * @brief Progress callback - displays progress bar
 */
void progress_callback(OdinDeviceHandle /*device*/, float progress, void* /*client_data*/) {
  const int pct = static_cast<int>(progress);
  const int bar_width = 40;
  const int filled = (pct * bar_width) / 100;

  std::cout << "\r[";
  for (int i = 0; i < bar_width; ++i) {
    if (i < filled)
      std::cout << "=";
    else if (i == filled)
      std::cout << ">";
    else
      std::cout << " ";
  }
  std::cout << "] " << pct << "%" << std::flush;

  if (progress >= 100.0f) {
    std::cout << " Done!\n";
  }
}

}  // namespace

// =============================================================================
// Main
// =============================================================================

int main(int argc, char **argv) {
  setvbuf(stdout, NULL, _IOLBF, 0);
  log_config(LOG_LEVEL_INFO, printf);
  install_signal_handlers();

  std::string firmware_path;
  std::string device_ip_arg;
  int discovery_timeout_ms = kDefaultDiscoveryTimeoutMs;

  // Parse arguments
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      print_usage(argv[0]);
      return 0;
    } else if ((arg == "--ip" || arg == "-i") && i + 1 < argc) {
      device_ip_arg = argv[++i];
    } else if (arg == "--timeout" && i + 1 < argc) {
      discovery_timeout_ms = std::atoi(argv[++i]);
    } else if (arg[0] != '-' && firmware_path.empty()) {
      firmware_path = arg;
    }
  }

  if (firmware_path.empty()) {
    std::cerr << "Error: firmware_path is required.\n";
    print_usage(argv[0]);
    return 1;
  }

  std::cout << "\n";
  std::cout << "========================================\n";
  std::cout << "  SDK VERSION: " << GetSdkVersion() << "\n";
  std::cout << "========================================\n";
  std::cout << "\n";
  std::cout << "ODIN OTA Tool (Simplified)\n";
  std::cout << "Firmware: " << firmware_path << "\n\n";

  // Device discovery or direct IP
  DiscoveredDevice target_device;
  
  if (!device_ip_arg.empty()) {
    // Direct IP mode
    target_device.network.ip_address = device_ip_arg;
    std::cout << "Using specified device IP: " << device_ip_arg << "\n";
  } else {
    // Device discovery
    std::cout << "Discovering devices (timeout: " << discovery_timeout_ms << "ms)...\n";
    std::vector<DiscoveredDevice> devices;
    if (!DiscoverDevices(devices, static_cast<uint32_t>(discovery_timeout_ms))) {
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

    // Select target device
    if (devices.size() == 1) {
      target_device = devices[0];
    } else {
      std::cout << "Multiple devices found. Enter index [0-" << devices.size() - 1 << "]: ";
      int idx = 0;
      if (!(std::cin >> idx) || idx < 0 || idx >= static_cast<int>(devices.size())) {
        std::cerr << "Invalid selection.\n";
        return 1;
      }
      target_device = devices[idx];
    }
  }

  std::cout << "Target device: " << target_device.network.ip_address;
  if (!target_device.sn.empty()) {
    std::cout << " (SN: " << target_device.sn << ")";
  }
  std::cout << "\n\n";

  // Connect to device
  std::cout << "Connecting to device...\n";
  OdinDeviceHandle handle = ConnectDevice(target_device);
  if (handle == kInvalidDeviceHandle) {
    std::cerr << "Failed to connect to device.\n";
    return 1;
  }
  g_active_handle = handle;
  std::cout << "Connected.\n";

  // Get and print firmware version
  std::string version;
  if (GetFirmwareVersion(handle, version) == OdinResult::kOk) {
    std::cout << "\n";
    std::cout << "========================================\n";
    std::cout << "  FIRMWARE VERSION: " << version << "\n";
    std::cout << "========================================\n";
    std::cout << "\n";
  }

  // Upgrade firmware - SDK handles everything!
  std::cout << "Starting firmware upgrade...\n";
  std::cout << "Progress:\n";
  
  OdinResult result = SendFileToDevice(handle, progress_callback, OdinFileType::kFirmware, firmware_path);

  // Cleanup
  DisconnectDevice(handle);
  g_active_handle = kInvalidDeviceHandle;

  // Result
  std::cout << "\n========================================\n";
  if (result == OdinResult::kOk) {
    std::cout << "Firmware upgrade completed successfully!\n";
    std::cout << "========================================\n";
    return 0;
  } else {
    std::cerr << "Firmware upgrade failed, result=" << static_cast<int>(result) << "\n";
    std::cout << "========================================\n";
    return 1;
  }
}
