#pragma once

#include "../IHotplugListener/IHotplugListener.h"
#include "../../discovery/IDiscovery/IDiscovery.h"

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace odin {
namespace sdk {

// =============================================================================
// UdpHotplugListener - UDP broadcast-based device hotplug detection
// =============================================================================

class UdpHotplugListener : public IHotplugListener {
 public:
  UdpHotplugListener();
  ~UdpHotplugListener() override;

  // IHotplugListener interface
  bool Start(const HotplugCallbacks& callbacks, bool enumerate_existing = true) override;
  void Stop() override;
  bool IsRunning() const override;
  void SetPollingInterval(uint32_t interval_ms) override;
  void SetOfflineTimeout(uint32_t timeout_ms) override;

  // Set custom discovery (default: OdinUdpDiscovery)
  void SetDiscovery(std::shared_ptr<IDiscovery> discovery);

 private:
  // Internal device tracking state
  struct DeviceState {
    DiscoveredDevice device;
    std::chrono::steady_clock::time_point last_seen;
    bool online = false;
  };

  // Worker thread function
  void WorkerThread();

  // Send discovery broadcast and collect responses
  void SendDiscoveryProbe();

  // Handle a discovered device (update state and fire callbacks)
  void HandleDiscoveredDevice(const DiscoveredDevice& device);
  
  // Note: CheckOfflineDevices() removed - offline detection now handled by heartbeat

  // Mark a device as offline (called when heartbeat fails)
  void MarkDeviceOffline(const DiscoveredDevice& device) override;

  // Thread safety
  mutable std::mutex mutex_;
  std::thread worker_thread_;
  std::atomic<bool> running_{false};

  // Configuration
  uint32_t polling_interval_ms_ = 1000;  // Default 1 second polling
  uint32_t offline_timeout_ms_ = 3000;   // Default 3 seconds offline threshold

  // Discovery (default will be created on first use)
  std::shared_ptr<IDiscovery> discovery_;

  // Callbacks
  HotplugCallbacks callbacks_;

  // Whether to enumerate existing devices on start
  bool enumerate_existing_ = true;
  bool first_probe_done_ = false;

  // Device tracking (key: device IP or SN)
  std::map<std::string, DeviceState> devices_;
};

}  // namespace sdk
}  // namespace odin
