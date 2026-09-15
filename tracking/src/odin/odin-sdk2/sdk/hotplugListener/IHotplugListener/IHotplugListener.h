#pragma once

#include <functional>
#include <memory>
#include <string>

#include "odin_lidar_def.h"

namespace odin {
namespace sdk {

// Hotplug callback types
using OnAttachCallback = std::function<void(const DiscoveredDevice& device)>;
using OnDetachCallback = std::function<void(const DiscoveredDevice& device)>;

// Hotplug callbacks structure - register both callbacks together
struct HotplugCallbacks {
  OnAttachCallback on_attach;  // Called when device comes online
  OnDetachCallback on_detach;  // Called when device goes offline
};

// =============================================================================
// IHotplugListener - Abstract interface for device hotplug detection
// =============================================================================

class IHotplugListener {
 public:
  virtual ~IHotplugListener() = default;

  /**
   * @brief Start listening for device hotplug events
   * @param callbacks Structure containing on_attach and on_detach callbacks
   * @param enumerate_existing If true, report already-connected devices via on_attach
   * @return true if started successfully
   */
  virtual bool Start(const HotplugCallbacks& callbacks, bool enumerate_existing = true) = 0;

  /**
   * @brief Stop listening for device hotplug events
   */
  virtual void Stop() = 0;

  /**
   * @brief Check if listener is running
   * @return true if running
   */
  virtual bool IsRunning() const = 0;

  /**
   * @brief Set the polling interval for device detection
   * @param interval_ms Interval in milliseconds
   */
  virtual void SetPollingInterval(uint32_t interval_ms) = 0;

  /**
   * @brief Set the timeout for considering a device offline
   * @param timeout_ms Timeout in milliseconds (device not responding)
   */
  virtual void SetOfflineTimeout(uint32_t timeout_ms) = 0;

  /**
   * @brief Mark a device as offline (used by heartbeat failure notification)
   * @param device The device to mark as offline
   */
  virtual void MarkDeviceOffline(const DiscoveredDevice& device) = 0;
};

}  // namespace sdk
}  // namespace odin
