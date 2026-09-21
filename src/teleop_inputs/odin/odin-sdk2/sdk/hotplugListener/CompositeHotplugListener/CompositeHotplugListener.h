#pragma once

#include "../IHotplugListener/IHotplugListener.h"
#include <memory>
#include <vector>
#include <mutex>

namespace odin {
namespace sdk {

/**
 * @brief Composite hotplug listener that manages multiple sub-listeners
 * 
 * This class implements the Composite pattern, allowing multiple hotplug
 * listeners (e.g., UDP for network devices, USB for USB devices) to be
 * managed as a single listener.
 */
class CompositeHotplugListener : public IHotplugListener {
 public:
  CompositeHotplugListener() = default;
  ~CompositeHotplugListener() override;

  /**
   * @brief Add a sub-listener to this composite
   * @param listener The listener to add (ownership transferred)
   */
  void AddListener(std::unique_ptr<IHotplugListener> listener);

  // IHotplugListener interface
  bool Start(const HotplugCallbacks& callbacks, bool enumerate_existing = true) override;
  void Stop() override;
  bool IsRunning() const override;
  void SetPollingInterval(uint32_t interval_ms) override;
  void SetOfflineTimeout(uint32_t timeout_ms) override;
  void MarkDeviceOffline(const DiscoveredDevice& device) override;

 private:
  std::vector<std::unique_ptr<IHotplugListener>> listeners_;
  mutable std::mutex mutex_;
  bool running_ = false;
};

}  // namespace sdk
}  // namespace odin
