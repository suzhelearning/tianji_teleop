#include "CompositeHotplugListener.h"

namespace odin {
namespace sdk {

CompositeHotplugListener::~CompositeHotplugListener() {
  Stop();
}

void CompositeHotplugListener::AddListener(std::unique_ptr<IHotplugListener> listener) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (listener) {
    listeners_.push_back(std::move(listener));
  }
}

bool CompositeHotplugListener::Start(const HotplugCallbacks& callbacks, bool enumerate_existing) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (running_) return false;
  if (listeners_.empty()) return false;

  bool any_started = false;
  for (auto& listener : listeners_) {
    if (listener->Start(callbacks, enumerate_existing)) {
      any_started = true;
    }
  }

  running_ = any_started;
  return any_started;
}

void CompositeHotplugListener::Stop() {
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto& listener : listeners_) {
    listener->Stop();
  }
  running_ = false;
}

bool CompositeHotplugListener::IsRunning() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return running_;
}

void CompositeHotplugListener::SetPollingInterval(uint32_t interval_ms) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto& listener : listeners_) {
    listener->SetPollingInterval(interval_ms);
  }
}

void CompositeHotplugListener::SetOfflineTimeout(uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto& listener : listeners_) {
    listener->SetOfflineTimeout(timeout_ms);
  }
}

void CompositeHotplugListener::MarkDeviceOffline(const DiscoveredDevice& device) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto& listener : listeners_) {
    listener->MarkDeviceOffline(device);
  }
}

}  // namespace sdk
}  // namespace odin
