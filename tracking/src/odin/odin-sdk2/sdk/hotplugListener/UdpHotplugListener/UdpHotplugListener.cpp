#include "UdpHotplugListener.h"

#include <cstring>
#ifndef _WIN32
#include <arpa/inet.h>
#endif

#include "../../logger/logger.h"
#include "../../utils/network_utils.h"

namespace odin {
namespace sdk {

namespace {
constexpr const char* kContextKeyHostIp = "host_ip";
}  // namespace

// =============================================================================
// UdpHotplugListener Implementation
// =============================================================================

UdpHotplugListener::UdpHotplugListener() = default;

UdpHotplugListener::~UdpHotplugListener() { Stop(); }

bool UdpHotplugListener::Start(const HotplugCallbacks& callbacks, bool enumerate_existing) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (running_.load()) return true;

  if (!callbacks.on_attach && !callbacks.on_detach) {
    LOG_ERROR("UdpHotplugListener: At least one callback is required\n");
    return false;
  }

  if (!network::AcquireWinsock()) {
    LOG_ERROR("UdpHotplugListener: Failed to initialize network\n");
    return false;
  }

  callbacks_ = callbacks;
  enumerate_existing_ = enumerate_existing;
  first_probe_done_ = false;
  devices_.clear();

  running_.store(true);
  worker_thread_ = std::thread(&UdpHotplugListener::WorkerThread, this);

  LOG_INFO("UdpHotplugListener: Started (poll=%ums, timeout=%ums, enumerate_existing=%s)\n",
           polling_interval_ms_, offline_timeout_ms_, enumerate_existing_ ? "true" : "false");
  return true;
}

void UdpHotplugListener::Stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_.load()) return;
    running_.store(false);
  }

  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    callbacks_ = HotplugCallbacks{};
    devices_.clear();
  }

  network::ReleaseWinsock();
  LOG_INFO("UdpHotplugListener: Stopped\n");
}

bool UdpHotplugListener::IsRunning() const { return running_.load(); }

void UdpHotplugListener::SetPollingInterval(uint32_t interval_ms) {
  std::lock_guard<std::mutex> lock(mutex_);
  polling_interval_ms_ = interval_ms;
}

void UdpHotplugListener::SetOfflineTimeout(uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(mutex_);
  offline_timeout_ms_ = timeout_ms;
}

void UdpHotplugListener::SetDiscovery(std::shared_ptr<IDiscovery> discovery) {
  std::lock_guard<std::mutex> lock(mutex_);
  discovery_ = std::move(discovery);
}

void UdpHotplugListener::WorkerThread() {
  LOG_DEBUG("UdpHotplugListener: Worker thread started\n");

  while (running_.load()) {
    SendDiscoveryProbe();
    // Note: Device offline detection is now handled by heartbeat mechanism in Odin2Device
    // CheckOfflineDevices() is no longer called here

    // Sleep for polling interval (check running_ periodically for quick shutdown)
    uint32_t sleep_interval = 100;  // Check every 100ms
    uint32_t elapsed = 0;
    while (running_.load() && elapsed < polling_interval_ms_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(sleep_interval));
      elapsed += sleep_interval;
    }
  }

  LOG_DEBUG("UdpHotplugListener: Worker thread exiting\n");
}

void UdpHotplugListener::SendDiscoveryProbe() {
  // Get discovery (use default if not set)
  auto discovery = discovery_;
  if (!discovery) discovery = DiscoveryFactory::GetDefault();

  auto local_ips = network::GetLocalIPAddresses();
  if (local_ips.empty()) {
    LOG_WARN("UdpHotplugListener: No local IP addresses found\n");
    return;
  }

  for (const auto& host_ip : local_ips) {
    // Skip loopback
    if (host_ip.find("127.") == 0) continue;

    // Create broadcast socket using shared utility
    network::SocketHandle sock = network::CreateBroadcastSocket(host_ip);
    if (sock == network::kInvalidSocket) continue;

    // Build context for discovery
    IDiscovery::Context context;
    context[kContextKeyHostIp] = host_ip;

    // Send discovery broadcast
    if (!network::SendDiscoveryBroadcast(sock, network::kDefaultCommandPort, context, discovery)) {
      network::CloseSocket(sock);
      continue;
    }

    // Wait for responses (short timeout)
    auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(polling_interval_ms_ / 2);
    std::vector<uint8_t> buffer(512);

    while (std::chrono::steady_clock::now() < deadline && running_.load()) {
      auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
          deadline - std::chrono::steady_clock::now());
      if (remaining.count() <= 0) break;

      timeval tv;
      tv.tv_sec = 0;
      tv.tv_usec = static_cast<long>(std::min<int64_t>(remaining.count(), 100) * 1000);

      fd_set read_fds;
      FD_ZERO(&read_fds);
      FD_SET(sock, &read_fds);

#ifdef _WIN32
      int ready = select(0, &read_fds, nullptr, nullptr, &tv);
#else
      int ready = select(static_cast<int>(sock + 1), &read_fds, nullptr, nullptr, &tv);
#endif
      if (ready <= 0 || !FD_ISSET(sock, &read_fds)) continue;

      sockaddr_in from_addr{};
      socklen_t addr_len = sizeof(from_addr);
      int len =
          recvfrom(sock, reinterpret_cast<char*>(buffer.data()), static_cast<int>(buffer.size()), 0,
                   reinterpret_cast<sockaddr*>(&from_addr), &addr_len);
      if (len <= 0) continue;

      // Parse response using discovery
      DiscoveredDevice device;
      if (discovery->ParseResponse(buffer.data(), static_cast<size_t>(len), device)) {
        // Use actual socket source IP instead of protocol payload IP
        char socket_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &from_addr.sin_addr, socket_ip, INET_ADDRSTRLEN);
        device.network.ip_address = socket_ip;
        device.host_ip = host_ip;
        HandleDiscoveredDevice(device);
      }
    }

    network::CloseSocket(sock);
  }

  // Mark first probe as done after processing all interfaces
  if (!first_probe_done_) {
    std::lock_guard<std::mutex> lock(mutex_);
    first_probe_done_ = true;
  }
}

void UdpHotplugListener::HandleDiscoveredDevice(const DiscoveredDevice& device) {
  bool should_notify = false;
  DiscoveredDevice notify_device;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto now = std::chrono::steady_clock::now();
    std::string key = device.sn.empty() ? device.network.ip_address : device.sn;

    auto it = devices_.find(key);
    if (it == devices_.end()) {
      // New device found
      DeviceState state;
      state.device = device;
      state.last_seen = now;
      state.online = true;
      devices_[key] = state;

      // Only report arrival if enumerate_existing is true OR this is not the first probe
      bool should_report = enumerate_existing_ || first_probe_done_;

      if (should_report) {
        LOG_INFO("UdpHotplugListener: Device arrived - IP=%s, SN=%s\n", device.network.ip_address.c_str(),
                 device.sn.c_str());
        should_notify = true;
        notify_device = device;
      } else {
        LOG_INFO("UdpHotplugListener: Device found (pre-existing, not reported) - IP=%s, SN=%s\n",
                 device.network.ip_address.c_str(), device.sn.c_str());
      }
    } else {
      // Update last seen time
      it->second.last_seen = now;
      it->second.device = device;  // Update device info in case it changed

      // If device was offline, mark it as back online
      if (!it->second.online) {
        it->second.online = true;
        LOG_INFO("UdpHotplugListener: Device back online - IP=%s, SN=%s\n", device.network.ip_address.c_str(),
                 device.sn.c_str());
        should_notify = true;
        notify_device = device;
      }
    }
  }

  // Call callback outside the lock to avoid deadlock/threading issues
  if (should_notify && callbacks_.on_attach) {
    callbacks_.on_attach(notify_device);
  }
}

// Note: CheckOfflineDevices() has been removed.
// Device offline detection is now handled by heartbeat mechanism in Odin2Device.
// When heartbeat fails, Odin2Device notifies OdinSdkImpl, which triggers on_detach callback.

void UdpHotplugListener::MarkDeviceOffline(const DiscoveredDevice& device) {
  std::lock_guard<std::mutex> lock(mutex_);
  std::string key = device.sn.empty() ? device.network.ip_address : device.sn;
  
  auto it = devices_.find(key);
  if (it != devices_.end()) {
    it->second.online = false;
    LOG_INFO("UdpHotplugListener: Device marked offline - IP=%s, SN=%s\n", 
             device.network.ip_address.c_str(), device.sn.c_str());
  }
}

}  // namespace sdk
}  // namespace odin
