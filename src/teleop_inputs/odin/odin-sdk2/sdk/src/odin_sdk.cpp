#include "OdinSdkImpl.h"
#include "../utils/network_utils.h"
#include "../hotplugListener/HotplugListenerFactory.h"

#include <memory>
#include <mutex>

namespace {
// Global hotplug listener instance (using base class pointer for polymorphism)
std::unique_ptr<odin::sdk::IHotplugListener> g_hotplug_listener;
std::mutex g_hotplug_mutex;

// Store callbacks for heartbeat failure notification
odin::sdk::HotplugCallbacks g_hotplug_callbacks;
}  // namespace

namespace odin {
namespace sdk {

// =============================================================================
// Public API Functions
// =============================================================================

OdinDeviceHandle ConnectDevice(const DiscoveredDevice &device) {
  // Ensure heartbeat failure callback is registered (in case StartHotplugListener was called first)
  // This allows on_detach to be triggered even if ConnectDevice is called after StartHotplugListener
  static std::once_flag once;
  std::call_once(once, []() {
    OdinSdkImpl::Instance().SetHeartbeatFailedNotifyCallback(
        [](const DiscoveredDevice& dev) {
          OnDetachCallback detach_cb;
          {
            std::lock_guard<std::mutex> lock(g_hotplug_mutex);
            detach_cb = g_hotplug_callbacks.on_detach;
            // Mark device as offline in hotplug listener so it can be re-discovered
            if (g_hotplug_listener) {
              g_hotplug_listener->MarkDeviceOffline(dev);
            }
          }
          if (detach_cb) {
            detach_cb(dev);
          }
        });
  });
  
  return OdinSdkImpl::Instance().ConnectDevice(device);
}

void DisconnectDevice(OdinDeviceHandle handle) { OdinSdkImpl::Instance().DisconnectDevice(handle); }

void RegisterPointCloudCallback(OdinDeviceHandle handle, OdinPointCloudCallback cb,
                                void *client_data) {
  OdinSdkImpl::Instance().RegisterPointCloudCallback(handle, cb, client_data);
}

void RegisterSlamCallback(OdinDeviceHandle handle, OdinSlamCallback cb, void *client_data) {
  OdinSdkImpl::Instance().RegisterSlamCallback(handle, cb, client_data);
}

void RegisterImageCallback(OdinDeviceHandle handle, OdinImageCallback cb, void *client_data) {
  OdinSdkImpl::Instance().RegisterImageCallback(handle, cb, client_data);
}

void RegisterImageCallback2(OdinDeviceHandle handle, OdinImageCallback2 cb, void *client_data) {
  OdinSdkImpl::Instance().RegisterImageCallback2(handle, cb, client_data);
}

void RegisterImuCallback(OdinDeviceHandle handle, OdinImuCallback cb, void *client_data) {
  OdinSdkImpl::Instance().RegisterImuCallback(handle, cb, client_data);
}

void RegisterOdomCallback(OdinDeviceHandle handle, OdinOdomCallback cb, void *client_data) {
  OdinSdkImpl::Instance().RegisterOdomCallback(handle, cb, client_data);
}

void EnableSlamOdomSyncForDevice(OdinDeviceHandle handle, bool enabled, uint32_t max_frame_lag) {
  OdinSdkImpl::Instance().EnableSlamOdomSyncForDevice(handle, enabled, max_frame_lag);
}

OdinResult SetOperatingMode(OdinDeviceHandle device, OdinOperatingMode mode,
                            uint32_t timeout_ms) {
  return OdinSdkImpl::Instance().SetOperatingMode(device, mode, timeout_ms);
}

OdinResult GetFirmwareVersion(OdinDeviceHandle device, std::string &version,
                              uint32_t timeout_ms) {
  return OdinSdkImpl::Instance().GetFirmwareVersion(device, version, timeout_ms);
}

OdinResult SetSensorMode(OdinDeviceHandle device, uint8_t mode, uint32_t timeout_ms) {
  return OdinSdkImpl::Instance().SetSensorMode(device, mode, timeout_ms);
}

OdinResult StartStream(OdinDeviceHandle device, OdinDataChannel channel,
                       OdinTransportMode transport, const OdinStreamCfg* mode,
                       uint32_t timeout_ms) {
  return OdinSdkImpl::Instance().StartStream(device, channel, transport, mode, timeout_ms);
}

OdinResult GetSensorCapability(OdinDeviceHandle device,
                               std::vector<OdinSensorCapability>& capabilities,
                               const std::vector<OdinDataChannel>& channels,
                               uint32_t timeout_ms) {
  return OdinSdkImpl::Instance().GetSensorCapability(device, capabilities, channels, timeout_ms);
}

OdinResult CloseStream(OdinDeviceHandle device, OdinDataChannel channel, uint32_t timeout_ms) {
  return OdinSdkImpl::Instance().CloseStream(device, channel, timeout_ms);
}

OdinResult GetNetworkAttribute(OdinDeviceHandle device, NetworkAttribute& attr, uint32_t timeout_ms) {
  return OdinSdkImpl::Instance().GetNetworkAttribute(device, attr, timeout_ms);
}

OdinResult SetNetworkAttribute(OdinDeviceHandle device, const NetworkAttribute& attr, uint32_t timeout_ms) {
  return OdinSdkImpl::Instance().SetNetworkAttribute(device, attr, timeout_ms);
}

OdinResult Reboot(OdinDeviceHandle device, uint32_t timeout_ms) {
  return OdinSdkImpl::Instance().Reboot(device, timeout_ms);
}

std::vector<std::string> GetLocalIPAddresses() { return network::GetLocalIPAddresses(); }

bool DiscoverDevices(std::vector<DiscoveredDevice> &devices, uint32_t timeout_ms) {
  return OdinSdkImpl::Instance().DiscoverDevices(devices, timeout_ms);
}

OdinResult GetDeviceFile(OdinDeviceHandle device, OdinUpgradeProgressCallback cb, OdinFileType type,
                         std::string save_path) {
  return OdinSdkImpl::Instance().GetDeviceFile(device, cb, type, save_path);
}

OdinResult SendFileToDevice(OdinDeviceHandle device, OdinUpgradeProgressCallback cb,
                            OdinFileType type, std::string file_path) {
  return OdinSdkImpl::Instance().SendFileToDevice(device, cb, type, file_path);
}

std::string GetSdkVersion() { return "2.0.2_20260518"; }

// =============================================================================
// Hotplug Listener API
// =============================================================================

bool StartHotplugListener(const HotplugCallbacks &callbacks, bool enumerate_existing) {
  std::lock_guard<std::mutex> lock(g_hotplug_mutex);
  if (g_hotplug_listener && g_hotplug_listener->IsRunning()) {
    return false;  // Already running
  }
  
  // Store callbacks for heartbeat failure notification
  // Note: The heartbeat failure callback is registered in ConnectDevice() using std::once_flag
  g_hotplug_callbacks = callbacks;
  
  g_hotplug_listener = HotplugListenerFactory::CreateDefault();
  return g_hotplug_listener->Start(callbacks, enumerate_existing);
}

void StopHotplugListener() {
  std::lock_guard<std::mutex> lock(g_hotplug_mutex);
  if (g_hotplug_listener) {
    g_hotplug_listener->Stop();
    g_hotplug_listener.reset();
  }
}

bool IsHotplugListenerRunning() {
  std::lock_guard<std::mutex> lock(g_hotplug_mutex);
  return g_hotplug_listener && g_hotplug_listener->IsRunning();
}

void SetHotplugPollingInterval(uint32_t interval_ms) {
  std::lock_guard<std::mutex> lock(g_hotplug_mutex);
  if (g_hotplug_listener) {
    g_hotplug_listener->SetPollingInterval(interval_ms);
  }
  // Also set heartbeat interval for devices
  OdinSdkImpl::Instance().SetHeartbeatInterval(interval_ms);
}

void SetHotplugOfflineTimeout(uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(g_hotplug_mutex);
  if (g_hotplug_listener) {
    g_hotplug_listener->SetOfflineTimeout(timeout_ms);
  }
  // Also set heartbeat timeout for devices
  OdinSdkImpl::Instance().SetHeartbeatTimeout(timeout_ms);
}

}  // namespace sdk
}  // namespace odin
