#pragma once

#include "odin_lidar_api.h"

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include "IDevice.h"

namespace odin {
namespace sdk {

// =============================================================================
// OdinSdkImpl - Core SDK Implementation (Singleton)
// =============================================================================

class OdinSdkImpl {
 public:
  // === Singleton Access ===
  static OdinSdkImpl &Instance();

  // === Public API ===
  OdinDeviceHandle ConnectDevice(const DiscoveredDevice &device);
  void DisconnectDevice(OdinDeviceHandle handle);

  // Callback registration (device must be connected)
  void RegisterPointCloudCallback(OdinDeviceHandle handle, OdinPointCloudCallback cb,
                                  void *user_data);
  void RegisterSlamCallback(OdinDeviceHandle handle, OdinSlamCallback cb, void *user_data);
  void RegisterImageCallback(OdinDeviceHandle handle, OdinImageCallback cb, void *user_data);
  void RegisterImageCallback2(OdinDeviceHandle handle, OdinImageCallback2 cb, void *user_data);
  void RegisterImuCallback(OdinDeviceHandle handle, OdinImuCallback cb, void *user_data);
  void RegisterOdomCallback(OdinDeviceHandle handle, OdinOdomCallback cb, void *user_data);
  void EnableSlamOdomSyncForDevice(OdinDeviceHandle handle, bool enabled, uint32_t max_frame_lag);

  // Command operations
  OdinResult GetFirmwareVersion(OdinDeviceHandle handle, std::string &version,
                                uint32_t timeout_ms = 1000);
  OdinResult SetOperatingMode(OdinDeviceHandle handle, OdinOperatingMode mode,
                              uint32_t timeout_ms = 1000);
  OdinResult SetSensorMode(OdinDeviceHandle handle, uint8_t mode, uint32_t timeout_ms = 1000);
  OdinResult GetDeviceFile(OdinDeviceHandle device, OdinUpgradeProgressCallback cb,
                           OdinFileType type, std::string save_path);
  OdinResult SendFileToDevice(OdinDeviceHandle device, OdinUpgradeProgressCallback cb,
                              OdinFileType type, std::string file_path);
  OdinResult StartStream(OdinDeviceHandle device, OdinDataChannel channel,
                         OdinTransportMode transport, const OdinStreamCfg* mode = nullptr,
                         uint32_t timeout_ms = 1000);
  OdinResult GetSensorCapability(OdinDeviceHandle device,
                                 std::vector<OdinSensorCapability>& capabilities,
                                 const std::vector<OdinDataChannel>& channels,
                                 uint32_t timeout_ms);
  OdinResult CloseStream(OdinDeviceHandle device, OdinDataChannel channel, uint32_t timeout_ms);
  OdinResult GetNetworkAttribute(OdinDeviceHandle device, NetworkAttribute& attr, uint32_t timeout_ms);
  OdinResult SetNetworkAttribute(OdinDeviceHandle device, const NetworkAttribute& attr, uint32_t timeout_ms);
  OdinResult Reboot(OdinDeviceHandle device, uint32_t timeout_ms);
  bool IsInitialized() const;

  // Device discovery
  bool DiscoverDevices(std::vector<DiscoveredDevice> &devices, uint32_t timeout_ms = 2000);

  // Heartbeat failure callback (called by device when heartbeat fails)
  using HeartbeatFailedNotifyCallback = std::function<void(const DiscoveredDevice& device)>;
  void SetHeartbeatFailedNotifyCallback(HeartbeatFailedNotifyCallback cb);
  
  // Heartbeat interval configuration
  void SetHeartbeatInterval(uint32_t interval_ms);
  uint32_t GetHeartbeatInterval() const;
  
  // Heartbeat timeout configuration
  void SetHeartbeatTimeout(uint32_t timeout_ms);
  uint32_t GetHeartbeatTimeout() const;

  
 private:
  void OnDeviceHeartbeatFailed(OdinDeviceHandle handle);
  // Device discovery helpers (USB only, network discovery moved to network_utils)
  bool DiscoverUsbDevices(std::vector<DiscoveredDevice> &devices, std::set<std::string> &seen_sns,
                          uint32_t timeout_ms);

  // === Device Access ===
  IDevice *GetDevice(OdinDeviceHandle handle);
  const IDevice *GetDevice(OdinDeviceHandle handle) const;

 private:
  // === Private Constructors (Singleton) ===
  OdinSdkImpl() = default;
  ~OdinSdkImpl();
  OdinSdkImpl(const OdinSdkImpl &) = delete;
  OdinSdkImpl &operator=(const OdinSdkImpl &) = delete;

  // === Private Methods ===
  // Device connection
  void DisconnectSingleDevice(OdinDeviceHandle handle);
  void DisconnectAllDevices();
  IDevice *GetDeviceLocked(OdinDeviceHandle handle);
  OdinDeviceHandle ResolveHandleLocked(OdinDeviceHandle handle) const;

  // === Member Variables ===
  // Device state
  mutable std::mutex state_mutex_;
  std::map<OdinDeviceHandle, std::unique_ptr<IDevice>> devices_;
  std::atomic<OdinDeviceHandle> next_handle_{1};
  OdinDeviceHandle default_handle_ = kInvalidDeviceHandle;
  
  // Heartbeat failure notification
  HeartbeatFailedNotifyCallback heartbeat_failed_notify_cb_;
  
  // Heartbeat interval (default 500ms)
  uint32_t heartbeat_interval_ms_ = 500;
  
  // Heartbeat timeout (default 3000ms)
  uint32_t heartbeat_timeout_ms_ = 3000;
};

}  // namespace sdk
}  // namespace odin
