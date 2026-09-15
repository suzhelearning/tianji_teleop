#include "OdinSdkImpl.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <cerrno>
#include <functional>
#include <limits>
#include <random>
#include <set>
#include <thread>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "odin2.h"
#include "ITransport.hpp"
#include "OdinProtocol.hpp"
#include "logger.h"
#include "network_utils.h"

#ifdef _WIN32
#include <sys/select.h>
#else
#include <sys/select.h>
#endif

namespace odin {
namespace sdk {

// =============================================================================
// OdinSdkImpl - Core Methods
// =============================================================================

OdinSdkImpl &OdinSdkImpl::Instance() {
  static OdinSdkImpl instance;
  return instance;
}

OdinSdkImpl::~OdinSdkImpl() { DisconnectAllDevices(); }

void OdinSdkImpl::DisconnectDevice(OdinDeviceHandle handle) { DisconnectSingleDevice(handle); }

void OdinSdkImpl::RegisterPointCloudCallback(OdinDeviceHandle handle, OdinPointCloudCallback cb,
                                             void *user_data) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  OdinDeviceHandle resolved = ResolveHandleLocked(handle);
  IDevice *device = GetDeviceLocked(resolved);
  if (device) device->RegisterPointCloudCallback(cb, user_data);
}

void OdinSdkImpl::RegisterSlamCallback(OdinDeviceHandle handle, OdinSlamCallback cb,
                                       void *user_data) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  OdinDeviceHandle resolved = ResolveHandleLocked(handle);
  IDevice *device = GetDeviceLocked(resolved);
  if (device) device->RegisterSlamCallback(cb, user_data);
}

void OdinSdkImpl::RegisterImageCallback(OdinDeviceHandle handle, OdinImageCallback cb,
                                        void *user_data) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  OdinDeviceHandle resolved = ResolveHandleLocked(handle);
  IDevice *device = GetDeviceLocked(resolved);
  if (device) device->RegisterImageCallback(cb, user_data);
}

void OdinSdkImpl::RegisterImageCallback2(OdinDeviceHandle handle, OdinImageCallback2 cb,
                                         void *user_data) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  OdinDeviceHandle resolved = ResolveHandleLocked(handle);
  IDevice *device = GetDeviceLocked(resolved);
  if (device) device->RegisterImageCallback2(cb, user_data);
}

void OdinSdkImpl::RegisterImuCallback(OdinDeviceHandle handle, OdinImuCallback cb,
                                      void *user_data) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  OdinDeviceHandle resolved = ResolveHandleLocked(handle);
  IDevice *device = GetDeviceLocked(resolved);
  if (device) device->RegisterImuCallback(cb, user_data);
}

void OdinSdkImpl::RegisterOdomCallback(OdinDeviceHandle handle, OdinOdomCallback cb,
                                       void *user_data) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  OdinDeviceHandle resolved = ResolveHandleLocked(handle);
  IDevice *device = GetDeviceLocked(resolved);
  if (device) device->RegisterOdomCallback(cb, user_data);
}

void OdinSdkImpl::EnableSlamOdomSyncForDevice(OdinDeviceHandle handle, bool enabled,
                                              uint32_t max_frame_lag) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  OdinDeviceHandle resolved = ResolveHandleLocked(handle);
  IDevice *device = GetDeviceLocked(resolved);
  if (device) {
    device->EnableSlamOdomSync(enabled, max_frame_lag);
    LOG_INFO("OdinSdkImpl: SLAM-Odom sync %s for device %d (max_lag=%u)\n",
             enabled ? "enabled" : "disabled", resolved, max_frame_lag);
  }
}

OdinResult OdinSdkImpl::GetFirmwareVersion(OdinDeviceHandle handle, std::string &version,
                                           uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  OdinDeviceHandle resolved = ResolveHandleLocked(handle);
  IDevice *device = GetDeviceLocked(resolved);
  if (device == nullptr) return OdinResult::kNotInitialized;
  return device->GetFirmwareVersion(version, timeout_ms);
}

OdinResult OdinSdkImpl::SetOperatingMode(OdinDeviceHandle handle, OdinOperatingMode mode,
                                         uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  OdinDeviceHandle resolved = ResolveHandleLocked(handle);
  IDevice *device = GetDeviceLocked(resolved);
  if (device == nullptr) return OdinResult::kNotInitialized;
  return device->SetOperatingMode(mode, timeout_ms);
}

bool OdinSdkImpl::IsInitialized() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return !devices_.empty();
}

IDevice *OdinSdkImpl::GetDeviceLocked(OdinDeviceHandle handle) {
  if (handle == kInvalidDeviceHandle) return nullptr;
  auto it = devices_.find(handle);
  if (it == devices_.end()) return nullptr;
  return it->second.get();
}

IDevice *OdinSdkImpl::GetDevice(OdinDeviceHandle handle) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return GetDeviceLocked(ResolveHandleLocked(handle));
}

const IDevice *OdinSdkImpl::GetDevice(OdinDeviceHandle handle) const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  OdinDeviceHandle resolved = (handle != kInvalidDeviceHandle) ? handle : default_handle_;
  if (resolved == kInvalidDeviceHandle) return nullptr;
  auto it = devices_.find(resolved);
  if (it == devices_.end()) return nullptr;
  return it->second.get();
}

OdinDeviceHandle OdinSdkImpl::ResolveHandleLocked(OdinDeviceHandle handle) const {
  if (handle != kInvalidDeviceHandle) return handle;
  return default_handle_;
}

bool OdinSdkImpl::DiscoverUsbDevices(std::vector<DiscoveredDevice> &devices,
                                     std::set<std::string> &seen_sns, uint32_t timeout_ms) {
  // TODO: Implement USB device discovery
  // This is a placeholder for future USB device enumeration
  (void)devices;
  (void)seen_sns;
  (void)timeout_ms;
  return false;
}

bool OdinSdkImpl::DiscoverDevices(std::vector<DiscoveredDevice> &devices, uint32_t timeout_ms) {
  devices.clear();

  // Discover network devices (Ethernet) using shared utility
  bool found_network = network::DiscoverNetworkDevices(devices, timeout_ms);

  // Discover USB devices (future)
  std::set<std::string> seen_sns;
  for (const auto &dev : devices) seen_sns.insert(dev.sn);
  bool found_usb = DiscoverUsbDevices(devices, seen_sns, timeout_ms);

  LOG_INFO("Discovery complete: found %zu device(s) (network=%d, usb=%d)\n", devices.size(),
           found_network ? 1 : 0, found_usb ? 1 : 0);
  return !devices.empty();
}

OdinDeviceHandle OdinSdkImpl::ConnectDevice(const DiscoveredDevice &discovered_device) {
  LOG_INFO("ConnectDevice: ip=%s, model=%s, sn=%s\n", discovered_device.network.ip_address.c_str(),
           discovered_device.model.c_str(), discovered_device.sn.c_str());

  // Step 1: Create device instance based on model
  OdinDeviceHandle handle = next_handle_.fetch_add(1);
  std::unique_ptr<IDevice> device;

  if (discovered_device.model == "ODIN2" || discovered_device.model.empty()) {
    device.reset(new Odin2Device(handle));
  } else {
    LOG_ERROR("Unknown device model: %s\n", discovered_device.model.c_str());
    return kInvalidDeviceHandle;
  }

  // Step 2: Configure heartbeat interval/timeout and register failure callback
  auto* odin2_device = dynamic_cast<Odin2Device*>(device.get());
  if (odin2_device) {
    odin2_device->SetHeartbeatInterval(heartbeat_interval_ms_);
    odin2_device->SetHeartbeatTimeout(heartbeat_timeout_ms_);
  }
  device->SetHeartbeatFailedCallback([this](OdinDeviceHandle h) {
    OnDeviceHeartbeatFailed(h);
  });

  // Step 3: Device handles connection internally (command channel only)
  // Note: Data channels are no longer started here.
  // Use StartStream() to subscribe to individual data streams after connection.
  if (!device->Connect(discovered_device)) {
    LOG_ERROR("Device connection failed\n");
    return kInvalidDeviceHandle;
  }

  // Step 4: Register device in SDK
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    devices_[handle] = std::move(device);
    if (default_handle_ == kInvalidDeviceHandle) {
      default_handle_ = handle;
    }
  }
  LOG_INFO("Device connected successfully: %s\n", discovered_device.network.ip_address.c_str());
  return handle;
}

void OdinSdkImpl::DisconnectSingleDevice(OdinDeviceHandle handle) {
  std::unique_ptr<IDevice> device;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    OdinDeviceHandle resolved = ResolveHandleLocked(handle);
    auto it = devices_.find(resolved);
    if (it == devices_.end()) {
      return;
    }
    device = std::move(it->second);
    devices_.erase(it);
    if (default_handle_ == resolved) {
      default_handle_ = devices_.empty() ? kInvalidDeviceHandle : devices_.begin()->first;
    }
  }

  if (device) {
    device->StopDataChannels();
    device->Disconnect();
  }
}

void OdinSdkImpl::DisconnectAllDevices() {
  std::vector<std::unique_ptr<IDevice>> devices;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (auto &kv : devices_) {
      devices.push_back(std::move(kv.second));
    }
    devices_.clear();
    default_handle_ = kInvalidDeviceHandle;
  }

  for (auto &device : devices) {
    if (device) {
      device->StopDataChannels();
      device->Disconnect();
    }
  }
}

// =============================================================================
// File Transfer
// =============================================================================

OdinResult OdinSdkImpl::GetDeviceFile(OdinDeviceHandle device, OdinUpgradeProgressCallback cb,
                                      OdinFileType type, std::string save_path) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  OdinDeviceHandle resolved = ResolveHandleLocked(device);
  IDevice *dev = GetDeviceLocked(resolved);
  if (dev == nullptr) return OdinResult::kInvalidArgument;

  // Delegate to device
  return dev->GetFile(type, save_path, cb);
}

OdinResult OdinSdkImpl::SendFileToDevice(OdinDeviceHandle device, OdinUpgradeProgressCallback cb,
                                         OdinFileType type, std::string file_path) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  OdinDeviceHandle resolved = ResolveHandleLocked(device);
  IDevice *dev = GetDeviceLocked(resolved);
  if (dev == nullptr) return OdinResult::kInvalidArgument;

  // Delegate to device
  return dev->SendFile(type, file_path, cb);
}

OdinResult OdinSdkImpl::SetSensorMode(OdinDeviceHandle device, uint8_t mode,
                                      uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  OdinDeviceHandle resolved = ResolveHandleLocked(device);
  IDevice *dev = GetDeviceLocked(resolved);
  if (dev == nullptr) return OdinResult::kNotInitialized;
  return dev->SetSensorMode(mode, timeout_ms);
}

OdinResult OdinSdkImpl::StartStream(OdinDeviceHandle device, OdinDataChannel channel,
                                    OdinTransportMode transport, const OdinStreamCfg* mode,
                                    uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  OdinDeviceHandle resolved = ResolveHandleLocked(device);
  IDevice *dev = GetDeviceLocked(resolved);
  if (dev == nullptr) return OdinResult::kNotInitialized;
  return dev->StartStream(channel, transport, mode, timeout_ms);
}

OdinResult OdinSdkImpl::GetSensorCapability(OdinDeviceHandle device,
                                            std::vector<OdinSensorCapability>& capabilities,
                                            const std::vector<OdinDataChannel>& channels,
                                            uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  OdinDeviceHandle resolved = ResolveHandleLocked(device);
  IDevice *dev = GetDeviceLocked(resolved);
  if (dev == nullptr) return OdinResult::kNotInitialized;
  return dev->GetSensorCapability(capabilities, channels, timeout_ms);
}

OdinResult OdinSdkImpl::CloseStream(OdinDeviceHandle device, OdinDataChannel channel,
                                    uint32_t timeout_ms) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  OdinDeviceHandle resolved = ResolveHandleLocked(device);
  IDevice *dev = GetDeviceLocked(resolved);
  if (dev == nullptr) return OdinResult::kNotInitialized;
  return dev->CloseStream(channel, timeout_ms);
}

void OdinSdkImpl::SetHeartbeatFailedNotifyCallback(HeartbeatFailedNotifyCallback cb) {
  heartbeat_failed_notify_cb_ = cb;
}

void OdinSdkImpl::OnDeviceHeartbeatFailed(OdinDeviceHandle handle) {
  DiscoveredDevice device;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto it = devices_.find(handle);
    if (it != devices_.end() && it->second) {
      device = it->second->GetDiscoveredDevice();
    }
  }
  
  LOG_INFO("OdinSdkImpl: Device heartbeat failed, handle=%u, sn=%s, ip=%s\n", 
           handle, device.sn.c_str(), device.network.ip_address.c_str());
  
  if (heartbeat_failed_notify_cb_) {
    LOG_INFO("OdinSdkImpl: Calling heartbeat_failed_notify_cb_ (on_detach)\n");
    heartbeat_failed_notify_cb_(device);
  } else {
    LOG_WARN("OdinSdkImpl: heartbeat_failed_notify_cb_ is not set, on_detach will not be called\n");
  }
}

void OdinSdkImpl::SetHeartbeatInterval(uint32_t interval_ms) {
  heartbeat_interval_ms_ = interval_ms;
  LOG_INFO("OdinSdkImpl: Heartbeat interval set to %u ms\n", interval_ms);
}

uint32_t OdinSdkImpl::GetHeartbeatInterval() const {
  return heartbeat_interval_ms_;
}

void OdinSdkImpl::SetHeartbeatTimeout(uint32_t timeout_ms) {
  heartbeat_timeout_ms_ = timeout_ms;
  LOG_INFO("OdinSdkImpl: Heartbeat timeout set to %u ms\n", timeout_ms);
}

uint32_t OdinSdkImpl::GetHeartbeatTimeout() const {
  return heartbeat_timeout_ms_;
}

OdinResult OdinSdkImpl::GetNetworkAttribute(OdinDeviceHandle device, NetworkAttribute& attr,
                                            uint32_t timeout_ms) {
  IDevice* dev = GetDevice(device);
  if (!dev) {
    return OdinResult::kNotInitialized;
  }
  return dev->GetNetworkAttribute(attr, timeout_ms);
}

OdinResult OdinSdkImpl::SetNetworkAttribute(OdinDeviceHandle device, const NetworkAttribute& attr,
                                            uint32_t timeout_ms) {
  IDevice* dev = GetDevice(device);
  if (!dev) {
    return OdinResult::kNotInitialized;
  }
  return dev->SetNetworkAttribute(attr, timeout_ms);
}

OdinResult OdinSdkImpl::Reboot(OdinDeviceHandle device, uint32_t timeout_ms) {
  IDevice* dev = GetDevice(device);
  if (!dev) {
    return OdinResult::kNotInitialized;
  }
  return dev->Reboot(timeout_ms);
}

}  // namespace sdk
}  // namespace odin
