#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>

#include "IDevice.h"
#include "ICommand.hpp"
#include "ICapture.hpp"
#include "OdinFrameAssembler.hpp"
#include "SlamOdomSynchronizer.h"
#include "https/HttpsFileTransfer.hpp"

namespace odin {
namespace sdk {

/**
 * @brief Device port configuration
 */
struct DevicePorts {
  uint16_t raw_point = 0;
  uint16_t slam = 0;
  uint16_t jpeg = 0;
  uint16_t imu = 0;
  uint16_t odom = 0;
  uint16_t file = 0;
  uint16_t jpeg2 = 0;
};

/**
 * @brief File information structure for file transfer
 */
#pragma pack(push, 1)
struct FileInfo {
  char filename[128];
  uint64_t filesize;
  uint8_t md5[16];
};
#pragma pack(pop)

/**
 * @brief Odin2 device implementation (Ethernet)
 *
 * Refactored device class that separates command handling and data capture via the
 * ICommand and ICapture interfaces. Odin2Device is only responsible for business
 * logic orchestration.
 */
class Odin2Device : public IDevice {
 public:
  explicit Odin2Device(OdinDeviceHandle handle);
  ~Odin2Device() override;

  // IDevice interface implementation
  bool Connect(const DiscoveredDevice& discovered_device) override;
  void Disconnect() override;
  bool IsConnected() const override;

  OdinDeviceHandle GetHandle() const override;
  std::string GetSerialNumber() const override;
  std::string GetModel() const override;
  MTConnectionType GetConnectionType() const override;
  const DiscoveredDevice& GetDiscoveredDevice() const override;

  OdinResult GetFirmwareVersion(std::string& version, uint32_t timeout_ms = 1000) override;
  OdinResult SetOperatingMode(OdinOperatingMode mode, uint32_t timeout_ms = 1000) override;

  // Callback registration - forwarded to capture_
  void RegisterPointCloudCallback(OdinPointCloudCallback cb, void* user_data) override;
  void RegisterSlamCallback(OdinSlamCallback cb, void* user_data) override;
  void RegisterImageCallback(OdinImageCallback cb, void* user_data) override;
  void RegisterImageCallback2(OdinImageCallback2 cb, void* user_data) override;
  void RegisterImuCallback(OdinImuCallback cb, void* user_data) override;
  void RegisterOdomCallback(OdinOdomCallback cb, void* user_data) override;

  // Data channel management
  void StopDataChannels() override;

  OdinResult SendFile(OdinFileType type, const std::string& file_path,
                      OdinUpgradeProgressCallback cb) override;
  OdinResult GetFile(OdinFileType type, const std::string& save_path,
                     OdinUpgradeProgressCallback cb) override;
  OdinResult SetSensorMode(uint8_t mode, uint32_t timeout_ms = 1000) override;
  OdinResult StartStream(OdinDataChannel channel, OdinTransportMode transport,
                         const OdinStreamCfg* mode = nullptr,
                         uint32_t timeout_ms = 1000) override;
  OdinResult GetSensorCapability(std::vector<OdinSensorCapability>& capabilities,
                                 const std::vector<OdinDataChannel>& channels,
                                 uint32_t timeout_ms) override;
  OdinResult CloseStream(OdinDataChannel channel, uint32_t timeout_ms) override;
  void EnableSlamOdomSync(bool enabled, uint32_t max_frame_lag = 10) override;
  void SetHeartbeatFailedCallback(HeartbeatFailedCallback cb) override;
  void SetHeartbeatInterval(uint32_t interval_ms);
  void SetHeartbeatTimeout(uint32_t timeout_ms);

  // Network configuration
  OdinResult GetNetworkAttribute(NetworkAttribute& attr, uint32_t timeout_ms = 3000) override;
  OdinResult SetNetworkAttribute(const NetworkAttribute& attr, uint32_t timeout_ms = 3000) override;
  OdinResult Reboot(uint32_t timeout_ms = 3000) override;

 private:
  // Port generation helper
  uint16_t GenerateAvailablePort();
  bool IsPortAvailable(const std::string& ip, uint16_t port);
  std::set<uint16_t> used_ports_;

  // Firmware OTA helper
  OdinResult SendFileFirmwareOta(const std::string& file_path, OdinUpgradeProgressCallback cb);

  // Heartbeat management
  void StartHeartbeat();
  void StopHeartbeat();
  void HeartbeatThread();

  // Device information (must be first for initialization order)
  OdinDeviceHandle handle_;

  // Core components
  std::unique_ptr<ICommand> command_;   // Command layer
  
  // Capture instances (one per channel)
  std::map<OdinDataChannel, std::unique_ptr<ICapture>> captures_;
  
  // Frame assemblers for protocol parsing (one per channel type)
  std::map<OdinDataChannel, std::unique_ptr<OdinFrameAssembler>> assemblers_;
  
  // User callbacks
  OdinPointCloudCallback point_cloud_cb_ = nullptr;
  void* point_cloud_user_data_ = nullptr;
  OdinSlamCallback slam_cb_ = nullptr;
  void* slam_user_data_ = nullptr;
  OdinImageCallback image_cb_ = nullptr;
  void* image_user_data_ = nullptr;
  OdinImageCallback2 image2_cb_ = nullptr;
  void* image2_user_data_ = nullptr;
  OdinImuCallback imu_cb_ = nullptr;
  void* imu_user_data_ = nullptr;
  OdinOdomCallback odom_cb_ = nullptr;
  void* odom_user_data_ = nullptr;
  DevicePorts ports_;
  std::string host_ip_;
  std::string device_ip_;
  std::string serial_number_;
  std::string model_;
  std::string firmware_version_;
  DiscoveredDevice discovered_device_;

  std::atomic<bool> connected_{false};
  mutable std::mutex mutex_;

  // File transfer
  std::unique_ptr<HttpsFileTransfer> https_transfer_;
  std::atomic<bool> file_transfer_active_{false};
  FileInfo file_info_;
  std::string save_path_;
  uint64_t bytes_transferred_{0};
  FILE* save_file_fd_{nullptr};
  std::mutex file_mutex_;

  // Heartbeat thread
  std::thread heartbeat_thread_;
  std::atomic<bool> heartbeat_running_{false};
  uint32_t heartbeat_interval_ms_ = 500;
  int heartbeat_max_failures_ = 6;
  HeartbeatFailedCallback heartbeat_failed_cb_;

  // SLAM-Odom synchronizer
  std::unique_ptr<SlamOdomSynchronizer> slam_odom_sync_;
};

}  // namespace sdk
}  // namespace odin
