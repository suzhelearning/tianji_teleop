#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "odin_lidar_def.h"  // Types only - no API function dependencies

namespace odin {
namespace sdk {

/**
 * @brief Device interface - abstract base class for all device types
 *
 * This interface allows OdinSdkImpl to work with different device types
 * (network, USB, 5G, Bluetooth) through a common abstraction.
 */
class IDevice {
 public:
  virtual ~IDevice() = default;

  // =========================================================================
  // Connection Management
  // =========================================================================

  /**
   * @brief Initialize and connect to the device
   * @param discovered_device Device info from discovery
   * @return true if connection successful
   */
  virtual bool Connect(const DiscoveredDevice& discovered_device) = 0;

  /**
   * @brief Disconnect from the device
   */
  virtual void Disconnect() = 0;

  /**
   * @brief Check if device is connected
   * @return true if connected
   */
  virtual bool IsConnected() const = 0;

  // =========================================================================
  // Device Information
  // =========================================================================

  /**
   * @brief Get device handle
   * @return Device handle
   */
  virtual OdinDeviceHandle GetHandle() const = 0;

  /**
   * @brief Get device serial number
   * @return Serial number string
   */
  virtual std::string GetSerialNumber() const = 0;

  /**
   * @brief Get device model name
   * @return Model name string
   */
  virtual std::string GetModel() const = 0;

  /**
   * @brief Get connection type
   * @return Connection type enum
   */
  virtual MTConnectionType GetConnectionType() const = 0;

  /**
   * @brief Get discovered device info (saved from Connect)
   * @return DiscoveredDevice structure
   */
  virtual const DiscoveredDevice& GetDiscoveredDevice() const = 0;

  // =========================================================================
  // Command Interface
  // =========================================================================

  /**
   * @brief Get device firmware version (synchronous)
   * @param version Output firmware version string (e.g., "1.2.3")
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult GetFirmwareVersion(std::string& version, uint32_t timeout_ms = 1000) = 0;

  /**
   * @brief Set operating mode synchronously
   * @param mode Operating mode (kStandby, kNormal, kUpgrade)
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult SetOperatingMode(OdinOperatingMode mode, uint32_t timeout_ms = 1000) = 0;

  // =========================================================================
  // Data Streaming & Callbacks
  // =========================================================================

  /**
   * @brief Register callbacks for data processing
   * Device handles its own data parsing and invokes these callbacks
   */
  virtual void RegisterPointCloudCallback(OdinPointCloudCallback cb, void* user_data) = 0;
  virtual void RegisterSlamCallback(OdinSlamCallback cb, void* user_data) = 0;
  virtual void RegisterImageCallback(OdinImageCallback cb, void* user_data) = 0;
  virtual void RegisterImageCallback2(OdinImageCallback2 cb, void* user_data) = 0;
  virtual void RegisterImuCallback(OdinImuCallback cb, void* user_data) = 0;
  virtual void RegisterOdomCallback(OdinOdomCallback cb, void* user_data) = 0;

  /**
   * @brief Stop data channels
   */
  virtual void StopDataChannels() = 0;

  // =========================================================================
  // File Operations
  // =========================================================================

  /**
   * @brief Send file to device
   *
   * For firmware upgrade (kFirmware type), handles complete OTA process:
   * - Auto-selects HTTP/HTTPS, manages state machine, waits for completion
   * - Progress: 0-100%, 100% means success
   *
   * @param type File type (kFirmware for OTA upgrade)
   * @param file_path Local file path
   * @param cb Progress callback (0-100%)
   * @return Operation result
   */
  virtual OdinResult SendFile(OdinFileType type, const std::string& file_path,
                              OdinUpgradeProgressCallback cb) = 0;

  /**
   * @brief Get file from device
   * @param type File type
   * @param save_path Local save path
   * @param cb Progress callback
   * @return Operation result
   */
  virtual OdinResult GetFile(OdinFileType type, const std::string& save_path,
                             OdinUpgradeProgressCallback cb) = 0;

  // =========================================================================
  // Sensor Configuration
  // =========================================================================

  /**
   * @brief Set sensor mode (synchronous)
   * @param mode Sensor mode value (0=HDR, 1=High Peak, 2=Low Peak, 3=Adaptive)
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult SetSensorMode(uint8_t mode, uint32_t timeout_ms = 1000) = 0;

  /**
   * @brief Start a data stream (cmd_id=0x05) - Synchronous
   * @param channel Data channel type
   * @param transport Transport mode (UDP/TCP)
   * @param mode Image stream mode (optional, only for Image channels)
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult StartStream(OdinDataChannel channel, OdinTransportMode transport,
                                 const OdinStreamCfg* mode = nullptr,
                                 uint32_t timeout_ms = 1000) = 0;

  /**
   * @brief Get sensor capabilities (cmd_id=0x06) - Synchronous
   * @param capabilities Output: list of sensor capabilities
   * @param channels Channels to query (empty = query all)
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult GetSensorCapability(std::vector<OdinSensorCapability>& capabilities,
                                         const std::vector<OdinDataChannel>& channels,
                                         uint32_t timeout_ms) = 0;

  /**
   * @brief Close a data stream (cmd_id=0x05) - Synchronous
   * @param channel Data channel type
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult CloseStream(OdinDataChannel channel, uint32_t timeout_ms) = 0;

  // =========================================================================
  // SLAM-Odom Synchronization
  // =========================================================================

  /**
   * @brief Enable/disable SLAM-Odom synchronization for this device
   * @param enabled Whether to enable sync
   * @param max_frame_lag Maximum frame count difference before discarding unmatched data
   */
  virtual void EnableSlamOdomSync(bool enabled, uint32_t max_frame_lag = 10) = 0;

  // =========================================================================
  // Network Configuration
  // =========================================================================

  /**
   * @brief Get device network attribute configuration
   *
   * Retrieves current network settings (IP, gateway, netmask, DHCP, PTP) via HTTP API.
   *
   * @param attr Output: network attribute structure
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult GetNetworkAttribute(NetworkAttribute& attr, uint32_t timeout_ms = 3000) = 0;

  /**
   * @brief Set device network attribute configuration
   *
   * Configures network settings (IP, gateway, netmask, DHCP, PTP) via HTTP API.
   * Changes take effect after device reboot.
   *
   * @param attr Network attribute to set
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult SetNetworkAttribute(const NetworkAttribute& attr, uint32_t timeout_ms = 3000) = 0;

  /**
   * @brief Reboot the device
   *
   * Sends reboot command via HTTP API. Device will restart.
   * Network settings changes take effect after reboot.
   *
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult Reboot(uint32_t timeout_ms = 3000) = 0;

  // =========================================================================
  // Connection Status Callbacks
  // =========================================================================

  /**
   * @brief Callback type for heartbeat failure notification
   * @param handle Device handle
   */
  using HeartbeatFailedCallback = std::function<void(OdinDeviceHandle handle)>;

  /**
   * @brief Register callback for heartbeat failure
   * Called when heartbeat fails (device offline)
   * @param cb Callback function
   */
  virtual void SetHeartbeatFailedCallback(HeartbeatFailedCallback cb) = 0;
};

}  // namespace sdk
}  // namespace odin
