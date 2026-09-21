/**
 * @file odin_lidar_api.h
 * @brief Odin LiDAR SDK Public API
 *
 * C++ interface for communicating with Odin LiDAR devices.
 * Provides device discovery, connection management, data reception, and command sending.
 *
 * All type definitions are in odin_lidar_def.h.
 * This file only contains function declarations.
 */

#pragma once

#include "odin_lidar_def.h"

namespace odin {
namespace sdk {

// ============================================================================
// Device Connection Management
// ============================================================================

/**
 * @brief Connect to a discovered device
 *
 * All ports are auto-generated:
 * - device_command_port: fixed 60001
 * - Other ports: random available ports (5-digit)
 * - host_ip: from device.host_ip (discovered interface)
 *
 * @param device Discovered device from DiscoverDevices()
 * @return Device handle, or kInvalidDeviceHandle on failure
 */
OdinDeviceHandle ConnectDevice(const DiscoveredDevice &device);

/**
 * @brief Disconnect from a device
 * @param handle Device handle
 */
void DisconnectDevice(OdinDeviceHandle handle);

// ============================================================================
// Data Callback Registration
// ============================================================================

/**
 * @brief Register raw point cloud callback (data_type=0)
 * @param handle Device handle (must be connected)
 * @param cb Callback function pointer
 * @param client_data User-defined data passed to the callback
 */
void RegisterPointCloudCallback(OdinDeviceHandle handle, OdinPointCloudCallback cb,
                                void *client_data);

/**
 * @brief Register SLAM point cloud callback (data_type=1)
 * @param handle Device handle (must be connected)
 * @param cb Callback function pointer
 * @param client_data User-defined data
 */
void RegisterSlamCallback(OdinDeviceHandle handle, OdinSlamCallback cb, void *client_data);

/**
 * @brief Register image callback - left camera (data_type=2)
 * @param handle Device handle (must be connected)
 * @param cb Callback function pointer
 * @param client_data User-defined data
 */
void RegisterImageCallback(OdinDeviceHandle handle, OdinImageCallback cb, void *client_data);

/**
 * @brief Register image callback - right camera (data_type=6)
 * @param handle Device handle (must be connected)
 * @param cb Callback function pointer
 * @param client_data User-defined data
 */
void RegisterImageCallback2(OdinDeviceHandle handle, OdinImageCallback2 cb, void *client_data);

/**
 * @brief Register IMU data callback (data_type=3)
 * @param handle Device handle (must be connected)
 * @param cb Callback function pointer
 * @param client_data User-defined data
 */
void RegisterImuCallback(OdinDeviceHandle handle, OdinImuCallback cb, void *client_data);

/**
 * @brief Register odometry data callback (data_type=4)
 * @param handle Device handle (must be connected)
 * @param cb Callback function pointer
 * @param client_data User-defined data
 */
void RegisterOdomCallback(OdinDeviceHandle handle, OdinOdomCallback cb, void *client_data);

/**
 * @brief Enable/disable SLAM and Odom data synchronization for specific device
 *
 * When enabled (default), SDK matches SLAM and Odom data by frame_count before invoking callbacks.
 * SLAM points are transformed from IMU frame to world frame using Odom pose.
 * Unmatched data exceeding max_frame_lag is discarded with a warning log.
 *
 * @param handle Device handle
 * @param enabled Whether to enable synchronization (default is true)
 * @param max_frame_lag Maximum frame count difference before discarding unmatched data (default 10)
 */
void EnableSlamOdomSyncForDevice(OdinDeviceHandle handle, bool enabled,
                                 uint32_t max_frame_lag = 10);

// ============================================================================
// Command Sending
// ============================================================================

/**
 * @brief Set device operating mode synchronously (cmd_id=0x02)
 * @param device Device handle
 * @param mode Target operating mode (kStandby, kNormal, kUpgrade)
 * @param timeout_ms Timeout in milliseconds (default: 1000)
 * @return Operation result
 */
OdinResult SetOperatingMode(OdinDeviceHandle device, OdinOperatingMode mode,
                            uint32_t timeout_ms = 1000);

/**
 * @brief Get file from device via HTTPS
 * @param device Device handle
 * @param cb Progress callback function
 * @param type File type
 * @param save_path Local path to save the file
 * @return Operation result
 */
OdinResult GetDeviceFile(OdinDeviceHandle device, OdinUpgradeProgressCallback cb, OdinFileType type,
                         std::string save_path);

/**
 * @brief Send file to device (supports firmware upgrade with kFirmware type)
 *
 * For firmware upgrade (kFirmware type), this function handles the complete OTA process:
 * - Automatically selects HTTP (port 8080) or falls back to HTTPS
 * - Manages OTA state machine (upload, trigger, wait for completion)
 * - Progress callback reports 0-100%, where 100% means success
 *
 * Progress breakdown for firmware upgrade:
 * - 0-50%: Firmware upload
 * - 50-60%: Verification
 * - 60-80%: Installation (MCU/SOC)
 * - 80-95%: Reboot
 * - 95-100%: Post-verification
 * - 100%: Success
 *
 * @param device Device handle
 * @param cb Progress callback function (0-100%)
 * @param type File type (kFirmware for OTA upgrade)
 * @param file_path Path to local file
 * @return Operation result
 */
OdinResult SendFileToDevice(OdinDeviceHandle device, OdinUpgradeProgressCallback cb,
                            OdinFileType type, std::string file_path);

/**
 * @brief Get device firmware version synchronously (cmd_id=0x06)
 * @param device Device handle
 * @param version Output firmware version string (e.g., "1.2.3")
 * @param timeout_ms Timeout in milliseconds (default: 1000)
 * @return Operation result
 */
OdinResult GetFirmwareVersion(OdinDeviceHandle device, std::string &version,
                              uint32_t timeout_ms = 1000);

/**
 * @brief Set sensor mode synchronously (cmd_id=0x07)
 * @param device Device handle
 * @param mode Sensor mode (0: HDR, 1: High Peak, 2: Low Peak, 3: Adaptive)
 * @param timeout_ms Timeout in milliseconds (default: 1000)
 * @return Operation result
 */
OdinResult SetSensorMode(OdinDeviceHandle device, uint8_t mode, uint32_t timeout_ms = 1000);

// ============================================================================
// Stream Control
// ============================================================================

/**
 * @brief Start a data stream (cmd_id=0x05) - Synchronous
 *
 * Starts a data stream with specified transport mode (UDP/TCP).
 * Port is auto-allocated internally by the SDK.
 * This is a blocking call that waits for device response.
 *
 * @param device Device handle
 * @param channel Data channel type (kRawPoint, kImage0, kImu, etc.)
 * @param transport Transport mode (kUdp or kTcp)
 * @param timeout_ms Timeout in milliseconds (default: 1000)
 * @return Operation result
 */
OdinResult StartStream(OdinDeviceHandle device, OdinDataChannel channel,
                       OdinTransportMode transport, const OdinStreamCfg* mode = nullptr,
                       uint32_t timeout_ms = 1000);

/**
 * @brief Get sensor capabilities (cmd_id=0x06) - Synchronous
 *
 * Queries the device for supported sensor modes for specified channels.
 * Format field interpretation depends on channel type (all use OdinDataFormat):
 * - Image: kImageMjpeg, kImageYuyv, kImageNv12, etc.
 * - PointCloud: kRawPointXyzic, kSlamPointXyzrgba, etc.
 * - IMU: kImu6Axis, kImu9Axis
 * - Odom: kOdomStandard
 *
 * @param device Device handle
 * @param capabilities Output: list of sensor capabilities
 * @param channels Channels to query (default empty = query all)
 * @param timeout_ms Timeout in milliseconds (default: 1000)
 * @return Operation result
 */
OdinResult GetSensorCapability(OdinDeviceHandle device,
                               std::vector<OdinSensorCapability>& capabilities,
                               const std::vector<OdinDataChannel>& channels = {},
                               uint32_t timeout_ms = 1000);

/**
 * @brief Close a data stream (cmd_id=0x05) - Synchronous
 *
 * Stops a data stream by sending dst_port=0 to device.
 * This is a blocking call that waits for device response.
 *
 * @param device Device handle
 * @param channel Data channel type to close
 * @param timeout_ms Timeout in milliseconds (default: 1000)
 * @return Operation result
 */
OdinResult CloseStream(OdinDeviceHandle device, OdinDataChannel channel,
                       uint32_t timeout_ms = 1000);

// ============================================================================
// Network Configuration
// ============================================================================

/**
 * @brief Get device network attribute configuration
 *
 * Retrieves current network settings (IP, gateway, netmask, DHCP, PTP) via HTTP API.
 *
 * @param device Device handle
 * @param attr Output: network attribute structure
 * @param timeout_ms Timeout in milliseconds (default: 3000)
 * @return Operation result
 */
OdinResult GetNetworkAttribute(OdinDeviceHandle device, NetworkAttribute& attr,
                               uint32_t timeout_ms = 3000);

/**
 * @brief Set device network attribute configuration
 *
 * Configures network settings (IP, gateway, netmask, DHCP, PTP) via HTTP API.
 * Changes take effect after device reboot.
 *
 * @param device Device handle
 * @param attr Network attribute to set
 * @param timeout_ms Timeout in milliseconds (default: 3000)
 * @return Operation result
 */
OdinResult SetNetworkAttribute(OdinDeviceHandle device, const NetworkAttribute& attr,
                               uint32_t timeout_ms = 3000);

/**
 * @brief Reboot the device
 *
 * Sends reboot command via HTTP API. Device will restart.
 * Network settings changes take effect after reboot.
 *
 * @param device Device handle
 * @param timeout_ms Timeout in milliseconds (default: 3000)
 * @return Operation result
 */
OdinResult Reboot(OdinDeviceHandle device, uint32_t timeout_ms = 3000);

// ============================================================================
// Device Discovery
// ============================================================================

/**
 * @brief Broadcast discover devices (cmd_id=0x01)
 *
 * Automatically enumerates all network interfaces and discovers devices on each subnet.
 * Uses a randomly generated port internally, retrying with different ports on failure.
 *
 * @param devices Output: list of discovered devices (deduplicated by SN)
 * @param timeout_ms Discovery timeout in milliseconds (divided among interfaces)
 * @return true if any device was found
 */
bool DiscoverDevices(std::vector<DiscoveredDevice> &devices, uint32_t timeout_ms = 2000);

/**
 * @brief Get SDK version string
 * @return Version string, e.g., "0.3.0_20260205"
 */
std::string GetSdkVersion();

}  // namespace sdk
}  // namespace odin

// Include hotplug types after namespace close to avoid circular dependency
#include "../sdk/hotplugListener/IHotplugListener/IHotplugListener.h"

namespace odin {
namespace sdk {

// ============================================================================
// Hotplug Listener
// ============================================================================

/**
 * @brief Start hotplug listener to detect device arrival/removal
 *
 * Uses UDP broadcast-based device discovery internally.
 * Separate callbacks for device online and offline events.
 *
 * @param callbacks Structure containing on_attach and on_detach callbacks
 * @param enumerate_existing If true, report already-connected devices via on_attach
 * @return true if started successfully
 */
bool StartHotplugListener(const HotplugCallbacks &callbacks, bool enumerate_existing = true);

/**
 * @brief Stop hotplug listener
 */
void StopHotplugListener();

/**
 * @brief Check if hotplug listener is running
 * @return true if running
 */
bool IsHotplugListenerRunning();

/**
 * @brief Set hotplug polling interval
 * @param interval_ms Interval in milliseconds (default 1000)
 */
void SetHotplugPollingInterval(uint32_t interval_ms);

/**
 * @brief Set hotplug offline timeout
 * @param timeout_ms Timeout in milliseconds (default 3000)
 */
void SetHotplugOfflineTimeout(uint32_t timeout_ms);

}  // namespace sdk
}  // namespace odin
