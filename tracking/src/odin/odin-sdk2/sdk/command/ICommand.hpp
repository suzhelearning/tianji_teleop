#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "odin_lidar_def.h"

namespace odin {
namespace sdk {

// Forward declaration - keep ICommand transport-agnostic
struct ITransportAddress;

/**
 * @brief Command channel abstract interface
 *
 * Responsible for encoding, sending, and parsing responses of device commands.
 * Different protocols (Odin protocol, GVCP, etc.) implement this interface.
 * The interface is decoupled from the underlying transport (TCP/UDP/USB/Serial).
 */
class ICommand {
 public:
  virtual ~ICommand() = default;

  // =========================================================================
  // Connection management
  // =========================================================================
  
  /**
   * @brief Connect to device
   * @param device_address Device address (transport-agnostic, parsed by implementation)
   * @param local_address  Local bind address (nullable, implementation chooses default)
   * @param handle         Device handle
   * @return true on successful connection
   */
  virtual bool Connect(const ITransportAddress& device_address,
                       const ITransportAddress* local_address,
                       OdinDeviceHandle handle) = 0;
  
  /**
   * @brief Disconnect from device
   */
  virtual void Disconnect() = 0;
  
  /**
   * @brief Check whether currently connected
   */
  virtual bool IsConnected() const = 0;

  // =========================================================================
  // Device commands
  // =========================================================================
  
  /**
   * @brief Get firmware version
   */
  virtual OdinResult GetFirmwareVersion(std::string& version, uint32_t timeout_ms = 1000) = 0;
  
  /**
   * @brief Set operating mode
   */
  virtual OdinResult SetOperatingMode(OdinOperatingMode mode, uint32_t timeout_ms = 1000) = 0;
  
  /**
   * @brief Set sensor mode
   */
  virtual OdinResult SetSensorMode(uint8_t mode, uint32_t timeout_ms = 1000) = 0;
  
  /**
   * @brief Get sensor capabilities
   */
  virtual OdinResult GetSensorCapability(std::vector<OdinSensorCapability>& capabilities,
                                          const std::vector<OdinDataChannel>& channels,
                                          uint32_t timeout_ms = 1000) = 0;

  // =========================================================================
  // Stream control commands
  // =========================================================================
  
  /**
   * @brief Send start-stream command
   * @param channel    Data channel
   * @param transport  Transport mode (UDP/TCP)
   * @param dst_port   Destination port
   * @param cfg        Stream configuration (optional, image channels only)
   * @param timeout_ms Timeout in milliseconds
   * @return Command execution result
   */
  virtual OdinResult SendStartStream(OdinDataChannel channel, OdinTransportMode transport,
                                      uint16_t dst_port, const OdinStreamCfg* cfg = nullptr,
                                      uint32_t timeout_ms = 1000) = 0;
  
  /**
   * @brief Send close-stream command
   */
  virtual OdinResult SendCloseStream(OdinDataChannel channel, uint32_t timeout_ms = 1000) = 0;

  // =========================================================================
  // Heartbeat
  // =========================================================================
  
  /**
   * @brief Check whether the device supports heartbeat
   */
  virtual bool IsHeartbeatSupported() const = 0;
  
  /**
   * @brief Send heartbeat
   * @param interval_ms Heartbeat interval in milliseconds
   * @param timeout_ms  Timeout in milliseconds
   * @return true on success
   */
  virtual bool SendHeartbeat(uint32_t interval_ms, uint32_t timeout_ms = 1000) = 0;
};

}  // namespace sdk
}  // namespace odin
