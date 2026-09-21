#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "../ICommand.hpp"
#include "odin_command_channel.h"

namespace odin {
namespace sdk {

// Odin2 Command IDs
enum class Odin2CmdId : uint16_t {
  kDeviceQuery = 0x01,
  kVersionQuery = 0x02,
  kCmdIdQueryCapability = 0x03,
  kSetMode = 0x04,
  kSensorMode = 0x05,
  kChannelConfig = 0x06,
  kHeartbeat = 0x07,
};

// TLV Type IDs for ChannelConfig command
enum class ChannelConfigTlvType : uint8_t {
  kDstPort = 0x01,
  kTransportMode = 0x02,
  kResolutionId = 0x03,
  kFpsId = 0x04,
  kFormat = 0x05,
};

/**
 * @brief Odin protocol command channel implementation
 *
 * Implements the ICommand interface and encapsulates Odin2 protocol
 * command encoding and parsing.
 */
class OdinCommand : public ICommand {
 public:
  OdinCommand();
  ~OdinCommand() override;

  // ICommand interface implementation
  bool Connect(const ITransportAddress& device_address,
               const ITransportAddress* local_address,
               OdinDeviceHandle handle) override;
  void Disconnect() override;
  bool IsConnected() const override;

  OdinResult GetFirmwareVersion(std::string& version, uint32_t timeout_ms = 1000) override;
  OdinResult SetOperatingMode(OdinOperatingMode mode, uint32_t timeout_ms = 1000) override;
  OdinResult SetSensorMode(uint8_t mode, uint32_t timeout_ms = 1000) override;
  OdinResult GetSensorCapability(std::vector<OdinSensorCapability>& capabilities,
                                  const std::vector<OdinDataChannel>& channels,
                                  uint32_t timeout_ms = 1000) override;

  OdinResult SendStartStream(OdinDataChannel channel, OdinTransportMode transport,
                              uint16_t dst_port, const OdinStreamCfg* cfg = nullptr,
                              uint32_t timeout_ms = 1000) override;
  OdinResult SendCloseStream(OdinDataChannel channel, uint32_t timeout_ms = 1000) override;

  bool IsHeartbeatSupported() const override;
  bool SendHeartbeat(uint32_t interval_ms, uint32_t timeout_ms = 1000) override;

  // Internal capability table access (used by StartStream to resolve resolution_id/fps_id)
  void UpdateCapabilityTable(const std::vector<OdinSensorCapability>& capabilities);
  bool LookupStreamConfig(OdinDataChannel channel, const OdinStreamCfg* cfg,
                          uint8_t& resolution_id, uint8_t& fps_id);

 private:
  // TLV building helper
  std::vector<uint8_t> BuildChannelConfigPayload(OdinDataChannel channel,
                                                  OdinTransportMode transport,
                                                  uint16_t dst_port,
                                                  const OdinStreamCfg* cfg);

  std::unique_ptr<CommandChannel> command_channel_;
  std::unique_ptr<CommandChannel> heartbeat_channel_;
  
  std::atomic<bool> connected_{false};
  bool heartbeat_supported_ = true;
  
  OdinDeviceHandle handle_ = kInvalidDeviceHandle;
  std::string device_ip_;
  std::string host_ip_;
  
  // Internal capability table
  std::map<OdinDataChannel, std::vector<OdinStreamCfg>> capability_table_;
  mutable std::mutex mutex_;
  std::mutex capability_mutex_;
};

}  // namespace sdk
}  // namespace odin
