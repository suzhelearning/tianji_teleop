#include "OdinCommand.hpp"

#include "cJSON.h"
#include "ITransport.hpp"
#include "TcpTransport.hpp"
#include "OdinProtocol.hpp"
#include "logger.h"

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#endif

namespace odin {
namespace sdk {

namespace {

#ifdef _WIN32
std::atomic<int> g_winsock_ref_count{0};

bool AcquireWinsock() {
  if (g_winsock_ref_count.fetch_add(1) == 0) {
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
      g_winsock_ref_count.fetch_sub(1);
      return false;
    }
  }
  return true;
}

void ReleaseWinsock() {
  if (g_winsock_ref_count.fetch_sub(1) == 1) {
    WSACleanup();
  }
}
#else
bool AcquireWinsock() { return true; }
void ReleaseWinsock() {}
#endif

}  // namespace

OdinCommand::OdinCommand() = default;

OdinCommand::~OdinCommand() {
  Disconnect();
}

bool OdinCommand::Connect(const ITransportAddress& device_address,
                           const ITransportAddress* local_address,
                           OdinDeviceHandle handle) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (connected_.load()) return true;

  // Resolve device address: Odin protocol uses NetworkAddress (TCP/UDP).
  const auto* dev_net = dynamic_cast<const NetworkAddress*>(&device_address);
  if (!dev_net) {
    LOG_ERROR("OdinCommand::Connect: unsupported device address type\n");
    return false;
  }
  device_ip_ = dev_net->ip;
  if (device_ip_.empty()) {
    LOG_ERROR("OdinCommand::Connect: empty device IP\n");
    return false;
  }

  // Resolve local address (optional - default to "0.0.0.0:0")
  NetworkAddress local_addr;
  local_addr.ip = "0.0.0.0";
  local_addr.port = 0;
  if (local_address) {
    const auto* loc_net = dynamic_cast<const NetworkAddress*>(local_address);
    if (loc_net && !loc_net->ip.empty()) {
      local_addr.ip = loc_net->ip;
      local_addr.port = loc_net->port;
    }
  }
  host_ip_ = local_addr.ip;
  handle_ = handle;

  if (!AcquireWinsock()) {
    LOG_ERROR("OdinCommand: AcquireWinsock failed\n");
    return false;
  }

  // Step 1: Try heartbeat channel (TCP 60002)
  heartbeat_supported_ = true;
  
  std::unique_ptr<ITransport> hb_transport(new TcpTransport());
  NetworkAddress hb_remote_addr;
  hb_remote_addr.ip = device_ip_;
  hb_remote_addr.port = 60002;
  hb_transport->SetRemoteTarget(hb_remote_addr);

  if (!hb_transport->Open(local_addr)) {
    LOG_WARN("OdinCommand: Heartbeat transport Open failed (port 60002)\n");
    heartbeat_supported_ = false;
  } else {
    std::unique_ptr<IProtocol> hb_protocol(new OdinProtocolV1());
    heartbeat_channel_.reset(new CommandChannel(std::move(hb_protocol), std::move(hb_transport)));

    CommandChannelConfig hb_cfg;
    hb_cfg.default_timeout_ms = 2000;
    hb_cfg.device_handle = handle_;

    if (!heartbeat_channel_->Start(hb_cfg)) {
      LOG_WARN("OdinCommand: Heartbeat CommandChannel start failed\n");
      heartbeat_channel_.reset();
      heartbeat_supported_ = false;
    } else {
      // Send first heartbeat to verify
      std::vector<uint8_t> hb_payload;
      OdinCommandSyncResponse hb_response;
      if (!heartbeat_channel_->SendSync(static_cast<uint16_t>(Odin2CmdId::kHeartbeat), 
                                         hb_payload, hb_response, 2000)) {
        LOG_WARN("OdinCommand: First heartbeat failed\n");
        heartbeat_channel_->Stop();
        heartbeat_channel_.reset();
        heartbeat_supported_ = false;
      } else {
        LOG_INFO("OdinCommand: Heartbeat channel established (port 60002)\n");
      }
    }
  }

  if (!heartbeat_supported_) {
    LOG_INFO("OdinCommand: Device does not support heartbeat\n");
  }

  // Step 2: Command channel (TCP 60001)
  std::unique_ptr<ITransport> cmd_transport(new TcpTransport());
  NetworkAddress cmd_remote_addr;
  cmd_remote_addr.ip = device_ip_;
  cmd_remote_addr.port = 60001;
  cmd_transport->SetRemoteTarget(cmd_remote_addr);

  if (!cmd_transport->Open(local_addr)) {
    LOG_ERROR("OdinCommand: Command transport Open failed (port 60001)\n");
    if (heartbeat_channel_) {
      heartbeat_channel_->Stop();
      heartbeat_channel_.reset();
    }
    ReleaseWinsock();
    return false;
  }

  std::unique_ptr<IProtocol> cmd_protocol(new OdinProtocolV1());
  command_channel_.reset(new CommandChannel(std::move(cmd_protocol), std::move(cmd_transport)));

  CommandChannelConfig cmd_cfg;
  cmd_cfg.default_timeout_ms = 3000;
  cmd_cfg.device_handle = handle_;

  if (!command_channel_->Start(cmd_cfg)) {
    LOG_ERROR("OdinCommand: Command CommandChannel start failed\n");
    command_channel_.reset();
    if (heartbeat_channel_) {
      heartbeat_channel_->Stop();
      heartbeat_channel_.reset();
    }
    ReleaseWinsock();
    return false;
  }

  connected_.store(true);
  LOG_INFO("OdinCommand: Connected to %s\n", device_ip_.c_str());
  return true;
}

void OdinCommand::Disconnect() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!connected_.load()) return;

  if (command_channel_) {
    command_channel_->Stop();
    command_channel_.reset();
  }

  if (heartbeat_channel_) {
    heartbeat_channel_->Stop();
    heartbeat_channel_.reset();
  }

  ReleaseWinsock();
  connected_.store(false);
  LOG_INFO("OdinCommand: Disconnected\n");
}

bool OdinCommand::IsConnected() const {
  return connected_.load();
}

OdinResult OdinCommand::GetFirmwareVersion(std::string& version, uint32_t timeout_ms) {
  std::vector<uint8_t> payload;

  std::lock_guard<std::mutex> lock(mutex_);
  if (!command_channel_) return OdinResult::kNotInitialized;

  OdinCommandSyncResponse response;
  if (!command_channel_->SendSync(static_cast<uint16_t>(Odin2CmdId::kVersionQuery), 
                                  payload, response, timeout_ms)) {
    return OdinResult::kTimeout;
  }

  if (response.result != OdinResult::kOk) {
    return response.result;
  }

  if (response.response.payload.size() >= 4) {
    uint8_t major = response.response.payload[1];
    uint8_t minor = response.response.payload[2];
    uint8_t patch = response.response.payload[3];
    version = std::to_string(major) + "." + std::to_string(minor) + "." + std::to_string(patch);
  } else {
    version = "unknown";
  }

  return OdinResult::kOk;
}

OdinResult OdinCommand::SetOperatingMode(OdinOperatingMode mode, uint32_t timeout_ms) {
  std::vector<uint8_t> payload;
  payload.push_back(static_cast<uint8_t>(mode));

  LOG_DEBUG("OdinCommand::SetOperatingMode: mode=%d\n", static_cast<int>(mode));

  std::lock_guard<std::mutex> lock(mutex_);
  if (!command_channel_) return OdinResult::kNotInitialized;

  OdinCommandSyncResponse response;
  if (!command_channel_->SendSync(static_cast<uint16_t>(Odin2CmdId::kSetMode), 
                                  payload, response, timeout_ms)) {
    return OdinResult::kTimeout;
  }

  return response.result;
}

OdinResult OdinCommand::SetSensorMode(uint8_t mode, uint32_t timeout_ms) {
  std::vector<uint8_t> payload;
  payload.push_back(0x00);
  payload.push_back(mode);

  LOG_DEBUG("OdinCommand::SetSensorMode: mode=%d\n", mode);

  std::lock_guard<std::mutex> lock(mutex_);
  if (!command_channel_) return OdinResult::kNotInitialized;

  OdinCommandSyncResponse response;
  if (!command_channel_->SendSync(static_cast<uint16_t>(Odin2CmdId::kSensorMode), 
                                  payload, response, timeout_ms)) {
    LOG_ERROR("OdinCommand::SetSensorMode: timeout\n");
    return OdinResult::kTimeout;
  }
  return response.result;
}

OdinResult OdinCommand::GetSensorCapability(std::vector<OdinSensorCapability>& capabilities,
                                             const std::vector<OdinDataChannel>& channels,
                                             uint32_t timeout_ms) {
  capabilities.clear();

  std::vector<uint8_t> payload;
  for (auto ch : channels) {
    payload.push_back(static_cast<uint8_t>(ch));
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!command_channel_) return OdinResult::kNotInitialized;

  OdinCommandSyncResponse response;
  if (!command_channel_->SendSync(static_cast<uint16_t>(Odin2CmdId::kCmdIdQueryCapability), 
                                  payload, response, timeout_ms)) {
    LOG_ERROR("OdinCommand::GetSensorCapability: timeout\n");
    return OdinResult::kTimeout;
  }

  const auto& resp_payload = response.response.payload;
  if (resp_payload.empty()) {
    LOG_ERROR("OdinCommand::GetSensorCapability: empty response\n");
    return OdinResult::kUnknownError;
  }

  // JSON format
  if (resp_payload[0] == '{') {
    std::string json_str(resp_payload.begin(), resp_payload.end());
    cJSON* root = cJSON_Parse(json_str.c_str());
    if (!root) {
      LOG_ERROR("OdinCommand::GetSensorCapability: JSON parse failed\n");
      return OdinResult::kUnknownError;
    }

    cJSON* channels_arr = cJSON_GetObjectItem(root, "channels");
    if (channels_arr && cJSON_IsArray(channels_arr)) {
      int arr_size = cJSON_GetArraySize(channels_arr);
      for (int i = 0; i < arr_size; ++i) {
        cJSON* ch_obj = cJSON_GetArrayItem(channels_arr, i);
        if (!ch_obj || !cJSON_IsObject(ch_obj)) continue;

        OdinSensorCapability cap;

        cJSON* data_type = cJSON_GetObjectItem(ch_obj, "data_type");
        if (data_type && cJSON_IsNumber(data_type)) {
          cap.channel = static_cast<OdinDataChannel>(data_type->valueint);
        }

        cJSON* format = cJSON_GetObjectItem(ch_obj, "format");
        OdinDataFormat channel_format = OdinDataFormat::kUnknown;
        if (format && cJSON_IsNumber(format)) {
          channel_format = static_cast<OdinDataFormat>(format->valueint);
        }

        cJSON* resolutions = cJSON_GetObjectItem(ch_obj, "resolutions");
        if (resolutions && cJSON_IsArray(resolutions)) {
          int res_count = cJSON_GetArraySize(resolutions);
          for (int j = 0; j < res_count; ++j) {
            cJSON* res_obj = cJSON_GetArrayItem(resolutions, j);
            if (!res_obj || !cJSON_IsObject(res_obj)) continue;

            OdinStreamCfg cfg;
            cfg.format = channel_format;

            cJSON* width = cJSON_GetObjectItem(res_obj, "width");
            if (width && cJSON_IsNumber(width)) {
              cfg.width = static_cast<uint16_t>(width->valueint);
            }

            cJSON* height = cJSON_GetObjectItem(res_obj, "height");
            if (height && cJSON_IsNumber(height)) {
              cfg.height = static_cast<uint16_t>(height->valueint);
            }

            cJSON* fps_arr = cJSON_GetObjectItem(res_obj, "fps");
            if (fps_arr && cJSON_IsArray(fps_arr)) {
              int fps_count = cJSON_GetArraySize(fps_arr);
              for (int k = 0; k < fps_count; ++k) {
                cJSON* fps_val = cJSON_GetArrayItem(fps_arr, k);
                if (fps_val && cJSON_IsNumber(fps_val)) {
                  OdinStreamCfg cfg_copy = cfg;
                  cfg_copy.fps = static_cast<uint16_t>(fps_val->valueint);
                  cap.modes.push_back(cfg_copy);
                }
              }
            } else {
              cap.modes.push_back(cfg);
            }
          }
        }

        if (channels.empty()) {
          capabilities.push_back(cap);
        } else {
          for (auto ch : channels) {
            if (ch == cap.channel) {
              capabilities.push_back(cap);
              break;
            }
          }
        }
      }
    }

    cJSON_Delete(root);
    UpdateCapabilityTable(capabilities);
    return OdinResult::kOk;
  }

  // Binary TLV format fallback
  if (resp_payload.size() < 2) {
    LOG_ERROR("OdinCommand::GetSensorCapability: response too short\n");
    return OdinResult::kUnknownError;
  }

  uint8_t ret_code = resp_payload[0];
  if (ret_code != 0) {
    LOG_ERROR("OdinCommand::GetSensorCapability: error code %d\n", ret_code);
    return OdinResult::kUnknownError;
  }

  uint8_t channel_count = resp_payload[1];
  size_t offset = 2;

  for (uint8_t i = 0; i < channel_count && offset < resp_payload.size(); i++) {
    if (offset + 2 > resp_payload.size()) break;

    OdinSensorCapability cap;
    cap.channel = static_cast<OdinDataChannel>(resp_payload[offset++]);
    uint8_t mode_count = resp_payload[offset++];

    for (uint8_t j = 0; j < mode_count && offset + 6 <= resp_payload.size(); j++) {
      OdinStreamCfg cfg;
      cfg.width = (static_cast<uint16_t>(resp_payload[offset]) << 8) |
                  static_cast<uint16_t>(resp_payload[offset + 1]);
      cfg.height = (static_cast<uint16_t>(resp_payload[offset + 2]) << 8) |
                   static_cast<uint16_t>(resp_payload[offset + 3]);
      cfg.fps = resp_payload[offset + 4];
      cfg.format = static_cast<OdinDataFormat>(resp_payload[offset + 5]);
      offset += 6;
      cap.modes.push_back(cfg);
    }

    if (channels.empty()) {
      capabilities.push_back(cap);
    } else {
      for (auto ch : channels) {
        if (ch == cap.channel) {
          capabilities.push_back(cap);
          break;
        }
      }
    }
  }

  UpdateCapabilityTable(capabilities);
  return OdinResult::kOk;
}

void OdinCommand::UpdateCapabilityTable(const std::vector<OdinSensorCapability>& capabilities) {
  std::lock_guard<std::mutex> lock(capability_mutex_);
  for (const auto& cap : capabilities) {
    capability_table_[cap.channel] = cap.modes;
  }
}

bool OdinCommand::LookupStreamConfig(OdinDataChannel channel, const OdinStreamCfg* cfg,
                                      uint8_t& resolution_id, uint8_t& fps_id) {
  resolution_id = 0xFF;
  fps_id = 0xFF;
  
  if (!cfg || (cfg->width == 0 && cfg->height == 0)) {
    return false;
  }

  std::lock_guard<std::mutex> lock(capability_mutex_);
  auto it = capability_table_.find(channel);
  if (it == capability_table_.end()) {
    return false;
  }

  const auto& configs = it->second;
  for (size_t i = 0; i < configs.size(); ++i) {
    if (configs[i].width == cfg->width && configs[i].height == cfg->height) {
      resolution_id = static_cast<uint8_t>(i);
      if (configs[i].fps == cfg->fps || cfg->fps == 0) {
        fps_id = 0;
      }
      return true;
    }
  }
  return false;
}

std::vector<uint8_t> OdinCommand::BuildChannelConfigPayload(OdinDataChannel channel,
                                                             OdinTransportMode transport,
                                                             uint16_t dst_port,
                                                             const OdinStreamCfg* cfg) {
  std::vector<uint8_t> payload;
  payload.reserve(cfg ? 20 : 8);

  // data_type (1 byte)
  payload.push_back(static_cast<uint8_t>(channel));

  // TLV: dst_port
  payload.push_back(static_cast<uint8_t>(ChannelConfigTlvType::kDstPort));
  payload.push_back(0x02);
  payload.push_back(static_cast<uint8_t>((dst_port >> 8) & 0xFF));
  payload.push_back(static_cast<uint8_t>(dst_port & 0xFF));

  // TLV: transport_mode
  payload.push_back(static_cast<uint8_t>(ChannelConfigTlvType::kTransportMode));
  payload.push_back(0x01);
  payload.push_back(static_cast<uint8_t>(transport));

  // Mode TLVs (Image channels only)
  if (cfg && (cfg->width > 0 || cfg->height > 0)) {
    uint8_t resolution_id = 0xFF;
    uint8_t fps_id = 0xFF;
    LookupStreamConfig(channel, cfg, resolution_id, fps_id);

    if (resolution_id != 0xFF) {
      payload.push_back(static_cast<uint8_t>(ChannelConfigTlvType::kResolutionId));
      payload.push_back(0x01);
      payload.push_back(resolution_id);
    }

    if (fps_id != 0xFF) {
      payload.push_back(static_cast<uint8_t>(ChannelConfigTlvType::kFpsId));
      payload.push_back(0x01);
      payload.push_back(fps_id);
    }

    if (cfg->format != OdinDataFormat::kUnknown) {
      payload.push_back(static_cast<uint8_t>(ChannelConfigTlvType::kFormat));
      payload.push_back(0x01);
      payload.push_back(static_cast<uint8_t>(cfg->format));
    }
  }

  return payload;
}

OdinResult OdinCommand::SendStartStream(OdinDataChannel channel, OdinTransportMode transport,
                                         uint16_t dst_port, const OdinStreamCfg* cfg,
                                         uint32_t timeout_ms) {
  auto payload = BuildChannelConfigPayload(channel, transport, dst_port, cfg);

  const char* transport_name = (transport == OdinTransportMode::kTcp) ? "TCP" : "UDP";
  LOG_DEBUG("OdinCommand::SendStartStream: channel=0x%02x, port=%u, %s\n",
            static_cast<uint8_t>(channel), dst_port, transport_name);

  std::lock_guard<std::mutex> lock(mutex_);
  if (!command_channel_) return OdinResult::kNotInitialized;

  OdinCommandSyncResponse response;
  if (!command_channel_->SendSync(static_cast<uint16_t>(Odin2CmdId::kChannelConfig), 
                                  payload, response, timeout_ms)) {
    LOG_ERROR("OdinCommand::SendStartStream: timeout\n");
    return OdinResult::kTimeout;
  }

  if (response.response.payload.empty()) {
    LOG_ERROR("OdinCommand::SendStartStream: empty response\n");
    return OdinResult::kUnknownError;
  }

  uint8_t ret_code = response.response.payload[0];
  if (ret_code != 0) {
    LOG_ERROR("OdinCommand::SendStartStream: error code %d\n", ret_code);
    return OdinResult::kUnknownError;
  }

  return OdinResult::kOk;
}

OdinResult OdinCommand::SendCloseStream(OdinDataChannel channel, uint32_t timeout_ms) {
  std::vector<uint8_t> payload;
  payload.reserve(5);

  payload.push_back(static_cast<uint8_t>(channel));
  payload.push_back(static_cast<uint8_t>(ChannelConfigTlvType::kDstPort));
  payload.push_back(0x02);
  payload.push_back(0x00);
  payload.push_back(0x00);

  LOG_DEBUG("OdinCommand::SendCloseStream: channel=0x%02x\n", static_cast<uint8_t>(channel));

  std::lock_guard<std::mutex> lock(mutex_);
  if (!command_channel_) return OdinResult::kNotInitialized;

  OdinCommandSyncResponse response;
  if (!command_channel_->SendSync(static_cast<uint16_t>(Odin2CmdId::kChannelConfig), 
                                  payload, response, timeout_ms)) {
    LOG_ERROR("OdinCommand::SendCloseStream: timeout\n");
    return OdinResult::kTimeout;
  }

  if (response.response.payload.empty()) {
    LOG_ERROR("OdinCommand::SendCloseStream: empty response\n");
    return OdinResult::kUnknownError;
  }

  uint8_t ret_code = response.response.payload[0];
  if (ret_code != 0) {
    LOG_ERROR("OdinCommand::SendCloseStream: error code %d\n", ret_code);
    return OdinResult::kUnknownError;
  }

  return OdinResult::kOk;
}

bool OdinCommand::IsHeartbeatSupported() const {
  return heartbeat_supported_;
}

bool OdinCommand::SendHeartbeat(uint32_t interval_ms, uint32_t timeout_ms) {
  std::vector<uint8_t> payload(2);
  uint16_t interval = static_cast<uint16_t>(interval_ms);
  payload[0] = static_cast<uint8_t>(interval & 0xFF);
  payload[1] = static_cast<uint8_t>((interval >> 8) & 0xFF);

  std::lock_guard<std::mutex> lock(mutex_);
  if (!heartbeat_channel_) {
    return false;
  }

  OdinCommandSyncResponse response;
  if (!heartbeat_channel_->SendSync(static_cast<uint16_t>(Odin2CmdId::kHeartbeat), 
                                    payload, response, timeout_ms)) {
    return false;
  }

  if (response.result != OdinResult::kOk) {
    return false;
  }

  const auto& ack_payload = response.response.payload;
  if (ack_payload.size() >= 5) {
    uint8_t ret_code = ack_payload[0];
    if (ret_code != 0) {
      return false;
    }
  }

  return true;
}

}  // namespace sdk
}  // namespace odin
