#include "OdinUdpDiscovery.h"

#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>

#include "../../protocol/OdinProtocol/OdinProtocol.hpp"

namespace odin {
namespace sdk {

namespace {
constexpr const char* kContextKeyHostIp = "host_ip";
}  // namespace

static constexpr uint16_t kCmdIdDeviceQuery = 0x01;
const char* OdinUdpDiscovery::GetName() const { return "ODIN_UDP"; }

bool OdinUdpDiscovery::IpStringToBytes(const std::string& ip, uint8_t* bytes) {
  unsigned int b0, b1, b2, b3;
  if (std::sscanf(ip.c_str(), "%u.%u.%u.%u", &b0, &b1, &b2, &b3) != 4) return false;
  if (b0 > 255 || b1 > 255 || b2 > 255 || b3 > 255) return false;
  bytes[0] = static_cast<uint8_t>(b0);
  bytes[1] = static_cast<uint8_t>(b1);
  bytes[2] = static_cast<uint8_t>(b2);
  bytes[3] = static_cast<uint8_t>(b3);
  return true;
}

bool OdinUdpDiscovery::BuildRequest(uint16_t seq, const Context& context,
                                    std::vector<uint8_t>& request_out) {
  OdinProtocolV1 protocol;

  // Extract host_ip from context (network-specific, optional)
  uint8_t host_bytes[4] = {0, 0, 0, 0};
  auto it = context.find(kContextKeyHostIp);
  if (it != context.end() && !it->second.empty() && it->second != "0.0.0.0") {
    if (!IpStringToBytes(it->second, host_bytes)) return false;
  }

  return protocol.Pack(kCmdIdDeviceQuery, seq, OdinConst::kCmdTypeReq, OdinConst::kSendTypeHost,
                       host_bytes, 4, request_out);
}

bool OdinUdpDiscovery::ParseResponse(const uint8_t* data, size_t length,
                                     DiscoveredDevice& device_out) {
  if (data == nullptr || length == 0) return false;

  OdinProtocolV1 protocol;
  std::unique_ptr<IPacket> packet;

  int consumed = protocol.Parse(data, length, packet);
  if (consumed <= 0 || !packet) return false;

  auto* odin_pkt = static_cast<OdinPacket*>(packet.get());

  // Check if this is a discovery response
  // Minimum payload: status(1) + SN(16) + IP(4) = 21 bytes
  // Extended payload: + model(32) = 53 bytes
  if (odin_pkt->GetCmdId() != kCmdIdDeviceQuery) return false;
  if (odin_pkt->GetCmdType() != OdinConst::kCmdTypeAck) return false;
  if (odin_pkt->GetPayloadSize() < 21) return false;

  const uint8_t* payload = odin_pkt->GetPayload();

  // Check status
  if (payload[0] != 0) return false;

  // Parse SN (bytes 1-16)
  std::string sn(reinterpret_cast<const char*>(payload + 1), 16);
  auto null_pos = sn.find('\0');
  if (null_pos != std::string::npos) sn.resize(null_pos);

  // Extract actual SN: if format is "xx-xxxxx", use part after '-'
  auto dash_pos = sn.find('-');
  if (dash_pos != std::string::npos && dash_pos + 1 < sn.size()) {
    sn = sn.substr(dash_pos + 1);
  }

  // Parse IP (bytes 17-20)
  std::string ip = std::to_string(payload[17]) + "." + std::to_string(payload[18]) + "." +
                   std::to_string(payload[19]) + "." + std::to_string(payload[20]);

  // Parse model (bytes 21-52, 32 characters) if available, otherwise use default
  std::string model = "ODIN2";
  if (odin_pkt->GetPayloadSize() >= 53) {
    model = std::string(reinterpret_cast<const char*>(payload + 21), 32);
    auto model_null_pos = model.find('\0');
    if (model_null_pos != std::string::npos) model.resize(model_null_pos);
  }

  device_out.network.ip_address = ip;
  device_out.sn = sn;
  device_out.model = model;
  device_out.firmware_version = "1.0.0";
  device_out.connection = MTConnectionType::kEthernet;

  return true;
}

// =============================================================================
// DiscoveryFactory
// =============================================================================

namespace {
std::shared_ptr<IDiscovery> g_default_discovery;
std::once_flag g_init_flag;

void InitDefaultDiscovery() { g_default_discovery = std::make_shared<OdinUdpDiscovery>(); }
}  // namespace

std::shared_ptr<IDiscovery> DiscoveryFactory::GetDefault() {
  std::call_once(g_init_flag, InitDefaultDiscovery);
  return g_default_discovery;
}

std::shared_ptr<IDiscovery> DiscoveryFactory::GetByName(const std::string& name) {
  if (name == "ODIN_UDP" || name.empty()) {
    return GetDefault();
  }
  // Future: Add more discoveries here
  // if (name == "ODIN_USB") return std::make_shared<OdinUsbDiscovery>();
  return nullptr;
}

}  // namespace sdk
}  // namespace odin
