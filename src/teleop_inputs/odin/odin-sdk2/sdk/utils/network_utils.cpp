#include "network_utils.h"

#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <random>
#include <set>

#ifdef _WIN32
#include <iphlpapi.h>
#pragma comment(lib, "iphlpapi.lib")
#pragma comment(lib, "ws2_32.lib")
#else
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#endif

#include "../protocol/OdinProtocol/OdinProtocol.hpp"

namespace odin {
namespace sdk {
namespace network {

// =============================================================================
// Winsock Management
// =============================================================================

#ifdef _WIN32
static std::mutex g_winsock_mutex;
static int g_winsock_refcnt = 0;

bool AcquireWinsock() {
  std::lock_guard<std::mutex> lock(g_winsock_mutex);
  if (g_winsock_refcnt == 0) {
    WSADATA data;
    if (WSAStartup(MAKEWORD(2, 2), &data) != 0) return false;
  }
  ++g_winsock_refcnt;
  return true;
}

void ReleaseWinsock() {
  std::lock_guard<std::mutex> lock(g_winsock_mutex);
  if (g_winsock_refcnt > 0) {
    --g_winsock_refcnt;
    if (g_winsock_refcnt == 0) WSACleanup();
  }
}
#else
bool AcquireWinsock() { return true; }
void ReleaseWinsock() {}
#endif

// =============================================================================
// Socket Operations
// =============================================================================

void CloseSocket(SocketHandle sock) {
  if (sock == kInvalidSocket) return;
#ifdef _WIN32
  closesocket(sock);
#else
  close(sock);
#endif
}

SocketHandle CreateBroadcastSocket(const std::string& host_ip, uint16_t host_port) {
  SocketHandle sock = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (sock == kInvalidSocket) return kInvalidSocket;

  // Enable broadcast
  int enable = 1;
  if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, reinterpret_cast<const char*>(&enable),
                 sizeof(enable)) != 0) {
    CloseSocket(sock);
    return kInvalidSocket;
  }

  // Bind to local address
  sockaddr_in local_addr{};
  local_addr.sin_family = AF_INET;
  local_addr.sin_port = htons(host_port);
  if (host_ip.empty() || host_ip == "0.0.0.0") {
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  } else if (inet_pton(AF_INET, host_ip.c_str(), &local_addr.sin_addr) != 1) {
    CloseSocket(sock);
    return kInvalidSocket;
  }

  if (bind(sock, reinterpret_cast<sockaddr*>(&local_addr), sizeof(local_addr)) != 0) {
    CloseSocket(sock);
    return kInvalidSocket;
  }

  return sock;
}

// =============================================================================
// IP Address Utilities
// =============================================================================

std::vector<std::string> GetLocalIPAddresses() {
  std::vector<std::string> ips;
#ifdef _WIN32
  ULONG bufLen = 15000;
  PIP_ADAPTER_ADDRESSES addresses = (PIP_ADAPTER_ADDRESSES)malloc(bufLen);
  if (addresses && GetAdaptersAddresses(AF_INET, 0, NULL, addresses, &bufLen) == NO_ERROR) {
    for (auto addr = addresses; addr; addr = addr->Next) {
      if (addr->OperStatus != IfOperStatusUp) continue;
      for (auto ua = addr->FirstUnicastAddress; ua; ua = ua->Next) {
        if (ua->Address.lpSockaddr->sa_family != AF_INET) continue;
        sockaddr_in* sa = (sockaddr_in*)ua->Address.lpSockaddr;
        char ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &sa->sin_addr, ip, sizeof(ip));
        if (strcmp(ip, "127.0.0.1") != 0) {
          ips.push_back(ip);
        }
      }
    }
  }
  if (addresses) free(addresses);
#else
  struct ifaddrs* ifaddr = nullptr;
  if (getifaddrs(&ifaddr) == 0) {
    for (auto ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
      if (ifa->ifa_addr == nullptr) continue;
      if (ifa->ifa_addr->sa_family != AF_INET) continue;
      if (!(ifa->ifa_flags & IFF_UP)) continue;
      if (ifa->ifa_flags & IFF_LOOPBACK) continue;

      sockaddr_in* sa = (sockaddr_in*)ifa->ifa_addr;
      char ip[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, &sa->sin_addr, ip, sizeof(ip));
      ips.push_back(ip);
    }
    freeifaddrs(ifaddr);
  }
#endif
  return ips;
}

bool IpStringToBytes(const std::string& ip, uint8_t* bytes_out) {
  if (ip.empty() || bytes_out == nullptr) return false;

  in_addr addr;
  if (inet_pton(AF_INET, ip.c_str(), &addr) != 1) return false;

  uint32_t host_order = ntohl(addr.s_addr);
  bytes_out[0] = static_cast<uint8_t>((host_order >> 24) & 0xFF);
  bytes_out[1] = static_cast<uint8_t>((host_order >> 16) & 0xFF);
  bytes_out[2] = static_cast<uint8_t>((host_order >> 8) & 0xFF);
  bytes_out[3] = static_cast<uint8_t>(host_order & 0xFF);
  return true;
}

std::string BytesToIpString(const uint8_t* bytes) {
  if (bytes == nullptr) return "";
  return std::to_string(bytes[0]) + "." + std::to_string(bytes[1]) + "." +
         std::to_string(bytes[2]) + "." + std::to_string(bytes[3]);
}

// =============================================================================
// Discovery Protocol Helpers
// =============================================================================

bool BuildDiscoveryPacket(const std::string& host_ip, uint16_t seq,
                          std::vector<uint8_t>& frame_out) {
  OdinProtocolV1 protocol;

  uint8_t host_bytes[4] = {0, 0, 0, 0};
  if (!host_ip.empty() && host_ip != "0.0.0.0") {
    if (!IpStringToBytes(host_ip, host_bytes)) return false;
  }

  return protocol.Pack(kCmdIdDeviceQuery, seq, OdinConst::kCmdTypeReq, OdinConst::kSendTypeHost,
                       host_bytes, 4, frame_out);
}

bool ParseDiscoveryResponse(const uint8_t* data, size_t length, DiscoveredDevice& device_out) {
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
  std::string ip = BytesToIpString(payload + 17);

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

bool SendDiscoveryBroadcast(SocketHandle sock, uint16_t target_port,
                            const IDiscovery::Context& context,
                            std::shared_ptr<IDiscovery> discovery) {
  static std::atomic<uint16_t> seq_counter{0};

  if (!discovery) discovery = DiscoveryFactory::GetDefault();

  std::vector<uint8_t> frame;
  if (!discovery->BuildRequest(seq_counter.fetch_add(1), context, frame)) {
    return false;
  }

  sockaddr_in broadcast_addr{};
  broadcast_addr.sin_family = AF_INET;
  broadcast_addr.sin_port = htons(target_port);
  broadcast_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);

  return sendto(sock, reinterpret_cast<const char*>(frame.data()), static_cast<int>(frame.size()),
                0, reinterpret_cast<sockaddr*>(&broadcast_addr), sizeof(broadcast_addr)) >= 0;
}

// =============================================================================
// Device Discovery
// =============================================================================

namespace {
constexpr const char* kContextKeyHostIp = "host_ip";
constexpr const char* kContextKeyPort = "port";
std::atomic<uint16_t> g_discovery_seq{0};

uint16_t GenerateRandomPort() {
  static std::random_device rd;
  static std::mt19937 gen(rd());
  static std::uniform_int_distribution<uint16_t> dis(10000, 65535);
  return dis(gen);
}
}  // namespace

bool DiscoverDevicesOnInterface(std::vector<DiscoveredDevice>& devices, uint32_t timeout_ms,
                                uint16_t host_port, uint16_t target_port,
                                const std::string& host_ip, std::shared_ptr<IDiscovery> discovery) {
  if (!discovery) discovery = DiscoveryFactory::GetDefault();
  if (host_port == 0) host_port = kDefaultCommandPort;
  if (target_port == 0) target_port = kDefaultCommandPort;
  if (timeout_ms == 0) timeout_ms = 2000;
  if (!AcquireWinsock()) return false;

  SocketHandle sock = CreateBroadcastSocket(host_ip, host_port);
  if (sock == kInvalidSocket) {
    ReleaseWinsock();
    return false;
  }

  // Build context for discovery
  IDiscovery::Context context;
  context[kContextKeyHostIp] = host_ip;
  context[kContextKeyPort] = std::to_string(target_port);

  bool success = true;
  do {
    // Build and send discovery request
    uint16_t seq = g_discovery_seq.fetch_add(1);
    std::vector<uint8_t> frame;
    if (!discovery->BuildRequest(seq, context, frame)) {
      success = false;
      break;
    }

    sockaddr_in broadcast_addr;
    std::memset(&broadcast_addr, 0, sizeof(broadcast_addr));
    broadcast_addr.sin_family = AF_INET;
    broadcast_addr.sin_port = htons(target_port);
    broadcast_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
    if (sendto(sock, reinterpret_cast<const char*>(frame.data()), static_cast<int>(frame.size()), 0,
               reinterpret_cast<sockaddr*>(&broadcast_addr), sizeof(broadcast_addr)) < 0) {
      success = false;
      break;
    }

    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    std::set<std::string> seen;
    std::vector<uint8_t> buffer(512);

    while (std::chrono::steady_clock::now() < deadline) {
      auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
          deadline - std::chrono::steady_clock::now());
      timeval tv;
      tv.tv_sec = static_cast<long>(remaining.count() / 1000);
      tv.tv_usec = static_cast<long>((remaining.count() % 1000) * 1000);

      fd_set read_fds;
      FD_ZERO(&read_fds);
      FD_SET(sock, &read_fds);
#ifdef _WIN32
      int ready = select(0, &read_fds, nullptr, nullptr, &tv);
#else
      int ready = select(static_cast<int>(sock + 1), &read_fds, nullptr, nullptr, &tv);
#endif
      if (ready <= 0 || !FD_ISSET(sock, &read_fds)) continue;

      sockaddr_in from_addr;
      socklen_t addr_len = sizeof(from_addr);
      int len =
          recvfrom(sock, reinterpret_cast<char*>(buffer.data()), static_cast<int>(buffer.size()), 0,
                   reinterpret_cast<sockaddr*>(&from_addr), &addr_len);
      if (len <= 0) continue;

      // Parse response using discovery
      DiscoveredDevice device;
      if (!discovery->ParseResponse(buffer.data(), static_cast<size_t>(len), device)) continue;

        // Use actual socket source IP instead of protocol payload IP
#ifdef _WIN32
      char socket_ip[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, &from_addr.sin_addr, socket_ip, INET_ADDRSTRLEN);
      device.network.ip_address = socket_ip;
#else
      char socket_ip[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, &from_addr.sin_addr, socket_ip, INET_ADDRSTRLEN);
      device.network.ip_address = socket_ip;
#endif

      if (!seen.insert(device.network.ip_address).second) continue;

      device.host_ip = host_ip;
      devices.push_back(device);
    }
  } while (false);

  CloseSocket(sock);
  ReleaseWinsock();
  return success;
}

bool DiscoverNetworkDevices(std::vector<DiscoveredDevice>& devices, uint32_t timeout_ms,
                            std::shared_ptr<IDiscovery> discovery) {
  if (!discovery) discovery = DiscoveryFactory::GetDefault();

  auto local_ips = GetLocalIPAddresses();
  if (local_ips.empty()) {
    local_ips.push_back("0.0.0.0");
  }

  uint32_t per_interface_timeout =
      std::max(500u, timeout_ms / static_cast<uint32_t>(local_ips.size()));

  constexpr int kMaxPortRetries = 3;
  std::set<std::string> seen_sns;
  bool found_any = false;

  for (const auto& ip : local_ips) {
    std::vector<DiscoveredDevice> found;
    bool success = false;
    for (int retry = 0; retry < kMaxPortRetries && !success; ++retry) {
      uint16_t host_port = GenerateRandomPort();
      success = DiscoverDevicesOnInterface(found, per_interface_timeout, host_port,
                                           kDefaultCommandPort, ip, discovery);
    }
    if (success) {
      for (const auto& dev : found) {
        if (seen_sns.find(dev.sn) == seen_sns.end()) {
          seen_sns.insert(dev.sn);
          devices.push_back(dev);
          found_any = true;
        }
      }
    }
  }
  return found_any;
}

}  // namespace network
}  // namespace sdk
}  // namespace odin
