#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "odin_lidar_api.h"

namespace odin {
namespace sdk {

// =============================================================================
// IDiscovery - Transport-agnostic interface for device discovery
// =============================================================================
// This interface is designed to be independent of the underlying transport
// mechanism (network, USB, serial, etc.). Transport-specific parameters are
// passed via a generic key-value context map.

class IDiscovery {
 public:
  using Context = std::map<std::string, std::string>;

  virtual ~IDiscovery() = default;

  // Get protocol name for logging/debugging
  virtual const char* GetName() const = 0;

  // Build a discovery request packet
  // @param seq: Sequence number for the request
  // @param context: Transport-specific context (e.g., "host_ip", "bus_id")
  // @param request_out: Output buffer for the built packet
  // @return true on success
  virtual bool BuildRequest(uint16_t seq, const Context& context,
                            std::vector<uint8_t>& request_out) = 0;

  // Parse a discovery response packet
  // @param data: Raw packet data
  // @param length: Length of the packet
  // @param device_out: Output device info if parsing succeeds
  // @return true if this is a valid response for this protocol
  virtual bool ParseResponse(const uint8_t* data, size_t length, DiscoveredDevice& device_out) = 0;
};

// =============================================================================
// DiscoveryFactory - Factory for creating discovery instances
// =============================================================================

class DiscoveryFactory {
 public:
  // Get the default discovery (currently OdinUdpDiscovery)
  static std::shared_ptr<IDiscovery> GetDefault();

  // Get discovery by name (e.g., "ODIN_UDP", "ODIN_USB")
  static std::shared_ptr<IDiscovery> GetByName(const std::string& name);
};

}  // namespace sdk
}  // namespace odin
