#pragma once

#include "tianji_qp_ik/telemetry.hpp"
#include "tianji_qp_ik/wuji_hand_teleop_protocol.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace tianji_qp_ik {

struct WujiHandUdpReceiverOptions {
  std::string bind_address{"127.0.0.1"};
  std::uint16_t port{16000U};
  double stale_timeout_seconds{0.100};
};

struct WujiHandReceiverStats {
  std::uint64_t datagrams{0U};
  std::uint64_t accepted{0U};
  std::uint64_t malformed{0U};
  std::uint64_t crc_failures{0U};
  std::uint64_t reordered{0U};
  std::uint64_t superseded{0U};
  std::uint64_t sequence{0U};
  std::int64_t latest_receive_monotonic_ns{0};
  bool stale{true};
};

class WujiHandUdpReceiver {
 public:
  WujiHandUdpReceiver(WujiHandUdpReceiverOptions options,
                      LatestSpscExchange<WujiHandTeleopFrame>& exchange);
  ~WujiHandUdpReceiver();

  WujiHandUdpReceiver(const WujiHandUdpReceiver&) = delete;
  WujiHandUdpReceiver& operator=(const WujiHandUdpReceiver&) = delete;

  void start();
  void stop() noexcept;
  std::uint16_t boundPort() const noexcept;
  WujiHandReceiverStats stats() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace tianji_qp_ik
