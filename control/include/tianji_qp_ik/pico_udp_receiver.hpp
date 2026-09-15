#pragma once

#include "tianji_qp_ik/pico_teleop_protocol.hpp"
#include "tianji_qp_ik/pico_trace_recorder.hpp"
#include "tianji_qp_ik/telemetry.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace tianji_qp_ik {

struct PicoUdpReceiverOptions {
  std::string bind_address{"127.0.0.1"};
  std::uint16_t port{15000U};
  double max_position_jump_m{0.15};
  double max_orientation_jump_rad{0.60};
  std::string record_path;
};

struct PicoReceiverStats {
  std::uint64_t datagrams{0};
  std::uint64_t accepted{0};
  std::uint64_t malformed{0};
  std::uint64_t crc_failures{0};
  std::uint64_t reordered{0};
  std::uint64_t jump_rejections{0};
  std::uint64_t superseded{0};
  std::uint64_t epoch_resets{0};
  std::uint64_t resynchronizations{0};
  std::uint64_t tracking_epoch{0};
  std::uint64_t sequence{0};
  std::int64_t latest_receive_monotonic_ns{0};
  double input_frequency_hz{0.0};
  PicoTraceRecorderState recording_state{PicoTraceRecorderState::kDisabled};
  std::uint64_t recorded_packets{0U};
  std::size_t recording_packet_size{0U};
};

class PicoUdpReceiver {
 public:
  PicoUdpReceiver(PicoUdpReceiverOptions options,
                  LatestSpscExchange<PicoTeleopFrame>& exchange);
  ~PicoUdpReceiver();

  PicoUdpReceiver(const PicoUdpReceiver&) = delete;
  PicoUdpReceiver& operator=(const PicoUdpReceiver&) = delete;

  void start();
  void stop() noexcept;
  std::uint16_t boundPort() const noexcept;
  PicoReceiverStats stats() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace tianji_qp_ik
