#pragma once

#include "tianji_qp_ik/pico_trace_recorder.hpp"

#include <cstddef>
#include <cstdint>

namespace tianji_qp_ik {

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
  // UDP validity is established by its decoder. ROS additionally carries
  // explicit source revocations, observable even without a replacement pose.
  bool source_valid{true};
};

class PicoInputReceiver {
 public:
  virtual ~PicoInputReceiver() = default;
  virtual void start() = 0;
  virtual void stop() noexcept = 0;
  virtual PicoReceiverStats stats() const noexcept = 0;
};

}  // namespace tianji_qp_ik
