#pragma once

#include "tianji_mapped_palm/pico_teleop_protocol.hpp"
#include "tianji_mapped_palm/pico_udp_receiver.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace tianji_mapped_palm {

struct PicoTraceReplayOptions {
  double max_position_jump_m{0.15};
  double max_orientation_jump_rad{0.60};
  bool use_mapped_corrected_palm_target{false};
};

class PicoTraceReplay {
 public:
  static constexpr std::int64_t kMonotonicEpochNs = 1'000'000'000LL;

  static PicoTraceReplay load(
      const std::string& path,
      PicoTraceReplayOptions options = PicoTraceReplayOptions{});

  std::size_t frameCount() const noexcept;
  std::int64_t durationNanoseconds() const noexcept;
  std::optional<PicoTeleopFrame> advanceTo(std::int64_t relative_time_ns);
  bool finished() const noexcept;
  PicoReceiverStats stats() const noexcept;

 private:
  struct Record {
    std::int64_t relative_source_ns{0};
    PicoTeleopFrame frame;
  };

  PicoTraceReplay(std::vector<Record> records, std::size_t input_frame_count,
                  std::int64_t input_duration_ns, PicoReceiverStats stats);

  std::vector<Record> records_;
  std::size_t input_frame_count_{0U};
  std::int64_t input_duration_ns_{0};
  std::size_t next_record_{0U};
  PicoReceiverStats stats_;
};

}  // namespace tianji_mapped_palm
