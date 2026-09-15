#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

namespace tianji_qp_ik {

enum class PicoTraceRecorderState {
  kDisabled,
  kWaitingForPacket,
  kRecording,
  kFinalized,
  kNoValidPackets,
  kFailed,
};

struct PicoTraceRecorderStats {
  PicoTraceRecorderState state{PicoTraceRecorderState::kWaitingForPacket};
  std::uint64_t records{0U};
  std::size_t packet_size{0U};
};

std::string_view picoTraceRecorderStateName(
    PicoTraceRecorderState state) noexcept;

class PicoTraceRecorder {
 public:
  explicit PicoTraceRecorder(std::string path);
  ~PicoTraceRecorder();

  PicoTraceRecorder(const PicoTraceRecorder&) = delete;
  PicoTraceRecorder& operator=(const PicoTraceRecorder&) = delete;

  bool append(const std::uint8_t* packet, std::size_t packet_size,
              std::int64_t receive_monotonic_ns) noexcept;
  void finish() noexcept;
  PicoTraceRecorderStats stats() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace tianji_qp_ik
