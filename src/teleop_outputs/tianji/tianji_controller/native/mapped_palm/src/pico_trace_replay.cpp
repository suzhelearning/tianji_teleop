#include "tianji_mapped_palm/pico_trace_replay.hpp"

#include "tianji_mapped_palm/pico_mapped_corrected_palm.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace tianji_mapped_palm {
namespace {

constexpr std::size_t kTraceHeaderSize = 16U;
constexpr std::size_t kRecordTimeSize = 8U;

std::uint16_t readLe16(const std::vector<std::uint8_t>& bytes,
                       std::size_t offset) {
  if (offset > bytes.size() || bytes.size() - offset < 2U) {
    throw std::runtime_error("truncated TJVR uint16 field");
  }
  return static_cast<std::uint16_t>(bytes[offset]) |
         (static_cast<std::uint16_t>(bytes[offset + 1U]) << 8U);
}

std::uint64_t readLe64(const std::vector<std::uint8_t>& bytes,
                       std::size_t offset) {
  if (offset > bytes.size() || bytes.size() - offset < 8U) {
    throw std::runtime_error("truncated TJVR uint64 field");
  }
  std::uint64_t value = 0U;
  for (std::size_t index = 0U; index < 8U; ++index) {
    value |= static_cast<std::uint64_t>(bytes[offset + index]) <<
             (8U * index);
  }
  return value;
}

bool supportedPacketSize(std::size_t size) noexcept {
  return size == kPicoTeleopPacketV1Size ||
         size == kPicoTeleopPacketV2Size ||
         size == kPicoTeleopPacketV3Size ||
         size == kPicoTeleopPacketV4Size;
}

[[noreturn]] void fail(const std::string& message) {
  throw std::runtime_error("invalid TJVR trace: " + message);
}

}  // namespace

PicoTraceReplay PicoTraceReplay::load(const std::string& path,
                                      PicoTraceReplayOptions options) {
  if (path.empty()) {
    throw std::invalid_argument("TJVR replay path must not be empty");
  }
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("failed to open TJVR replay: " + path);
  }
  const std::vector<std::uint8_t> bytes{
      std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
  if (!input.eof() && input.fail()) {
    throw std::runtime_error("failed to read TJVR replay: " + path);
  }
  if (bytes.size() < kTraceHeaderSize) {
    fail("truncated header");
  }
  if (bytes[0] != 'T' || bytes[1] != 'J' || bytes[2] != 'V' ||
      bytes[3] != 'T') {
    fail("wrong magic");
  }
  if (readLe16(bytes, 4U) != 1U) {
    fail("unsupported container version");
  }
  const std::size_t packet_size = readLe16(bytes, 6U);
  if (!supportedPacketSize(packet_size)) {
    fail("unsupported packet size");
  }
  const std::uint64_t record_count_u64 = readLe64(bytes, 8U);
  if (record_count_u64 == 0U ||
      record_count_u64 > std::numeric_limits<std::size_t>::max()) {
    fail("invalid record count");
  }
  const std::size_t record_count =
      static_cast<std::size_t>(record_count_u64);
  const std::size_t record_size = kRecordTimeSize + packet_size;
  if (record_count >
      (std::numeric_limits<std::size_t>::max() - kTraceHeaderSize) /
          record_size) {
    fail("record count overflows container size");
  }
  const std::size_t expected_size =
      kTraceHeaderSize + record_count * record_size;
  if (bytes.size() != expected_size) {
    fail(bytes.size() < expected_size ? "truncated record" :
                                        "unexpected trailing bytes");
  }

  std::vector<Record> records;
  records.reserve(record_count);
  PicoReceiverStats stats;
  stats.datagrams = record_count;
  PicoTeleopStreamGate gate(options.max_position_jump_m,
                            options.max_orientation_jump_rad);
  std::int64_t first_source_ns = 0;
  std::int64_t previous_source_ns = 0;
  std::uint64_t previous_sequence = 0U;
  std::uint64_t previous_receive_relative = 0U;
  for (std::size_t index = 0U; index < record_count; ++index) {
    const std::size_t record_offset =
        kTraceHeaderSize + index * record_size;
    const std::uint64_t receive_relative = readLe64(bytes, record_offset);
    if (index > 0U && receive_relative < previous_receive_relative) {
      fail("decreasing recorded receive time");
    }
    previous_receive_relative = receive_relative;

    const std::uint8_t* packet = bytes.data() + record_offset + kRecordTimeSize;
    PicoPacketDecodeResult decoded =
        decodePicoTeleopPacket(packet, packet_size);
    if (!decoded.frame.has_value()) {
      fail("packet decode failure at record " + std::to_string(index));
    }
    PicoTeleopFrame frame = std::move(*decoded.frame);
    const std::uint64_t original_sequence = frame.sequence;
    const std::int64_t original_source_ns = frame.source_timestamp_ns;
    if (frame.sequence == 0U || frame.tracking_epoch == 0U ||
        frame.source_timestamp_ns <= 0) {
      fail("zero packet metadata at record " + std::to_string(index));
    }
    if (index > 0U && frame.sequence <= previous_sequence) {
      fail("non-increasing packet sequence at record " +
           std::to_string(index));
    }
    if (index > 0U && frame.source_timestamp_ns < previous_source_ns) {
      fail("decreasing source timestamp at record " +
           std::to_string(index));
    }
    if (index == 0U) {
      first_source_ns = frame.source_timestamp_ns;
    }
    const std::int64_t relative_source_ns =
        frame.source_timestamp_ns - first_source_ns;
    // Match replay_pico_udp_trace.py: a stored trace keeps the original packet
    // for provenance, while replay presents one fresh monotonic stream.
    frame.sequence = index + 1U;
    frame.source_timestamp_ns = kMonotonicEpochNs + relative_source_ns;
    frame.bridge_send_monotonic_ns =
        kMonotonicEpochNs + relative_source_ns;
    frame.receive_monotonic_ns = kMonotonicEpochNs + relative_source_ns;
    if (options.use_mapped_corrected_palm_target) {
      const PicoMappedCorrectedPalmResult mapped =
          selectMappedCorrectedPalm(frame);
      if (!mapped.valid) {
        ++stats.malformed;
        previous_source_ns = original_source_ns;
        previous_sequence = original_sequence;
        continue;
      }
      frame.left = mapped.left;
      frame.right = mapped.right;
    }
    const PicoStreamDecision decision = gate.evaluate(frame);
    if (!decision.accepted) {
      if (decision.reason == PicoStreamRejectReason::kPositionJump ||
          decision.reason == PicoStreamRejectReason::kOrientationJump) {
        ++stats.jump_rejections;
      } else if (decision.reason == PicoStreamRejectReason::kOutOfOrder ||
                 decision.reason == PicoStreamRejectReason::kEpochRollback) {
        ++stats.reordered;
      } else {
        ++stats.malformed;
      }
      previous_source_ns = original_source_ns;
      previous_sequence = original_sequence;
      continue;
    }
    if (decision.epoch_changed) {
      ++stats.epoch_resets;
    }
    frame.stream_discontinuity = decision.stream_discontinuity;
    if (decision.stream_discontinuity) {
      ++stats.resynchronizations;
    }
    frame.resynchronization_generation = stats.resynchronizations;
    records.push_back(Record{relative_source_ns, std::move(frame)});
    ++stats.accepted;
    previous_source_ns = original_source_ns;
    previous_sequence = original_sequence;
  }
  if (records.empty()) {
    fail("no frames accepted by stream gate");
  }
  const std::int64_t input_duration_ns = previous_source_ns - first_source_ns;
  stats.tracking_epoch = records.back().frame.tracking_epoch;
  stats.sequence = records.back().frame.sequence;
  stats.latest_receive_monotonic_ns =
      records.back().frame.receive_monotonic_ns;
  if (input_duration_ns > 0 && stats.accepted > 1U) {
    stats.input_frequency_hz =
        static_cast<double>(stats.accepted - 1U) * 1.0e9 /
        static_cast<double>(input_duration_ns);
  }
  return PicoTraceReplay(std::move(records), record_count, input_duration_ns,
                         stats);
}

PicoTraceReplay::PicoTraceReplay(std::vector<Record> records,
                                 std::size_t input_frame_count,
                                 std::int64_t input_duration_ns,
                                 PicoReceiverStats stats)
    : records_(std::move(records)),
      input_frame_count_(input_frame_count),
      input_duration_ns_(input_duration_ns),
      stats_(stats) {}

std::size_t PicoTraceReplay::frameCount() const noexcept {
  return input_frame_count_;
}

std::int64_t PicoTraceReplay::durationNanoseconds() const noexcept {
  return input_duration_ns_;
}

std::optional<PicoTeleopFrame> PicoTraceReplay::advanceTo(
    std::int64_t relative_time_ns) {
  if (relative_time_ns < 0) {
    throw std::invalid_argument("TJVR replay time must not be negative");
  }
  std::optional<PicoTeleopFrame> latest;
  std::size_t due_count = 0U;
  while (next_record_ < records_.size() &&
         records_[next_record_].relative_source_ns <= relative_time_ns) {
    latest = records_[next_record_].frame;
    ++next_record_;
    ++due_count;
  }
  if (due_count > 1U) {
    stats_.superseded += due_count - 1U;
  }
  return latest;
}

bool PicoTraceReplay::finished() const noexcept {
  return next_record_ == records_.size();
}

PicoReceiverStats PicoTraceReplay::stats() const noexcept { return stats_; }

}  // namespace tianji_mapped_palm
