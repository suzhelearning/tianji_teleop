#include "tianji_qp_ik/pico_trace_recorder.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <system_error>
#include <utility>

namespace tianji_qp_ik {
namespace {

constexpr std::size_t kTraceHeaderSize = 16U;
constexpr off_t kTraceCountOffset = 8;

bool supportedPacketSize(std::size_t size) noexcept {
  return size == 160U || size == 208U || size == 400U || size == 656U;
}

void writeLe16(std::uint8_t* destination, std::uint16_t value) noexcept {
  destination[0] = static_cast<std::uint8_t>(value & 0xffU);
  destination[1] = static_cast<std::uint8_t>((value >> 8U) & 0xffU);
}

void writeLe64(std::uint8_t* destination, std::uint64_t value) noexcept {
  for (std::size_t index = 0U; index < 8U; ++index) {
    destination[index] =
        static_cast<std::uint8_t>((value >> (8U * index)) & 0xffU);
  }
}

bool writeAll(int file_descriptor, const std::uint8_t* bytes,
              std::size_t size) noexcept {
  std::size_t written = 0U;
  while (written < size) {
    const ssize_t result =
        write(file_descriptor, bytes + written, size - written);
    if (result < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    if (result == 0) {
      return false;
    }
    written += static_cast<std::size_t>(result);
  }
  return true;
}

}  // namespace

struct PicoTraceRecorder::Impl {
  explicit Impl(std::string path_in) : path(std::move(path_in)) {
    if (path.empty()) {
      throw std::invalid_argument("PICO trace path must not be empty");
    }
    file_descriptor =
        open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0666);
    if (file_descriptor < 0) {
      throw std::system_error(errno, std::generic_category(),
                              "failed to create PICO trace");
    }
  }

  ~Impl() { finish(); }

  bool append(const std::uint8_t* packet, std::size_t size,
              std::int64_t receive_ns) noexcept {
    const PicoTraceRecorderState current_state =
        state.load(std::memory_order_acquire);
    if (current_state == PicoTraceRecorderState::kFailed ||
        current_state == PicoTraceRecorderState::kFinalized ||
        current_state == PicoTraceRecorderState::kNoValidPackets) {
      return false;
    }
    if (packet == nullptr || receive_ns <= 0 || !supportedPacketSize(size)) {
      fail();
      return false;
    }
    if (current_state == PicoTraceRecorderState::kWaitingForPacket) {
      packet_size.store(size, std::memory_order_relaxed);
      first_receive_ns = receive_ns;
      std::array<std::uint8_t, kTraceHeaderSize> header{};
      header[0] = 'T';
      header[1] = 'J';
      header[2] = 'V';
      header[3] = 'T';
      writeLe16(header.data() + 4U, 1U);
      writeLe16(header.data() + 6U, static_cast<std::uint16_t>(size));
      writeLe64(header.data() + 8U, 0U);
      if (!writeAll(file_descriptor, header.data(), header.size())) {
        fail();
        return false;
      }
      state.store(PicoTraceRecorderState::kRecording,
                  std::memory_order_release);
    }
    if (size != packet_size.load(std::memory_order_relaxed) ||
        receive_ns < first_receive_ns) {
      fail();
      return false;
    }

    std::array<std::uint8_t, 8U> relative_time{};
    writeLe64(relative_time.data(),
              static_cast<std::uint64_t>(receive_ns - first_receive_ns));
    if (!writeAll(file_descriptor, relative_time.data(), relative_time.size()) ||
        !writeAll(file_descriptor, packet, size)) {
      fail();
      return false;
    }
    records.fetch_add(1U, std::memory_order_release);
    return true;
  }

  void finish() noexcept {
    const PicoTraceRecorderState current_state =
        state.load(std::memory_order_acquire);
    if (current_state == PicoTraceRecorderState::kFinalized ||
        current_state == PicoTraceRecorderState::kNoValidPackets ||
        current_state == PicoTraceRecorderState::kFailed) {
      return;
    }
    if (current_state == PicoTraceRecorderState::kWaitingForPacket) {
      closeFile();
      unlink(path.c_str());
      state.store(PicoTraceRecorderState::kNoValidPackets,
                  std::memory_order_release);
      return;
    }

    if (lseek(file_descriptor, kTraceCountOffset, SEEK_SET) < 0) {
      fail();
      return;
    }
    std::array<std::uint8_t, 8U> count{};
    writeLe64(count.data(), records.load(std::memory_order_acquire));
    if (!writeAll(file_descriptor, count.data(), count.size()) ||
        fsync(file_descriptor) != 0) {
      fail();
      return;
    }
    closeFile();
    state.store(PicoTraceRecorderState::kFinalized,
                std::memory_order_release);
  }

  void fail() noexcept {
    closeFile();
    unlink(path.c_str());
    state.store(PicoTraceRecorderState::kFailed, std::memory_order_release);
  }

  void closeFile() noexcept {
    if (file_descriptor >= 0) {
      close(file_descriptor);
      file_descriptor = -1;
    }
  }

  PicoTraceRecorderStats stats() const noexcept {
    return {state.load(std::memory_order_acquire),
            records.load(std::memory_order_acquire),
            packet_size.load(std::memory_order_acquire)};
  }

  std::string path;
  int file_descriptor{-1};
  std::atomic<PicoTraceRecorderState> state{
      PicoTraceRecorderState::kWaitingForPacket};
  std::atomic<std::uint64_t> records{0U};
  std::atomic<std::size_t> packet_size{0U};
  std::int64_t first_receive_ns{0};
};

std::string_view picoTraceRecorderStateName(
    PicoTraceRecorderState state) noexcept {
  switch (state) {
    case PicoTraceRecorderState::kDisabled:
      return "disabled";
    case PicoTraceRecorderState::kWaitingForPacket:
      return "waiting_for_packet";
    case PicoTraceRecorderState::kRecording:
      return "recording";
    case PicoTraceRecorderState::kFinalized:
      return "finalized";
    case PicoTraceRecorderState::kNoValidPackets:
      return "no_valid_packets";
    case PicoTraceRecorderState::kFailed:
      return "failed";
  }
  return "unknown";
}

PicoTraceRecorder::PicoTraceRecorder(std::string path)
    : impl_(std::make_unique<Impl>(std::move(path))) {}

PicoTraceRecorder::~PicoTraceRecorder() = default;

bool PicoTraceRecorder::append(const std::uint8_t* packet,
                               std::size_t packet_size,
                               std::int64_t receive_monotonic_ns) noexcept {
  return impl_->append(packet, packet_size, receive_monotonic_ns);
}

void PicoTraceRecorder::finish() noexcept { impl_->finish(); }

PicoTraceRecorderStats PicoTraceRecorder::stats() const noexcept {
  return impl_->stats();
}

}  // namespace tianji_qp_ik
