#include "tianji_qp_ik/pico_udp_receiver.hpp"

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <system_error>
#include <thread>
#include <utility>

namespace tianji_qp_ik {
namespace {

std::int64_t monotonicNowNs() noexcept {
  timespec now{};
  if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
    return 0;
  }
  return static_cast<std::int64_t>(now.tv_sec) * 1000000000LL + now.tv_nsec;
}

}  // namespace

struct PicoUdpReceiver::Impl {
  Impl(PicoUdpReceiverOptions options_in,
       LatestSpscExchange<PicoTeleopFrame>& exchange_in)
      : options(std::move(options_in)),
        exchange(exchange_in),
        gate(options.max_position_jump_m,
             options.max_orientation_jump_rad) {}

  ~Impl() { stop(); }

  void start() {
    if (running.load(std::memory_order_acquire)) {
      return;
    }

    if (!options.record_path.empty() && recorder == nullptr) {
      recorder = std::make_unique<PicoTraceRecorder>(options.record_path);
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(options.port);
    if (inet_pton(AF_INET, options.bind_address.c_str(), &address.sin_addr) != 1) {
      throw std::invalid_argument("PICO UDP bind_address must be a valid IPv4 address");
    }

    const int new_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (new_socket < 0) {
      throw std::system_error(errno, std::generic_category(),
                              "failed to create PICO UDP socket");
    }
    const int reuse_address = 1;
    if (setsockopt(new_socket, SOL_SOCKET, SO_REUSEADDR, &reuse_address,
                   sizeof(reuse_address)) != 0) {
      const int error = errno;
      close(new_socket);
      throw std::system_error(error, std::generic_category(),
                              "failed to configure PICO UDP socket");
    }
    if (bind(new_socket, reinterpret_cast<const sockaddr*>(&address),
             sizeof(address)) != 0) {
      const int error = errno;
      close(new_socket);
      throw std::system_error(error, std::generic_category(),
                              "failed to bind PICO UDP socket");
    }

    sockaddr_in bound_address{};
    socklen_t bound_size = sizeof(bound_address);
    if (getsockname(new_socket, reinterpret_cast<sockaddr*>(&bound_address),
                    &bound_size) != 0) {
      const int error = errno;
      close(new_socket);
      throw std::system_error(error, std::generic_category(),
                              "failed to query PICO UDP port");
    }

    socket_fd = new_socket;
    gate.reset();
    resetFrequency();
    bound_port.store(ntohs(bound_address.sin_port), std::memory_order_release);
    running.store(true, std::memory_order_release);
    try {
      receiver_thread = std::thread([this] { receiveLoop(); });
    } catch (...) {
      running.store(false, std::memory_order_release);
      bound_port.store(0U, std::memory_order_release);
      close(socket_fd);
      socket_fd = -1;
      throw;
    }
  }

  void stop() noexcept {
    running.store(false, std::memory_order_release);
    if (receiver_thread.joinable()) {
      receiver_thread.join();
    }
    if (socket_fd >= 0) {
      close(socket_fd);
      socket_fd = -1;
    }
    if (recorder != nullptr) {
      recorder->finish();
    }
    bound_port.store(0U, std::memory_order_release);
  }

  void receiveLoop() noexcept {
    std::array<std::uint8_t, kPicoTeleopMaximumPacketSize + 1U> bytes{};
    pollfd descriptor{};
    descriptor.fd = socket_fd;
    descriptor.events = POLLIN;

    while (running.load(std::memory_order_acquire)) {
      descriptor.revents = 0;
      const int ready = poll(&descriptor, 1, 20);
      if (ready < 0) {
        if (errno == EINTR) {
          continue;
        }
        break;
      }
      if (ready == 0 || (descriptor.revents & POLLIN) == 0) {
        continue;
      }

      const ssize_t received = recvfrom(socket_fd, bytes.data(), bytes.size(), 0,
                                        nullptr, nullptr);
      if (received < 0) {
        if (errno == EINTR) {
          continue;
        }
        break;
      }
      processDatagram(bytes.data(), static_cast<std::size_t>(received));
      datagrams.fetch_add(1U, std::memory_order_release);
    }
  }

  void processDatagram(const std::uint8_t* bytes, std::size_t size) noexcept {
    PicoPacketDecodeResult decoded = decodePicoTeleopPacket(bytes, size);
    if (!decoded.frame.has_value()) {
      if (decoded.error == PicoPacketError::kCrcMismatch) {
        crc_failures.fetch_add(1U, std::memory_order_relaxed);
      } else {
        malformed.fetch_add(1U, std::memory_order_relaxed);
      }
      return;
    }

    decoded.frame->receive_monotonic_ns = monotonicNowNs();
    if (decoded.frame->receive_monotonic_ns <= 0) {
      malformed.fetch_add(1U, std::memory_order_relaxed);
      return;
    }
    if (recorder != nullptr) {
      (void)recorder->append(bytes, size,
                             decoded.frame->receive_monotonic_ns);
    }
    const PicoStreamDecision decision = gate.evaluate(*decoded.frame);
    if (!decision.accepted) {
      if (decision.reason == PicoStreamRejectReason::kPositionJump ||
          decision.reason == PicoStreamRejectReason::kOrientationJump) {
        jump_rejections.fetch_add(1U, std::memory_order_relaxed);
      } else if (decision.reason == PicoStreamRejectReason::kOutOfOrder ||
                 decision.reason == PicoStreamRejectReason::kEpochRollback) {
        reordered.fetch_add(1U, std::memory_order_relaxed);
      } else {
        malformed.fetch_add(1U, std::memory_order_relaxed);
      }
      return;
    }

    if (decision.epoch_changed) {
      epoch_resets.fetch_add(1U, std::memory_order_relaxed);
    }
    decoded.frame->stream_discontinuity = decision.stream_discontinuity;
    if (decision.stream_discontinuity) {
      decoded.frame->resynchronization_generation =
          resynchronizations.fetch_add(1U, std::memory_order_relaxed) + 1U;
    } else {
      decoded.frame->resynchronization_generation =
          resynchronizations.load(std::memory_order_relaxed);
    }
    updateFrequency(decoded.frame->source_timestamp_ns,
                    decision.epoch_changed || decision.stream_discontinuity);
    if (exchange.publish(*decoded.frame) == LatestPublishResult::kSuperseded) {
      superseded.fetch_add(1U, std::memory_order_relaxed);
    }
    tracking_epoch.store(decoded.frame->tracking_epoch, std::memory_order_relaxed);
    sequence.store(decoded.frame->sequence, std::memory_order_relaxed);
    latest_receive_monotonic_ns.store(decoded.frame->receive_monotonic_ns,
                                      std::memory_order_relaxed);
    accepted.fetch_add(1U, std::memory_order_release);
  }

  void resetFrequency() noexcept {
    source_gap_count = 0U;
    source_gap_cursor = 0U;
    previous_source_timestamp_ns = 0;
    input_frequency_hz.store(0.0, std::memory_order_relaxed);
  }

  void updateFrequency(std::int64_t source_timestamp_ns,
                       bool epoch_changed) noexcept {
    if (epoch_changed || previous_source_timestamp_ns == 0) {
      resetFrequency();
      previous_source_timestamp_ns = source_timestamp_ns;
      return;
    }
    const std::int64_t gap = source_timestamp_ns - previous_source_timestamp_ns;
    previous_source_timestamp_ns = source_timestamp_ns;
    if (gap <= 0) {
      return;
    }
    source_gaps[source_gap_cursor] = gap;
    source_gap_cursor = (source_gap_cursor + 1U) % source_gaps.size();
    source_gap_count = std::min(source_gap_count + 1U, source_gaps.size());

    std::array<std::int64_t, 32> sorted = source_gaps;
    std::sort(sorted.begin(),
              sorted.begin() + static_cast<std::ptrdiff_t>(source_gap_count));
    double median_ns = 0.0;
    const std::size_t middle = source_gap_count / 2U;
    if (source_gap_count % 2U == 0U) {
      median_ns = 0.5 * (static_cast<double>(sorted[middle - 1U]) +
                         static_cast<double>(sorted[middle]));
    } else {
      median_ns = static_cast<double>(sorted[middle]);
    }
    input_frequency_hz.store(1.0e9 / median_ns, std::memory_order_relaxed);
  }

  PicoReceiverStats stats() const noexcept {
    PicoReceiverStats snapshot;
    snapshot.datagrams = datagrams.load(std::memory_order_acquire);
    snapshot.accepted = accepted.load(std::memory_order_acquire);
    snapshot.malformed = malformed.load(std::memory_order_relaxed);
    snapshot.crc_failures = crc_failures.load(std::memory_order_relaxed);
    snapshot.reordered = reordered.load(std::memory_order_relaxed);
    snapshot.jump_rejections = jump_rejections.load(std::memory_order_relaxed);
    snapshot.superseded = superseded.load(std::memory_order_relaxed);
    snapshot.epoch_resets = epoch_resets.load(std::memory_order_relaxed);
    snapshot.resynchronizations =
        resynchronizations.load(std::memory_order_relaxed);
    snapshot.tracking_epoch = tracking_epoch.load(std::memory_order_relaxed);
    snapshot.sequence = sequence.load(std::memory_order_relaxed);
    snapshot.latest_receive_monotonic_ns =
        latest_receive_monotonic_ns.load(std::memory_order_relaxed);
    snapshot.input_frequency_hz =
        input_frequency_hz.load(std::memory_order_relaxed);
    if (recorder != nullptr) {
      const PicoTraceRecorderStats recording = recorder->stats();
      snapshot.recording_state = recording.state;
      snapshot.recorded_packets = recording.records;
      snapshot.recording_packet_size = recording.packet_size;
    }
    return snapshot;
  }

  PicoUdpReceiverOptions options;
  LatestSpscExchange<PicoTeleopFrame>& exchange;
  PicoTeleopStreamGate gate;
  std::unique_ptr<PicoTraceRecorder> recorder;
  int socket_fd{-1};
  std::thread receiver_thread;
  std::atomic<bool> running{false};
  std::atomic<std::uint16_t> bound_port{0U};
  std::atomic<std::uint64_t> datagrams{0U};
  std::atomic<std::uint64_t> accepted{0U};
  std::atomic<std::uint64_t> malformed{0U};
  std::atomic<std::uint64_t> crc_failures{0U};
  std::atomic<std::uint64_t> reordered{0U};
  std::atomic<std::uint64_t> jump_rejections{0U};
  std::atomic<std::uint64_t> superseded{0U};
  std::atomic<std::uint64_t> epoch_resets{0U};
  std::atomic<std::uint64_t> resynchronizations{0U};
  std::atomic<std::uint64_t> tracking_epoch{0U};
  std::atomic<std::uint64_t> sequence{0U};
  std::atomic<std::int64_t> latest_receive_monotonic_ns{0};
  std::atomic<double> input_frequency_hz{0.0};
  std::array<std::int64_t, 32> source_gaps{};
  std::size_t source_gap_count{0U};
  std::size_t source_gap_cursor{0U};
  std::int64_t previous_source_timestamp_ns{0};
};

PicoUdpReceiver::PicoUdpReceiver(
    PicoUdpReceiverOptions options,
    LatestSpscExchange<PicoTeleopFrame>& exchange)
    : impl_(std::make_unique<Impl>(std::move(options), exchange)) {}

PicoUdpReceiver::~PicoUdpReceiver() = default;

void PicoUdpReceiver::start() { impl_->start(); }

void PicoUdpReceiver::stop() noexcept { impl_->stop(); }

std::uint16_t PicoUdpReceiver::boundPort() const noexcept {
  return impl_->bound_port.load(std::memory_order_acquire);
}

PicoReceiverStats PicoUdpReceiver::stats() const noexcept {
  return impl_->stats();
}

}  // namespace tianji_qp_ik
