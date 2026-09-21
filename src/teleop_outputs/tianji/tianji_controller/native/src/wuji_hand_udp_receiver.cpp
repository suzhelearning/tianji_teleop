#include "tianji_qp_ik/wuji_hand_udp_receiver.hpp"

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
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

struct WujiHandUdpReceiver::Impl {
  Impl(WujiHandUdpReceiverOptions options_in,
       LatestSpscExchange<WujiHandTeleopFrame>& exchange_in)
      : options(std::move(options_in)), exchange(exchange_in) {
    if (!std::isfinite(options.stale_timeout_seconds) ||
        options.stale_timeout_seconds <= 0.0) {
      throw std::invalid_argument(
          "Wuji Hand stale_timeout_seconds must be finite and positive");
    }
  }

  ~Impl() { stop(); }

  void start() {
    if (running.load(std::memory_order_acquire)) {
      return;
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(options.port);
    if (inet_pton(AF_INET, options.bind_address.c_str(), &address.sin_addr) !=
        1) {
      throw std::invalid_argument(
          "Wuji Hand UDP bind_address must be a valid IPv4 address");
    }

    const int new_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (new_socket < 0) {
      throw std::system_error(errno, std::generic_category(),
                              "failed to create Wuji Hand UDP socket");
    }
    const int reuse_address = 1;
    if (setsockopt(new_socket, SOL_SOCKET, SO_REUSEADDR, &reuse_address,
                   sizeof(reuse_address)) != 0) {
      const int error = errno;
      close(new_socket);
      throw std::system_error(error, std::generic_category(),
                              "failed to configure Wuji Hand UDP socket");
    }
    if (bind(new_socket, reinterpret_cast<const sockaddr*>(&address),
             sizeof(address)) != 0) {
      const int error = errno;
      close(new_socket);
      throw std::system_error(error, std::generic_category(),
                              "failed to bind Wuji Hand UDP socket");
    }

    sockaddr_in bound_address{};
    socklen_t bound_size = sizeof(bound_address);
    if (getsockname(new_socket, reinterpret_cast<sockaddr*>(&bound_address),
                    &bound_size) != 0) {
      const int error = errno;
      close(new_socket);
      throw std::system_error(error, std::generic_category(),
                              "failed to query Wuji Hand UDP port");
    }

    socket_fd = new_socket;
    initialized.store(false, std::memory_order_release);
    last_sequence.store(0U, std::memory_order_release);
    latest_receive_monotonic_ns.store(0, std::memory_order_release);
    last_publication_timestamp_ns = 0;
    last_left_source_timestamp_ns = 0;
    last_right_source_timestamp_ns = 0;
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
    bound_port.store(0U, std::memory_order_release);
  }

  void receiveLoop() noexcept {
    std::array<std::uint8_t, kWujiHandTeleopPacketSize + 1U> bytes{};
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
      const ssize_t received = recvfrom(socket_fd, bytes.data(), bytes.size(),
                                        0, nullptr, nullptr);
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
    WujiHandPacketDecodeResult decoded =
        decodeWujiHandTeleopPacket(bytes, size);
    if (!decoded.frame.has_value()) {
      if (decoded.error == WujiHandPacketError::kCrcMismatch) {
        crc_failures.fetch_add(1U, std::memory_order_relaxed);
      } else {
        malformed.fetch_add(1U, std::memory_order_relaxed);
      }
      return;
    }

    const std::uint64_t sequence = decoded.frame->sequence;
    if (initialized.load(std::memory_order_acquire) &&
        (sequence <= last_sequence.load(std::memory_order_acquire) ||
         decoded.frame->source_timestamp_ns <= last_publication_timestamp_ns ||
         (decoded.frame->left_valid &&
          decoded.frame->left_source_timestamp_ns < last_left_source_timestamp_ns) ||
         (decoded.frame->right_valid &&
          decoded.frame->right_source_timestamp_ns < last_right_source_timestamp_ns))) {
      reordered.fetch_add(1U, std::memory_order_relaxed);
      return;
    }
    decoded.frame->receive_monotonic_ns = monotonicNowNs();
    if (decoded.frame->receive_monotonic_ns <= 0 ||
        decoded.frame->source_timestamp_ns > decoded.frame->receive_monotonic_ns) {
      malformed.fetch_add(1U, std::memory_order_relaxed);
      return;
    }
    if (exchange.publish(*decoded.frame) == LatestPublishResult::kSuperseded) {
      superseded.fetch_add(1U, std::memory_order_relaxed);
    }
    last_publication_timestamp_ns = decoded.frame->source_timestamp_ns;
    if (decoded.frame->left_valid) {
      last_left_source_timestamp_ns = decoded.frame->left_source_timestamp_ns;
    }
    if (decoded.frame->right_valid) {
      last_right_source_timestamp_ns = decoded.frame->right_source_timestamp_ns;
    }
    last_sequence.store(sequence, std::memory_order_release);
    initialized.store(true, std::memory_order_release);
    latest_receive_monotonic_ns.store(decoded.frame->receive_monotonic_ns,
                                      std::memory_order_release);
    accepted.fetch_add(1U, std::memory_order_release);
  }

  WujiHandReceiverStats stats() const noexcept {
    WujiHandReceiverStats snapshot;
    snapshot.datagrams = datagrams.load(std::memory_order_acquire);
    snapshot.accepted = accepted.load(std::memory_order_acquire);
    snapshot.malformed = malformed.load(std::memory_order_relaxed);
    snapshot.crc_failures = crc_failures.load(std::memory_order_relaxed);
    snapshot.reordered = reordered.load(std::memory_order_relaxed);
    snapshot.superseded = superseded.load(std::memory_order_relaxed);
    snapshot.sequence = last_sequence.load(std::memory_order_acquire);
    snapshot.latest_receive_monotonic_ns =
        latest_receive_monotonic_ns.load(std::memory_order_acquire);
    const std::int64_t now = monotonicNowNs();
    const double timeout_ns_double =
        options.stale_timeout_seconds * 1000000000.0;
    const double max_timeout_ns = static_cast<double>(
        std::numeric_limits<std::int64_t>::max());
    const std::int64_t timeout_ns =
        timeout_ns_double >= max_timeout_ns
            ? std::numeric_limits<std::int64_t>::max()
            : static_cast<std::int64_t>(timeout_ns_double);
    snapshot.stale = snapshot.latest_receive_monotonic_ns <= 0 || now <= 0 ||
                     now - snapshot.latest_receive_monotonic_ns > timeout_ns;
    return snapshot;
  }

  WujiHandUdpReceiverOptions options;
  LatestSpscExchange<WujiHandTeleopFrame>& exchange;
  int socket_fd{-1};
  std::thread receiver_thread;
  std::atomic<bool> running{false};
  std::atomic<bool> initialized{false};
  std::atomic<std::uint16_t> bound_port{0U};
  std::atomic<std::uint64_t> datagrams{0U};
  std::atomic<std::uint64_t> accepted{0U};
  std::atomic<std::uint64_t> malformed{0U};
  std::atomic<std::uint64_t> crc_failures{0U};
  std::atomic<std::uint64_t> reordered{0U};
  std::atomic<std::uint64_t> superseded{0U};
  std::atomic<std::uint64_t> last_sequence{0U};
  std::atomic<std::int64_t> latest_receive_monotonic_ns{0};
  // Receiver-thread-owned watermarks; reset only before starting that thread.
  std::int64_t last_publication_timestamp_ns{0};
  std::int64_t last_left_source_timestamp_ns{0};
  std::int64_t last_right_source_timestamp_ns{0};
};

WujiHandUdpReceiver::WujiHandUdpReceiver(
    WujiHandUdpReceiverOptions options,
    LatestSpscExchange<WujiHandTeleopFrame>& exchange)
    : impl_(std::make_unique<Impl>(std::move(options), exchange)) {}

WujiHandUdpReceiver::~WujiHandUdpReceiver() = default;

void WujiHandUdpReceiver::start() { impl_->start(); }

void WujiHandUdpReceiver::stop() noexcept { impl_->stop(); }

std::uint16_t WujiHandUdpReceiver::boundPort() const noexcept {
  return impl_->bound_port.load(std::memory_order_acquire);
}

WujiHandReceiverStats WujiHandUdpReceiver::stats() const noexcept {
  return impl_->stats();
}

}  // namespace tianji_qp_ik
