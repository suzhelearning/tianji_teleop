// Optional native Hand2 fixed-rate scheduler process.
//
// stdin: TJHS session updates, TJHI canonical frames, TJHR reset or TJHX shutdown.
// stdout: TJHK startup handshake (optional), followed by TJHO fixed-size
// processed/command frames. The process has no device, Zenoh, robot or MuJoCo
// dependency; the existing drivers and their Python adapters remain the
// source of the validated canonical input frames.

#include "pipeline.hpp"
#include "scheduler.hpp"
#include "scheduler_wire.hpp"

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <mutex>
#include <optional>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>

namespace {
using namespace tianji_hand;
using namespace tianji_hand::scheduler_wire;

struct Options {
  std::string manifest;
  std::int64_t period_ns = 5'000'000;
  std::int64_t freshness_ns = 200'000'000;
  std::int64_t filter_continuity_ns = 0;
  std::size_t output_capacity = 256;
  bool startup_handshake = false;
};

std::int64_t parse_i64(const char* value, const char* field) {
  if (!value || !*value) throw std::invalid_argument(std::string(field) + " is empty");
  char* end = nullptr;
  const long long parsed = std::strtoll(value, &end, 10);
  if (!end || *end != '\0' || parsed <= 0)
    throw std::invalid_argument(std::string(field) + " must be a positive int64");
  return static_cast<std::int64_t>(parsed);
}

std::size_t parse_size(const char* value, const char* field) {
  const auto parsed = parse_i64(value, field);
  return static_cast<std::size_t>(parsed);
}

Options parse_options(int argc, char** argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--manifest" && index + 1 < argc) options.manifest = argv[++index];
    else if (argument == "--period-ns" && index + 1 < argc)
      options.period_ns = parse_i64(argv[++index], "--period-ns");
    else if (argument == "--filter-continuity-ns" && index + 1 < argc)
      options.filter_continuity_ns = parse_i64(argv[++index], "--filter-continuity-ns");
    else if (argument == "--freshness-ns" && index + 1 < argc)
      options.freshness_ns = parse_i64(argv[++index], "--freshness-ns");
    else if (argument == "--output-capacity" && index + 1 < argc)
      options.output_capacity = parse_size(argv[++index], "--output-capacity");
    else if (argument == "--startup-handshake") options.startup_handshake = true;
    else throw std::invalid_argument("unknown native hand scheduler argument: " + argument);
  }
  if (options.manifest.empty()) throw std::invalid_argument("--manifest is required");
  return options;
}

bool wait_readable(int fd, const std::atomic<bool>& cancelled) {
  pollfd descriptor{fd, POLLIN, 0};
  while (!cancelled.load(std::memory_order_acquire)) {
    const int result = ::poll(&descriptor, 1, 100);
    if (result > 0) {
      if (descriptor.revents & POLLNVAL)
        throw std::runtime_error("native hand scheduler input descriptor is invalid");
      if (descriptor.revents & POLLERR)
        throw std::runtime_error("native hand scheduler input pipe failed");
      if (descriptor.revents & (POLLIN | POLLHUP)) return true;
      continue;
    }
    if (result == 0) continue;
    if (errno == EINTR) continue;
    throw std::runtime_error("native hand scheduler input poll failed");
  }
  return false;
}

bool read_exact(int fd, std::uint8_t* data, std::size_t size, const std::atomic<bool>& cancelled,
                bool allow_eof) {
  std::size_t offset = 0;
  while (offset < size) {
    if (!wait_readable(fd, cancelled)) return false;
    const auto count = ::read(fd, data + offset, size - offset);
    if (count == 0) {
      if (offset == 0 && allow_eof) return false;
      throw std::runtime_error("truncated native hand scheduler frame");
    }
    if (count < 0) {
      if (errno == EINTR || errno == EAGAIN) continue;
      throw std::runtime_error("native hand scheduler input read failed");
    }
    offset += static_cast<std::size_t>(count);
  }
  return true;
}

bool read_header(std::array<std::uint8_t, kHeaderSize>& header,
                 const std::atomic<bool>& cancelled) {
  return read_exact(STDIN_FILENO, header.data(), header.size(), cancelled, true);
}

void read_body(std::array<std::uint8_t, kMaximumFrameSize>& frame, std::size_t size,
               const std::atomic<bool>& cancelled) {
  if (size < kHeaderSize || size > frame.size())
    throw std::runtime_error("native hand scheduler frame size out of bounds");
  if (!read_exact(STDIN_FILENO, frame.data() + kHeaderSize, size - kHeaderSize, cancelled, false))
    throw std::runtime_error("native hand scheduler input cancelled");
}

void write_all(const std::uint8_t* data, std::size_t size, std::mutex& mutex,
               const std::atomic<bool>& cancelled) {
  std::lock_guard<std::mutex> lock(mutex);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  std::size_t offset = 0;
  while (offset < size) {
    if (cancelled.load(std::memory_order_acquire))
      throw std::runtime_error("native hand scheduler output cancelled");
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now());
    if (remaining.count() <= 0)
      throw std::runtime_error("native hand scheduler output timeout");
    pollfd descriptor{STDOUT_FILENO, POLLOUT, 0};
    const int result = ::poll(&descriptor, 1, static_cast<int>(remaining.count()));
    if (result == 0) throw std::runtime_error("native hand scheduler output timeout");
    if (result < 0) {
      if (errno == EINTR) continue;
      throw std::runtime_error("native hand scheduler output poll failed");
    }
    if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))
      throw std::runtime_error("native hand scheduler output pipe failed");
    const auto count = ::write(STDOUT_FILENO, data + offset, size - offset);
    if (count < 0) {
      if (errno == EINTR || errno == EAGAIN) continue;
      throw std::runtime_error("native hand scheduler output write failed");
    }
    if (count == 0) throw std::runtime_error("native hand scheduler output pipe closed");
    offset += static_cast<std::size_t>(count);
  }
}

int run(const Options& options) {
  Manifest manifest = load_manifest(options.manifest);
  std::remove(options.manifest.c_str());
  SidePipeline left(manifest.left);
  SidePipeline right(manifest.right);
  HandScheduler scheduler(
      HandSchedulerConfig{options.period_ns, options.freshness_ns, options.output_capacity,
                          options.filter_continuity_ns},
      [&left](const double* points, std::uint64_t sequence, double* output) {
        left.solve(points, sequence, output);
      },
      [&right](const double* points, std::uint64_t sequence, double* output) {
        right.solve(points, sequence, output);
      },
      [&left] { left.reset(); }, [&right] { right.reset(); });

  std::mutex output_mutex;
  std::atomic<bool> writer_failed{false};
  std::optional<std::string> writer_error;
  if (options.startup_handshake) {
    const auto handshake = encode_handshake();
    write_all(handshake.data(), handshake.size(), output_mutex, writer_failed);
  }
  scheduler.start();
  std::thread writer([&] {
    try {
      while (true) {
        HandCommand command;
        if (!scheduler.wait_pop(command, std::chrono::milliseconds(100))) {
          if (scheduler.failed()) {
            writer_error = scheduler.failure();
            writer_failed.store(true, std::memory_order_release);
            break;
          }
          if (scheduler.stopped()) break;
          continue;
        }
        const auto frame = encode_output(command);
        write_all(frame.data(), frame.size(), output_mutex, writer_failed);
      }
    } catch (const std::exception& error) {
      writer_failed.store(true);
      writer_error = error.what();
      scheduler.request_stop();
    }
  });

  try {
    std::array<std::uint8_t, kHeaderSize> header{};
    std::array<std::uint8_t, kMaximumFrameSize> frame{};
    while (read_header(header, writer_failed)) {
      const auto size = scheduler_wire::load_le<std::uint16_t>(header.data() + 6);
      if (size < kHeaderSize || size > kMaximumFrameSize)
        throw std::runtime_error("native hand scheduler declared frame size invalid");
      std::memcpy(frame.data(), header.data(), kHeaderSize);
      read_body(frame, size, writer_failed);
      if (magic_is(frame.data(), kInputMagic)) {
        if (size != kInputSize) throw std::runtime_error("native hand scheduler input size mismatch");
        if (!scheduler.submit_input(decode_input(frame.data(), size)))
          throw std::runtime_error(scheduler.failure().empty() ? "native hand input rejected" : scheduler.failure());
      } else if (magic_is(frame.data(), kResetMagic)) {
        const auto request=decode_reset_request(frame.data(),size);
        const auto current=scheduler.epoch();
        const auto now=std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
        if(scheduler.phase()!=HandPhase::idle || current==std::numeric_limits<std::int64_t>::max() ||
           request.epoch!=current+1 || request.timestamp_ns>static_cast<std::uint64_t>(now) ||
           !scheduler.submit_session(request)) throw std::runtime_error("hand reset requires idle next epoch");
        const auto ack=scheduler.wait_reset_ack(request,std::chrono::milliseconds(2000));
        if(!ack) throw std::runtime_error("hand reset failed, cancelled or timed out");
        const auto reply=encode_reset_ack(*ack);
        write_all(reply.data(),reply.size(),output_mutex,writer_failed);
      } else if (magic_is(frame.data(), kSessionMagic)) {
        if (size != kSessionSize) throw std::runtime_error("native hand scheduler session size mismatch");
        if (!scheduler.submit_session(decode_session(frame.data(), size)))
          throw std::runtime_error(scheduler.failure().empty() ? "native hand session rejected" : scheduler.failure());
      } else if (magic_is(frame.data(), kShutdownMagic)) {
        if (size != kShutdownSize || frame[4] != scheduler_wire::kVersion || frame[5] != 0 ||
            scheduler_wire::load_le<std::uint16_t>(frame.data() + 6) != kShutdownSize)
          throw std::runtime_error("invalid native hand scheduler shutdown");
        break;
      } else {
        throw std::runtime_error("unknown native hand scheduler frame magic");
      }
    }
  } catch (...) {
    scheduler.stop();
    if (writer.joinable()) writer.join();
    throw;
  }
  scheduler.stop();
  if (writer.joinable()) writer.join();
  if (writer_failed.load())
    throw std::runtime_error(writer_error.value_or("native hand scheduler output failed"));
  if (scheduler.failed()) throw std::runtime_error(scheduler.failure());
  return 0;
}
}  // namespace

int main(int argc, char** argv) {
  try {
    // A closed Python reader must become a normal writer error that can wake
    // the input loop, rather than terminating the process asynchronously via
    // SIGPIPE while leaving the scheduler lifecycle ambiguous.
    std::signal(SIGPIPE, SIG_IGN);
    return run(parse_options(argc, argv));
  } catch (const std::exception& error) {
    std::cerr << "native hand scheduler failed: " << error.what() << '\n';
    return 1;
  }
}
