// Input-only collector. No viewer, command publisher or device lifecycle code.
#include "tianji_qp_ik/pico_teleop_protocol.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <system_error>

namespace {
constexpr std::int64_t kSecond = 1000000000;
volatile std::sig_atomic_t interrupted = 0;
void onSignal(int) { interrupted = 1; }
struct Stage { int second; const char* action; const char* prompt; };
constexpr std::array<Stage, 9> stages{{
    {0, "unannotated", "自然站立，双手放松并静止"},
    {5, "natural_reach", "双手自然向前伸，再收回"},
    {11, "hands_approach", "双手逐渐靠近，再分开；手柄不要碰撞"},
    {17, "crossing", "双手在胸前交叉，再展开"},
    {23, "unequal_height", "左手高、右手低，再交换"},
    {29, "single_hand", "左手静止、右手移动；听到交换提示后换手"},
    {35, "bilateral_motion", "双手一起左右、上下移动，尽量保持间距"},
    {41, "extension_boundary", "双臂缓慢接近伸直，再收回，不要锁肘"},
    {47, "unannotated", "回到舒适位置并静止"},
}};
std::int64_t nowNs() {
  timespec ts{};
  if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
    throw std::system_error(errno, std::generic_category(), "clock_gettime");
  return static_cast<std::int64_t>(ts.tv_sec) * kSecond + ts.tv_nsec;
}
void check(bool ok, const char* operation) {
  if (!ok) throw std::system_error(errno, std::generic_category(), operation);
}
struct Fd {
  int value{-1};
  explicit Fd(int fd) : value(fd) { check(fd >= 0, "open/socket"); }
  ~Fd() { if (value >= 0) ::close(value); }
  void close() { const int fd = value; value = -1; check(::close(fd) == 0, "close"); }
};
void writeAll(int fd, const void* data, std::size_t size) {
  const auto* bytes = static_cast<const std::uint8_t*>(data);
  while (size) {
    const auto n = ::write(fd, bytes, size);
    if (n < 0 && errno == EINTR) continue;
    check(n > 0, "write");
    bytes += n;
    size -= static_cast<std::size_t>(n);
  }
}
void write64(int fd, std::uint64_t value) {
  std::array<std::uint8_t, 8> bytes{};
  for (std::size_t i = 0; i < bytes.size(); ++i)
    bytes[i] = static_cast<std::uint8_t>(value >> (8 * i));
  writeAll(fd, bytes.data(), bytes.size());
}
int integer(const char* value, int low, int high) {
  std::size_t end = 0;
  const std::string text(value);
  const int result = std::stoi(text, &end);
  if (end != text.size() || result < low || result > high)
    throw std::runtime_error("integer option outside allowed range");
  return result;
}
void describe() {
  std::cout << "{\"duration_s\":50,\"stages\":[";
  for (std::size_t i = 0; i < stages.size(); ++i) {
    if (i) std::cout << ',';
    std::cout << "{\"scheduled_ns\":" << stages[i].second * kSecond
              << ",\"action\":\"" << stages[i].action
              << "\",\"prompt\":\"" << stages[i].prompt << "\"}";
  }
  std::cout << "],\"motion_authorized\":false}\n";
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc == 2 && std::string(argv[1]) == "--describe") { describe(); return 0; }
    std::string output;
    int port = 15000, countdown = 5, wait_seconds = 30;
    for (int i = 1; i < argc; ++i) {
      const std::string arg(argv[i]);
      if (++i == argc) throw std::runtime_error("missing option value");
      if (arg == "--output") output = argv[i];
      else if (arg == "--port") port = integer(argv[i], 1, 65535);
      else if (arg == "--countdown") countdown = integer(argv[i], 0, 60);
      else if (arg == "--wait-seconds") wait_seconds = integer(argv[i], 1, 3600);
      else throw std::runtime_error("unknown option: " + arg);
    }
    if (output.empty()) throw std::runtime_error("--output existing session directory required");
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    Fd sock(::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = htons(static_cast<std::uint16_t>(port));
    // No SO_REUSEADDR/PORT: another consumer is an error, never share its input.
    check(::bind(sock.value, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0, "bind");
    Fd trace(::open((output + "/input.tjvr.partial").c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0644));
    Fd journal(::open((output + "/events.jsonl").c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0644));
    const std::array<std::uint8_t, 16> header{{'T','J','V','T',1,0,144,2,0,0,0,0,0,0,0,0}};
    writeAll(trace.value, header.data(), header.size());
    auto event = [&](const std::string& json) {
      const auto line = json + "\n";
      writeAll(journal.value, line.data(), line.size());
      std::cout << line << std::flush;
      if (!std::cout) throw std::runtime_error("prompt output failed");
    };
    const auto waiting = nowNs();
    event("{\"type\":\"waiting\",\"port\":" + std::to_string(port) + "}");
    std::int64_t ready = 0, first = 0, last = 0;
    std::uint64_t count = 0;
    std::size_t next_stage = 0;
    int last_countdown = -1;
    bool swapped = false;
    std::array<std::uint8_t, 65536> packet{};
    while (!interrupted) {
      const auto now = nowNs();
      if (!first && now - (ready ? ready + countdown * kSecond : waiting) > wait_seconds * kSecond)
        throw std::runtime_error("timeout waiting for input; partial retained");
      if (ready && !first && now < ready + countdown * kSecond) {
        const int remaining = static_cast<int>((ready + countdown * kSecond - now + kSecond - 1) / kSecond);
        if (remaining != last_countdown) {
          last_countdown = remaining;
          event("{\"type\":\"countdown\",\"remaining_s\":" + std::to_string(remaining) + "}");
        }
      }
      if (first) {
        if (now - first >= 50 * kSecond) break;
        if (next_stage < stages.size() && now - first >= stages[next_stage].second * kSecond) {
          const auto& stage = stages[next_stage++];
          event("{\"type\":\"prompt\",\"scheduled_ns\":" + std::to_string(stage.second * kSecond) +
                ",\"receive_relative_ns\":" + std::to_string(nowNs() - first) +
                ",\"action\":\"" + stage.action + "\",\"prompt\":\"" + stage.prompt + "\"}");
        }
        if (!swapped && now - first >= 32 * kSecond) {
          swapped = true;
          event("{\"type\":\"cue\",\"receive_relative_ns\":" + std::to_string(nowNs() - first) +
                ",\"prompt\":\"交换：右手静止、左手移动\"}");
        }
      }
      pollfd descriptor{sock.value, POLLIN, 0};
      const int polled = ::poll(&descriptor, 1, 10);
      if (polled < 0 && errno == EINTR) continue;
      check(polled >= 0, "poll");
      if (!polled) continue;
      const auto size = ::recv(sock.value, packet.data(), packet.size(), 0);
      const auto received = nowNs();
      if (size < 0 && errno == EINTR) continue;
      check(size >= 0, "recv");
      if (first && received - first >= 50 * kSecond) break;
      const auto decoded = tianji_qp_ik::decodePicoTeleopPacket(packet.data(), static_cast<std::size_t>(size));
      if (size != 656 || !decoded.frame) {
        Fd rejected(::open((output + "/rejected.datagram").c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0644));
        writeAll(rejected.value, packet.data(), static_cast<std::size_t>(size));
        check(::fsync(rejected.value) == 0, "fsync rejected");
        event("{\"type\":\"invalid_packet\",\"monotonic_ns\":" + std::to_string(received) + "}");
        throw std::runtime_error("invalid/non-v4 TJVR packet; raw datagram and partial retained");
      }
      if (!ready) {
        ready = received;
        event("{\"type\":\"ready\",\"monotonic_ns\":" + std::to_string(ready) + "}");
      }
      if (received < ready + countdown * kSecond) continue;
      if (!first) {
        first = received;
        event("{\"type\":\"origin\",\"first_receive_monotonic_ns\":" + std::to_string(first) + "}");
      }
      write64(trace.value, static_cast<std::uint64_t>(received - first));
      writeAll(trace.value, packet.data(), static_cast<std::size_t>(size));
      ++count;
      last = received;
    }
    if (interrupted) throw std::runtime_error("interrupted; partial retained");
    if (!count || next_stage != stages.size()) throw std::runtime_error("incomplete capture/prompt sequence");
    check(::lseek(trace.value, 8, SEEK_SET) == 8, "seek count");
    write64(trace.value, count);
    check(::fsync(trace.value) == 0, "fsync trace");
    trace.close();
    event("{\"type\":\"finished\",\"frames\":" + std::to_string(count) +
          ",\"last_receive_relative_ns\":" + std::to_string(last - first) +
          ",\"stop_relative_ns\":" + std::to_string(nowNs() - first) + "}");
    check(::fsync(journal.value) == 0, "fsync journal");
    journal.close();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
