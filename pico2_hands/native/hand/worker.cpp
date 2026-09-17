// Legacy-compatible native Hand2 worker entry point.
// The pipeline itself is shared with the opt-in fixed-rate scheduler.

#include "pipeline.hpp"

#include <array>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
using namespace tianji_hand;

std::uint64_t field_u64(const std::array<std::uint8_t, kRequestSize>& message,
                        std::size_t offset) {
  return load_le<std::uint64_t>(message.data() + offset);
}

std::uint32_t field_u32(const std::array<std::uint8_t, kRequestSize>& message,
                        std::size_t offset) {
  return load_le<std::uint32_t>(message.data() + offset);
}

void emit_response(std::uint8_t flags, std::uint64_t sequence, std::uint64_t timestamp,
                   const std::array<double, kJoints * 2>& positions) {
  std::vector<std::uint8_t> response;
  response.reserve(kResponseSize);
  response.insert(response.end(), kResponseMagic, kResponseMagic + 4);
  response.push_back(static_cast<std::uint8_t>(kVersion));
  response.push_back(flags);
  append_le(response, static_cast<std::uint16_t>(kResponseSize));
  append_le(response, sequence);
  append_le(response, timestamp);
  for (double value : positions) append_double(response, value);
  if (response.size() != kResponseSize)
    throw std::runtime_error("native Hand2 response size error");
  std::cout.write(reinterpret_cast<const char*>(response.data()),
                  static_cast<std::streamsize>(response.size()));
  std::cout.flush();
}

int run(const std::string& manifest_path, std::uint16_t selected_side,
        bool startup_handshake) {
  Manifest manifest = load_manifest(manifest_path);
  std::remove(manifest_path.c_str());
  if (selected_side != manifest.selected_side)
    throw std::runtime_error("native Hand2 worker side does not match manifest");
  SidePipeline left(manifest.left);
  SidePipeline right(manifest.right);
  if (startup_handshake) {
    std::cout << R"({"schema_version":1,"kind":"wuji_worker_ready","algorithm":"official_wuji_hand2","callbacks":0})"
              << '\n';
    std::cout.flush();
  }
  std::array<std::uint8_t, kRequestSize> message{};
  std::optional<std::uint64_t> previous_sequence;
  std::uint64_t previous_timestamp = 0;
  while (true) {
    try {
      read_exact(std::cin, message.data(), message.size(), true);
    } catch (const std::ios_base::failure&) {
      return 0;
    }
    if (std::memcmp(message.data(), kRequestMagic, 4) != 0 || message[4] != kVersion)
      throw std::runtime_error("native Hand2 request magic/version mismatch");
    const auto flags = message[5];
    const auto declared_size = load_le<std::uint16_t>(message.data() + 6);
    const auto sequence = field_u64(message, 8);
    const auto timestamp = field_u64(message, 16);
    const auto point_count = field_u32(message, 24);
    const auto reserved = field_u32(message, 28);
    if (flags != selected_side || declared_size != kRequestSize || reserved != 0 ||
        (point_count != kPoints && point_count != kBilateralPoints) || sequence == 0 ||
        sequence > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()) ||
        timestamp == 0 || timestamp > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))
      throw std::runtime_error("invalid native Hand2 request metadata");
    if (previous_sequence && sequence <= *previous_sequence)
      throw std::runtime_error("native Hand2 request sequence rollback");
    if (previous_timestamp && timestamp < previous_timestamp)
      throw std::runtime_error("native Hand2 request timestamp rollback");
    std::array<double, kJoints * 2> positions{};
    const auto* point_bytes = message.data() + 32;
    std::array<double, kPoints> right_points{};
    std::array<double, kPoints> left_points{};
    if (point_count == kPoints) {
      for (std::size_t i = 0; i < kPoints; ++i)
        right_points[i] = load_double(point_bytes + i * 8);
      if (selected_side == kLeft) left.solve(right_points.data(), sequence, positions.data());
      else right.solve(right_points.data(), sequence, positions.data() + kJoints);
    } else {
      for (std::size_t i = 0; i < kPoints; ++i)
        right_points[i] = load_double(point_bytes + i * 8);
      for (std::size_t i = 0; i < kPoints; ++i)
        left_points[i] = load_double(point_bytes + (kPoints + i) * 8);
      right.solve(right_points.data(), sequence, positions.data() + kJoints);
      left.solve(left_points.data(), sequence, positions.data());
    }
    require_finite(positions.data(), positions.size(), "response");
    const std::uint8_t response_flags = point_count == kPoints
        ? static_cast<std::uint8_t>(selected_side == kLeft ? 1 : 2) : 3;
    emit_response(response_flags, sequence, timestamp, positions);
    previous_sequence = sequence;
    previous_timestamp = timestamp;
  }
}
}  // namespace

int main(int argc, char** argv) {
  try {
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);
    std::string manifest;
    std::uint16_t selected_side = 0;
    bool startup_handshake = false;
    for (int i = 1; i < argc; ++i) {
      const std::string argument(argv[i]);
      if (argument == "--manifest" && i + 1 < argc) manifest = argv[++i];
      else if (argument == "--single-hand-side" && i + 1 < argc) {
        const std::string side(argv[++i]);
        selected_side = side == "left" ? kLeft : side == "right" ? kRight : 0;
      } else if (argument == "--startup-handshake") startup_handshake = true;
      else throw std::invalid_argument("unknown native Hand2 worker argument: " + argument);
    }
    if (manifest.empty() || !selected_side)
      throw std::invalid_argument("manifest and single-hand-side are required");
    return run(manifest, selected_side, startup_handshake);
  } catch (const std::exception& error) {
    std::cerr << "native Hand2 worker failed: " << error.what() << '\n';
    return 1;
  }
}
