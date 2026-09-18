#pragma once

// Shared native Hand2 pipeline and manifest reader.
//
// The manifest is produced once by the existing pinned Python configuration
// loader. Everything after that boundary (geometry, optimizer, filter,
// limits and canonical permutation) is owned by this C++ module. Keeping the
// pipeline in a shared header lets the fixed-rate scheduler and the legacy
// binary worker use exactly the same implementation.

#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <istream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

namespace tianji_hand {

constexpr std::uint16_t kVersion = 1;
constexpr std::uint16_t kLeft = 1;
constexpr std::uint16_t kRight = 2;
constexpr std::size_t kFrames = 36;
constexpr std::size_t kJoints = 20;
constexpr std::size_t kPoints = 63;
constexpr std::size_t kBilateralPoints = 126;
constexpr std::size_t kRequestSize = 4 + 1 + 1 + 2 + 8 + 8 + 4 + 4 + kBilateralPoints * 8;
constexpr std::size_t kResponseSize = 4 + 1 + 1 + 2 + 8 + 8 + kJoints * 2 * 8;
constexpr char kManifestMagic[] = "TJWM";
constexpr char kRequestMagic[] = "TJWI";
constexpr char kResponseMagic[] = "TJHR";

extern "C" {
int tianji_hand_geometry_prepare(const double*, std::size_t, const double*, std::size_t,
                                  int, double*) noexcept;
void* tianji_hand_filter_create(double) noexcept;
void tianji_hand_filter_destroy(void*) noexcept;
int tianji_hand_filter_reset(void*) noexcept;
int tianji_hand_filter_next(void*, const double*, std::size_t, double*, int) noexcept;
void* tianji_hand_optimizer_create(const char*, const char* const*, std::size_t,
                                   const double*, std::size_t) noexcept;
void tianji_hand_optimizer_destroy(void*) noexcept;
int tianji_hand_optimizer_state(void*, const double*, double*, int) noexcept;
int tianji_hand_optimizer_solve(void*, const double*, std::size_t, const double*, double*) noexcept;
int tianji_hand_optimizer_bind_current_thread(void*) noexcept;
const char* tianji_hand_optimizer_error() noexcept;
}

template <class T>
T load_le(const std::uint8_t* data) {
  static_assert(std::is_integral_v<T>);
  using U = std::make_unsigned_t<T>;
  U value = 0;
  for (std::size_t i = 0; i < sizeof(T); ++i)
    value |= static_cast<U>(data[i]) << (8 * i);
  return static_cast<T>(value);
}

inline double load_double(const std::uint8_t* data) {
  const auto bits = load_le<std::uint64_t>(data);
  double value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

template <class T>
void append_le(std::vector<std::uint8_t>& output, T value) {
  static_assert(std::is_integral_v<T>);
  using U = std::make_unsigned_t<T>;
  const U unsigned_value = static_cast<U>(value);
  for (std::size_t i = 0; i < sizeof(T); ++i)
    output.push_back(static_cast<std::uint8_t>(unsigned_value >> (8 * i)));
}

inline void append_double(std::vector<std::uint8_t>& output, double value) {
  std::uint64_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  append_le(output, bits);
}

inline void read_exact(std::istream& input, void* destination, std::size_t size,
                       bool allow_eof = false) {
  input.read(static_cast<char*>(destination), static_cast<std::streamsize>(size));
  const auto received = static_cast<std::size_t>(input.gcount());
  if (received == 0 && allow_eof && input.eof()) throw std::ios_base::failure("eof");
  if (received != size) throw std::runtime_error("truncated native Hand2 message");
}

inline std::string read_string(const std::vector<std::uint8_t>& bytes, std::size_t& offset,
                               std::uint32_t length, std::size_t maximum, const char* field) {
  if (length == 0 || length > maximum || offset > bytes.size() - length)
    throw std::runtime_error(std::string("invalid native Hand2 ") + field);
  std::string value(reinterpret_cast<const char*>(bytes.data() + offset), length);
  offset += length;
  if (value.find('\0') != std::string::npos)
    throw std::runtime_error(std::string("embedded NUL in native Hand2 ") + field);
  return value;
}

inline void require_finite(const double* values, std::size_t count, const char* field) {
  for (std::size_t i = 0; i < count; ++i)
    if (!std::isfinite(values[i]))
      throw std::runtime_error(std::string("nonfinite native Hand2 ") + field);
}

struct SideRecord {
  std::uint16_t side = 0;
  std::string urdf;
  std::array<std::string, kFrames> names;
  std::array<double, 18> geometry{};
  std::array<double, 35> optimizer{};
  std::array<double, 40> limits{};
  std::array<std::int32_t, kJoints> permutation{};
  double alpha = 0.0;
};

class SidePipeline {
 public:
  explicit SidePipeline(const SideRecord& record) : record_(record) {
    if (record_.side != kLeft && record_.side != kRight)
      throw std::runtime_error("invalid native Hand2 side");
    require_finite(record_.geometry.data(), record_.geometry.size(), "geometry");
    require_finite(record_.optimizer.data(), record_.optimizer.size(), "optimizer");
    require_finite(record_.limits.data(), record_.limits.size(), "limits");
    if (!std::isfinite(record_.alpha) || !(record_.alpha > 0.0 && record_.alpha <= 1.0))
      throw std::runtime_error("invalid native Hand2 filter alpha");
    for (std::size_t i = 0; i < kJoints; ++i) {
      if (record_.permutation[i] < 0 ||
          record_.permutation[i] >= static_cast<std::int32_t>(kJoints))
        throw std::runtime_error("invalid native Hand2 joint permutation");
      if (!(record_.limits[2 * i] < record_.limits[2 * i + 1]))
        throw std::runtime_error("invalid native Hand2 joint limits");
    }
    std::array<const char*, kFrames> pointers{};
    for (std::size_t i = 0; i < kFrames; ++i) pointers[i] = record_.names[i].c_str();
    optimizer_ = tianji_hand_optimizer_create(record_.urdf.c_str(), pointers.data(), kFrames,
                                               record_.optimizer.data(), record_.optimizer.size());
    if (!optimizer_) throw std::runtime_error(optimizer_error());
    filter_ = tianji_hand_filter_create(record_.alpha);
    if (!filter_) {
      tianji_hand_optimizer_destroy(optimizer_);
      optimizer_ = nullptr;
      throw std::runtime_error("native Hand2 filter allocation failed");
    }
  }

  ~SidePipeline() {
    tianji_hand_filter_destroy(filter_);
    tianji_hand_optimizer_destroy(optimizer_);
  }

  SidePipeline(const SidePipeline&) = delete;
  SidePipeline& operator=(const SidePipeline&) = delete;

  void reset() {
    if (tianji_hand_optimizer_bind_current_thread(optimizer_) != 0)
      throw std::runtime_error(optimizer_error());
    if (tianji_hand_filter_reset(filter_) != 0)
      throw std::runtime_error("native Hand2 filter reset failed");
    if (tianji_hand_optimizer_state(optimizer_, nullptr, nullptr, 1) < 0)
      throw std::runtime_error(optimizer_error());
    previous_sequence_.reset();
  }

  void solve(const double* raw_points, std::uint64_t sequence, double* output) {
    if (!raw_points || !output) throw std::invalid_argument("null native Hand2 solve buffer");
    if (tianji_hand_optimizer_bind_current_thread(optimizer_) != 0)
      throw std::runtime_error(optimizer_error());
    if (previous_sequence_ && sequence != *previous_sequence_ + 1) reset();
    std::array<double, kPoints> prepared{};
    if (tianji_hand_geometry_prepare(raw_points, kPoints, record_.geometry.data(),
                                     record_.geometry.size(), record_.side == kLeft ? 1 : 0,
                                     prepared.data()) != 0)
      throw std::runtime_error("native Hand2 geometry rejected frame");
    std::array<double, kJoints> solved{}, filtered{}, canonical{};
    const int status = tianji_hand_optimizer_solve(optimizer_, prepared.data(), kPoints,
                                                   nullptr, solved.data());
    if (status < 0) throw std::runtime_error(optimizer_error());
    if (tianji_hand_filter_next(filter_, solved.data(), kJoints, filtered.data(), 1) != 0)
      throw std::runtime_error("native Hand2 filter rejected result");
    // The official Python optimizer returns float32 and the Python LPFilter
    // therefore performs float32 arithmetic. Preserve that boundary exactly.
    for (double& value : filtered) value = static_cast<double>(static_cast<float>(value));
    require_finite(filtered.data(), filtered.size(), "joint result");
    for (std::size_t i = 0; i < kJoints; ++i) {
      const double lower = record_.limits[2 * i];
      const double upper = record_.limits[2 * i + 1];
      canonical[i] = std::min(std::max(filtered[i], lower), upper);
    }
    for (std::size_t i = 0; i < kJoints; ++i) output[i] = canonical[record_.permutation[i]];
    require_finite(output, kJoints, "canonical result");
    previous_sequence_ = sequence;
  }

 private:
  static const char* optimizer_error() {
    const char* value = tianji_hand_optimizer_error();
    return value && *value ? value : "native Hand2 optimizer failed";
  }

  SideRecord record_;
  void* optimizer_ = nullptr;
  void* filter_ = nullptr;
  std::optional<std::uint64_t> previous_sequence_;
};

struct Manifest {
  std::uint16_t selected_side = 0;
  SideRecord left;
  SideRecord right;
};

inline Manifest load_manifest(const std::string& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("cannot open native Hand2 manifest");
  input.seekg(0, std::ios::end);
  const auto size = input.tellg();
  if (size < 0 || size > 2 * 1024 * 1024)
    throw std::runtime_error("native Hand2 manifest size invalid");
  input.seekg(0, std::ios::beg);
  std::vector<std::uint8_t> bytes(static_cast<std::size_t>(size));
  if (!bytes.empty()) read_exact(input, bytes.data(), bytes.size());
  if (bytes.size() < 12 || std::memcmp(bytes.data(), kManifestMagic, 4) != 0)
    throw std::runtime_error("native Hand2 manifest magic mismatch");
  if (load_le<std::uint16_t>(bytes.data() + 4) != kVersion)
    throw std::runtime_error("native Hand2 manifest version mismatch");
  Manifest result;
  result.selected_side = load_le<std::uint16_t>(bytes.data() + 6);
  const auto records = load_le<std::uint32_t>(bytes.data() + 8);
  if ((result.selected_side != kLeft && result.selected_side != kRight) || records != 2)
    throw std::runtime_error("native Hand2 manifest selection/count mismatch");
  std::size_t offset = 12;
  std::array<SideRecord*, 2> destinations{&result.left, &result.right};
  for (auto* destination : destinations) {
    constexpr std::size_t kSideHeader = 2 + 2 + 6 * 4 + 8;
    if (offset > bytes.size() || bytes.size() - offset < kSideHeader)
      throw std::runtime_error("truncated native Hand2 side manifest");
    destination->side = load_le<std::uint16_t>(bytes.data() + offset);
    offset += 4;  // side and reserved
    const auto urdf_length = load_le<std::uint32_t>(bytes.data() + offset); offset += 4;
    const auto name_count = load_le<std::uint32_t>(bytes.data() + offset); offset += 4;
    const auto geometry_count = load_le<std::uint32_t>(bytes.data() + offset); offset += 4;
    const auto optimizer_count = load_le<std::uint32_t>(bytes.data() + offset); offset += 4;
    const auto limit_count = load_le<std::uint32_t>(bytes.data() + offset); offset += 4;
    const auto permutation_count = load_le<std::uint32_t>(bytes.data() + offset); offset += 4;
    destination->alpha = load_double(bytes.data() + offset); offset += 8;
    if (name_count != kFrames || geometry_count != 18 || optimizer_count != 35 ||
        limit_count != 40 || permutation_count != 20)
      throw std::runtime_error("native Hand2 manifest count mismatch");
    destination->urdf = read_string(bytes, offset, urdf_length, 1 << 20, "URDF path");
    for (auto& name : destination->names) {
      if (offset > bytes.size() || bytes.size() - offset < 4)
        throw std::runtime_error("truncated native Hand2 model name");
      const auto length = load_le<std::uint32_t>(bytes.data() + offset); offset += 4;
      name = read_string(bytes, offset, length, 4096, "model name");
    }
    for (double& value : destination->geometry) {
      if (offset > bytes.size() || bytes.size() - offset < 8)
        throw std::runtime_error("truncated native Hand2 geometry");
      value = load_double(bytes.data() + offset); offset += 8;
    }
    for (double& value : destination->optimizer) {
      if (offset > bytes.size() || bytes.size() - offset < 8)
        throw std::runtime_error("truncated native Hand2 optimizer");
      value = load_double(bytes.data() + offset); offset += 8;
    }
    for (double& value : destination->limits) {
      if (offset > bytes.size() || bytes.size() - offset < 8)
        throw std::runtime_error("truncated native Hand2 limits");
      value = load_double(bytes.data() + offset); offset += 8;
    }
    for (std::int32_t& value : destination->permutation) {
      if (offset > bytes.size() || bytes.size() - offset < 4)
        throw std::runtime_error("truncated native Hand2 permutation");
      value = load_le<std::int32_t>(bytes.data() + offset); offset += 4;
    }
  }
  if (offset != bytes.size() || result.left.side != kLeft || result.right.side != kRight)
    throw std::runtime_error("native Hand2 manifest trailing data or side order mismatch");
  return result;
}

}  // namespace tianji_hand
