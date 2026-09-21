#pragma once

// Fixed-size little-endian protocol for the optional native hand scheduler.
// There are no C++ struct layouts on the wire. Drivers remain outside this
// process; only their validated canonical 21-point output crosses TJHI.

#include "scheduler.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace tianji_hand {
namespace scheduler_wire {

constexpr std::size_t kHeaderSize = 8;
constexpr std::size_t kInputSize = 8 + 8 + 8 + 8 + 4 + 4 + 126 * 8;
constexpr std::size_t kSessionSize = 40;
constexpr std::size_t kShutdownSize = 8;
constexpr std::size_t kOutputSize = 8 + 8 + 8 + 8 + 8 + 8 + 1 + 1 + 2 + 4 + 40 * 8;
constexpr std::size_t kHandshakeSize = 16;
constexpr std::size_t kMaximumFrameSize = kInputSize;
constexpr std::uint8_t kVersion = 1;
constexpr char kInputMagic[] = "TJHI";
constexpr char kSessionMagic[] = "TJHS";
constexpr char kShutdownMagic[] = "TJHX";
constexpr char kOutputMagic[] = "TJHO";
constexpr char kHandshakeMagic[] = "TJHK";
constexpr char kResetMagic[] = "TJHR";
constexpr char kResetAckMagic[] = "TJHA";
constexpr std::size_t kResetAckSize=40;

template <class T>
void store_le(std::uint8_t* data, T value) {
  static_assert(std::is_integral_v<T>);
  using U = std::make_unsigned_t<T>;
  const U raw = static_cast<U>(value);
  for (std::size_t i = 0; i < sizeof(T); ++i)
    data[i] = static_cast<std::uint8_t>(raw >> (8 * i));
}

template <class T>
T load_le(const std::uint8_t* data) {
  static_assert(std::is_integral_v<T>);
  using U = std::make_unsigned_t<T>;
  U raw = 0;
  for (std::size_t i = 0; i < sizeof(T); ++i)
    raw |= static_cast<U>(data[i]) << (8 * i);
  return static_cast<T>(raw);
}

inline void store_double(std::uint8_t* data, double value) {
  std::uint64_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  store_le(data, bits);
}

inline double load_double(const std::uint8_t* data) {
  const auto bits = load_le<std::uint64_t>(data);
  double value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

inline bool magic_is(const std::uint8_t* data, const char (&magic)[5]) {
  return std::memcmp(data, magic, 4) == 0;
}

inline void validate_header(const std::uint8_t* data, const char (&magic)[5],
                            std::uint16_t expected_size) {
  if (!magic_is(data, magic) || data[4] != kVersion ||
      load_le<std::uint16_t>(data + 6) != expected_size)
    throw std::runtime_error("invalid native hand scheduler frame header");
}

inline bool positive_i64(std::uint64_t value) {
  return value > 0 && value <= static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max());
}

inline std::array<std::uint8_t, kInputSize> encode_input(const HandInput& input) {
  if (input.flags == 0 || (input.flags & ~std::uint8_t{3}) != 0 || input.sequence == 0 ||
      input.sequence > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()) ||
      input.timestamp_ns == 0 ||
      input.timestamp_ns > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()) ||
      input.generation > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))
    throw std::invalid_argument("invalid native hand scheduler input metadata");
  for (const double value : input.points)
    if (!std::isfinite(value)) throw std::invalid_argument("invalid native hand scheduler input points");
  std::array<std::uint8_t, kInputSize> frame{};
  std::memcpy(frame.data(), kInputMagic, 4);
  frame[4] = kVersion;
  frame[5] = input.flags;
  store_le(frame.data() + 6, static_cast<std::uint16_t>(kInputSize));
  store_le(frame.data() + 8, input.sequence);
  store_le(frame.data() + 16, input.timestamp_ns);
  store_le(frame.data() + 24, input.generation);
  store_le(frame.data() + 32, static_cast<std::uint32_t>(126));
  store_le(frame.data() + 36, static_cast<std::uint32_t>(0));
  for (std::size_t i = 0; i < input.points.size(); ++i)
    store_double(frame.data() + 40 + i * 8, input.points[i]);
  return frame;
}

inline HandInput decode_input(const std::uint8_t* frame, std::size_t size) {
  if (size != kInputSize) throw std::runtime_error("native hand scheduler input size mismatch");
  validate_header(frame, kInputMagic, static_cast<std::uint16_t>(kInputSize));
  const auto flags = frame[5];
  const auto point_count = load_le<std::uint32_t>(frame + 32);
  const auto reserved = load_le<std::uint32_t>(frame + 36);
  HandInput input;
  input.flags = flags;
  input.sequence = load_le<std::uint64_t>(frame + 8);
  input.timestamp_ns = load_le<std::uint64_t>(frame + 16);
  input.generation = load_le<std::uint64_t>(frame + 24);
  if (point_count != 126 || reserved != 0 || flags == 0 || (flags & ~std::uint8_t{3}) != 0 ||
      !positive_i64(input.sequence) || !positive_i64(input.timestamp_ns) ||
      input.generation > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))
    throw std::runtime_error("native hand scheduler input shape mismatch");
  for (std::size_t i = 0; i < input.points.size(); ++i)
    input.points[i] = load_double(frame + 40 + i * 8);
  for (const double value : input.points)
    if (!std::isfinite(value)) throw std::runtime_error("native hand scheduler input is nonfinite");
  return input;
}

inline std::array<std::uint8_t, kSessionSize> encode_session(const HandSession& session) {
  if (session.epoch <= 0 || session.sequence == 0 || session.timestamp_ns == 0 ||
      session.epoch > std::numeric_limits<std::int64_t>::max() ||
      session.sequence > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()) ||
      session.timestamp_ns > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()) ||
      static_cast<std::uint8_t>(session.phase) > static_cast<std::uint8_t>(HandPhase::fault))
    throw std::invalid_argument("invalid native hand scheduler session metadata");
  std::array<std::uint8_t, kSessionSize> frame{};
  std::memcpy(frame.data(), kSessionMagic, 4);
  frame[4] = kVersion;
  frame[5] = static_cast<std::uint8_t>(session.phase);
  store_le(frame.data() + 6, static_cast<std::uint16_t>(kSessionSize));
  store_le(frame.data() + 8, static_cast<std::uint64_t>(session.epoch));
  store_le(frame.data() + 16, session.sequence);
  store_le(frame.data() + 24, session.timestamp_ns);
  return frame;
}

inline HandSession decode_session(const std::uint8_t* frame, std::size_t size) {
  if (size != kSessionSize) throw std::runtime_error("native hand scheduler session size mismatch");
  validate_header(frame, kSessionMagic, static_cast<std::uint16_t>(kSessionSize));
  if (frame[5] > static_cast<std::uint8_t>(HandPhase::fault) || frame[32] != 0 || frame[33] != 0 ||
      load_le<std::uint16_t>(frame + 34) != 0 || load_le<std::uint32_t>(frame + 36) != 0)
    throw std::runtime_error("native hand scheduler session flags are nonzero");
  const auto epoch = load_le<std::uint64_t>(frame + 8);
  const auto sequence = load_le<std::uint64_t>(frame + 16);
  const auto timestamp = load_le<std::uint64_t>(frame + 24);
  if (!positive_i64(epoch) || sequence > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()) ||
      !positive_i64(timestamp))
    throw std::runtime_error("native hand scheduler session metadata is invalid");
  return HandSession{static_cast<std::int64_t>(epoch), sequence, timestamp,
                     static_cast<HandPhase>(frame[5])};
}

// Explicit reset extension. Legacy TJHS still emits no additional frames.
inline std::array<std::uint8_t,kSessionSize> encode_reset_request(const HandSession& request) {
  if(request.phase!=HandPhase::idle || request.epoch<=1 || request.sequence==0)
    throw std::invalid_argument("hand reset requires idle next epoch");
  auto frame=encode_session(request);std::memcpy(frame.data(),kResetMagic,4);return frame;
}
inline HandSession decode_reset_request(const std::uint8_t* bytes,std::size_t size) {
  if(size!=kSessionSize) throw std::runtime_error("hand reset request size mismatch");
  validate_header(bytes,kResetMagic,kSessionSize);
  std::array<std::uint8_t,kSessionSize> frame{};
  std::memcpy(frame.data(),bytes,size);std::memcpy(frame.data(),kSessionMagic,4);
  const auto value=decode_session(frame.data(),frame.size());
  if(value.phase!=HandPhase::idle || value.epoch<=1 || value.sequence==0)
    throw std::runtime_error("invalid hand reset request");
  return value;
}
inline std::array<std::uint8_t,kResetAckSize> encode_reset_ack(const HandResetAck& ack) {
  if(ack.epoch<=1 || !positive_i64(ack.sequence) || !positive_i64(ack.requested_ns) ||
     !positive_i64(ack.completed_ns) || ack.completed_ns<ack.requested_ns)
    throw std::invalid_argument("invalid hand reset ACK");
  std::array<std::uint8_t,kResetAckSize> frame{};
  std::memcpy(frame.data(),kResetAckMagic,4);frame[4]=kVersion;
  store_le(frame.data()+6,static_cast<std::uint16_t>(frame.size()));
  store_le(frame.data()+8,ack.epoch);store_le(frame.data()+16,ack.sequence);
  store_le(frame.data()+24,ack.requested_ns);store_le(frame.data()+32,ack.completed_ns);
  return frame;
}
inline HandResetAck decode_reset_ack(const std::uint8_t* bytes,std::size_t size) {
  if(size!=kResetAckSize) throw std::runtime_error("hand reset ACK size mismatch");
  validate_header(bytes,kResetAckMagic,kResetAckSize);
  const auto epoch=load_le<std::uint64_t>(bytes+8);
  HandResetAck ack{static_cast<std::int64_t>(epoch),load_le<std::uint64_t>(bytes+16),
                   load_le<std::uint64_t>(bytes+24),load_le<std::uint64_t>(bytes+32)};
  if(bytes[5]!=0 || !positive_i64(epoch) || epoch<=1 || !positive_i64(ack.sequence) ||
     !positive_i64(ack.requested_ns) || !positive_i64(ack.completed_ns) || ack.completed_ns<ack.requested_ns)
    throw std::runtime_error("invalid hand reset ACK metadata");
  return ack;
}

inline std::array<std::uint8_t, kShutdownSize> encode_shutdown() {
  std::array<std::uint8_t, kShutdownSize> frame{};
  std::memcpy(frame.data(), kShutdownMagic, 4);
  frame[4] = kVersion;
  store_le(frame.data() + 6, static_cast<std::uint16_t>(kShutdownSize));
  return frame;
}

inline std::array<std::uint8_t, kHandshakeSize> encode_handshake() {
  std::array<std::uint8_t, kHandshakeSize> frame{};
  std::memcpy(frame.data(), kHandshakeMagic, 4);
  frame[4] = kVersion;
  store_le(frame.data() + 6, static_cast<std::uint16_t>(kHandshakeSize));
  store_le(frame.data() + 8, static_cast<std::uint64_t>(1));
  return frame;
}

inline HandCommand decode_output(const std::uint8_t* frame, std::size_t size) {
  if (size != kOutputSize) throw std::runtime_error("native hand scheduler output size mismatch");
  validate_header(frame, kOutputMagic, static_cast<std::uint16_t>(kOutputSize));
  HandCommand command;
  command.output_sequence = load_le<std::uint64_t>(frame + 8);
  command.input_sequence = load_le<std::uint64_t>(frame + 16);
  command.input_timestamp_ns = load_le<std::uint64_t>(frame + 24);
  command.epoch = static_cast<std::int64_t>(load_le<std::uint64_t>(frame + 32));
  command.scheduler_timestamp_ns = load_le<std::uint64_t>(frame + 40);
  command.valid_flags = frame[48];
  command.phase = static_cast<HandPhase>(frame[49]);
  command.status = static_cast<HandOutputStatus>(load_le<std::uint16_t>(frame + 50));
  if (frame[5] != command.valid_flags || load_le<std::uint32_t>(frame + 52) != 0 ||
      !positive_i64(command.output_sequence) || !positive_i64(command.input_sequence) ||
      !positive_i64(command.input_timestamp_ns) || command.epoch <= 0 ||
      command.epoch > std::numeric_limits<std::int64_t>::max() ||
      !positive_i64(command.scheduler_timestamp_ns) || command.valid_flags == 0 ||
      (command.valid_flags & ~std::uint8_t{3}) != 0 ||
      static_cast<std::uint8_t>(command.phase) > static_cast<std::uint8_t>(HandPhase::fault) ||
      (command.status != HandOutputStatus::processed && command.status != HandOutputStatus::command &&
       command.status != HandOutputStatus::stale))
    throw std::runtime_error("invalid native hand scheduler output metadata");
  for (std::size_t i = 0; i < command.positions.size(); ++i)
    command.positions[i] = load_double(frame + 56 + i * 8);
  for (const double value : command.positions)
    if (!std::isfinite(value)) throw std::runtime_error("native hand scheduler output is nonfinite");
  return command;
}

inline std::array<std::uint8_t, kOutputSize> encode_output(const HandCommand& command) {
  if (command.output_sequence == 0 || command.input_sequence == 0 || command.input_timestamp_ns == 0 ||
      command.epoch <= 0 || command.epoch > std::numeric_limits<std::int64_t>::max() ||
      command.output_sequence > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()) ||
      command.input_sequence > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()) ||
      command.input_timestamp_ns > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()) ||
      command.scheduler_timestamp_ns > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()) ||
      command.valid_flags == 0 ||
      (command.valid_flags & ~std::uint8_t{3}) != 0 ||
      static_cast<std::uint8_t>(command.phase) > static_cast<std::uint8_t>(HandPhase::fault) ||
      (command.status != HandOutputStatus::processed && command.status != HandOutputStatus::command &&
       command.status != HandOutputStatus::stale))
    throw std::invalid_argument("invalid native hand scheduler output metadata");
  for (const double value : command.positions)
    if (!std::isfinite(value)) throw std::invalid_argument("invalid native hand scheduler output positions");
  std::array<std::uint8_t, kOutputSize> frame{};
  std::memcpy(frame.data(), kOutputMagic, 4);
  frame[4] = kVersion;
  frame[5] = command.valid_flags;
  store_le(frame.data() + 6, static_cast<std::uint16_t>(kOutputSize));
  store_le(frame.data() + 8, command.output_sequence);
  store_le(frame.data() + 16, command.input_sequence);
  store_le(frame.data() + 24, command.input_timestamp_ns);
  store_le(frame.data() + 32, static_cast<std::uint64_t>(command.epoch));
  store_le(frame.data() + 40, command.scheduler_timestamp_ns);
  frame[48] = command.valid_flags;
  frame[49] = static_cast<std::uint8_t>(command.phase);
  store_le(frame.data() + 50, static_cast<std::uint16_t>(command.status));
  for (std::size_t i = 0; i < command.positions.size(); ++i)
    store_double(frame.data() + 56 + i * 8, command.positions[i]);
  return frame;
}

}  // namespace scheduler_wire
}  // namespace tianji_hand
