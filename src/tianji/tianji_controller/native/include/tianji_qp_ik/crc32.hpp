#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace tianji_qp_ik {
namespace detail {

// Reflected IEEE CRC-32 (the TJVR, TJH2 and TJRC wire polynomial).
inline constexpr std::array<std::uint32_t, 256> kCrc32Table = [] {
  std::array<std::uint32_t, 256> table{};
  for (std::size_t index = 0; index < table.size(); ++index) {
    std::uint32_t crc = static_cast<std::uint32_t>(index);
    for (int bit = 0; bit < 8; ++bit) {
      const std::uint32_t mask = 0U - (crc & 1U);
      crc = (crc >> 1U) ^ (0xedb88320U & mask);
    }
    table[index] = crc;
  }
  return table;
}();

}  // namespace detail

// No allocation or runtime table initialization; packet byte order is unchanged.
inline std::uint32_t crc32(const std::uint8_t* bytes, std::size_t size) noexcept {
  std::uint32_t crc = 0xffffffffU;
  for (std::size_t index = 0; index < size; ++index) {
    crc = (crc >> 8U) ^ detail::kCrc32Table[(crc ^ bytes[index]) & 0xffU];
  }
  return crc ^ 0xffffffffU;
}

}  // namespace tianji_qp_ik
