#pragma once

#include <string>

namespace pico_odin {

inline bool is_expected_skeleton_frame(
  const std::string & actual, const std::string & expected)
{
  return !expected.empty() && actual == expected;
}

}  // namespace pico_odin
