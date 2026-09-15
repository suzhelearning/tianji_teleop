#pragma once

#include <cstdint>
#include <filesystem>

namespace pico_bridge
{

// Reserves the next non-zero epoch in a durable state file.  The update is
// written to a sibling temporary file and renamed so a process restart cannot
// silently reuse the previous boot's epoch.
std::uint64_t reserve_tracking_epoch(const std::filesystem::path & state_file);

}  // namespace pico_bridge
