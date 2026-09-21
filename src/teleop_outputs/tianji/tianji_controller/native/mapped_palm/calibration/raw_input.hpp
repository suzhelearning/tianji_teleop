#pragma once
#include <cstdint>
#include <vector>
#include <array>
namespace tianji_control {
struct RawProgress {
  bool accepted=false;
  // The decoder saw a complete, structurally valid input frame.  This is
  // intentionally separate from stream-gate acceptance: the Python reference
  // records decoded frames even when the continuity/jump gate rejects them.
  bool decoded=false;
  std::uint64_t ingress_sequence=0;
  std::uint64_t epoch=0,sequence=0,generation=0;
  bool discontinuity=false,skeleton_valid=false;
  bool rotations_valid=false;
  std::array<std::array<double,3>,2> palms{};
};
class RawInputEndpoint {
 public:
  virtual ~RawInputEndpoint()=default;
  virtual RawProgress ingest(const std::vector<std::uint8_t>&)=0;
};
}
