#pragma once

#include "tianji_mapped_palm/pico_teleop_protocol.hpp"

namespace tianji_mapped_palm {

// EE poses obtained directly from the corrected PICO skeleton.  This adapter
// deliberately does not know about Spark, robot geometry, or any controller
// state; it only applies the fixed PICO-to-EE basis convention.
struct PicoMappedCorrectedPalmResult {
  bool valid{false};
  Pose left;
  Pose right;
};

PicoMappedCorrectedPalmResult selectMappedCorrectedPalm(
    const PicoTeleopFrame& frame) noexcept;

}  // namespace tianji_mapped_palm
