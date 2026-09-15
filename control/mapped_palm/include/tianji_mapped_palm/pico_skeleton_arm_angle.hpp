#pragma once

#include "tianji_mapped_palm/pico_teleop_protocol.hpp"

namespace tianji_mapped_palm {

// Builds the PICO arm-plane direction consumed by ArmAngleTaskBuilder.  The
// adapter does not unwrap or rate-limit angles; those stateful operations stay
// in the existing ArmDirectionReferenceManager/ArmAngleTaskBuilder.
DualArmDirectionReferences selectMappedSkeletonArmDirections(
    const PicoUpperLimbSkeleton& skeleton) noexcept;

}  // namespace tianji_mapped_palm
