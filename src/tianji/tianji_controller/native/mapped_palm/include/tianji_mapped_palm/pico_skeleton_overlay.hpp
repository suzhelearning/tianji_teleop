#pragma once

#include "tianji_mapped_palm/pico_teleop_protocol.hpp"
#include "tianji_mapped_palm/spark_upper_retarget.hpp"

#include <mujoco/mujoco.h>

namespace tianji_mapped_palm {

enum class SkeletonOverlayStyle { kPico, kSpark };

PicoUpperLimbSkeleton sparkUpperLimbSkeleton(
    const SparkUpperTargets& targets) noexcept;

void appendPicoUpperLimbSkeleton(const PicoUpperLimbSkeleton& skeleton,
                                 mjvScene* scene,
                                 SkeletonOverlayStyle style =
                                     SkeletonOverlayStyle::kPico);

}  // namespace tianji_mapped_palm
