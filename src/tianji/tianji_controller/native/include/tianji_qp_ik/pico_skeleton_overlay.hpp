#pragma once

#include "tianji_qp_ik/pico_teleop_protocol.hpp"
#include "tianji_qp_ik/spark_upper_retarget.hpp"

#include <mujoco/mujoco.h>

namespace tianji_qp_ik {

enum class SkeletonOverlayStyle { kPico, kSpark };

PicoUpperLimbSkeleton sparkUpperLimbSkeleton(
    const SparkUpperTargets& targets) noexcept;

void appendPicoUpperLimbSkeleton(const PicoUpperLimbSkeleton& skeleton,
                                 mjvScene* scene,
                                 SkeletonOverlayStyle style =
                                     SkeletonOverlayStyle::kPico);

}  // namespace tianji_qp_ik
