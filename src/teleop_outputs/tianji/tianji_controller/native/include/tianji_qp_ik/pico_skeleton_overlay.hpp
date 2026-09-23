#pragma once

#include "tianji_qp_ik/pico_teleop_protocol.hpp"
#include "tianji_qp_ik/shared_root_retarget.hpp"

#include <mujoco/mujoco.h>

namespace tianji_qp_ik {

enum class SkeletonOverlayStyle { kPico, kSharedRoot };

PicoUpperLimbSkeleton sharedRootUpperLimbSkeleton(
    const SharedRootTargets& targets) noexcept;

void appendPicoUpperLimbSkeleton(const PicoUpperLimbSkeleton& skeleton,
                                 mjvScene* scene,
                                 SkeletonOverlayStyle style =
                                     SkeletonOverlayStyle::kPico);

}  // namespace tianji_qp_ik
