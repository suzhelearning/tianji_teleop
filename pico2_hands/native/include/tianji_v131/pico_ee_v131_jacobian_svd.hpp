// Ported from TJ_arm_control_pico_ee_ik; see PORTING.md.
#pragma once

#include "tianji_v131/types.hpp"

namespace tianji_v131 {

struct PicoEeV131JacobianSvdResult {
  bool valid{false};
  double sigma_min{0.0};
  Vec7 nullspace_basis{Vec7::Zero()};
  double cartesian_leakage{0.0};
};

double picoEeV131MinimumSingularValue(const Mat67& jacobian) noexcept;

PicoEeV131JacobianSvdResult computePicoEeV131JacobianSvd(
    const Mat67& jacobian) noexcept;

}  // namespace tianji_v131
