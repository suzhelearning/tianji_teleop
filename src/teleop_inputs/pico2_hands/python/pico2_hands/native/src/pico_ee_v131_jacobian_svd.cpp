// Ported from TJ_arm_control_pico_ee_ik; see PORTING.md.
#include "tianji_v131/pico_ee_v131_jacobian_svd.hpp"

#include <Eigen/SVD>

#include <cmath>
#include <limits>

namespace tianji_v131 {
namespace {

constexpr double kMaximumCartesianLeakage = 1.0e-9;

}  // namespace

double picoEeV131MinimumSingularValue(const Mat67& jacobian) noexcept {
  if (!jacobian.allFinite()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const Eigen::JacobiSVD<Mat67> svd(jacobian);
  if (svd.info() != Eigen::Success || svd.singularValues().size() == 0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return svd.singularValues().minCoeff();
}

PicoEeV131JacobianSvdResult computePicoEeV131JacobianSvd(
    const Mat67& jacobian) noexcept {
  PicoEeV131JacobianSvdResult result;
  if (!jacobian.allFinite()) {
    return result;
  }
  const Eigen::JacobiSVD<Mat67> svd(jacobian, Eigen::ComputeFullV);
  if (svd.info() != Eigen::Success ||
      svd.singularValues().size() == 0 ||
      svd.matrixV().rows() != kArmDof ||
      svd.matrixV().cols() != kArmDof) {
    return result;
  }
  result.sigma_min = svd.singularValues().minCoeff();
  result.nullspace_basis = svd.matrixV().col(kArmDof - 1);
  const double norm = result.nullspace_basis.norm();
  if (!std::isfinite(result.sigma_min) ||
      !result.nullspace_basis.allFinite() ||
      !std::isfinite(norm) || norm <= 1.0e-12) {
    return PicoEeV131JacobianSvdResult{};
  }
  result.nullspace_basis /= norm;
  result.cartesian_leakage =
      (jacobian * result.nullspace_basis).norm();
  result.valid = std::isfinite(result.cartesian_leakage) &&
                 result.cartesian_leakage < kMaximumCartesianLeakage;
  if (!result.valid) {
    return PicoEeV131JacobianSvdResult{};
  }
  return result;
}

}  // namespace tianji_v131
