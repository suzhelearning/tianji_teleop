#include "tianji_qp_ik/iterative_pose_dls.hpp"

#include "tianji_qp_ik/so3.hpp"

#include <Eigen/SVD>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

namespace tianji_qp_ik {
namespace {

bool finitePose(const Pose& pose) {
  return pose.position.allFinite() && isProperRotation(pose.rotation);
}

bool inside(const Vec7& q, const Vec7& lower, const Vec7& upper) {
  return q.allFinite() && (q.array() >= lower.array()).all() &&
         (q.array() <= upper.array()).all();
}

double elapsedMicroseconds(std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double, std::micro>(
             std::chrono::steady_clock::now() - start)
      .count();
}

}  // namespace

Vec7 clampPostureReferenceToInterior(const Vec7& requested,
                                     const ArmLimits& limits,
                                     double margin_rad) noexcept {
  if (!requested.allFinite() || !limits.lower_position.allFinite() ||
      !limits.upper_position.allFinite() || !std::isfinite(margin_rad) ||
      margin_rad < 0.0) {
    return requested;
  }
  Vec7 result = requested;
  for (int joint = 0; joint < kArmDof; ++joint) {
    double lower = limits.lower_position[joint] + margin_rad;
    double upper = limits.upper_position[joint] - margin_rad;
    if (lower > upper) {
      lower = 0.5 * (limits.lower_position[joint] +
                     limits.upper_position[joint]);
      upper = lower;
    }
    result[joint] = std::clamp(result[joint], lower, upper);
  }
  return result;
}

IterativePoseDlsIk7::IterativePoseDlsIk7(IterativeDlsConfig config,
                                         double margin_rad)
    : config_(std::move(config)), margin_rad_(margin_rad) {}

PoseDlsResult IterativePoseDlsIk7::solve(const PoseDlsInput& input) const {
  const auto start = std::chrono::steady_clock::now();
  PoseDlsResult result;
  result.q = input.seed;
  const Vec7 lower = input.limits.lower_position.array() + margin_rad_;
  const Vec7 upper = input.limits.upper_position.array() - margin_rad_;
  if (!finitePose(input.target) || !input.seed.allFinite() ||
      !input.limits.lower_position.allFinite() ||
      !input.limits.upper_position.allFinite() || !input.evaluate ||
      (lower.array() > upper.array()).any() ||
      !inside(input.seed, input.limits.lower_position,
              input.limits.upper_position)) {
    result.detail = "invalid_pose_dls_input";
    result.solve_time_us = elapsedMicroseconds(start);
    return result;
  }

  Vec7 current = input.seed.cwiseMax(lower).cwiseMin(upper);
  result.joint_projection_count =
      static_cast<int>((current.array() != input.seed.array()).count());
  result.q = current;
  ArmKinematicSample sample = input.evaluate(current);
  if (!finitePose(sample.tcp_pose) || !sample.tcp_jacobian.allFinite()) {
    result.detail = "invalid_pose_dls_kinematics";
    result.solve_time_us = elapsedMicroseconds(start);
    return result;
  }
  if (input.candidate_feasible && !input.candidate_feasible(sample)) {
    result.detail = "infeasible_pose_dls_seed";
    result.solve_time_us = elapsedMicroseconds(start);
    return result;
  }
  Vec6 error = poseErrorWorld(input.target, sample.tcp_pose);
  result.initial_position_error_m = error.head<3>().norm();
  result.initial_orientation_error_rad = error.tail<3>().norm();
  const auto converged = [this](const Vec6& value) {
    return value.head<3>().norm() <= config_.position_tolerance_m &&
           value.tail<3>().norm() <= config_.orientation_tolerance_rad;
  };
  const auto merit = [this](const Vec6& value) {
    Vec6 weighted = value;
    weighted.head<3>() *= config_.position_gain;
    weighted.tail<3>() *= config_.orientation_gain;
    return weighted.squaredNorm();
  };
  const bool active_secondary =
      input.secondary_task.active &&
      input.secondary_task.jacobian.allFinite() &&
      std::isfinite(input.secondary_task.target) &&
      std::isfinite(input.secondary_task.activation) &&
      input.secondary_task.activation > 0.0 &&
      input.secondary_task.jacobian.squaredNorm() > 1.0e-16;
  // Preserve the nearest continuous IK branch once the Cartesian target is
  // already satisfied. Continuing to optimize a wrapped S1 arm-angle error
  // after convergence can walk an otherwise valid seed around the redundant
  // manifold and eventually select the opposite elbow branch. This matches
  // the proven DLS_IK baseline; the secondary objective only biases steps
  // that are already required for Cartesian convergence.
  if (converged(error)) {
    result.status = PoseDlsStatus::kConverged;
    result.detail = "pose_dls_converged";
    result.position_error_m = error.head<3>().norm();
    result.orientation_error_rad = error.tail<3>().norm();
    result.solve_time_us = elapsedMicroseconds(start);
    return result;
  }

  Vec7 best = current;
  Vec6 best_error = error;
  double best_merit = merit(error);
  bool best_updated = false;
  for (int iteration = 0; iteration < config_.max_iterations; ++iteration) {
    Eigen::JacobiSVD<Mat67> svd(
        sample.tcp_jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);
    if (svd.info() != Eigen::Success || !svd.singularValues().allFinite()) {
      result.detail = "pose_dls_svd_failed";
      break;
    }
    const double sigma_min = svd.singularValues().minCoeff();
    const double damping_ratio = std::clamp(
        (config_.singular_value_threshold - sigma_min) /
            config_.singular_value_threshold,
        0.0, 1.0);
    const double damping = config_.minimum_damping +
        (config_.maximum_damping - config_.minimum_damping) *
            damping_ratio * damping_ratio;
    Eigen::Matrix<double, kArmDof, 6> damped =
        Eigen::Matrix<double, kArmDof, 6>::Zero();
    for (int index = 0; index < 6; ++index) {
      const double sigma = svd.singularValues()[index];
      damped(index, index) = sigma / (sigma * sigma + damping * damping);
    }
    const Eigen::Matrix<double, kArmDof, 6> pseudo_inverse =
        svd.matrixV() * damped * svd.matrixU().transpose();
    Vec6 commanded_error = error;
    commanded_error.head<3>() *= config_.position_gain;
    commanded_error.tail<3>() *= config_.orientation_gain;
    Vec7 step = pseudo_inverse * commanded_error;

    const double rank_tolerance = Eigen::NumTraits<double>::epsilon() *
        static_cast<double>(std::max(sample.tcp_jacobian.rows(),
                                     sample.tcp_jacobian.cols())) *
        std::max(1.0, svd.singularValues().maxCoeff());
    Mat77 nullspace = Mat77::Zero();
    for (int column = 0; column < kArmDof; ++column) {
      if (column >= svd.singularValues().size() ||
          svd.singularValues()[column] <= rank_tolerance) {
        nullspace += svd.matrixV().col(column) *
                     svd.matrixV().col(column).transpose();
      }
    }
    const Vec7 nominal =
        0.5 * (input.limits.lower_position + input.limits.upper_position);
    Vec7 secondary = config_.nominal_posture_gain * (nominal - current);
    if (input.secondary_task.active &&
        input.secondary_task.jacobian.allFinite() &&
        std::isfinite(input.secondary_task.target) &&
        std::isfinite(input.secondary_task.activation)) {
      const double norm_squared = input.secondary_task.jacobian.squaredNorm();
      if (norm_squared > 1.0e-16) {
        // `target` is the arm-angle error measured at `input.seed`.  The
        // Cartesian DLS loop can take several internal steps before it
        // converges; treating that same error as a fresh command on every
        // iteration repeatedly applies the secondary motion and can select a
        // distant elbow branch.  Consume the locally predicted arm-angle
        // change accumulated since the seed instead.
        const double remaining_target =
            input.secondary_task.target -
            input.secondary_task.jacobian.dot(current - input.seed);
        secondary += config_.arm_angle_gain *
            std::clamp(input.secondary_task.activation, 0.0, 1.0) *
            remaining_target / norm_squared *
            input.secondary_task.jacobian;
      }
    }
    step += nullspace * secondary;
    for (int joint = 0; joint < kArmDof; ++joint) {
      step[joint] = std::clamp(step[joint],
                               -config_.maximum_joint_step_rad,
                               config_.maximum_joint_step_rad);
    }
    if (step.norm() > config_.maximum_step_norm_rad) {
      step *= config_.maximum_step_norm_rad / step.norm();
    }
    if (!step.allFinite() || step.norm() <= 1.0e-14) {
      result.detail = "pose_dls_no_direction";
      break;
    }
    result.iterations = iteration + 1;
    result.minimum_singular_value = sigma_min;
    result.damping = damping;
    Vec7 candidate = current;
    ArmKinematicSample candidate_sample;
    Vec6 candidate_error = Vec6::Zero();
    bool candidate_valid = false;
    int selected_projection_count = 0;
    double step_scale = 1.0;
    constexpr int kMaximumFeasibilityBacktracks = 12;
    for (int backtrack = 0;
         backtrack <= kMaximumFeasibilityBacktracks; ++backtrack) {
      const Vec7 unprojected = current + step_scale * step;
      candidate = unprojected.cwiseMax(lower).cwiseMin(upper);
      candidate_sample = input.evaluate(candidate);
      candidate_error =
          poseErrorWorld(input.target, candidate_sample.tcp_pose);
      const bool finite_candidate = candidate.allFinite() &&
          candidate_error.allFinite() &&
          candidate_sample.tcp_jacobian.allFinite();
      const bool feasible_candidate = finite_candidate &&
          (!input.candidate_feasible ||
           input.candidate_feasible(candidate_sample));
      if (feasible_candidate) {
        selected_projection_count = static_cast<int>(
            (unprojected.array() != candidate.array()).count());
        candidate_valid = true;
        break;
      }
      step_scale *= 0.5;
    }
    if (!candidate_valid) {
      result.detail = "pose_dls_no_feasible_candidate";
      break;
    }
    result.joint_projection_count += selected_projection_count;
    const double candidate_merit = merit(candidate_error);
    const bool improves_best =
        candidate_merit + config_.minimum_merit_improvement < best_merit;
    const bool preserves_converged_secondary =
        active_secondary && converged(best_error) && converged(candidate_error);
    if (improves_best || preserves_converged_secondary) {
      best = candidate;
      best_error = candidate_error;
      best_merit = candidate_merit;
      best_updated = true;
    }
    // Continue from the finite candidate even when this one is not the best.
    // DLS may need a non-monotonic internal step to cross a local branch, but
    // only best is ever published to the controller.
    current = candidate;
    sample = candidate_sample;
    error = candidate_error;
    if (converged(error)) {
      result.status = PoseDlsStatus::kConverged;
      result.detail = "pose_dls_converged";
      break;
    }
  }

  result.q = best;
  result.position_error_m = best_error.head<3>().norm();
  result.orientation_error_rad = best_error.tail<3>().norm();
  if (converged(best_error)) {
    result.status = PoseDlsStatus::kConverged;
  } else if (best_updated && best.allFinite() && best_error.allFinite()) {
    result.status = PoseDlsStatus::kImproved;
    result.detail = "pose_dls_iteration_limited";
  } else if (result.detail == "not_solved") {
    result.detail = "pose_dls_no_improvement";
  }
  result.solve_time_us = elapsedMicroseconds(start);
  return result;
}

}  // namespace tianji_qp_ik
