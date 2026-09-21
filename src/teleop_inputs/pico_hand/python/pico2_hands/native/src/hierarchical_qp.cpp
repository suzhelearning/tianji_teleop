// Ported from TJ_arm_control_pico_ee_ik; see PORTING.md.
#include "tianji_v131/hierarchical_qp.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace tianji_v131 {
namespace {

Mat77 strictCartesianNullspace(const Mat67& jacobian) {
  if (!jacobian.allFinite()) {
    return Mat77::Zero();
  }
  const Eigen::JacobiSVD<Mat67> svd(jacobian, Eigen::ComputeFullV);
  if (svd.info() != Eigen::Success) {
    return Mat77::Zero();
  }
  double maximum_singular_value = 0.0;
  for (const double singular_value : svd.singularValues()) {
    maximum_singular_value =
        std::max(maximum_singular_value, singular_value);
  }
  const double rank_tolerance =
      Eigen::NumTraits<double>::epsilon() *
      static_cast<double>(std::max(jacobian.rows(), jacobian.cols())) *
      std::max(1.0, maximum_singular_value);
  int rank = 0;
  for (const double singular_value : svd.singularValues()) {
    if (singular_value > rank_tolerance) {
      ++rank;
    }
  }
  Mat77 nullspace = Mat77::Zero();
  for (int column = rank; column < kArmDof; ++column) {
    nullspace.noalias() += svd.matrixV().col(column) *
                           svd.matrixV().col(column).transpose();
  }
  return nullspace;
}

Vec7 minimumNormCartesianVelocity(const Mat67& jacobian,
                                  const Vec6& desired_twist) {
  if (!jacobian.allFinite() || !desired_twist.allFinite()) {
    return Vec7::Zero();
  }
  const Eigen::JacobiSVD<Mat67> svd(
      jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);
  if (svd.info() != Eigen::Success) {
    return Vec7::Zero();
  }
  return svd.solve(desired_twist);
}

}  // namespace

HierarchicalQpBuilder::HierarchicalQpBuilder(
    HierarchicalQpConfig config, PicoEeV131VelocityQpConfig v131_config)
    : config_(config), v131_config_(std::move(v131_config)) {}

HierarchicalQpProblem HierarchicalQpBuilder::build(const ArmIkInput& input) const {
  HierarchicalQpProblem problem;
  const bool v131_active = v131_config_.enabled &&
                           input.pico_ee_v131_task.active;
  const auto& v131_task = input.pico_ee_v131_task;
  const bool adaptive_nullspace_mode =
      v131_active && v131_config_.adaptive_nullspace_enabled &&
      (v131_task.adaptive_nullspace_mode ||
       v131_task.adaptive_nullspace_active);
  const bool adaptive_nullspace_active =
      adaptive_nullspace_mode &&
      v131_task.adaptive_nullspace_active &&
      v131_task.nullspace_basis.allFinite() &&
      v131_task.nullspace_basis.norm() > 1.0e-9 &&
      (input.jacobian * v131_task.nullspace_basis).norm() <= 1.0e-8 &&
      std::isfinite(v131_task.alpha_command) &&
      std::isfinite(v131_task.alpha_previous) &&
      std::isfinite(v131_task.alpha_trend) &&
      std::isfinite(v131_task.nullspace_reference_weight) &&
      v131_task.nullspace_reference_weight >= 0.0;
  const double activation =
      v131_active && std::isfinite(input.pico_ee_v131_task.activation)
          ? std::clamp(input.pico_ee_v131_task.activation, 0.0, 1.0)
          : 0.0;
  const double velocity_multiplier =
      v131_active
          ? 1.0 + activation * (v131_config_.singularity_velocity_multiplier -
                                1.0)
          : 1.0;
  const double posture_multiplier =
      v131_active
          ? 1.0 + activation * (v131_config_.singularity_posture_multiplier -
                                1.0)
          : 1.0;
  const double base_qdot_weight =
      v131_active
          ? v131_config_.velocity_regularization_weight * velocity_multiplier +
                v131_config_.continuity_weight + v131_config_.jerk_trend_weight
          : config_.lambda_reg + config_.continuity_weight +
                config_.jerk_weight;
  if (!v131_active) {
    problem.H.topLeftCorner<kArmDof, kArmDof>() =
        base_qdot_weight * Mat77::Identity();
  } else {
    for (int joint = 0; joint < kArmDof; ++joint) {
      const double scale = input.pico_ee_v131_task.velocity_scale[joint];
      const double inverse_scale_squared =
          1.0 / (std::max(std::abs(scale), 1.0e-9) *
                 std::max(std::abs(scale), 1.0e-9));
      problem.H(joint, joint) = base_qdot_weight * inverse_scale_squared;
    }
  }
  const double position_slack_weight =
      v131_active ? v131_config_.slack_weight_position
                  : config_.slack_weight_position;
  const double orientation_slack_weight =
      v131_active
          ? v131_config_.slack_weight_orientation *
                (1.0 + activation *
                           (v131_config_.singularity_orientation_scale - 1.0))
          : config_.slack_weight_orientation;
  const double position_slack_scale =
      v131_active ? v131_config_.slack_position_scale
                  : config_.slack_position_scale;
  const double orientation_slack_scale =
      v131_active ? v131_config_.slack_orientation_scale
                  : config_.slack_orientation_scale;
  problem.H.block<kTrackingEqualities, kTrackingEqualities>(
      kSlackStartIndex, kSlackStartIndex)
      .diagonal() <<
      position_slack_weight / (position_slack_scale * position_slack_scale),
      position_slack_weight / (position_slack_scale * position_slack_scale),
      position_slack_weight / (position_slack_scale * position_slack_scale),
      orientation_slack_weight /
          (orientation_slack_scale * orientation_slack_scale),
      orientation_slack_weight /
          (orientation_slack_scale * orientation_slack_scale),
      orientation_slack_weight /
          (orientation_slack_scale * orientation_slack_scale);

  Vec7 qdot_nominal = Vec7::Zero();
  if (v131_active) {
    const double tau = std::max(v131_config_.posture_time_constant_seconds,
                                1.0e-9);
    qdot_nominal =
        (input.pico_ee_v131_task.zero_posture - input.q_measured) / tau;
    for (int joint = 0; joint < kArmDof; ++joint) {
      const double limit = std::abs(input.limits.velocity[joint]);
      if (std::isfinite(limit) && limit > 0.0) {
        qdot_nominal[joint] =
            std::clamp(qdot_nominal[joint], -limit, limit);
      }
      const double scale = input.pico_ee_v131_task.velocity_scale[joint];
      const double inverse_scale_squared =
          1.0 / (std::max(std::abs(scale), 1.0e-9) *
                 std::max(std::abs(scale), 1.0e-9));
      const double posture_weight = adaptive_nullspace_mode
                                        ? 0.0
                                        : v131_config_.posture_weight *
                                              posture_multiplier *
                                              (joint >= 4
                                                   ? v131_config_
                                                         .wrist_posture_weight_scale
                                                   : 1.0);
      problem.H(joint, joint) += posture_weight * inverse_scale_squared;
      problem.g[joint] -=
          inverse_scale_squared *
          (v131_config_.continuity_weight * input.qdot_prev[joint] +
           v131_config_.jerk_trend_weight *
               (input.qdot_prev[joint] + input.qddot_prev[joint] * input.dt) +
           posture_weight * qdot_nominal[joint]);
    }
    if (adaptive_nullspace_active) {
      const Vec7 basis = v131_task.nullspace_basis.normalized();
      const double reference_weight =
          v131_task.nullspace_reference_weight * posture_multiplier;
      const double continuity_weight =
          v131_config_.nullspace_continuity_weight;
      const double jerk_weight = v131_config_.nullspace_jerk_weight;
      const double total_weight =
          reference_weight + continuity_weight + jerk_weight;
      problem.H.topLeftCorner<kArmDof, kArmDof>().noalias() +=
          total_weight * basis * basis.transpose();
      problem.g.head<kArmDof>().noalias() -=
          (reference_weight * v131_task.alpha_command +
           continuity_weight * v131_task.alpha_previous +
           jerk_weight * v131_task.alpha_trend) *
          basis;
    }
  } else {
    const Vec7 q_nominal =
        0.5 * (input.limits.lower_position + input.limits.upper_position);
    qdot_nominal = -config_.nominal_gain * (input.q_measured - q_nominal);
    problem.g.head<kArmDof>() =
        -config_.continuity_weight * input.qdot_prev -
        config_.jerk_weight *
            (input.qdot_prev + input.qddot_prev * input.dt);
    for (int joint = 0; joint < kArmDof; ++joint) {
      const double posture_weight =
          config_.posture_weight *
          (joint >= 4 ? config_.wrist_posture_weight_scale : 1.0);
      problem.H(joint, joint) += posture_weight;
      problem.g[joint] -= posture_weight * qdot_nominal[joint];
    }
  }
  if (!v131_active && input.posture_task.active &&
      (input.posture_task.source == JointVelocityPostureSource::kSparkSoftQp ||
       input.posture_task.source ==
           JointVelocityPostureSource::kSparkJointReference ||
       input.posture_task.source ==
           JointVelocityPostureSource::kSparkFeedforwardJointReference) &&
      input.posture_task.target.allFinite() &&
      std::isfinite(input.posture_task.activation) &&
      input.posture_task.activation > 0.0 &&
      std::isfinite(input.posture_task.weight) &&
      input.posture_task.weight > 0.0) {
    const double weight =
        input.posture_task.activation * input.posture_task.weight;
    problem.H.topLeftCorner<kArmDof, kArmDof>().diagonal().array() += weight;
    problem.g.head<kArmDof>() -= weight * input.posture_task.target;
  }
  if (!v131_active && input.posture_task.active &&
      (input.posture_task.source ==
           JointVelocityPostureSource::kSparkJointReference ||
       input.posture_task.source ==
           JointVelocityPostureSource::kSparkFeedforwardJointReference) &&
      std::isfinite(input.posture_task.activation) &&
      input.posture_task.activation > 0.0 &&
      std::isfinite(input.posture_task.smoothness_weight) &&
      input.posture_task.smoothness_weight > 0.0) {
    const double weight = input.posture_task.activation *
                          input.posture_task.smoothness_weight;
    problem.H.topLeftCorner<kArmDof, kArmDof>().diagonal().array() += weight;
    problem.g.head<kArmDof>() -= weight * input.qdot_prev;
  }
  if (!v131_active && input.posture_task.active &&
      input.posture_task.source ==
          JointVelocityPostureSource::kSparkFeedforwardJointReference &&
      std::isfinite(input.posture_task.activation) &&
      input.posture_task.activation > 0.0 &&
      std::isfinite(input.posture_task.jerk_smoothness_weight) &&
      input.posture_task.jerk_smoothness_weight > 0.0 &&
      std::isfinite(input.dt) && input.dt > 0.0 &&
      input.qddot_prev.allFinite()) {
    const double weight = input.posture_task.activation *
                          input.posture_task.jerk_smoothness_weight;
    const Vec7 predicted_qdot =
        input.qdot_prev + input.qddot_prev * input.dt;
    problem.H.topLeftCorner<kArmDof, kArmDof>().diagonal().array() += weight;
    problem.g.head<kArmDof>() -= weight * predicted_qdot;
  }
  if (!v131_active && config_.arm_angle_weight > 0.0 &&
      input.arm_angle_task.active &&
      input.arm_angle_task.jacobian.allFinite() &&
      std::isfinite(input.arm_angle_task.target) &&
      std::isfinite(input.arm_angle_task.activation) &&
      input.arm_angle_task.activation > 0.0 &&
      std::isfinite(input.arm_angle_task.weight_scale) &&
      input.arm_angle_task.weight_scale > 0.0) {
    const double weight =
        config_.arm_angle_weight * input.arm_angle_task.activation *
        input.arm_angle_task.weight_scale;
    // Without task scaling, preserve the historical strict Cartesian task and
    // spend only its exact null-space on arm-angle tracking.  With task
    // scaling enabled, a non-nullspace-only arm task may participate directly:
    // beta_p/beta_R then reduce Cartesian magnitude coherently when the arm
    // posture and hard dynamic bounds cannot all be met at full speed.
    const bool nullspace_only =
        !config_.task_scaling_enabled || input.arm_angle_task.nullspace_only;
    Vec7 effective_jacobian = input.arm_angle_task.jacobian;
    double effective_target = input.arm_angle_task.target;
    if (nullspace_only) {
      const Mat77 nullspace = strictCartesianNullspace(input.jacobian);
      effective_jacobian = nullspace * input.arm_angle_task.jacobian;
      const Vec7 cartesian_particular = minimumNormCartesianVelocity(
          input.jacobian, input.desired_twist);
      effective_target -=
          input.arm_angle_task.jacobian.dot(cartesian_particular);
    }
    problem.H.topLeftCorner<kArmDof, kArmDof>().noalias() +=
        weight * effective_jacobian * effective_jacobian.transpose();
    problem.g.head<kArmDof>().noalias() -=
        weight * effective_target * effective_jacobian;
  }
  if (v131_active && input.pico_ee_v131_task.escape_active &&
      input.pico_ee_v131_task.gradient_valid &&
      input.pico_ee_v131_task.gradient.allFinite() &&
      std::isfinite(input.pico_ee_v131_task.requested_sigma_dot) &&
      input.pico_ee_v131_task.gradient.squaredNorm() > 1.0e-12 &&
      std::isfinite(input.pico_ee_v131_task.gradient_age_seconds) &&
      input.pico_ee_v131_task.gradient_age_seconds <=
          v131_config_.gradient_max_age_seconds) {
    const Vec7 gradient =
        input.pico_ee_v131_task.gradient.normalized();
    const double weight = v131_config_.singularity_escape_weight * activation;
    const double requested_sigma_dot =
        input.pico_ee_v131_task.requested_sigma_dot;
    problem.H.topLeftCorner<kArmDof, kArmDof>().noalias() +=
        weight * gradient * gradient.transpose();
    problem.g.head<kArmDof>().noalias() -=
        weight * requested_sigma_dot * gradient;
  }
  problem.A.leftCols<kArmDof>() = input.jacobian;
  problem.A.block<kTrackingEqualities, kTrackingEqualities>(
      0, kSlackStartIndex) =
      Eigen::Matrix<double, kTrackingEqualities, kTrackingEqualities>::Identity();
  const bool task_scaling_enabled =
      v131_active ? true : config_.task_scaling_enabled;
  if (task_scaling_enabled) {
    problem.A.block<3, 1>(0, kBetaPositionIndex) =
        -input.desired_twist.head<3>();
    problem.A.block<3, 1>(3, kBetaOrientationIndex) =
        -input.desired_twist.tail<3>();
    problem.equality.setZero();
    const double beta_position_weight =
        v131_active ? v131_config_.task_scaling_weight_position
                    : config_.task_scaling_weight_position;
    const double beta_orientation_weight =
        v131_active ? v131_config_.task_scaling_weight_orientation
                    : config_.task_scaling_weight_orientation;
    const double beta_position_min =
        v131_active ? v131_config_.task_scaling_min_position
                    : config_.task_scaling_min_position;
    const double beta_orientation_min =
        v131_active ? v131_config_.task_scaling_min_orientation
                    : config_.task_scaling_min_orientation;
    problem.H(kBetaPositionIndex, kBetaPositionIndex) =
        beta_position_weight;
    problem.H(kBetaOrientationIndex, kBetaOrientationIndex) =
        beta_orientation_weight;
    problem.g[kBetaPositionIndex] =
        -beta_position_weight;
    problem.g[kBetaOrientationIndex] =
        -beta_orientation_weight;
    problem.lower[kBetaPositionIndex] =
        beta_position_min;
    problem.upper[kBetaPositionIndex] = 1.0;
    problem.lower[kBetaOrientationIndex] =
        beta_orientation_min;
    problem.upper[kBetaOrientationIndex] = 1.0;
  } else {
    problem.equality = input.desired_twist;
    // Keep the extended 15-variable problem strictly positive definite while
    // fixing both inactive beta variables to zero. This is algebraically
    // equivalent to the previous 13-variable velocity QP.
    problem.H(kBetaPositionIndex, kBetaPositionIndex) = 1.0;
    problem.H(kBetaOrientationIndex, kBetaOrientationIndex) = 1.0;
    problem.lower[kBetaPositionIndex] = 0.0;
    problem.upper[kBetaPositionIndex] = 0.0;
    problem.lower[kBetaOrientationIndex] = 0.0;
    problem.upper[kBetaOrientationIndex] = 0.0;
  }
  problem.lower.head<kArmDof>() = input.bounds.lower;
  problem.upper.head<kArmDof>() = input.bounds.upper;
  problem.lower.segment<kTrackingEqualities>(kSlackStartIndex).setConstant(
      -std::numeric_limits<double>::infinity());
  problem.upper.segment<kTrackingEqualities>(kSlackStartIndex).setConstant(
      std::numeric_limits<double>::infinity());
  problem.joint_bounds = input.bounds;
  problem.linear_constraint = input.linear_constraint;
  problem.branch_lock_constraint = input.branch_lock_constraint;
  return problem;
}

SafetyDecision validateHierarchicalProblem(
    const HierarchicalQpProblem& problem, const SafetyConfig& safety) {
  if (!problem.H.allFinite() || !problem.g.allFinite() ||
      !problem.A.allFinite() || !problem.equality.allFinite() ||
      !problem.lower.head<kArmDof>().allFinite() ||
      !problem.upper.head<kArmDof>().allFinite()) {
    return {false, HoldReason::kNonFiniteProblem, -1};
  }
  if (problem.linear_constraint.active &&
      (!problem.linear_constraint.jacobian.allFinite() ||
       std::isnan(problem.linear_constraint.lower) ||
       std::isnan(problem.linear_constraint.upper) ||
       problem.linear_constraint.lower > problem.linear_constraint.upper)) {
    return {false, HoldReason::kNonFiniteProblem, -1};
  }
  if (problem.branch_lock_constraint.active &&
      (!problem.branch_lock_constraint.jacobian.allFinite() ||
       std::isnan(problem.branch_lock_constraint.lower) ||
       std::isnan(problem.branch_lock_constraint.upper) ||
       problem.branch_lock_constraint.lower >
           problem.branch_lock_constraint.upper)) {
    return {false, HoldReason::kNonFiniteProblem, -1};
  }

  for (int index = 0; index < kArmDof; ++index) {
    if (problem.lower[index] > problem.upper[index]) {
      return {false, HoldReason::kInfeasibleBounds, index};
    }
  }
  for (int index = kSlackStartIndex;
       index < kSlackStartIndex + kSlackVariables; ++index) {
    if (!std::isinf(problem.lower[index]) || problem.lower[index] >= 0.0 ||
        !std::isinf(problem.upper[index]) || problem.upper[index] <= 0.0) {
      return {false, HoldReason::kNonFiniteProblem, -1};
    }
  }
  for (const int index : {kBetaPositionIndex, kBetaOrientationIndex}) {
    if (!std::isfinite(problem.lower[index]) ||
        !std::isfinite(problem.upper[index]) ||
        problem.lower[index] > problem.upper[index]) {
      return {false, HoldReason::kNonFiniteProblem, -1};
    }
  }
  const double symmetry_error =
      (problem.H - problem.H.transpose()).cwiseAbs().maxCoeff();
  if (symmetry_error > safety.hessian_eigenvalue_tolerance) {
    return {false, HoldReason::kInvalidHessian, -1};
  }
  Eigen::SelfAdjointEigenSolver<MatVelocityQp> eigenvalues(
      problem.H, Eigen::EigenvaluesOnly);
  if (eigenvalues.info() != Eigen::Success ||
      eigenvalues.eigenvalues().minCoeff() <= safety.hessian_eigenvalue_tolerance) {
    return {false, HoldReason::kInvalidHessian, -1};
  }
  return {true, HoldReason::kNone, -1};
}

SafetyDecision validateHierarchicalSolution(
    const HierarchicalQpProblem& problem,
    const HierarchicalQpSolution& solution,
    const HierarchicalQpConfig& config,
    const SafetyConfig& safety) {
  const SafetyDecision problem_decision =
      validateHierarchicalProblem(problem, safety);
  if (!problem_decision.accepted) {
    return problem_decision;
  }
  if (solution.status != SolverStatus::kSolved) {
    return {false, HoldReason::kSolverFailure, -1};
  }
  if (!solution.x.allFinite()) {
    return {false, HoldReason::kNonFiniteSolution, -1};
  }
  for (int index = 0; index < kArmDof; ++index) {
    if (solution.x[index] < problem.lower[index] - safety.bound_tolerance ||
        solution.x[index] > problem.upper[index] + safety.bound_tolerance) {
      return {false, HoldReason::kBoundViolation, index};
    }
  }
  for (const int index : {kBetaPositionIndex, kBetaOrientationIndex}) {
    if (solution.x[index] <
            problem.lower[index] - safety.bound_tolerance ||
        solution.x[index] >
            problem.upper[index] + safety.bound_tolerance) {
      return {false, HoldReason::kBoundViolation, index};
    }
  }
  if (problem.linear_constraint.active) {
    const double value = problem.linear_constraint.jacobian.dot(
        solution.x.head<kArmDof>());
    if (!std::isfinite(value) ||
        value < problem.linear_constraint.lower - safety.bound_tolerance ||
        value > problem.linear_constraint.upper + safety.bound_tolerance) {
      return {false, HoldReason::kBoundViolation, -1};
    }
  }
  if (problem.branch_lock_constraint.active) {
    const double value = problem.branch_lock_constraint.jacobian.dot(
        solution.x.head<kArmDof>());
    if (!std::isfinite(value) ||
        value < problem.branch_lock_constraint.lower -
                    safety.bound_tolerance ||
        value > problem.branch_lock_constraint.upper +
                    safety.bound_tolerance) {
      return {false, HoldReason::kBoundViolation, -1};
    }
  }
  const double equality_residual =
      (problem.A * solution.x - problem.equality).norm();
  if (!std::isfinite(equality_residual) ||
      equality_residual > config.equality_tolerance) {
    return {false, HoldReason::kEqualityViolation, -1};
  }
  return {true, HoldReason::kNone, -1};
}

}  // namespace tianji_v131
