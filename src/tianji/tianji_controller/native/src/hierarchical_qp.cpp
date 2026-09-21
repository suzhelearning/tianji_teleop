#include "tianji_qp_ik/hierarchical_qp.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>

namespace tianji_qp_ik {
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

HierarchicalQpBuilder::HierarchicalQpBuilder(HierarchicalQpConfig config)
    : config_(config) {}

HierarchicalQpProblem HierarchicalQpBuilder::build(const ArmIkInput& input) const {
  HierarchicalQpProblem problem;
  const double base_qdot_weight = config_.lambda_reg +
                                  config_.continuity_weight +
                                  config_.jerk_weight;
  problem.H.topLeftCorner<kArmDof, kArmDof>() =
      base_qdot_weight * Mat77::Identity();
  problem.H.block<kTrackingEqualities, kTrackingEqualities>(
      kSlackStartIndex, kSlackStartIndex)
      .diagonal() <<
      config_.slack_weight_position /
          (config_.slack_position_scale * config_.slack_position_scale),
      config_.slack_weight_position /
          (config_.slack_position_scale * config_.slack_position_scale),
      config_.slack_weight_position /
          (config_.slack_position_scale * config_.slack_position_scale),
      config_.slack_weight_orientation /
          (config_.slack_orientation_scale * config_.slack_orientation_scale),
      config_.slack_weight_orientation /
          (config_.slack_orientation_scale * config_.slack_orientation_scale),
      config_.slack_weight_orientation /
          (config_.slack_orientation_scale * config_.slack_orientation_scale);

  const Vec7 q_nominal =
      0.5 * (input.limits.lower_position + input.limits.upper_position);
  const Vec7 qdot_nominal =
      -config_.nominal_gain * (input.q_measured - q_nominal);
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
  if (input.posture_task.active &&
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
  if (input.posture_task.active &&
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
  if (input.posture_task.active &&
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
  if (config_.arm_angle_weight > 0.0 && input.arm_angle_task.active &&
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
  problem.A.leftCols<kArmDof>() = input.jacobian;
  problem.A.block<kTrackingEqualities, kTrackingEqualities>(
      0, kSlackStartIndex) =
      Eigen::Matrix<double, kTrackingEqualities, kTrackingEqualities>::Identity();
  if (config_.task_scaling_enabled) {
    problem.A.block<3, 1>(0, kBetaPositionIndex) =
        -input.desired_twist.head<3>();
    problem.A.block<3, 1>(3, kBetaOrientationIndex) =
        -input.desired_twist.tail<3>();
    problem.equality.setZero();
    problem.H(kBetaPositionIndex, kBetaPositionIndex) =
        config_.task_scaling_weight_position;
    problem.H(kBetaOrientationIndex, kBetaOrientationIndex) =
        config_.task_scaling_weight_orientation;
    problem.g[kBetaPositionIndex] =
        -config_.task_scaling_weight_position;
    problem.g[kBetaOrientationIndex] =
        -config_.task_scaling_weight_orientation;
    problem.lower[kBetaPositionIndex] =
        config_.task_scaling_min_position;
    problem.upper[kBetaPositionIndex] = 1.0;
    problem.lower[kBetaOrientationIndex] =
        config_.task_scaling_min_orientation;
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

}  // namespace tianji_qp_ik
