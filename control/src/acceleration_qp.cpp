#include "tianji_qp_ik/acceleration_qp.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/SVD>

#include <cmath>
#include <limits>

namespace tianji_qp_ik {
namespace {

Mat77 strictCartesianNullspace(const Mat67& jacobian) {
  if (!jacobian.allFinite()) {
    return Mat77::Identity();
  }
  Eigen::JacobiSVD<Mat67> svd(jacobian, Eigen::ComputeFullV);
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

Vec7 minimumNormCartesianParticular(const Mat67& jacobian,
                                     const Vec6& cartesian_acceleration) {
  if (!jacobian.allFinite() || !cartesian_acceleration.allFinite()) {
    return Vec7::Zero();
  }
  const Eigen::JacobiSVD<Mat67> svd(
      jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);
  return svd.solve(cartesian_acceleration);
}

}  // namespace

AccelerationQpProblem AccelerationQpBuilder::build(
    const ArmAccelerationInput& input) const {
  AccelerationQpProblem problem;
  const double qddot_weight = config_.regularization + config_.jerk_weight;
  problem.H.topLeftCorner<kArmDof, kArmDof>() =
      qddot_weight * Mat77::Identity();
  problem.H.block<6, 6>(kSlackStartIndex, kSlackStartIndex).diagonal() <<
      config_.slack_weight_position /
          (config_.slack_linear_scale * config_.slack_linear_scale),
      config_.slack_weight_position /
          (config_.slack_linear_scale * config_.slack_linear_scale),
      config_.slack_weight_position /
          (config_.slack_linear_scale * config_.slack_linear_scale),
      config_.slack_weight_orientation /
          (config_.slack_angular_scale * config_.slack_angular_scale),
      config_.slack_weight_orientation /
          (config_.slack_angular_scale * config_.slack_angular_scale),
      config_.slack_weight_orientation /
          (config_.slack_angular_scale * config_.slack_angular_scale);

  const Vec7 q_nominal = input.posture_reference_active &&
                                 input.posture_reference.allFinite()
      ? input.posture_reference
      : 0.5 * (input.limits.lower_position + input.limits.upper_position);
  const Vec7 qdot_nominal = input.posture_reference_active &&
                                    input.posture_velocity_reference.allFinite()
      ? input.posture_velocity_reference
      : Vec7::Zero();
  const Vec7 qddot_nominal = input.posture_reference_active &&
                                     input.posture_acceleration_reference.allFinite()
      ? input.posture_acceleration_reference
      : Vec7::Zero();
  const Vec7 qddot_posture =
      qddot_nominal + config_.posture_kp * (q_nominal - input.q_model) +
      config_.posture_kd * (qdot_nominal - input.qdot_model);
  problem.g.head<kArmDof>() =
      -config_.jerk_weight * input.qddot_previous;
  if (input.posture_reference_active) {
    const Mat77 nullspace = strictCartesianNullspace(input.jacobian);
    problem.H.topLeftCorner<kArmDof, kArmDof>().noalias() +=
        config_.posture_weight * nullspace;
    problem.g.head<kArmDof>().noalias() -=
        config_.posture_weight * nullspace * qddot_posture;
  } else {
    problem.H.topLeftCorner<kArmDof, kArmDof>().diagonal().array() +=
        config_.posture_weight;
    problem.g.head<kArmDof>().noalias() -=
        config_.posture_weight * qddot_posture;
  }
  const Vec6 cartesian_equality =
      input.desired_acceleration - input.jdot_qdot;
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
    const bool nullspace_only = input.arm_angle_task.nullspace_only;
    const Vec7 effective_jacobian = nullspace_only
        ? strictCartesianNullspace(input.jacobian) *
              input.arm_angle_task.jacobian
        : input.arm_angle_task.jacobian;
    const double effective_target = nullspace_only
        ? input.arm_angle_task.target -
              input.arm_angle_task.jacobian.dot(
                  minimumNormCartesianParticular(input.jacobian,
                                                  cartesian_equality))
        : input.arm_angle_task.target;
    problem.H.topLeftCorner<kArmDof, kArmDof>().noalias() +=
        weight * effective_jacobian * effective_jacobian.transpose();
    problem.g.head<kArmDof>().noalias() -=
        weight * effective_target * effective_jacobian;
  }

  problem.A.leftCols<kArmDof>() = input.jacobian;
  problem.A.block<6, 6>(0, kSlackStartIndex) =
      Eigen::Matrix<double, 6, 6>::Identity();
  if (config_.task_scaling_enabled) {
    problem.A.block<3, 1>(0, kBetaPositionIndex) =
        -input.desired_acceleration.head<3>();
    problem.A.block<3, 1>(3, kBetaOrientationIndex) =
        -input.desired_acceleration.tail<3>();
    problem.equality = -input.jdot_qdot;
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
    problem.equality = input.desired_acceleration - input.jdot_qdot;
    problem.H(kBetaPositionIndex, kBetaPositionIndex) = 1.0;
    problem.H(kBetaOrientationIndex, kBetaOrientationIndex) = 1.0;
    problem.lower[kBetaPositionIndex] = 0.0;
    problem.upper[kBetaPositionIndex] = 0.0;
    problem.lower[kBetaOrientationIndex] = 0.0;
    problem.upper[kBetaOrientationIndex] = 0.0;
  }
  problem.lower.head<kArmDof>() = input.bounds.lower;
  problem.upper.head<kArmDof>() = input.bounds.upper;
  problem.lower.segment<6>(kSlackStartIndex).setConstant(
      -std::numeric_limits<double>::infinity());
  problem.upper.segment<6>(kSlackStartIndex).setConstant(
      std::numeric_limits<double>::infinity());
  problem.joint_bounds = input.bounds;
  problem.linear_constraint = input.linear_constraint;
  return problem;
}

SafetyDecision validateAccelerationProblem(const AccelerationQpProblem& problem,
                                           const SafetyConfig& safety) {
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
  if ((problem.H - problem.H.transpose()).cwiseAbs().maxCoeff() >
      safety.hessian_eigenvalue_tolerance) {
    return {false, HoldReason::kInvalidHessian, -1};
  }
  Eigen::SelfAdjointEigenSolver<Mat15> eigenvalues(problem.H,
                                                   Eigen::EigenvaluesOnly);
  if (eigenvalues.info() != Eigen::Success ||
      eigenvalues.eigenvalues().minCoeff() <=
          safety.hessian_eigenvalue_tolerance) {
    return {false, HoldReason::kInvalidHessian, -1};
  }
  return {true, HoldReason::kNone, -1};
}

SafetyDecision validateAccelerationSolution(
    const AccelerationQpProblem& problem,
    const AccelerationQpSolution& solution,
    const AccelerationQpConfig& config, const SafetyConfig& safety) {
  const SafetyDecision problem_decision =
      validateAccelerationProblem(problem, safety);
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
    if (solution.x[index] <
            problem.lower[index] - safety.bound_tolerance ||
        solution.x[index] >
            problem.upper[index] + safety.bound_tolerance) {
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
  const double residual = (problem.A * solution.x - problem.equality).norm();
  if (!std::isfinite(residual) || residual > config.equality_tolerance) {
    return {false, HoldReason::kEqualityViolation, -1};
  }
  return {true, HoldReason::kNone, -1};
}

}  // namespace tianji_qp_ik
