#include "tianji_qp_ik/nullspace_dls.hpp"

#include <gtest/gtest.h>

#include <Eigen/LU>

#include <cmath>

namespace tianji_qp_ik {
namespace {

DlsConfig dlsConfig() {
  DlsConfig config;
  config.damping = 1e-3;
  config.nominal_gain = 0.2;
  return config;
}

ArmIkInput fullRankInputWithWideBounds() {
  ArmIkInput input;
  input.q_measured << -0.3, 0.2, -0.1, 0.4, -0.2, 0.1, 0.3;
  input.limits.lower_position = Vec7::Constant(-2.0);
  input.limits.upper_position = Vec7::Constant(2.0);
  input.limits.velocity = Vec7::Constant(5.0);
  input.bounds.lower = Vec7::Constant(-10.0);
  input.bounds.upper = Vec7::Constant(10.0);
  input.desired_twist << 0.25, -0.18, 0.12, 0.20, -0.14, 0.16;
  input.jacobian <<
      1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
      0.0, 1.0, 0.0, 0.0, 0.0, 0.1, 0.0,
      0.0, 0.0, 1.0, 0.0, 0.1, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.1,
      0.0, 0.0, 0.1, 0.0, 1.0, 0.0, 0.0,
      0.0, 0.1, 0.0, 0.0, 0.0, 1.0, 0.0;
  return input;
}

Vec7 analyticRawVelocity(const ArmIkInput& input) {
  const DlsConfig config = dlsConfig();
  const Eigen::Matrix<double, 6, 6> inverse_term =
      (input.jacobian * input.jacobian.transpose() +
       config.damping * config.damping *
           Eigen::Matrix<double, 6, 6>::Identity())
          .inverse();
  const Eigen::Matrix<double, 7, 6> pinv =
      input.jacobian.transpose() * inverse_term;
  const Vec7 qdot_nominal =
      -config.nominal_gain *
      (input.q_measured -
       0.5 * (input.limits.lower_position + input.limits.upper_position));
  return pinv * input.desired_twist +
         (Mat77::Identity() - pinv * input.jacobian) * qdot_nominal;
}

TEST(NullspaceDlsIk, MatchesDampedNullspaceFormulaInsideBounds) {
  const ArmIkInput input = fullRankInputWithWideBounds();
  NullspaceDlsIk7 solver(dlsConfig());
  const ArmIkResult result = solver.solve(input);

  ASSERT_EQ(result.status, SolverStatus::kSolved) << result.detail;
  EXPECT_TRUE(result.qdot.isApprox(analyticRawVelocity(input), 1e-10));
  EXPECT_TRUE(result.slack.isApprox(
      input.desired_twist - input.jacobian * result.qdot, 1e-12));
  EXPECT_LT(result.equality_residual, 1e-14);
}

TEST(NullspaceDlsIk, UniformlyScalesAllComponentsAtOneActiveBound) {
  ArmIkInput input = fullRankInputWithWideBounds();
  const Vec7 raw = analyticRawVelocity(input);
  Eigen::Index limiting_index = 0;
  raw.maxCoeff(&limiting_index);
  ASSERT_GT(raw[limiting_index], 0.0);
  input.bounds.upper[limiting_index] = 0.25 * raw[limiting_index];

  NullspaceDlsIk7 solver(dlsConfig());
  const ArmIkResult result = solver.solve(input);

  ASSERT_EQ(result.status, SolverStatus::kSolved) << result.detail;
  EXPECT_NEAR(result.qdot[limiting_index], input.bounds.upper[limiting_index], 1e-12);
  for (int index = 0; index < kArmDof; ++index) {
    if (std::abs(raw[index]) > 1e-12) {
      EXPECT_NEAR(result.qdot[index] / raw[index], 0.25, 1e-10);
    }
  }
}

TEST(NullspaceDlsIk, HandlesAccelerationBoundsThatExcludeZero) {
  ArmIkInput input = fullRankInputWithWideBounds();
  input.qdot_prev = Vec7::Constant(0.10);
  input.bounds.lower = Vec7::Constant(0.08);
  input.bounds.upper = Vec7::Constant(0.12);

  NullspaceDlsIk7 solver(dlsConfig());
  const ArmIkResult result = solver.solve(input);

  ASSERT_EQ(result.status, SolverStatus::kSolved) << result.detail;
  EXPECT_TRUE((result.qdot.array() >= input.bounds.lower.array() - 1e-12).all());
  EXPECT_TRUE((result.qdot.array() <= input.bounds.upper.array() + 1e-12).all());
  EXPECT_TRUE(result.qdot.allFinite());
}

TEST(NullspaceDlsIk, RemainsFiniteForSingularAndZeroJacobians) {
  for (int jacobian_case = 0; jacobian_case < 2; ++jacobian_case) {
    ArmIkInput input = fullRankInputWithWideBounds();
    if (jacobian_case == 0) {
      input.jacobian.row(5) = input.jacobian.row(4);
    } else {
      input.jacobian.setZero();
    }
    NullspaceDlsIk7 solver(dlsConfig());
    const ArmIkResult result = solver.solve(input);
    EXPECT_EQ(result.status, SolverStatus::kSolved) << result.detail;
    EXPECT_TRUE(result.qdot.allFinite());
    EXPECT_TRUE(result.slack.allFinite());
  }
}

}  // namespace
}  // namespace tianji_qp_ik
