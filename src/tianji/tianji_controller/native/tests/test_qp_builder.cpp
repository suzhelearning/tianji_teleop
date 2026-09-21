#include "tianji_qp_ik/qp_builder.hpp"

#include <gtest/gtest.h>

#include <Eigen/Eigenvalues>
#include <algorithm>

namespace tianji_qp_ik {
namespace {

QpConfig qpConfig() {
  QpConfig config;
  config.position_weight = 2.0;
  config.orientation_weight = 0.5;
  config.regularization = 1e-4;
  config.nominal_weight = 0.02;
  config.nominal_gain = 0.3;
  return config;
}

ArmLimits limits() {
  ArmLimits result;
  result.lower_position = Vec7::Constant(-1.0);
  result.upper_position = Vec7::Constant(2.0);
  result.velocity = Vec7::LinSpaced(0.5, 1.1);
  return result;
}

TEST(QpBuilder, ExpandsWeightedLeastSquaresExactly) {
  JointLimitConfig limit_config;
  limit_config.margin_rad = 0.05;
  limit_config.velocity_scale = 0.4;
  QpBuilder builder(qpConfig(), limit_config);

  Mat67 jacobian;
  for (int row = 0; row < jacobian.rows(); ++row) {
    for (int column = 0; column < jacobian.cols(); ++column) {
      jacobian(row, column) = 0.03 * static_cast<double>(1 + row + 2 * column);
    }
  }
  const Vec6 desired = Vec6::LinSpaced(-0.2, 0.3);
  const Vec7 q = Vec7::LinSpaced(-0.4, 0.8);
  const QpProblem7 problem = builder.build(jacobian, desired, q, limits(), 0.001);

  Eigen::Matrix<double, 6, 1> squared_weights;
  squared_weights << 4.0, 4.0, 4.0, 0.25, 0.25, 0.25;
  const Mat77 expected_h = jacobian.transpose() * squared_weights.asDiagonal() * jacobian +
                           (qpConfig().regularization + qpConfig().nominal_weight) *
                               Mat77::Identity();
  const Vec7 nominal = 0.5 * (limits().lower_position + limits().upper_position);
  const Vec7 qdot_nominal = -qpConfig().nominal_gain * (q - nominal);
  const Vec7 expected_g = -jacobian.transpose() * squared_weights.asDiagonal() * desired -
                          qpConfig().nominal_weight * qdot_nominal;

  EXPECT_TRUE(problem.H.isApprox(expected_h, 1e-13));
  EXPECT_TRUE(problem.g.isApprox(expected_g, 1e-13));
  EXPECT_TRUE(problem.H.isApprox(problem.H.transpose(), 1e-14));
  Eigen::SelfAdjointEigenSolver<Mat77> eigenvalues(problem.H);
  EXPECT_GT(eigenvalues.eigenvalues().minCoeff(), 0.0);
}

TEST(QpBuilder, IntersectsVelocityAndOneStepPositionBounds) {
  JointLimitConfig limit_config;
  limit_config.margin_rad = 0.05;
  limit_config.velocity_scale = 0.4;
  QpBuilder builder(qpConfig(), limit_config);
  const ArmLimits arm_limits = limits();
  Vec7 q = Vec7::Constant(0.5);
  q[0] = arm_limits.lower_position[0] + limit_config.margin_rad + 1e-4;
  q[1] = arm_limits.upper_position[1] - limit_config.margin_rad - 2e-4;
  constexpr double kDt = 0.001;

  const QpProblem7 problem =
      builder.build(Mat67::Zero(), Vec6::Zero(), q, arm_limits, kDt);
  for (int index = 0; index < kArmDof; ++index) {
    const double expected_lower =
        std::max(-limit_config.velocity_scale * arm_limits.velocity[index],
                 (arm_limits.lower_position[index] + limit_config.margin_rad - q[index]) / kDt);
    const double expected_upper =
        std::min(limit_config.velocity_scale * arm_limits.velocity[index],
                 (arm_limits.upper_position[index] - limit_config.margin_rad - q[index]) / kDt);
    EXPECT_DOUBLE_EQ(problem.lower[index], expected_lower);
    EXPECT_DOUBLE_EQ(problem.upper[index], expected_upper);
  }
  EXPECT_NEAR(problem.lower[0], -0.1, 1e-10);
  EXPECT_NEAR(problem.upper[1], 0.2, 1e-10);
}

TEST(QpBuilder, RejectsInvalidTimeStep) {
  QpBuilder builder(qpConfig(), JointLimitConfig{});
  EXPECT_THROW(builder.build(Mat67::Zero(), Vec6::Zero(), Vec7::Zero(), limits(), 0.0),
               std::invalid_argument);
}

TEST(QpBuilder, EnforcesOneStepMarginNearBothLimitsOfEveryJoint) {
  JointLimitConfig limit_config;
  limit_config.margin_rad = 0.05;
  limit_config.velocity_scale = 0.4;
  QpBuilder builder(qpConfig(), limit_config);
  const ArmLimits arm_limits = limits();
  constexpr double kDt = 0.001;
  constexpr double kDistanceInsideMargin = 1e-4;

  for (int joint = 0; joint < kArmDof; ++joint) {
    Vec7 q = 0.5 * (arm_limits.lower_position + arm_limits.upper_position);
    q[joint] = arm_limits.lower_position[joint] + limit_config.margin_rad +
               kDistanceInsideMargin;
    QpProblem7 problem = builder.build(Mat67::Zero(), Vec6::Zero(), q, arm_limits, kDt);
    EXPECT_NEAR(problem.lower[joint], -kDistanceInsideMargin / kDt, 1e-10);

    q[joint] = arm_limits.upper_position[joint] - limit_config.margin_rad -
               kDistanceInsideMargin;
    problem = builder.build(Mat67::Zero(), Vec6::Zero(), q, arm_limits, kDt);
    EXPECT_NEAR(problem.upper[joint], kDistanceInsideMargin / kDt, 1e-10);
  }
}

}  // namespace
}  // namespace tianji_qp_ik
