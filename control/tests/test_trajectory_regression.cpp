#include "tianji_qp_ik/controller.hpp"
#include "tianji_qp_ik/so3.hpp"
#include "tianji_qp_ik/target_manager.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <memory>

namespace tianji_qp_ik {
namespace {

constexpr double kDt = 0.005;

TEST(TrajectoryRegression, DualArmCircleRemainsFiniteAndInsideSafetyMargins) {
  const std::filesystem::path root(TIANJI_PROJECT_SOURCE_DIR);
  const QpIkConfig config =
      loadConfig((root / "config" / "qp_ik_hierarchical.yaml").string());
  MujocoRobot robot((root / "models" / "marvin_m6_qp_test.xml").string());
  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    robot.setArmPosition(side, 0.5 * (robot.mapping(side).limits.lower_position +
                                     robot.mapping(side).limits.upper_position));
  }
  robot.forward();
  TargetManager targets(config,
                        {robot.tcpPose(ArmSide::kLeft), robot.tcpPose(ArmSide::kRight)});
  targets.setMode(TargetMode::kCircle, 0.0);
  DualArmController controller(robot, config);

  for (int step = 0; step < 600; ++step) {
    const ControllerDiagnostics result =
        controller.step(targets.sample(static_cast<double>(step) * kDt), kDt);
    ASSERT_TRUE(result.accepted) << "step=" << step << " reason=" << toString(result.hold_reason);
    for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
      const Vec7 q = robot.armPosition(side);
      const ArmLimits limits = robot.mapping(side).limits;
      EXPECT_TRUE(q.allFinite());
      EXPECT_GE((q - limits.lower_position).minCoeff(), config.joint_limits.margin_rad - 1e-9);
      EXPECT_GE((limits.upper_position - q).minCoeff(), config.joint_limits.margin_rad - 1e-9);
    }
  }
}

TEST(TrajectoryRegression, BothArmsConvergeToCombinedScriptedPose) {
  const std::filesystem::path root(TIANJI_PROJECT_SOURCE_DIR);
  const QpIkConfig config =
      loadConfig((root / "config" / "qp_ik_hierarchical.yaml").string());
  MujocoRobot robot((root / "models" / "marvin_m6_qp_test.xml").string());
  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    robot.setArmPosition(side, 0.5 * (robot.mapping(side).limits.lower_position +
                                     robot.mapping(side).limits.upper_position));
  }
  robot.forward();
  TargetManager targets(config,
                        {robot.tcpPose(ArmSide::kLeft), robot.tcpPose(ArmSide::kRight)});
  targets.setMode(TargetMode::kCombined, 0.0);
  DualArmController controller(robot, config);

  DualArmTargets scripted;
  for (int step = 0; step <= 500; ++step) {
    scripted = targets.sample(static_cast<double>(step) * kDt);
    ASSERT_TRUE(controller.step(scripted, kDt).accepted) << "script step=" << step;
  }
  for (int step = 0; step < 600; ++step) {
    ASSERT_TRUE(controller.step(scripted, kDt).accepted) << "settle step=" << step;
  }
  robot.forward();

  double maximum_position_error = 0.0;
  double maximum_orientation_error = 0.0;
  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    const Pose desired = side == ArmSide::kLeft ? scripted.left : scripted.right;
    const Pose actual = robot.tcpPose(side);
    const double position_error = (desired.position - actual.position).norm();
    const double orientation_error = rotationDistance(desired.rotation, actual.rotation);
    maximum_position_error = std::max(maximum_position_error, position_error);
    maximum_orientation_error = std::max(maximum_orientation_error, orientation_error);
    EXPECT_LT(position_error, 0.002) << toString(side);
    EXPECT_LT(orientation_error, 3.14159265358979323846 / 180.0) << toString(side);
  }
  std::cout << "combined_both_arm_max_position_error_m=" << maximum_position_error
            << " combined_both_arm_max_orientation_error_rad="
            << maximum_orientation_error << '\n';
}

}  // namespace
}  // namespace tianji_qp_ik
