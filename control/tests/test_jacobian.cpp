#include "tianji_qp_ik/mujoco_robot.hpp"
#include "tianji_qp_ik/so3.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <random>
#include <string>

namespace tianji_qp_ik {
namespace {

std::string modelPath() {
  return (std::filesystem::path(TIANJI_PROJECT_SOURCE_DIR) / "models" /
          "marvin_m6_qp_test.xml")
      .string();
}

TEST(MujocoJacobian, MatchesCentralFiniteDifferenceForBothArms) {
  MujocoRobot robot(modelPath());
  std::mt19937 generator(20260809U);
  constexpr double kEpsilon = 1e-6;
  double maximum_error = 0.0;

  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    const ArmLimits limits = robot.mapping(side).limits;
    for (int sample = 0; sample < 20; ++sample) {
      Vec7 q;
      for (int joint = 0; joint < kArmDof; ++joint) {
        const double margin = std::min(0.15, 0.2 * (limits.upper_position[joint] -
                                                   limits.lower_position[joint]));
        std::uniform_real_distribution<double> distribution(limits.lower_position[joint] + margin,
                                                            limits.upper_position[joint] - margin);
        q[joint] = distribution(generator);
      }

      robot.setArmPosition(side, q);
      robot.forward();
      const Mat67 analytic = robot.tcpJacobianWorld(side);

      for (int joint = 0; joint < kArmDof; ++joint) {
        Vec7 plus_q = q;
        plus_q[joint] += kEpsilon;
        robot.setArmPosition(side, plus_q);
        robot.forward();
        const Pose plus = robot.tcpPose(side);

        Vec7 minus_q = q;
        minus_q[joint] -= kEpsilon;
        robot.setArmPosition(side, minus_q);
        robot.forward();
        const Pose minus = robot.tcpPose(side);

        const Eigen::Vector3d position_fd =
            (plus.position - minus.position) / (2.0 * kEpsilon);
        const Eigen::Vector3d rotation_fd =
            so3Log(plus.rotation * minus.rotation.transpose()) / (2.0 * kEpsilon);
        maximum_error =
            std::max(maximum_error,
                     (analytic.topRows<3>().col(joint) - position_fd).cwiseAbs().maxCoeff());
        maximum_error =
            std::max(maximum_error,
                     (analytic.bottomRows<3>().col(joint) - rotation_fd).cwiseAbs().maxCoeff());
      }
      robot.setArmPosition(side, q);
      robot.forward();
    }
  }

  std::cout << "jacobian_seed=20260809 maximum_finite_difference_error=" << maximum_error
            << '\n';
  EXPECT_LT(maximum_error, 1e-5) << "maximum finite-difference error=" << maximum_error;
}

}  // namespace
}  // namespace tianji_qp_ik
