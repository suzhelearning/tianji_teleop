#include "tianji_qp_ik/mujoco_robot.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <random>
#include <string>

namespace tianji_qp_ik {
namespace {

std::string modelPath() {
  return (std::filesystem::path(TIANJI_PROJECT_SOURCE_DIR) / "models" /
          "marvin_m6_qp_test.xml")
      .string();
}

TEST(MujocoJdotQdot, MatchesIndependentCenteredDirectionalDerivative) {
  MujocoRobot robot(modelPath());
  std::mt19937 generator(20260810U);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  constexpr double kEpsilon = 1e-6;

  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    const ArmLimits limits = robot.mapping(side).limits;
    for (int sample = 0; sample < 12; ++sample) {
      Vec7 q;
      Vec7 qdot;
      for (int joint = 0; joint < kArmDof; ++joint) {
        const double lower = limits.lower_position[joint] + 0.2;
        const double upper = limits.upper_position[joint] - 0.2;
        q[joint] = lower + 0.5 * (unit(generator) + 1.0) * (upper - lower);
        qdot[joint] = 0.4 * unit(generator);
      }
      const Vec6 model =
          robot.tcpJacobianDotTimesVelocityWorld(side, q, qdot);

      robot.setArmPosition(side, q + kEpsilon * qdot);
      robot.forward();
      const Vec6 plus = robot.tcpJacobianWorld(side) * qdot;
      robot.setArmPosition(side, q - kEpsilon * qdot);
      robot.forward();
      const Vec6 minus = robot.tcpJacobianWorld(side) * qdot;
      const Vec6 numeric = (plus - minus) / (2.0 * kEpsilon);

      EXPECT_LT((model - numeric).norm(), 1e-5);
    }
  }
}

}  // namespace
}  // namespace tianji_qp_ik
