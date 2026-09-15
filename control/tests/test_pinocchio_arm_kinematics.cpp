#include "tianji_qp_ik/pinocchio_arm_kinematics.hpp"
#include "tianji_qp_ik/so3.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace tianji_qp_ik {
namespace {

constexpr const char* kUrdfPath =
    TIANJI_PROJECT_SOURCE_DIR "/models/marvin_m6_s_ccs_696_v4_local.urdf";
constexpr const char* kMujocoModelPath =
    TIANJI_PROJECT_SOURCE_DIR "/models/marvin_m6_qp_test.xml";
constexpr const char* kWuji2MujocoModelPath =
    TIANJI_PROJECT_SOURCE_DIR "/models/marvin_m6_wuji2.xml";

void expectSamplesNear(const ArmKinematicSample& pinocchio_sample,
                       const ArmKinematicSample& mujoco_sample) {
  constexpr double kTolerance = 1.0e-5;
  EXPECT_TRUE(pinocchio_sample.tcp_pose.position.isApprox(
      mujoco_sample.tcp_pose.position, kTolerance));
  EXPECT_LT(rotationDistance(pinocchio_sample.tcp_pose.rotation,
                             mujoco_sample.tcp_pose.rotation),
            kTolerance);
  EXPECT_TRUE(pinocchio_sample.tcp_jacobian.isApprox(
      mujoco_sample.tcp_jacobian, kTolerance));
  EXPECT_TRUE(pinocchio_sample.shoulder_position.isApprox(
      mujoco_sample.shoulder_position, kTolerance));
  EXPECT_TRUE(pinocchio_sample.elbow_position.isApprox(
      mujoco_sample.elbow_position, kTolerance));
  EXPECT_TRUE(pinocchio_sample.wrist_position.isApprox(
      mujoco_sample.wrist_position, kTolerance));
  EXPECT_LT(rotationDistance(pinocchio_sample.shoulder_rotation,
                             mujoco_sample.shoulder_rotation), kTolerance);
  EXPECT_LT(rotationDistance(pinocchio_sample.elbow_rotation,
                             mujoco_sample.elbow_rotation), kTolerance);
  EXPECT_LT(rotationDistance(pinocchio_sample.wrist_rotation,
                             mujoco_sample.wrist_rotation), kTolerance);
  EXPECT_TRUE(pinocchio_sample.shoulder_position_jacobian.isApprox(
      mujoco_sample.shoulder_position_jacobian, kTolerance));
  EXPECT_TRUE(pinocchio_sample.elbow_position_jacobian.isApprox(
      mujoco_sample.elbow_position_jacobian, kTolerance));
  EXPECT_TRUE(pinocchio_sample.wrist_position_jacobian.isApprox(
      mujoco_sample.wrist_position_jacobian, kTolerance));
}

TEST(PinocchioArmKinematics, LoadsTianjiJointAndFrameMapping) {
  PinocchioArmKinematics kinematics(kUrdfPath);
  EXPECT_EQ(kinematics.configurationSize(), 14);
  EXPECT_EQ(kinematics.velocitySize(), 14);
  EXPECT_THROW(PinocchioArmKinematics("missing.urdf"), std::runtime_error);
}

TEST(PinocchioArmKinematics, MatchesMujocoAcrossBothArmWorkspaces) {
  PinocchioArmKinematics kinematics(kUrdfPath);
  MujocoRobot robot(kMujocoModelPath);
  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    const ArmLimits& limits = robot.mapping(side).limits;
    const Vec7 midpoint =
        0.5 * (limits.lower_position + limits.upper_position);
    const Vec7 half_range =
        0.5 * (limits.upper_position - limits.lower_position);
    for (int sample_index = 0; sample_index <= 20; ++sample_index) {
      Vec7 q = midpoint;
      if (sample_index != 0) {
        for (int joint = 0; joint < kArmDof; ++joint) {
          q[joint] += 0.35 * half_range[joint] *
                      std::sin(0.73 * static_cast<double>(sample_index) +
                               0.41 * static_cast<double>(joint));
        }
      }
      SCOPED_TRACE("side=" + toString(side) +
                   " sample=" + std::to_string(sample_index));
      expectSamplesNear(kinematics.sample(side, q),
                        robot.armKinematicsAt(side, q));
    }
  }
}

TEST(PinocchioArmKinematics, MatchesMujocoHandTcpAcrossBothArms) {
  MujocoRobot robot(kWuji2MujocoModelPath);
  const std::array<Pose, 2> tcp_relative_to_link7{
      robot.tcpRelativeToLink7(ArmSide::kLeft),
      robot.tcpRelativeToLink7(ArmSide::kRight)};
  PinocchioArmKinematics kinematics(kUrdfPath, tcp_relative_to_link7);

  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    const ArmLimits& limits = robot.mapping(side).limits;
    const Vec7 midpoint =
        0.5 * (limits.lower_position + limits.upper_position);
    const Vec7 half_range = 0.5 * (limits.upper_position - limits.lower_position);
    for (int sample_index = 0; sample_index <= 20; ++sample_index) {
      Vec7 q = midpoint;
      if (sample_index != 0) {
        for (int joint = 0; joint < kArmDof; ++joint) {
          q[joint] += 0.35 * half_range[joint] *
                      std::sin(0.73 * static_cast<double>(sample_index) +
                               0.41 * static_cast<double>(joint));
        }
      }
      SCOPED_TRACE("side=" + toString(side) +
                   " sample=" + std::to_string(sample_index));
      expectSamplesNear(kinematics.sample(side, q),
                        robot.armKinematicsAt(side, q));
    }
  }
}

TEST(PinocchioArmKinematics, RejectsNonFiniteJointPosition) {
  PinocchioArmKinematics kinematics(kUrdfPath);
  Vec7 invalid = Vec7::Zero();
  invalid[3] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(kinematics.sample(ArmSide::kLeft, invalid),
               std::invalid_argument);
}

}  // namespace
}  // namespace tianji_qp_ik
