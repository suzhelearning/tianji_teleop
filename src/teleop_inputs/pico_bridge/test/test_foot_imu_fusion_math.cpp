#include <gtest/gtest.h>
#include <cmath>
#include "pico_bridge/foot_imu_fusion_math.hpp"

namespace fusion = pico_bridge::fusion;

TEST(FootImuFusionMath, RejectsZeroQuaternion) {
  fusion::Quaternion out{};
  EXPECT_FALSE(fusion::try_normalize({0, 0, 0, 0}, out));
}

TEST(FootImuFusionMath, NeutralCalibrationPreservesPicoFootPose) {
  const fusion::Quaternion pelvis0{0, 0, std::sin(M_PI/8), std::cos(M_PI/8)};
  const fusion::Quaternion imu0{0, 0, std::sin(M_PI/5), std::cos(M_PI/5)};
  const fusion::Quaternion foot0{0, std::sin(M_PI/12), 0, std::cos(M_PI/12)};
  const auto foot = fusion::aligned_foot_orientation(pelvis0, imu0, pelvis0, imu0, foot0);
  for (size_t i = 0; i < 4; ++i) EXPECT_NEAR(foot[i], foot0[i], 1e-9);
}

TEST(FootImuFusionMath, NeutralOffsetPreservesPicoFootPosition) {
  const fusion::Quaternion foot0{0, std::sin(M_PI/12), 0, std::cos(M_PI/12)};
  const fusion::Vector3 ankle{1, 2, 3};
  const fusion::Vector3 original_foot{1.2, 2.1, 2.95};
  const fusion::Vector3 world_offset{0.2, 0.1, -0.05};
  const auto local = fusion::rotate(fusion::inverse(foot0), world_offset);
  const auto rebuilt = fusion::reconstruct_foot_position(ankle, foot0, local);
  for (size_t i = 0; i < 3; ++i) EXPECT_NEAR(rebuilt[i], original_foot[i], 1e-9);
}

TEST(FootImuFusionMath, RelativeAnkleUsesShankAndFoot) {
  const fusion::Quaternion shank{0, 0, 0, 1};
  const fusion::Quaternion foot{std::sin(M_PI/8), 0, 0, std::cos(M_PI/8)};
  const auto ankle = fusion::relative_ankle_orientation(shank, foot);
  for (size_t i = 0; i < 4; ++i) EXPECT_NEAR(ankle[i], foot[i], 1e-9);
}

TEST(FootImuFusionMath, AngularDistanceHandlesQuaternionSign) {
  EXPECT_NEAR(fusion::angular_distance({0,0,0,1}, {0,0,0,-1}), 0.0, 1e-9);
}

TEST(FootImuFusionMath, CalibrationStepRequiresStableFeet) {
  const fusion::Quaternion neutral{0, 0, 0, 1};
  const fusion::Quaternion moved{0, 0, std::sin(0.1), std::cos(0.1)};
  EXPECT_TRUE(fusion::calibration_step_is_stable(
    neutral, neutral, neutral, neutral, neutral, neutral, neutral, neutral, neutral, neutral, 0.2));
  EXPECT_FALSE(fusion::calibration_step_is_stable(
    neutral, neutral, neutral, moved, neutral, neutral, neutral, neutral, neutral, neutral, 0.05));
}
