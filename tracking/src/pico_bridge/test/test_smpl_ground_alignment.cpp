#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "pico_bridge/smpl_ground_alignment.hpp"

namespace pb = pico_bridge;

namespace {

std::vector<pb::SmplPose> skeleton(double sole_height = -1.6) {
  std::vector<pb::SmplPose> poses(24);
  for (auto & pose : poses) pose.orientation.w = 1.0;
  poses[0].position.z = -0.8;
  poses[10].position.z = sole_height + 0.022;
  poses[11].position.z = sole_height + 0.022;
  poses[15].position.z = 0.0;
  return poses;
}

pb::GroundAlignmentOptions options() {
  pb::GroundAlignmentOptions value;
  value.foot_half_extents = {0.115, 0.050, 0.022};
  value.stable_window_frames = 30;
  value.stability_tolerance_m = 0.02;
  value.require_world_reset = true;
  return value;
}

}  // namespace

TEST(SmplGroundAlignment, OrientedSoleHeightUsesFootRotation) {
  auto pose = skeleton()[10];
  EXPECT_NEAR(pb::foot_sole_height(pose, options().foot_half_extents), -1.6, 1e-12);

  const double half_angle = M_PI / 4.0;
  pose.orientation = {0.0, std::sin(half_angle), 0.0, std::cos(half_angle)};
  EXPECT_NEAR(
    pb::foot_sole_height(pose, options().foot_half_extents),
    pose.position.z - 0.115, 1e-12);
}

TEST(SmplGroundAlignment, DoesNotLockBeforeWorldReset) {
  pb::SmplGroundAlignment alignment(options());
  for (int index = 0; index < 60; ++index) {
    EXPECT_FALSE(alignment.observe(skeleton()));
  }
  EXPECT_FALSE(alignment.locked());
}

TEST(SmplGroundAlignment, LocksOnThirtyStablePostResetFrames) {
  pb::SmplGroundAlignment alignment(options());
  alignment.reset();
  for (int index = 0; index < 29; ++index) {
    EXPECT_FALSE(alignment.observe(skeleton(-1.600 + index * 0.0001)));
  }
  EXPECT_TRUE(alignment.observe(skeleton(-1.598)));
  ASSERT_TRUE(alignment.locked());
  ASSERT_TRUE(alignment.floor_height().has_value());
  EXPECT_NEAR(*alignment.floor_height(), -1.599, 0.002);
}

TEST(SmplGroundAlignment, RejectsUnstableAndInvalidFootWindows) {
  pb::SmplGroundAlignment alignment(options());
  alignment.reset();
  for (int index = 0; index < 29; ++index) {
    EXPECT_FALSE(alignment.observe(skeleton(-1.6)));
  }
  EXPECT_FALSE(alignment.observe(skeleton(-1.5)));
  EXPECT_FALSE(alignment.locked());

  auto invalid = skeleton();
  invalid[10].orientation = {};
  EXPECT_FALSE(alignment.observe(invalid));
  EXPECT_FALSE(alignment.locked());
}

TEST(SmplGroundAlignment, TranslatesEveryJointOnlyAlongWorldZ) {
  pb::SmplGroundAlignment alignment(options());
  alignment.reset();
  auto input = skeleton();
  input[0].position.x = 0.4;
  input[0].position.y = -0.2;
  input[0].orientation = {0.1, 0.2, 0.3, 0.9};
  for (int index = 0; index < 30; ++index) ASSERT_EQ(alignment.observe(input), index == 29);

  const auto output = alignment.transform(input);

  ASSERT_EQ(output.size(), input.size());
  EXPECT_DOUBLE_EQ(output[0].position.x, input[0].position.x);
  EXPECT_DOUBLE_EQ(output[0].position.y, input[0].position.y);
  EXPECT_NEAR(output[0].position.z, 0.8, 1e-12);
  EXPECT_EQ(output[0].orientation, input[0].orientation);
  EXPECT_NEAR(pb::foot_sole_height(output[10], options().foot_half_extents), 0.0, 1e-12);
  EXPECT_NEAR(pb::foot_sole_height(output[11], options().foot_half_extents), 0.0, 1e-12);
  EXPECT_NEAR(output[15].position.z, 1.6, 1e-12);
}

TEST(SmplGroundAlignment, ResetClearsLockedFloorAndStopsTransform) {
  pb::SmplGroundAlignment alignment(options());
  alignment.reset();
  for (int index = 0; index < 30; ++index) alignment.observe(skeleton());
  ASSERT_TRUE(alignment.locked());

  alignment.reset();

  EXPECT_FALSE(alignment.locked());
  EXPECT_FALSE(alignment.floor_height().has_value());
  EXPECT_THROW(alignment.transform(skeleton()), std::logic_error);
}

TEST(SmplGroundAlignment, RejectsInvalidFramesAfterFloorIsLocked) {
  pb::SmplGroundAlignment alignment(options());
  alignment.reset();
  for (int index = 0; index < 30; ++index) alignment.observe(skeleton());
  ASSERT_TRUE(alignment.locked());
  auto invalid = skeleton();
  invalid[3].position.z = std::numeric_limits<double>::quiet_NaN();

  EXPECT_FALSE(alignment.observe(invalid));
  EXPECT_TRUE(alignment.locked());
}
