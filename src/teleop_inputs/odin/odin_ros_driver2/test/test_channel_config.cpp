#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "utility/channel_config.hpp"

namespace driver = odin_ros_driver;

TEST(ChannelConfig, OdomProfileEnablesOnlyOdometry) {
  const auto config = driver::load_channel_enable_config(
    std::string(TEST_CONFIG_DIR) + "/control_command_odom.yaml");

  EXPECT_FALSE(config.raw_point);
  EXPECT_FALSE(config.slam_point);
  EXPECT_FALSE(config.image0);
  EXPECT_FALSE(config.image1);
  EXPECT_FALSE(config.imu);
  EXPECT_TRUE(config.odom);
}

TEST(ChannelConfig, MissingFileFailsClosed) {
  EXPECT_THROW(
    driver::load_channel_enable_config("/definitely/missing/odin-profile.yaml"),
    std::runtime_error);
}

TEST(ChannelConfig, EmptyProfileSelectionFailsClosed) {
  EXPECT_THROW(
    driver::resolve_channel_config_path("", TEST_CONFIG_DIR),
    std::runtime_error);
}

TEST(ChannelConfig, ResolvesRelativeAndAbsoluteProfileSelections) {
  EXPECT_EQ(
    driver::resolve_channel_config_path(
      "control_command_odom.yaml", "/opt/share/odin_ros_driver_rev1"),
    "/opt/share/odin_ros_driver_rev1/config/control_command_odom.yaml");
  EXPECT_EQ(
    driver::resolve_channel_config_path(
      "/tmp/odin-profile.yaml", "/opt/share/odin_ros_driver_rev1"),
    "/tmp/odin-profile.yaml");
}

TEST(ChannelConfig, ExplicitProfilesSelectTheirExpectedChannels) {
  const auto full = driver::load_channel_enable_config(
    std::string(TEST_CONFIG_DIR) + "/control_command.yaml");
  const auto slam = driver::load_channel_enable_config(
    std::string(TEST_CONFIG_DIR) + "/control_command_slam.yaml");
  const auto odom = driver::load_channel_enable_config(
    std::string(TEST_CONFIG_DIR) + "/control_command_odom.yaml");

  EXPECT_TRUE(full.raw_point);
  EXPECT_TRUE(full.slam_point);
  EXPECT_TRUE(full.imu);
  EXPECT_TRUE(full.odom);
  EXPECT_TRUE(slam.raw_point);
  EXPECT_TRUE(slam.slam_point);
  EXPECT_TRUE(slam.imu);
  EXPECT_TRUE(slam.odom);
  EXPECT_FALSE(odom.raw_point);
  EXPECT_FALSE(odom.slam_point);
  EXPECT_FALSE(odom.imu);
  EXPECT_TRUE(odom.odom);
}

TEST(ChannelConfig, MissingRequiredChannelFailsClosed) {
  const YAML::Node root = YAML::Load(R"(
register_keys:
  enable_raw_point: 0
  enable_slam_point: 0
  enable_image0: 0
  enable_image1: 0
  enable_imu: 0
)");

  EXPECT_THROW(driver::parse_channel_enable_config(root), std::runtime_error);
}

TEST(ChannelConfig, NonBooleanChannelValueFailsClosed) {
  const YAML::Node root = YAML::Load(R"(
register_keys:
  enable_raw_point: 0
  enable_slam_point: 0
  enable_image0: 0
  enable_image1: 0
  enable_imu: 0
  enable_odom: 2
)");

  EXPECT_THROW(driver::parse_channel_enable_config(root), std::runtime_error);
}
