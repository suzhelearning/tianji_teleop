#include "tianji_qp_ik/config.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <unistd.h>
#include <utility>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace tianji_qp_ik {
namespace {

std::string projectConfigText() {
  const std::filesystem::path source =
      std::filesystem::path(TIANJI_PROJECT_SOURCE_DIR) / "config" / "qp_ik_pico_shared_root_dls.yaml";
  std::ifstream input(source);
  auto node = YAML::Load(input);
  node["controller"].remove("initial_left_q_rad");
  node["controller"].remove("initial_right_q_rad");
  node["controller"].remove("home_config");
  node["controller"].remove("initial_posture_enabled");
  return YAML::Dump(node);
}

std::filesystem::path writeTemporaryConfig(const std::string& name, const std::string& text) {
  const std::filesystem::path path =
      std::filesystem::temp_directory_path() /
      (std::to_string(::getpid()) + "_" + name);
  std::ofstream output(path);
  output << text;
  return path;
}

TEST(Config, ResolvesHomeRelativeToControllerConfigAndEnablesPosture) {
  const auto home = writeTemporaryConfig(
      "relative_home.yaml",
      "left_home_rad: [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7]\n"
      "right_home_rad: [-0.1, 0.2, -0.3, 0.4, -0.5, 0.6, -0.7]\n");
  auto node = YAML::Load(projectConfigText());
  node["controller"]["home_config"] = home.filename().string();
  const auto path = writeTemporaryConfig("relative_home_controller.yaml", YAML::Dump(node));
  const auto config = loadConfig(path.string());
  ArmLimits limits;
  limits.lower_position = Vec7::Constant(-2.0);
  limits.upper_position = Vec7::Constant(2.0);
  const Vec7 expected =
      (Vec7() << 0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7).finished();

  EXPECT_TRUE(configuredInitialPosture(config.controller, limits, ArmSide::kLeft)
                  .isApprox(expected));
  EXPECT_TRUE(configuredInitialPosture(config.controller, limits, ArmSide::kRight)
                  .isApprox(-expected));
  std::filesystem::remove(path);
  std::filesystem::remove(home);
}

TEST(Config, MeasuredPostureOverridesHomeEvenWhenHomeFileIsUnavailable) {
  const auto home = writeTemporaryConfig(
      "overridden_home.yaml",
      "left_home_rad: [0, 0, 0, 0, 0, 0, 0]\n"
      "right_home_rad: [0, 0, 0, 0, 0, 0, 0]\n");
  auto node = YAML::Load(projectConfigText());
  node["controller"]["initial_posture_enabled"] = true;
  node["controller"]["initial_left_q_rad"] = YAML::Load("[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]");
  node["controller"]["initial_right_q_rad"] = YAML::Load("[-0.1, -0.2, -0.3, -0.4, -0.5, -0.6, -0.7]");
  node["controller"]["home_config"] = home.filename().string();
  const auto path = writeTemporaryConfig("measured_home_controller.yaml", YAML::Dump(node));
  ArmLimits limits;
  limits.lower_position = Vec7::Constant(-2.0);
  limits.upper_position = Vec7::Constant(2.0);
  const Vec7 expected =
      (Vec7() << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7).finished();
  const auto config = loadConfig(path.string());
  EXPECT_TRUE(configuredInitialPosture(config.controller, limits, ArmSide::kLeft)
                  .isApprox(expected));
  EXPECT_TRUE(configuredInitialPosture(config.controller, limits, ArmSide::kRight)
                  .isApprox(-expected));

  std::filesystem::remove(home);
  const auto without_home = loadConfig(path.string());
  EXPECT_TRUE(configuredInitialPosture(without_home.controller, limits,
                                      ArmSide::kLeft).isApprox(expected));
  EXPECT_TRUE(configuredInitialPosture(without_home.controller, limits,
                                      ArmSide::kRight).isApprox(-expected));
  std::filesystem::remove(path);
}

TEST(Config, RejectsPartialMeasuredPostureRatherThanBlendingWithHome) {
  const auto home = writeTemporaryConfig(
      "partial_override_home.yaml",
      "left_home_rad: [0, 0, 0, 0, 0, 0, 0]\n"
      "right_home_rad: [0, 0, 0, 0, 0, 0, 0]\n");
  auto node = YAML::Load(projectConfigText());
  node["controller"]["initial_posture_enabled"] = true;
  node["controller"]["initial_left_q_rad"] = YAML::Load("[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]");
  node["controller"]["home_config"] = home.filename().string();
  const auto path = writeTemporaryConfig("partial_home_controller.yaml", YAML::Dump(node));

  EXPECT_THROW(loadConfig(path.string()), std::runtime_error);
  std::filesystem::remove(path);
  std::filesystem::remove(home);
}

TEST(Config, RejectsMalformedInitialPostureVector) {
  auto node = YAML::Load(projectConfigText());
  node["controller"]["initial_posture_enabled"] = true;
  node["controller"]["initial_left_q_rad"] = YAML::Load("[0.0, 0.0]");
  node["controller"]["initial_right_q_rad"] = YAML::Load("[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]");
  const std::filesystem::path path =
      writeTemporaryConfig("malformed_initial_posture.yaml", YAML::Dump(node));

  EXPECT_THROW(loadConfig(path.string()), std::runtime_error);
  std::filesystem::remove(path);
}

TEST(Config, RejectsInitialPostureOutsideNamedArmJointLimit) {
  ControllerConfig controller;
  controller.initial_posture_enabled = true;
  controller.initial_left_q_rad.setZero();
  controller.initial_right_q_rad.setZero();
  controller.initial_left_q_rad[2] = 1.1;
  ArmLimits limits;
  limits.lower_position = Vec7::Constant(-1.0);
  limits.upper_position = Vec7::Constant(1.0);

  EXPECT_THROW(configuredInitialPosture(controller, limits, ArmSide::kLeft),
               std::runtime_error);
}

TEST(Config, RejectsNonPositiveControlRate) {
  auto node = YAML::Load(projectConfigText());
  node["controller"]["rate_hz"] = 0.0;
  const auto path = writeTemporaryConfig("tianji_invalid_qp_ik.yaml", YAML::Dump(node));
  EXPECT_THROW(loadConfig(path.string()), std::runtime_error);
  std::filesystem::remove(path);
}

TEST(Config, RejectsUnknownIkAlgorithm) {
  auto node = YAML::Load(projectConfigText());
  node["ik"]["algorithm"] = "unknown";
  const auto path = writeTemporaryConfig("tianji_unknown_ik.yaml", YAML::Dump(node));
  EXPECT_THROW(loadConfig(path.string()), std::runtime_error);
  std::filesystem::remove(path);
}

TEST(Config, RejectsNonPositivePicoTeleopJumpLimits) {
  const std::vector<std::pair<std::string, std::string>> limits{
      {"0.0", "0.60"},
      {"0.15", "-0.1"},
  };
  for (const auto& [position_limit, orientation_limit] : limits) {
    auto node = YAML::Load(projectConfigText());
    node["pico_teleop"]["max_position_jump_m"] = position_limit;
    node["pico_teleop"]["max_orientation_jump_rad"] = orientation_limit;
    const auto path = writeTemporaryConfig("tianji_invalid_pico_teleop.yaml", YAML::Dump(node));
    EXPECT_THROW(loadConfig(path.string()), std::runtime_error);
    std::filesystem::remove(path);
  }
}

}  // namespace
}  // namespace tianji_qp_ik
