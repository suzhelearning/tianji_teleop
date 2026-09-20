#include "tianji_qp_ik/config.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <unistd.h>
#include <utility>
#include <vector>

namespace tianji_qp_ik {
namespace {

std::string projectConfigText() {
  const std::filesystem::path source =
      std::filesystem::path(TIANJI_PROJECT_SOURCE_DIR) / "config" / "qp_ik.yaml";
  std::ifstream input(source);
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

void replaceOnce(std::string& text, const std::string& from, const std::string& to) {
  const std::size_t offset = text.find(from);
  ASSERT_NE(offset, std::string::npos);
  text.replace(offset, from.size(), to);
}

std::filesystem::path writeTemporaryConfig(const std::string& name, const std::string& text) {
  const std::filesystem::path path =
      std::filesystem::temp_directory_path() /
      (std::to_string(::getpid()) + "_" + name);
  std::ofstream output(path);
  output << text;
  return path;
}

TEST(Config, LoadsProjectDefaults) {
  const std::filesystem::path path =
      std::filesystem::path(TIANJI_PROJECT_SOURCE_DIR) / "config" / "qp_ik.yaml";
  const QpIkConfig config = loadConfig(path.string());
  EXPECT_DOUBLE_EQ(config.controller.rate_hz, 1000.0);
  EXPECT_FALSE(config.controller.model_state_only);
  EXPECT_EQ(config.qp.solver, SolverBackend::kQpoases);
  EXPECT_DOUBLE_EQ(config.joint_limits.velocity_scale, 0.20);
  EXPECT_DOUBLE_EQ(config.qpoases.cpu_time_limit_seconds, 0.00035);
  EXPECT_DOUBLE_EQ(config.safety.bound_tolerance, 1e-8);
  EXPECT_FALSE(config.cartesian_otg.enabled);
  EXPECT_FALSE(config.cartesian_otg.translation_position_mode);
  EXPECT_EQ(config.control_level, ControlLevel::kVelocity);
  EXPECT_DOUBLE_EQ(config.pico_teleop.max_position_jump_m, 0.15);
  EXPECT_DOUBLE_EQ(config.pico_teleop.max_orientation_jump_rad, 0.60);
  EXPECT_TRUE(config.arm_angle.enabled);
  EXPECT_DOUBLE_EQ(config.arm_angle.kp_velocity, 32.0);
  EXPECT_DOUBLE_EQ(config.arm_angle.reference_rate_limit_rad_s, 8.0);
  EXPECT_DOUBLE_EQ(config.arm_angle.reference_projection_hold_enter, 0.05);
  EXPECT_DOUBLE_EQ(config.arm_angle.reference_projection_hold_exit, 0.10);
  EXPECT_DOUBLE_EQ(config.arm_angle.branch_lock_radius_m, 0.015);
  EXPECT_DOUBLE_EQ(config.arm_angle.error_branch_hysteresis_rad, 0.20);
  EXPECT_FALSE(config.arm_angle.reference_governor_enabled);
  EXPECT_DOUBLE_EQ(config.arm_angle.reference_governor_enter_error_rad, 1.0);
  EXPECT_DOUBLE_EQ(config.arm_angle.reference_governor_exit_error_rad, 0.5);
  EXPECT_DOUBLE_EQ(config.arm_angle.reference_governor_tracking_error_rad,
                   0.15);
  EXPECT_DOUBLE_EQ(config.arm_angle.joint_limit_soft_margin_rad, 0.30);
  EXPECT_DOUBLE_EQ(
      config.arm_angle.joint_limit_recovery_gain_rad_s_per_rad, 2.0);
  EXPECT_FALSE(config.arm_angle.nullspace_only);
  EXPECT_DOUBLE_EQ(config.arm_angle.continuity_kp_velocity, 2.0);
  EXPECT_DOUBLE_EQ(config.arm_angle.continuity_max_velocity_rad_s, 0.8);
  EXPECT_DOUBLE_EQ(config.arm_angle.continuity_kp_acceleration, 8.0);
  EXPECT_DOUBLE_EQ(config.arm_angle.continuity_kd_acceleration, 4.0);
  EXPECT_DOUBLE_EQ(config.arm_angle.continuity_max_acceleration_rad_s2, 8.0);
  EXPECT_DOUBLE_EQ(config.arm_angle.continuity_weight_scale, 30.0);
  EXPECT_DOUBLE_EQ(config.acceleration_qp.arm_angle_weight, 10.0);
  EXPECT_DOUBLE_EQ(config.upper_arm_outward.minimum_outward_distance_m, 0.0);
  EXPECT_DOUBLE_EQ(config.upper_arm_outward.velocity_gain, 8.0);
  EXPECT_DOUBLE_EQ(config.upper_arm_outward.acceleration_kp, 100.0);
  EXPECT_DOUBLE_EQ(config.upper_arm_outward.acceleration_kd, 20.0);
}

TEST(Config, SharedRootDisabledExperimentLoadsAndEnabledCannotSilentlyUseLegacy) {
  EXPECT_NO_THROW(loadConfig(std::string(TIANJI_PROJECT_SOURCE_DIR) +
                            "/config/qp_ik_pico_shared_root.yaml"));
  const auto enabled = writeTemporaryConfig("shared_root_enabled.yaml",
      projectConfigText() + "\nspark_shared_root:\n  enabled: true\n");
  EXPECT_THROW(loadConfig(enabled.string()), std::runtime_error);
  std::filesystem::remove(enabled);
  const auto mixed = writeTemporaryConfig("shared_root_mix.yaml",
      projectConfigText() + "\nspark_shared_root:\n  enabled: false\n  mix: 0.5\n");
  EXPECT_THROW(loadConfig(mixed.string()), std::runtime_error);
  std::filesystem::remove(mixed);
}

TEST(Config, LoadsAndNamesSparkDirectVelocityQp) {
  std::string text = projectConfigText();
  replaceOnce(text, "algorithm: hierarchical_qp",
              "algorithm: spark_direct_velocity_qp");
  const std::filesystem::path path =
      writeTemporaryConfig("spark_direct_velocity_qp.yaml", text);

  const QpIkConfig config = loadConfig(path.string());

  EXPECT_EQ(config.ik_algorithm, IkAlgorithm::kSparkDirectVelocityQp);
  EXPECT_EQ(toString(config.ik_algorithm), "spark_direct_velocity_qp");
  EXPECT_TRUE(usesSparkVelocityQp(config.ik_algorithm));
  EXPECT_TRUE(usesHierarchicalVelocityQp(config.ik_algorithm));
  std::filesystem::remove(path);
}

TEST(Config, LoadsAndNamesSparkPoseVelocityQp) {
  std::string text = projectConfigText();
  replaceOnce(text, "algorithm: hierarchical_qp",
              "algorithm: spark_pose_velocity_qp");
  const std::filesystem::path path =
      writeTemporaryConfig("spark_pose_velocity_qp.yaml", text);

  const QpIkConfig config = loadConfig(path.string());

  EXPECT_EQ(config.ik_algorithm, IkAlgorithm::kSparkPoseVelocityQp);
  EXPECT_EQ(toString(config.ik_algorithm), "spark_pose_velocity_qp");
  EXPECT_TRUE(usesSparkVelocityQp(config.ik_algorithm));
  EXPECT_TRUE(usesHierarchicalVelocityQp(config.ik_algorithm));
  std::filesystem::remove(path);
}

TEST(Config, LoadsAndNamesSparkUpperQpoasesDirect) {
  std::string text = projectConfigText();
  replaceOnce(text, "algorithm: hierarchical_qp",
              "algorithm: spark_upper_qpoases_direct");
  const std::filesystem::path path =
      writeTemporaryConfig("spark_upper_qpoases_direct.yaml", text);

  const QpIkConfig config = loadConfig(path.string());

  EXPECT_EQ(config.ik_algorithm, IkAlgorithm::kSparkUpperQpoasesDirect);
  EXPECT_EQ(toString(config.ik_algorithm), "spark_upper_qpoases_direct");
  EXPECT_TRUE(usesSparkUpperQpoases(config.ik_algorithm));
  EXPECT_TRUE(usesSparkUpperQpoasesDirect(config.ik_algorithm));
  EXPECT_FALSE(usesSparkVelocityQp(config.ik_algorithm));
  std::filesystem::remove(path);
}

TEST(Config, LoadsAndNamesSparkUpperQpoasesVelocityQp) {
  std::string text = projectConfigText();
  replaceOnce(text, "algorithm: hierarchical_qp",
              "algorithm: spark_upper_qpoases_velocity_qp");
  const std::filesystem::path path =
      writeTemporaryConfig("spark_upper_qpoases_velocity_qp.yaml", text);

  const QpIkConfig config = loadConfig(path.string());

  EXPECT_EQ(config.ik_algorithm,
            IkAlgorithm::kSparkUpperQpoasesVelocityQp);
  EXPECT_EQ(toString(config.ik_algorithm),
            "spark_upper_qpoases_velocity_qp");
  EXPECT_TRUE(usesSparkUpperQpoases(config.ik_algorithm));
  EXPECT_FALSE(usesSparkUpperQpoasesDirect(config.ik_algorithm));
  EXPECT_TRUE(usesSparkVelocityQp(config.ik_algorithm));
  EXPECT_TRUE(usesHierarchicalVelocityQp(config.ik_algorithm));
  std::filesystem::remove(path);
}

TEST(Config, LoadsAndNamesSparkUpperQpoasesCartesianOtgVelocityQp) {
  std::string text = projectConfigText();
  replaceOnce(text, "algorithm: hierarchical_qp",
              "algorithm: spark_upper_qpoases_cartesian_otg_velocity_qp");
  const std::filesystem::path path = writeTemporaryConfig(
      "spark_upper_qpoases_cartesian_otg_velocity_qp.yaml", text);

  const QpIkConfig config = loadConfig(path.string());

  EXPECT_EQ(config.ik_algorithm,
            IkAlgorithm::kSparkUpperQpoasesCartesianOtgVelocityQp);
  EXPECT_EQ(toString(config.ik_algorithm),
            "spark_upper_qpoases_cartesian_otg_velocity_qp");
  EXPECT_TRUE(usesSparkGuidance(config.ik_algorithm));
  EXPECT_TRUE(usesSparkUpperQpoases(config.ik_algorithm));
  EXPECT_TRUE(usesSparkVelocityQp(config.ik_algorithm));
  EXPECT_TRUE(usesHierarchicalVelocityQp(config.ik_algorithm));
  EXPECT_TRUE(usesSparkOtgConsistentVelocityQp(config.ik_algorithm));
  EXPECT_FALSE(usesSparkUpperQpoasesDirect(config.ik_algorithm));
  std::filesystem::remove(path);
}

TEST(Config, LoadsAndNamesSparkUpperQpoasesFeedforwardVelocityQp) {
  std::string text = projectConfigText();
  replaceOnce(text, "algorithm: hierarchical_qp",
              "algorithm: spark_upper_qpoases_feedforward_velocity_qp");
  text += R"(
spark_feedforward_velocity_qp:
  alpha: 0.65
  beta: 0.12
  dt_median_window: 31
  dt_min_ratio: 0.5
  dt_max_ratio: 1.5
  maximum_joint_jump_rad: 0.35
  stale_velocity_decay_seconds: 0.02
  source_stationary_velocity_rad_s: 0.05
  velocity_reversal_decay: 0.10
  velocity_stationary_decay: 0.25
  target_braking_acceleration_scale: 0.75
  palm_twist_filter_alpha: 0.90
  palm_twist_lowpass_cutoff_hz: 0.80
  reference_velocity_scale: 1.0
  reference_acceleration_scale: 1.0
  reference_jerk_scale: 1.0
  position_feedforward_gain: 1.0
  orientation_feedforward_gain: 1.0
  position_high_frequency_feedforward_gain: 0.75
  orientation_high_frequency_feedforward_gain: 0.65
  joint_position_gain: 4.0
  attack_seconds: 0.08
  release_seconds: 0.05
)";
  const std::filesystem::path path = writeTemporaryConfig(
      "spark_upper_qpoases_feedforward_velocity_qp.yaml", text);

  const QpIkConfig config = loadConfig(path.string());

  EXPECT_EQ(config.ik_algorithm,
            IkAlgorithm::kSparkUpperQpoasesFeedforwardVelocityQp);
  EXPECT_EQ(toString(config.ik_algorithm),
            "spark_upper_qpoases_feedforward_velocity_qp");
  EXPECT_TRUE(usesSparkGuidance(config.ik_algorithm));
  EXPECT_TRUE(usesSparkUpperQpoases(config.ik_algorithm));
  EXPECT_TRUE(usesSparkVelocityQp(config.ik_algorithm));
  EXPECT_TRUE(usesHierarchicalVelocityQp(config.ik_algorithm));
  EXPECT_TRUE(usesSparkFeedforwardVelocityQp(config.ik_algorithm));
  EXPECT_FALSE(usesSparkOtgConsistentVelocityQp(config.ik_algorithm));
  EXPECT_FALSE(usesSparkUpperQpoasesDirect(config.ik_algorithm));
  EXPECT_DOUBLE_EQ(config.spark_feedforward_velocity_qp.alpha, 0.65);
  EXPECT_DOUBLE_EQ(config.spark_feedforward_velocity_qp.beta, 0.12);
  EXPECT_EQ(config.spark_feedforward_velocity_qp.dt_median_window, 31);
  EXPECT_DOUBLE_EQ(config.spark_feedforward_velocity_qp.dt_min_ratio, 0.5);
  EXPECT_DOUBLE_EQ(config.spark_feedforward_velocity_qp.dt_max_ratio, 1.5);
  EXPECT_DOUBLE_EQ(
      config.spark_feedforward_velocity_qp.maximum_joint_jump_rad, 0.35);
  EXPECT_DOUBLE_EQ(
      config.spark_feedforward_velocity_qp.stale_velocity_decay_seconds,
      0.02);
  EXPECT_DOUBLE_EQ(
      config.spark_feedforward_velocity_qp.source_stationary_velocity_rad_s,
      0.05);
  EXPECT_DOUBLE_EQ(
      config.spark_feedforward_velocity_qp.velocity_reversal_decay, 0.10);
  EXPECT_DOUBLE_EQ(
      config.spark_feedforward_velocity_qp.velocity_stationary_decay, 0.25);
  EXPECT_DOUBLE_EQ(
      config.spark_feedforward_velocity_qp.target_braking_acceleration_scale,
      0.75);
  EXPECT_DOUBLE_EQ(
      config.spark_feedforward_velocity_qp.palm_twist_filter_alpha, 0.90);
  EXPECT_DOUBLE_EQ(
      config.spark_feedforward_velocity_qp.palm_twist_lowpass_cutoff_hz,
      0.80);
  EXPECT_DOUBLE_EQ(
      config.spark_feedforward_velocity_qp.position_feedforward_gain, 1.0);
  EXPECT_DOUBLE_EQ(
      config.spark_feedforward_velocity_qp.orientation_feedforward_gain,
      1.0);
  EXPECT_DOUBLE_EQ(
      config.spark_feedforward_velocity_qp.position_high_frequency_feedforward_gain,
      0.75);
  EXPECT_DOUBLE_EQ(
      config.spark_feedforward_velocity_qp.orientation_high_frequency_feedforward_gain,
      0.65);
  EXPECT_DOUBLE_EQ(
      config.spark_feedforward_velocity_qp.reference_position_gain, 10.0);
  EXPECT_DOUBLE_EQ(
      config.spark_feedforward_velocity_qp.joint_position_gain, 4.0);
  EXPECT_DOUBLE_EQ(config.spark_feedforward_velocity_qp.attack_seconds, 0.08);
  EXPECT_DOUBLE_EQ(config.spark_feedforward_velocity_qp.release_seconds, 0.05);
  std::filesystem::remove(path);
}

TEST(Config, LoadsAndNamesSparkUpperQpoasesHeadroomFeedforwardVelocityQp) {
  std::string text = projectConfigText();
  replaceOnce(
      text, "algorithm: hierarchical_qp",
      "algorithm: spark_upper_qpoases_headroom_feedforward_velocity_qp");
  text += R"(
spark_headroom_feedforward_velocity_qp:
  enabled: true
  low_headroom: 0.10
  full_headroom: 0.30
  reduction_time_seconds: 0.020
  recovery_time_seconds: 0.100
  jerk_usage_attack_seconds: 0.050
  jerk_usage_release_seconds: 0.150
  task_scaling_min_position: 0.75
  task_scaling_min_orientation: 0.75
  stationary_reference_hold_linear_velocity_m_s: 0.01
  stationary_reference_hold_angular_velocity_rad_s: 0.05
  settled_hold_enabled: true
  settled_hold_dwell_seconds: 0.30
  settled_hold_headroom_enter: 0.05
)";
  const auto path = writeTemporaryConfig(
      "spark_upper_qpoases_headroom_feedforward_velocity_qp.yaml", text);

  const QpIkConfig config = loadConfig(path.string());

  EXPECT_EQ(config.ik_algorithm,
            IkAlgorithm::kSparkUpperQpoasesHeadroomFeedforwardVelocityQp);
  EXPECT_EQ(toString(config.ik_algorithm),
            "spark_upper_qpoases_headroom_feedforward_velocity_qp");
  EXPECT_TRUE(usesSparkHeadroomFeedforwardVelocityQp(config.ik_algorithm));
  EXPECT_TRUE(usesSparkFeedforwardVelocityQp(config.ik_algorithm));
  EXPECT_TRUE(usesSparkGuidance(config.ik_algorithm));
  EXPECT_TRUE(usesSparkUpperQpoases(config.ik_algorithm));
  EXPECT_TRUE(usesHierarchicalVelocityQp(config.ik_algorithm));
  EXPECT_DOUBLE_EQ(
      config.spark_headroom_feedforward_velocity_qp.low_headroom, 0.10);
  EXPECT_DOUBLE_EQ(
      config.spark_headroom_feedforward_velocity_qp.full_headroom, 0.30);
  EXPECT_DOUBLE_EQ(
      config.spark_headroom_feedforward_velocity_qp.reduction_time_seconds,
      0.020);
  EXPECT_DOUBLE_EQ(
      config.spark_headroom_feedforward_velocity_qp.recovery_time_seconds,
      0.100);
  EXPECT_DOUBLE_EQ(
      config.spark_headroom_feedforward_velocity_qp.jerk_usage_attack_seconds,
      0.050);
  EXPECT_DOUBLE_EQ(
      config.spark_headroom_feedforward_velocity_qp.jerk_usage_release_seconds,
      0.150);
  EXPECT_DOUBLE_EQ(
      config.spark_headroom_feedforward_velocity_qp
          .stationary_reference_hold_linear_velocity_m_s,
      0.01);
  EXPECT_DOUBLE_EQ(
      config.spark_headroom_feedforward_velocity_qp
          .stationary_reference_hold_angular_velocity_rad_s,
      0.05);
  EXPECT_TRUE(
      config.spark_headroom_feedforward_velocity_qp.settled_hold_enabled);
  EXPECT_DOUBLE_EQ(
      config.spark_headroom_feedforward_velocity_qp
          .settled_hold_dwell_seconds,
      0.30);
  EXPECT_DOUBLE_EQ(
      config.spark_headroom_feedforward_velocity_qp
          .settled_hold_headroom_enter,
      0.05);
  std::filesystem::remove(path);
}

TEST(Config, RejectsInvalidSparkHeadroomFeedforwardSettings) {
  const std::vector<std::pair<std::string, std::string>> invalid_settings{
      {"low_headroom: 0.10", "low_headroom: -0.01"},
      {"full_headroom: 0.30", "full_headroom: 0.10"},
      {"reduction_time_seconds: 0.020",
       "reduction_time_seconds: 0.0"},
      {"recovery_time_seconds: 0.100", "recovery_time_seconds: 0.0"},
      {"jerk_usage_attack_seconds: 0.050",
       "jerk_usage_attack_seconds: 0.0"},
      {"jerk_usage_release_seconds: 0.150",
       "jerk_usage_release_seconds: 0.0"},
      {"task_scaling_min_position: 0.75",
       "task_scaling_min_position: 1.0"},
      {"task_scaling_min_orientation: 0.75",
       "task_scaling_min_orientation: -0.1"},
      {"stationary_reference_hold_linear_velocity_m_s: 0.01",
       "stationary_reference_hold_linear_velocity_m_s: 0.0"},
      {"stationary_reference_hold_angular_velocity_rad_s: 0.05",
       "stationary_reference_hold_angular_velocity_rad_s: -0.1"},
      {"settled_hold_dwell_seconds: 0.30",
       "settled_hold_dwell_seconds: 0.0"},
      {"settled_hold_headroom_enter: 0.05",
       "settled_hold_headroom_enter: 1.1"},
  };
  for (const auto& [valid, invalid] : invalid_settings) {
    std::string text = projectConfigText();
    text += R"(
spark_headroom_feedforward_velocity_qp:
  low_headroom: 0.10
  full_headroom: 0.30
  reduction_time_seconds: 0.020
  recovery_time_seconds: 0.100
  jerk_usage_attack_seconds: 0.050
  jerk_usage_release_seconds: 0.150
  task_scaling_min_position: 0.75
  task_scaling_min_orientation: 0.75
  stationary_reference_hold_linear_velocity_m_s: 0.01
  stationary_reference_hold_angular_velocity_rad_s: 0.05
  settled_hold_enabled: true
  settled_hold_dwell_seconds: 0.30
  settled_hold_headroom_enter: 0.05
)";
    replaceOnce(text, valid, invalid);
    const auto path = writeTemporaryConfig("invalid_spark_headroom.yaml", text);
    EXPECT_THROW(loadConfig(path.string()), std::runtime_error) << invalid;
    std::filesystem::remove(path);
  }
}

TEST(Config, RejectsInvalidSparkFeedforwardVelocityQpSettings) {
  const std::vector<std::pair<std::string, std::string>> invalid_settings{
      {"alpha: 0.65", "alpha: 0.0"},
      {"beta: 0.12", "beta: 1.1"},
      {"dt_median_window: 31", "dt_median_window: 30"},
      {"dt_min_ratio: 0.5", "dt_min_ratio: 1.0"},
      {"dt_max_ratio: 1.5", "dt_max_ratio: 1.0"},
      {"source_stationary_velocity_rad_s: 0.05",
       "source_stationary_velocity_rad_s: -0.01"},
      {"velocity_reversal_decay: 0.10", "velocity_reversal_decay: 1.1"},
      {"velocity_stationary_decay: 0.25", "velocity_stationary_decay: -0.1"},
      {"target_braking_acceleration_scale: 0.75",
       "target_braking_acceleration_scale: 1.1"},
      {"palm_twist_filter_alpha: 0.90",
       "palm_twist_filter_alpha: 0.0"},
      {"palm_twist_lowpass_cutoff_hz: 0.80",
       "palm_twist_lowpass_cutoff_hz: 0.0"},
      {"attack_seconds: 0.08", "attack_seconds: 0.0"},
  };
  for (const auto& [valid, invalid] : invalid_settings) {
    std::string text = projectConfigText();
    text += R"(
spark_feedforward_velocity_qp:
  alpha: 0.65
  beta: 0.12
  dt_median_window: 31
  dt_min_ratio: 0.5
  dt_max_ratio: 1.5
  source_stationary_velocity_rad_s: 0.05
  velocity_reversal_decay: 0.10
  velocity_stationary_decay: 0.25
  target_braking_acceleration_scale: 0.75
  palm_twist_filter_alpha: 0.90
  palm_twist_lowpass_cutoff_hz: 0.80
  attack_seconds: 0.08
)";
    replaceOnce(text, valid, invalid);
    const std::filesystem::path path =
        writeTemporaryConfig("invalid_spark_feedforward.yaml", text);
    EXPECT_THROW(loadConfig(path.string()), std::runtime_error) << invalid;
    std::filesystem::remove(path);
  }
}

TEST(Config, LoadsOptionalModelStateOnlyControllerPolicy) {
  std::string text = projectConfigText();
  replaceOnce(text, "controller:\n  rate_hz: 1000.0",
              "controller:\n  rate_hz: 1000.0\n  model_state_only: true");
  const std::filesystem::path path =
      writeTemporaryConfig("model_state_only.yaml", text);

  const QpIkConfig config = loadConfig(path.string());

  EXPECT_TRUE(config.controller.model_state_only);
  std::filesystem::remove(path);
}

TEST(Config, InitialPostureDefaultsToDisabledMidpoint) {
  const std::filesystem::path path =
      std::filesystem::path(TIANJI_PROJECT_SOURCE_DIR) / "config" / "qp_ik.yaml";
  const QpIkConfig config = loadConfig(path.string());
  ArmLimits limits;
  limits.lower_position = Vec7::Constant(-2.0);
  limits.upper_position = Vec7::Constant(4.0);

  EXPECT_FALSE(config.controller.initial_posture_enabled);
  EXPECT_TRUE(configuredInitialPosture(config.controller, limits,
                                       ArmSide::kLeft)
                  .isApprox(Vec7::Constant(1.0)));
  EXPECT_TRUE(configuredInitialPosture(config.controller, limits,
                                       ArmSide::kRight)
                  .isApprox(Vec7::Constant(1.0)));
}

TEST(Config, ResolvesHomeRelativeToControllerConfigAndEnablesPosture) {
  const auto home = writeTemporaryConfig(
      "relative_home.yaml",
      "left_home_rad: [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7]\n"
      "right_home_rad: [-0.1, 0.2, -0.3, 0.4, -0.5, 0.6, -0.7]\n");
  std::string text = projectConfigText();
  replaceOnce(text, "controller:\n  rate_hz: 1000.0",
              "controller:\n  rate_hz: 1000.0\n"
              "  home_config: " + home.filename().string());
  const auto path = writeTemporaryConfig("relative_home_controller.yaml", text);
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
  std::string text = projectConfigText();
  replaceOnce(text, "controller:\n  rate_hz: 1000.0",
              "controller:\n  rate_hz: 1000.0\n"
              "  initial_posture_enabled: true\n"
              "  initial_left_q_rad: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]\n"
              "  initial_right_q_rad: [-0.1, -0.2, -0.3, -0.4, -0.5, -0.6, -0.7]\n"
              "  home_config: " + home.filename().string());
  const auto path = writeTemporaryConfig("measured_home_controller.yaml", text);
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
  std::string text = projectConfigText();
  replaceOnce(text, "controller:\n  rate_hz: 1000.0",
              "controller:\n  rate_hz: 1000.0\n"
              "  initial_posture_enabled: true\n"
              "  initial_left_q_rad: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]\n"
              "  home_config: " + home.filename().string());
  const auto path = writeTemporaryConfig("partial_home_controller.yaml", text);

  EXPECT_THROW(loadConfig(path.string()), std::runtime_error);
  std::filesystem::remove(path);
  std::filesystem::remove(home);
}

TEST(Config, RejectsMalformedInitialPostureVector) {
  std::string text = projectConfigText();
  replaceOnce(text, "controller:\n  rate_hz: 1000.0",
              "controller:\n  rate_hz: 1000.0\n"
              "  initial_posture_enabled: true\n"
              "  initial_left_q_rad: [0.0, 0.0]\n"
              "  initial_right_q_rad: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]");
  const std::filesystem::path path =
      writeTemporaryConfig("malformed_initial_posture.yaml", text);

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

  try {
    (void)configuredInitialPosture(controller, limits, ArmSide::kLeft);
    FAIL() << "expected an out-of-range posture to be rejected";
  } catch (const std::runtime_error& error) {
    EXPECT_NE(std::string(error.what()).find("left"), std::string::npos);
    EXPECT_NE(std::string(error.what()).find("3"), std::string::npos);
  }
}

TEST(Config, LoadsHierarchicalQpProfile) {
  const auto path = std::filesystem::path(TIANJI_PROJECT_SOURCE_DIR) /
                    "config" / "qp_ik_hierarchical.yaml";
  const QpIkConfig config = loadConfig(path.string());
  EXPECT_EQ(config.ik_algorithm, IkAlgorithm::kHierarchicalQp);
  EXPECT_DOUBLE_EQ(config.controller.rate_hz, 200.0);
  EXPECT_DOUBLE_EQ(config.hierarchical_qp.slack_weight_position, 3.0e4);
  EXPECT_DOUBLE_EQ(config.hierarchical_qp.slack_weight_orientation, 3.0e3);
  EXPECT_DOUBLE_EQ(config.hierarchical_qp.slack_position_scale, 1.0);
  EXPECT_DOUBLE_EQ(config.hierarchical_qp.slack_orientation_scale, 1.0);
  EXPECT_DOUBLE_EQ(config.dls.damping, 1.0e-3);
  EXPECT_DOUBLE_EQ(config.cartesian_servo.kff_linear, 0.0);
  EXPECT_DOUBLE_EQ(config.cartesian_servo.kff_angular, 0.0);
  EXPECT_FALSE(config.cartesian_otg.enabled);
}

TEST(Config, LoadsCartesianOtgVelocityProfile) {
  const auto path = std::filesystem::path(TIANJI_PROJECT_SOURCE_DIR) /
                    "config" / "qp_ik_cartesian_otg_velocity.yaml";
  const QpIkConfig config = loadConfig(path.string());
  EXPECT_DOUBLE_EQ(config.trajectories.frequency_hz, 1.0);
  EXPECT_TRUE(config.cartesian_otg.enabled);
  EXPECT_DOUBLE_EQ(config.cartesian_otg.translation_velocity_max, 2.0);
  EXPECT_DOUBLE_EQ(config.cartesian_otg.translation_acceleration_max, 12.0);
  EXPECT_DOUBLE_EQ(config.cartesian_otg.translation_jerk_max, 120.0);
  EXPECT_DOUBLE_EQ(config.cartesian_otg.angular_velocity_max, 8.0);
  EXPECT_DOUBLE_EQ(config.cartesian_otg.angular_acceleration_max, 40.0);
  EXPECT_DOUBLE_EQ(config.cartesian_otg.angular_jerk_max, 400.0);
  EXPECT_DOUBLE_EQ(config.cartesian_otg.orientation_gain, 10.0);
  EXPECT_DOUBLE_EQ(config.cartesian_otg.orientation_settle_error_rad, 1.0e-5);
  EXPECT_DOUBLE_EQ(config.cartesian_otg.angular_settle_velocity_rad_s, 1.0e-4);
  EXPECT_DOUBLE_EQ(config.cartesian_otg.angular_settle_acceleration_rad_s2,
                   1.0e-3);
  EXPECT_DOUBLE_EQ(config.cartesian_servo.kff_linear, 0.0);
  EXPECT_DOUBLE_EQ(config.cartesian_servo.kff_angular, 0.0);
  EXPECT_DOUBLE_EQ(config.cartesian_servo.max_linear_velocity, 2.5);
  EXPECT_DOUBLE_EQ(config.cartesian_servo.max_angular_velocity, 8.0);
  EXPECT_TRUE(config.joint_limits.max_acceleration_rad_s2.isApprox(
      Vec7::Constant(60.0)));
  EXPECT_TRUE(config.joint_limits.braking_acceleration_rad_s2.isApprox(
      Vec7::Constant(45.0)));
  EXPECT_EQ(config.control_level, ControlLevel::kVelocity);
}

TEST(Config, LoadsCartesianOtgAccelerationProfile) {
  const auto path = std::filesystem::path(TIANJI_PROJECT_SOURCE_DIR) /
                    "config" / "qp_ik_cartesian_otg_acceleration.yaml";
  const QpIkConfig config = loadConfig(path.string());
  EXPECT_EQ(config.control_level, ControlLevel::kAcceleration);
  EXPECT_TRUE(config.cartesian_otg.enabled);
  EXPECT_DOUBLE_EQ(config.cartesian_otg.translation_velocity_max, 2.0);
  EXPECT_DOUBLE_EQ(config.cartesian_otg.translation_acceleration_max, 12.0);
  EXPECT_DOUBLE_EQ(config.cartesian_otg.translation_jerk_max, 120.0);
  EXPECT_DOUBLE_EQ(config.cartesian_otg.angular_velocity_max, 8.0);
  EXPECT_DOUBLE_EQ(config.cartesian_otg.angular_acceleration_max, 40.0);
  EXPECT_DOUBLE_EQ(config.cartesian_otg.angular_jerk_max, 400.0);
  EXPECT_DOUBLE_EQ(config.cartesian_otg.orientation_settle_error_rad, 1.0e-5);
  EXPECT_DOUBLE_EQ(config.cartesian_otg.angular_settle_velocity_rad_s, 1.0e-4);
  EXPECT_DOUBLE_EQ(config.cartesian_otg.angular_settle_acceleration_rad_s2,
                   1.0e-3);
  EXPECT_DOUBLE_EQ(config.cartesian_acceleration.kp_position, 100.0);
  EXPECT_DOUBLE_EQ(config.cartesian_acceleration.kd_position, 20.0);
  EXPECT_DOUBLE_EQ(config.cartesian_acceleration.kp_orientation, 70.0);
  EXPECT_DOUBLE_EQ(config.cartesian_acceleration.kd_orientation, 16.0);
  EXPECT_DOUBLE_EQ(config.cartesian_acceleration.linear_limit, 30.0);
  EXPECT_DOUBLE_EQ(config.cartesian_acceleration.angular_limit, 100.0);
  EXPECT_DOUBLE_EQ(config.acceleration_qp.slack_linear_scale, 5.0);
  EXPECT_DOUBLE_EQ(config.acceleration_qp.slack_angular_scale, 15.0);
  EXPECT_TRUE(config.joint_acceleration_limits.max_acceleration_rad_s2.isApprox(
      Vec7::Constant(60.0)));
  EXPECT_TRUE(config.joint_acceleration_limits.braking_acceleration_rad_s2.isApprox(
      Vec7::Constant(45.0)));
  EXPECT_TRUE(config.joint_limits.max_acceleration_rad_s2.isApprox(
      Vec7::Constant(60.0)));
  EXPECT_TRUE(config.joint_limits.braking_acceleration_rad_s2.isApprox(
      Vec7::Constant(45.0)));
  EXPECT_TRUE(config.joint_acceleration_limits.hard_jerk_enabled);
  EXPECT_TRUE(config.joint_acceleration_limits.max_jerk_rad_s3.isApprox(
      Vec7::Constant(6000.0)));
  EXPECT_DOUBLE_EQ(config.trajectories.frequency_hz, 1.0);
}


TEST(Config, LoadsAccelerationTaskScalingConfiguration) {
  std::string text = projectConfigText();
  text += "\nacceleration_qp:\n"
              "  task_scaling_enabled: true\n"
              "  task_scaling_min_position: 0.20\n"
              "  task_scaling_min_orientation: 0.30\n"
              "  task_scaling_weight_position: 120.0\n"
              "  task_scaling_weight_orientation: 80.0\n";
  const auto path =
      writeTemporaryConfig("tianji_acceleration_task_scaling.yaml", text);
  const QpIkConfig config = loadConfig(path.string());
  std::filesystem::remove(path);

  EXPECT_TRUE(config.acceleration_qp.task_scaling_enabled);
  EXPECT_DOUBLE_EQ(config.acceleration_qp.task_scaling_min_position, 0.20);
  EXPECT_DOUBLE_EQ(config.acceleration_qp.task_scaling_min_orientation, 0.30);
  EXPECT_DOUBLE_EQ(config.acceleration_qp.task_scaling_weight_position, 120.0);
  EXPECT_DOUBLE_EQ(config.acceleration_qp.task_scaling_weight_orientation,
                   80.0);
}

TEST(Config, LoadsVelocityTaskScalingConfiguration) {
  std::string text = projectConfigText();
  replaceOnce(text,
              "  nominal_gain: 0.2\n"
              "  slack_weight_position: 3.0e4",
              "  nominal_gain: 0.2\n"
              "  task_scaling_enabled: true\n"
              "  task_scaling_min_position: 0.20\n"
              "  task_scaling_min_orientation: 0.30\n"
              "  task_scaling_weight_position: 120.0\n"
              "  task_scaling_weight_orientation: 80.0\n"
              "  slack_weight_position: 3.0e4");
  const auto path =
      writeTemporaryConfig("tianji_velocity_task_scaling.yaml", text);
  const QpIkConfig config = loadConfig(path.string());
  std::filesystem::remove(path);

  EXPECT_TRUE(config.hierarchical_qp.task_scaling_enabled);
  EXPECT_DOUBLE_EQ(config.hierarchical_qp.task_scaling_min_position, 0.20);
  EXPECT_DOUBLE_EQ(config.hierarchical_qp.task_scaling_min_orientation, 0.30);
  EXPECT_DOUBLE_EQ(config.hierarchical_qp.task_scaling_weight_position, 120.0);
  EXPECT_DOUBLE_EQ(config.hierarchical_qp.task_scaling_weight_orientation,
                   80.0);
}

TEST(Config, LoadsQpLimitObservationProfile) {
  const auto standard_path = std::filesystem::path(TIANJI_PROJECT_SOURCE_DIR) /
                             "config" / "qp_ik_hierarchical.yaml";
  const auto path = std::filesystem::path(TIANJI_PROJECT_SOURCE_DIR) /
                    "config" / "qp_ik_qp_limit.yaml";
  const QpIkConfig standard = loadConfig(standard_path.string());
  const QpIkConfig config = loadConfig(path.string());
  EXPECT_EQ(config.ik_algorithm, IkAlgorithm::kHierarchicalQp);
  EXPECT_TRUE(config.cartesian_servo.kp_position.isApprox(
      Eigen::Vector3d::Constant(15.0)));
  EXPECT_TRUE(config.cartesian_servo.kp_orientation.isApprox(
      Eigen::Vector3d::Constant(10.0)));
  EXPECT_DOUBLE_EQ(config.cartesian_servo.kff_linear, 0.8);
  EXPECT_DOUBLE_EQ(config.cartesian_servo.kff_angular, 0.8);
  EXPECT_DOUBLE_EQ(config.cartesian_servo.feedforward_filter_cutoff_hz, 15.0);
  EXPECT_DOUBLE_EQ(config.cartesian_servo.prediction_horizon_seconds, 0.015);
  EXPECT_DOUBLE_EQ(config.cartesian_servo.target_timeout_seconds, 0.075);
  EXPECT_DOUBLE_EQ(config.cartesian_servo.max_linear_velocity, 1.2);
  EXPECT_DOUBLE_EQ(config.cartesian_servo.max_angular_velocity, 4.0);
  EXPECT_DOUBLE_EQ(config.joint_limits.velocity_scale, 1.00);
  EXPECT_DOUBLE_EQ(config.joint_limits.margin_rad, 0.05);
  EXPECT_TRUE(config.joint_limits.max_acceleration_rad_s2.isApprox(
      Vec7::Constant(20.0)));
  EXPECT_TRUE(config.joint_limits.braking_acceleration_rad_s2.isApprox(
      Vec7::Constant(15.0)));
  EXPECT_DOUBLE_EQ(config.hierarchical_qp.slack_position_scale, 1.0);
  EXPECT_DOUBLE_EQ(config.hierarchical_qp.slack_orientation_scale, 2.0);
  EXPECT_DOUBLE_EQ(config.safety.reference_tracking_warn_rad, 0.10);
  EXPECT_DOUBLE_EQ(config.safety.reference_tracking_stop_rad, 0.20);

  EXPECT_DOUBLE_EQ(config.controller.rate_hz, standard.controller.rate_hz);
  EXPECT_EQ(config.ik_algorithm, standard.ik_algorithm);
  EXPECT_EQ(config.qp.solver, standard.qp.solver);
  EXPECT_DOUBLE_EQ(config.qp.position_weight, standard.qp.position_weight);
  EXPECT_DOUBLE_EQ(config.qp.orientation_weight, standard.qp.orientation_weight);
  EXPECT_DOUBLE_EQ(config.qp.regularization, standard.qp.regularization);
  EXPECT_DOUBLE_EQ(config.qp.nominal_weight, standard.qp.nominal_weight);
  EXPECT_DOUBLE_EQ(config.qp.nominal_gain, standard.qp.nominal_gain);
  EXPECT_DOUBLE_EQ(config.hierarchical_qp.lambda_reg,
                   standard.hierarchical_qp.lambda_reg);
  EXPECT_DOUBLE_EQ(config.hierarchical_qp.posture_weight,
                   standard.hierarchical_qp.posture_weight);
  EXPECT_DOUBLE_EQ(config.hierarchical_qp.continuity_weight,
                   standard.hierarchical_qp.continuity_weight);
  EXPECT_DOUBLE_EQ(config.hierarchical_qp.nominal_gain,
                   standard.hierarchical_qp.nominal_gain);
  EXPECT_DOUBLE_EQ(config.hierarchical_qp.slack_weight_position,
                   standard.hierarchical_qp.slack_weight_position);
  EXPECT_DOUBLE_EQ(config.hierarchical_qp.slack_weight_orientation,
                   standard.hierarchical_qp.slack_weight_orientation);
  EXPECT_DOUBLE_EQ(config.hierarchical_qp.equality_tolerance,
                   standard.hierarchical_qp.equality_tolerance);
  EXPECT_DOUBLE_EQ(config.dls.damping, standard.dls.damping);
  EXPECT_DOUBLE_EQ(config.dls.nominal_gain, standard.dls.nominal_gain);
  EXPECT_DOUBLE_EQ(config.joint_limits.margin_rad,
                   standard.joint_limits.margin_rad);
  EXPECT_DOUBLE_EQ(config.joint_limits.velocity_scale,
                   standard.joint_limits.velocity_scale);
  EXPECT_EQ(config.osqp.max_iterations, standard.osqp.max_iterations);
  EXPECT_DOUBLE_EQ(config.osqp.absolute_tolerance,
                   standard.osqp.absolute_tolerance);
  EXPECT_DOUBLE_EQ(config.osqp.relative_tolerance,
                   standard.osqp.relative_tolerance);
  EXPECT_EQ(config.osqp.polishing, standard.osqp.polishing);
  EXPECT_EQ(config.qpoases.max_working_set_recalculations,
            standard.qpoases.max_working_set_recalculations);
  EXPECT_DOUBLE_EQ(config.qpoases.cpu_time_limit_seconds,
                   standard.qpoases.cpu_time_limit_seconds);
  EXPECT_DOUBLE_EQ(config.safety.bound_tolerance,
                   standard.safety.bound_tolerance);
  EXPECT_DOUBLE_EQ(config.safety.hessian_eigenvalue_tolerance,
                   standard.safety.hessian_eigenvalue_tolerance);
  EXPECT_DOUBLE_EQ(config.safety.max_target_position_step,
                   standard.safety.max_target_position_step);
  EXPECT_DOUBLE_EQ(config.safety.max_target_orientation_step,
                   standard.safety.max_target_orientation_step);
  EXPECT_DOUBLE_EQ(config.trajectories.circle_radius,
                   standard.trajectories.circle_radius);
  EXPECT_DOUBLE_EQ(config.trajectories.figure_eight_width,
                   standard.trajectories.figure_eight_width);
  EXPECT_DOUBLE_EQ(config.trajectories.figure_eight_height,
                   standard.trajectories.figure_eight_height);
  EXPECT_DOUBLE_EQ(config.trajectories.angular_amplitude,
                   standard.trajectories.angular_amplitude);
  EXPECT_DOUBLE_EQ(config.trajectories.frequency_hz,
                   standard.trajectories.frequency_hz);
}

TEST(Config, RejectsNonPositiveControlRate) {
  std::string text = projectConfigText();
  replaceOnce(text, "rate_hz: 1000.0", "rate_hz: 0.0");
  const auto path = writeTemporaryConfig("tianji_invalid_qp_ik.yaml", text);
  EXPECT_THROW(loadConfig(path.string()), std::runtime_error);
  std::filesystem::remove(path);
}

TEST(Config, RejectsUnknownSolver) {
  const std::filesystem::path source =
      std::filesystem::path(TIANJI_PROJECT_SOURCE_DIR) / "config" / "qp_ik.yaml";
  const std::filesystem::path path =
      std::filesystem::temp_directory_path() /
      (std::to_string(::getpid()) + "_tianji_unknown_solver.yaml");
  std::ifstream input(source);
  std::string text((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  text.replace(text.find("solver: qpoases"), std::string("solver: qpoases").size(),
               "solver: unknown");
  {
    std::ofstream output(path);
    output << text;
  }
  EXPECT_THROW(loadConfig(path.string()), std::runtime_error);
  std::filesystem::remove(path);
}

TEST(Config, RejectsUnknownIkAlgorithm) {
  std::string text = projectConfigText();
  replaceOnce(text, "algorithm: hierarchical_qp", "algorithm: unknown");
  const auto path = writeTemporaryConfig("tianji_unknown_ik.yaml", text);
  EXPECT_THROW(loadConfig(path.string()), std::runtime_error);
  std::filesystem::remove(path);
}

TEST(Config, RejectsNonPositiveHierarchicalWeightsAndTolerance) {
  const std::vector<std::pair<std::string, std::string>> replacements{
      {"slack_weight_position: 3.0e4", "slack_weight_position: 0.0"},
      {"slack_weight_orientation: 3.0e3", "slack_weight_orientation: -1.0"},
      {"equality_tolerance: 1.0e-8", "equality_tolerance: 0.0"},
  };
  for (const auto& replacement : replacements) {
    std::string text = projectConfigText();
    replaceOnce(text, replacement.first, replacement.second);
    const auto path = writeTemporaryConfig("tianji_invalid_hierarchical.yaml", text);
    EXPECT_THROW(loadConfig(path.string()), std::runtime_error);
    std::filesystem::remove(path);
  }
}

TEST(Config, RejectsNegativeContinuityWeight) {
  std::string text = projectConfigText();
  replaceOnce(text, "continuity_weight: 1.0e-3", "continuity_weight: -1.0");
  const auto path = writeTemporaryConfig("tianji_invalid_continuity.yaml", text);
  EXPECT_THROW(loadConfig(path.string()), std::runtime_error);
  std::filesystem::remove(path);
}

TEST(Config, AllowsZeroAndRejectsNegativeAccelerationArmAngleWeight) {
  std::string zero_text = projectConfigText();
  zero_text += "\nacceleration_qp:\n  arm_angle_weight: 0.0\n";
  const auto zero_path =
      writeTemporaryConfig("tianji_zero_arm_angle_weight.yaml", zero_text);
  EXPECT_DOUBLE_EQ(loadConfig(zero_path.string()).acceleration_qp.arm_angle_weight,
                   0.0);
  std::filesystem::remove(zero_path);

  std::string negative_text = projectConfigText();
  negative_text += "\nacceleration_qp:\n  arm_angle_weight: -1.0\n";
  const auto negative_path = writeTemporaryConfig(
      "tianji_negative_arm_angle_weight.yaml", negative_text);
  EXPECT_THROW(loadConfig(negative_path.string()), std::runtime_error);
  std::filesystem::remove(negative_path);
}

TEST(Config, RejectsNonPositiveDlsDamping) {
  std::string text = projectConfigText();
  replaceOnce(text, "damping: 1.0e-3", "damping: 0.0");
  const auto path = writeTemporaryConfig("tianji_invalid_dls.yaml", text);
  EXPECT_THROW(loadConfig(path.string()), std::runtime_error);
  std::filesystem::remove(path);
}

TEST(Config, RejectsNonPositivePicoTeleopJumpLimits) {
  const std::vector<std::pair<std::string, std::string>> limits{
      {"0.0", "0.60"},
      {"0.15", "-0.1"},
  };
  for (const auto& [position_limit, orientation_limit] : limits) {
    std::string text = projectConfigText();
    text += "\npico_teleop:\n  max_position_jump_m: " + position_limit +
            "\n  max_orientation_jump_rad: " + orientation_limit + "\n";
    const auto path = writeTemporaryConfig("tianji_invalid_pico_teleop.yaml", text);
    EXPECT_THROW(loadConfig(path.string()), std::runtime_error);
    std::filesystem::remove(path);
  }
}

TEST(Config, RejectsInvalidArmAngleConfiguration) {
  const std::vector<std::pair<std::string, std::string>> replacements{
      {"max_velocity_rad_s: 24.0", "max_velocity_rad_s: 0.0"},
      {"max_velocity_rad_s: 24.0",
       "max_velocity_rad_s: 24.0\n"
       "  continuity_max_velocity_rad_s: 0.0"},
      {"full_weight_radius_m: 0.050", "full_weight_radius_m: 0.010"},
      {"full_weight_radius_m: 0.050",
       "full_weight_radius_m: 0.050\n"
       "  branch_lock_radius_m: 0.010"},
      {"reference_rate_limit_rad_s: 8.0",
       "reference_rate_limit_rad_s: 0.0"},
      {"reference_rate_limit_rad_s: 8.0",
       "reference_rate_limit_rad_s: 8.0\n"
       "  reference_projection_hold_enter: 0.20\n"
       "  reference_projection_hold_exit: 0.10"},
      {"reference_rate_limit_rad_s: 8.0",
       "reference_rate_limit_rad_s: 8.0\n"
       "  error_branch_hysteresis_rad: 0.0"},
      {"max_velocity_rad_s: 24.0",
       "max_velocity_rad_s: 24.0\n"
       "  joint_limit_soft_margin_rad: 0.0"},
      {"max_velocity_rad_s: 24.0",
       "max_velocity_rad_s: 24.0\n"
       "  joint_limit_recovery_gain_rad_s_per_rad: 0.0"},
  };
  for (const auto& replacement : replacements) {
    std::string text = projectConfigText();
    replaceOnce(text, replacement.first, replacement.second);
    const auto path =
        writeTemporaryConfig("tianji_invalid_arm_angle.yaml", text);
    EXPECT_THROW(loadConfig(path.string()), std::runtime_error)
        << replacement.first;
    std::filesystem::remove(path);
  }
}

TEST(Config, RejectsInvalidUpperArmOutwardConfiguration) {
  const std::vector<std::string> invalid_sections{
      "  minimum_outward_distance_m: -0.001\n",
      "  velocity_gain: 0.0\n",
      "  acceleration_kp: -1.0\n",
      "  acceleration_kd: -1.0\n",
  };
  for (std::size_t index = 0; index < invalid_sections.size(); ++index) {
    std::string text = projectConfigText();
    text += "\nupper_arm_outward:\n" + invalid_sections[index];
    const auto path = writeTemporaryConfig(
        "tianji_invalid_upper_arm_outward_" + std::to_string(index) +
            ".yaml",
        text);
    EXPECT_THROW(loadConfig(path.string()), std::runtime_error) << index;
    std::filesystem::remove(path);
  }
}

TEST(Config, RejectsCartesianOtgPredictionBeyondTargetTimeout) {
  std::string text = projectConfigText();
  text += "\ncartesian_otg:\n"
          "  enabled: true\n"
          "  translation_prediction_enabled: true\n"
          "  translation_prediction_horizon_seconds: 0.100\n";
  const auto path =
      writeTemporaryConfig("tianji_invalid_otg_prediction_horizon.yaml", text);
  EXPECT_THROW(loadConfig(path.string()), std::runtime_error);
  std::filesystem::remove(path);
}

TEST(Config, RejectsInvalidCartesianOtgStationaryHoldThresholds) {
  const std::vector<std::string> invalid_hold_sections{
      "  stationary_hold_dwell_seconds: 0.0\n",
      "  translation_hold_enter_velocity_m_s: 0.12\n"
      "  translation_hold_exit_velocity_m_s: 0.06\n",
      "  translation_hold_exit_position_error_m: 0.0\n",
      "  orientation_hold_enter_velocity_rad_s: 0.25\n"
      "  orientation_hold_exit_velocity_rad_s: 0.12\n",
      "  orientation_hold_exit_error_rad: -0.01\n",
  };
  for (std::size_t index = 0; index < invalid_hold_sections.size(); ++index) {
    std::string text = projectConfigText();
    text += "\ncartesian_otg:\n  stationary_hold_enabled: true\n" +
            invalid_hold_sections[index];
    const auto path = writeTemporaryConfig(
        "tianji_invalid_otg_stationary_hold_" + std::to_string(index) +
            ".yaml",
        text);
    EXPECT_THROW(loadConfig(path.string()), std::runtime_error) << index;
    std::filesystem::remove(path);
  }
}

}  // namespace
}  // namespace tianji_qp_ik
