#pragma once

#include "tianji_qp_ik/types.hpp"

#include <Eigen/Core>

#include <string>

namespace tianji_qp_ik {

enum class IkAlgorithm { kPicoEeFrankaDls };

struct ControllerConfig {
  double rate_hz{1000.0};
  bool model_state_only{false};
  bool initial_posture_enabled{false};
  Vec7 initial_left_q_rad{Vec7::Zero()};
  Vec7 initial_right_q_rad{Vec7::Zero()};
  // Explicit Pinocchio model for the DLS route. No FK fallback.
  std::string pico_ee_dls_kinematics_urdf_path;
};

Vec7 configuredInitialPosture(const ControllerConfig& config,
                              const ArmLimits& limits, ArmSide side);

struct CartesianServoConfig {
  double kff_linear{0.0};
  double kff_angular{0.0};
  double feedforward_filter_cutoff_hz{15.0};
  double prediction_horizon_seconds{0.015};
  double target_timeout_seconds{0.075};
};

struct IterativeDlsConfig {
  int max_iterations{20};
  double position_tolerance_m{5.0e-4};
  double orientation_tolerance_rad{1.0e-2};
  double position_gain{1.0};
  double orientation_gain{1.0};
  double minimum_damping{1.0e-6};
  double maximum_damping{0.3};
  double singular_value_threshold{0.05};
  double maximum_step_norm_rad{0.35};
  double maximum_joint_step_rad{0.20};
  double minimum_merit_improvement{1.0e-10};
  double nominal_posture_gain{0.05};
  double arm_angle_gain{0.025};
};

struct SharedRootShapeConfig {
  double maximum_joint_rotation_jump_rad{0.75};
};

struct DlsPostureRuckigConfig {
  bool enabled{true};
  double velocity_scale{0.6};
  Vec7 max_velocity_rad_s{
      (Vec7() << 0.8, 0.8, 1.0, 1.0, 1.2, 1.2, 1.2).finished()};
  Vec7 max_acceleration_rad_s2{
      (Vec7() << 7.854, 7.854, 15.708, 15.708, 15.708, 15.708,
       15.708).finished()};
  Vec7 max_jerk_rad_s3{
      (Vec7() << 600.0, 600.0, 1000.0, 1000.0, 1000.0, 1000.0,
       1000.0).finished()};
  double validation_tolerance{1.0e-8};
};

struct JointLimitConfig {
  double margin_rad{0.05};
  Vec7 max_acceleration_rad_s2{Vec7::Constant(1.0e6)};
  Vec7 max_jerk_rad_s3{Vec7::Constant(1.0e6)};
};
struct SafetyConfig {
  double bound_tolerance{1e-8};
  double max_target_position_step{0.05};
  double max_target_orientation_step{0.20};
};

struct TrajectoryConfig {
  double circle_radius{0.06};
  double figure_eight_width{0.08};
  double figure_eight_height{0.05};
  double angular_amplitude{0.35};
  double frequency_hz{0.10};
};

struct PicoTeleopConfig {
  double max_position_jump_m{0.15};
  double max_orientation_jump_rad{0.60};
};

struct PicoEeFrankaDlsConfig {
  DlsPostureRuckigConfig post_smoothing;
  bool enabled{false};
  double max_arm_plane_rate_rad_s{0.0};
  bool arm_plane_direction_guidance{false};
  Vec7 home_left_rad{
      (Vec7() << 0.9599310886, -1.1344640138, -1.2217304764,
       -1.0471975512, 1.0471975512, 0.0, 0.0)
          .finished()};
  Vec7 home_right_rad{
      (Vec7() << -0.9599310886, -1.1344640138, 1.2217304764,
       -1.0471975512, -1.0471975512, 0.0, 0.0)
          .finished()};
  Vec7 max_velocity_rad_s{Vec7::Constant(4.0)};
};

struct QpIkConfig {
  PicoEeFrankaDlsConfig pico_ee_franka_dls;
  std::string shared_root_profile_path;
  ControllerConfig controller;
  IkAlgorithm ik_algorithm{IkAlgorithm::kPicoEeFrankaDls};
  CartesianServoConfig cartesian_servo;
  IterativeDlsConfig iterative_dls;
  SharedRootShapeConfig shared_root_shape;
  JointLimitConfig joint_limits;
  SafetyConfig safety;
  TrajectoryConfig trajectories;
  PicoTeleopConfig pico_teleop;
};

enum class ConfigConsumer { kStandalone, kSharedRootAware };
QpIkConfig loadConfig(const std::string& path,
                     ConfigConsumer consumer = ConfigConsumer::kStandalone);
std::string toString(IkAlgorithm algorithm);

}  // namespace tianji_qp_ik
