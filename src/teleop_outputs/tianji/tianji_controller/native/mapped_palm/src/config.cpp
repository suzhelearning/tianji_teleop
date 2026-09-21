#include "tianji_mapped_palm/config.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <string>

namespace tianji_mapped_palm {
namespace {

template <typename T>
T required(const YAML::Node& parent, const char* key) {
  const YAML::Node value = parent[key];
  if (!value) {
    throw std::runtime_error(std::string("missing configuration key: ") + key);
  }
  return value.as<T>();
}

Eigen::Vector3d requiredVector3(const YAML::Node& parent, const char* key) {
  const YAML::Node value = parent[key];
  if (!value || !value.IsSequence() || value.size() != 3) {
    throw std::runtime_error(std::string("configuration key must contain three values: ") + key);
  }
  return {value[0].as<double>(), value[1].as<double>(), value[2].as<double>()};
}

Eigen::Vector3d optionalVector3(const YAML::Node& parent, const char* key,
                               const Eigen::Vector3d& default_value) {
  const YAML::Node value = parent[key];
  if (!value) {
    return default_value;
  }
  if (!value.IsSequence() || value.size() != 3) {
    throw std::runtime_error(std::string("configuration key must contain "
                                         "three values: ") + key);
  }
  return {value[0].as<double>(), value[1].as<double>(),
          value[2].as<double>()};
}

double optionalDouble(const YAML::Node& parent, const char* key,
                      double default_value) {
  const YAML::Node value = parent[key];
  return value ? value.as<double>() : default_value;
}

int optionalInt(const YAML::Node& parent, const char* key, int default_value) {
  const YAML::Node value = parent[key];
  return value ? value.as<int>() : default_value;
}

Vec7 optionalVector7(const YAML::Node& parent, const char* key,
                     const Vec7& default_value) {
  const YAML::Node value = parent[key];
  if (!value) {
    return default_value;
  }
  if (value.IsScalar()) {
    return Vec7::Constant(value.as<double>());
  }
  if (!value.IsSequence() || value.size() != 7) {
    throw std::runtime_error(std::string("configuration key must be a scalar "
                                         "or contain seven values: ") + key);
  }
  Vec7 result;
  for (int index = 0; index < 7; ++index) {
    result[index] = value[static_cast<std::size_t>(index)].as<double>();
  }
  return result;
}

Vec7 optionalStrictVector7(const YAML::Node& parent, const char* key,
                           const Vec7& default_value) {
  const YAML::Node value = parent[key];
  if (!value) {
    return default_value;
  }
  if (!value.IsSequence() || value.size() != 7) {
    throw std::runtime_error(std::string("configuration key must contain "
                                         "seven values: ") + key);
  }
  Vec7 result;
  for (int index = 0; index < kArmDof; ++index) {
    result[index] = value[static_cast<std::size_t>(index)].as<double>();
    if (!std::isfinite(result[index])) {
      throw std::runtime_error(std::string("configuration key must contain "
                                           "seven finite values: ") + key);
    }
  }
  return result;
}

bool optionalBool(const YAML::Node& parent, const char* key,
                  bool default_value) {
  const YAML::Node value = parent[key];
  return value ? value.as<bool>() : default_value;
}

void requirePositive(double value, const char* name) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::runtime_error(std::string(name) + " must be finite and positive");
  }
}

void requireNonNegative(double value, const char* name) {
  if (!std::isfinite(value) || value < 0.0) {
    throw std::runtime_error(std::string(name) + " must be finite and non-negative");
  }
}

void requirePositive(const Vec7& values, const char* name) {
  for (int index = 0; index < values.size(); ++index) {
    if (!std::isfinite(values[index]) || values[index] <= 0.0) {
      throw std::runtime_error(std::string(name) +
                               " must contain seven finite positive values");
    }
  }
}

SolverBackend parseSolver(const std::string& value) {
  if (value == "osqp") {
    return SolverBackend::kOsqp;
  }
  if (value == "qpoases") {
    return SolverBackend::kQpoases;
  }
  throw std::runtime_error("qp.solver must be 'osqp' or 'qpoases'");
}

PicoEeTargetSource parsePicoEeTargetSourceValue(const std::string& value) {
  if (value == "packet_targets") {
    return PicoEeTargetSource::kPacketTargets;
  }
  if (value == "mapped_corrected_palm") {
    return PicoEeTargetSource::kMappedCorrectedPalm;
  }
  throw std::runtime_error(
      "pico_teleop.ee_target_source must be 'packet_targets' or "
      "'mapped_corrected_palm'");
}

PicoArmAngleReferenceSource parsePicoArmAngleReferenceSourceValue(
    const std::string& value) {
  if (value == "packet_direction") {
    return PicoArmAngleReferenceSource::kPacketDirection;
  }
  if (value == "mapped_skeleton_plane") {
    return PicoArmAngleReferenceSource::kMappedSkeletonPlane;
  }
  throw std::runtime_error(
      "pico_teleop.arm_angle_reference_source must be 'packet_direction' or "
      "'mapped_skeleton_plane'");
}

PicoMappedArmAngleVectorMode parsePicoMappedArmAngleVectorMode(
    const std::string& value) {
  if (value == "disabled") {
    return PicoMappedArmAngleVectorMode::kDisabled;
  }
  if (value == "shadow") {
    return PicoMappedArmAngleVectorMode::kShadow;
  }
  if (value == "active") {
    return PicoMappedArmAngleVectorMode::kActive;
  }
  throw std::runtime_error(
      "pico_mapped_arm_angle_vector_nullspace.mode must be 'disabled', "
      "'shadow', or 'active'");
}

PicoMappedVectorQpWeightMode parsePicoMappedVectorQpWeightMode(
    const std::string& value) {
  if (value == "legacy") {
    return PicoMappedVectorQpWeightMode::kLegacy;
  }
  if (value == "decoupled") {
    return PicoMappedVectorQpWeightMode::kDecoupled;
  }
  throw std::runtime_error(
      "pico_mapped_arm_angle_vector_nullspace.qp_weight_mode must be "
      "'legacy' or 'decoupled'");
}

VelocityQpSmoothnessMode parseVelocityQpSmoothnessMode(
    const std::string& value) {
  if (value == "legacy") {
    return VelocityQpSmoothnessMode::kLegacy;
  }
  if (value == "normalized_split") {
    return VelocityQpSmoothnessMode::kNormalizedSplit;
  }
  throw std::runtime_error(
      "hierarchical_qp.smoothness_mode must be 'legacy' or "
      "'normalized_split'");
}

PicoEeTwistEstimatorMode parsePicoEeTwistEstimatorMode(
    const std::string& value) {
  if (value == "legacy") {
    return PicoEeTwistEstimatorMode::kLegacy;
  }
  if (value == "shadow") {
    return PicoEeTwistEstimatorMode::kShadow;
  }
  if (value == "active") {
    return PicoEeTwistEstimatorMode::kActive;
  }
  throw std::runtime_error(
      "pico_ee_twist_estimator.mode must be 'legacy', 'shadow', or 'active'");
}

PicoEeFeedforwardAllocatorMode parsePicoEeFeedforwardAllocatorMode(
    const std::string& value) {
  if (value == "legacy") {
    return PicoEeFeedforwardAllocatorMode::kLegacy;
  }
  if (value == "shadow") {
    return PicoEeFeedforwardAllocatorMode::kShadow;
  }
  if (value == "active") {
    return PicoEeFeedforwardAllocatorMode::kActive;
  }
  throw std::runtime_error(
      "pico_ee_feedforward_allocator.mode must be 'legacy', 'shadow', or "
      "'active'");
}

PicoEeTaskAllocationMode parsePicoEeTaskAllocationMode(
    const std::string& value) {
  if (value == "disabled") {
    return PicoEeTaskAllocationMode::kDisabled;
  }
  if (value == "shadow") {
    return PicoEeTaskAllocationMode::kShadow;
  }
  if (value == "active") {
    return PicoEeTaskAllocationMode::kActive;
  }
  throw std::runtime_error(
      "pico_ee_task_allocator.mode must be 'disabled', 'shadow', or "
      "'active'");
}

IkAlgorithm parseIkAlgorithm(const std::string& value) {
  if (value == "hierarchical_qp") {
    return IkAlgorithm::kHierarchicalQp;
  }
  if (value == "nullspace_dls") {
    return IkAlgorithm::kNullspaceDls;
  }
  if (value == "spark_guided_velocity_qp") {
    return IkAlgorithm::kSparkGuidedVelocityQp;
  }
  if (value == "spark_direct_velocity_qp") {
    return IkAlgorithm::kSparkDirectVelocityQp;
  }
  if (value == "spark_pose_velocity_qp") {
    return IkAlgorithm::kSparkPoseVelocityQp;
  }
  if (value == "spark_upper_qpoases_direct") {
    return IkAlgorithm::kSparkUpperQpoasesDirect;
  }
  if (value == "spark_upper_qpoases_velocity_qp") {
    return IkAlgorithm::kSparkUpperQpoasesVelocityQp;
  }
  if (value == "spark_upper_qpoases_cartesian_otg_velocity_qp") {
    return IkAlgorithm::kSparkUpperQpoasesCartesianOtgVelocityQp;
  }
  if (value == "spark_upper_qpoases_feedforward_velocity_qp") {
    return IkAlgorithm::kSparkUpperQpoasesFeedforwardVelocityQp;
  }
  if (value == "spark_upper_qpoases_headroom_feedforward_velocity_qp") {
    return IkAlgorithm::kSparkUpperQpoasesHeadroomFeedforwardVelocityQp;
  }
  if (value == "pico_ee_mapped_corrected_palm_velocity_qp") {
    return IkAlgorithm::kPicoEeMappedCorrectedPalmVelocityQp;
  }
  throw std::runtime_error(
      "ik.algorithm must be 'hierarchical_qp', 'nullspace_dls', or "
      "one of the Spark velocity-QP modes or "
      "'pico_ee_mapped_corrected_palm_velocity_qp'");
}

ControlLevel parseControlLevel(const std::string& value) {
  if (value == "velocity") {
    return ControlLevel::kVelocity;
  }
  if (value == "acceleration") {
    return ControlLevel::kAcceleration;
  }
  throw std::runtime_error("control.level must be 'velocity' or 'acceleration'");
}

CartesianOtgDynamicLimits dynamicLimitsFrom(
    const CartesianOtgConfig& config) {
  CartesianOtgDynamicLimits limits;
  limits.translation_velocity_max = config.translation_velocity_max;
  limits.translation_acceleration_max = config.translation_acceleration_max;
  limits.translation_jerk_max = config.translation_jerk_max;
  limits.angular_velocity_max = config.angular_velocity_max;
  limits.angular_acceleration_max = config.angular_acceleration_max;
  limits.angular_jerk_max = config.angular_jerk_max;
  return limits;
}

void loadDynamicLimits(const YAML::Node& node,
                       CartesianOtgDynamicLimits& limits) {
  if (!node) {
    return;
  }
  limits.translation_velocity_max = optionalDouble(
      node, "translation_velocity_max", limits.translation_velocity_max);
  limits.translation_acceleration_max = optionalDouble(
      node, "translation_acceleration_max",
      limits.translation_acceleration_max);
  limits.translation_jerk_max = optionalDouble(
      node, "translation_jerk_max", limits.translation_jerk_max);
  limits.angular_velocity_max = optionalDouble(
      node, "angular_velocity_max", limits.angular_velocity_max);
  limits.angular_acceleration_max = optionalDouble(
      node, "angular_acceleration_max", limits.angular_acceleration_max);
  limits.angular_jerk_max = optionalDouble(
      node, "angular_jerk_max", limits.angular_jerk_max);
}

void validateDynamicLimits(const CartesianOtgDynamicLimits& limits,
                           const std::string& prefix) {
  requirePositive(limits.translation_velocity_max,
                  (prefix + ".translation_velocity_max").c_str());
  requirePositive(limits.translation_acceleration_max,
                  (prefix + ".translation_acceleration_max").c_str());
  requirePositive(limits.translation_jerk_max,
                  (prefix + ".translation_jerk_max").c_str());
  requirePositive(limits.angular_velocity_max,
                  (prefix + ".angular_velocity_max").c_str());
  requirePositive(limits.angular_acceleration_max,
                  (prefix + ".angular_acceleration_max").c_str());
  requirePositive(limits.angular_jerk_max,
                  (prefix + ".angular_jerk_max").c_str());
}

void applyDynamicLimits(const CartesianOtgDynamicLimits& limits,
                        CartesianOtgConfig& config) {
  config.translation_velocity_max = limits.translation_velocity_max;
  config.translation_acceleration_max = limits.translation_acceleration_max;
  config.translation_jerk_max = limits.translation_jerk_max;
  config.angular_velocity_max = limits.angular_velocity_max;
  config.angular_acceleration_max = limits.angular_acceleration_max;
  config.angular_jerk_max = limits.angular_jerk_max;
}

}  // namespace

PicoEeTargetSource parsePicoEeTargetSource(const std::string& value) {
  return parsePicoEeTargetSourceValue(value);
}

PicoArmAngleReferenceSource parsePicoArmAngleReferenceSource(
    const std::string& value) {
  return parsePicoArmAngleReferenceSourceValue(value);
}

QpIkConfig loadConfig(const std::string& path) {
  const YAML::Node root = YAML::LoadFile(path);
  QpIkConfig config;

  const YAML::Node controller = root["controller"];
  config.controller.rate_hz = required<double>(controller, "rate_hz");
  config.controller.model_state_only = optionalBool(
      controller, "model_state_only", config.controller.model_state_only);
  const bool has_home_config = static_cast<bool>(controller["home_config"]);
  const bool has_initial_left =
      static_cast<bool>(controller["initial_left_q_rad"]);
  const bool has_initial_right =
      static_cast<bool>(controller["initial_right_q_rad"]);
  if (has_initial_left != has_initial_right) {
    throw std::runtime_error(
        "controller initial posture override requires both left and right "
        "seven-value vectors");
  }
  config.controller.initial_posture_enabled = optionalBool(
      controller, "initial_posture_enabled",
      has_home_config || config.controller.initial_posture_enabled);
  // Measured-state runtime configurations override Home as a complete pair.
  // Do not resolve the deployment-relative path from their temporary location.
  if (has_home_config && !has_initial_left) {
    std::filesystem::path home_path =
        required<std::string>(controller, "home_config");
    if (home_path.empty()) {
      throw std::runtime_error("controller.home_config must not be empty");
    }
    if (home_path.is_relative()) {
      home_path = std::filesystem::path(path).parent_path() / home_path;
    }
    const YAML::Node home = YAML::LoadFile(home_path.string());
    if (!home["left_home_rad"] || !home["right_home_rad"]) {
      throw std::runtime_error(
          "controller.home_config requires left_home_rad and right_home_rad");
    }
    config.controller.initial_left_q_rad = optionalStrictVector7(
        home, "left_home_rad", config.controller.initial_left_q_rad);
    config.controller.initial_right_q_rad = optionalStrictVector7(
        home, "right_home_rad", config.controller.initial_right_q_rad);
  }
  config.controller.initial_left_q_rad = optionalStrictVector7(
      controller, "initial_left_q_rad",
      config.controller.initial_left_q_rad);
  config.controller.initial_right_q_rad = optionalStrictVector7(
      controller, "initial_right_q_rad",
      config.controller.initial_right_q_rad);
  if (config.controller.initial_posture_enabled &&
      !has_home_config && !has_initial_left) {
    throw std::runtime_error(
        "enabled controller initial posture requires home_config or both "
        "left and right seven-value vectors");
  }

  const YAML::Node control = root["control"];
  if (control && control["level"]) {
    config.control_level =
        parseControlLevel(control["level"].as<std::string>());
  }

  config.ik_algorithm = parseIkAlgorithm(required<std::string>(root["ik"], "algorithm"));

  const YAML::Node servo = root["cartesian_servo"];
  config.cartesian_servo.kp_position = requiredVector3(servo, "kp_position");
  config.cartesian_servo.kp_orientation = requiredVector3(servo, "kp_orientation");
  config.cartesian_servo.adaptive_gain_enabled = optionalBool(
      servo, "adaptive_gain_enabled",
      config.cartesian_servo.adaptive_gain_enabled);
  config.cartesian_servo.kp_position_near = optionalVector3(
      servo, "kp_position_near", config.cartesian_servo.kp_position_near);
  config.cartesian_servo.kp_orientation_near = optionalVector3(
      servo, "kp_orientation_near",
      config.cartesian_servo.kp_orientation_near);
  config.cartesian_servo.motion_gain_enabled = optionalBool(
      servo, "motion_gain_enabled",
      config.cartesian_servo.motion_gain_enabled);
  config.cartesian_servo.kp_position_motion = optionalVector3(
      servo, "kp_position_motion",
      config.cartesian_servo.kp_position_motion);
  config.cartesian_servo.kp_orientation_motion = optionalVector3(
      servo, "kp_orientation_motion",
      config.cartesian_servo.kp_orientation_motion);
  config.cartesian_servo.position_gain_transition_start_m = optionalDouble(
      servo, "position_gain_transition_start_m",
      config.cartesian_servo.position_gain_transition_start_m);
  config.cartesian_servo.position_gain_transition_end_m = optionalDouble(
      servo, "position_gain_transition_end_m",
      config.cartesian_servo.position_gain_transition_end_m);
  config.cartesian_servo.orientation_gain_transition_start_rad =
      optionalDouble(servo, "orientation_gain_transition_start_rad",
                     config.cartesian_servo
                         .orientation_gain_transition_start_rad);
  config.cartesian_servo.orientation_gain_transition_end_rad =
      optionalDouble(servo, "orientation_gain_transition_end_rad",
                     config.cartesian_servo.orientation_gain_transition_end_rad);
  config.cartesian_servo.kff_linear =
      optionalDouble(servo, "kff_linear", config.cartesian_servo.kff_linear);
  config.cartesian_servo.kff_angular =
      optionalDouble(servo, "kff_angular", config.cartesian_servo.kff_angular);
  config.cartesian_servo.feedforward_filter_cutoff_hz = optionalDouble(
      servo, "feedforward_filter_cutoff_hz",
      config.cartesian_servo.feedforward_filter_cutoff_hz);
  config.cartesian_servo.prediction_horizon_seconds = optionalDouble(
      servo, "prediction_horizon_seconds",
      config.cartesian_servo.prediction_horizon_seconds);
  config.cartesian_servo.target_timeout_seconds = optionalDouble(
      servo, "target_timeout_seconds",
      config.cartesian_servo.target_timeout_seconds);
  config.cartesian_servo.max_linear_velocity = required<double>(servo, "max_linear_velocity");
  config.cartesian_servo.max_angular_velocity = required<double>(servo, "max_angular_velocity");

  const YAML::Node pico_teleop = root["pico_teleop"];
  if (pico_teleop) {
    config.pico_teleop.max_position_jump_m = optionalDouble(
        pico_teleop, "max_position_jump_m",
        config.pico_teleop.max_position_jump_m);
    config.pico_teleop.max_orientation_jump_rad = optionalDouble(
        pico_teleop, "max_orientation_jump_rad",
        config.pico_teleop.max_orientation_jump_rad);
    if (pico_teleop["ee_target_source"]) {
      config.pico_teleop.ee_target_source = parsePicoEeTargetSourceValue(
          pico_teleop["ee_target_source"].as<std::string>());
    }
    if (pico_teleop["arm_angle_reference_source"]) {
      config.pico_teleop.arm_angle_reference_source =
          parsePicoArmAngleReferenceSourceValue(
              pico_teleop["arm_angle_reference_source"].as<std::string>());
    }
  }

  if (usesPicoEeMappedCorrectedPalm(config.ik_algorithm) &&
      (config.pico_teleop.ee_target_source !=
           PicoEeTargetSource::kMappedCorrectedPalm ||
       config.pico_teleop.arm_angle_reference_source !=
           PicoArmAngleReferenceSource::kMappedSkeletonPlane)) {
    throw std::runtime_error(
        "pico_ee_mapped_corrected_palm_velocity_qp requires "
        "pico_teleop.ee_target_source=mapped_corrected_palm and "
        "pico_teleop.arm_angle_reference_source=mapped_skeleton_plane");
  }

  const YAML::Node arm_angle = root["arm_angle"];
  if (arm_angle) {
    config.arm_angle.enabled = optionalBool(
        arm_angle, "enabled", config.arm_angle.enabled);
    config.arm_angle.nullspace_only = optionalBool(
        arm_angle, "nullspace_only", config.arm_angle.nullspace_only);
    config.arm_angle.kp_velocity = optionalDouble(
        arm_angle, "kp_velocity", config.arm_angle.kp_velocity);
    config.arm_angle.max_velocity_rad_s = optionalDouble(
        arm_angle, "max_velocity_rad_s",
        config.arm_angle.max_velocity_rad_s);
    config.arm_angle.kp_acceleration = optionalDouble(
        arm_angle, "kp_acceleration", config.arm_angle.kp_acceleration);
    config.arm_angle.kd_acceleration = optionalDouble(
        arm_angle, "kd_acceleration", config.arm_angle.kd_acceleration);
    config.arm_angle.max_acceleration_rad_s2 = optionalDouble(
        arm_angle, "max_acceleration_rad_s2",
        config.arm_angle.max_acceleration_rad_s2);
    config.arm_angle.minimum_radius_m = optionalDouble(
        arm_angle, "minimum_radius_m", config.arm_angle.minimum_radius_m);
    config.arm_angle.full_weight_radius_m = optionalDouble(
        arm_angle, "full_weight_radius_m",
        config.arm_angle.full_weight_radius_m);
    config.arm_angle.branch_lock_radius_m = optionalDouble(
        arm_angle, "branch_lock_radius_m",
        config.arm_angle.branch_lock_radius_m);
    config.arm_angle.reference_rate_limit_rad_s = optionalDouble(
        arm_angle, "reference_rate_limit_rad_s",
        config.arm_angle.reference_rate_limit_rad_s);
    config.arm_angle.reference_projection_hold_enter = optionalDouble(
        arm_angle, "reference_projection_hold_enter",
        config.arm_angle.reference_projection_hold_enter);
    config.arm_angle.reference_projection_hold_exit = optionalDouble(
        arm_angle, "reference_projection_hold_exit",
        config.arm_angle.reference_projection_hold_exit);
    config.arm_angle.error_branch_hysteresis_rad = optionalDouble(
        arm_angle, "error_branch_hysteresis_rad",
        config.arm_angle.error_branch_hysteresis_rad);
    config.arm_angle.reference_governor_enabled = optionalBool(
        arm_angle, "reference_governor_enabled",
        config.arm_angle.reference_governor_enabled);
    config.arm_angle.reference_governor_enter_error_rad = optionalDouble(
        arm_angle, "reference_governor_enter_error_rad",
        config.arm_angle.reference_governor_enter_error_rad);
    config.arm_angle.reference_governor_exit_error_rad = optionalDouble(
        arm_angle, "reference_governor_exit_error_rad",
        config.arm_angle.reference_governor_exit_error_rad);
    config.arm_angle.reference_governor_tracking_error_rad = optionalDouble(
        arm_angle, "reference_governor_tracking_error_rad",
        config.arm_angle.reference_governor_tracking_error_rad);
    config.arm_angle.tracking_envelope_enabled = optionalBool(
        arm_angle, "tracking_envelope_enabled",
        config.arm_angle.tracking_envelope_enabled);
    config.arm_angle.tracking_envelope_max_error_rad = optionalDouble(
        arm_angle, "tracking_envelope_max_error_rad",
        config.arm_angle.tracking_envelope_max_error_rad);
    config.arm_angle.tracking_envelope_gain = optionalDouble(
        arm_angle, "tracking_envelope_gain",
        config.arm_angle.tracking_envelope_gain);
    config.arm_angle.continuity_kp_velocity = optionalDouble(
        arm_angle, "continuity_kp_velocity",
        config.arm_angle.continuity_kp_velocity);
    config.arm_angle.continuity_max_velocity_rad_s = optionalDouble(
        arm_angle, "continuity_max_velocity_rad_s",
        config.arm_angle.continuity_max_velocity_rad_s);
    config.arm_angle.continuity_kp_acceleration = optionalDouble(
        arm_angle, "continuity_kp_acceleration",
        config.arm_angle.continuity_kp_acceleration);
    config.arm_angle.continuity_kd_acceleration = optionalDouble(
        arm_angle, "continuity_kd_acceleration",
        config.arm_angle.continuity_kd_acceleration);
    config.arm_angle.continuity_max_acceleration_rad_s2 = optionalDouble(
        arm_angle, "continuity_max_acceleration_rad_s2",
        config.arm_angle.continuity_max_acceleration_rad_s2);
    config.arm_angle.continuity_weight_scale = optionalDouble(
        arm_angle, "continuity_weight_scale",
        config.arm_angle.continuity_weight_scale);
    config.arm_angle.joint_limit_soft_margin_rad = optionalDouble(
        arm_angle, "joint_limit_soft_margin_rad",
        config.arm_angle.joint_limit_soft_margin_rad);
    config.arm_angle.joint_limit_recovery_gain_rad_s_per_rad = optionalDouble(
        arm_angle, "joint_limit_recovery_gain_rad_s_per_rad",
        config.arm_angle.joint_limit_recovery_gain_rad_s_per_rad);
    config.arm_angle.branch_lock_enabled = optionalBool(
        arm_angle, "branch_lock_enabled", config.arm_angle.branch_lock_enabled);
    config.arm_angle.branch_lock_kp_velocity = optionalDouble(
        arm_angle, "branch_lock_kp_velocity",
        config.arm_angle.branch_lock_kp_velocity);
    config.arm_angle.branch_lock_kp_acceleration = optionalDouble(
        arm_angle, "branch_lock_kp_acceleration",
        config.arm_angle.branch_lock_kp_acceleration);
    config.arm_angle.branch_lock_kd_acceleration = optionalDouble(
        arm_angle, "branch_lock_kd_acceleration",
        config.arm_angle.branch_lock_kd_acceleration);
  }

  const YAML::Node upper_arm_outward = root["upper_arm_outward"];
  if (upper_arm_outward) {
    config.upper_arm_outward.minimum_outward_distance_m = optionalDouble(
        upper_arm_outward, "minimum_outward_distance_m",
        config.upper_arm_outward.minimum_outward_distance_m);
    config.upper_arm_outward.velocity_gain = optionalDouble(
        upper_arm_outward, "velocity_gain",
        config.upper_arm_outward.velocity_gain);
    config.upper_arm_outward.acceleration_kp = optionalDouble(
        upper_arm_outward, "acceleration_kp",
        config.upper_arm_outward.acceleration_kp);
    config.upper_arm_outward.acceleration_kd = optionalDouble(
        upper_arm_outward, "acceleration_kd",
        config.upper_arm_outward.acceleration_kd);
  }

  const YAML::Node cartesian_otg = root["cartesian_otg"];
  if (cartesian_otg) {
    config.cartesian_otg.enabled = optionalBool(
        cartesian_otg, "enabled", config.cartesian_otg.enabled);
    config.cartesian_otg.translation_position_mode = optionalBool(
        cartesian_otg, "translation_position_mode",
        config.cartesian_otg.translation_position_mode);
    config.cartesian_otg.translation_tracking_enabled = optionalBool(
        cartesian_otg, "translation_tracking_enabled",
        config.cartesian_otg.translation_tracking_enabled);
    config.cartesian_otg.translation_tracking_gain = optionalDouble(
        cartesian_otg, "translation_tracking_gain",
        config.cartesian_otg.translation_tracking_gain);
    config.cartesian_otg.translation_stationary_velocity_threshold =
        optionalDouble(
            cartesian_otg, "translation_stationary_velocity_threshold",
            config.cartesian_otg.translation_stationary_velocity_threshold);
    config.cartesian_otg.translation_prediction_enabled = optionalBool(
        cartesian_otg, "translation_prediction_enabled",
        config.cartesian_otg.translation_prediction_enabled);
    config.cartesian_otg.translation_prediction_horizon_seconds =
        optionalDouble(
            cartesian_otg, "translation_prediction_horizon_seconds",
            config.cartesian_otg.translation_prediction_horizon_seconds);
    config.cartesian_otg.stationary_hold_enabled = optionalBool(
        cartesian_otg, "stationary_hold_enabled",
        config.cartesian_otg.stationary_hold_enabled);
    config.cartesian_otg.stationary_hold_dwell_seconds = optionalDouble(
        cartesian_otg, "stationary_hold_dwell_seconds",
        config.cartesian_otg.stationary_hold_dwell_seconds);
    config.cartesian_otg.translation_hold_enter_velocity_m_s = optionalDouble(
        cartesian_otg, "translation_hold_enter_velocity_m_s",
        config.cartesian_otg.translation_hold_enter_velocity_m_s);
    config.cartesian_otg.translation_hold_exit_velocity_m_s = optionalDouble(
        cartesian_otg, "translation_hold_exit_velocity_m_s",
        config.cartesian_otg.translation_hold_exit_velocity_m_s);
    config.cartesian_otg.translation_hold_exit_position_error_m =
        optionalDouble(
            cartesian_otg, "translation_hold_exit_position_error_m",
            config.cartesian_otg.translation_hold_exit_position_error_m);
    config.cartesian_otg.orientation_hold_enter_velocity_rad_s =
        optionalDouble(
            cartesian_otg, "orientation_hold_enter_velocity_rad_s",
            config.cartesian_otg.orientation_hold_enter_velocity_rad_s);
    config.cartesian_otg.orientation_hold_exit_velocity_rad_s =
        optionalDouble(
            cartesian_otg, "orientation_hold_exit_velocity_rad_s",
            config.cartesian_otg.orientation_hold_exit_velocity_rad_s);
    config.cartesian_otg.orientation_hold_exit_error_rad = optionalDouble(
        cartesian_otg, "orientation_hold_exit_error_rad",
        config.cartesian_otg.orientation_hold_exit_error_rad);
    config.cartesian_otg.translation_velocity_max = optionalDouble(
        cartesian_otg, "translation_velocity_max",
        config.cartesian_otg.translation_velocity_max);
    config.cartesian_otg.translation_acceleration_max = optionalDouble(
        cartesian_otg, "translation_acceleration_max",
        config.cartesian_otg.translation_acceleration_max);
    config.cartesian_otg.translation_jerk_max = optionalDouble(
        cartesian_otg, "translation_jerk_max",
        config.cartesian_otg.translation_jerk_max);
    config.cartesian_otg.angular_velocity_max = optionalDouble(
        cartesian_otg, "angular_velocity_max",
        config.cartesian_otg.angular_velocity_max);
    config.cartesian_otg.angular_acceleration_max = optionalDouble(
        cartesian_otg, "angular_acceleration_max",
        config.cartesian_otg.angular_acceleration_max);
    config.cartesian_otg.angular_jerk_max = optionalDouble(
        cartesian_otg, "angular_jerk_max",
        config.cartesian_otg.angular_jerk_max);
    config.cartesian_otg.orientation_gain = optionalDouble(
        cartesian_otg, "orientation_gain",
        config.cartesian_otg.orientation_gain);
    config.cartesian_otg.orientation_settle_error_rad = optionalDouble(
        cartesian_otg, "orientation_settle_error_rad",
        config.cartesian_otg.orientation_settle_error_rad);
    config.cartesian_otg.angular_settle_velocity_rad_s = optionalDouble(
        cartesian_otg, "angular_settle_velocity_rad_s",
        config.cartesian_otg.angular_settle_velocity_rad_s);
    config.cartesian_otg.angular_settle_acceleration_rad_s2 = optionalDouble(
        cartesian_otg, "angular_settle_acceleration_rad_s2",
        config.cartesian_otg.angular_settle_acceleration_rad_s2);
  }

  config.cartesian_otg_control_profiles.velocity =
      dynamicLimitsFrom(config.cartesian_otg);
  config.cartesian_otg_control_profiles.acceleration =
      dynamicLimitsFrom(config.cartesian_otg);
  const YAML::Node cartesian_otg_control_profiles =
      root["cartesian_otg_control_profiles"];
  if (cartesian_otg_control_profiles) {
    config.cartesian_otg_control_profiles.enabled = optionalBool(
        cartesian_otg_control_profiles, "enabled",
        config.cartesian_otg_control_profiles.enabled);
    loadDynamicLimits(cartesian_otg_control_profiles["velocity"],
                      config.cartesian_otg_control_profiles.velocity);
    loadDynamicLimits(cartesian_otg_control_profiles["acceleration"],
                      config.cartesian_otg_control_profiles.acceleration);
  }

  const YAML::Node cartesian_acceleration = root["cartesian_acceleration"];
  if (cartesian_acceleration) {
    config.cartesian_acceleration.kp_position = optionalDouble(
        cartesian_acceleration, "kp_position",
        config.cartesian_acceleration.kp_position);
    config.cartesian_acceleration.kd_position = optionalDouble(
        cartesian_acceleration, "kd_position",
        config.cartesian_acceleration.kd_position);
    config.cartesian_acceleration.kp_orientation = optionalDouble(
        cartesian_acceleration, "kp_orientation",
        config.cartesian_acceleration.kp_orientation);
    config.cartesian_acceleration.kd_orientation = optionalDouble(
        cartesian_acceleration, "kd_orientation",
        config.cartesian_acceleration.kd_orientation);
    config.cartesian_acceleration.linear_limit = optionalDouble(
        cartesian_acceleration, "linear_limit",
        config.cartesian_acceleration.linear_limit);
    config.cartesian_acceleration.angular_limit = optionalDouble(
        cartesian_acceleration, "angular_limit",
        config.cartesian_acceleration.angular_limit);
  }

  const YAML::Node acceleration_qp = root["acceleration_qp"];
  if (acceleration_qp) {
    config.acceleration_qp.slack_weight_position = optionalDouble(
        acceleration_qp, "slack_weight_position",
        config.acceleration_qp.slack_weight_position);
    config.acceleration_qp.slack_weight_orientation = optionalDouble(
        acceleration_qp, "slack_weight_orientation",
        config.acceleration_qp.slack_weight_orientation);
    config.acceleration_qp.slack_linear_scale = optionalDouble(
        acceleration_qp, "slack_linear_scale",
        config.acceleration_qp.slack_linear_scale);
    config.acceleration_qp.slack_angular_scale = optionalDouble(
        acceleration_qp, "slack_angular_scale",
        config.acceleration_qp.slack_angular_scale);
    config.acceleration_qp.regularization = optionalDouble(
        acceleration_qp, "regularization",
        config.acceleration_qp.regularization);
    config.acceleration_qp.jerk_weight = optionalDouble(
        acceleration_qp, "jerk_weight", config.acceleration_qp.jerk_weight);
    config.acceleration_qp.posture_weight = optionalDouble(
        acceleration_qp, "posture_weight",
        config.acceleration_qp.posture_weight);
    config.acceleration_qp.posture_kp = optionalDouble(
        acceleration_qp, "posture_kp", config.acceleration_qp.posture_kp);
    config.acceleration_qp.posture_kd = optionalDouble(
        acceleration_qp, "posture_kd", config.acceleration_qp.posture_kd);
    config.acceleration_qp.arm_angle_weight = optionalDouble(
        acceleration_qp, "arm_angle_weight",
        config.acceleration_qp.arm_angle_weight);
    config.acceleration_qp.task_scaling_enabled = optionalBool(
        acceleration_qp, "task_scaling_enabled",
        config.acceleration_qp.task_scaling_enabled);
    config.acceleration_qp.task_scaling_min_position = optionalDouble(
        acceleration_qp, "task_scaling_min_position",
        config.acceleration_qp.task_scaling_min_position);
    config.acceleration_qp.task_scaling_min_orientation = optionalDouble(
        acceleration_qp, "task_scaling_min_orientation",
        config.acceleration_qp.task_scaling_min_orientation);
    config.acceleration_qp.task_scaling_weight_position = optionalDouble(
        acceleration_qp, "task_scaling_weight_position",
        config.acceleration_qp.task_scaling_weight_position);
    config.acceleration_qp.task_scaling_weight_orientation = optionalDouble(
        acceleration_qp, "task_scaling_weight_orientation",
        config.acceleration_qp.task_scaling_weight_orientation);
    config.acceleration_qp.equality_tolerance = optionalDouble(
        acceleration_qp, "equality_tolerance",
        config.acceleration_qp.equality_tolerance);
  }

  const YAML::Node iterative_dls = root["iterative_dls"];
  if (iterative_dls) {
    config.iterative_dls.posture_reference_enabled = optionalBool(
        iterative_dls, "posture_reference_enabled",
        config.iterative_dls.posture_reference_enabled);
    config.iterative_dls.max_iterations = optionalInt(
        iterative_dls, "max_iterations", config.iterative_dls.max_iterations);
    config.iterative_dls.position_tolerance_m = optionalDouble(
        iterative_dls, "position_tolerance_m",
        config.iterative_dls.position_tolerance_m);
    config.iterative_dls.orientation_tolerance_rad = optionalDouble(
        iterative_dls, "orientation_tolerance_rad",
        config.iterative_dls.orientation_tolerance_rad);
    config.iterative_dls.position_gain = optionalDouble(
        iterative_dls, "position_gain", config.iterative_dls.position_gain);
    config.iterative_dls.orientation_gain = optionalDouble(
        iterative_dls, "orientation_gain",
        config.iterative_dls.orientation_gain);
    config.iterative_dls.minimum_damping = optionalDouble(
        iterative_dls, "minimum_damping",
        config.iterative_dls.minimum_damping);
    config.iterative_dls.maximum_damping = optionalDouble(
        iterative_dls, "maximum_damping",
        config.iterative_dls.maximum_damping);
    config.iterative_dls.singular_value_threshold = optionalDouble(
        iterative_dls, "singular_value_threshold",
        config.iterative_dls.singular_value_threshold);
    config.iterative_dls.maximum_step_norm_rad = optionalDouble(
        iterative_dls, "maximum_step_norm_rad",
        config.iterative_dls.maximum_step_norm_rad);
    config.iterative_dls.maximum_joint_step_rad = optionalDouble(
        iterative_dls, "maximum_joint_step_rad",
        config.iterative_dls.maximum_joint_step_rad);
    config.iterative_dls.minimum_merit_improvement = optionalDouble(
        iterative_dls, "minimum_merit_improvement",
        config.iterative_dls.minimum_merit_improvement);
    config.iterative_dls.nominal_posture_gain = optionalDouble(
        iterative_dls, "nominal_posture_gain",
        config.iterative_dls.nominal_posture_gain);
    config.iterative_dls.arm_angle_gain = optionalDouble(
        iterative_dls, "arm_angle_gain", config.iterative_dls.arm_angle_gain);
    config.iterative_dls.maximum_goal_step_rad = optionalDouble(
        iterative_dls, "maximum_goal_step_rad",
        config.iterative_dls.maximum_goal_step_rad);
    config.iterative_dls.posture_reference_margin_rad = optionalDouble(
        iterative_dls, "posture_reference_margin_rad",
        config.iterative_dls.posture_reference_margin_rad);
  }

  const YAML::Node spark_upper_qpoases = root["spark_upper_qpoases"];
  if (spark_upper_qpoases) {
    auto& spark = config.spark_upper_qpoases;
    spark.calibration_frames = optionalInt(
        spark_upper_qpoases, "calibration_frames", spark.calibration_frames);
    spark.minimum_scale = optionalDouble(
        spark_upper_qpoases, "minimum_scale", spark.minimum_scale);
    spark.maximum_scale = optionalDouble(
        spark_upper_qpoases, "maximum_scale", spark.maximum_scale);
    spark.stage1_max_iterations = optionalInt(
        spark_upper_qpoases, "stage1_max_iterations",
        spark.stage1_max_iterations);
    spark.stage2_max_iterations = optionalInt(
        spark_upper_qpoases, "stage2_max_iterations",
        spark.stage2_max_iterations);
    spark.convergence_delta = optionalDouble(
        spark_upper_qpoases, "convergence_delta", spark.convergence_delta);
    spark.integration_step = optionalDouble(
        spark_upper_qpoases, "integration_step", spark.integration_step);
    spark.target_blend_seconds = optionalDouble(
        spark_upper_qpoases, "target_blend_seconds",
        spark.target_blend_seconds);
    spark.maximum_joint_rotation_jump_rad = optionalDouble(
        spark_upper_qpoases, "maximum_joint_rotation_jump_rad",
        spark.maximum_joint_rotation_jump_rad);
    spark.ik_cycle_budget_seconds = optionalDouble(
        spark_upper_qpoases, "ik_cycle_budget_seconds",
        spark.ik_cycle_budget_seconds);
    spark.trust_region_rad = optionalDouble(
        spark_upper_qpoases, "trust_region_rad", spark.trust_region_rad);
    spark.damping = optionalDouble(
        spark_upper_qpoases, "damping", spark.damping);
    spark.joint_limit_margin_rad = optionalDouble(
        spark_upper_qpoases, "joint_limit_margin_rad",
        spark.joint_limit_margin_rad);
    spark.stage1_upper_direction_weight = optionalDouble(
        spark_upper_qpoases, "stage1_upper_direction_weight",
        spark.stage1_upper_direction_weight);
    spark.stage1_forearm_direction_weight = optionalDouble(
        spark_upper_qpoases, "stage1_forearm_direction_weight",
        spark.stage1_forearm_direction_weight);
    spark.stage1_palm_orientation_weight = optionalDouble(
        spark_upper_qpoases, "stage1_palm_orientation_weight",
        spark.stage1_palm_orientation_weight);
    spark.stage2_upper_direction_weight = optionalDouble(
        spark_upper_qpoases, "stage2_upper_direction_weight",
        spark.stage2_upper_direction_weight);
    spark.stage2_forearm_direction_weight = optionalDouble(
        spark_upper_qpoases, "stage2_forearm_direction_weight",
        spark.stage2_forearm_direction_weight);
    spark.stage2_palm_orientation_weight = optionalDouble(
        spark_upper_qpoases, "stage2_palm_orientation_weight",
        spark.stage2_palm_orientation_weight);
    spark.stage2_elbow_position_weight = optionalDouble(
        spark_upper_qpoases, "stage2_elbow_position_weight",
        spark.stage2_elbow_position_weight);
    spark.stage2_wrist_position_weight = optionalDouble(
        spark_upper_qpoases, "stage2_wrist_position_weight",
        spark.stage2_wrist_position_weight);
    spark.stage2_palm_position_weight = optionalDouble(
        spark_upper_qpoases, "stage2_palm_position_weight",
        spark.stage2_palm_position_weight);
    spark.posture_weight = optionalDouble(
        spark_upper_qpoases, "posture_weight", spark.posture_weight);
    spark.joint_reference_weight = optionalDouble(
        spark_upper_qpoases, "joint_reference_weight",
        spark.joint_reference_weight);
    spark.joint_reference_attack_seconds = optionalDouble(
        spark_upper_qpoases, "joint_reference_attack_seconds",
        spark.joint_reference_attack_seconds);
    spark.joint_reference_release_seconds = optionalDouble(
        spark_upper_qpoases, "joint_reference_release_seconds",
        spark.joint_reference_release_seconds);
    spark.joint_reference_smoothness_weight = optionalDouble(
        spark_upper_qpoases, "joint_reference_smoothness_weight",
        spark.joint_reference_smoothness_weight);
    spark.posture_position_gain = optionalDouble(
        spark_upper_qpoases, "posture_position_gain",
        spark.posture_position_gain);
    spark.otg_position_tolerance_m = optionalDouble(
        spark_upper_qpoases, "otg_position_tolerance_m",
        spark.otg_position_tolerance_m);
    spark.otg_orientation_tolerance_rad = optionalDouble(
        spark_upper_qpoases, "otg_orientation_tolerance_rad",
        spark.otg_orientation_tolerance_rad);
    spark.otg_continuity_weight = optionalDouble(
        spark_upper_qpoases, "otg_continuity_weight",
        spark.otg_continuity_weight);
    spark.enforce_hard_jerk_bounds = optionalBool(
        spark_upper_qpoases, "enforce_hard_jerk_bounds",
        spark.enforce_hard_jerk_bounds);
  }

  const YAML::Node spark_feedforward =
      root["spark_feedforward_velocity_qp"];
  if (spark_feedforward) {
    auto& feedforward = config.spark_feedforward_velocity_qp;
    feedforward.enabled = optionalBool(
        spark_feedforward, "enabled", feedforward.enabled);
    feedforward.alpha = optionalDouble(
        spark_feedforward, "alpha", feedforward.alpha);
    feedforward.beta = optionalDouble(
        spark_feedforward, "beta", feedforward.beta);
    feedforward.dt_median_window = optionalInt(
        spark_feedforward, "dt_median_window",
        feedforward.dt_median_window);
    feedforward.dt_min_ratio = optionalDouble(
        spark_feedforward, "dt_min_ratio", feedforward.dt_min_ratio);
    feedforward.dt_max_ratio = optionalDouble(
        spark_feedforward, "dt_max_ratio", feedforward.dt_max_ratio);
    feedforward.maximum_joint_jump_rad = optionalDouble(
        spark_feedforward, "maximum_joint_jump_rad",
        feedforward.maximum_joint_jump_rad);
    feedforward.stale_velocity_decay_seconds = optionalDouble(
        spark_feedforward, "stale_velocity_decay_seconds",
        feedforward.stale_velocity_decay_seconds);
    feedforward.source_stationary_velocity_rad_s = optionalDouble(
        spark_feedforward, "source_stationary_velocity_rad_s",
        feedforward.source_stationary_velocity_rad_s);
    feedforward.velocity_reversal_decay = optionalDouble(
        spark_feedforward, "velocity_reversal_decay",
        feedforward.velocity_reversal_decay);
    feedforward.velocity_stationary_decay = optionalDouble(
        spark_feedforward, "velocity_stationary_decay",
        feedforward.velocity_stationary_decay);
    feedforward.target_braking_acceleration_scale = optionalDouble(
        spark_feedforward, "target_braking_acceleration_scale",
        feedforward.target_braking_acceleration_scale);
    feedforward.palm_twist_filter_alpha = optionalDouble(
        spark_feedforward, "palm_twist_filter_alpha",
        feedforward.palm_twist_filter_alpha);
    feedforward.palm_twist_lowpass_cutoff_hz = optionalDouble(
        spark_feedforward, "palm_twist_lowpass_cutoff_hz",
        feedforward.palm_twist_lowpass_cutoff_hz);
    feedforward.reference_velocity_scale = optionalDouble(
        spark_feedforward, "reference_velocity_scale",
        feedforward.reference_velocity_scale);
    feedforward.reference_acceleration_scale = optionalDouble(
        spark_feedforward, "reference_acceleration_scale",
        feedforward.reference_acceleration_scale);
    feedforward.reference_jerk_scale = optionalDouble(
        spark_feedforward, "reference_jerk_scale",
        feedforward.reference_jerk_scale);
    feedforward.reference_position_gain = optionalDouble(
        spark_feedforward, "reference_position_gain",
        feedforward.reference_position_gain);
    feedforward.position_feedforward_gain = optionalDouble(
        spark_feedforward, "position_feedforward_gain",
        feedforward.position_feedforward_gain);
    feedforward.orientation_feedforward_gain = optionalDouble(
        spark_feedforward, "orientation_feedforward_gain",
        feedforward.orientation_feedforward_gain);
    feedforward.position_high_frequency_feedforward_gain = optionalDouble(
        spark_feedforward, "position_high_frequency_feedforward_gain",
        feedforward.position_high_frequency_feedforward_gain);
    feedforward.orientation_high_frequency_feedforward_gain = optionalDouble(
        spark_feedforward, "orientation_high_frequency_feedforward_gain",
        feedforward.orientation_high_frequency_feedforward_gain);
    feedforward.joint_position_gain = optionalDouble(
        spark_feedforward, "joint_position_gain",
        feedforward.joint_position_gain);
    feedforward.attack_seconds = optionalDouble(
        spark_feedforward, "attack_seconds", feedforward.attack_seconds);
    feedforward.release_seconds = optionalDouble(
        spark_feedforward, "release_seconds", feedforward.release_seconds);
  }

  const YAML::Node spark_headroom =
      root["spark_headroom_feedforward_velocity_qp"];
  if (spark_headroom) {
    auto& headroom = config.spark_headroom_feedforward_velocity_qp;
    headroom.enabled = optionalBool(
        spark_headroom, "enabled", headroom.enabled);
    headroom.joint_reference_smoothness_weight = optionalDouble(
        spark_headroom, "joint_reference_smoothness_weight",
        headroom.joint_reference_smoothness_weight);
    headroom.joint_reference_jerk_smoothness_weight = optionalDouble(
        spark_headroom, "joint_reference_jerk_smoothness_weight",
        headroom.joint_reference_jerk_smoothness_weight);
    headroom.low_headroom = optionalDouble(
        spark_headroom, "low_headroom", headroom.low_headroom);
    headroom.full_headroom = optionalDouble(
        spark_headroom, "full_headroom", headroom.full_headroom);
    headroom.reduction_time_seconds = optionalDouble(
        spark_headroom, "reduction_time_seconds",
        headroom.reduction_time_seconds);
    headroom.recovery_time_seconds = optionalDouble(
        spark_headroom, "recovery_time_seconds",
        headroom.recovery_time_seconds);
    headroom.jerk_usage_attack_seconds = optionalDouble(
        spark_headroom, "jerk_usage_attack_seconds",
        headroom.jerk_usage_attack_seconds);
    headroom.jerk_usage_release_seconds = optionalDouble(
        spark_headroom, "jerk_usage_release_seconds",
        headroom.jerk_usage_release_seconds);
    headroom.task_scaling_min_position = optionalDouble(
        spark_headroom, "task_scaling_min_position",
        headroom.task_scaling_min_position);
    headroom.task_scaling_min_orientation = optionalDouble(
        spark_headroom, "task_scaling_min_orientation",
        headroom.task_scaling_min_orientation);
    headroom.stationary_reference_hold_linear_velocity_m_s = optionalDouble(
        spark_headroom, "stationary_reference_hold_linear_velocity_m_s",
        headroom.stationary_reference_hold_linear_velocity_m_s);
    headroom.stationary_reference_hold_angular_velocity_rad_s = optionalDouble(
        spark_headroom, "stationary_reference_hold_angular_velocity_rad_s",
        headroom.stationary_reference_hold_angular_velocity_rad_s);
    headroom.settled_hold_enabled = optionalBool(
        spark_headroom, "settled_hold_enabled",
        headroom.settled_hold_enabled);
    headroom.settled_hold_dwell_seconds = optionalDouble(
        spark_headroom, "settled_hold_dwell_seconds",
        headroom.settled_hold_dwell_seconds);
    headroom.settled_hold_headroom_enter = optionalDouble(
        spark_headroom, "settled_hold_headroom_enter",
        headroom.settled_hold_headroom_enter);
  }

  const YAML::Node pico_ee_headroom = root["pico_ee_headroom"];
  if (pico_ee_headroom) {
    config.pico_ee_headroom.low_frequency_min_scale = optionalDouble(
        pico_ee_headroom, "low_frequency_min_scale",
        config.pico_ee_headroom.low_frequency_min_scale);
    config.pico_ee_headroom.high_frequency_min_scale = optionalDouble(
        pico_ee_headroom, "high_frequency_min_scale",
        config.pico_ee_headroom.high_frequency_min_scale);
    config.pico_ee_headroom.redundancy_authority_enabled = optionalBool(
        pico_ee_headroom, "redundancy_authority_enabled",
        config.pico_ee_headroom.redundancy_authority_enabled);
    config.pico_ee_headroom.redundancy_zero_headroom = optionalDouble(
        pico_ee_headroom, "redundancy_zero_headroom",
        config.pico_ee_headroom.redundancy_zero_headroom);
    config.pico_ee_headroom.redundancy_full_headroom = optionalDouble(
        pico_ee_headroom, "redundancy_full_headroom",
        config.pico_ee_headroom.redundancy_full_headroom);
  }

  const YAML::Node pico_ee_motion_confidence =
      root["pico_ee_motion_confidence"];
  if (pico_ee_motion_confidence) {
    auto& motion = config.pico_ee_motion_confidence;
    motion.enabled = optionalBool(pico_ee_motion_confidence, "enabled",
                                  motion.enabled);
    motion.linear_quiet_m_s = optionalDouble(
        pico_ee_motion_confidence, "linear_quiet_m_s",
        motion.linear_quiet_m_s);
    motion.linear_tracking_m_s = optionalDouble(
        pico_ee_motion_confidence, "linear_tracking_m_s",
        motion.linear_tracking_m_s);
    motion.angular_quiet_rad_s = optionalDouble(
        pico_ee_motion_confidence, "angular_quiet_rad_s",
        motion.angular_quiet_rad_s);
    motion.angular_tracking_rad_s = optionalDouble(
        pico_ee_motion_confidence, "angular_tracking_rad_s",
        motion.angular_tracking_rad_s);
    motion.attack_seconds = optionalDouble(
        pico_ee_motion_confidence, "attack_seconds", motion.attack_seconds);
    motion.release_seconds = optionalDouble(
        pico_ee_motion_confidence, "release_seconds", motion.release_seconds);
  }

  const YAML::Node pico_ee_twist_estimator =
      root["pico_ee_twist_estimator"];
  if (pico_ee_twist_estimator) {
    auto& estimator = config.pico_ee_twist_estimator;
    if (pico_ee_twist_estimator["mode"]) {
      estimator.mode = parsePicoEeTwistEstimatorMode(
          pico_ee_twist_estimator["mode"].as<std::string>());
    }
    estimator.fit_window = optionalInt(
        pico_ee_twist_estimator, "fit_window", estimator.fit_window);
    estimator.maximum_condition_number = optionalDouble(
        pico_ee_twist_estimator, "maximum_condition_number",
        estimator.maximum_condition_number);
    estimator.position_full_confidence_residual_m = optionalDouble(
        pico_ee_twist_estimator, "position_full_confidence_residual_m",
        estimator.position_full_confidence_residual_m);
    estimator.position_invalid_residual_m = optionalDouble(
        pico_ee_twist_estimator, "position_invalid_residual_m",
        estimator.position_invalid_residual_m);
    estimator.orientation_full_confidence_residual_rad = optionalDouble(
        pico_ee_twist_estimator, "orientation_full_confidence_residual_rad",
        estimator.orientation_full_confidence_residual_rad);
    estimator.orientation_invalid_residual_rad = optionalDouble(
        pico_ee_twist_estimator, "orientation_invalid_residual_rad",
        estimator.orientation_invalid_residual_rad);
    estimator.age_full_confidence_seconds = optionalDouble(
        pico_ee_twist_estimator, "age_full_confidence_seconds",
        estimator.age_full_confidence_seconds);
    estimator.maximum_age_seconds = optionalDouble(
        pico_ee_twist_estimator, "maximum_age_seconds",
        estimator.maximum_age_seconds);
    estimator.confidence_attack_seconds = optionalDouble(
        pico_ee_twist_estimator, "confidence_attack_seconds",
        estimator.confidence_attack_seconds);
    estimator.confidence_release_seconds = optionalDouble(
        pico_ee_twist_estimator, "confidence_release_seconds",
        estimator.confidence_release_seconds);
    estimator.extra_lead_seconds = optionalDouble(
        pico_ee_twist_estimator, "extra_lead_seconds",
        estimator.extra_lead_seconds);
  }

  const YAML::Node pico_ee_feedforward_allocator =
      root["pico_ee_feedforward_allocator"];
  if (pico_ee_feedforward_allocator) {
    auto& allocator = config.pico_ee_feedforward_allocator;
    if (pico_ee_feedforward_allocator["mode"]) {
      allocator.mode = parsePicoEeFeedforwardAllocatorMode(
          pico_ee_feedforward_allocator["mode"].as<std::string>());
    }
    allocator.bisection_steps = optionalInt(
        pico_ee_feedforward_allocator, "bisection_steps",
        allocator.bisection_steps);
    allocator.max_preview_solves = optionalInt(
        pico_ee_feedforward_allocator, "max_preview_solves",
        allocator.max_preview_solves);
    allocator.maximum_scale_increase_per_cycle = optionalDouble(
        pico_ee_feedforward_allocator, "maximum_scale_increase_per_cycle",
        allocator.maximum_scale_increase_per_cycle);
    allocator.maximum_soft_bound_usage = optionalDouble(
        pico_ee_feedforward_allocator, "maximum_soft_bound_usage",
        allocator.maximum_soft_bound_usage);
    allocator.task_scale_tolerance = optionalDouble(
        pico_ee_feedforward_allocator, "task_scale_tolerance",
        allocator.task_scale_tolerance);
    allocator.normalized_residual_tolerance = optionalDouble(
        pico_ee_feedforward_allocator, "normalized_residual_tolerance",
        allocator.normalized_residual_tolerance);
    allocator.normalized_residual_floor = optionalDouble(
        pico_ee_feedforward_allocator, "normalized_residual_floor",
        allocator.normalized_residual_floor);
    allocator.arm_angle_residual_tolerance = optionalDouble(
        pico_ee_feedforward_allocator, "arm_angle_residual_tolerance",
        allocator.arm_angle_residual_tolerance);
    allocator.maximum_nullspace_leakage = optionalDouble(
        pico_ee_feedforward_allocator, "maximum_nullspace_leakage",
        allocator.maximum_nullspace_leakage);
    allocator.preview_budget_seconds_per_arm = optionalDouble(
        pico_ee_feedforward_allocator, "preview_budget_seconds_per_arm",
        allocator.preview_budget_seconds_per_arm);
    allocator.mismatch_p99_tolerance_rad_s = optionalDouble(
        pico_ee_feedforward_allocator, "mismatch_p99_tolerance_rad_s",
        allocator.mismatch_p99_tolerance_rad_s);
    allocator.mismatch_max_tolerance_rad_s = optionalDouble(
        pico_ee_feedforward_allocator, "mismatch_max_tolerance_rad_s",
        allocator.mismatch_max_tolerance_rad_s);
  }

  const YAML::Node pico_ee_task_allocator = root["pico_ee_task_allocator"];
  if (pico_ee_task_allocator) {
    auto& allocator = config.pico_ee_task_allocator;
    if (pico_ee_task_allocator["mode"]) {
      allocator.mode = parsePicoEeTaskAllocationMode(
          pico_ee_task_allocator["mode"].as<std::string>());
    }
    allocator.preserve_feedback = optionalBool(
        pico_ee_task_allocator, "preserve_feedback",
        allocator.preserve_feedback);
    allocator.nullspace_only = optionalBool(
        pico_ee_task_allocator, "nullspace_only", allocator.nullspace_only);
    allocator.nullspace_search_enabled = optionalBool(
        pico_ee_task_allocator, "nullspace_search_enabled",
        allocator.nullspace_search_enabled);
    allocator.nullspace_search_step = optionalDouble(
        pico_ee_task_allocator, "nullspace_search_step",
        allocator.nullspace_search_step);
    allocator.nullspace_reference_weight = optionalDouble(
        pico_ee_task_allocator, "nullspace_reference_weight",
        allocator.nullspace_reference_weight);
    allocator.nullspace_continuity_weight = optionalDouble(
        pico_ee_task_allocator, "nullspace_continuity_weight",
        allocator.nullspace_continuity_weight);
    allocator.nullspace_jerk_weight = optionalDouble(
        pico_ee_task_allocator, "nullspace_jerk_weight",
        allocator.nullspace_jerk_weight);
    allocator.nullspace_arm_rate_weight = optionalDouble(
        pico_ee_task_allocator, "nullspace_arm_rate_weight",
        allocator.nullspace_arm_rate_weight);
    allocator.bisection_steps = optionalInt(
        pico_ee_task_allocator, "bisection_steps",
        allocator.bisection_steps);
    allocator.max_preview_solves = optionalInt(
        pico_ee_task_allocator, "max_preview_solves",
        allocator.max_preview_solves);
    allocator.minimum_task_scale = optionalDouble(
        pico_ee_task_allocator, "minimum_task_scale",
        allocator.minimum_task_scale);
    allocator.minimum_preview_task_scale = optionalDouble(
        pico_ee_task_allocator, "minimum_preview_task_scale",
        allocator.minimum_preview_task_scale);
    allocator.maximum_soft_bound_usage = optionalDouble(
        pico_ee_task_allocator, "maximum_soft_bound_usage",
        allocator.maximum_soft_bound_usage);
    allocator.normalized_residual_tolerance = optionalDouble(
        pico_ee_task_allocator, "normalized_residual_tolerance",
        allocator.normalized_residual_tolerance);
    allocator.preview_budget_seconds_per_arm = optionalDouble(
        pico_ee_task_allocator, "preview_budget_seconds_per_arm",
        allocator.preview_budget_seconds_per_arm);
  }

  const YAML::Node mapped_vector =
      root["pico_mapped_arm_angle_vector_nullspace"];
  if (mapped_vector) {
    auto& vector_config = config.pico_mapped_arm_angle_vector_nullspace;
    auto& governor = vector_config.governor;
    if (mapped_vector["mode"]) {
      vector_config.mode = parsePicoMappedArmAngleVectorMode(
          mapped_vector["mode"].as<std::string>());
    }
    if (mapped_vector["qp_weight_mode"]) {
      vector_config.qp_weight_mode = parsePicoMappedVectorQpWeightMode(
          mapped_vector["qp_weight_mode"].as<std::string>());
    }
    vector_config.qp_tracking_weight = optionalDouble(
        mapped_vector, "qp_tracking_weight",
        vector_config.qp_tracking_weight);
    governor.observability_min = optionalDouble(
        mapped_vector, "observability_min", governor.observability_min);
    governor.maximum_cartesian_leakage = optionalDouble(
        mapped_vector, "maximum_cartesian_leakage",
        governor.maximum_cartesian_leakage);
    governor.maximum_velocity_rad_s = optionalDouble(
        mapped_vector, "maximum_velocity_rad_s",
        governor.maximum_velocity_rad_s);
    governor.maximum_acceleration_rad_s2 = optionalDouble(
        mapped_vector, "maximum_acceleration_rad_s2",
        governor.maximum_acceleration_rad_s2);
    governor.maximum_jerk_rad_s3 = optionalDouble(
        mapped_vector, "maximum_jerk_rad_s3",
        governor.maximum_jerk_rad_s3);
    governor.reference_weight = optionalDouble(
        mapped_vector, "reference_weight", governor.reference_weight);
    governor.continuity_weight = optionalDouble(
        mapped_vector, "continuity_weight", governor.continuity_weight);
    governor.acceleration_weight = optionalDouble(
        mapped_vector, "acceleration_weight", governor.acceleration_weight);
    governor.jerk_weight = optionalDouble(
        mapped_vector, "jerk_weight", governor.jerk_weight);
    vector_config.qp_jerk_weight = optionalDouble(
        mapped_vector, "qp_jerk_weight", vector_config.qp_jerk_weight);
    vector_config.qp_continuity_weight = optionalDouble(
        mapped_vector, "qp_continuity_weight",
        vector_config.qp_continuity_weight);
    governor.basis_rotation_full_health_rad_s = optionalDouble(
        mapped_vector, "basis_rotation_full_health_rad_s",
        governor.basis_rotation_full_health_rad_s);
    governor.basis_rotation_zero_health_rad_s = optionalDouble(
        mapped_vector, "basis_rotation_zero_health_rad_s",
        governor.basis_rotation_zero_health_rad_s);
    governor.basis_transport_enabled = optionalBool(
        mapped_vector, "basis_transport_enabled",
        governor.basis_transport_enabled);
    governor.basis_transport_weight = optionalDouble(
        mapped_vector, "basis_transport_weight",
        governor.basis_transport_weight);
    governor.reversal_deadband_rad_s = optionalDouble(
        mapped_vector, "reversal_deadband_rad_s",
        governor.reversal_deadband_rad_s);
  }

  const YAML::Node dls_posture_ruckig = root["dls_posture_ruckig"];
  if (dls_posture_ruckig) {
    config.dls_posture_ruckig.enabled = optionalBool(
        dls_posture_ruckig, "enabled",
        config.dls_posture_ruckig.enabled);
    config.dls_posture_ruckig.velocity_scale = optionalDouble(
        dls_posture_ruckig, "velocity_scale",
        config.dls_posture_ruckig.velocity_scale);
    config.dls_posture_ruckig.max_velocity_rad_s = optionalVector7(
        dls_posture_ruckig, "max_velocity_rad_s",
        config.dls_posture_ruckig.max_velocity_rad_s);
    config.dls_posture_ruckig.max_acceleration_rad_s2 = optionalVector7(
        dls_posture_ruckig, "max_acceleration_rad_s2",
        config.dls_posture_ruckig.max_acceleration_rad_s2);
    config.dls_posture_ruckig.max_jerk_rad_s3 = optionalVector7(
        dls_posture_ruckig, "max_jerk_rad_s3",
        config.dls_posture_ruckig.max_jerk_rad_s3);
    config.dls_posture_ruckig.validation_tolerance = optionalDouble(
        dls_posture_ruckig, "validation_tolerance",
        config.dls_posture_ruckig.validation_tolerance);
  }

  const YAML::Node acceleration_limits = root["joint_acceleration_limits"];
  if (acceleration_limits) {
    config.joint_acceleration_limits.margin_rad = optionalDouble(
        acceleration_limits, "margin_rad",
        config.joint_acceleration_limits.margin_rad);
    config.joint_acceleration_limits.velocity_scale = optionalDouble(
        acceleration_limits, "velocity_scale",
        config.joint_acceleration_limits.velocity_scale);
    config.joint_acceleration_limits.max_acceleration_rad_s2 = optionalVector7(
        acceleration_limits, "max_acceleration_rad_s2",
        config.joint_acceleration_limits.max_acceleration_rad_s2);
    config.joint_acceleration_limits.braking_acceleration_rad_s2 = optionalVector7(
        acceleration_limits, "braking_acceleration_rad_s2",
        config.joint_acceleration_limits.braking_acceleration_rad_s2);
    config.joint_acceleration_limits.hard_jerk_enabled = optionalBool(
        acceleration_limits, "hard_jerk_enabled",
        config.joint_acceleration_limits.hard_jerk_enabled);
    config.joint_acceleration_limits.max_jerk_rad_s3 = optionalVector7(
        acceleration_limits, "max_jerk_rad_s3",
        config.joint_acceleration_limits.max_jerk_rad_s3);
  }

  const YAML::Node qp = root["qp"];
  config.qp.solver = parseSolver(required<std::string>(qp, "solver"));
  config.qp.position_weight = required<double>(qp, "position_weight");
  config.qp.orientation_weight = required<double>(qp, "orientation_weight");
  config.qp.regularization = required<double>(qp, "regularization");
  config.qp.nominal_weight = required<double>(qp, "nominal_weight");
  config.qp.nominal_gain = required<double>(qp, "nominal_gain");

  const YAML::Node hierarchical_qp = root["hierarchical_qp"];
  config.hierarchical_qp.lambda_reg = required<double>(hierarchical_qp, "lambda_reg");
  config.hierarchical_qp.posture_weight =
      required<double>(hierarchical_qp, "posture_weight");
  config.hierarchical_qp.wrist_posture_weight_scale = optionalDouble(
      hierarchical_qp, "wrist_posture_weight_scale",
      config.hierarchical_qp.wrist_posture_weight_scale);
  config.hierarchical_qp.continuity_weight =
      required<double>(hierarchical_qp, "continuity_weight");
  config.hierarchical_qp.jerk_weight = optionalDouble(
      hierarchical_qp, "jerk_weight",
      config.hierarchical_qp.jerk_weight);
  if (hierarchical_qp["smoothness_mode"]) {
    config.hierarchical_qp.smoothness_mode = parseVelocityQpSmoothnessMode(
        hierarchical_qp["smoothness_mode"].as<std::string>());
  }
  config.hierarchical_qp.full_acceleration_weight = optionalDouble(
      hierarchical_qp, "full_acceleration_weight",
      config.hierarchical_qp.full_acceleration_weight);
  config.hierarchical_qp.full_jerk_weight = optionalDouble(
      hierarchical_qp, "full_jerk_weight",
      config.hierarchical_qp.full_jerk_weight);
  config.hierarchical_qp.full_acceleration_scale_rad_s2 = optionalVector7(
      hierarchical_qp, "full_acceleration_scale_rad_s2",
      config.hierarchical_qp.full_acceleration_scale_rad_s2);
  config.hierarchical_qp.full_jerk_scale_rad_s3 = optionalVector7(
      hierarchical_qp, "full_jerk_scale_rad_s3",
      config.hierarchical_qp.full_jerk_scale_rad_s3);
  config.hierarchical_qp.nullspace_acceleration_weight = optionalDouble(
      hierarchical_qp, "nullspace_acceleration_weight",
      config.hierarchical_qp.nullspace_acceleration_weight);
  config.hierarchical_qp.nullspace_jerk_weight = optionalDouble(
      hierarchical_qp, "nullspace_jerk_weight",
      config.hierarchical_qp.nullspace_jerk_weight);
  config.hierarchical_qp.nullspace_acceleration_scale_rad_s2 = optionalDouble(
      hierarchical_qp, "nullspace_acceleration_scale_rad_s2",
      config.hierarchical_qp.nullspace_acceleration_scale_rad_s2);
  config.hierarchical_qp.nullspace_jerk_scale_rad_s3 = optionalDouble(
      hierarchical_qp, "nullspace_jerk_scale_rad_s3",
      config.hierarchical_qp.nullspace_jerk_scale_rad_s3);
  config.hierarchical_qp.nominal_gain = required<double>(hierarchical_qp, "nominal_gain");
  config.hierarchical_qp.arm_angle_weight = optionalDouble(
      hierarchical_qp, "arm_angle_weight",
      config.hierarchical_qp.arm_angle_weight);
  config.hierarchical_qp.task_scaling_enabled = optionalBool(
      hierarchical_qp, "task_scaling_enabled",
      config.hierarchical_qp.task_scaling_enabled);
  config.hierarchical_qp.task_scaling_min_position = optionalDouble(
      hierarchical_qp, "task_scaling_min_position",
      config.hierarchical_qp.task_scaling_min_position);
  config.hierarchical_qp.task_scaling_min_orientation = optionalDouble(
      hierarchical_qp, "task_scaling_min_orientation",
      config.hierarchical_qp.task_scaling_min_orientation);
  config.hierarchical_qp.task_scaling_weight_position = optionalDouble(
      hierarchical_qp, "task_scaling_weight_position",
      config.hierarchical_qp.task_scaling_weight_position);
  config.hierarchical_qp.task_scaling_weight_orientation = optionalDouble(
      hierarchical_qp, "task_scaling_weight_orientation",
      config.hierarchical_qp.task_scaling_weight_orientation);
  config.hierarchical_qp.slack_weight_position =
      required<double>(hierarchical_qp, "slack_weight_position");
  config.hierarchical_qp.slack_weight_orientation =
      required<double>(hierarchical_qp, "slack_weight_orientation");
  config.hierarchical_qp.slack_position_scale = optionalDouble(
      hierarchical_qp, "slack_position_scale",
      config.hierarchical_qp.slack_position_scale);
  config.hierarchical_qp.slack_orientation_scale = optionalDouble(
      hierarchical_qp, "slack_orientation_scale",
      config.hierarchical_qp.slack_orientation_scale);
  config.hierarchical_qp.equality_tolerance =
      required<double>(hierarchical_qp, "equality_tolerance");

  const YAML::Node dls = root["dls"];
  config.dls.damping = required<double>(dls, "damping");
  config.dls.nominal_gain = required<double>(dls, "nominal_gain");

  const YAML::Node limits = root["joint_limits"];
  config.joint_limits.margin_rad = required<double>(limits, "margin_rad");
  config.joint_limits.velocity_scale = required<double>(limits, "velocity_scale");
  config.joint_limits.max_acceleration_rad_s2 = optionalVector7(
      limits, "max_acceleration_rad_s2",
      config.joint_limits.max_acceleration_rad_s2);
  config.joint_limits.braking_acceleration_rad_s2 = optionalVector7(
      limits, "braking_acceleration_rad_s2",
      config.joint_limits.braking_acceleration_rad_s2);
  config.joint_limits.hard_jerk_enabled = optionalBool(
      limits, "hard_jerk_enabled",
      config.joint_limits.hard_jerk_enabled);
  config.joint_limits.max_jerk_rad_s3 = optionalVector7(
      limits, "max_jerk_rad_s3",
      config.joint_limits.max_jerk_rad_s3);

  const YAML::Node osqp = root["osqp"];
  config.osqp.max_iterations = required<int>(osqp, "max_iterations");
  config.osqp.absolute_tolerance = required<double>(osqp, "absolute_tolerance");
  config.osqp.relative_tolerance = required<double>(osqp, "relative_tolerance");
  config.osqp.polishing = required<bool>(osqp, "polishing");

  const YAML::Node qpoases = root["qpoases"];
  config.qpoases.max_working_set_recalculations =
      required<int>(qpoases, "max_working_set_recalculations");
  config.qpoases.cpu_time_limit_seconds = required<double>(qpoases, "cpu_time_limit_seconds");

  const YAML::Node safety = root["safety"];
  config.safety.bound_tolerance = required<double>(safety, "bound_tolerance");
  config.safety.hessian_eigenvalue_tolerance =
      required<double>(safety, "hessian_eigenvalue_tolerance");
  config.safety.max_target_position_step = required<double>(safety, "max_target_position_step");
  config.safety.max_target_orientation_step =
      required<double>(safety, "max_target_orientation_step");
  config.safety.reference_tracking_warn_rad = optionalDouble(
      safety, "reference_tracking_warn_rad",
      config.safety.reference_tracking_warn_rad);
  config.safety.reference_tracking_stop_rad = optionalDouble(
      safety, "reference_tracking_stop_rad",
      config.safety.reference_tracking_stop_rad);
  config.safety.velocity_reference_tracking_warn_rad_s = optionalDouble(
      safety, "velocity_reference_tracking_warn_rad_s",
      config.safety.velocity_reference_tracking_warn_rad_s);
  config.safety.velocity_reference_tracking_stop_rad_s = optionalDouble(
      safety, "velocity_reference_tracking_stop_rad_s",
      config.safety.velocity_reference_tracking_stop_rad_s);

  const YAML::Node trajectories = root["trajectories"];
  config.trajectories.circle_radius = required<double>(trajectories, "circle_radius");
  config.trajectories.figure_eight_width = required<double>(trajectories, "figure_eight_width");
  config.trajectories.figure_eight_height = required<double>(trajectories, "figure_eight_height");
  config.trajectories.angular_amplitude = required<double>(trajectories, "angular_amplitude");
  config.trajectories.frequency_hz = required<double>(trajectories, "frequency_hz");

  requirePositive(config.controller.rate_hz, "controller.rate_hz");
  for (int index = 0; index < 3; ++index) {
    requirePositive(config.cartesian_servo.kp_position[index], "cartesian_servo.kp_position");
    requirePositive(config.cartesian_servo.kp_orientation[index],
                    "cartesian_servo.kp_orientation");
    requirePositive(config.cartesian_servo.kp_position_near[index],
                    "cartesian_servo.kp_position_near");
    requirePositive(config.cartesian_servo.kp_orientation_near[index],
                    "cartesian_servo.kp_orientation_near");
    requirePositive(config.cartesian_servo.kp_position_motion[index],
                    "cartesian_servo.kp_position_motion");
    requirePositive(config.cartesian_servo.kp_orientation_motion[index],
                    "cartesian_servo.kp_orientation_motion");
    if (config.cartesian_servo.kp_position_motion[index] <
            config.cartesian_servo.kp_position_near[index] ||
        config.cartesian_servo.kp_position_motion[index] >
            config.cartesian_servo.kp_position[index] ||
        config.cartesian_servo.kp_orientation_motion[index] <
            config.cartesian_servo.kp_orientation_near[index] ||
        config.cartesian_servo.kp_orientation_motion[index] >
            config.cartesian_servo.kp_orientation[index]) {
      throw std::runtime_error(
          "Cartesian motion gains must lie between near and far gains");
    }
  }
  requireNonNegative(config.cartesian_servo.position_gain_transition_start_m,
                     "cartesian_servo.position_gain_transition_start_m");
  requirePositive(config.cartesian_servo.position_gain_transition_end_m,
                  "cartesian_servo.position_gain_transition_end_m");
  if (config.cartesian_servo.position_gain_transition_start_m >=
      config.cartesian_servo.position_gain_transition_end_m) {
    throw std::runtime_error(
        "Cartesian position gain transition start must be below end");
  }
  requireNonNegative(
      config.cartesian_servo.orientation_gain_transition_start_rad,
      "cartesian_servo.orientation_gain_transition_start_rad");
  requirePositive(config.cartesian_servo.orientation_gain_transition_end_rad,
                  "cartesian_servo.orientation_gain_transition_end_rad");
  if (config.cartesian_servo.orientation_gain_transition_start_rad >=
      config.cartesian_servo.orientation_gain_transition_end_rad) {
    throw std::runtime_error(
        "Cartesian orientation gain transition start must be below end");
  }
  requireNonNegative(config.cartesian_servo.kff_linear,
                     "cartesian_servo.kff_linear");
  requireNonNegative(config.cartesian_servo.kff_angular,
                     "cartesian_servo.kff_angular");
  requirePositive(config.cartesian_servo.feedforward_filter_cutoff_hz,
                  "cartesian_servo.feedforward_filter_cutoff_hz");
  requirePositive(config.cartesian_servo.prediction_horizon_seconds,
                  "cartesian_servo.prediction_horizon_seconds");
  requirePositive(config.cartesian_servo.target_timeout_seconds,
                  "cartesian_servo.target_timeout_seconds");
  if (config.cartesian_servo.prediction_horizon_seconds >
      config.cartesian_servo.target_timeout_seconds) {
    throw std::runtime_error(
        "cartesian_servo.prediction_horizon_seconds must not exceed target timeout");
  }
  requirePositive(config.cartesian_servo.max_linear_velocity,
                  "cartesian_servo.max_linear_velocity");
  requirePositive(config.cartesian_servo.max_angular_velocity,
                  "cartesian_servo.max_angular_velocity");
  requirePositive(config.pico_teleop.max_position_jump_m,
                  "pico_teleop.max_position_jump_m");
  requirePositive(config.pico_teleop.max_orientation_jump_rad,
                  "pico_teleop.max_orientation_jump_rad");
  requireNonNegative(config.arm_angle.kp_velocity, "arm_angle.kp_velocity");
  requirePositive(config.arm_angle.max_velocity_rad_s,
                  "arm_angle.max_velocity_rad_s");
  requireNonNegative(config.arm_angle.kp_acceleration,
                     "arm_angle.kp_acceleration");
  requireNonNegative(config.arm_angle.kd_acceleration,
                     "arm_angle.kd_acceleration");
  requirePositive(config.arm_angle.max_acceleration_rad_s2,
                  "arm_angle.max_acceleration_rad_s2");
  requirePositive(config.arm_angle.minimum_radius_m,
                  "arm_angle.minimum_radius_m");
  requirePositive(config.arm_angle.full_weight_radius_m,
                  "arm_angle.full_weight_radius_m");
  if (config.arm_angle.full_weight_radius_m <=
      config.arm_angle.minimum_radius_m) {
    throw std::runtime_error(
        "arm_angle.full_weight_radius_m must exceed minimum_radius_m");
  }
  requirePositive(config.arm_angle.branch_lock_radius_m,
                  "arm_angle.branch_lock_radius_m");
  if (config.arm_angle.branch_lock_radius_m <
          config.arm_angle.minimum_radius_m ||
      config.arm_angle.branch_lock_radius_m >
          config.arm_angle.full_weight_radius_m) {
    throw std::runtime_error(
        "arm_angle.branch_lock_radius_m must be within "
        "[minimum_radius_m, full_weight_radius_m]");
  }
  requirePositive(config.arm_angle.reference_rate_limit_rad_s,
                  "arm_angle.reference_rate_limit_rad_s");
  requirePositive(config.arm_angle.reference_projection_hold_enter,
                  "arm_angle.reference_projection_hold_enter");
  requirePositive(config.arm_angle.reference_projection_hold_exit,
                  "arm_angle.reference_projection_hold_exit");
  if (config.arm_angle.reference_projection_hold_enter >=
          config.arm_angle.reference_projection_hold_exit ||
      config.arm_angle.reference_projection_hold_exit > 1.0) {
    throw std::runtime_error(
        "arm_angle reference projection thresholds must satisfy "
        "0 < hold_enter < hold_exit <= 1");
  }
  requirePositive(config.arm_angle.error_branch_hysteresis_rad,
                  "arm_angle.error_branch_hysteresis_rad");
  if (config.arm_angle.error_branch_hysteresis_rad >=
      3.14159265358979323846) {
    throw std::runtime_error(
        "arm_angle.error_branch_hysteresis_rad must be smaller than pi");
  }
  requirePositive(config.arm_angle.reference_governor_enter_error_rad,
                  "arm_angle.reference_governor_enter_error_rad");
  requirePositive(config.arm_angle.reference_governor_exit_error_rad,
                  "arm_angle.reference_governor_exit_error_rad");
  requirePositive(config.arm_angle.reference_governor_tracking_error_rad,
                  "arm_angle.reference_governor_tracking_error_rad");
  if (config.arm_angle.reference_governor_exit_error_rad >=
          config.arm_angle.reference_governor_enter_error_rad ||
      config.arm_angle.reference_governor_tracking_error_rad >
          config.arm_angle.reference_governor_exit_error_rad ||
      config.arm_angle.reference_governor_enter_error_rad >=
          3.14159265358979323846) {
    throw std::runtime_error(
        "arm_angle reference governor thresholds must satisfy "
        "0 < tracking_error <= exit_error < enter_error < pi");
  }
  requirePositive(config.arm_angle.tracking_envelope_max_error_rad,
                  "arm_angle.tracking_envelope_max_error_rad");
  if (config.arm_angle.tracking_envelope_max_error_rad >=
      3.14159265358979323846) {
    throw std::runtime_error(
        "arm_angle.tracking_envelope_max_error_rad must be smaller than pi");
  }
  requirePositive(config.arm_angle.tracking_envelope_gain,
                  "arm_angle.tracking_envelope_gain");
  requireNonNegative(config.arm_angle.continuity_kp_velocity,
                     "arm_angle.continuity_kp_velocity");
  requirePositive(config.arm_angle.continuity_max_velocity_rad_s,
                  "arm_angle.continuity_max_velocity_rad_s");
  requireNonNegative(config.arm_angle.continuity_kp_acceleration,
                     "arm_angle.continuity_kp_acceleration");
  requireNonNegative(config.arm_angle.continuity_kd_acceleration,
                     "arm_angle.continuity_kd_acceleration");
  requirePositive(config.arm_angle.continuity_max_acceleration_rad_s2,
                  "arm_angle.continuity_max_acceleration_rad_s2");
  requirePositive(config.arm_angle.continuity_weight_scale,
                  "arm_angle.continuity_weight_scale");
  requirePositive(config.arm_angle.joint_limit_soft_margin_rad,
                  "arm_angle.joint_limit_soft_margin_rad");
  requirePositive(config.arm_angle.joint_limit_recovery_gain_rad_s_per_rad,
                  "arm_angle.joint_limit_recovery_gain_rad_s_per_rad");
  requireNonNegative(config.arm_angle.branch_lock_kp_velocity,
                     "arm_angle.branch_lock_kp_velocity");
  requireNonNegative(config.arm_angle.branch_lock_kp_acceleration,
                     "arm_angle.branch_lock_kp_acceleration");
  requireNonNegative(config.arm_angle.branch_lock_kd_acceleration,
                     "arm_angle.branch_lock_kd_acceleration");
  requireNonNegative(
      config.upper_arm_outward.minimum_outward_distance_m,
      "upper_arm_outward.minimum_outward_distance_m");
  requirePositive(config.upper_arm_outward.velocity_gain,
                  "upper_arm_outward.velocity_gain");
  requireNonNegative(config.upper_arm_outward.acceleration_kp,
                     "upper_arm_outward.acceleration_kp");
  requireNonNegative(config.upper_arm_outward.acceleration_kd,
                     "upper_arm_outward.acceleration_kd");
  requirePositive(config.cartesian_otg.translation_velocity_max,
                  "cartesian_otg.translation_velocity_max");
  requirePositive(config.cartesian_otg.translation_tracking_gain,
                  "cartesian_otg.translation_tracking_gain");
  requirePositive(
      config.cartesian_otg.translation_stationary_velocity_threshold,
      "cartesian_otg.translation_stationary_velocity_threshold");
  requirePositive(config.cartesian_otg.translation_prediction_horizon_seconds,
                  "cartesian_otg.translation_prediction_horizon_seconds");
  if (config.cartesian_otg.translation_prediction_horizon_seconds >
      config.cartesian_servo.target_timeout_seconds) {
    throw std::runtime_error(
        "cartesian_otg.translation_prediction_horizon_seconds must not "
        "exceed target timeout");
  }
  requirePositive(config.cartesian_otg.stationary_hold_dwell_seconds,
                  "cartesian_otg.stationary_hold_dwell_seconds");
  requirePositive(config.cartesian_otg.translation_hold_enter_velocity_m_s,
                  "cartesian_otg.translation_hold_enter_velocity_m_s");
  requirePositive(config.cartesian_otg.translation_hold_exit_velocity_m_s,
                  "cartesian_otg.translation_hold_exit_velocity_m_s");
  if (config.cartesian_otg.translation_hold_enter_velocity_m_s >=
      config.cartesian_otg.translation_hold_exit_velocity_m_s) {
    throw std::runtime_error(
        "cartesian_otg.translation_hold_enter_velocity_m_s must be less "
        "than translation_hold_exit_velocity_m_s");
  }
  requirePositive(
      config.cartesian_otg.translation_hold_exit_position_error_m,
      "cartesian_otg.translation_hold_exit_position_error_m");
  requirePositive(config.cartesian_otg.orientation_hold_enter_velocity_rad_s,
                  "cartesian_otg.orientation_hold_enter_velocity_rad_s");
  requirePositive(config.cartesian_otg.orientation_hold_exit_velocity_rad_s,
                  "cartesian_otg.orientation_hold_exit_velocity_rad_s");
  if (config.cartesian_otg.orientation_hold_enter_velocity_rad_s >=
      config.cartesian_otg.orientation_hold_exit_velocity_rad_s) {
    throw std::runtime_error(
        "cartesian_otg.orientation_hold_enter_velocity_rad_s must be less "
        "than orientation_hold_exit_velocity_rad_s");
  }
  requirePositive(config.cartesian_otg.orientation_hold_exit_error_rad,
                  "cartesian_otg.orientation_hold_exit_error_rad");
  requirePositive(config.cartesian_otg.translation_acceleration_max,
                  "cartesian_otg.translation_acceleration_max");
  requirePositive(config.cartesian_otg.translation_jerk_max,
                  "cartesian_otg.translation_jerk_max");
  requirePositive(config.cartesian_otg.angular_velocity_max,
                  "cartesian_otg.angular_velocity_max");
  requirePositive(config.cartesian_otg.angular_acceleration_max,
                  "cartesian_otg.angular_acceleration_max");
  requirePositive(config.cartesian_otg.angular_jerk_max,
                  "cartesian_otg.angular_jerk_max");
  requirePositive(config.cartesian_otg.orientation_gain,
                  "cartesian_otg.orientation_gain");
  requirePositive(config.cartesian_otg.orientation_settle_error_rad,
                  "cartesian_otg.orientation_settle_error_rad");
  requirePositive(config.cartesian_otg.angular_settle_velocity_rad_s,
                  "cartesian_otg.angular_settle_velocity_rad_s");
  requirePositive(config.cartesian_otg.angular_settle_acceleration_rad_s2,
                  "cartesian_otg.angular_settle_acceleration_rad_s2");
  validateDynamicLimits(config.cartesian_otg_control_profiles.velocity,
                        "cartesian_otg_control_profiles.velocity");
  validateDynamicLimits(config.cartesian_otg_control_profiles.acceleration,
                        "cartesian_otg_control_profiles.acceleration");
  requirePositive(config.cartesian_acceleration.kp_position,
                  "cartesian_acceleration.kp_position");
  requirePositive(config.cartesian_acceleration.kd_position,
                  "cartesian_acceleration.kd_position");
  requirePositive(config.cartesian_acceleration.kp_orientation,
                  "cartesian_acceleration.kp_orientation");
  requirePositive(config.cartesian_acceleration.kd_orientation,
                  "cartesian_acceleration.kd_orientation");
  requirePositive(config.cartesian_acceleration.linear_limit,
                  "cartesian_acceleration.linear_limit");
  requirePositive(config.cartesian_acceleration.angular_limit,
                  "cartesian_acceleration.angular_limit");
  requirePositive(config.acceleration_qp.slack_weight_position,
                  "acceleration_qp.slack_weight_position");
  requirePositive(config.acceleration_qp.slack_weight_orientation,
                  "acceleration_qp.slack_weight_orientation");
  requirePositive(config.acceleration_qp.slack_linear_scale,
                  "acceleration_qp.slack_linear_scale");
  requirePositive(config.acceleration_qp.slack_angular_scale,
                  "acceleration_qp.slack_angular_scale");
  requirePositive(config.acceleration_qp.regularization,
                  "acceleration_qp.regularization");
  requirePositive(config.acceleration_qp.jerk_weight,
                  "acceleration_qp.jerk_weight");
  requirePositive(config.acceleration_qp.posture_weight,
                  "acceleration_qp.posture_weight");
  requirePositive(config.acceleration_qp.posture_kp,
                  "acceleration_qp.posture_kp");
  requirePositive(config.acceleration_qp.posture_kd,
                  "acceleration_qp.posture_kd");
  requireNonNegative(config.acceleration_qp.arm_angle_weight,
                     "acceleration_qp.arm_angle_weight");
  requireNonNegative(config.acceleration_qp.task_scaling_min_position,
                     "acceleration_qp.task_scaling_min_position");
  requireNonNegative(config.acceleration_qp.task_scaling_min_orientation,
                     "acceleration_qp.task_scaling_min_orientation");
  if (config.acceleration_qp.task_scaling_min_position > 1.0 ||
      config.acceleration_qp.task_scaling_min_orientation > 1.0) {
    throw std::runtime_error(
        "acceleration_qp task scaling minima must be within [0, 1]");
  }
  requirePositive(config.acceleration_qp.task_scaling_weight_position,
                  "acceleration_qp.task_scaling_weight_position");
  requirePositive(config.acceleration_qp.task_scaling_weight_orientation,
                  "acceleration_qp.task_scaling_weight_orientation");
  requirePositive(config.acceleration_qp.equality_tolerance,
                  "acceleration_qp.equality_tolerance");
  requirePositive(config.joint_acceleration_limits.margin_rad,
                  "joint_acceleration_limits.margin_rad");
  requirePositive(config.joint_acceleration_limits.max_acceleration_rad_s2,
                  "joint_acceleration_limits.max_acceleration_rad_s2");
  requirePositive(config.joint_acceleration_limits.braking_acceleration_rad_s2,
                  "joint_acceleration_limits.braking_acceleration_rad_s2");
  requirePositive(config.joint_acceleration_limits.max_jerk_rad_s3,
                  "joint_acceleration_limits.max_jerk_rad_s3");
  if (!std::isfinite(config.joint_acceleration_limits.velocity_scale) ||
      config.joint_acceleration_limits.velocity_scale <= 0.0 ||
      config.joint_acceleration_limits.velocity_scale > 1.0) {
    throw std::runtime_error(
        "joint_acceleration_limits.velocity_scale must be in (0, 1]");
  }
  requirePositive(config.qp.position_weight, "qp.position_weight");
  requirePositive(config.qp.orientation_weight, "qp.orientation_weight");
  requirePositive(config.qp.regularization, "qp.regularization");
  requireNonNegative(config.qp.nominal_weight, "qp.nominal_weight");
  requireNonNegative(config.qp.nominal_gain, "qp.nominal_gain");
  requirePositive(config.hierarchical_qp.lambda_reg, "hierarchical_qp.lambda_reg");
  requireNonNegative(config.hierarchical_qp.posture_weight,
                     "hierarchical_qp.posture_weight");
  requirePositive(config.hierarchical_qp.wrist_posture_weight_scale,
                  "hierarchical_qp.wrist_posture_weight_scale");
  requireNonNegative(config.hierarchical_qp.continuity_weight,
                     "hierarchical_qp.continuity_weight");
  requireNonNegative(config.hierarchical_qp.jerk_weight,
                     "hierarchical_qp.jerk_weight");
  requireNonNegative(config.hierarchical_qp.full_acceleration_weight,
                     "hierarchical_qp.full_acceleration_weight");
  requireNonNegative(config.hierarchical_qp.full_jerk_weight,
                     "hierarchical_qp.full_jerk_weight");
  requirePositive(
      config.hierarchical_qp.full_acceleration_scale_rad_s2,
      "hierarchical_qp.full_acceleration_scale_rad_s2");
  requirePositive(config.hierarchical_qp.full_jerk_scale_rad_s3,
                  "hierarchical_qp.full_jerk_scale_rad_s3");
  requireNonNegative(config.hierarchical_qp.nullspace_acceleration_weight,
                     "hierarchical_qp.nullspace_acceleration_weight");
  requireNonNegative(config.hierarchical_qp.nullspace_jerk_weight,
                     "hierarchical_qp.nullspace_jerk_weight");
  requirePositive(
      config.hierarchical_qp.nullspace_acceleration_scale_rad_s2,
      "hierarchical_qp.nullspace_acceleration_scale_rad_s2");
  requirePositive(config.hierarchical_qp.nullspace_jerk_scale_rad_s3,
                  "hierarchical_qp.nullspace_jerk_scale_rad_s3");
  requireNonNegative(config.hierarchical_qp.nominal_gain,
                     "hierarchical_qp.nominal_gain");
  requireNonNegative(config.hierarchical_qp.arm_angle_weight,
                     "hierarchical_qp.arm_angle_weight");
  requireNonNegative(config.hierarchical_qp.task_scaling_min_position,
                     "hierarchical_qp.task_scaling_min_position");
  requireNonNegative(config.hierarchical_qp.task_scaling_min_orientation,
                     "hierarchical_qp.task_scaling_min_orientation");
  if (config.hierarchical_qp.task_scaling_min_position > 1.0 ||
      config.hierarchical_qp.task_scaling_min_orientation > 1.0) {
    throw std::runtime_error(
        "hierarchical_qp task scaling minima must be within [0, 1]");
  }
  requirePositive(config.hierarchical_qp.task_scaling_weight_position,
                  "hierarchical_qp.task_scaling_weight_position");
  requirePositive(config.hierarchical_qp.task_scaling_weight_orientation,
                  "hierarchical_qp.task_scaling_weight_orientation");
  requirePositive(config.hierarchical_qp.slack_weight_position,
                  "hierarchical_qp.slack_weight_position");
  requirePositive(config.hierarchical_qp.slack_weight_orientation,
                  "hierarchical_qp.slack_weight_orientation");
  requirePositive(config.hierarchical_qp.slack_position_scale,
                  "hierarchical_qp.slack_position_scale");
  requirePositive(config.hierarchical_qp.slack_orientation_scale,
                  "hierarchical_qp.slack_orientation_scale");
  requirePositive(config.hierarchical_qp.equality_tolerance,
                  "hierarchical_qp.equality_tolerance");
  const auto& mapped_vector_config =
      config.pico_mapped_arm_angle_vector_nullspace.governor;
  requirePositive(mapped_vector_config.observability_min,
                  "pico_mapped_arm_angle_vector_nullspace.observability_min");
  requirePositive(
      mapped_vector_config.maximum_cartesian_leakage,
      "pico_mapped_arm_angle_vector_nullspace.maximum_cartesian_leakage");
  requirePositive(
      mapped_vector_config.maximum_velocity_rad_s,
      "pico_mapped_arm_angle_vector_nullspace.maximum_velocity_rad_s");
  requirePositive(
      mapped_vector_config.maximum_acceleration_rad_s2,
      "pico_mapped_arm_angle_vector_nullspace.maximum_acceleration_rad_s2");
  requirePositive(
      mapped_vector_config.maximum_jerk_rad_s3,
      "pico_mapped_arm_angle_vector_nullspace.maximum_jerk_rad_s3");
  requireNonNegative(
      mapped_vector_config.reference_weight,
      "pico_mapped_arm_angle_vector_nullspace.reference_weight");
  requireNonNegative(
      mapped_vector_config.continuity_weight,
      "pico_mapped_arm_angle_vector_nullspace.continuity_weight");
  requireNonNegative(
      mapped_vector_config.acceleration_weight,
      "pico_mapped_arm_angle_vector_nullspace.acceleration_weight");
  requireNonNegative(
      mapped_vector_config.jerk_weight,
      "pico_mapped_arm_angle_vector_nullspace.jerk_weight");
  requireNonNegative(
      config.pico_mapped_arm_angle_vector_nullspace.qp_tracking_weight,
      "pico_mapped_arm_angle_vector_nullspace.qp_tracking_weight");
  requireNonNegative(
      config.pico_mapped_arm_angle_vector_nullspace.qp_continuity_weight,
      "pico_mapped_arm_angle_vector_nullspace.qp_continuity_weight");
  requireNonNegative(
      config.pico_mapped_arm_angle_vector_nullspace.qp_jerk_weight,
      "pico_mapped_arm_angle_vector_nullspace.qp_jerk_weight");
  requirePositive(
      mapped_vector_config.basis_rotation_full_health_rad_s,
      "pico_mapped_arm_angle_vector_nullspace."
      "basis_rotation_full_health_rad_s");
  requirePositive(
      mapped_vector_config.basis_rotation_zero_health_rad_s,
      "pico_mapped_arm_angle_vector_nullspace."
      "basis_rotation_zero_health_rad_s");
  if (mapped_vector_config.basis_rotation_zero_health_rad_s <=
      mapped_vector_config.basis_rotation_full_health_rad_s) {
    throw std::runtime_error(
        "pico_mapped_arm_angle_vector_nullspace basis rotation thresholds "
        "must satisfy 0 < full_health < zero_health");
  }
  if (!std::isfinite(mapped_vector_config.basis_transport_weight) ||
      mapped_vector_config.basis_transport_weight < 0.0 ||
      mapped_vector_config.basis_transport_weight > 1.0) {
    throw std::runtime_error(
        "pico_mapped_arm_angle_vector_nullspace.basis_transport_weight "
        "must be in [0, 1]");
  }
  requireNonNegative(
      mapped_vector_config.reversal_deadband_rad_s,
      "pico_mapped_arm_angle_vector_nullspace.reversal_deadband_rad_s");
  if (config.pico_mapped_arm_angle_vector_nullspace.mode ==
          PicoMappedArmAngleVectorMode::kActive &&
      mapped_vector_config.reference_weight +
              mapped_vector_config.continuity_weight +
              mapped_vector_config.acceleration_weight +
              mapped_vector_config.jerk_weight <=
          0.0) {
    throw std::runtime_error(
        "active pico_mapped_arm_angle_vector_nullspace requires a positive "
        "objective weight");
  }
  requirePositive(config.dls.damping, "dls.damping");
  requireNonNegative(config.dls.nominal_gain, "dls.nominal_gain");
  if (config.iterative_dls.max_iterations <= 0) {
    throw std::runtime_error("iterative_dls.max_iterations must be positive");
  }
  requirePositive(config.iterative_dls.position_tolerance_m,
                  "iterative_dls.position_tolerance_m");
  requirePositive(config.iterative_dls.orientation_tolerance_rad,
                  "iterative_dls.orientation_tolerance_rad");
  requirePositive(config.iterative_dls.position_gain,
                  "iterative_dls.position_gain");
  requirePositive(config.iterative_dls.orientation_gain,
                  "iterative_dls.orientation_gain");
  requirePositive(config.iterative_dls.minimum_damping,
                  "iterative_dls.minimum_damping");
  requirePositive(config.iterative_dls.maximum_damping,
                  "iterative_dls.maximum_damping");
  if (config.iterative_dls.minimum_damping >
      config.iterative_dls.maximum_damping) {
    throw std::runtime_error(
        "iterative_dls.minimum_damping must not exceed maximum_damping");
  }
  requirePositive(config.iterative_dls.singular_value_threshold,
                  "iterative_dls.singular_value_threshold");
  requirePositive(config.iterative_dls.maximum_step_norm_rad,
                  "iterative_dls.maximum_step_norm_rad");
  requirePositive(config.iterative_dls.maximum_joint_step_rad,
                  "iterative_dls.maximum_joint_step_rad");
  requireNonNegative(config.iterative_dls.minimum_merit_improvement,
                     "iterative_dls.minimum_merit_improvement");
  requireNonNegative(config.iterative_dls.nominal_posture_gain,
                     "iterative_dls.nominal_posture_gain");
  requireNonNegative(config.iterative_dls.arm_angle_gain,
                     "iterative_dls.arm_angle_gain");
  requirePositive(config.iterative_dls.maximum_goal_step_rad,
                  "iterative_dls.maximum_goal_step_rad");
  requireNonNegative(config.iterative_dls.posture_reference_margin_rad,
                     "iterative_dls.posture_reference_margin_rad");
  if (config.iterative_dls.posture_reference_margin_rad <
      config.joint_limits.margin_rad) {
    throw std::runtime_error(
        "iterative_dls.posture_reference_margin_rad must not be smaller "
        "than joint_limits.margin_rad");
  }
  if (config.spark_upper_qpoases.calibration_frames <= 0 ||
      config.spark_upper_qpoases.stage1_max_iterations <= 0 ||
      config.spark_upper_qpoases.stage2_max_iterations <= 0) {
    throw std::runtime_error(
        "spark_upper_qpoases frame and iteration counts must be positive");
  }
  requirePositive(config.spark_upper_qpoases.minimum_scale,
                  "spark_upper_qpoases.minimum_scale");
  requirePositive(config.spark_upper_qpoases.maximum_scale,
                  "spark_upper_qpoases.maximum_scale");
  if (config.spark_upper_qpoases.minimum_scale >=
      config.spark_upper_qpoases.maximum_scale) {
    throw std::runtime_error(
        "spark_upper_qpoases.minimum_scale must be below maximum_scale");
  }
  requirePositive(config.spark_upper_qpoases.convergence_delta,
                  "spark_upper_qpoases.convergence_delta");
  requirePositive(config.spark_upper_qpoases.integration_step,
                  "spark_upper_qpoases.integration_step");
  requirePositive(config.spark_upper_qpoases.target_blend_seconds,
                  "spark_upper_qpoases.target_blend_seconds");
  requirePositive(config.spark_upper_qpoases.maximum_joint_rotation_jump_rad,
                  "spark_upper_qpoases.maximum_joint_rotation_jump_rad");
  requirePositive(config.spark_upper_qpoases.ik_cycle_budget_seconds,
                  "spark_upper_qpoases.ik_cycle_budget_seconds");
  requirePositive(config.spark_upper_qpoases.trust_region_rad,
                  "spark_upper_qpoases.trust_region_rad");
  requirePositive(config.spark_upper_qpoases.damping,
                  "spark_upper_qpoases.damping");
  requirePositive(config.spark_upper_qpoases.joint_limit_margin_rad,
                  "spark_upper_qpoases.joint_limit_margin_rad");
  requirePositive(config.spark_upper_qpoases.stage1_upper_direction_weight,
                  "spark_upper_qpoases.stage1_upper_direction_weight");
  requirePositive(config.spark_upper_qpoases.stage1_forearm_direction_weight,
                  "spark_upper_qpoases.stage1_forearm_direction_weight");
  requirePositive(config.spark_upper_qpoases.stage1_palm_orientation_weight,
                  "spark_upper_qpoases.stage1_palm_orientation_weight");
  requirePositive(config.spark_upper_qpoases.stage2_upper_direction_weight,
                  "spark_upper_qpoases.stage2_upper_direction_weight");
  requirePositive(config.spark_upper_qpoases.stage2_forearm_direction_weight,
                  "spark_upper_qpoases.stage2_forearm_direction_weight");
  requirePositive(config.spark_upper_qpoases.stage2_palm_orientation_weight,
                  "spark_upper_qpoases.stage2_palm_orientation_weight");
  requirePositive(config.spark_upper_qpoases.stage2_elbow_position_weight,
                  "spark_upper_qpoases.stage2_elbow_position_weight");
  requirePositive(config.spark_upper_qpoases.stage2_wrist_position_weight,
                  "spark_upper_qpoases.stage2_wrist_position_weight");
  requirePositive(config.spark_upper_qpoases.stage2_palm_position_weight,
                  "spark_upper_qpoases.stage2_palm_position_weight");
  requirePositive(config.spark_upper_qpoases.posture_weight,
                  "spark_upper_qpoases.posture_weight");
  requirePositive(config.spark_upper_qpoases.joint_reference_weight,
                  "spark_upper_qpoases.joint_reference_weight");
  requirePositive(config.spark_upper_qpoases.joint_reference_attack_seconds,
                  "spark_upper_qpoases.joint_reference_attack_seconds");
  requirePositive(config.spark_upper_qpoases.joint_reference_release_seconds,
                  "spark_upper_qpoases.joint_reference_release_seconds");
  requirePositive(
      config.spark_upper_qpoases.joint_reference_smoothness_weight,
      "spark_upper_qpoases.joint_reference_smoothness_weight");
  requirePositive(config.spark_upper_qpoases.posture_position_gain,
                  "spark_upper_qpoases.posture_position_gain");
  requirePositive(config.spark_upper_qpoases.otg_position_tolerance_m,
                  "spark_upper_qpoases.otg_position_tolerance_m");
  requirePositive(config.spark_upper_qpoases.otg_orientation_tolerance_rad,
                  "spark_upper_qpoases.otg_orientation_tolerance_rad");
  requireNonNegative(config.spark_upper_qpoases.otg_continuity_weight,
                     "spark_upper_qpoases.otg_continuity_weight");

  const auto& feedforward = config.spark_feedforward_velocity_qp;
  if (!std::isfinite(feedforward.alpha) || feedforward.alpha <= 0.0 ||
      feedforward.alpha > 1.0) {
    throw std::runtime_error(
        "spark_feedforward_velocity_qp.alpha must be in (0, 1]");
  }
  if (!std::isfinite(feedforward.beta) || feedforward.beta <= 0.0 ||
      feedforward.beta > 1.0) {
    throw std::runtime_error(
        "spark_feedforward_velocity_qp.beta must be in (0, 1]");
  }
  if (feedforward.dt_median_window < 3 ||
      feedforward.dt_median_window % 2 == 0) {
    throw std::runtime_error(
        "spark_feedforward_velocity_qp.dt_median_window must be odd and at least 3");
  }
  if (!std::isfinite(feedforward.dt_min_ratio) ||
      feedforward.dt_min_ratio <= 0.0 ||
      feedforward.dt_min_ratio >= 1.0) {
    throw std::runtime_error(
        "spark_feedforward_velocity_qp.dt_min_ratio must be in (0, 1)");
  }
  if (!std::isfinite(feedforward.dt_max_ratio) ||
      feedforward.dt_max_ratio <= 1.0) {
    throw std::runtime_error(
        "spark_feedforward_velocity_qp.dt_max_ratio must exceed 1");
  }
  requirePositive(feedforward.maximum_joint_jump_rad,
                  "spark_feedforward_velocity_qp.maximum_joint_jump_rad");
  requirePositive(
      feedforward.stale_velocity_decay_seconds,
      "spark_feedforward_velocity_qp.stale_velocity_decay_seconds");
  requireNonNegative(
      feedforward.source_stationary_velocity_rad_s,
      "spark_feedforward_velocity_qp.source_stationary_velocity_rad_s");
  const auto requireUnitInterval = [](double value, const char* name) {
    if (!std::isfinite(value) || value < 0.0 || value > 1.0) {
      throw std::runtime_error(std::string(name) + " must be in [0, 1]");
    }
  };
  requireUnitInterval(
      feedforward.velocity_reversal_decay,
      "spark_feedforward_velocity_qp.velocity_reversal_decay");
  requireUnitInterval(
      feedforward.velocity_stationary_decay,
      "spark_feedforward_velocity_qp.velocity_stationary_decay");
  if (!std::isfinite(feedforward.target_braking_acceleration_scale) ||
      feedforward.target_braking_acceleration_scale <= 0.0 ||
      feedforward.target_braking_acceleration_scale > 1.0) {
    throw std::runtime_error(
        "spark_feedforward_velocity_qp.target_braking_acceleration_scale "
        "must be in (0, 1]");
  }
  if (!std::isfinite(feedforward.palm_twist_filter_alpha) ||
      feedforward.palm_twist_filter_alpha <= 0.0 ||
      feedforward.palm_twist_filter_alpha > 1.0) {
    throw std::runtime_error(
        "spark_feedforward_velocity_qp.palm_twist_filter_alpha must be in (0, 1]");
  }
  requirePositive(
      feedforward.palm_twist_lowpass_cutoff_hz,
      "spark_feedforward_velocity_qp.palm_twist_lowpass_cutoff_hz");
  requirePositive(feedforward.reference_velocity_scale,
                  "spark_feedforward_velocity_qp.reference_velocity_scale");
  requirePositive(
      feedforward.reference_acceleration_scale,
      "spark_feedforward_velocity_qp.reference_acceleration_scale");
  requirePositive(feedforward.reference_jerk_scale,
                  "spark_feedforward_velocity_qp.reference_jerk_scale");
  requirePositive(feedforward.reference_position_gain,
                  "spark_feedforward_velocity_qp.reference_position_gain");
  requireNonNegative(
      feedforward.position_feedforward_gain,
      "spark_feedforward_velocity_qp.position_feedforward_gain");
  requireNonNegative(
      feedforward.orientation_feedforward_gain,
      "spark_feedforward_velocity_qp.orientation_feedforward_gain");
  requireNonNegative(
      feedforward.position_high_frequency_feedforward_gain,
      "spark_feedforward_velocity_qp.position_high_frequency_feedforward_gain");
  requireNonNegative(
      feedforward.orientation_high_frequency_feedforward_gain,
      "spark_feedforward_velocity_qp.orientation_high_frequency_feedforward_gain");
  requirePositive(feedforward.joint_position_gain,
                  "spark_feedforward_velocity_qp.joint_position_gain");
  requirePositive(feedforward.attack_seconds,
                  "spark_feedforward_velocity_qp.attack_seconds");
  requirePositive(feedforward.release_seconds,
                  "spark_feedforward_velocity_qp.release_seconds");

  const auto& headroom = config.spark_headroom_feedforward_velocity_qp;
  requirePositive(
      headroom.joint_reference_smoothness_weight,
      "spark_headroom_feedforward_velocity_qp.joint_reference_smoothness_weight");
  requireNonNegative(
      headroom.joint_reference_jerk_smoothness_weight,
      "spark_headroom_feedforward_velocity_qp.joint_reference_jerk_smoothness_weight");
  if (!std::isfinite(headroom.low_headroom) ||
      headroom.low_headroom < 0.0 || headroom.low_headroom >= 1.0) {
    throw std::runtime_error(
        "spark_headroom_feedforward_velocity_qp.low_headroom must be in [0, 1)");
  }
  if (!std::isfinite(headroom.full_headroom) ||
      headroom.full_headroom <= headroom.low_headroom ||
      headroom.full_headroom > 1.0) {
    throw std::runtime_error(
        "spark_headroom_feedforward_velocity_qp.full_headroom must exceed low_headroom and be at most 1");
  }
  requirePositive(
      headroom.reduction_time_seconds,
      "spark_headroom_feedforward_velocity_qp.reduction_time_seconds");
  requirePositive(
      headroom.recovery_time_seconds,
      "spark_headroom_feedforward_velocity_qp.recovery_time_seconds");
  requirePositive(
      headroom.jerk_usage_attack_seconds,
      "spark_headroom_feedforward_velocity_qp.jerk_usage_attack_seconds");
  requirePositive(
      headroom.jerk_usage_release_seconds,
      "spark_headroom_feedforward_velocity_qp.jerk_usage_release_seconds");
  const auto requireTaskMinimum = [](double value, const char* name) {
    if (!std::isfinite(value) || value < 0.0 || value >= 1.0) {
      throw std::runtime_error(std::string(name) + " must be in [0, 1)");
    }
  };
  requireTaskMinimum(
      headroom.task_scaling_min_position,
      "spark_headroom_feedforward_velocity_qp.task_scaling_min_position");
  requireTaskMinimum(
      headroom.task_scaling_min_orientation,
      "spark_headroom_feedforward_velocity_qp.task_scaling_min_orientation");
  if (!std::isfinite(
          headroom.stationary_reference_hold_linear_velocity_m_s) ||
      headroom.stationary_reference_hold_linear_velocity_m_s <= 0.0) {
    throw std::runtime_error(
        "spark_headroom_feedforward_velocity_qp.stationary_reference_hold_linear_velocity_m_s must be positive and finite");
  }
  if (!std::isfinite(
          headroom.stationary_reference_hold_angular_velocity_rad_s) ||
      headroom.stationary_reference_hold_angular_velocity_rad_s <= 0.0) {
    throw std::runtime_error(
        "spark_headroom_feedforward_velocity_qp.stationary_reference_hold_angular_velocity_rad_s must be positive and finite");
  }
  requirePositive(
      headroom.settled_hold_dwell_seconds,
      "spark_headroom_feedforward_velocity_qp.settled_hold_dwell_seconds");
  if (!std::isfinite(headroom.settled_hold_headroom_enter) ||
      headroom.settled_hold_headroom_enter < 0.0 ||
      headroom.settled_hold_headroom_enter > 1.0) {
    throw std::runtime_error(
        "spark_headroom_feedforward_velocity_qp.settled_hold_headroom_enter must be in [0, 1]");
  }

  if (!std::isfinite(config.pico_ee_headroom.low_frequency_min_scale) ||
      config.pico_ee_headroom.low_frequency_min_scale < 0.0 ||
      config.pico_ee_headroom.low_frequency_min_scale > 1.0) {
    throw std::runtime_error(
        "pico_ee_headroom.low_frequency_min_scale must be in [0, 1]");
  }
  if (!std::isfinite(config.pico_ee_headroom.high_frequency_min_scale) ||
      config.pico_ee_headroom.high_frequency_min_scale < 0.0 ||
      config.pico_ee_headroom.high_frequency_min_scale > 1.0) {
    throw std::runtime_error(
        "pico_ee_headroom.high_frequency_min_scale must be in [0, 1]");
  }
  requireNonNegative(config.pico_ee_headroom.redundancy_zero_headroom,
                     "pico_ee_headroom.redundancy_zero_headroom");
  requirePositive(config.pico_ee_headroom.redundancy_full_headroom,
                  "pico_ee_headroom.redundancy_full_headroom");
  if (config.pico_ee_headroom.redundancy_zero_headroom >=
          config.pico_ee_headroom.redundancy_full_headroom ||
      config.pico_ee_headroom.redundancy_full_headroom > 1.0) {
    throw std::runtime_error(
        "PICO EE redundancy headroom thresholds must increase within [0, 1]");
  }

  const auto& motion = config.pico_ee_motion_confidence;
  requireNonNegative(motion.linear_quiet_m_s,
                     "pico_ee_motion_confidence.linear_quiet_m_s");
  requirePositive(motion.linear_tracking_m_s,
                  "pico_ee_motion_confidence.linear_tracking_m_s");
  if (motion.linear_quiet_m_s >= motion.linear_tracking_m_s) {
    throw std::runtime_error(
        "PICO EE motion linear quiet threshold must be below tracking threshold");
  }
  requireNonNegative(motion.angular_quiet_rad_s,
                     "pico_ee_motion_confidence.angular_quiet_rad_s");
  requirePositive(motion.angular_tracking_rad_s,
                  "pico_ee_motion_confidence.angular_tracking_rad_s");
  if (motion.angular_quiet_rad_s >= motion.angular_tracking_rad_s) {
    throw std::runtime_error(
        "PICO EE motion angular quiet threshold must be below tracking threshold");
  }
  requirePositive(motion.attack_seconds,
                  "pico_ee_motion_confidence.attack_seconds");
  requirePositive(motion.release_seconds,
                  "pico_ee_motion_confidence.release_seconds");

  const auto& estimator = config.pico_ee_twist_estimator;
  if (estimator.fit_window != 3 && estimator.fit_window != 5 &&
      estimator.fit_window != 7) {
    throw std::runtime_error(
        "pico_ee_twist_estimator.fit_window must be 3, 5, or 7");
  }
  requirePositive(estimator.maximum_condition_number,
                  "pico_ee_twist_estimator.maximum_condition_number");
  requirePositive(
      estimator.position_full_confidence_residual_m,
      "pico_ee_twist_estimator.position_full_confidence_residual_m");
  if (!std::isfinite(estimator.position_invalid_residual_m) ||
      estimator.position_invalid_residual_m <=
          estimator.position_full_confidence_residual_m) {
    throw std::runtime_error(
        "pico_ee_twist_estimator position residual thresholds must satisfy "
        "0 < full_confidence < invalid");
  }
  requirePositive(
      estimator.orientation_full_confidence_residual_rad,
      "pico_ee_twist_estimator.orientation_full_confidence_residual_rad");
  if (!std::isfinite(estimator.orientation_invalid_residual_rad) ||
      estimator.orientation_invalid_residual_rad <=
          estimator.orientation_full_confidence_residual_rad) {
    throw std::runtime_error(
        "pico_ee_twist_estimator orientation residual thresholds must "
        "satisfy 0 < full_confidence < invalid");
  }
  requireNonNegative(estimator.age_full_confidence_seconds,
                     "pico_ee_twist_estimator.age_full_confidence_seconds");
  if (!std::isfinite(estimator.maximum_age_seconds) ||
      estimator.maximum_age_seconds <= estimator.age_full_confidence_seconds) {
    throw std::runtime_error(
        "pico_ee_twist_estimator age thresholds must satisfy 0 <= "
        "full_confidence < maximum_age");
  }
  requirePositive(estimator.confidence_attack_seconds,
                  "pico_ee_twist_estimator.confidence_attack_seconds");
  requirePositive(estimator.confidence_release_seconds,
                  "pico_ee_twist_estimator.confidence_release_seconds");
  if (!std::isfinite(estimator.extra_lead_seconds) ||
      estimator.extra_lead_seconds < 0.0 ||
      estimator.extra_lead_seconds > 0.010) {
    throw std::runtime_error(
        "pico_ee_twist_estimator.extra_lead_seconds must be in [0, 0.010]");
  }

  const auto& allocator = config.pico_ee_feedforward_allocator;
  if (allocator.bisection_steps < 0) {
    throw std::runtime_error(
        "pico_ee_feedforward_allocator.bisection_steps must be non-negative");
  }
  if (allocator.max_preview_solves <= 0) {
    throw std::runtime_error(
        "pico_ee_feedforward_allocator.max_preview_solves must be positive");
  }
  if (!std::isfinite(allocator.maximum_scale_increase_per_cycle) ||
      allocator.maximum_scale_increase_per_cycle < 0.0 ||
      allocator.maximum_scale_increase_per_cycle > 1.0) {
    throw std::runtime_error(
        "pico_ee_feedforward_allocator.maximum_scale_increase_per_cycle must "
        "be in [0, 1]");
  }
  if (!std::isfinite(allocator.maximum_soft_bound_usage) ||
      allocator.maximum_soft_bound_usage <= 0.0 ||
      allocator.maximum_soft_bound_usage > 1.0) {
    throw std::runtime_error(
        "pico_ee_feedforward_allocator.maximum_soft_bound_usage must be in "
        "(0, 1]");
  }
  requireNonNegative(allocator.task_scale_tolerance,
                     "pico_ee_feedforward_allocator.task_scale_tolerance");
  requireNonNegative(
      allocator.normalized_residual_tolerance,
      "pico_ee_feedforward_allocator.normalized_residual_tolerance");
  requirePositive(allocator.normalized_residual_floor,
                  "pico_ee_feedforward_allocator.normalized_residual_floor");
  requireNonNegative(
      allocator.arm_angle_residual_tolerance,
      "pico_ee_feedforward_allocator.arm_angle_residual_tolerance");
  requirePositive(
      allocator.maximum_nullspace_leakage,
      "pico_ee_feedforward_allocator.maximum_nullspace_leakage");
  requirePositive(
      allocator.preview_budget_seconds_per_arm,
      "pico_ee_feedforward_allocator.preview_budget_seconds_per_arm");
  requirePositive(
      allocator.mismatch_p99_tolerance_rad_s,
      "pico_ee_feedforward_allocator.mismatch_p99_tolerance_rad_s");
  if (!std::isfinite(allocator.mismatch_max_tolerance_rad_s) ||
      allocator.mismatch_max_tolerance_rad_s <
          allocator.mismatch_p99_tolerance_rad_s) {
    throw std::runtime_error(
        "pico_ee_feedforward_allocator mismatch tolerances must satisfy "
        "0 < p99 <= max");
  }

  const auto& task_allocator = config.pico_ee_task_allocator;
  if (task_allocator.bisection_steps < 0) {
    throw std::runtime_error(
        "pico_ee_task_allocator.bisection_steps must be non-negative");
  }
  if (task_allocator.max_preview_solves <= 0) {
    throw std::runtime_error(
        "pico_ee_task_allocator.max_preview_solves must be positive");
  }
  if (!std::isfinite(task_allocator.minimum_task_scale) ||
      task_allocator.minimum_task_scale <= 0.0 ||
      task_allocator.minimum_task_scale > 1.0) {
    throw std::runtime_error(
        "pico_ee_task_allocator.minimum_task_scale must be in (0, 1]");
  }
  if (!std::isfinite(task_allocator.minimum_preview_task_scale) ||
      task_allocator.minimum_preview_task_scale <= 0.0 ||
      task_allocator.minimum_preview_task_scale > 1.0) {
    throw std::runtime_error(
        "pico_ee_task_allocator.minimum_preview_task_scale must be in (0, 1]");
  }
  if (!std::isfinite(task_allocator.maximum_soft_bound_usage) ||
      task_allocator.maximum_soft_bound_usage <= 0.0 ||
      task_allocator.maximum_soft_bound_usage > 1.0) {
    throw std::runtime_error(
        "pico_ee_task_allocator.maximum_soft_bound_usage must be in (0, 1]");
  }
  requireNonNegative(task_allocator.normalized_residual_tolerance,
                     "pico_ee_task_allocator.normalized_residual_tolerance");
  requirePositive(task_allocator.nullspace_search_step,
                  "pico_ee_task_allocator.nullspace_search_step");
  requireNonNegative(task_allocator.nullspace_reference_weight,
                     "pico_ee_task_allocator.nullspace_reference_weight");
  requireNonNegative(task_allocator.nullspace_continuity_weight,
                     "pico_ee_task_allocator.nullspace_continuity_weight");
  requireNonNegative(task_allocator.nullspace_jerk_weight,
                     "pico_ee_task_allocator.nullspace_jerk_weight");
  requireNonNegative(task_allocator.nullspace_arm_rate_weight,
                     "pico_ee_task_allocator.nullspace_arm_rate_weight");
  if (task_allocator.nullspace_reference_weight == 0.0 &&
      task_allocator.nullspace_continuity_weight == 0.0 &&
      task_allocator.nullspace_jerk_weight == 0.0 &&
      task_allocator.nullspace_arm_rate_weight == 0.0) {
    throw std::runtime_error(
        "pico_ee_task_allocator nullspace search weights must not all be zero");
  }
  requirePositive(task_allocator.preview_budget_seconds_per_arm,
                  "pico_ee_task_allocator.preview_budget_seconds_per_arm");

  if (!std::isfinite(config.dls_posture_ruckig.velocity_scale) ||
      config.dls_posture_ruckig.velocity_scale <= 0.0 ||
      config.dls_posture_ruckig.velocity_scale > 1.0) {
    throw std::runtime_error(
        "dls_posture_ruckig.velocity_scale must be in (0, 1]");
  }
  requirePositive(config.dls_posture_ruckig.max_acceleration_rad_s2,
                  "dls_posture_ruckig.max_acceleration_rad_s2");
  requirePositive(config.dls_posture_ruckig.max_velocity_rad_s,
                  "dls_posture_ruckig.max_velocity_rad_s");
  requirePositive(config.dls_posture_ruckig.max_jerk_rad_s3,
                  "dls_posture_ruckig.max_jerk_rad_s3");
  requirePositive(config.dls_posture_ruckig.validation_tolerance,
                  "dls_posture_ruckig.validation_tolerance");
  requirePositive(config.joint_limits.margin_rad, "joint_limits.margin_rad");
  if (!std::isfinite(config.joint_limits.velocity_scale) ||
      config.joint_limits.velocity_scale <= 0.0 || config.joint_limits.velocity_scale > 1.0) {
    throw std::runtime_error("joint_limits.velocity_scale must be in (0, 1]");
  }
  requirePositive(config.joint_limits.max_acceleration_rad_s2,
                  "joint_limits.max_acceleration_rad_s2");
  requirePositive(config.joint_limits.braking_acceleration_rad_s2,
                  "joint_limits.braking_acceleration_rad_s2");
  requirePositive(config.joint_limits.max_jerk_rad_s3,
                  "joint_limits.max_jerk_rad_s3");
  if (config.osqp.max_iterations <= 0 || config.qpoases.max_working_set_recalculations <= 0) {
    throw std::runtime_error("solver iteration limits must be positive");
  }
  requirePositive(config.osqp.absolute_tolerance, "osqp.absolute_tolerance");
  requirePositive(config.osqp.relative_tolerance, "osqp.relative_tolerance");
  requirePositive(config.qpoases.cpu_time_limit_seconds, "qpoases.cpu_time_limit_seconds");
  requirePositive(config.safety.bound_tolerance, "safety.bound_tolerance");
  requireNonNegative(config.safety.hessian_eigenvalue_tolerance,
                     "safety.hessian_eigenvalue_tolerance");
  requirePositive(config.safety.max_target_position_step, "safety.max_target_position_step");
  requirePositive(config.safety.max_target_orientation_step,
                  "safety.max_target_orientation_step");
  requirePositive(config.safety.reference_tracking_warn_rad,
                  "safety.reference_tracking_warn_rad");
  requirePositive(config.safety.reference_tracking_stop_rad,
                  "safety.reference_tracking_stop_rad");
  if (config.safety.reference_tracking_warn_rad >=
      config.safety.reference_tracking_stop_rad) {
    throw std::runtime_error(
        "safety reference tracking warning must be below stop threshold");
  }
  requirePositive(config.safety.velocity_reference_tracking_warn_rad_s,
                  "safety.velocity_reference_tracking_warn_rad_s");
  requirePositive(config.safety.velocity_reference_tracking_stop_rad_s,
                  "safety.velocity_reference_tracking_stop_rad_s");
  if (config.safety.velocity_reference_tracking_warn_rad_s >=
      config.safety.velocity_reference_tracking_stop_rad_s) {
    throw std::runtime_error(
        "velocity reference tracking warning must be below stop threshold");
  }
  requirePositive(config.trajectories.circle_radius, "trajectories.circle_radius");
  requirePositive(config.trajectories.figure_eight_width, "trajectories.figure_eight_width");
  requirePositive(config.trajectories.figure_eight_height, "trajectories.figure_eight_height");
  requirePositive(config.trajectories.angular_amplitude, "trajectories.angular_amplitude");
  requirePositive(config.trajectories.frequency_hz, "trajectories.frequency_hz");

  return config;
}

Vec7 configuredInitialPosture(const ControllerConfig& config,
                              const ArmLimits& limits, ArmSide side) {
  if (!config.initial_posture_enabled) {
    return 0.5 * (limits.lower_position + limits.upper_position);
  }

  const Vec7& posture = side == ArmSide::kLeft
                            ? config.initial_left_q_rad
                            : config.initial_right_q_rad;
  const char* side_name = side == ArmSide::kLeft ? "left" : "right";
  for (int joint = 0; joint < kArmDof; ++joint) {
    if (!std::isfinite(posture[joint])) {
      throw std::runtime_error(
          std::string("configured ") + side_name + " initial posture joint " +
          std::to_string(joint + 1) + " is not finite");
    }
    if (!std::isfinite(limits.lower_position[joint]) ||
        !std::isfinite(limits.upper_position[joint]) ||
        limits.lower_position[joint] > limits.upper_position[joint]) {
      throw std::runtime_error(
          std::string("invalid ") + side_name + " model limit for joint " +
          std::to_string(joint + 1));
    }
    if (posture[joint] < limits.lower_position[joint] ||
        posture[joint] > limits.upper_position[joint]) {
      throw std::runtime_error(
          std::string("configured ") + side_name + " initial posture joint " +
          std::to_string(joint + 1) + " is outside model joint limits");
    }
  }
  return posture;
}

CartesianOtgConfig cartesianOtgConfigForControlLevel(
    const QpIkConfig& config, ControlLevel level) {
  CartesianOtgConfig result = config.cartesian_otg;
  if (!config.cartesian_otg_control_profiles.enabled) {
    return result;
  }
  const CartesianOtgDynamicLimits& limits =
      level == ControlLevel::kVelocity
          ? config.cartesian_otg_control_profiles.velocity
          : config.cartesian_otg_control_profiles.acceleration;
  applyDynamicLimits(limits, result);
  return result;
}

std::string toString(SolverBackend backend) {
  return backend == SolverBackend::kOsqp ? "osqp" : "qpoases";
}

std::string toString(PicoEeTargetSource source) {
  switch (source) {
    case PicoEeTargetSource::kPacketTargets:
      return "packet_targets";
    case PicoEeTargetSource::kMappedCorrectedPalm:
      return "mapped_corrected_palm";
  }
  return "packet_targets";
}

std::string toString(PicoArmAngleReferenceSource source) {
  switch (source) {
    case PicoArmAngleReferenceSource::kPacketDirection:
      return "packet_direction";
    case PicoArmAngleReferenceSource::kMappedSkeletonPlane:
      return "mapped_skeleton_plane";
  }
  return "packet_direction";
}

std::string toString(PicoMappedVectorQpWeightMode mode) {
  switch (mode) {
    case PicoMappedVectorQpWeightMode::kLegacy:
      return "legacy";
    case PicoMappedVectorQpWeightMode::kDecoupled:
      return "decoupled";
  }
  return "legacy";
}

std::string toString(VelocityQpSmoothnessMode mode) {
  switch (mode) {
    case VelocityQpSmoothnessMode::kLegacy:
      return "legacy";
    case VelocityQpSmoothnessMode::kNormalizedSplit:
      return "normalized_split";
  }
  return "legacy";
}

std::string toString(IkAlgorithm algorithm) {
  switch (algorithm) {
    case IkAlgorithm::kHierarchicalQp:
      return "hierarchical_qp";
    case IkAlgorithm::kNullspaceDls:
      return "nullspace_dls";
    case IkAlgorithm::kSparkGuidedVelocityQp:
      return "spark_guided_velocity_qp";
    case IkAlgorithm::kSparkDirectVelocityQp:
      return "spark_direct_velocity_qp";
    case IkAlgorithm::kSparkPoseVelocityQp:
      return "spark_pose_velocity_qp";
    case IkAlgorithm::kSparkUpperQpoasesDirect:
      return "spark_upper_qpoases_direct";
    case IkAlgorithm::kSparkUpperQpoasesVelocityQp:
      return "spark_upper_qpoases_velocity_qp";
    case IkAlgorithm::kSparkUpperQpoasesCartesianOtgVelocityQp:
      return "spark_upper_qpoases_cartesian_otg_velocity_qp";
    case IkAlgorithm::kSparkUpperQpoasesFeedforwardVelocityQp:
      return "spark_upper_qpoases_feedforward_velocity_qp";
    case IkAlgorithm::kSparkUpperQpoasesHeadroomFeedforwardVelocityQp:
      return "spark_upper_qpoases_headroom_feedforward_velocity_qp";
    case IkAlgorithm::kPicoEeMappedCorrectedPalmVelocityQp:
      return "pico_ee_mapped_corrected_palm_velocity_qp";
  }
  return "hierarchical_qp";
}

std::string toString(ControlLevel level) {
  return level == ControlLevel::kVelocity ? "velocity" : "acceleration";
}

}  // namespace tianji_mapped_palm
