#include "tianji_qp_ik/config.hpp"
#include "tianji_qp_ik/shared_root_options.hpp"
#include "tianji_qp_ik/resource_paths.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <string>

namespace tianji_qp_ik {
namespace {

template <typename T>
T required(const YAML::Node& parent, const char* key) {
  const YAML::Node value = parent[key];
  if (!value) {
    throw std::runtime_error(std::string("missing configuration key: ") + key);
  }
  return value.as<T>();
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

IkAlgorithm parseIkAlgorithm(const std::string& value) {
  if (value == "pico_ee_franka_dls") return IkAlgorithm::kPicoEeFrankaDls;
  throw std::runtime_error("ik.algorithm must be 'pico_ee_franka_dls'");
}
}  // namespace

QpIkConfig loadConfig(const std::string& path, ConfigConsumer consumer) {
  const YAML::Node root = YAML::LoadFile(path);
  QpIkConfig config;
  const YAML::Node shared_root = root["shared_root"];
  if (shared_root) {
    if (!shared_root.IsMap() || shared_root["mix"] ||
        shared_root["palm_position_mix"] || shared_root["palm_orientation_mix"]) {
      throw std::runtime_error("invalid shared_root configuration; mix is unsupported");
    }
    if (required<bool>(shared_root, "enabled")) {
      if (consumer != ConfigConsumer::kSharedRootAware)
        throw std::runtime_error("enabled shared-root requires an explicitly aware consumer");
      config.shared_root_profile_path = loadSharedRootOptions(path).profile_path;
    }
  }

  const YAML::Node controller = root["controller"];
  config.controller.rate_hz = required<double>(controller, "rate_hz");
  if (controller["pico_ee_dls_kinematics_urdf_path"]) {
    const auto value=controller["pico_ee_dls_kinematics_urdf_path"].as<std::string>();
    if (!value.empty()) {
      config.controller.pico_ee_dls_kinematics_urdf_path=controllerResource(path,value).string();
    }
  }
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
    home_path = controllerResource(path, home_path);
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

  config.ik_algorithm = parseIkAlgorithm(required<std::string>(root["ik"], "algorithm"));
  if (const auto node = root["pico_ee_franka_dls"]) {
    auto& c = config.pico_ee_franka_dls;
    c.enabled = optionalBool(node, "enabled", c.enabled);
    c.arm_plane_direction_guidance = optionalBool(node, "arm_plane_direction_guidance", c.arm_plane_direction_guidance);
    c.max_arm_plane_rate_rad_s = optionalDouble(node, "max_arm_plane_rate_rad_s", c.max_arm_plane_rate_rad_s);
    c.home_left_rad = optionalStrictVector7(node, "home_left_rad", c.home_left_rad);
    c.home_right_rad = optionalStrictVector7(node, "home_right_rad", c.home_right_rad);
    c.max_velocity_rad_s = optionalStrictVector7(node, "max_velocity_rad_s", c.max_velocity_rad_s);
    const auto post = node["post_smoothing"];
    if (!post || required<std::string>(post, "mode") != "ruckig")
      throw std::runtime_error("Franka DLS port requires explicit Ruckig smoothing");
    c.post_smoothing.enabled = true;
    c.post_smoothing.velocity_scale = 1.0;
    c.post_smoothing.max_velocity_rad_s = optionalStrictVector7(post, "max_velocity_rad_s", c.max_velocity_rad_s);
    c.post_smoothing.max_acceleration_rad_s2 = optionalStrictVector7(post, "max_acceleration_rad_s2", c.post_smoothing.max_acceleration_rad_s2);
    c.post_smoothing.max_jerk_rad_s3 = optionalStrictVector7(post, "max_jerk_rad_s3", c.post_smoothing.max_jerk_rad_s3);
    c.post_smoothing.validation_tolerance = optionalDouble(post, "validation_tolerance", 1e-8);
  }

  const YAML::Node servo = root["cartesian_servo"];
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
  const YAML::Node pico_teleop = root["pico_teleop"];
  if (pico_teleop) {
    config.pico_teleop.max_position_jump_m = optionalDouble(
        pico_teleop, "max_position_jump_m",
        config.pico_teleop.max_position_jump_m);
    config.pico_teleop.max_orientation_jump_rad = optionalDouble(
        pico_teleop, "max_orientation_jump_rad",
        config.pico_teleop.max_orientation_jump_rad);
  }
  const YAML::Node iterative_dls = root["iterative_dls"];
  if (iterative_dls) {
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
  }
  const YAML::Node shape = root["shared_root_shape"];
  config.shared_root_shape.maximum_joint_rotation_jump_rad = optionalDouble(
      shape, "maximum_joint_rotation_jump_rad",
      config.shared_root_shape.maximum_joint_rotation_jump_rad);
  const YAML::Node limits = root["joint_limits"];
  config.joint_limits.margin_rad = required<double>(limits, "margin_rad");
  config.joint_limits.max_acceleration_rad_s2 = optionalVector7(
      limits, "max_acceleration_rad_s2",
      config.joint_limits.max_acceleration_rad_s2);
  config.joint_limits.max_jerk_rad_s3 = optionalVector7(
      limits, "max_jerk_rad_s3",
      config.joint_limits.max_jerk_rad_s3);
  const YAML::Node lower = limits["position_lower_rad"];
  const YAML::Node upper = limits["position_upper_rad"];
  if (static_cast<bool>(lower) != static_cast<bool>(upper)) {
    throw std::runtime_error(
        "joint_limits requires position_lower_rad and position_upper_rad together");
  }
  if (lower) {
    if (!lower.IsSequence() || !upper.IsSequence() ||
        lower.size() != 2 * kArmDof || upper.size() != 2 * kArmDof) {
      throw std::runtime_error("joint_limits position bounds require fourteen finite values");
    }
    std::array<ArmLimits, 2> execution;
    for (int index = 0; index < 2 * kArmDof; ++index) {
      const double lo = lower[index].as<double>();
      const double hi = upper[index].as<double>();
      if (!std::isfinite(lo) || !std::isfinite(hi) || lo >= hi) {
        throw std::runtime_error("joint_limits position bounds require finite lower < upper");
      }
      execution[index / kArmDof].lower_position[index % kArmDof] = lo;
      execution[index / kArmDof].upper_position[index % kArmDof] = hi;
    }
    config.joint_limits.execution_limits = execution;
  }
  const YAML::Node safety = root["safety"];
  config.safety.bound_tolerance = required<double>(safety, "bound_tolerance");
  config.safety.max_target_position_step = required<double>(safety, "max_target_position_step");
  config.safety.max_target_orientation_step =
      required<double>(safety, "max_target_orientation_step");
  const YAML::Node trajectories = root["trajectories"];
  config.trajectories.circle_radius = required<double>(trajectories, "circle_radius");
  config.trajectories.figure_eight_width = required<double>(trajectories, "figure_eight_width");
  config.trajectories.figure_eight_height = required<double>(trajectories, "figure_eight_height");
  config.trajectories.angular_amplitude = required<double>(trajectories, "angular_amplitude");
  config.trajectories.frequency_hz = required<double>(trajectories, "frequency_hz");

  requirePositive(config.controller.rate_hz, "controller.rate_hz");
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

  requirePositive(config.shared_root_shape.maximum_joint_rotation_jump_rad,
                  "shared_root_shape.maximum_joint_rotation_jump_rad");
  requireNonNegative(config.pico_ee_franka_dls.max_arm_plane_rate_rad_s,
                     "pico_ee_franka_dls.max_arm_plane_rate_rad_s");
  requirePositive(config.pico_ee_franka_dls.max_velocity_rad_s,
                  "pico_ee_franka_dls.max_velocity_rad_s");
  const auto& smoothing = config.pico_ee_franka_dls.post_smoothing;
  requirePositive(smoothing.max_velocity_rad_s, "post_smoothing.max_velocity_rad_s");
  requirePositive(smoothing.max_acceleration_rad_s2, "post_smoothing.max_acceleration_rad_s2");
  requirePositive(smoothing.max_jerk_rad_s3, "post_smoothing.max_jerk_rad_s3");
  requirePositive(smoothing.validation_tolerance, "post_smoothing.validation_tolerance");
  requirePositive(config.joint_limits.margin_rad, "joint_limits.margin_rad");
  requirePositive(config.joint_limits.max_acceleration_rad_s2, "joint_limits.max_acceleration_rad_s2");
  requirePositive(config.joint_limits.max_jerk_rad_s3, "joint_limits.max_jerk_rad_s3");
  requirePositive(config.safety.bound_tolerance, "safety.bound_tolerance");
  requirePositive(config.safety.max_target_position_step, "safety.max_target_position_step");
  requirePositive(config.safety.max_target_orientation_step, "safety.max_target_orientation_step");
  requireNonNegative(config.cartesian_servo.kff_linear, "cartesian_servo.kff_linear");
  requireNonNegative(config.cartesian_servo.kff_angular, "cartesian_servo.kff_angular");
  requirePositive(config.cartesian_servo.feedforward_filter_cutoff_hz, "cartesian_servo.feedforward_filter_cutoff_hz");
  requireNonNegative(config.cartesian_servo.prediction_horizon_seconds, "cartesian_servo.prediction_horizon_seconds");
  requirePositive(config.cartesian_servo.target_timeout_seconds, "cartesian_servo.target_timeout_seconds");
  requirePositive(config.pico_teleop.max_position_jump_m, "pico_teleop.max_position_jump_m");
  requirePositive(config.pico_teleop.max_orientation_jump_rad, "pico_teleop.max_orientation_jump_rad");
  requirePositive(config.trajectories.frequency_hz, "trajectories.frequency_hz");
  return config;
}

ArmLimits effectiveArmLimits(const JointLimitConfig& config,
                            const ArmLimits& model, ArmSide side) {
  requirePositive(config.margin_rad, "joint_limits.margin_rad");
  if (!model.lower_position.allFinite() || !model.upper_position.allFinite() ||
      !model.velocity.allFinite() || (model.velocity.array() <= 0.0).any() ||
      (model.lower_position.array() >= model.upper_position.array()).any()) {
    throw std::runtime_error("invalid model joint limits");
  }
  ArmLimits result = model;
  if (config.execution_limits) {
    const auto& bounds = (*config.execution_limits)[side == ArmSide::kLeft ? 0 : 1];
    if (!bounds.lower_position.allFinite() || !bounds.upper_position.allFinite() ||
        (bounds.lower_position.array() >= bounds.upper_position.array()).any()) {
      throw std::runtime_error("invalid execution joint position bounds");
    }
    result.lower_position = result.lower_position.cwiseMax(bounds.lower_position);
    result.upper_position = result.upper_position.cwiseMin(bounds.upper_position);
  }
  const Vec7 lower = result.lower_position.array() + config.margin_rad;
  const Vec7 upper = result.upper_position.array() - config.margin_rad;
  if (!lower.allFinite() || !upper.allFinite() ||
      (lower.array() >= upper.array()).any()) {
    throw std::runtime_error(
        std::string(side == ArmSide::kLeft ? "left" : "right") +
        " effective joint limits are empty or infeasible with joint_limits.margin_rad");
  }
  return result;
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
          std::string("invalid ") + side_name + " motion limit for joint " +
          std::to_string(joint + 1));
    }
    if (posture[joint] < limits.lower_position[joint] ||
        posture[joint] > limits.upper_position[joint]) {
      throw std::runtime_error(
          std::string("configured ") + side_name + " initial posture joint " +
          std::to_string(joint + 1) + " is outside effective joint limits");
    }
  }
  return posture;
}

std::string toString(IkAlgorithm) { return "pico_ee_franka_dls"; }

}  // namespace tianji_qp_ik
