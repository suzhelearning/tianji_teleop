#include "tianji_qp_ik/benchmark_dataset.hpp"
#include "tianji_qp_ik/acceleration_controller.hpp"
#include "tianji_qp_ik/cartesian_otg.hpp"
#include "tianji_qp_ik/controller.hpp"
#include "tianji_qp_ik/mujoco_robot.hpp"
#include "tianji_qp_ik/so3.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

namespace tianji_qp_ik {
namespace {

struct Options {
  std::string config_path{"config/qp_ik_cartesian_otg_velocity.yaml"};
  std::string model_path{"models/marvin_m6_qp_test.xml"};
  std::string output_path{"cartesian_otg_velocity_ab.csv"};
  int steps{600};
  std::string scenario{"all"};
  std::string arm_angle_profile{"legacy"};
};

enum class Scenario {
  kStraight,
  kCircle,
  kFigureEight,
  kReversal,
  kStop,
  kJitter,
  kOrientation,
  kNearLimit,
  kSingular,
  kUnreachable,
  kTimestampJitter,
  kDroppedFrames,
};

enum class ControllerKind {
  kDirectVelocityQp,
  kOtgVelocityQp,
  kOtgAccelerationQp,
};

enum class ArmAngleProfileKind {
  kLegacy,
  kFixed,
  kDynamicConsistent,
};

struct Metrics {
  ControllerKind controller{ControllerKind::kDirectVelocityQp};
  Scenario scenario{Scenario::kStraight};
  ArmAngleProfileKind arm_angle_profile{ArmAngleProfileKind::kLegacy};
  double position_rms{0.0};
  double position_p95{0.0};
  double orientation_rms{0.0};
  double orientation_p95{0.0};
  double peak_error{0.0};
  double phase_lag_seconds{0.0};
  double settling_time_seconds{0.0};
  double reference_acceleration_peak{0.0};
  double reference_jerk_peak{0.0};
  double qdot_variation_rms{0.0};
  double qdot_peak{0.0};
  double qddot_rms{0.0};
  double qddot_peak{0.0};
  double joint_jerk_rms{0.0};
  double joint_jerk_peak{0.0};
  double position_reference_error_peak{0.0};
  double velocity_reference_error_peak{0.0};
  double slack_rms{0.0};
  double solve_p99_us{0.0};
  double solve_mean_us{0.0};
  std::uint64_t active_bounds{0U};
  double left_arm_angle_rms{0.0};
  double left_arm_angle_p95{0.0};
  double left_arm_angle_max{0.0};
  double right_arm_angle_rms{0.0};
  double right_arm_angle_p95{0.0};
  double right_arm_angle_max{0.0};
  double left_outward_min{std::numeric_limits<double>::infinity()};
  double right_outward_min{std::numeric_limits<double>::infinity()};
  std::uint64_t left_outward_active_count{0U};
  std::uint64_t right_outward_active_count{0U};
  std::uint64_t left_arm_angle_held_count{0U};
  std::uint64_t right_arm_angle_held_count{0U};
  std::uint64_t failures{0U};
  bool hard_bounds_ok{true};
  bool finite{true};
};

Options parseOptions(int argc, char** argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--help") {
      std::cout << "Usage: tianji_cartesian_otg_benchmark [--config FILE] "
                   "[--model FILE] [--output FILE] [--steps COUNT] "
                   "[--scenario all|circle] "
                   "[--arm-angle-profile legacy|fixed|dynamic_consistent|both]\n";
      std::exit(0);
    }
    if (index + 1 >= argc) {
      throw std::invalid_argument("missing value after " + argument);
    }
    const std::string value(argv[++index]);
    if (argument == "--config") {
      options.config_path = value;
    } else if (argument == "--model") {
      options.model_path = value;
    } else if (argument == "--output") {
      options.output_path = value;
    } else if (argument == "--steps") {
      options.steps = std::stoi(value);
    } else if (argument == "--scenario") {
      options.scenario = value;
    } else if (argument == "--arm-angle-profile") {
      options.arm_angle_profile = value;
    } else {
      throw std::invalid_argument("unknown option: " + argument);
    }
  }
  if (options.steps < 40) {
    throw std::invalid_argument("--steps must be at least 40");
  }
  if (options.scenario != "all" && options.scenario != "circle") {
    throw std::invalid_argument("--scenario must be all or circle");
  }
  if (options.arm_angle_profile != "legacy" &&
      options.arm_angle_profile != "fixed" &&
      options.arm_angle_profile != "dynamic_consistent" &&
      options.arm_angle_profile != "both") {
    throw std::invalid_argument(
        "--arm-angle-profile must be legacy, fixed, dynamic_consistent, or both");
  }
  return options;
}

std::string toString(ControllerKind kind) {
  switch (kind) {
    case ControllerKind::kDirectVelocityQp:
      return "direct_velocity_qp";
    case ControllerKind::kOtgVelocityQp:
      return "otg_velocity_qp";
    case ControllerKind::kOtgAccelerationQp:
      return "otg_acceleration_qp";
  }
  return "unknown";
}

std::string toString(ArmAngleProfileKind profile) {
  switch (profile) {
    case ArmAngleProfileKind::kLegacy:
      return "legacy";
    case ArmAngleProfileKind::kFixed:
      return "fixed";
    case ArmAngleProfileKind::kDynamicConsistent:
      return "dynamic_consistent";
  }
  return "unknown";
}

std::string toString(Scenario scenario) {
  constexpr std::array<const char*, 12> names{
      "straight", "circle", "figure_eight", "reversal", "stop", "jitter",
      "orientation", "near_limit", "singular", "unreachable",
      "timestamp_jitter", "dropped_frames"};
  return names.at(static_cast<std::size_t>(scenario));
}

double percentile(std::vector<double> values, double quantile) {
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const double index = quantile * static_cast<double>(values.size() - 1U);
  const std::size_t lower = static_cast<std::size_t>(std::floor(index));
  const std::size_t upper = static_cast<std::size_t>(std::ceil(index));
  return values[lower] + (index - static_cast<double>(lower)) *
                             (values[upper] - values[lower]);
}

ArmDirectionReference armDirectionFromGeometry(
    const ArmKinematicSample& sample) {
  ArmDirectionReference result;
  const Eigen::Vector3d shoulder_to_wrist =
      sample.wrist_position - sample.shoulder_position;
  if (!shoulder_to_wrist.allFinite() ||
      shoulder_to_wrist.norm() <= 1.0e-9) {
    return result;
  }
  const Eigen::Vector3d axis = shoulder_to_wrist.normalized();
  Eigen::Vector3d radial = sample.elbow_position - sample.shoulder_position;
  radial -= axis * axis.dot(radial);
  if (!radial.allFinite() || radial.norm() <= 1.0e-9) {
    return result;
  }
  result.valid = true;
  result.direction = radial.normalized();
  result.source = ArmDirectionReferenceSource::kPico;
  return result;
}

ArmDirectionReference dynamicConsistentDirection(
    ArmSide side, const ArmDirectionReference& fixed,
    const Eigen::Vector3d& shoulder, const Pose& target, double time) {
  ArmDirectionReference result;
  Eigen::Vector3d axis = target.position - shoulder;
  if (!axis.allFinite() || axis.norm() <= 1.0e-9) {
    return result;
  }
  axis.normalize();
  Eigen::Vector3d projected = fixed.direction - axis * axis.dot(fixed.direction);
  if (!fixed.valid || !projected.allFinite() || projected.norm() <= 1.0e-6) {
    const Eigen::Vector3d outward =
        side == ArmSide::kLeft ? Eigen::Vector3d(0.0, 1.0, 0.0)
                               : Eigen::Vector3d(0.0, -1.0, 0.0);
    projected = outward - axis * axis.dot(outward);
  }
  if (!projected.allFinite() || projected.norm() <= 1.0e-6) {
    projected = axis.unitOrthogonal();
  }
  projected.normalize();
  constexpr double kAmplitudeRad = 0.3490658503988659;
  constexpr double kFrequencyHz = 0.8;
  constexpr double kTwoPi = 6.283185307179586476925286766559;
  const double angle =
      kAmplitudeRad * std::sin(kTwoPi * kFrequencyHz * time);
  result.valid = true;
  result.direction =
      (Eigen::AngleAxisd(angle, axis) * projected).normalized();
  result.source = ArmDirectionReferenceSource::kPico;
  return result;
}

DualArmDirectionReferences armDirectionsAt(
    ArmAngleProfileKind profile,
    const DualArmDirectionReferences& fixed,
    const Eigen::Vector3d& left_shoulder,
    const Eigen::Vector3d& right_shoulder,
    const DualArmTargets& target, double time) {
  if (profile == ArmAngleProfileKind::kLegacy) {
    return defaultArmDirectionReferences();
  }
  if (profile == ArmAngleProfileKind::kFixed) {
    return fixed;
  }
  DualArmDirectionReferences result;
  result.left = dynamicConsistentDirection(
      ArmSide::kLeft, fixed.left, left_shoulder, target.left, time);
  result.right = dynamicConsistentDirection(
      ArmSide::kRight, fixed.right, right_shoulder, target.right, time);
  return result;
}

double estimatePositionLag(const std::vector<Vec6>& targets,
                           const std::vector<Vec6>& measured, double dt) {
  if (targets.empty() || targets.size() != measured.size()) {
    return 0.0;
  }
  const std::size_t max_lag = std::min(
      targets.size() - 1U,
      static_cast<std::size_t>(std::llround(0.25 / dt)));
  std::size_t best_lag = 0U;
  double best_error = std::numeric_limits<double>::infinity();
  for (std::size_t lag = 0U; lag <= max_lag; ++lag) {
    double error = 0.0;
    const std::size_t count = targets.size() - lag;
    for (std::size_t index = 0U; index < count; ++index) {
      error += (targets[index] - measured[index + lag]).squaredNorm();
    }
    error /= static_cast<double>(count);
    if (error < best_error) {
      best_error = error;
      best_lag = lag;
    }
  }
  return static_cast<double>(best_lag) * dt;
}

double estimateSettlingTime(const std::vector<double>& errors, double dt,
                            double peak_error) {
  const double threshold = std::max(0.01, 0.05 * peak_error);
  for (std::size_t start = 0U; start < errors.size(); ++start) {
    if (std::all_of(errors.begin() + static_cast<std::ptrdiff_t>(start),
                    errors.end(), [threshold](double error) {
                      return error <= threshold;
                    })) {
      return static_cast<double>(start) * dt;
    }
  }
  return static_cast<double>(errors.size()) * dt;
}

Vec7 initialPosition(ArmSide side, const ArmLimits& limits, Scenario scenario) {
  if (scenario == Scenario::kSingular) {
    return safeSingularBenchmarkPosition(side);
  }
  Vec7 result = 0.5 * (limits.lower_position + limits.upper_position);
  if (scenario == Scenario::kNearLimit) {
    for (int joint = 0; joint < kArmDof; ++joint) {
      result[joint] = joint % 2 == 0
                          ? limits.upper_position[joint] - 0.30
                          : limits.lower_position[joint] + 0.30;
    }
  }
  return result;
}

DualArmTargets targetAt(const DualArmTargets& base, Scenario scenario,
                        double time) {
  constexpr double kPi = 3.14159265358979323846;
  const double phase = 2.0 * kPi * 0.8 * time;
  DualArmTargets target = base;
  const double sign = scenario == Scenario::kReversal
                          ? (std::fmod(time, 0.5) < 0.25 ? 1.0 : -1.0)
                          : 1.0;
  switch (scenario) {
    case Scenario::kStraight:
      target.left.position.x() += std::min(0.15 * time, 0.08);
      target.right.position.x() -= std::min(0.15 * time, 0.08);
      break;
    case Scenario::kCircle:
      target.left.position += Eigen::Vector3d(
          0.04 * (std::cos(phase) - 1.0), 0.0, 0.04 * std::sin(phase));
      target.right.position += Eigen::Vector3d(
          -0.04 * (std::cos(phase) - 1.0), 0.0, 0.04 * std::sin(phase));
      break;
    case Scenario::kFigureEight:
      target.left.position += Eigen::Vector3d(
          0.04 * std::sin(phase), 0.0, 0.025 * std::sin(2.0 * phase));
      target.right.position += Eigen::Vector3d(
          -0.04 * std::sin(phase), 0.0, 0.025 * std::sin(2.0 * phase));
      break;
    case Scenario::kReversal:
      target.left.position.x() += 0.05 * sign;
      target.right.position.x() -= 0.05 * sign;
      break;
    case Scenario::kStop:
      target.left.position.x() += std::min(0.20 * time, 0.05);
      target.right.position.x() -= std::min(0.20 * time, 0.05);
      break;
    case Scenario::kJitter: {
      const double jitter = 0.002 * std::sin(2.0 * kPi * 17.0 * time);
      target.left.position += Eigen::Vector3d(0.03, jitter, -jitter);
      target.right.position += Eigen::Vector3d(-0.03, -jitter, jitter);
      break;
    }
    case Scenario::kOrientation:
      target.left.rotation =
          Eigen::AngleAxisd(0.35 * std::sin(phase),
                            Eigen::Vector3d::UnitZ())
              .toRotationMatrix() *
          base.left.rotation;
      target.right.rotation =
          Eigen::AngleAxisd(-0.35 * std::sin(phase),
                            Eigen::Vector3d::UnitZ())
              .toRotationMatrix() *
          base.right.rotation;
      break;
    case Scenario::kNearLimit:
      target.left.position.x() += 0.04;
      target.right.position.x() -= 0.04;
      break;
    case Scenario::kSingular:
      target.left.rotation =
          Eigen::AngleAxisd(0.10, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
          base.left.rotation;
      target.right.rotation =
          Eigen::AngleAxisd(-0.10, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
          base.right.rotation;
      break;
    case Scenario::kUnreachable:
      target.left.position.x() += 3.0;
      target.right.position.x() -= 3.0;
      break;
    case Scenario::kTimestampJitter:
    case Scenario::kDroppedFrames:
      target.left.position.x() += 0.04 * std::sin(phase);
      target.right.position.x() -= 0.04 * std::sin(phase);
      break;
  }
  return target;
}

bool publishFrame(Scenario scenario, int step) {
  if (scenario == Scenario::kDroppedFrames) {
    return step % 4 == 0;
  }
  if (scenario == Scenario::kTimestampJitter) {
    return step % 3 != 1;
  }
  return true;
}

bool withinMargins(const Vec7& q, const ArmLimits& limits,
                   const QpIkConfig& config) {
  return q.allFinite() &&
         (q.array() >= limits.lower_position.array() +
                           config.joint_limits.margin_rad -
                           config.safety.bound_tolerance)
             .all() &&
         (q.array() <= limits.upper_position.array() -
                           config.joint_limits.margin_rad +
                           config.safety.bound_tolerance)
             .all();
}

Metrics runScenario(const Options& options, QpIkConfig config,
                    ControllerKind kind, Scenario scenario,
                    ArmAngleProfileKind arm_angle_profile) {
  config.cartesian_otg.enabled = kind != ControllerKind::kDirectVelocityQp;
  config.control_level = kind == ControllerKind::kOtgAccelerationQp
                             ? ControlLevel::kAcceleration
                             : ControlLevel::kVelocity;
  if (kind == ControllerKind::kDirectVelocityQp) {
    config.cartesian_servo.kff_linear = 0.8;
    config.cartesian_servo.kff_angular = 0.8;
  }
  MujocoRobot robot(options.model_path);
  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    robot.setArmPosition(
        side, initialPosition(side, robot.mapping(side).limits, scenario));
  }
  robot.forward();
  const DualArmTargets base{robot.tcpPose(ArmSide::kLeft),
                            robot.tcpPose(ArmSide::kRight)};
  const ArmKinematicSample left_initial = robot.armKinematicsAt(
      ArmSide::kLeft, robot.armPosition(ArmSide::kLeft));
  const ArmKinematicSample right_initial = robot.armKinematicsAt(
      ArmSide::kRight, robot.armPosition(ArmSide::kRight));
  DualArmDirectionReferences fixed_arm_directions;
  fixed_arm_directions.left = armDirectionFromGeometry(left_initial);
  fixed_arm_directions.right = armDirectionFromGeometry(right_initial);
  DualArmTargets sampled = base;
  DualArmTargets previous_sampled = base;
  DualArmController controller(robot, config);
  DualArmAccelerationController acceleration_controller(robot, config);
  if (arm_angle_profile != ArmAngleProfileKind::kLegacy) {
    controller.setArmAngleReferenceMode(ArmAngleReferenceMode::kPicoOutward);
    acceleration_controller.setArmAngleReferenceMode(
        ArmAngleReferenceMode::kPicoOutward);
  }
  const double dt = 1.0 / config.controller.rate_hz;
  CartesianReferenceGenerator left_otg(config.cartesian_otg, dt);
  CartesianReferenceGenerator right_otg(config.cartesian_otg, dt);
  left_otg.reset(base.left);
  right_otg.reset(base.right);

  Metrics metrics;
  metrics.controller = kind;
  metrics.scenario = scenario;
  metrics.arm_angle_profile = arm_angle_profile;
  std::vector<double> position_errors;
  std::vector<double> orientation_errors;
  std::vector<double> left_arm_angle_errors;
  std::vector<double> right_arm_angle_errors;
  std::vector<double> solve_times;
  std::vector<Vec6> target_positions;
  std::vector<Vec6> measured_positions;
  double position_squared_sum = 0.0;
  double orientation_squared_sum = 0.0;
  double variation_squared_sum = 0.0;
  double qddot_squared_sum = 0.0;
  double joint_jerk_squared_sum = 0.0;
  double slack_squared_sum = 0.0;
  Vec7 previous_left_qdot = Vec7::Zero();
  Vec7 previous_right_qdot = Vec7::Zero();
  Vec7 previous_left_qddot = Vec7::Zero();
  Vec7 previous_right_qddot = Vec7::Zero();
  Vec6 previous_acceleration = Vec6::Zero();
  int frames_since_update = 0;

  for (int step = 0; step < options.steps; ++step) {
    const double time = static_cast<double>(step) * dt;
    ++frames_since_update;
    if (publishFrame(scenario, step)) {
      previous_sampled = sampled;
      sampled = targetAt(base, scenario, time);
      const double frame_dt = static_cast<double>(frames_since_update) * dt;
      sampled.left_twist = poseErrorWorld(sampled.left, previous_sampled.left) /
                           frame_dt;
      sampled.right_twist =
          poseErrorWorld(sampled.right, previous_sampled.right) / frame_dt;
      if (step == 0) {
        sampled.left_twist.setZero();
        sampled.right_twist.setZero();
      }
      frames_since_update = 0;
    }
    const bool stale = frames_since_update * dt >
                       config.cartesian_servo.target_timeout_seconds;
    sampled.left_stale = stale;
    sampled.right_stale = stale;

    ControllerDiagnostics result;
    AccelerationControllerDiagnostics acceleration_result;
    const DualArmDirectionReferences arm_directions = armDirectionsAt(
        arm_angle_profile, fixed_arm_directions,
        left_initial.shoulder_position, right_initial.shoulder_position,
        sampled, time);
    Vec6 reference_acceleration = Vec6::Zero();
    if (kind != ControllerKind::kDirectVelocityQp) {
      DualArmReferences references;
      references.left = left_otg.update(sampled.left, sampled.left_twist,
                                        sampled.left_stale, dt);
      references.right = right_otg.update(sampled.right, sampled.right_twist,
                                          sampled.right_stale, dt);
      reference_acceleration = references.left.acceleration;
      if (kind == ControllerKind::kOtgAccelerationQp) {
        acceleration_result = acceleration_controller.step(
            references, arm_directions, dt);
      } else {
        result = controller.step(references, arm_directions, dt);
      }
    } else {
      result = controller.step(sampled, arm_directions, dt);
      reference_acceleration =
          (sampled.left_twist - result.left.reference.acceleration) / dt;
    }
    const bool accepted = kind == ControllerKind::kOtgAccelerationQp
                              ? acceleration_result.left.accepted &&
                                    acceleration_result.right.accepted
                              : result.left.accepted && result.right.accepted;
    if (!accepted) {
      std::cerr << "benchmark_failure controller=" << toString(kind)
                << " scenario=" << toString(scenario) << " step=" << step
                << " left="
                << toString(kind == ControllerKind::kOtgAccelerationQp
                                ? acceleration_result.left.hold_reason
                                : result.left.hold_reason)
                << " right="
                << toString(kind == ControllerKind::kOtgAccelerationQp
                                ? acceleration_result.right.hold_reason
                                : result.right.hold_reason)
                << '\n';
      if (kind == ControllerKind::kOtgAccelerationQp) {
        for (const auto* arm : {&acceleration_result.left,
                                &acceleration_result.right}) {
          if (arm->hold_joint_index >= 0) {
            const int joint = arm->hold_joint_index;
            std::cerr << "  joint=" << joint << " q_ref=" << arm->q_ref[joint]
                      << " qdot_ref=" << arm->qdot_ref[joint]
                      << " lower=" << arm->bounds.lower[joint]
                      << " upper=" << arm->bounds.upper[joint] << '\n';
          }
        }
      }
      ++metrics.failures;
      continue;
    }

    const Vec7 left_qdot = kind == ControllerKind::kOtgAccelerationQp
                               ? acceleration_result.left.qdot_ref
                               : result.left.ik.qdot;
    const Vec7 right_qdot = kind == ControllerKind::kOtgAccelerationQp
                                ? acceleration_result.right.qdot_ref
                                : result.right.ik.qdot;
    const Vec7 left_qddot = kind == ControllerKind::kOtgAccelerationQp
                                ? acceleration_result.left.qp.qddot
                                : (left_qdot - previous_left_qdot) / dt;
    const Vec7 right_qddot = kind == ControllerKind::kOtgAccelerationQp
                                 ? acceleration_result.right.qp.qddot
                                 : (right_qdot - previous_right_qdot) / dt;
    const Vec6 left_slack = kind == ControllerKind::kOtgAccelerationQp
                                ? acceleration_result.left.qp.slack
                                : result.left.ik.slack;
    const Vec6 right_slack = kind == ControllerKind::kOtgAccelerationQp
                                 ? acceleration_result.right.qp.slack
                                 : result.right.ik.slack;
    const double left_solve = kind == ControllerKind::kOtgAccelerationQp
                                  ? acceleration_result.left.qp.solve_time_us
                                  : result.left.ik.solve_time_us;
    const double right_solve = kind == ControllerKind::kOtgAccelerationQp
                                   ? acceleration_result.right.qp.solve_time_us
                                   : result.right.ik.solve_time_us;
    const auto record_secondary_metrics = [&](const auto& left_diagnostics,
                                              const auto& right_diagnostics) {
      if (left_diagnostics.arm_angle.active) {
        const double error = std::abs(left_diagnostics.arm_angle.error_rad);
        left_arm_angle_errors.push_back(error);
        metrics.left_arm_angle_max =
            std::max(metrics.left_arm_angle_max, error);
      }
      if (right_diagnostics.arm_angle.active) {
        const double error = std::abs(right_diagnostics.arm_angle.error_rad);
        right_arm_angle_errors.push_back(error);
        metrics.right_arm_angle_max =
            std::max(metrics.right_arm_angle_max, error);
      }
      metrics.left_arm_angle_held_count +=
          left_diagnostics.arm_angle.reference_projection_held ? 1U : 0U;
      metrics.right_arm_angle_held_count +=
          right_diagnostics.arm_angle.reference_projection_held ? 1U : 0U;
      if (left_diagnostics.upper_arm_outward.state.valid) {
        metrics.left_outward_min = std::min(
            metrics.left_outward_min,
            left_diagnostics.upper_arm_outward.state.distance_m);
      }
      if (right_diagnostics.upper_arm_outward.state.valid) {
        metrics.right_outward_min = std::min(
            metrics.right_outward_min,
            right_diagnostics.upper_arm_outward.state.distance_m);
      }
      metrics.left_outward_active_count +=
          left_diagnostics.upper_arm_outward.constraint_active ? 1U : 0U;
      metrics.right_outward_active_count +=
          right_diagnostics.upper_arm_outward.constraint_active ? 1U : 0U;
    };
    if (kind == ControllerKind::kOtgAccelerationQp) {
      record_secondary_metrics(acceleration_result.left,
                               acceleration_result.right);
    } else {
      record_secondary_metrics(result.left, result.right);
    }

    robot.forward();
    const Pose left_pose = robot.tcpPose(ArmSide::kLeft);
    const Pose right_pose = robot.tcpPose(ArmSide::kRight);
    const double position_error = std::max(
        (sampled.left.position - left_pose.position).norm(),
        (sampled.right.position - right_pose.position).norm());
    const double orientation_error = std::max(
        rotationDistance(sampled.left.rotation, left_pose.rotation),
        rotationDistance(sampled.right.rotation, right_pose.rotation));
    position_errors.push_back(position_error);
    orientation_errors.push_back(orientation_error);
    Vec6 target_positions_sample;
    target_positions_sample << sampled.left.position, sampled.right.position;
    Vec6 measured_positions_sample;
    measured_positions_sample << left_pose.position, right_pose.position;
    target_positions.push_back(target_positions_sample);
    measured_positions.push_back(measured_positions_sample);
    position_squared_sum += position_error * position_error;
    orientation_squared_sum += orientation_error * orientation_error;
    variation_squared_sum +=
        (left_qdot - previous_left_qdot).squaredNorm() +
        (right_qdot - previous_right_qdot).squaredNorm();
    qddot_squared_sum += left_qddot.squaredNorm() + right_qddot.squaredNorm();
    const Vec7 left_jerk = (left_qddot - previous_left_qddot) / dt;
    const Vec7 right_jerk = (right_qddot - previous_right_qddot) / dt;
    joint_jerk_squared_sum +=
        left_jerk.squaredNorm() + right_jerk.squaredNorm();
    metrics.qddot_peak = std::max(
        metrics.qddot_peak,
        std::max(left_qddot.cwiseAbs().maxCoeff(),
                 right_qddot.cwiseAbs().maxCoeff()));
    metrics.qdot_peak = std::max(
        metrics.qdot_peak,
        std::max(left_qdot.cwiseAbs().maxCoeff(),
                 right_qdot.cwiseAbs().maxCoeff()));
    metrics.joint_jerk_peak = std::max(
        metrics.joint_jerk_peak,
        std::max(left_jerk.cwiseAbs().maxCoeff(),
                 right_jerk.cwiseAbs().maxCoeff()));
    slack_squared_sum += left_slack.squaredNorm() + right_slack.squaredNorm();
    previous_left_qdot = left_qdot;
    previous_right_qdot = right_qdot;
    previous_left_qddot = left_qddot;
    previous_right_qddot = right_qddot;
    metrics.reference_acceleration_peak = std::max(
        metrics.reference_acceleration_peak, reference_acceleration.norm());
    metrics.reference_jerk_peak = std::max(
        metrics.reference_jerk_peak,
        (reference_acceleration - previous_acceleration).norm() / dt);
    previous_acceleration = reference_acceleration;
    metrics.peak_error = std::max(metrics.peak_error, position_error);
    if (kind == ControllerKind::kOtgAccelerationQp) {
      metrics.active_bounds += static_cast<std::uint64_t>(
          acceleration_result.left.qp.active_bound_count +
          acceleration_result.right.qp.active_bound_count);
      metrics.position_reference_error_peak = std::max(
          metrics.position_reference_error_peak,
          std::max(acceleration_result.left.position_reference_error,
                   acceleration_result.right.position_reference_error));
      metrics.velocity_reference_error_peak = std::max(
          metrics.velocity_reference_error_peak,
          std::max(acceleration_result.left.velocity_reference_error,
                   acceleration_result.right.velocity_reference_error));
    } else {
      metrics.active_bounds += static_cast<std::uint64_t>(
          result.left.ik.active_position_bound_count +
          result.left.ik.active_velocity_bound_count +
          result.left.ik.active_acceleration_bound_count +
          result.left.ik.active_braking_bound_count +
          result.right.ik.active_position_bound_count +
          result.right.ik.active_velocity_bound_count +
          result.right.ik.active_acceleration_bound_count +
          result.right.ik.active_braking_bound_count);
    }
    solve_times.push_back(left_solve);
    solve_times.push_back(right_solve);
    for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
      metrics.hard_bounds_ok =
          metrics.hard_bounds_ok &&
          withinMargins(robot.armPosition(side), robot.mapping(side).limits,
                        config);
    }
  }

  const double count = static_cast<double>(position_errors.size());
  if (count == 0.0) {
    metrics.finite = false;
    return metrics;
  }
  metrics.position_rms = std::sqrt(position_squared_sum / count);
  metrics.orientation_rms = std::sqrt(orientation_squared_sum / count);
  metrics.position_p95 = percentile(position_errors, 0.95);
  metrics.orientation_p95 = percentile(orientation_errors, 0.95);
  metrics.phase_lag_seconds =
      estimatePositionLag(target_positions, measured_positions, dt);
  metrics.settling_time_seconds =
      estimateSettlingTime(position_errors, dt, metrics.peak_error);
  metrics.qdot_variation_rms =
      std::sqrt(variation_squared_sum /
                (count * 2.0 * static_cast<double>(kArmDof)));
  metrics.qddot_rms = std::sqrt(
      qddot_squared_sum / (count * 2.0 * static_cast<double>(kArmDof)));
  metrics.joint_jerk_rms = std::sqrt(
      joint_jerk_squared_sum /
      (count * 2.0 * static_cast<double>(kArmDof)));
  metrics.slack_rms = std::sqrt(slack_squared_sum / (count * 2.0));
  const auto rms = [](const std::vector<double>& values) {
    if (values.empty()) {
      return 0.0;
    }
    const double squared_sum = std::inner_product(
        values.begin(), values.end(), values.begin(), 0.0);
    return std::sqrt(squared_sum / static_cast<double>(values.size()));
  };
  metrics.left_arm_angle_rms = rms(left_arm_angle_errors);
  metrics.left_arm_angle_p95 = percentile(left_arm_angle_errors, 0.95);
  metrics.right_arm_angle_rms = rms(right_arm_angle_errors);
  metrics.right_arm_angle_p95 = percentile(right_arm_angle_errors, 0.95);
  if (!std::isfinite(metrics.left_outward_min)) {
    metrics.left_outward_min = 0.0;
  }
  if (!std::isfinite(metrics.right_outward_min)) {
    metrics.right_outward_min = 0.0;
  }
  metrics.solve_p99_us = percentile(solve_times, 0.99);
  metrics.solve_mean_us =
      std::accumulate(solve_times.begin(), solve_times.end(), 0.0) /
      static_cast<double>(solve_times.size());
  metrics.finite = std::isfinite(metrics.position_rms) &&
                   std::isfinite(metrics.position_p95) &&
                   std::isfinite(metrics.orientation_rms) &&
                   std::isfinite(metrics.orientation_p95) &&
                   std::isfinite(metrics.phase_lag_seconds) &&
                   std::isfinite(metrics.settling_time_seconds) &&
                   std::isfinite(metrics.reference_acceleration_peak) &&
                   std::isfinite(metrics.reference_jerk_peak) &&
                   std::isfinite(metrics.qdot_variation_rms) &&
                   std::isfinite(metrics.qdot_peak) &&
                   std::isfinite(metrics.qddot_rms) &&
                   std::isfinite(metrics.qddot_peak) &&
                   std::isfinite(metrics.joint_jerk_rms) &&
                   std::isfinite(metrics.joint_jerk_peak) &&
                   std::isfinite(metrics.slack_rms) &&
                   std::isfinite(metrics.solve_mean_us) &&
                   std::isfinite(metrics.solve_p99_us);
  return metrics;
}

void writeCsv(const std::string& path, const std::vector<Metrics>& results,
              bool extended_arm_angle_metrics) {
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("cannot open benchmark output: " + path);
  }
  output << "controller,scenario,";
  if (extended_arm_angle_metrics) {
    output << "arm_angle_profile,";
  }
  output << "position_rms_m,position_p95_m,orientation_rms_rad,"
            "orientation_p95_rad,peak_error_m,phase_lag_seconds,settling_time_seconds,"
            "reference_acceleration_peak,reference_jerk_peak,qdot_variation_rms,"
            "qdot_peak,qddot_rms,qddot_peak,"
            "joint_jerk_rms,joint_jerk_peak,position_reference_error_peak,"
            "velocity_reference_error_peak,slack_rms,active_bounds,"
            "solve_mean_us,solve_p99_us,failures,hard_bounds_ok,finite";
  if (extended_arm_angle_metrics) {
    output << ",left_arm_angle_rms_rad,left_arm_angle_p95_rad,"
              "left_arm_angle_max_rad,right_arm_angle_rms_rad,"
              "right_arm_angle_p95_rad,right_arm_angle_max_rad,"
              "left_outward_min_m,right_outward_min_m,"
              "left_outward_active_count,right_outward_active_count,"
              "left_arm_angle_held_count,right_arm_angle_held_count";
  }
  output << '\n';
  output << std::setprecision(12);
  for (const Metrics& metrics : results) {
    output << toString(metrics.controller) << ',' << toString(metrics.scenario)
           << ',';
    if (extended_arm_angle_metrics) {
      output << toString(metrics.arm_angle_profile) << ',';
    }
    output << metrics.position_rms << ',' << metrics.position_p95 << ','
           << metrics.orientation_rms << ',' << metrics.orientation_p95 << ','
           << metrics.peak_error << ',' << metrics.phase_lag_seconds << ','
           << metrics.settling_time_seconds << ','
           << metrics.reference_acceleration_peak << ','
           << metrics.reference_jerk_peak << ','
           << metrics.qdot_variation_rms << ',' << metrics.qdot_peak << ','
           << metrics.qddot_rms << ','
           << metrics.qddot_peak << ',' << metrics.joint_jerk_rms << ','
           << metrics.joint_jerk_peak << ','
           << metrics.position_reference_error_peak << ','
           << metrics.velocity_reference_error_peak << ',' << metrics.slack_rms
           << ',' << metrics.active_bounds << ',' << metrics.solve_mean_us << ','
           << metrics.solve_p99_us << ','
           << metrics.failures << ',' << metrics.hard_bounds_ok << ','
           << metrics.finite;
    if (extended_arm_angle_metrics) {
      output << ',' << metrics.left_arm_angle_rms << ','
             << metrics.left_arm_angle_p95 << ','
             << metrics.left_arm_angle_max << ','
             << metrics.right_arm_angle_rms << ','
             << metrics.right_arm_angle_p95 << ','
             << metrics.right_arm_angle_max << ','
             << metrics.left_outward_min << ','
             << metrics.right_outward_min << ','
             << metrics.left_outward_active_count << ','
             << metrics.right_outward_active_count << ','
             << metrics.left_arm_angle_held_count << ','
             << metrics.right_arm_angle_held_count;
    }
    output << '\n';
  }
}

int run(int argc, char** argv) {
  const Options options = parseOptions(argc, argv);
  QpIkConfig config = loadConfig(options.config_path);
  // This is an offline quality benchmark, not the real-time deadline test.
  // Avoid turning host scheduling preemption into a controller failure while
  // retaining the measured solve-time distribution in the CSV.
  config.qpoases.cpu_time_limit_seconds =
      std::max(config.qpoases.cpu_time_limit_seconds, 0.02);
  constexpr std::array<Scenario, 12> all_scenarios{
      Scenario::kStraight, Scenario::kCircle, Scenario::kFigureEight,
      Scenario::kReversal,
      Scenario::kStop, Scenario::kJitter, Scenario::kOrientation,
      Scenario::kNearLimit, Scenario::kSingular, Scenario::kUnreachable,
      Scenario::kTimestampJitter, Scenario::kDroppedFrames};
  constexpr std::array<ControllerKind, 3> controllers{
      ControllerKind::kDirectVelocityQp, ControllerKind::kOtgVelocityQp,
      ControllerKind::kOtgAccelerationQp};
  std::vector<Scenario> scenarios;
  if (options.scenario == "circle") {
    scenarios.push_back(Scenario::kCircle);
  } else {
    scenarios.assign(all_scenarios.begin(), all_scenarios.end());
  }
  std::vector<ArmAngleProfileKind> arm_angle_profiles;
  if (options.arm_angle_profile == "both") {
    arm_angle_profiles = {ArmAngleProfileKind::kFixed,
                          ArmAngleProfileKind::kDynamicConsistent};
  } else if (options.arm_angle_profile == "fixed") {
    arm_angle_profiles = {ArmAngleProfileKind::kFixed};
  } else if (options.arm_angle_profile == "dynamic_consistent") {
    arm_angle_profiles = {ArmAngleProfileKind::kDynamicConsistent};
  } else {
    arm_angle_profiles = {ArmAngleProfileKind::kLegacy};
  }
  std::vector<Metrics> results;
  results.reserve(scenarios.size() * controllers.size() *
                  arm_angle_profiles.size());
  bool accepted = true;
  for (const ArmAngleProfileKind arm_angle_profile : arm_angle_profiles) {
    for (const ControllerKind controller : controllers) {
      for (const Scenario scenario : scenarios) {
        Metrics metrics = runScenario(options, config, controller, scenario,
                                      arm_angle_profile);
        accepted = accepted && metrics.finite && metrics.hard_bounds_ok &&
                   metrics.failures == 0U;
        std::cout << toString(controller) << '/' << toString(scenario) << '/'
                  << toString(arm_angle_profile)
                  << " rms=" << metrics.position_rms
                  << " failures=" << metrics.failures
                  << " bounds=" << metrics.hard_bounds_ok << '\n';
        results.push_back(metrics);
      }
    }
  }
  writeCsv(options.output_path, results,
           options.arm_angle_profile != "legacy");
  return accepted ? 0 : 2;
}

}  // namespace
}  // namespace tianji_qp_ik

int main(int argc, char** argv) {
  try {
    return tianji_qp_ik::run(argc, argv);
  } catch (const std::exception& exception) {
    std::cerr << "fatal: " << exception.what() << '\n';
    return 1;
  }
}
