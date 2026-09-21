#include "tianji_qp_ik/benchmark_dataset.hpp"
#include "tianji_qp_ik/controller.hpp"
#include "tianji_qp_ik/mujoco_robot.hpp"
#include "tianji_qp_ik/so3.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace tianji_qp_ik {
namespace {

struct Options {
  std::string config_path{"config/qp_ik_hierarchical.yaml"};
  std::string model_path{"models/marvin_m6_qp_test.xml"};
  std::string output_path{"hierarchical_ik_ab.csv"};
  int steps{600};
};

enum class Scenario {
  kCentral,
  kCombined,
  kFast,
  kNearLimit,
  kSingular,
  kUnreachable,
};

struct Metrics {
  IkAlgorithm algorithm{IkAlgorithm::kHierarchicalQp};
  Scenario scenario{Scenario::kCentral};
  double position_rms_m{0.0};
  double orientation_rms_rad{0.0};
  double settling_time_s{0.0};
  double qdot_variation_rms{0.0};
  double maximum_velocity_ratio{0.0};
  double slack_position_rms{0.0};
  double slack_orientation_rms{0.0};
  std::uint64_t active_position_bounds{0U};
  std::uint64_t active_velocity_bounds{0U};
  double solve_p50_us{0.0};
  double solve_p95_us{0.0};
  double solve_p99_us{0.0};
  double final_position_error_m{0.0};
  double final_orientation_error_rad{0.0};
  std::uint64_t failures{0U};
  bool hard_bounds_ok{true};
  bool finite{true};
};

Options parseOptions(int argc, char** argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--help") {
      std::cout << "Usage: tianji_hierarchical_ik_benchmark [--config FILE] "
                   "[--model FILE] [--output FILE] [--steps COUNT]\n";
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
    } else {
      throw std::invalid_argument("unknown option: " + argument);
    }
  }
  if (options.steps < 20) {
    throw std::invalid_argument("--steps must be at least 20");
  }
  return options;
}

std::string toString(Scenario scenario) {
  switch (scenario) {
    case Scenario::kCentral:
      return "central";
    case Scenario::kCombined:
      return "combined";
    case Scenario::kFast:
      return "fast";
    case Scenario::kNearLimit:
      return "near_limit";
    case Scenario::kSingular:
      return "singular";
    case Scenario::kUnreachable:
      return "unreachable";
  }
  return "unknown";
}

double percentile(std::vector<double> values, double quantile) {
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const double position = quantile * static_cast<double>(values.size() - 1U);
  const std::size_t lower = static_cast<std::size_t>(std::floor(position));
  const std::size_t upper = static_cast<std::size_t>(std::ceil(position));
  const double fraction = position - static_cast<double>(lower);
  return values[lower] + fraction * (values[upper] - values[lower]);
}

Vec7 initialPosition(ArmSide side, const ArmLimits& limits, Scenario scenario) {
  if (scenario == Scenario::kSingular) {
    return safeSingularBenchmarkPosition(side);
  }
  if (scenario != Scenario::kNearLimit) {
    return 0.5 * (limits.lower_position + limits.upper_position);
  }
  Vec7 q;
  for (int index = 0; index < kArmDof; ++index) {
    q[index] = index % 2 == 0 ? limits.upper_position[index] - 0.08
                              : limits.lower_position[index] + 0.08;
  }
  return q;
}

DualArmTargets targetAt(const DualArmTargets& base, Scenario scenario,
                        double time_seconds) {
  DualArmTargets target = base;
  switch (scenario) {
    case Scenario::kCentral:
      target.left.position.x() += 0.03;
      target.right.position.x() -= 0.03;
      break;
    case Scenario::kCombined:
      target.left.position += Eigen::Vector3d(0.03, 0.02, 0.015);
      target.right.position += Eigen::Vector3d(-0.03, -0.02, 0.015);
      target.left.rotation =
          Eigen::AngleAxisd(0.15, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
          base.left.rotation;
      target.right.rotation =
          Eigen::AngleAxisd(-0.15, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
          base.right.rotation;
      break;
    case Scenario::kFast: {
      constexpr double kPi = 3.14159265358979323846;
      const double phase = 2.0 * kPi * 1.5 * time_seconds;
      const double sine = std::sin(phase);
      const double cosine = std::cos(phase);
      target.left.position += Eigen::Vector3d(0.07 * sine, 0.05 * cosine, 0.03 * sine);
      target.right.position += Eigen::Vector3d(-0.07 * sine, -0.05 * cosine, 0.03 * sine);
      target.left.rotation =
          Eigen::AngleAxisd(0.30 * sine, Eigen::Vector3d::UnitY()).toRotationMatrix() *
          base.left.rotation;
      target.right.rotation =
          Eigen::AngleAxisd(-0.30 * sine, Eigen::Vector3d::UnitY()).toRotationMatrix() *
          base.right.rotation;
      break;
    }
    case Scenario::kNearLimit:
      target.left.position.x() += 0.05;
      target.right.position.x() -= 0.05;
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
  }
  return target;
}

bool insidePositionMargins(const Vec7& q, const ArmLimits& limits,
                           double margin, double tolerance) {
  return q.allFinite() &&
         (q.array() >= limits.lower_position.array() + margin - tolerance).all() &&
         (q.array() <= limits.upper_position.array() - margin + tolerance).all();
}

Metrics runScenario(const Options& options, const QpIkConfig& base_config,
                    IkAlgorithm algorithm, Scenario scenario) {
  QpIkConfig config = base_config;
  config.ik_algorithm = algorithm;
  // This benchmark isolates Cartesian IK performance.  A world-frame elbow
  // reference can be infeasible under the side-specific J3 safety envelope
  // and would otherwise turn the singularity case into an arm-angle test.
  config.arm_angle.enabled = false;
  MujocoRobot robot(options.model_path);
  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    robot.setArmPosition(
        side, initialPosition(side, robot.mapping(side).limits, scenario));
  }
  robot.forward();
  const DualArmTargets base{robot.tcpPose(ArmSide::kLeft),
                            robot.tcpPose(ArmSide::kRight)};
  DualArmController controller(robot, config);

  Metrics metrics;
  metrics.algorithm = algorithm;
  metrics.scenario = scenario;
  const double dt = 1.0 / config.controller.rate_hz;
  Vec7 previous_left = Vec7::Zero();
  Vec7 previous_right = Vec7::Zero();
  double position_squared_sum = 0.0;
  double orientation_squared_sum = 0.0;
  double variation_squared_sum = 0.0;
  double slack_position_squared_sum = 0.0;
  double slack_orientation_squared_sum = 0.0;
  std::size_t arm_samples = 0U;
  std::size_t velocity_samples = 0U;
  std::vector<double> solve_times;
  solve_times.reserve(static_cast<std::size_t>(2 * options.steps));
  bool settled = false;
  metrics.settling_time_s = static_cast<double>(options.steps) * dt;

  for (int step = 0; step < options.steps; ++step) {
    const double time = static_cast<double>(step) * dt;
    const DualArmTargets target = targetAt(base, scenario, time);
    const ControllerDiagnostics result = controller.step(target, dt);
    if (!result.accepted) {
      ++metrics.failures;
      std::cerr << "benchmark_failure algorithm=" << toString(algorithm)
                << " scenario=" << toString(scenario)
                << " step=" << step
                << " hold_reason=" << toString(result.hold_reason)
                << " left_reason=" << toString(result.left.hold_reason)
                << " right_reason=" << toString(result.right.hold_reason)
                << " left_joint=" << result.left.safety.joint_index
                << " right_joint=" << result.right.safety.joint_index
                << " left_solver=" << static_cast<int>(result.left.ik.status)
                << " right_solver=" << static_cast<int>(result.right.ik.status)
                << '\n';
      continue;
    }

    const double left_position = result.left.pose_error.head<3>().norm();
    const double right_position = result.right.pose_error.head<3>().norm();
    const double left_orientation = result.left.pose_error.tail<3>().norm();
    const double right_orientation = result.right.pose_error.tail<3>().norm();
    position_squared_sum += left_position * left_position +
                            right_position * right_position;
    orientation_squared_sum += left_orientation * left_orientation +
                               right_orientation * right_orientation;
    slack_position_squared_sum += result.left.ik.slack.head<3>().squaredNorm() +
                                   result.right.ik.slack.head<3>().squaredNorm();
    slack_orientation_squared_sum += result.left.ik.slack.tail<3>().squaredNorm() +
                                      result.right.ik.slack.tail<3>().squaredNorm();
    variation_squared_sum +=
        (result.left.ik.qdot - previous_left).squaredNorm() +
        (result.right.ik.qdot - previous_right).squaredNorm();
    previous_left = result.left.ik.qdot;
    previous_right = result.right.ik.qdot;
    arm_samples += 2U;
    velocity_samples += static_cast<std::size_t>(2 * kArmDof);
    solve_times.push_back(result.left.ik.solve_time_us);
    solve_times.push_back(result.right.ik.solve_time_us);
    metrics.maximum_velocity_ratio = std::max(
        metrics.maximum_velocity_ratio,
        std::max(result.left.ik.qdot_max_ratio,
                 result.right.ik.qdot_max_ratio));
    metrics.active_position_bounds += static_cast<std::uint64_t>(
        result.left.ik.active_position_bound_count +
        result.right.ik.active_position_bound_count);
    metrics.active_velocity_bounds += static_cast<std::uint64_t>(
        result.left.ik.active_velocity_bound_count +
        result.right.ik.active_velocity_bound_count);

    if (!settled && left_position < 0.002 && right_position < 0.002 &&
        left_orientation < 3.14159265358979323846 / 180.0 &&
        right_orientation < 3.14159265358979323846 / 180.0) {
      metrics.settling_time_s = time;
      settled = true;
    }

    for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
      const ArmLimits& limits = robot.mapping(side).limits;
      metrics.hard_bounds_ok =
          metrics.hard_bounds_ok &&
          insidePositionMargins(robot.armPosition(side), limits,
                                config.joint_limits.margin_rad,
                                config.safety.bound_tolerance);
    }
  }

  if (arm_samples == 0U || velocity_samples == 0U) {
    metrics.finite = false;
    return metrics;
  }
  const double arm_denominator = static_cast<double>(arm_samples);
  metrics.position_rms_m = std::sqrt(position_squared_sum / arm_denominator);
  metrics.orientation_rms_rad =
      std::sqrt(orientation_squared_sum / arm_denominator);
  metrics.qdot_variation_rms =
      std::sqrt(variation_squared_sum / static_cast<double>(velocity_samples));
  metrics.slack_position_rms =
      std::sqrt(slack_position_squared_sum / arm_denominator);
  metrics.slack_orientation_rms =
      std::sqrt(slack_orientation_squared_sum / arm_denominator);
  metrics.solve_p50_us = percentile(solve_times, 0.50);
  metrics.solve_p95_us = percentile(solve_times, 0.95);
  metrics.solve_p99_us = percentile(solve_times, 0.99);
  robot.forward();
  const DualArmTargets final_target = targetAt(
      base, scenario, static_cast<double>(options.steps - 1) * dt);
  const Pose final_left = robot.tcpPose(ArmSide::kLeft);
  const Pose final_right = robot.tcpPose(ArmSide::kRight);
  metrics.final_position_error_m = std::max(
      (final_target.left.position - final_left.position).norm(),
      (final_target.right.position - final_right.position).norm());
  metrics.final_orientation_error_rad = std::max(
      rotationDistance(final_target.left.rotation, final_left.rotation),
      rotationDistance(final_target.right.rotation, final_right.rotation));
  metrics.finite =
      std::isfinite(metrics.position_rms_m) &&
      std::isfinite(metrics.orientation_rms_rad) &&
      std::isfinite(metrics.qdot_variation_rms) &&
      std::isfinite(metrics.maximum_velocity_ratio) &&
      std::isfinite(metrics.slack_position_rms) &&
      std::isfinite(metrics.slack_orientation_rms) &&
      std::isfinite(metrics.solve_p99_us) &&
      std::isfinite(metrics.final_position_error_m) &&
      std::isfinite(metrics.final_orientation_error_rad);
  return metrics;
}

void writeCsv(const std::string& path, const std::vector<Metrics>& results) {
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("cannot open benchmark output: " + path);
  }
  output << "algorithm,scenario,position_rms_m,orientation_rms_rad,settling_time_s,"
            "qdot_variation_rms,maximum_velocity_ratio,slack_position_rms,"
            "slack_orientation_rms,active_position_bounds,active_velocity_bounds,"
            "solve_p50_us,solve_p95_us,solve_p99_us,final_position_error_m,"
            "final_orientation_error_rad,failures\n";
  output << std::setprecision(12);
  for (const Metrics& metrics : results) {
    output << toString(metrics.algorithm) << ',' << toString(metrics.scenario) << ','
           << metrics.position_rms_m << ',' << metrics.orientation_rms_rad << ','
           << metrics.settling_time_s << ',' << metrics.qdot_variation_rms << ','
           << metrics.maximum_velocity_ratio << ',' << metrics.slack_position_rms << ','
           << metrics.slack_orientation_rms << ',' << metrics.active_position_bounds << ','
           << metrics.active_velocity_bounds << ',' << metrics.solve_p50_us << ','
           << metrics.solve_p95_us << ',' << metrics.solve_p99_us << ','
           << metrics.final_position_error_m << ','
           << metrics.final_orientation_error_rad << ','
           << metrics.failures << '\n';
  }
}

int run(int argc, char** argv) {
  const Options options = parseOptions(argc, argv);
  const QpIkConfig config = loadConfig(options.config_path);
  constexpr std::array<Scenario, 6> kScenarios{
      Scenario::kCentral, Scenario::kCombined, Scenario::kFast,
      Scenario::kNearLimit, Scenario::kSingular, Scenario::kUnreachable};
  constexpr std::array<IkAlgorithm, 2> kAlgorithms{
      IkAlgorithm::kHierarchicalQp, IkAlgorithm::kNullspaceDls};

  std::vector<Metrics> results;
  results.reserve(kScenarios.size() * kAlgorithms.size());
  bool accepted = true;
  double qp_p99_us = 0.0;
  for (const IkAlgorithm algorithm : kAlgorithms) {
    for (const Scenario scenario : kScenarios) {
      const Metrics metrics = runScenario(options, config, algorithm, scenario);
      std::cout << "algorithm=" << toString(metrics.algorithm)
                << " scenario=" << toString(metrics.scenario)
                << " position_rms_m=" << metrics.position_rms_m
                << " orientation_rms_rad=" << metrics.orientation_rms_rad
                << " qdot_variation_rms=" << metrics.qdot_variation_rms
                << " max_velocity_ratio=" << metrics.maximum_velocity_ratio
                << " slack_position_rms=" << metrics.slack_position_rms
                << " slack_orientation_rms=" << metrics.slack_orientation_rms
                << " final_position_error_m=" << metrics.final_position_error_m
                << " final_orientation_error_rad="
                << metrics.final_orientation_error_rad
                << " solve_p99_us=" << metrics.solve_p99_us
                << " failures=" << metrics.failures << '\n';
      accepted = accepted && metrics.finite && metrics.hard_bounds_ok &&
                 metrics.failures == 0U &&
                 metrics.maximum_velocity_ratio <=
                     config.joint_limits.velocity_scale + 1e-8;
      // Both solvers must track regular reachable tasks.  The constrained QP
      // must additionally recover from the deliberately rank-deficient
      // fixture.  DLS singular metrics remain in the CSV as a comparison, but
      // do not gate acceptance because the side-specific J3 envelope removes
      // the redundancy branch on which the legacy DLS fixture relied.
      const bool requires_reachable_tracking =
          scenario == Scenario::kCentral || scenario == Scenario::kCombined ||
          (scenario == Scenario::kSingular &&
           algorithm == IkAlgorithm::kHierarchicalQp);
      if (requires_reachable_tracking) {
        accepted = accepted && metrics.final_position_error_m < 0.002 &&
                   metrics.final_orientation_error_rad <
                       3.14159265358979323846 / 180.0;
      }
      if (algorithm == IkAlgorithm::kHierarchicalQp &&
          scenario == Scenario::kUnreachable) {
        accepted = accepted &&
                   std::hypot(metrics.slack_position_rms,
                              metrics.slack_orientation_rms) > 1e-6;
      }
      if (algorithm == IkAlgorithm::kHierarchicalQp) {
        qp_p99_us = std::max(qp_p99_us, metrics.solve_p99_us);
      }
      results.push_back(metrics);
    }
  }
  writeCsv(options.output_path, results);
  accepted = accepted && qp_p99_us < 5000.0;
  std::cout << "benchmark_complete accepted=" << accepted
            << " qp_solve_p99_max_us=" << qp_p99_us
            << " rows=" << results.size() << '\n';
  return accepted ? 0 : 2;
}

}  // namespace
}  // namespace tianji_qp_ik

int main(int argc, char** argv) {
  try {
    return tianji_qp_ik::run(argc, argv);
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return 1;
  }
}
