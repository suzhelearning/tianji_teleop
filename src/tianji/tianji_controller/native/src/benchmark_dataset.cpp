#include "tianji_qp_ik/benchmark_dataset.hpp"

#include "tianji_qp_ik/cartesian_servo.hpp"
#include "tianji_qp_ik/mujoco_robot.hpp"
#include "tianji_qp_ik/qp_builder.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <random>
#include <stdexcept>

namespace tianji_qp_ik {
namespace {

constexpr std::uint64_t kFnvOffset = 1469598103934665603ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

void hashWord(std::uint64_t word, std::uint64_t& hash) {
  for (int byte = 0; byte < 8; ++byte) {
    hash ^= (word >> (8 * byte)) & 0xffULL;
    hash *= kFnvPrime;
  }
}

void hashDouble(double value, std::uint64_t& hash) {
  std::uint64_t word = 0;
  static_assert(sizeof(word) == sizeof(value), "unexpected double width");
  std::memcpy(&word, &value, sizeof(word));
  hashWord(word, hash);
}

Vec7 sampleJointPosition(ArmSide side, const ArmLimits& limits,
                         BenchmarkCategory category,
                         std::size_t sample_index, std::mt19937& generator,
                         double margin_rad) {
  const Vec7 center = 0.5 * (limits.lower_position + limits.upper_position);
  const Vec7 half_range = 0.5 * (limits.upper_position - limits.lower_position);
  std::uniform_real_distribution<double> random_unit(-1.0, 1.0);
  Vec7 q = center;

  switch (category) {
    case BenchmarkCategory::kNominal:
      for (int joint = 0; joint < kArmDof; ++joint) {
        q[joint] += 0.35 * half_range[joint] * random_unit(generator);
      }
      break;
    case BenchmarkCategory::kJointLimit: {
      for (int joint = 0; joint < kArmDof; ++joint) {
        q[joint] += 0.15 * half_range[joint] * random_unit(generator);
      }
      const int joint = static_cast<int>(sample_index % static_cast<std::size_t>(kArmDof));
      const bool upper = ((sample_index / static_cast<std::size_t>(kArmDof)) % 2U) == 0U;
      q[joint] = upper ? limits.upper_position[joint] - margin_rad - 1e-4
                       : limits.lower_position[joint] + margin_rad + 1e-4;
      break;
    }
    case BenchmarkCategory::kSingular:
      q = safeSingularBenchmarkPosition(side);
      for (int joint = 0; joint < kArmDof; ++joint) {
        q[joint] += 1e-5 * random_unit(generator);
      }
      break;
    case BenchmarkCategory::kUnreachable:
      for (int joint = 0; joint < kArmDof; ++joint) {
        q[joint] += 0.20 * half_range[joint] * random_unit(generator);
      }
      break;
    case BenchmarkCategory::kContinuous: {
      const double phase = 0.013 * static_cast<double>(sample_index);
      for (int joint = 0; joint < kArmDof; ++joint) {
        q[joint] += 0.30 * half_range[joint] *
                    std::sin(phase + 0.37 * static_cast<double>(joint));
      }
      break;
    }
  }
  return q.cwiseMax((limits.lower_position.array() + margin_rad + 1e-6).matrix())
      .cwiseMin((limits.upper_position.array() - margin_rad - 1e-6).matrix());
}

Pose desiredPose(const Pose& current, BenchmarkCategory category, std::size_t sample_index,
                 ArmSide side, std::mt19937& generator) {
  std::uniform_real_distribution<double> random_unit(-1.0, 1.0);
  const double sign = side == ArmSide::kLeft ? 1.0 : -1.0;
  Pose desired = current;
  Eigen::Vector3d translation;
  Eigen::Vector3d rotation_vector;

  if (category == BenchmarkCategory::kUnreachable) {
    translation = Eigen::Vector3d(2.0, 1.5 * sign, 1.0);
    rotation_vector = Eigen::Vector3d(1.2, -0.8 * sign, 0.6);
  } else if (category == BenchmarkCategory::kContinuous) {
    const double phase = 0.013 * static_cast<double>(sample_index);
    translation = Eigen::Vector3d(0.04 * std::cos(phase),
                                  sign * 0.035 * std::sin(phase),
                                  0.025 * std::sin(2.0 * phase));
    rotation_vector = Eigen::Vector3d(0.15 * std::sin(phase),
                                      sign * 0.12 * std::cos(phase),
                                      0.10 * std::sin(0.5 * phase));
  } else {
    translation = Eigen::Vector3d(0.03 * random_unit(generator),
                                  0.03 * random_unit(generator),
                                  0.03 * random_unit(generator));
    rotation_vector = Eigen::Vector3d(0.12 * random_unit(generator),
                                      0.12 * random_unit(generator),
                                      0.12 * random_unit(generator));
  }

  desired.position += translation;
  const double angle = rotation_vector.norm();
  if (angle > 0.0) {
    desired.rotation = Eigen::AngleAxisd(angle, rotation_vector / angle).toRotationMatrix() *
                       current.rotation;
  }
  return desired;
}

QpProblem7 buildArmProblem(MujocoRobot& robot, ArmSide side, const QpIkConfig& config,
                           const QpBuilder& builder, BenchmarkCategory category,
                           std::size_t sample_index, std::mt19937& generator) {
  const Pose current = robot.tcpPose(side);
  const Pose desired = desiredPose(current, category, sample_index, side, generator);
  const Vec6 twist = cartesianServoTwist(config.cartesian_servo, desired, current);
  return builder.build(robot.tcpJacobianWorld(side), twist, robot.armPosition(side),
                       robot.mapping(side).limits, 1.0 / config.controller.rate_hz);
}

void hashProblem(const QpProblem7& problem, std::uint64_t& hash) {
  for (int column = 0; column < kArmDof; ++column) {
    for (int row = 0; row < kArmDof; ++row) {
      hashDouble(problem.H(row, column), hash);
    }
  }
  for (int row = 0; row < kArmDof; ++row) {
    hashDouble(problem.g[row], hash);
    hashDouble(problem.lower[row], hash);
    hashDouble(problem.upper[row], hash);
  }
}

}  // namespace

Vec7 safeSingularBenchmarkPosition(ArmSide side) {
  Vec7 singular;
  singular << -1.52612342198, -4.2180397057e-06, -1.57081169724,
      -2.4307, 0.00799372293299, 0.9472, 1.4708;
  if (side == ArmSide::kLeft) {
    // J1 only rotates this rank-deficient fixture about the base.  Keep it
    // well inside the left teleoperation envelope so the singularity test does
    // not simultaneously become a joint-limit test.
    singular[0] = 0.0;
  } else {
    // Mirror J3 into the right arm's permitted half-space.
    singular[2] = -singular[2];
  }
  return singular;
}

std::vector<BenchmarkSample> generateBenchmarkDataset(const std::string& model_path,
                                                       const QpIkConfig& config,
                                                       std::size_t sample_count,
                                                       std::uint32_t seed) {
  if (config.controller.rate_hz <= 0.0) {
    throw std::invalid_argument("benchmark controller rate must be positive");
  }
  MujocoRobot robot(model_path);
  QpBuilder builder(config.qp, config.joint_limits);
  std::mt19937 generator(seed);
  std::vector<BenchmarkSample> samples;
  samples.reserve(sample_count);

  for (std::size_t index = 0; index < sample_count; ++index) {
    const BenchmarkCategory category =
        static_cast<BenchmarkCategory>(index % kBenchmarkCategoryCount);
    robot.setArmPosition(ArmSide::kLeft,
                         sampleJointPosition(ArmSide::kLeft,
                                             robot.mapping(ArmSide::kLeft).limits, category,
                                             index, generator, config.joint_limits.margin_rad));
    robot.setArmPosition(ArmSide::kRight,
                         sampleJointPosition(ArmSide::kRight,
                                             robot.mapping(ArmSide::kRight).limits, category,
                                             index + 17U, generator,
                                             config.joint_limits.margin_rad));
    robot.forward();

    BenchmarkSample sample;
    sample.category = category;
    sample.left = buildArmProblem(robot, ArmSide::kLeft, config, builder, category, index,
                                  generator);
    sample.right = buildArmProblem(robot, ArmSide::kRight, config, builder, category, index,
                                   generator);
    samples.push_back(sample);
  }
  return samples;
}

std::uint64_t hashBenchmarkDataset(const std::vector<BenchmarkSample>& samples) {
  std::uint64_t hash = kFnvOffset;
  hashWord(static_cast<std::uint64_t>(samples.size()), hash);
  for (const BenchmarkSample& sample : samples) {
    hashWord(static_cast<std::uint64_t>(sample.category), hash);
    hashProblem(sample.left, hash);
    hashProblem(sample.right, hash);
  }
  return hash;
}

std::array<std::size_t, kBenchmarkCategoryCount> benchmarkCategoryCounts(
    const std::vector<BenchmarkSample>& samples) {
  std::array<std::size_t, kBenchmarkCategoryCount> result{};
  for (const BenchmarkSample& sample : samples) {
    ++result.at(static_cast<std::size_t>(sample.category));
  }
  return result;
}

std::string benchmarkCategoryName(BenchmarkCategory category) {
  switch (category) {
    case BenchmarkCategory::kNominal:
      return "nominal";
    case BenchmarkCategory::kJointLimit:
      return "joint_limit";
    case BenchmarkCategory::kSingular:
      return "singular";
    case BenchmarkCategory::kUnreachable:
      return "unreachable";
    case BenchmarkCategory::kContinuous:
      return "continuous";
  }
  return "unknown";
}

}  // namespace tianji_qp_ik
