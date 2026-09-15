#include "tianji_qp_ik/benchmark_dataset.hpp"
#include "tianji_qp_ik/config.hpp"
#include "tianji_qp_ik/mujoco_robot.hpp"

#include <gtest/gtest.h>

#include <Eigen/SVD>

#include <algorithm>
#include <array>
#include <cstddef>
#include <iostream>
#include <string>

namespace tianji_qp_ik {
namespace {

std::string sourcePath(const std::string& relative) {
  return std::string(TIANJI_PROJECT_SOURCE_DIR) + "/" + relative;
}

TEST(BenchmarkDatasetTest, SafeSingularFixtureIsInteriorAndRankDeficient) {
  const QpIkConfig config =
      loadConfig(sourcePath("config/qp_ik_hierarchical.yaml"));
  MujocoRobot robot(sourcePath("models/marvin_m6_qp_test.xml"));
  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    const Vec7 q = safeSingularBenchmarkPosition(side);
    const ArmLimits& limits = robot.mapping(side).limits;
    EXPECT_TRUE((q.array() >= limits.lower_position.array() +
                                 config.joint_limits.margin_rad)
                    .all());
    EXPECT_TRUE((q.array() <= limits.upper_position.array() -
                                 config.joint_limits.margin_rad)
                    .all());
    robot.setArmPosition(side, q);
  }
  robot.forward();

  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    const Eigen::JacobiSVD<Mat67> decomposition(
        robot.tcpJacobianWorld(side));
    const auto singular_values = decomposition.singularValues();
    double minimum_singular_value = singular_values[0];
    for (Eigen::Index index = 1; index < singular_values.size(); ++index) {
      minimum_singular_value =
          std::min(minimum_singular_value, singular_values[index]);
    }
    EXPECT_LT(minimum_singular_value, 1e-4);
  }
}

TEST(BenchmarkDatasetTest, IsDeterministicBalancedAndFinite) {
  const QpIkConfig config = loadConfig(sourcePath("config/qp_ik.yaml"));
  const std::string model = sourcePath("models/marvin_m6_qp_test.xml");

  const auto first = generateBenchmarkDataset(model, config, 103U, 20260809U);
  const auto repeat = generateBenchmarkDataset(model, config, 103U, 20260809U);
  const auto changed = generateBenchmarkDataset(model, config, 103U, 20260810U);
  std::cout << "benchmark_dataset_seed=20260809 hash=0x" << std::hex
            << hashBenchmarkDataset(first) << std::dec << '\n';

  ASSERT_EQ(first.size(), 103U);
  EXPECT_EQ(hashBenchmarkDataset(first), hashBenchmarkDataset(repeat));
  EXPECT_NE(hashBenchmarkDataset(first), hashBenchmarkDataset(changed));

  const std::array<std::size_t, kBenchmarkCategoryCount> counts =
      benchmarkCategoryCounts(first);
  const auto [minimum, maximum] = std::minmax_element(counts.begin(), counts.end());
  EXPECT_LE(*maximum - *minimum, 1U);

  for (const BenchmarkSample& sample : first) {
    for (const QpProblem7* problem : {&sample.left, &sample.right}) {
      EXPECT_TRUE(problem->H.allFinite());
      EXPECT_TRUE(problem->g.allFinite());
      EXPECT_TRUE(problem->lower.allFinite());
      EXPECT_TRUE(problem->upper.allFinite());
      EXPECT_TRUE((problem->lower.array() <= problem->upper.array()).all());
    }
  }
}

}  // namespace
}  // namespace tianji_qp_ik
