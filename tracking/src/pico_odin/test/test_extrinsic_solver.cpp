#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "pico_odin/extrinsic_solver.hpp"

namespace po = pico_odin;

namespace {

struct SyntheticData {
  std::vector<po::HostTimedPose> pico;
  std::vector<po::HostTimedPose> odin;
};

SyntheticData make_trajectory(
  const po::Pose3 & pelvis_T_odin,
  bool enable_pitch,
  bool enable_yaw,
  double odin_receive_lag = 0.024)
{
  const po::Pose3 odin_world_T_pico_world{
    Eigen::Quaterniond(Eigen::AngleAxisd(0.37, Eigen::Vector3d::UnitZ())),
    Eigen::Vector3d(1.2, -0.4, 0.3)};
  SyntheticData data;
  constexpr double dt = 0.02;
  constexpr int count = 280;
  for (int index = 0; index < count; ++index) {
    const double time = index * dt;
    double pitch = 0.0;
    double yaw = 0.0;
    if (enable_pitch && time <= 2.6) {
      pitch = 0.32 * std::sin(2.0 * M_PI * time / 2.6);
    }
    if (enable_yaw && time >= 2.6) {
      yaw = 0.28 * std::sin(2.0 * M_PI * (time - 2.6) / 2.8);
    }
    const Eigen::Quaterniond rotation(
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
    const Eigen::Vector3d translation(
      0.08 * std::sin(0.7 * time),
      0.03 * std::sin(0.5 * time),
      1.02 + 0.04 * std::sin(0.9 * time));
    const po::Pose3 pico_world_T_pelvis{rotation, translation};
    const po::Pose3 odin_world_T_odin = po::compose(
      po::compose(odin_world_T_pico_world, pico_world_T_pelvis),
      pelvis_T_odin);

    data.pico.push_back({time, pico_world_T_pelvis});
    data.odin.push_back({time + odin_receive_lag, odin_world_T_odin});
  }
  return data;
}

po::Pose3 expected_extrinsics() {
  return po::Pose3{
    Eigen::Quaterniond(
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(0.14, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(-0.03, Eigen::Vector3d::UnitX())),
    Eigen::Vector3d(-0.27, 0.01, 0.09)};
}

}  // namespace

TEST(ExtrinsicSolver, RecoversKnownBackMountedTransformAndReceiveLag) {
  const auto expected = expected_extrinsics();
  const auto data = make_trajectory(expected, true, true);
  po::SolverOptions options;
  options.min_pairs = 80;

  const auto result = po::solve_extrinsics(data.pico, data.odin, options);

  ASSERT_TRUE(result.success) << result.message;
  EXPECT_LT(po::angular_distance(result.pelvis_T_odin.rotation, expected.rotation), 0.035);
  EXPECT_LT((result.pelvis_T_odin.translation - expected.translation).norm(), 0.03);
  EXPECT_NEAR(result.receive_lag_sec, 0.024, 0.006);
  EXPECT_GE(result.metrics.sample_count, 80);
  EXPECT_GT(result.metrics.refinement_iterations, 0);
}

TEST(ExtrinsicSolver, EstimatesReceiveLagOnExplicitHostTimeline) {
  const auto data = make_trajectory(expected_extrinsics(), true, true, 0.024);
  std::vector<po::HostTimedPose> pico;
  std::vector<po::HostTimedPose> odin;
  pico.reserve(data.pico.size());
  odin.reserve(data.odin.size());
  for (const auto & sample : data.pico) {
    pico.push_back({sample.receipt_sec, sample.pose});
  }
  for (const auto & sample : data.odin) {
    odin.push_back({sample.receipt_sec, sample.pose});
  }

  const auto estimate = po::estimate_receive_lag(pico, odin, po::SolverOptions{});

  ASSERT_TRUE(estimate.valid);
  EXPECT_NEAR(estimate.lag_sec, 0.024, 0.006);
  EXPECT_GT(estimate.correlation, 0.25);
  EXPECT_FALSE(estimate.at_boundary);
}

TEST(ExtrinsicSolver, ReceiveLagApiRejectsInvalidSearchConfiguration) {
  const auto data = make_trajectory(expected_extrinsics(), true, true);
  po::SolverOptions options;
  options.receive_lag_step_sec = std::numeric_limits<double>::infinity();

  EXPECT_FALSE(po::estimate_receive_lag(data.pico, data.odin, options).valid);

  options = po::SolverOptions{};
  auto unsorted = data.pico;
  std::swap(unsorted[0], unsorted[1]);
  EXPECT_FALSE(po::estimate_receive_lag(unsorted, data.odin, options).valid);
}

TEST(ExtrinsicSolver, RejectsSparsePoseOutliersDuringRobustSolve) {
  const auto expected = expected_extrinsics();
  auto data = make_trajectory(expected, true, true);
  for (std::size_t index = 0; index < data.odin.size(); ++index) {
    data.odin[index].pose.translation += Eigen::Vector3d(
      0.0015 * std::sin(0.31 * index),
      0.0010 * std::cos(0.23 * index),
      0.0012 * std::sin(0.17 * index));
  }
  for (const std::size_t index : {70U, 125U, 200U}) {
    data.odin[index].pose.translation += Eigen::Vector3d(0.35, -0.20, 0.25);
    data.odin[index].pose.rotation = Eigen::Quaterniond(
      Eigen::AngleAxisd(0.35, Eigen::Vector3d(1.0, 1.0, 0.5).normalized())) *
      data.odin[index].pose.rotation;
  }

  const auto result = po::solve_extrinsics(data.pico, data.odin, po::SolverOptions{});

  ASSERT_TRUE(result.success) << result.message;
  EXPECT_LT(po::angular_distance(result.pelvis_T_odin.rotation, expected.rotation), 0.08);
  EXPECT_LT((result.pelvis_T_odin.translation - expected.translation).norm(), 0.07);
}

TEST(ExtrinsicSolver, RejectsStaticTrajectory) {
  const auto data = make_trajectory(expected_extrinsics(), false, false, 0.0);
  const auto result = po::solve_extrinsics(data.pico, data.odin, po::SolverOptions{});

  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("pitch excitation"), std::string::npos);
}

TEST(ExtrinsicSolver, RejectsPitchOnlyTrajectoryAsTranslationDegenerate) {
  const auto data = make_trajectory(expected_extrinsics(), true, false, 0.0);
  const auto result = po::solve_extrinsics(data.pico, data.odin, po::SolverOptions{});

  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("yaw excitation"), std::string::npos);
}

TEST(ExtrinsicSolver, SideOffsetPriorRegularizesWeaklyKnownMountingY) {
  auto expected = expected_extrinsics();
  expected.translation.y() = 0.12;
  const auto data = make_trajectory(expected, true, true);

  po::SolverOptions weak_prior;
  weak_prior.y_prior_sigma_m = 1e6;
  weak_prior.max_translation_rms_m = 1.0;
  const auto weak = po::solve_extrinsics(data.pico, data.odin, weak_prior);

  po::SolverOptions strong_prior = weak_prior;
  strong_prior.y_prior_sigma_m = 0.002;
  const auto strong = po::solve_extrinsics(data.pico, data.odin, strong_prior);

  ASSERT_TRUE(weak.success) << weak.message;
  ASSERT_TRUE(strong.success) << strong.message;
  EXPECT_LT(
    std::abs(strong.pelvis_T_odin.translation.y()),
    std::abs(weak.pelvis_T_odin.translation.y()) * 0.5);
}

TEST(ExtrinsicSolver, RejectsReceiveLagAtSearchBoundary) {
  const auto data = make_trajectory(expected_extrinsics(), true, true, 0.30);

  const auto result = po::solve_extrinsics(data.pico, data.odin, po::SolverOptions{});

  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("time alignment"), std::string::npos);
}

TEST(ExtrinsicSolver, RejectsPoorMotionCorrelationBeforeHandEyeSolve) {
  auto data = make_trajectory(expected_extrinsics(), true, true);
  for (auto & sample : data.odin) {
    sample.pose.rotation = Eigen::Quaterniond::Identity();
  }

  const auto result = po::solve_extrinsics(data.pico, data.odin, po::SolverOptions{});

  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("time alignment"), std::string::npos);
}

TEST(ExtrinsicSolver, RejectsInvalidNumericalOptions) {
  const auto data = make_trajectory(expected_extrinsics(), true, true);
  po::SolverOptions options;
  options.y_prior_sigma_m = 0.0;

  const auto result = po::solve_extrinsics(data.pico, data.odin, options);

  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("solver options"), std::string::npos);
}

TEST(ExtrinsicSolver, AppliesConfiguredBackMountBounds) {
  const auto data = make_trajectory(expected_extrinsics(), true, true);
  po::SolverOptions options;
  options.mount_x_min_m = -0.20;

  const auto result = po::solve_extrinsics(data.pico, data.odin, options);

  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("physical bounds"), std::string::npos);
}

TEST(ExtrinsicSolver, RejectsANonUprightFinalNeutralWindow) {
  const auto expected = expected_extrinsics();
  auto data = make_trajectory(expected, true, true);
  const po::Pose3 odin_world_T_pico_world = po::compose(
    data.odin.front().pose,
    po::inverse(po::compose(data.pico.front().pose, expected)));
  for (std::size_t index = data.pico.size() - 10; index < data.pico.size(); ++index) {
    data.pico[index].pose.rotation = Eigen::Quaterniond(
      Eigen::AngleAxisd(0.55, Eigen::Vector3d::UnitX()));
    data.odin[index].pose = po::compose(
      po::compose(odin_world_T_pico_world, data.pico[index].pose), expected);
  }

  const auto result = po::solve_extrinsics(
    data.pico, data.odin, po::SolverOptions{});

  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("final neutral"), std::string::npos);
}
