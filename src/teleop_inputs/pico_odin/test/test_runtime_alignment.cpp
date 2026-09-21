#include <cmath>
#include <array>
#include <vector>

#include <gtest/gtest.h>

#include "pico_odin/runtime_alignment.hpp"

namespace po = pico_odin;

namespace {

po::Pose3 mounting_transform() {
  return po::Pose3{
    Eigen::Quaterniond(
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(0.12, Eigen::Vector3d::UnitY())),
    Eigen::Vector3d(-0.26, 0.0, 0.08)};
}

}  // namespace

TEST(RuntimeAlignment, InitialHeightComesFromPicoAndThenUsesOdinDelta) {
  const auto pelvis_T_odin = mounting_transform();
  po::RuntimeAlignment alignment(pelvis_T_odin);
  const po::Pose3 pico_world_T_pelvis0{
    Eigen::Quaterniond(Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ())),
    Eigen::Vector3d(0.4, -0.2, 1.03)};
  const po::Pose3 odin_world_T_odin0{
    pico_world_T_pelvis0.rotation * pelvis_T_odin.rotation,
    Eigen::Vector3d::Zero()};
  alignment.initialize(pico_world_T_pelvis0, odin_world_T_odin0);

  const auto initial = alignment.corrected_pelvis(odin_world_T_odin0);
  EXPECT_LT((initial.translation - pico_world_T_pelvis0.translation).norm(), 1e-10);
  EXPECT_LT(po::angular_distance(initial.rotation, pico_world_T_pelvis0.rotation), 1e-10);

  auto moved = odin_world_T_odin0;
  moved.translation.z() += 0.12;
  const auto output = alignment.corrected_pelvis(moved);
  EXPECT_NEAR(output.translation.z(), pico_world_T_pelvis0.translation.z() + 0.12, 1e-10);
}

TEST(RuntimeAlignment, ClearRemovesOnlySessionAlignment) {
  po::RuntimeAlignment alignment(mounting_transform());
  alignment.initialize(po::Pose3{}, po::Pose3{});
  ASSERT_TRUE(alignment.initialized());

  alignment.clear();

  EXPECT_FALSE(alignment.initialized());
  EXPECT_THROW(alignment.corrected_pelvis(po::Pose3{}), std::logic_error);
}

TEST(RuntimeAlignment, SkeletonAnchoringRigidlyTransformsAllTwentyFourJoints) {
  po::RuntimeAlignment alignment(mounting_transform());
  std::vector<po::Pose3> joints;
  for (int index = 0; index < 24; ++index) {
    joints.push_back(po::Pose3{
      Eigen::Quaterniond(Eigen::AngleAxisd(0.01 * index, Eigen::Vector3d::UnitX())),
      Eigen::Vector3d(0.02 * index, 0.01 * (index % 3), 1.0 + 0.015 * index)});
  }
  const po::Pose3 corrected_root{
    Eigen::Quaterniond(Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitZ())),
    Eigen::Vector3d(2.0, -1.0, 1.08)};

  const auto anchored = alignment.anchor_skeleton(joints, corrected_root);

  ASSERT_EQ(anchored.size(), 24U);
  EXPECT_LT((anchored.front().translation - corrected_root.translation).norm(), 1e-10);
  EXPECT_LT(po::angular_distance(anchored.front().rotation, corrected_root.rotation), 1e-10);
  for (std::size_t index = 1; index < joints.size(); ++index) {
    const double original_length =
      (joints[index].translation - joints.front().translation).norm();
    const double anchored_length =
      (anchored[index].translation - anchored.front().translation).norm();
    EXPECT_NEAR(anchored_length, original_length, 1e-10);
  }
}

TEST(RuntimeAlignment, TwistCovarianceUsesRigidBodyAdjoint) {
  const po::Pose3 pelvis_T_odin{
    Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.0, 0.0, 0.2)};
  std::array<double, 36> covariance{};
  covariance[3 * 6 + 3] = 0.04;

  const auto transformed = po::transform_covariance(
    covariance, po::twist_adjoint(pelvis_T_odin));

  EXPECT_NEAR(transformed[1 * 6 + 1], 0.0016, 1e-12);
  EXPECT_NEAR(transformed[3 * 6 + 3], 0.04, 1e-12);
}

TEST(RuntimeAlignment, UnknownCovarianceMarkerIsPreserved) {
  std::array<double, 36> covariance{};
  covariance[0] = -1.0;

  const auto transformed = po::transform_covariance(
    covariance, Eigen::Matrix<double, 6, 6>::Identity());

  EXPECT_EQ(transformed, covariance);
}
