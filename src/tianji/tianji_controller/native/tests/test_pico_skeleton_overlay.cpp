#include "tianji_qp_ik/pico_skeleton_overlay.hpp"

#include <gtest/gtest.h>

#include <mujoco/mujoco.h>

#include <array>

namespace tianji_qp_ik {
namespace {

TEST(PicoSkeletonOverlay, AppendsEightJointsAndSevenBones) {
  PicoUpperLimbSkeleton skeleton;
  skeleton.valid = true;
  for (std::size_t index = 0; index < skeleton.points.size(); ++index) {
    skeleton.points[index] = Eigen::Vector3d(
        0.1 * static_cast<double>(index),
        0.2 * static_cast<double>(index),
        1.0 + 0.05 * static_cast<double>(index));
  }
  std::array<mjvGeom, 20> storage{};
  mjvScene scene;
  mjv_defaultScene(&scene);
  scene.maxgeom = static_cast<int>(storage.size());
  scene.geoms = storage.data();

  appendPicoUpperLimbSkeleton(skeleton, &scene);

  EXPECT_EQ(scene.ngeom, 15);
  int spheres = 0;
  int lines = 0;
  for (int index = 0; index < scene.ngeom; ++index) {
    spheres += scene.geoms[index].type == mjGEOM_SPHERE ? 1 : 0;
    lines += scene.geoms[index].type == mjGEOM_LINE ? 1 : 0;
    EXPECT_EQ(scene.geoms[index].category, mjCAT_DECOR);
  }
  EXPECT_EQ(spheres, 8);
  EXPECT_EQ(lines, 7);
}

TEST(PicoSkeletonOverlay, HidesInvalidSkeletonAndHonorsCapacity) {
  std::array<mjvGeom, 1> storage{};
  mjvScene scene;
  mjv_defaultScene(&scene);
  scene.maxgeom = 1;
  scene.geoms = storage.data();
  PicoUpperLimbSkeleton skeleton;

  appendPicoUpperLimbSkeleton(skeleton, &scene);
  EXPECT_EQ(scene.ngeom, 0);

  skeleton.valid = true;
  appendPicoUpperLimbSkeleton(skeleton, &scene);
  EXPECT_EQ(scene.ngeom, 1);
}

TEST(PicoSkeletonOverlay, ConvertsAndDrawsProcessedSparkSkeletonDistinctly) {
  SparkUpperTargets targets;
  targets.valid = true;
  targets.left.shoulder = {0.0, 0.2, 1.0};
  targets.left.elbow = {0.1, 0.4, 0.8};
  targets.left.wrist = {0.2, 0.5, 0.7};
  targets.left.hand = {0.3, 0.6, 0.65};
  targets.right.shoulder = {0.0, -0.2, 1.0};
  targets.right.elbow = {0.1, -0.4, 0.8};
  targets.right.wrist = {0.2, -0.5, 0.7};
  targets.right.hand = {0.3, -0.6, 0.65};
  const PicoUpperLimbSkeleton spark = sparkUpperLimbSkeleton(targets);
  ASSERT_TRUE(spark.valid);
  EXPECT_TRUE(spark.points[kPicoLeftElbowPoint].isApprox(targets.left.elbow));
  EXPECT_TRUE(spark.points[kPicoRightHandPoint].isApprox(targets.right.hand));

  PicoUpperLimbSkeleton pico = spark;
  std::array<mjvGeom, 32> storage{};
  mjvScene scene;
  mjv_defaultScene(&scene);
  scene.maxgeom = static_cast<int>(storage.size());
  scene.geoms = storage.data();
  appendPicoUpperLimbSkeleton(pico, &scene, SkeletonOverlayStyle::kPico);
  appendPicoUpperLimbSkeleton(spark, &scene, SkeletonOverlayStyle::kSpark);

  ASSERT_EQ(scene.ngeom, 30);
  EXPECT_NE(scene.geoms[0].rgba[0], scene.geoms[15].rgba[0]);
  EXPECT_NE(scene.geoms[1].rgba[1], scene.geoms[16].rgba[1]);
}

}  // namespace
}  // namespace tianji_qp_ik
