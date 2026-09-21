#include "tianji_qp_ik/interactive_marker.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <limits>
#include <optional>

namespace tianji_qp_ik {
namespace {

Pose identityPose() {
  return Pose{};
}

MarkerCamera frontCamera() {
  MarkerCamera camera;
  camera.forward = Eigen::Vector3d::UnitY();
  camera.right = Eigen::Vector3d::UnitX();
  camera.up = Eigen::Vector3d::UnitZ();
  camera.world_per_ndc = 1.0;
  return camera;
}

MarkerPointer pointerAt(double axis_parameter) {
  MarkerPointer pointer;
  pointer.ray.origin = Eigen::Vector3d(axis_parameter, -1.0, 0.0);
  pointer.ray.direction = Eigen::Vector3d::UnitY();
  pointer.ndc = Eigen::Vector2d(axis_parameter, 0.0);
  return pointer;
}

MarkerPointer pointerThrough(const Eigen::Vector3d& point,
                             const Eigen::Vector3d& direction,
                             const Eigen::Vector2d& ndc = Eigen::Vector2d::Zero()) {
  MarkerPointer pointer;
  pointer.ray.origin = point - direction.normalized();
  pointer.ray.direction = direction.normalized();
  pointer.ndc = ndc;
  return pointer;
}

double ringRadius() {
  return MarkerStyle{}.scale * MarkerStyle{}.ring_radius_ratio;
}

TEST(InteractiveMarkerTest, PicksTranslationXArrow) {
  InteractiveMarker6D marker;
  const Ray3d ray{Eigen::Vector3d(0.12, -1.0, 0.0), Eigen::Vector3d::UnitY()};

  EXPECT_EQ(marker.pick(identityPose(), ray), MarkerHandle::kTranslateX);
}

TEST(InteractiveMarkerTest, TranslationXPreservesOtherComponents) {
  InteractiveMarker6D marker;
  const Pose start = identityPose();
  ASSERT_TRUE(marker.beginDrag(MarkerHandle::kTranslateX, start, pointerAt(0.10),
                               frontCamera()));

  const std::optional<Pose> moved = marker.updateDrag(pointerAt(0.16));

  ASSERT_TRUE(moved.has_value());
  EXPECT_NEAR(moved->position.x(), 0.06, 1e-9);
  EXPECT_NEAR(moved->position.y(), 0.0, 1e-12);
  EXPECT_NEAR(moved->position.z(), 0.0, 1e-12);
  EXPECT_TRUE(moved->rotation.isApprox(start.rotation, 1e-12));
}

TEST(InteractiveMarkerTest, RejectsNonFinitePickRay) {
  InteractiveMarker6D marker;
  Ray3d ray{Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX()};
  ray.origin.x() = std::numeric_limits<double>::quiet_NaN();

  EXPECT_EQ(marker.pick(identityPose(), ray), MarkerHandle::kNone);
}

TEST(InteractiveMarkerTest, PicksCenterAndAllRotationRings) {
  InteractiveMarker6D marker;
  const Pose pose = identityPose();
  const double component = ringRadius() / std::sqrt(2.0);

  EXPECT_EQ(marker.pick(pose, Ray3d{Eigen::Vector3d(0.0, 0.0, -1.0),
                                    Eigen::Vector3d::UnitZ()}),
            MarkerHandle::kCenter);
  EXPECT_EQ(marker.pick(pose,
                        pointerThrough(Eigen::Vector3d(0.0, component, component),
                                       Eigen::Vector3d::UnitX())
                            .ray),
            MarkerHandle::kRotateX);
  EXPECT_EQ(marker.pick(pose,
                        pointerThrough(Eigen::Vector3d(component, 0.0, component),
                                       Eigen::Vector3d::UnitY())
                            .ray),
            MarkerHandle::kRotateY);
  EXPECT_EQ(marker.pick(pose,
                        pointerThrough(Eigen::Vector3d(component, component, 0.0),
                                       Eigen::Vector3d::UnitZ())
                            .ray),
            MarkerHandle::kRotateZ);
}

TEST(InteractiveMarkerTest, CenterDragMovesInCapturedCameraPlane) {
  InteractiveMarker6D marker;
  const Pose start = identityPose();
  const MarkerPointer begin =
      pointerThrough(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitY());
  ASSERT_TRUE(marker.beginDrag(MarkerHandle::kCenter, start, begin, frontCamera()));

  const MarkerPointer current =
      pointerThrough(Eigen::Vector3d(0.05, 0.0, 0.03), Eigen::Vector3d::UnitY());
  const std::optional<Pose> moved = marker.updateDrag(current);

  ASSERT_TRUE(moved.has_value());
  EXPECT_TRUE(moved->position.isApprox(Eigen::Vector3d(0.05, 0.0, 0.03), 1e-12));
  EXPECT_TRUE(moved->rotation.isApprox(start.rotation, 1e-12));
}

TEST(InteractiveMarkerTest, RotateWorldZPreservesPositionAndProducesProperRotation) {
  InteractiveMarker6D marker;
  const Pose start = identityPose();
  const double radius = ringRadius();
  MarkerCamera camera;
  camera.forward = Eigen::Vector3d::UnitZ();
  camera.right = Eigen::Vector3d::UnitX();
  camera.up = Eigen::Vector3d::UnitY();
  ASSERT_TRUE(marker.beginDrag(
      MarkerHandle::kRotateZ, start,
      pointerThrough(Eigen::Vector3d(radius, 0.0, 0.0), Eigen::Vector3d::UnitZ()), camera));

  constexpr double kAngle = 0.4;
  const Eigen::Vector3d radial(radius * std::cos(kAngle), radius * std::sin(kAngle), 0.0);
  const std::optional<Pose> moved =
      marker.updateDrag(pointerThrough(radial, Eigen::Vector3d::UnitZ()));

  ASSERT_TRUE(moved.has_value());
  EXPECT_TRUE(moved->position.isApprox(start.position, 1e-12));
  const Eigen::Matrix3d expected =
      Eigen::AngleAxisd(kAngle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  EXPECT_TRUE(moved->rotation.isApprox(expected, 1e-9));
  EXPECT_NEAR(moved->rotation.determinant(), 1.0, 1e-12);
}

TEST(InteractiveMarkerTest, RotateLocalZUsesCapturedTargetAxis) {
  InteractiveMarker6D marker;
  marker.setFrame(MarkerFrame::kLocal);
  Pose start = identityPose();
  start.rotation = Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const Eigen::Vector3d axis = start.rotation.col(2);
  const Eigen::Vector3d radial_start = ringRadius() * start.rotation.col(0);
  MarkerCamera camera;
  camera.forward = axis;
  camera.right = start.rotation.col(0);
  camera.up = start.rotation.col(1);
  ASSERT_TRUE(marker.beginDrag(MarkerHandle::kRotateZ, start,
                               pointerThrough(radial_start, axis), camera));

  constexpr double kAngle = 0.4;
  const Eigen::Vector3d radial_current =
      Eigen::AngleAxisd(kAngle, axis) * radial_start;
  const std::optional<Pose> moved =
      marker.updateDrag(pointerThrough(radial_current, axis));

  ASSERT_TRUE(moved.has_value());
  EXPECT_TRUE(moved->position.isApprox(start.position, 1e-12));
  const Eigen::Matrix3d expected =
      Eigen::AngleAxisd(kAngle, axis).toRotationMatrix() * start.rotation;
  EXPECT_TRUE(moved->rotation.isApprox(expected, 1e-9));
}

TEST(InteractiveMarkerTest, ParallelTranslationFallsBackToScreenAxis) {
  InteractiveMarker6D marker;
  const Pose start = identityPose();
  MarkerCamera camera;
  camera.forward = Eigen::Vector3d::UnitX();
  camera.right = Eigen::Vector3d::UnitX();
  camera.up = Eigen::Vector3d::UnitZ();
  camera.world_per_ndc = 0.5;
  MarkerPointer begin;
  begin.ray = Ray3d{Eigen::Vector3d(-1.0, 0.0, 0.0), Eigen::Vector3d::UnitX()};
  begin.ndc = Eigen::Vector2d::Zero();
  ASSERT_TRUE(marker.beginDrag(MarkerHandle::kTranslateX, start, begin, camera));

  MarkerPointer current = begin;
  current.ndc.x() = 0.2;
  const std::optional<Pose> moved = marker.updateDrag(current);

  ASSERT_TRUE(moved.has_value());
  EXPECT_NEAR(moved->position.x(), 0.1, 1e-12);
}

TEST(InteractiveMarkerTest, ActiveHandleRemainsCapturedOutsidePickRegion) {
  InteractiveMarker6D marker;
  ASSERT_TRUE(marker.beginDrag(MarkerHandle::kTranslateX, identityPose(), pointerAt(0.10),
                               frontCamera()));
  EXPECT_EQ(marker.activeHandle(), MarkerHandle::kTranslateX);

  const std::optional<Pose> moved = marker.updateDrag(pointerAt(0.50));

  ASSERT_TRUE(moved.has_value());
  EXPECT_NEAR(moved->position.x(), 0.40, 1e-12);
  EXPECT_EQ(marker.activeHandle(), MarkerHandle::kTranslateX);
  marker.endDrag();
  EXPECT_EQ(marker.activeHandle(), MarkerHandle::kNone);
}

TEST(InteractiveMarkerTest, RejectsDegenerateAndNanDragUpdates) {
  InteractiveMarker6D marker;
  MarkerPointer zero_direction = pointerAt(0.10);
  zero_direction.ray.direction.setZero();
  EXPECT_FALSE(marker.beginDrag(MarkerHandle::kTranslateX, identityPose(), zero_direction,
                                frontCamera()));

  ASSERT_TRUE(marker.beginDrag(MarkerHandle::kTranslateX, identityPose(), pointerAt(0.10),
                               frontCamera()));
  MarkerPointer non_finite = pointerAt(0.20);
  non_finite.ray.origin.y() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(marker.updateDrag(non_finite).has_value());
  marker.cancelDrag();
  EXPECT_EQ(marker.activeHandle(), MarkerHandle::kNone);
}

TEST(InteractiveMarkerTest, WorldAndLocalGeometryUseDifferentAxes) {
  Pose pose = identityPose();
  pose.rotation = Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitY()).toRotationMatrix();
  InteractiveMarker6D marker;
  EXPECT_TRUE(marker.geometry(pose).axes[2].isApprox(Eigen::Vector3d::UnitZ(), 1e-12));

  marker.setFrame(MarkerFrame::kLocal);

  EXPECT_TRUE(marker.geometry(pose).axes[2].isApprox(pose.rotation.col(2), 1e-12));
}

TEST(ManualTargetPreviewTest, KeepsNewestPoseUntilAcknowledged) {
  ManualTargetPreview preview;
  const Pose snapshot = identityPose();
  Pose requested = snapshot;
  requested.position.x() = 0.25;
  preview.record(ArmSide::kLeft, requested, 42U);

  const Pose pending = preview.resolve(ArmSide::kLeft, snapshot, 41U);
  EXPECT_TRUE(pending.position.isApprox(requested.position));
  EXPECT_TRUE(pending.rotation.isApprox(requested.rotation));
  const Pose acknowledged = preview.resolve(ArmSide::kLeft, snapshot, 42U);
  EXPECT_TRUE(acknowledged.position.isApprox(snapshot.position));
  EXPECT_TRUE(acknowledged.rotation.isApprox(snapshot.rotation));
}

TEST(ManualTargetPreviewTest, TracksArmsIndependentlyAndCanCancel) {
  ManualTargetPreview preview;
  Pose left = identityPose();
  Pose right = identityPose();
  left.position.x() = 0.1;
  right.position.y() = -0.2;
  preview.record(ArmSide::kLeft, left, 5U);
  preview.record(ArmSide::kRight, right, 6U);

  preview.cancel(ArmSide::kLeft);

  EXPECT_TRUE(preview.resolve(ArmSide::kLeft, identityPose(), 0U)
                  .position.isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(preview.resolve(ArmSide::kRight, identityPose(), 0U)
                  .position.isApprox(right.position));
  preview.cancelAll();
  EXPECT_TRUE(preview.resolve(ArmSide::kRight, identityPose(), 0U)
                  .position.isApprox(Eigen::Vector3d::Zero()));
}

}  // namespace
}  // namespace tianji_qp_ik
