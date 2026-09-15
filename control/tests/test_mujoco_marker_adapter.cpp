#include "tianji_qp_ik/mujoco_marker_adapter.hpp"
#include "tianji_qp_ik/mujoco_robot.hpp"

#include <gtest/gtest.h>

#include <mujoco/mujoco.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace tianji_qp_ik {
namespace {

mjvScene testCameraScene(bool orthographic = false) {
  mjvScene scene;
  mjv_defaultScene(&scene);
  scene.scale = 1.0F;
  for (mjvGLCamera& camera : scene.camera) {
    camera.pos[0] = 0.0F;
    camera.pos[1] = 0.0F;
    camera.pos[2] = -2.0F;
    camera.forward[0] = 0.0F;
    camera.forward[1] = 0.0F;
    camera.forward[2] = 1.0F;
    camera.up[0] = 0.0F;
    camera.up[1] = 1.0F;
    camera.up[2] = 0.0F;
    camera.frustum_center = 0.0F;
    camera.frustum_width = 1.0F;
    camera.frustum_bottom = -0.5F;
    camera.frustum_top = 0.5F;
    camera.frustum_near = 1.0F;
    camera.frustum_far = 100.0F;
    camera.orthographic = orthographic ? 1 : 0;
  }
  return scene;
}

constexpr int kPickerWidth = 1000;
constexpr int kPickerHeight = 1000;

Eigen::Vector2d projectedPixel(const mjvScene& scene, const Eigen::Vector3d& point) {
  const MarkerScreenPoint projected =
      projectMarkerPoint(scene, point, kPickerWidth, kPickerHeight);
  EXPECT_TRUE(projected.visible);
  return projected.pixel;
}

TEST(MujocoMarkerAdapterTest, CenterCursorProducesForwardRay) {
  const mjvScene scene = testCameraScene();

  const MarkerPointer pointer = markerPointerFromScene(
      scene, 1.0, 0.5, 0.5, Eigen::Vector3d::Zero());

  EXPECT_TRUE(pointer.ray.origin.isApprox(Eigen::Vector3d(0.0, 0.0, -2.0), 1e-12));
  EXPECT_TRUE(pointer.ray.direction.isApprox(Eigen::Vector3d::UnitZ(), 1e-12));
  EXPECT_TRUE(pointer.ndc.isApprox(Eigen::Vector2d::Zero(), 1e-12));
}

TEST(MujocoMarkerAdapterTest, PerspectiveCornerRayUsesFrustumAndAspect) {
  const mjvScene scene = testCameraScene();

  const MarkerPointer pointer = markerPointerFromScene(
      scene, 2.0, 1.0, 1.0, Eigen::Vector3d::Zero());

  const Eigen::Vector3d expected = Eigen::Vector3d(-1.0, 0.5, 1.0).normalized();
  EXPECT_TRUE(pointer.ray.direction.isApprox(expected, 1e-12));
}

TEST(MujocoMarkerAdapterTest, OrthographicCursorOffsetsOrigin) {
  const mjvScene scene = testCameraScene(true);

  const MarkerPointer pointer = markerPointerFromScene(
      scene, 2.0, 1.0, 1.0, Eigen::Vector3d::Zero());

  EXPECT_TRUE(pointer.ray.origin.isApprox(Eigen::Vector3d(-1.0, 0.5, -2.0), 1e-12));
  EXPECT_TRUE(pointer.ray.direction.isApprox(Eigen::Vector3d::UnitZ(), 1e-12));
}

TEST(MujocoMarkerAdapterTest, CameraBasisAndScaleAreFinite) {
  const mjvScene scene = testCameraScene();

  const MarkerCamera camera = markerCameraFromScene(scene, Eigen::Vector3d::Zero());

  EXPECT_TRUE(camera.forward.isApprox(Eigen::Vector3d::UnitZ(), 1e-12));
  EXPECT_TRUE(camera.right.isApprox(-Eigen::Vector3d::UnitX(), 1e-12));
  EXPECT_TRUE(camera.up.isApprox(Eigen::Vector3d::UnitY(), 1e-12));
  EXPECT_NEAR(camera.world_per_ndc, 1.0, 1e-12);
}

TEST(MujocoMarkerAdapterTest, ProjectsCenterAndRejectsInvalidViewport) {
  const mjvScene scene = testCameraScene();

  const MarkerScreenPoint center =
      projectMarkerPoint(scene, Eigen::Vector3d::Zero(), kPickerWidth, kPickerHeight);
  ASSERT_TRUE(center.visible);
  EXPECT_TRUE(center.pixel.isApprox(Eigen::Vector2d(500.0, 500.0), 1e-9));
  EXPECT_GT(center.depth, 0.0);

  EXPECT_FALSE(projectMarkerPoint(scene, Eigen::Vector3d::Zero(), 0, kPickerHeight).visible);
  EXPECT_EQ(pickMarkerHandle(MarkerGeometry{}, scene, 0, kPickerHeight,
                             Eigen::Vector2d(500.0, 500.0)).handle,
            MarkerHandle::kNone);
}

TEST(MujocoMarkerAdapterTest, RejectsInvalidSceneScaleWithoutProjection) {
  mjvScene scene = testCameraScene();
  scene.scale = 0.0F;

  const MarkerScreenPoint projected =
      projectMarkerPoint(scene, Eigen::Vector3d::Zero(), kPickerWidth, kPickerHeight);

  EXPECT_FALSE(projected.visible);
}

TEST(MujocoMarkerAdapterTest, OrthographicProjectionIgnoresDepth) {
  const mjvScene scene = testCameraScene(true);
  const MarkerScreenPoint near_point = projectMarkerPoint(
      scene, Eigen::Vector3d(0.1, 0.0, 0.0), kPickerWidth, kPickerHeight);
  const MarkerScreenPoint far_point = projectMarkerPoint(
      scene, Eigen::Vector3d(0.1, 0.0, 1.0), kPickerWidth, kPickerHeight);

  ASSERT_TRUE(near_point.visible);
  ASSERT_TRUE(far_point.visible);
  EXPECT_NEAR(near_point.pixel.x(), far_point.pixel.x(), 1e-9);
  EXPECT_NEAR(near_point.pixel.y(), far_point.pixel.y(), 1e-9);
}

TEST(MujocoMarkerAdapterTest, ScreenPickerSelectsNearestProjectedTranslationArrow) {
  const mjvScene scene = testCameraScene();
  InteractiveMarker6D marker;
  const MarkerGeometry geometry = marker.geometry(Pose{});
  const Eigen::Vector2d origin = projectedPixel(scene, geometry.origin);
  const Eigen::Vector2d x_end = projectedPixel(
      scene, geometry.origin + geometry.style.scale * geometry.axes[0]);
  const Eigen::Vector2d cursor = origin + 0.90 * (x_end - origin);

  const MarkerScreenPick pick =
      pickMarkerHandle(geometry, scene, kPickerWidth, kPickerHeight, cursor);

  EXPECT_EQ(pick.handle, MarkerHandle::kTranslateX);
  EXPECT_LT(pick.distance_pixels, 16.0);
}

TEST(MujocoMarkerAdapterTest, ScreenPickerUsesLocalMarkerAxes) {
  const mjvScene scene = testCameraScene();
  InteractiveMarker6D marker;
  marker.setFrame(MarkerFrame::kLocal);
  Pose pose;
  pose.rotation = Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const MarkerGeometry geometry = marker.geometry(pose);
  const Eigen::Vector3d endpoint =
      geometry.origin + geometry.style.scale * geometry.axes[0];

  const MarkerScreenPick pick = pickMarkerHandle(
      geometry, scene, kPickerWidth, kPickerHeight, projectedPixel(scene, endpoint));

  EXPECT_EQ(pick.handle, MarkerHandle::kTranslateX);
}

TEST(MujocoMarkerAdapterTest, ScreenPickerSelectsProjectedRotationRing) {
  const mjvScene scene = testCameraScene();
  InteractiveMarker6D marker;
  const MarkerGeometry geometry = marker.geometry(Pose{});
  const Eigen::Vector3d normal = geometry.axes[1].normalized();
  const Eigen::Vector3d basis = normal.unitOrthogonal();
  const Eigen::Vector3d ring_point =
      geometry.origin + geometry.style.scale * geometry.style.ring_radius_ratio * basis;

  const MarkerScreenPick pick = pickMarkerHandle(
      geometry, scene, kPickerWidth, kPickerHeight, projectedPixel(scene, ring_point));

  EXPECT_EQ(pick.handle, MarkerHandle::kRotateY);
  EXPECT_LT(pick.distance_pixels, 1e-9);
}

TEST(MujocoMarkerAdapterTest, ScreenPickerSelectsCenterAndRejectsBlankCursor) {
  const mjvScene scene = testCameraScene();
  InteractiveMarker6D marker;
  const MarkerGeometry geometry = marker.geometry(Pose{});

  const MarkerScreenPick center = pickMarkerHandle(
      geometry, scene, kPickerWidth, kPickerHeight,
      projectedPixel(scene, geometry.origin));
  EXPECT_EQ(center.handle, MarkerHandle::kCenter);

  const MarkerScreenPick blank = pickMarkerHandle(
      geometry, scene, kPickerWidth, kPickerHeight, Eigen::Vector2d(20.0, 20.0));
  EXPECT_EQ(blank.handle, MarkerHandle::kNone);
  EXPECT_TRUE(std::isinf(blank.distance_pixels));
}

TEST(MujocoMarkerAdapterTest, AppendsThreeArrowsThreeRingsAndCenter) {
  std::vector<mjvGeom> storage(200);
  mjvScene scene = testCameraScene();
  scene.maxgeom = static_cast<int>(storage.size());
  scene.ngeom = 0;
  scene.geoms = storage.data();
  InteractiveMarker6D marker;

  appendInteractiveMarker(marker.geometry(Pose{}), MarkerHandle::kTranslateX,
                          MarkerHandle::kNone, &scene);

  EXPECT_EQ(scene.ngeom, 148);
  const int arrows = static_cast<int>(std::count_if(
      storage.begin(), storage.begin() + scene.ngeom,
      [](const mjvGeom& geom) { return geom.type == mjGEOM_ARROW; }));
  const int lines = static_cast<int>(std::count_if(
      storage.begin(), storage.begin() + scene.ngeom,
      [](const mjvGeom& geom) { return geom.type == mjGEOM_LINE; }));
  const int spheres = static_cast<int>(std::count_if(
      storage.begin(), storage.begin() + scene.ngeom,
      [](const mjvGeom& geom) { return geom.type == mjGEOM_SPHERE; }));
  EXPECT_EQ(arrows, 3);
  EXPECT_EQ(lines, 144);
  EXPECT_EQ(spheres, 1);
  const auto highlighted = std::find_if(
      storage.begin(), storage.begin() + scene.ngeom,
      [](const mjvGeom& geom) { return geom.type == mjGEOM_ARROW && geom.rgba[0] > 0.8F; });
  ASSERT_NE(highlighted, storage.begin() + scene.ngeom);
  EXPECT_GT(highlighted->emission, 0.0F);
}

TEST(MujocoMarkerAdapterTest, NeverExceedsSceneCapacity) {
  std::array<mjvGeom, 1> storage{};
  mjvScene scene = testCameraScene();
  scene.maxgeom = 1;
  scene.ngeom = 0;
  scene.geoms = storage.data();
  InteractiveMarker6D marker;

  appendInteractiveMarker(marker.geometry(Pose{}), MarkerHandle::kNone,
                          MarkerHandle::kNone, &scene);

  EXPECT_EQ(scene.ngeom, 1);
  EXPECT_LE(scene.ngeom, scene.maxgeom);
}

TEST(MujocoMarkerAdapterTest, RayMatchesMujocoSelectionInRealScene) {
  const std::string model_path =
      std::string(TIANJI_PROJECT_SOURCE_DIR) + "/models/marvin_m6_qp_test.xml";
  MujocoRobot robot(model_path);
  robot.forward();
  mjvCamera camera;
  mjvOption options;
  mjvScene scene;
  mjv_defaultFreeCamera(robot.model(), &camera);
  mjv_defaultOption(&options);
  mjv_defaultScene(&scene);
  mjv_makeScene(robot.model(), &scene, 2000);
  constexpr double kAspect = 1.6;
  mjv_updateScene(robot.model(), robot.data(), &options, nullptr, &camera, mjCAT_ALL, &scene);

  int hit_count = 0;
  for (int y_index = 1; y_index < 10; ++y_index) {
    for (int x_index = 1; x_index < 16; ++x_index) {
      const double relx = static_cast<double>(x_index) / 16.0;
      const double rely = static_cast<double>(y_index) / 10.0;
      mjtNum selected_point[3]{};
      int geom_id = -1;
      int flex_id = -1;
      int skin_id = -1;
      const int body_id = mjv_select(robot.model(), robot.data(), &options, kAspect,
                                     relx, rely, &scene, selected_point,
                                     &geom_id, &flex_id, &skin_id);
      if (body_id < 0) {
        continue;
      }
      ++hit_count;
      const MarkerPointer pointer = markerPointerFromScene(
          scene, kAspect, relx, rely, Eigen::Vector3d::Zero());
      const Eigen::Vector3d point(selected_point[0], selected_point[1], selected_point[2]);
      const double parameter = pointer.ray.direction.dot(point - pointer.ray.origin);
      const Eigen::Vector3d closest = pointer.ray.origin + parameter * pointer.ray.direction;
      EXPECT_LT((closest - point).norm(), 1e-6)
          << "screen coordinate " << relx << ", " << rely
          << " point=" << point.transpose()
          << " origin=" << pointer.ray.origin.transpose()
          << " direction=" << pointer.ray.direction.transpose()
          << " gl_forward=" << scene.camera[0].forward[0] << ','
          << scene.camera[0].forward[1] << ',' << scene.camera[0].forward[2]
          << " gl_up=" << scene.camera[0].up[0] << ',' << scene.camera[0].up[1] << ','
          << scene.camera[0].up[2];
      const MarkerScreenPoint projected =
          projectMarkerPoint(scene, point, 1600, 1000);
      ASSERT_TRUE(projected.visible);
      EXPECT_NEAR(projected.pixel.x(), relx * 1600.0, 1e-4);
      EXPECT_NEAR(projected.pixel.y(), rely * 1000.0, 1e-4);
    }
  }
  EXPECT_GT(hit_count, 0);
  mjv_freeScene(&scene);
}

}  // namespace
}  // namespace tianji_qp_ik
