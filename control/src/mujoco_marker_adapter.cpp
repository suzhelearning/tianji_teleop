#include "tianji_qp_ik/mujoco_marker_adapter.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace tianji_qp_ik {
namespace {

constexpr double kCameraEpsilon = 1e-12;
constexpr int kRingSegments = 48;
constexpr double kScreenPickPixels = 16.0;
constexpr double kCenterPickPixels = 12.0;

Eigen::Vector3d vectorFromFloat3(const float value[3]) {
  return {static_cast<double>(value[0]), static_cast<double>(value[1]),
          static_cast<double>(value[2])};
}

Eigen::Vector3d vectorFromMjtNum3(const mjtNum value[3]) {
  return {static_cast<double>(value[0]), static_cast<double>(value[1]),
          static_cast<double>(value[2])};
}

struct CameraData {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d forward{Eigen::Vector3d::UnitZ()};
  Eigen::Vector3d right{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d up{Eigen::Vector3d::UnitY()};
  double full_height{1.0};
  bool orthographic{false};
};

CameraData cameraData(const mjvScene& scene) {
  const mjvGLCamera average = mjv_averageCamera(&scene.camera[0], &scene.camera[1]);
  CameraData result;
  result.position = vectorFromFloat3(average.pos);
  result.forward = vectorFromFloat3(average.forward).normalized();
  const Eigen::Vector3d raw_up = vectorFromFloat3(average.up).normalized();
  result.right = result.forward.cross(raw_up).normalized();
  result.up = result.right.cross(result.forward).normalized();
  result.orthographic = average.orthographic != 0;
  if (result.orthographic) {
    result.full_height = static_cast<double>(average.frustum_top - average.frustum_bottom);
  } else {
    result.full_height = static_cast<double>(mjv_frustumHeight(&scene));
  }
  if (!result.position.allFinite() || !result.forward.allFinite() ||
      !result.right.allFinite() || !result.up.allFinite() ||
      !std::isfinite(result.full_height) || result.full_height <= kCameraEpsilon) {
    return CameraData{};
  }
  return result;
}

std::array<mjtNum, 3> pointArray(const Eigen::Vector3d& point) {
  return {static_cast<mjtNum>(point.x()), static_cast<mjtNum>(point.y()),
          static_cast<mjtNum>(point.z())};
}

std::array<float, 4> axisColor(std::size_t axis, bool highlighted) {
  constexpr std::array<std::array<float, 4>, 3> kColors{{
      {{0.90F, 0.15F, 0.15F, 0.82F}},
      {{0.15F, 0.90F, 0.20F, 0.82F}},
      {{0.20F, 0.40F, 1.00F, 0.82F}},
  }};
  std::array<float, 4> color = kColors[axis];
  if (highlighted) {
    color[0] = std::min(1.0F, color[0] + 0.10F);
    color[1] = std::min(1.0F, color[1] + 0.10F);
    color[2] = std::min(1.0F, color[2] + 0.10F);
    color[3] = 1.0F;
  }
  return color;
}

MarkerHandle translationHandle(std::size_t axis) {
  constexpr std::array<MarkerHandle, 3> kHandles{
      MarkerHandle::kTranslateX, MarkerHandle::kTranslateY, MarkerHandle::kTranslateZ};
  return kHandles[axis];
}

MarkerHandle rotationHandle(std::size_t axis) {
  constexpr std::array<MarkerHandle, 3> kHandles{
      MarkerHandle::kRotateX, MarkerHandle::kRotateY, MarkerHandle::kRotateZ};
  return kHandles[axis];
}

bool highlighted(MarkerHandle handle, MarkerHandle hovered, MarkerHandle active) {
  return handle == hovered || handle == active;
}

double distanceToSegment(const Eigen::Vector2d& point,
                         const Eigen::Vector2d& begin,
                         const Eigen::Vector2d& end) {
  const Eigen::Vector2d segment = end - begin;
  const double squared_length = segment.squaredNorm();
  const double parameter = squared_length > kCameraEpsilon
                               ? std::clamp((point - begin).dot(segment) / squared_length,
                                             0.0, 1.0)
                               : 0.0;
  return (point - (begin + parameter * segment)).norm();
}

bool betterCandidate(double distance, double depth, MarkerHandle handle,
                     const MarkerScreenPick& best) {
  constexpr double kTieEpsilon = 1e-9;
  if (best.handle == MarkerHandle::kNone ||
      distance < best.distance_pixels - kTieEpsilon) {
    return true;
  }
  if (std::abs(distance - best.distance_pixels) <= kTieEpsilon &&
      depth < best.depth - kTieEpsilon) {
    return true;
  }
  return std::abs(distance - best.distance_pixels) <= kTieEpsilon &&
         std::abs(depth - best.depth) <= kTieEpsilon &&
         static_cast<int>(handle) < static_cast<int>(best.handle);
}

mjvGeom* appendGeom(mjvScene* scene) {
  if (scene == nullptr || scene->geoms == nullptr || scene->ngeom >= scene->maxgeom) {
    return nullptr;
  }
  return &scene->geoms[scene->ngeom++];
}

void finishDecorativeGeom(mjvGeom* geom, bool is_highlighted) {
  geom->objtype = mjOBJ_UNKNOWN;
  geom->objid = -1;
  geom->category = mjCAT_DECOR;
  geom->emission = is_highlighted ? 0.45F : 0.10F;
  geom->specular = 0.25F;
  geom->shininess = 0.35F;
}

void appendCenter(const MarkerGeometry& geometry, MarkerHandle hovered,
                  MarkerHandle active, mjvScene* scene) {
  mjvGeom* geom = appendGeom(scene);
  if (geom == nullptr) {
    return;
  }
  const bool is_highlighted = highlighted(MarkerHandle::kCenter, hovered, active);
  const mjtNum radius = static_cast<mjtNum>(geometry.style.scale *
                                           geometry.style.center_radius_ratio);
  const std::array<mjtNum, 3> size{radius, 0.0, 0.0};
  const std::array<mjtNum, 3> position = pointArray(geometry.origin);
  const std::array<float, 4> color = is_highlighted
                                          ? std::array<float, 4>{1.0F, 0.85F, 0.15F, 1.0F}
                                          : std::array<float, 4>{0.95F, 0.95F, 0.95F, 0.88F};
  mjv_initGeom(geom, mjGEOM_SPHERE, size.data(), position.data(), nullptr, color.data());
  finishDecorativeGeom(geom, is_highlighted);
}

void appendArrows(const MarkerGeometry& geometry, MarkerHandle hovered,
                  MarkerHandle active, mjvScene* scene) {
  for (std::size_t axis = 0U; axis < geometry.axes.size(); ++axis) {
    mjvGeom* geom = appendGeom(scene);
    if (geom == nullptr) {
      return;
    }
    const MarkerHandle handle = translationHandle(axis);
    const bool is_highlighted = highlighted(handle, hovered, active);
    const std::array<float, 4> color = axisColor(axis, is_highlighted);
    mjv_initGeom(geom, mjGEOM_ARROW, nullptr, nullptr, nullptr, color.data());
    const Eigen::Vector3d begin =
        geometry.origin + geometry.style.scale * geometry.style.arrow_start_ratio *
                              geometry.axes[axis];
    const Eigen::Vector3d end =
        geometry.origin + geometry.style.scale * geometry.axes[axis];
    const std::array<mjtNum, 3> begin_array = pointArray(begin);
    const std::array<mjtNum, 3> end_array = pointArray(end);
    const mjtNum width = static_cast<mjtNum>(
        geometry.style.scale * (is_highlighted ? 0.060 : 0.045));
    mjv_connector(geom, mjGEOM_ARROW, width, begin_array.data(), end_array.data());
    finishDecorativeGeom(geom, is_highlighted);
  }
}

void appendRings(const MarkerGeometry& geometry, MarkerHandle hovered,
                 MarkerHandle active, mjvScene* scene) {
  constexpr double kTwoPi = 6.283185307179586476925286766559;
  const double radius = geometry.style.scale * geometry.style.ring_radius_ratio;
  for (std::size_t axis = 0U; axis < geometry.axes.size(); ++axis) {
    const Eigen::Vector3d normal = geometry.axes[axis].normalized();
    const Eigen::Vector3d basis_u = normal.unitOrthogonal();
    const Eigen::Vector3d basis_v = normal.cross(basis_u).normalized();
    const MarkerHandle handle = rotationHandle(axis);
    const bool is_highlighted = highlighted(handle, hovered, active);
    const std::array<float, 4> color = axisColor(axis, is_highlighted);
    for (int segment = 0; segment < kRingSegments; ++segment) {
      mjvGeom* geom = appendGeom(scene);
      if (geom == nullptr) {
        return;
      }
      const double angle_begin = kTwoPi * static_cast<double>(segment) /
                                 static_cast<double>(kRingSegments);
      const double angle_end = kTwoPi * static_cast<double>(segment + 1) /
                               static_cast<double>(kRingSegments);
      const Eigen::Vector3d begin =
          geometry.origin + radius * (std::cos(angle_begin) * basis_u +
                                      std::sin(angle_begin) * basis_v);
      const Eigen::Vector3d end =
          geometry.origin + radius * (std::cos(angle_end) * basis_u +
                                      std::sin(angle_end) * basis_v);
      const std::array<mjtNum, 3> begin_array = pointArray(begin);
      const std::array<mjtNum, 3> end_array = pointArray(end);
      mjv_initGeom(geom, mjGEOM_LINE, nullptr, nullptr, nullptr, color.data());
      const mjtNum width = static_cast<mjtNum>(is_highlighted ? 6.0 : 3.0);
      mjv_connector(geom, mjGEOM_LINE, width, begin_array.data(), end_array.data());
      finishDecorativeGeom(geom, is_highlighted);
    }
  }
}

}  // namespace

MarkerScreenPoint projectMarkerPoint(const mjvScene& scene,
                                     const Eigen::Vector3d& point,
                                     int viewport_width, int viewport_height) {
  MarkerScreenPoint result;
  if (viewport_width <= 0 || viewport_height <= 0 || !point.allFinite()) {
    return result;
  }
  if (!std::isfinite(static_cast<double>(scene.scale)) ||
      static_cast<double>(scene.scale) <= kCameraEpsilon) {
    return result;
  }

  mjtNum head_array[3]{};
  mjtNum forward_array[3]{};
  mjtNum up_array[3]{};
  mjv_cameraInModel(head_array, forward_array, up_array, &scene);
  const Eigen::Vector3d head = vectorFromMjtNum3(head_array);
  const Eigen::Vector3d forward = vectorFromMjtNum3(forward_array).normalized();
  const Eigen::Vector3d up = vectorFromMjtNum3(up_array).normalized();
  const Eigen::Vector3d right = forward.cross(up).normalized();
  const Eigen::Vector3d relative = point - head;
  const double depth = relative.dot(forward);
  const double frustum_height = static_cast<double>(mjv_frustumHeight(&scene));
  const mjvGLCamera average = mjv_averageCamera(&scene.camera[0], &scene.camera[1]);
  if (!head.allFinite() || !forward.allFinite() || !up.allFinite() ||
      !right.allFinite() || !std::isfinite(depth) || depth <= kCameraEpsilon ||
      !std::isfinite(frustum_height) || frustum_height <= kCameraEpsilon) {
    return result;
  }

  const double scale = average.orthographic != 0
                           ? static_cast<double>(viewport_height) / frustum_height
                           : static_cast<double>(viewport_height) /
                                 (frustum_height * depth);
  result.pixel = Eigen::Vector2d(
      0.5 * static_cast<double>(viewport_width) + relative.dot(right) * scale,
      0.5 * static_cast<double>(viewport_height) + relative.dot(up) * scale);
  result.depth = depth;
  result.visible = result.pixel.allFinite();
  return result;
}

MarkerScreenPick pickMarkerHandle(const MarkerGeometry& geometry,
                                  const mjvScene& scene,
                                  int viewport_width, int viewport_height,
                                  const Eigen::Vector2d& cursor_pixel) {
  MarkerScreenPick result;
  if (viewport_width <= 0 || viewport_height <= 0 || !cursor_pixel.allFinite() ||
      !geometry.origin.allFinite() || !std::isfinite(geometry.style.scale) ||
      geometry.style.scale <= 0.0) {
    return result;
  }
  for (const Eigen::Vector3d& axis : geometry.axes) {
    if (!axis.allFinite() || axis.squaredNorm() <= kCameraEpsilon) {
      return result;
    }
  }

  const auto consider = [&result](MarkerHandle handle, double distance, double depth,
                                  double tolerance) {
    if (!std::isfinite(distance) || !std::isfinite(depth) || distance > tolerance) {
      return;
    }
    if (betterCandidate(distance, depth, handle, result)) {
      result.handle = handle;
      result.distance_pixels = distance;
      result.depth = depth;
    }
  };

  const MarkerScreenPoint center =
      projectMarkerPoint(scene, geometry.origin, viewport_width, viewport_height);
  if (center.visible) {
    const double center_distance = (cursor_pixel - center.pixel).norm();
    if (center_distance <= kCenterPickPixels) {
      result.handle = MarkerHandle::kCenter;
      result.distance_pixels = center_distance;
      result.depth = center.depth;
      return result;
    }
  }

  for (std::size_t axis_index = 0U; axis_index < geometry.axes.size(); ++axis_index) {
    const Eigen::Vector3d axis = geometry.axes[axis_index].normalized();
    const Eigen::Vector3d begin =
        geometry.origin + geometry.style.scale * geometry.style.arrow_start_ratio * axis;
    const Eigen::Vector3d end = geometry.origin + geometry.style.scale * axis;
    const MarkerScreenPoint begin_screen =
        projectMarkerPoint(scene, begin, viewport_width, viewport_height);
    const MarkerScreenPoint end_screen =
        projectMarkerPoint(scene, end, viewport_width, viewport_height);
    if (begin_screen.visible && end_screen.visible) {
      consider(translationHandle(axis_index),
               distanceToSegment(cursor_pixel, begin_screen.pixel, end_screen.pixel),
               std::min(begin_screen.depth, end_screen.depth), kScreenPickPixels);
    }
  }

  constexpr double kTwoPi = 6.283185307179586476925286766559;
  const double ring_radius = geometry.style.scale * geometry.style.ring_radius_ratio;
  if (!std::isfinite(ring_radius) || ring_radius <= 0.0) {
    return result;
  }
  for (std::size_t axis_index = 0U; axis_index < geometry.axes.size(); ++axis_index) {
    const Eigen::Vector3d normal = geometry.axes[axis_index].normalized();
    const Eigen::Vector3d basis_u = normal.unitOrthogonal();
    const Eigen::Vector3d basis_v = normal.cross(basis_u).normalized();
    if (!basis_u.allFinite() || !basis_v.allFinite()) {
      continue;
    }
    for (int segment = 0; segment < kRingSegments; ++segment) {
      const double angle_begin = kTwoPi * static_cast<double>(segment) /
                                 static_cast<double>(kRingSegments);
      const double angle_end = kTwoPi * static_cast<double>(segment + 1) /
                               static_cast<double>(kRingSegments);
      const Eigen::Vector3d begin =
          geometry.origin + ring_radius * (std::cos(angle_begin) * basis_u +
                                           std::sin(angle_begin) * basis_v);
      const Eigen::Vector3d end =
          geometry.origin + ring_radius * (std::cos(angle_end) * basis_u +
                                           std::sin(angle_end) * basis_v);
      const MarkerScreenPoint begin_screen =
          projectMarkerPoint(scene, begin, viewport_width, viewport_height);
      const MarkerScreenPoint end_screen =
          projectMarkerPoint(scene, end, viewport_width, viewport_height);
      if (begin_screen.visible && end_screen.visible) {
        consider(rotationHandle(axis_index),
                 distanceToSegment(cursor_pixel, begin_screen.pixel, end_screen.pixel),
                 std::min(begin_screen.depth, end_screen.depth), kScreenPickPixels);
      }
    }
  }
  return result;
}

MarkerCamera markerCameraFromScene(const mjvScene& scene,
                                   const Eigen::Vector3d& marker_origin) {
  const CameraData camera_data = cameraData(scene);
  MarkerCamera result;
  result.forward = camera_data.forward;
  result.right = camera_data.right;
  result.up = camera_data.up;
  if (camera_data.orthographic) {
    result.world_per_ndc = 0.5 * camera_data.full_height;
  } else {
    const double depth = std::max(kCameraEpsilon,
                                  camera_data.forward.dot(marker_origin - camera_data.position));
    result.world_per_ndc = 0.5 * camera_data.full_height * depth;
  }
  return result;
}

MarkerPointer markerPointerFromScene(const mjvScene& scene, double aspect_ratio,
                                     double relx, double rely,
                                     const Eigen::Vector3d& marker_origin) {
  const CameraData camera_data = cameraData(scene);
  const double safe_aspect =
      std::isfinite(aspect_ratio) && aspect_ratio > kCameraEpsilon ? aspect_ratio : 1.0;
  const double clamped_x = std::clamp(relx, 0.0, 1.0);
  const double clamped_y = std::clamp(rely, 0.0, 1.0);
  const Eigen::Vector2d ndc(2.0 * clamped_x - 1.0, 2.0 * clamped_y - 1.0);
  const double half_height = 0.5 * camera_data.full_height;
  const Eigen::Vector3d offset =
      camera_data.right * ndc.x() * half_height * safe_aspect +
      camera_data.up * ndc.y() * half_height;

  MarkerPointer result;
  result.ndc = ndc;
  if (camera_data.orthographic) {
    result.ray.origin = camera_data.position + offset;
    result.ray.direction = camera_data.forward;
  } else {
    result.ray.origin = camera_data.position;
    result.ray.direction = (camera_data.forward + offset).normalized();
  }
  (void)marker_origin;
  return result;
}

void appendInteractiveMarker(const MarkerGeometry& geometry,
                             MarkerHandle hovered, MarkerHandle active,
                             mjvScene* scene) {
  if (scene == nullptr || !geometry.origin.allFinite() ||
      !std::isfinite(geometry.style.scale) || geometry.style.scale <= 0.0) {
    return;
  }
  appendCenter(geometry, hovered, active, scene);
  appendArrows(geometry, hovered, active, scene);
  appendRings(geometry, hovered, active, scene);
}

}  // namespace tianji_qp_ik
