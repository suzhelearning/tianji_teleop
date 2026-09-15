#include "tianji_qp_ik/interactive_marker.hpp"

#include "tianji_qp_ik/so3.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace tianji_qp_ik {
namespace {

constexpr double kGeometryEpsilon = 1e-12;

bool finiteRay(const Ray3d& ray) {
  return ray.origin.allFinite() && ray.direction.allFinite() &&
         ray.direction.squaredNorm() > kGeometryEpsilon;
}

bool finitePose(const Pose& pose) {
  return pose.position.allFinite() && isProperRotation(pose.rotation);
}

bool finiteCamera(const MarkerCamera& camera) {
  return camera.forward.allFinite() && camera.right.allFinite() && camera.up.allFinite() &&
         camera.forward.squaredNorm() > kGeometryEpsilon &&
         camera.right.squaredNorm() > kGeometryEpsilon &&
         camera.up.squaredNorm() > kGeometryEpsilon &&
         std::isfinite(camera.world_per_ndc) && camera.world_per_ndc > 0.0;
}

std::optional<Eigen::Vector3d> normalizedDirection(const Ray3d& ray) {
  if (!finiteRay(ray)) {
    return std::nullopt;
  }
  return ray.direction.normalized();
}

double rayPointDistance(const Ray3d& ray, const Eigen::Vector3d& point) {
  const Eigen::Vector3d direction = ray.direction.normalized();
  const double parameter = std::max(0.0, direction.dot(point - ray.origin));
  return (ray.origin + parameter * direction - point).norm();
}

double raySegmentDistance(const Ray3d& ray, const Eigen::Vector3d& begin,
                          const Eigen::Vector3d& end) {
  const Eigen::Vector3d direction = ray.direction.normalized();
  const Eigen::Vector3d segment = end - begin;
  const double segment_squared = segment.squaredNorm();
  if (segment_squared <= kGeometryEpsilon) {
    return rayPointDistance(ray, begin);
  }

  const Eigen::Vector3d offset = ray.origin - begin;
  const double coupling = direction.dot(segment);
  const double direction_offset = direction.dot(offset);
  const double segment_offset = segment.dot(offset);
  const double denominator = segment_squared - coupling * coupling;
  double segment_parameter = 0.0;
  if (denominator > kGeometryEpsilon) {
    segment_parameter =
        std::clamp((segment_offset - coupling * direction_offset) / denominator, 0.0, 1.0);
  } else {
    segment_parameter = std::clamp(-segment_offset / segment_squared, 0.0, 1.0);
  }
  const Eigen::Vector3d segment_point = begin + segment_parameter * segment;
  const double ray_parameter = std::max(0.0, direction.dot(segment_point - ray.origin));
  return (ray.origin + ray_parameter * direction - segment_point).norm();
}

std::optional<double> closestAxisParameter(const Ray3d& ray,
                                           const Eigen::Vector3d& origin,
                                           const Eigen::Vector3d& axis) {
  const std::optional<Eigen::Vector3d> direction = normalizedDirection(ray);
  if (!direction.has_value() || !origin.allFinite() || !axis.allFinite() ||
      axis.squaredNorm() <= kGeometryEpsilon) {
    return std::nullopt;
  }
  const Eigen::Vector3d unit_axis = axis.normalized();
  const double coupling = direction->dot(unit_axis);
  const double denominator = 1.0 - coupling * coupling;
  if (denominator <= kGeometryEpsilon) {
    return std::nullopt;
  }
  const Eigen::Vector3d offset = ray.origin - origin;
  const double parameter =
      (unit_axis.dot(offset) - coupling * direction->dot(offset)) / denominator;
  if (!std::isfinite(parameter)) {
    return std::nullopt;
  }
  return parameter;
}

std::optional<Eigen::Vector3d> rayPlaneIntersection(const Ray3d& ray,
                                                    const Eigen::Vector3d& plane_point,
                                                    const Eigen::Vector3d& plane_normal) {
  const std::optional<Eigen::Vector3d> direction = normalizedDirection(ray);
  if (!direction.has_value() || !plane_point.allFinite() || !plane_normal.allFinite() ||
      plane_normal.squaredNorm() <= kGeometryEpsilon) {
    return std::nullopt;
  }
  const Eigen::Vector3d normal = plane_normal.normalized();
  const double denominator = normal.dot(*direction);
  if (std::abs(denominator) <= kGeometryEpsilon) {
    return std::nullopt;
  }
  const double distance = normal.dot(plane_point - ray.origin) / denominator;
  if (!std::isfinite(distance) || distance < 0.0) {
    return std::nullopt;
  }
  const Eigen::Vector3d point = ray.origin + distance * *direction;
  return point.allFinite() ? std::optional<Eigen::Vector3d>(point) : std::nullopt;
}

std::optional<std::size_t> translationAxisIndex(MarkerHandle handle) {
  switch (handle) {
    case MarkerHandle::kTranslateX:
      return 0U;
    case MarkerHandle::kTranslateY:
      return 1U;
    case MarkerHandle::kTranslateZ:
      return 2U;
    default:
      return std::nullopt;
  }
}

std::optional<std::size_t> rotationAxisIndex(MarkerHandle handle) {
  switch (handle) {
    case MarkerHandle::kRotateX:
      return 0U;
    case MarkerHandle::kRotateY:
      return 1U;
    case MarkerHandle::kRotateZ:
      return 2U;
    default:
      return std::nullopt;
  }
}

MarkerHandle translationHandle(std::size_t index) {
  constexpr std::array<MarkerHandle, 3> kHandles{
      MarkerHandle::kTranslateX, MarkerHandle::kTranslateY, MarkerHandle::kTranslateZ};
  return kHandles[index];
}

MarkerHandle rotationHandle(std::size_t index) {
  constexpr std::array<MarkerHandle, 3> kHandles{
      MarkerHandle::kRotateX, MarkerHandle::kRotateY, MarkerHandle::kRotateZ};
  return kHandles[index];
}

std::size_t armIndex(ArmSide side) {
  return side == ArmSide::kLeft ? 0U : 1U;
}

}  // namespace

InteractiveMarker6D::InteractiveMarker6D(MarkerStyle style) : style_(style) {}

MarkerGeometry InteractiveMarker6D::geometry(const Pose& pose) const {
  MarkerGeometry result;
  result.origin = pose.position;
  result.style = style_;
  if (frame_ == MarkerFrame::kLocal) {
    for (std::size_t axis = 0U; axis < result.axes.size(); ++axis) {
      result.axes[axis] = pose.rotation.col(static_cast<Eigen::Index>(axis));
    }
  }
  return result;
}

MarkerHandle InteractiveMarker6D::pick(const Pose& pose, const Ray3d& ray) const {
  if (!finitePose(pose) || !finiteRay(ray) || !std::isfinite(style_.scale) ||
      style_.scale <= 0.0) {
    return MarkerHandle::kNone;
  }
  const MarkerGeometry marker_geometry = geometry(pose);
  MarkerHandle selected = MarkerHandle::kNone;
  double selected_score = std::numeric_limits<double>::infinity();

  const double center_radius = style_.scale * style_.center_radius_ratio;
  const double center_score = rayPointDistance(ray, marker_geometry.origin) / center_radius;
  if (center_score <= 1.0) {
    selected = MarkerHandle::kCenter;
    selected_score = center_score;
  }

  const double arrow_radius = style_.scale * style_.arrow_pick_radius_ratio;
  for (std::size_t index = 0U; index < marker_geometry.axes.size(); ++index) {
    const Eigen::Vector3d begin = marker_geometry.origin +
                                  style_.scale * style_.arrow_start_ratio *
                                      marker_geometry.axes[index];
    const Eigen::Vector3d end =
        marker_geometry.origin + style_.scale * marker_geometry.axes[index];
    const double score = raySegmentDistance(ray, begin, end) / arrow_radius;
    if (score <= 1.0 && score < selected_score) {
      selected = translationHandle(index);
      selected_score = score;
    }
  }


  const double ring_radius = style_.scale * style_.ring_radius_ratio;
  const double ring_pick_radius = style_.scale * style_.ring_pick_radius_ratio;
  for (std::size_t index = 0U; index < marker_geometry.axes.size(); ++index) {
    const std::optional<Eigen::Vector3d> intersection = rayPlaneIntersection(
        ray, marker_geometry.origin, marker_geometry.axes[index]);
    if (!intersection.has_value()) {
      continue;
    }
    const double radial_distance = (*intersection - marker_geometry.origin).norm();
    const double score = std::abs(radial_distance - ring_radius) / ring_pick_radius;
    if (score <= 1.0 && score < selected_score) {
      selected = rotationHandle(index);
      selected_score = score;
    }
  }
  return selected;
}

bool InteractiveMarker6D::beginDrag(MarkerHandle handle, const Pose& pose,
                                    const MarkerPointer& pointer,
                                    const MarkerCamera& camera) {
  if (!finitePose(pose) || !finiteRay(pointer.ray) || !pointer.ndc.allFinite() ||
      !finiteCamera(camera)) {
    cancelDrag();
    return false;
  }
  const MarkerGeometry marker_geometry = geometry(pose);
  DragState state;
  state.start_pose = pose;
  state.start_pointer = pointer;
  state.camera = camera;

  if (handle == MarkerHandle::kCenter) {
    state.plane_normal = camera.forward.normalized();
    const std::optional<Eigen::Vector3d> intersection =
        rayPlaneIntersection(pointer.ray, pose.position, state.plane_normal);
    if (!intersection.has_value()) {
      cancelDrag();
      return false;
    }
    state.start_plane_point = *intersection;
  } else if (const std::optional<std::size_t> translation_axis_index =
                 translationAxisIndex(handle)) {
    state.axis = marker_geometry.axes[*translation_axis_index].normalized();
    const std::optional<double> parameter =
        closestAxisParameter(pointer.ray, pose.position, state.axis);
    if (parameter.has_value()) {
      state.start_axis_parameter = *parameter;
    } else {
      state.use_screen_fallback = true;
    }
  } else if (const std::optional<std::size_t> rotation_axis_index =
                 rotationAxisIndex(handle)) {
    state.axis = marker_geometry.axes[*rotation_axis_index].normalized();
    state.plane_normal = state.axis;
    const std::optional<Eigen::Vector3d> intersection =
        rayPlaneIntersection(pointer.ray, pose.position, state.plane_normal);
    if (!intersection.has_value()) {
      cancelDrag();
      return false;
    }
    state.start_radial = *intersection - pose.position;
    state.start_radial -= state.axis * state.axis.dot(state.start_radial);
    if (!state.start_radial.allFinite() ||
        state.start_radial.norm() <= style_.scale * style_.ring_pick_radius_ratio) {
      cancelDrag();
      return false;
    }
  } else {
    cancelDrag();
    return false;
  }

  drag_ = state;
  active_handle_ = handle;
  return true;
}

std::optional<Pose> InteractiveMarker6D::updateDrag(const MarkerPointer& pointer) const {
  if (!drag_.has_value() || !finiteRay(pointer.ray) || !pointer.ndc.allFinite()) {
    return std::nullopt;
  }
  Pose result = drag_->start_pose;

  if (active_handle_ == MarkerHandle::kCenter) {
    const std::optional<Eigen::Vector3d> intersection = rayPlaneIntersection(
        pointer.ray, drag_->start_pose.position, drag_->plane_normal);
    if (!intersection.has_value()) {
      return std::nullopt;
    }
    result.position += *intersection - drag_->start_plane_point;
  } else if (translationAxisIndex(active_handle_).has_value()) {
    double translation = 0.0;
    if (!drag_->use_screen_fallback) {
      const std::optional<double> parameter = closestAxisParameter(
          pointer.ray, drag_->start_pose.position, drag_->axis);
      if (parameter.has_value()) {
        translation = *parameter - drag_->start_axis_parameter;
      } else {
        return std::nullopt;
      }
    } else {
      const Eigen::Vector2d screen_axis(drag_->axis.dot(drag_->camera.right),
                                        drag_->axis.dot(drag_->camera.up));
      const Eigen::Vector2d pointer_delta = pointer.ndc - drag_->start_pointer.ndc;
      if (screen_axis.squaredNorm() > kGeometryEpsilon) {
        translation = pointer_delta.dot(screen_axis) / screen_axis.squaredNorm() *
                      drag_->camera.world_per_ndc;
      } else {
        translation = -pointer_delta.y() * drag_->camera.world_per_ndc;
      }
    }
    result.position += translation * drag_->axis;
  } else if (rotationAxisIndex(active_handle_).has_value()) {
    double angle = 0.0;
    const std::optional<Eigen::Vector3d> intersection = rayPlaneIntersection(
        pointer.ray, drag_->start_pose.position, drag_->plane_normal);
    if (intersection.has_value()) {
      Eigen::Vector3d current_radial = *intersection - drag_->start_pose.position;
      current_radial -= drag_->axis * drag_->axis.dot(current_radial);
      if (current_radial.squaredNorm() <= kGeometryEpsilon) {
        return std::nullopt;
      }
      const Eigen::Vector3d start_unit = drag_->start_radial.normalized();
      const Eigen::Vector3d current_unit = current_radial.normalized();
      angle = std::atan2(drag_->axis.dot(start_unit.cross(current_unit)),
                         start_unit.dot(current_unit));
    } else {
      const Eigen::Vector3d tangent =
          drag_->axis.cross(drag_->start_radial.normalized());
      const Eigen::Vector2d screen_tangent(tangent.dot(drag_->camera.right),
                                           tangent.dot(drag_->camera.up));
      if (screen_tangent.squaredNorm() <= kGeometryEpsilon) {
        return std::nullopt;
      }
      const Eigen::Vector2d pointer_delta = pointer.ndc - drag_->start_pointer.ndc;
      const double tangent_distance =
          pointer_delta.dot(screen_tangent) / screen_tangent.squaredNorm() *
          drag_->camera.world_per_ndc;
      angle = tangent_distance / drag_->start_radial.norm();
    }
    if (!std::isfinite(angle)) {
      return std::nullopt;
    }
    result.rotation =
        Eigen::AngleAxisd(angle, drag_->axis).toRotationMatrix() * drag_->start_pose.rotation;
    const Eigen::Quaterniond normalized(result.rotation);
    if (!std::isfinite(normalized.norm()) || normalized.norm() <= kGeometryEpsilon) {
      return std::nullopt;
    }
    result.rotation = normalized.normalized().toRotationMatrix();
  } else {
    return std::nullopt;
  }

  if (!finitePose(result)) {
    return std::nullopt;
  }
  return result;
}

void InteractiveMarker6D::endDrag() noexcept {
  drag_.reset();
  active_handle_ = MarkerHandle::kNone;
}

void InteractiveMarker6D::cancelDrag() noexcept {
  endDrag();
}

void ManualTargetPreview::record(ArmSide side, const Pose& pose,
                                 std::uint64_t command_id) {
  if (!finitePose(pose)) {
    return;
  }
  Pending& pending = pending_[armIndex(side)];
  pending.pose = pose;
  pending.command_id = command_id;
  pending.active = true;
}

Pose ManualTargetPreview::resolve(ArmSide side, const Pose& snapshot_pose,
                                  std::uint64_t last_processed_command_id) {
  Pending& pending = pending_[armIndex(side)];
  if (pending.active && last_processed_command_id >= pending.command_id) {
    pending.active = false;
  }
  return pending.active ? pending.pose : snapshot_pose;
}

void ManualTargetPreview::cancel(ArmSide side) noexcept {
  pending_[armIndex(side)].active = false;
}

void ManualTargetPreview::cancelAll() noexcept {
  for (Pending& pending : pending_) {
    pending.active = false;
  }
}

}  // namespace tianji_qp_ik
