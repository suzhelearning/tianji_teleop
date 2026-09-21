#include "tianji_qp_ik/pico_skeleton_overlay.hpp"

#include <array>
#include <utility>

namespace tianji_qp_ik {
namespace {

mjvGeom* appendGeom(mjvScene* scene) {
  if (scene == nullptr || scene->geoms == nullptr ||
      scene->ngeom >= scene->maxgeom) {
    return nullptr;
  }
  return &scene->geoms[scene->ngeom++];
}

void finishGeom(mjvGeom* geom) {
  geom->objtype = mjOBJ_UNKNOWN;
  geom->objid = -1;
  geom->category = mjCAT_DECOR;
  geom->emission = 0.25F;
  geom->specular = 0.15F;
  geom->shininess = 0.25F;
}

std::array<mjtNum, 3> pointArray(const Eigen::Vector3d& point) {
  return {static_cast<mjtNum>(point.x()), static_cast<mjtNum>(point.y()),
          static_cast<mjtNum>(point.z())};
}

std::array<float, 4> pointColor(std::size_t index,
                                SkeletonOverlayStyle style) {
  if (style == SkeletonOverlayStyle::kSpark) {
    if (index <= kPicoLeftHandPoint) {
      return {1.00F, 0.55F, 0.05F, 0.92F};
    }
    return {0.30F, 1.00F, 0.20F, 0.92F};
  }
  if (index <= kPicoLeftHandPoint) {
    return {0.05F, 0.90F, 1.00F, 0.78F};
  }
  return {1.00F, 0.25F, 0.75F, 0.78F};
}

}  // namespace

PicoUpperLimbSkeleton sparkUpperLimbSkeleton(
    const SparkUpperTargets& targets) noexcept {
  PicoUpperLimbSkeleton result;
  if (!targets.valid) {
    return result;
  }
  result.points = {
      targets.left.shoulder, targets.left.elbow, targets.left.wrist,
      targets.left.hand, targets.right.shoulder, targets.right.elbow,
      targets.right.wrist, targets.right.hand};
  for (const Eigen::Vector3d& point : result.points) {
    if (!point.allFinite()) {
      return {};
    }
  }
  result.valid = true;
  return result;
}

void appendPicoUpperLimbSkeleton(const PicoUpperLimbSkeleton& skeleton,
                                 mjvScene* scene,
                                 SkeletonOverlayStyle style) {
  if (!skeleton.valid) {
    return;
  }
  for (const Eigen::Vector3d& point : skeleton.points) {
    if (!point.allFinite()) {
      return;
    }
  }

  constexpr std::array<std::pair<std::size_t, std::size_t>, 7> kBones{{
      {kPicoLeftShoulderPoint, kPicoRightShoulderPoint},
      {kPicoLeftShoulderPoint, kPicoLeftElbowPoint},
      {kPicoLeftElbowPoint, kPicoLeftWristPoint},
      {kPicoLeftWristPoint, kPicoLeftHandPoint},
      {kPicoRightShoulderPoint, kPicoRightElbowPoint},
      {kPicoRightElbowPoint, kPicoRightWristPoint},
      {kPicoRightWristPoint, kPicoRightHandPoint},
  }};
  for (const auto& [begin_index, end_index] : kBones) {
    mjvGeom* geom = appendGeom(scene);
    if (geom == nullptr) {
      return;
    }
    const std::array<float, 4> color =
        begin_index == kPicoLeftShoulderPoint &&
                end_index == kPicoRightShoulderPoint
            ? (style == SkeletonOverlayStyle::kSpark
                   ? std::array<float, 4>{1.00F, 0.90F, 0.10F, 0.88F}
                   : std::array<float, 4>{0.95F, 0.95F, 0.95F, 0.68F})
            : pointColor(end_index, style);
    const auto begin = pointArray(skeleton.points[begin_index]);
    const auto end = pointArray(skeleton.points[end_index]);
    mjv_initGeom(geom, mjGEOM_LINE, nullptr, nullptr, nullptr, color.data());
    mjv_connector(geom, mjGEOM_LINE, static_cast<mjtNum>(5.0), begin.data(),
                  end.data());
    finishGeom(geom);
  }

  for (std::size_t index = 0; index < skeleton.points.size(); ++index) {
    mjvGeom* geom = appendGeom(scene);
    if (geom == nullptr) {
      return;
    }
    const std::array<mjtNum, 3> size{static_cast<mjtNum>(0.022), 0.0, 0.0};
    const auto position = pointArray(skeleton.points[index]);
    const auto color = pointColor(index, style);
    mjv_initGeom(geom, mjGEOM_SPHERE, size.data(), position.data(), nullptr,
                 color.data());
    finishGeom(geom);
  }
}

}  // namespace tianji_qp_ik
