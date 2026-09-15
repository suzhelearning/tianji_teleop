#pragma once

#include "tianji_qp_ik/interactive_marker.hpp"

#include <mujoco/mujoco.h>

#include <Eigen/Core>

#include <limits>

namespace tianji_qp_ik {

struct MarkerScreenPoint {
  Eigen::Vector2d pixel{Eigen::Vector2d::Zero()};
  double depth{0.0};
  bool visible{false};
};

struct MarkerScreenPick {
  MarkerHandle handle{MarkerHandle::kNone};
  double distance_pixels{std::numeric_limits<double>::infinity()};
  double depth{std::numeric_limits<double>::infinity()};
};

MarkerScreenPoint projectMarkerPoint(const mjvScene& scene,
                                     const Eigen::Vector3d& point,
                                     int viewport_width, int viewport_height);

MarkerScreenPick pickMarkerHandle(const MarkerGeometry& geometry,
                                  const mjvScene& scene,
                                  int viewport_width, int viewport_height,
                                  const Eigen::Vector2d& cursor_pixel);

MarkerCamera markerCameraFromScene(const mjvScene& scene,
                                   const Eigen::Vector3d& marker_origin);

MarkerPointer markerPointerFromScene(const mjvScene& scene, double aspect_ratio,
                                     double relx, double rely,
                                     const Eigen::Vector3d& marker_origin);

void appendInteractiveMarker(const MarkerGeometry& geometry,
                             MarkerHandle hovered, MarkerHandle active,
                             mjvScene* scene);

}  // namespace tianji_qp_ik
