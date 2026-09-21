#pragma once

#include "tianji_qp_ik/types.hpp"

#include <Eigen/Core>

#include <array>
#include <cstdint>
#include <optional>

namespace tianji_qp_ik {

enum class MarkerFrame { kWorld, kLocal };

enum class MarkerHandle {
  kNone,
  kCenter,
  kTranslateX,
  kTranslateY,
  kTranslateZ,
  kRotateX,
  kRotateY,
  kRotateZ,
};

struct Ray3d {
  Eigen::Vector3d origin{Eigen::Vector3d::Zero()};
  Eigen::Vector3d direction{Eigen::Vector3d::UnitZ()};
};

struct MarkerPointer {
  Ray3d ray;
  Eigen::Vector2d ndc{Eigen::Vector2d::Zero()};
};

struct MarkerCamera {
  Eigen::Vector3d forward{Eigen::Vector3d::UnitZ()};
  Eigen::Vector3d right{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d up{Eigen::Vector3d::UnitY()};
  double world_per_ndc{1.0};
};

struct MarkerStyle {
  double scale{0.18};
  double center_radius_ratio{0.14};
  double arrow_start_ratio{0.22};
  double arrow_pick_radius_ratio{0.09};
  double ring_radius_ratio{0.72};
  double ring_pick_radius_ratio{0.09};
};

struct MarkerGeometry {
  Eigen::Vector3d origin{Eigen::Vector3d::Zero()};
  std::array<Eigen::Vector3d, 3> axes{
      Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ()};
  MarkerStyle style;
};

class InteractiveMarker6D {
 public:
  explicit InteractiveMarker6D(MarkerStyle style = {});

  void setFrame(MarkerFrame frame) noexcept { frame_ = frame; }
  MarkerFrame frame() const noexcept { return frame_; }
  MarkerHandle activeHandle() const noexcept { return active_handle_; }

  MarkerGeometry geometry(const Pose& pose) const;
  MarkerHandle pick(const Pose& pose, const Ray3d& ray) const;
  bool beginDrag(MarkerHandle handle, const Pose& pose,
                 const MarkerPointer& pointer, const MarkerCamera& camera);
  std::optional<Pose> updateDrag(const MarkerPointer& pointer) const;
  void endDrag() noexcept;
  void cancelDrag() noexcept;

 private:
  struct DragState {
    Pose start_pose;
    MarkerPointer start_pointer;
    MarkerCamera camera;
    Eigen::Vector3d axis{Eigen::Vector3d::Zero()};
    Eigen::Vector3d plane_normal{Eigen::Vector3d::Zero()};
    Eigen::Vector3d start_plane_point{Eigen::Vector3d::Zero()};
    Eigen::Vector3d start_radial{Eigen::Vector3d::Zero()};
    double start_axis_parameter{0.0};
    bool use_screen_fallback{false};
  };

  MarkerStyle style_;
  MarkerFrame frame_{MarkerFrame::kWorld};
  MarkerHandle active_handle_{MarkerHandle::kNone};
  std::optional<DragState> drag_;
};

class ManualTargetPreview {
 public:
  void record(ArmSide side, const Pose& pose, std::uint64_t command_id);
  Pose resolve(ArmSide side, const Pose& snapshot_pose,
               std::uint64_t last_processed_command_id);
  void cancel(ArmSide side) noexcept;
  void cancelAll() noexcept;

 private:
  struct Pending {
    Pose pose;
    std::uint64_t command_id{0U};
    bool active{false};
  };

  std::array<Pending, 2> pending_;
};

}  // namespace tianji_qp_ik
