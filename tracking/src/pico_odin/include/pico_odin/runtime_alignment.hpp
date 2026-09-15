#pragma once

#include <array>
#include <optional>
#include <vector>

#include "pico_odin/se3.hpp"

namespace pico_odin {

using Covariance6 = std::array<double, 36>;

Eigen::Matrix<double, 6, 6> twist_adjoint(const Pose3 & pelvis_T_odin);
Covariance6 transform_covariance(
  const Covariance6 & covariance,
  const Eigen::Matrix<double, 6, 6> & jacobian);

class RuntimeAlignment {
 public:
  explicit RuntimeAlignment(Pose3 pelvis_T_odin);

  void clear();
  void initialize(
    const Pose3 & pico_world_T_pelvis0,
    const Pose3 & odin_world_T_odin0);
  bool initialized() const;
  Pose3 corrected_pelvis(const Pose3 & odin_world_T_odin) const;
  std::vector<Pose3> anchor_skeleton(
    const std::vector<Pose3> & pico_joints,
    const Pose3 & corrected_root) const;
  const Pose3 & pelvis_T_odin() const { return pelvis_T_odin_; }
  const Pose3 & session_transform() const;

 private:
  Pose3 pelvis_T_odin_;
  std::optional<Pose3> pico_world_T_odin_world_;
};

}  // namespace pico_odin
