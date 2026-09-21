#pragma once

#include "tianji_qp_ik/mujoco_robot.hpp"

#include <memory>
#include <array>
#include <string>

namespace tianji_qp_ik {

class CeresPinocchioArmKinematics {
 public:
  explicit CeresPinocchioArmKinematics(const std::string& urdf_path);
  CeresPinocchioArmKinematics(const std::string& urdf_path,
      const std::array<Pose, 2>& tcp_relative_to_link7);
  ~CeresPinocchioArmKinematics();

  CeresPinocchioArmKinematics(const CeresPinocchioArmKinematics&) = delete;
  CeresPinocchioArmKinematics& operator=(const CeresPinocchioArmKinematics&) = delete;
  CeresPinocchioArmKinematics(CeresPinocchioArmKinematics&&) noexcept;
  CeresPinocchioArmKinematics& operator=(CeresPinocchioArmKinematics&&) noexcept;

  int configurationSize() const noexcept;
  int velocitySize() const noexcept;
  ArmLimits limits(ArmSide side) const;
  // Fast EE-only sample for the direct PICO DLS hot path. The returned
  // sample's TCP pose/Jacobian are valid; the intermediate-link fields are
  // intentionally left at their default values.
  ArmKinematicSample sampleTcp(ArmSide side, const Vec7& q);
  ArmKinematicSample sample(ArmSide side, const Vec7& q);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace tianji_qp_ik
