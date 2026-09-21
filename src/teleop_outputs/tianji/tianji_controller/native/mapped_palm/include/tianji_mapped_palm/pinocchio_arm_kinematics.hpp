#pragma once

#include "tianji_mapped_palm/mujoco_robot.hpp"

#include <array>
#include <memory>
#include <string>

namespace tianji_mapped_palm {

class PinocchioArmKinematics {
 public:
  explicit PinocchioArmKinematics(const std::string& urdf_path);
  PinocchioArmKinematics(
      const std::string& urdf_path,
      const std::array<Pose, 2>& tcp_relative_to_link7);
  ~PinocchioArmKinematics();

  PinocchioArmKinematics(const PinocchioArmKinematics&) = delete;
  PinocchioArmKinematics& operator=(const PinocchioArmKinematics&) = delete;
  PinocchioArmKinematics(PinocchioArmKinematics&&) noexcept;
  PinocchioArmKinematics& operator=(PinocchioArmKinematics&&) noexcept;

  int configurationSize() const noexcept;
  int velocitySize() const noexcept;
  ArmKinematicSample sample(ArmSide side, const Vec7& q);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace tianji_mapped_palm
