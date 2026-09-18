#pragma once
#include "tianji_teleop/ik/arm_ik_solver.hpp"
#include <memory>
namespace tianji_teleop {
class DexhandQpArmIk final : public ArmIkSolver {
public:
  DexhandQpArmIk(const std::string& urdf, const IkSettings& settings, const std::string& model_path);
  ~DexhandQpArmIk() override;
  void reset(ArmSide side) const override;
  bool owns_reference_state() const override { return true; }
  void command_feedback(ArmSide side, const ArmJointVector& q) const override;
  IkResult solve_timed(ArmSide side, const Eigen::Isometry3d& target,
    const ArmJointVector& q, const Eigen::Vector3d& elbow,
    double source_time, double receive_time, double now) const override;
  Eigen::Isometry3d forward(ArmSide side, const ArmJointVector& q) const override;
  IkResult solve(ArmSide side, const Eigen::Isometry3d& target,
    const ArmJointVector& q, const Eigen::Vector3d& elbow) const override;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
