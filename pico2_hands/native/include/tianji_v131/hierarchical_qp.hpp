// Ported from TJ_arm_control_pico_ee_ik; see PORTING.md.
#pragma once

#include "tianji_v131/safety.hpp"
#include "tianji_v131/velocity_ik.hpp"

#include <Eigen/Core>

#include <string_view>

namespace tianji_v131 {

inline constexpr int kSlackStartIndex = kArmDof;
inline constexpr int kSlackVariables = 6;
inline constexpr int kBetaPositionIndex = kSlackStartIndex + kSlackVariables;
inline constexpr int kBetaOrientationIndex = kBetaPositionIndex + 1;
inline constexpr int kHierarchicalVariables = kBetaOrientationIndex + 1;
inline constexpr int kAccelerationVariables = kBetaOrientationIndex + 1;
inline constexpr int kTrackingEqualities = 6;
inline constexpr int kVelocityQpConstraints = kTrackingEqualities + 2;
inline constexpr int kAccelerationQpConstraints = kTrackingEqualities + 1;

using VecVelocityQp = Eigen::Matrix<double, kHierarchicalVariables, 1>;
using MatVelocityQp =
    Eigen::Matrix<double, kHierarchicalVariables, kHierarchicalVariables>;
using Mat6xVelocityQp =
    Eigen::Matrix<double, kTrackingEqualities, kHierarchicalVariables>;
using Vec15 = Eigen::Matrix<double, kAccelerationVariables, 1>;
using Mat15 = Eigen::Matrix<double, kAccelerationVariables, kAccelerationVariables>;
using Mat6x15 = Eigen::Matrix<double, kTrackingEqualities, kAccelerationVariables>;

struct HierarchicalQpProblem {
  MatVelocityQp H{MatVelocityQp::Zero()};
  VecVelocityQp g{VecVelocityQp::Zero()};
  Mat6xVelocityQp A{Mat6xVelocityQp::Zero()};
  VecVelocityQp lower{VecVelocityQp::Zero()};
  VecVelocityQp upper{VecVelocityQp::Zero()};
  Vec6 equality{Vec6::Zero()};
  JointVelocityBounds joint_bounds;
  LinearJointConstraint linear_constraint;
  LinearJointConstraint branch_lock_constraint;
};

struct HierarchicalQpSolution {
  SolverStatus status{SolverStatus::kInvalidInput};
  VecVelocityQp x{VecVelocityQp::Zero()};
  int iterations{0};
  double update_time_us{0.0};
  double solve_time_us{0.0};
  std::string_view detail{"not_initialized"};
};

class HierarchicalQpBuilder {
 public:
  explicit HierarchicalQpBuilder(
      HierarchicalQpConfig config,
      PicoEeV131VelocityQpConfig v131_config = {});

  HierarchicalQpProblem build(const ArmIkInput& input) const;

 private:
  HierarchicalQpConfig config_;
  PicoEeV131VelocityQpConfig v131_config_;
};

SafetyDecision validateHierarchicalProblem(
    const HierarchicalQpProblem& problem, const SafetyConfig& safety);

SafetyDecision validateHierarchicalSolution(
    const HierarchicalQpProblem& problem,
    const HierarchicalQpSolution& solution,
    const HierarchicalQpConfig& config,
    const SafetyConfig& safety);

}  // namespace tianji_v131
