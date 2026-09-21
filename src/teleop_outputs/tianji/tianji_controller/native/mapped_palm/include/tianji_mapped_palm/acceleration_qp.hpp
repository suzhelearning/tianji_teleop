#pragma once

#include "tianji_mapped_palm/acceleration_ik.hpp"
#include "tianji_mapped_palm/config.hpp"
#include "tianji_mapped_palm/hierarchical_qp.hpp"
#include "tianji_mapped_palm/safety.hpp"

namespace tianji_mapped_palm {

struct AccelerationQpProblem {
  Mat15 H{Mat15::Zero()};
  Vec15 g{Vec15::Zero()};
  Mat6x15 A{Mat6x15::Zero()};
  Vec15 lower{Vec15::Zero()};
  Vec15 upper{Vec15::Zero()};
  Vec6 equality{Vec6::Zero()};
  JointAccelerationBounds joint_bounds;
  LinearJointConstraint linear_constraint;
};

struct AccelerationQpSolution {
  SolverStatus status{SolverStatus::kInvalidInput};
  Vec15 x{Vec15::Zero()};
  int iterations{0};
  double update_time_us{0.0};
  double solve_time_us{0.0};
  std::string_view detail{"not_initialized"};
};

class AccelerationQpBuilder {
 public:
  explicit AccelerationQpBuilder(AccelerationQpConfig config)
      : config_(config) {}
  AccelerationQpProblem build(const ArmAccelerationInput& input) const;

 private:
  AccelerationQpConfig config_;
};

SafetyDecision validateAccelerationProblem(const AccelerationQpProblem& problem,
                                           const SafetyConfig& safety);
SafetyDecision validateAccelerationSolution(
    const AccelerationQpProblem& problem,
    const AccelerationQpSolution& solution,
    const AccelerationQpConfig& config, const SafetyConfig& safety);

}  // namespace tianji_mapped_palm
