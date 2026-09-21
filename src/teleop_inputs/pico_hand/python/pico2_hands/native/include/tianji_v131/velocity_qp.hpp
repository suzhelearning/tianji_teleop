#pragma once
#include "tianji_v131/hierarchical_qpoases_solver.hpp"
#include "tianji_v131/pico_ee_v131_singularity.hpp"
#include "tianji_v131/full_joint_temporal_envelope.hpp"
#include "tianji_v131/cartesian_otg.hpp"
#include "tianji_v131/cartesian_servo.hpp"
#include "tianji_v131/velocity_profile.hpp"
#include <string>
#include <optional>

namespace tianji_v131 {
std::optional<Vec7> boundedSolverFallback(const ArmIkInput& input,
  const JointLimitConfig& limits, const SafetyConfig& safety);
struct VelocityResult {
  bool accepted{false};
  bool fallback_applied{false};
  Vec7 q{Vec7::Zero()};
  Vec7 previous_q{Vec7::Zero()}, qdot{Vec7::Zero()}, qddot{Vec7::Zero()};
  std::string detail{"v131_not_solved"};
  int active_set_iterations{0};
  int active_bound_count{0};
  double solve_time_us{0};
  double minimum_singular_value{0};
  CartesianReference reference;
  CartesianServoBreakdown servo;
  double task_scale_position{1.0}, task_scale_orientation{1.0};
};
class VelocityQp {
 public:
  explicit VelocityQp(ArmLimits limits, QpIkConfig profile = originalVelocityProfile(),
    std::unique_ptr<IHierarchicalQpSolver> solver = {});
  void reset();
  void feedback(const Vec7& accepted_q, double dt);
  VelocityResult solve(ArmSide side, const Pose& target, const Vec7& q,
    std::function<ArmKinematicSample(const Vec7&)> evaluate, double dt,
    double source_time = -1.0, double receive_time = -1.0, double now = -1.0,
    std::function<ArmKinematicSample(const Vec7&)> gradient_evaluate = {});
 private:
  void clearJointHistory();
  ArmLimits limits_;
  QpIkConfig profile_;
  std::unique_ptr<TargetManager> targets_;
  std::unique_ptr<CartesianReferenceGenerator> reference_;
  double sample_time_{0};
  bool model_initialized_{false};
  Vec7 model_q_{Vec7::Zero()};
  PicoEeV131VelocityQpConfig config_;
  HierarchicalQpConfig hierarchy_;
  JointLimitConfig bounds_;
  SafetyConfig safety_;
  PinocchioArmKinematics kinematics_;
  std::unique_ptr<PicoEeV131SingularityEscape7> escape_;
  std::unique_ptr<HierarchicalQpBuilder> builder_;
  std::unique_ptr<IHierarchicalQpSolver> solver_;
  bool initialized_{false}, history_{false};
  Vec7 last_q_{Vec7::Zero()}, velocity_{Vec7::Zero()}, acceleration_{Vec7::Zero()};
};
}
