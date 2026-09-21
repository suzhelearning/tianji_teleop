#include "tianji_v131/velocity_qp.hpp"
#include "tianji_v131/so3.hpp"
#include <chrono>
#include <cmath>

namespace tianji_v131 {
// Numerical portion of DualArmController::applyBoundedFallback. This path is
// used only after a solver failure, never to bypass invalid input/bounds.
std::optional<Vec7> boundedSolverFallback(const ArmIkInput& input,
    const JointLimitConfig& limits,const SafetyConfig& safety) {
  if(!std::isfinite(input.dt) || input.dt<=0.0 ||
     (input.bounds.lower.array()>input.bounds.upper.array()).any()) return std::nullopt;
  const Vec7 requested=input.qdot_prev+input.qddot_prev*input.dt;
  const Vec7 fallback=requested.cwiseMax(input.bounds.lower).cwiseMin(input.bounds.upper);
  const Vec7 candidate=input.q_ref+fallback*input.dt;
  if(!fallback.allFinite() || !candidate.allFinite() ||
     (candidate.array()<input.limits.lower_position.array()+limits.margin_rad-safety.bound_tolerance).any() ||
     (candidate.array()>input.limits.upper_position.array()-limits.margin_rad+safety.bound_tolerance).any())
    return std::nullopt;
  return fallback;
}
VelocityQp::VelocityQp(ArmLimits limits, QpIkConfig profile,
    std::unique_ptr<IHierarchicalQpSolver> solver)
  : limits_(limits), profile_(std::move(profile)),
    config_(profile_.pico_ee_v131_velocity_qp), hierarchy_(profile_.hierarchical_qp),
    bounds_(profile_.joint_limits), safety_(profile_.safety),
    solver_(solver ? std::move(solver) : std::make_unique<HierarchicalQpoasesSolver>(profile_.qpoases)) {
  builder_ = std::make_unique<HierarchicalQpBuilder>(hierarchy_, config_);
}
void VelocityQp::clearJointHistory() {
  history_=false; initialized_=false; solver_->reset();
  if(escape_) escape_->reset();
  velocity_.setZero(); acceleration_.setZero();
}
void VelocityQp::reset() {
  clearJointHistory();
  targets_.reset(); reference_.reset(); sample_time_=0.0;
  model_initialized_=false;
}
void VelocityQp::feedback(const Vec7& accepted_q, double dt) {
  if(!accepted_q.allFinite() || !std::isfinite(dt) || dt<=0.0)
    throw std::invalid_argument("invalid v131 accepted command feedback");
  // model_state_only: external delivery/actuator history is not the controller
  // reference history. Lifecycle reset + the next solve seed reanchors it.
}
VelocityResult VelocityQp::solve(ArmSide side, const Pose& target, const Vec7& seed,
    std::function<ArmKinematicSample(const Vec7&)> evaluate, double dt,
    double source_time, double receive_time, double now,
    std::function<ArmKinematicSample(const Vec7&)> gradient_evaluate) {
  VelocityResult result; result.q=model_initialized_?model_q_:seed;
  if(!seed.allFinite() || !target.position.allFinite() || !target.rotation.allFinite() ||
      !isProperRotation(target.rotation) || !std::isfinite(dt) || dt<=0 || !evaluate ||
      !std::isfinite(source_time) || !std::isfinite(receive_time) || !std::isfinite(now)) {
    result.detail="v131_invalid_input"; clearJointHistory(); return result;
  }
  kinematics_.evaluate=gradient_evaluate?std::move(gradient_evaluate):evaluate;
  if(!escape_) escape_=std::make_unique<PicoEeV131SingularityEscape7>(config_,kinematics_,side);
  if(!model_initialized_) {model_q_=seed; model_initialized_=true;}
  const Vec7 q=model_q_;
  result.previous_q=q;
  const auto sample=evaluate(q);
  // Untimed callers (native probes) represent a fresh frame on each control tick.
  if(now < 0.0) now=sample_time_+dt;
  if(source_time < 0.0) source_time=now;
  if(receive_time < 0.0) receive_time=now;
  sample_time_=now;
  if(!targets_) {
    DualArmTargets initial; initial.left=sample.tcp_pose; initial.right=sample.tcp_pose;
    targets_=std::make_unique<TargetManager>(profile_,initial);
    targets_->setMode(TargetMode::kManual,now);
    reference_=std::make_unique<CartesianReferenceGenerator>(profile_.cartesian_otg,dt);
    reference_->reset(sample.tcp_pose);
  }
  // Paired API is the original PICO packet path. Each host arm owns its own
  // manager; duplicating into its unused side preserves that API's semantics.
  targets_->setManualTargets(target,target,source_time,receive_time);
  const auto targets=targets_->sample(now);
  const auto desired=side==ArmSide::kLeft?targets.left:targets.right;
  const auto twist=side==ArmSide::kLeft?targets.left_twist:targets.right_twist;
  const bool stale=side==ArmSide::kLeft?targets.left_stale:targets.right_stale;
  result.reference=profile_.cartesian_otg.enabled
    ? reference_->update(desired,twist,stale,dt)
    : (side==ArmSide::kLeft?directReferences(targets).left:directReferences(targets).right);
  if(!result.reference.valid) {result.detail="v131_cartesian_otg_rejected";clearJointHistory();return result;}
  ArmIkInput input;
  input.q_measured=q; input.q_ref=q; input.qdot_prev=velocity_; input.qddot_prev=acceleration_;
  input.limits=limits_; input.dt=dt; input.jacobian=sample.tcp_jacobian;
  result.servo=cartesianReferenceServoBreakdown(profile_.cartesian_servo,result.reference,sample.tcp_pose);
  input.desired_twist=profile_.cartesian_otg.enabled?result.servo.command:
    cartesianServoTwist(profile_.cartesian_servo,desired,sample.tcp_pose,twist);
  if(!input.desired_twist.allFinite()) {result.detail="v131_cartesian_servo_rejected";clearJointHistory();return result;}
  input.bounds=computeJointVelocityBounds(q,velocity_,acceleration_,limits_,bounds_,dt);
  input.bounds=applyFullJointTemporalEnvelope(input.bounds,velocity_,acceleration_,dt,
    history_,config_.temporal_envelope).bounds;
  auto& task=input.pico_ee_v131_task;
  task.active=true; task.velocity_scale=limits_.velocity;
  task.zero_posture=side==ArmSide::kLeft?config_.zero_posture_left:config_.zero_posture_right;
  auto svd=computePicoEeV131JacobianSvd(input.jacobian);
  const auto singularity=escape_->update(q,svd,now);
  task.sigma_min=singularity.sigma_min; task.activation=singularity.activation;
  task.gradient=singularity.gradient; task.gradient_valid=singularity.gradient_valid;
  task.gradient_age_seconds=singularity.gradient_age_seconds;
  task.requested_sigma_dot=singularity.requested_sigma_dot; task.escape_active=singularity.escape_active;
  result.minimum_singular_value=singularity.sigma_min;
  const auto problem=builder_->build(input);
  if(!validateHierarchicalProblem(problem,safety_).accepted) {
    result.detail="v131_invalid_problem_or_bounds"; clearJointHistory(); return result;
  }
  HierarchicalQpSolution solution;
  bool accepted=false;
  for(int attempt=0; attempt<2; ++attempt) {
    if(!initialized_) initialized_=solver_->initialize(problem);
    if(initialized_) {
      solution=solver_->solve(problem);
      result.active_set_iterations+=solution.iterations; result.solve_time_us+=solution.solve_time_us;
      accepted=validateHierarchicalSolution(problem,solution,hierarchy_,safety_).accepted;
      if(accepted) break;
    }
    initialized_=false; solver_->reset();
  }
  std::optional<Vec7> fallback;
  // Original HierarchicalQpIk7 promotes Solved-but-invalid results to a
  // numerical solver failure after cold retry. Both failure forms are eligible
  // for the controller's bounded fallback; invalid problems never reach here.
  if(!accepted)
    fallback=boundedSolverFallback(input,bounds_,safety_);
  if(!accepted && !fallback) {result.detail="v131_qpoases_rejected";clearJointHistory();return result;}
  const Vec7 next_velocity=fallback?*fallback:Vec7(solution.x.head<7>());
  result.task_scale_position=solution.x[kBetaPositionIndex];
  result.task_scale_orientation=solution.x[kBetaOrientationIndex];
  const Vec7 candidate=q+dt*next_velocity;
  if(!candidate.allFinite() ||
      (candidate.array()<limits_.lower_position.array()+bounds_.margin_rad-safety_.bound_tolerance).any() ||
      (candidate.array()>limits_.upper_position.array()-bounds_.margin_rad+safety_.bound_tolerance).any()) {
    result.detail="v131_output_bounds_rejected"; clearJointHistory(); return result;
  }
  acceleration_=(next_velocity-velocity_)/dt; velocity_=next_velocity;
  model_q_=candidate;
  result.qdot=velocity_; result.qddot=acceleration_;
  last_q_=candidate; history_=true;
  result.accepted=accepted; result.fallback_applied=fallback.has_value();
  result.q=candidate; result.detail="pico_ee_v131_velocity_qp_solved";
  if(fallback) result.detail="v131_bounded_solver_fallback";
  return result;
}
}
