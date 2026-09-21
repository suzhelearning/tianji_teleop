#include "tianji_v131/velocity_qp.hpp"
#include <iostream>
#include <stdexcept>
#include <cmath>

// Exercise failure policy without depending on wall-clock qpOASES timeouts.
class FaultInjectingSolver final : public tianji_v131::IHierarchicalQpSolver {
 public:
  enum class Fault {none, invalid_solution, initialization};
  Fault fault{Fault::none};
  tianji_v131::HierarchicalQpoasesSolver real{tianji_v131::originalVelocityProfile().qpoases};
  bool initialize(const tianji_v131::HierarchicalQpProblem& p) override {
    return fault!=Fault::initialization && real.initialize(p);
  }
  tianji_v131::HierarchicalQpSolution solve(const tianji_v131::HierarchicalQpProblem& p) override {
    auto out=real.solve(p);
    if(fault==Fault::invalid_solution) {
      out.status=tianji_v131::SolverStatus::kSolved;
      out.x[7]+=1.0;  // invalid task equality despite a successful solver status
    }
    return out;
  }
  void reset() override {real.reset();}
};

int main() {
  using namespace tianji_v131;
  ArmLimits limits;
  limits.lower_position.setConstant(-2.0);
  limits.upper_position.setConstant(2.0);
  limits.velocity.setConstant(1.0);
  VelocityQp solver(limits);
  Vec7 q = Vec7::Zero();
  Pose target;
  target.position = Eigen::Vector3d(0.1, -0.05, 0.03);
  auto evaluate = [](const Vec7& x) {
    ArmKinematicSample sample;
    sample.tcp_pose.position = x.head<3>();
    sample.tcp_jacobian.setZero();
    sample.tcp_jacobian.leftCols<6>().setIdentity();
    return sample;
  };
  // The complete wrapper must agree with the original target manager + OTG
  // + adaptive servo, including repeated samples and the 50 ms stale boundary.
  const auto profile=originalVelocityProfile();
  for(const auto fault:{FaultInjectingSolver::Fault::invalid_solution,
                       FaultInjectingSolver::Fault::initialization}) {
    auto injected=std::make_unique<FaultInjectingSolver>();
    auto* control=injected.get();
    VelocityQp failing(limits,profile,std::move(injected));
    VelocityResult before;
    for(int tick=0;tick<50;++tick)
      before=failing.solve(ArmSide::kLeft,target,Vec7::Zero(),evaluate,.005);
    if(!before.accepted || before.qdot.norm()<1e-4) throw std::runtime_error("failure test did not establish motion");
    control->fault=fault;
    // Invalid solved result forces a cold retry; init failure is exercised on
    // the following tick, after that retry resets the numerical solver.
    if(fault==FaultInjectingSolver::Fault::initialization) {
      control->fault=FaultInjectingSolver::Fault::invalid_solution;
      before=failing.solve(ArmSide::kLeft,target,Vec7::Zero(),evaluate,.005);
      control->fault=fault;
    }
    const auto actual=failing.solve(ArmSide::kLeft,target,Vec7::Zero(),evaluate,.005);
    ArmIkInput input; input.q_ref=before.q;input.qdot_prev=before.qdot;
    input.qddot_prev=before.qddot;input.limits=limits;input.dt=.005;
    input.bounds=computeJointVelocityBounds(before.q,before.qdot,before.qddot,limits,profile.joint_limits,.005);
    input.bounds=applyFullJointTemporalEnvelope(input.bounds,before.qdot,before.qddot,.005,true,
      profile.pico_ee_v131_velocity_qp.temporal_envelope).bounds;
    const auto expected=boundedSolverFallback(input,profile.joint_limits,profile.safety);
    if(!expected || actual.accepted || !actual.fallback_applied ||
       (actual.q-(before.q+.005*(*expected))).norm()>1e-12 ||
       (actual.qdot-*expected).norm()>1e-12)
      throw std::runtime_error("solver validation/init failure did not preserve original bounded fallback");
    control->fault=FaultInjectingSolver::Fault::none;
    const auto recovered=failing.solve(ArmSide::kLeft,target,Vec7::Zero(),evaluate,.005);
    if(!recovered.accepted || (recovered.previous_q-actual.q).norm()>1e-12)
      throw std::runtime_error("fallback recovery lost integrated reference state");
  }
  TargetManager manager(profile, DualArmTargets{});
  manager.setMode(TargetMode::kManual,1.0);
  CartesianReferenceGenerator generator(profile.cartesian_otg,0.005);
  generator.reset(Pose{});
  VelocityQp timed(limits);
  Vec7 timed_q=Vec7::Zero();
  double stamp=1.0;
  for(int tick=0;tick<100;++tick) {
    const double now=1.0+tick*0.005;
    if(tick<40 && tick%2==0) stamp=now;
    manager.setManualTargets(target,target,stamp,stamp);
    const auto sampled=manager.sample(now);
    const auto expected=generator.update(sampled.left,sampled.left_twist,sampled.left_stale,0.005);
    const auto expected_servo=cartesianReferenceServoBreakdown(profile.cartesian_servo,expected,evaluate(timed_q).tcp_pose);
    const auto actual=timed.solve(ArmSide::kLeft,target,timed_q,evaluate,0.005,stamp,stamp,now);
    if(!actual.accepted) throw std::runtime_error(actual.detail);
    if((actual.reference.pose.position-expected.pose.position).norm()>1e-12 ||
       (actual.reference.twist-expected.twist).norm()>1e-12 ||
       (actual.servo.command-expected_servo.command).norm()>1e-12 ||
       actual.reference.stale!=expected.stale)
      throw std::runtime_error("original reference path mismatch");
    if(tick>55 && !actual.reference.stale) throw std::runtime_error("repeated frame refreshed timeout");
    timed_q=actual.q;
  }
  timed.reset();
  const auto invalid_time=timed.solve(ArmSide::kLeft,target,Vec7::Zero(),evaluate,0.005,
    std::numeric_limits<double>::quiet_NaN(),0,0);
  if(invalid_time.accepted) throw std::runtime_error("nonfinite target timestamp accepted");
  const auto restarted=timed.solve(ArmSide::kLeft,target,Vec7::Zero(),evaluate,0.005,0,0,0);
  if(!restarted.accepted || restarted.reference.stale || restarted.q.norm()>0.0002)
    throw std::runtime_error("reset retained target/OTG history");
  CartesianReference near_reference;
  near_reference.pose.position.x()=0.01;
  auto near=cartesianReferenceServoBreakdown(profile.cartesian_servo,near_reference,Pose{});
  if(std::abs(near.feedback.x()-0.04)>1e-12) throw std::runtime_error("adaptive near gain");
  near_reference.pose.position.x()=2.0;
  auto far=cartesianReferenceServoBreakdown(profile.cartesian_servo,near_reference,Pose{});
  if(std::abs(far.feedback.x()-30.0)>1e-12 || std::abs(far.command.x()-3.0)>1e-12)
    throw std::runtime_error("adaptive far gain / Cartesian cap");
  for(int i=0; i<400; ++i) {
    const auto r = solver.solve(ArmSide::kLeft, target, q, evaluate, 0.005);
    if(!r.accepted || !r.q.allFinite()) throw std::runtime_error(r.detail);
    if(i==0 && (r.q-q).norm()>0.0002)
      throw std::runtime_error("missing Cartesian OTG: initial jerk-limited reference jumped");
    if((r.q-q).cwiseAbs().maxCoeff()>0.00500001) throw std::runtime_error("velocity bound");
    q=r.q;
  }
  if((q.head<3>()-target.position).norm()>0.002) throw std::runtime_error("tracking failed");
  limits.velocity.setConstant(4.0);
  VelocityQp fast(limits);
  q.setZero(); Vec7 velocity=Vec7::Zero(), acceleration=Vec7::Zero();
  target.position=Eigen::Vector3d(1.0,0.3,-0.2);
  double peak=0;
  for(int tick=0;tick<300;++tick) {
    auto actual=fast.solve(ArmSide::kLeft,target,q,evaluate,0.005);
    if(!actual.accepted) throw std::runtime_error(actual.detail);
    Vec7 v=(actual.q-q)/0.005, a=(v-velocity)/0.005, j=(a-acceleration)/0.005;
    if(v.cwiseAbs().maxCoeff()>4.0+1e-7 ||
       (a.cwiseAbs().array()>profile.joint_limits.max_acceleration_rad_s2.array()+1e-6).any() ||
       (tick>0 && (j.cwiseAbs().array()>profile.joint_limits.max_jerk_rad_s3.array()+1e-4).any()))
      throw std::runtime_error("full velocity/acceleration/jerk envelope");
    if(actual.task_scale_position<0.75-1e-8 || actual.task_scale_position>1+1e-8)
      throw std::runtime_error("task scaling bounds");
    peak=std::max(peak,v.cwiseAbs().maxCoeff());
    q=actual.q;velocity=v;acceleration=a;
  }
  if(peak<1.3) throw std::runtime_error("legacy speed box still active");
  ArmIkInput fallback_input;
  fallback_input.q_ref.setZero(); fallback_input.qdot_prev.setConstant(0.2);
  fallback_input.qddot_prev.setConstant(1.0); fallback_input.dt=0.005;
  fallback_input.limits=limits;
  fallback_input.bounds.lower.setConstant(-0.1);
  fallback_input.bounds.upper.setConstant(0.1);
  auto fallback=boundedSolverFallback(fallback_input,profile.joint_limits,profile.safety);
  if(!fallback || (fallback->array()-0.1).abs().maxCoeff()>1e-12)
    throw std::runtime_error("original bounded fallback");
  fallback_input.q_ref.setConstant(2.0);
  if(boundedSolverFallback(fallback_input,profile.joint_limits,profile.safety))
    throw std::runtime_error("fallback violated margin");
  VelocityQp autonomous(limits), advancing(limits);
  Vec7 advanced=Vec7::Zero();
  for(int tick=0;tick<150;++tick) {
    autonomous.feedback(Vec7::Zero(),0.005);
    auto actual=autonomous.solve(ArmSide::kLeft,target,Vec7::Zero(),evaluate,0.005);
    auto expected=advancing.solve(ArmSide::kLeft,target,advanced,evaluate,0.005);
    if(!actual.accepted || !expected.accepted) throw std::runtime_error("model-state solve failed");
    if((actual.q-expected.q).norm()>1e-10)
      throw std::runtime_error("external command overwrote model q_ref/history");
    advanced=expected.q;
  }
  autonomous.reset(); advancing.reset();
  Vec7 reseed=Vec7::Constant(0.2);
  auto actual=autonomous.solve(ArmSide::kLeft,target,reseed,evaluate,0.005);
  auto expected=advancing.solve(ArmSide::kLeft,target,reseed,evaluate,0.005);
  if(!actual.accepted || (actual.q-expected.q).norm()>1e-10)
    throw std::runtime_error("explicit reset did not reseed model state");
  std::cout << "v131 velocity QP tracking and bounds passed\n";
}
