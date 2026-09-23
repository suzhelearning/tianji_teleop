#include "tianji_qp_ik/shared_root_guidance.hpp"
#include "tianji_qp_ik/so3.hpp"
#include <cmath>
#include <stdexcept>

namespace tianji_qp_ik {
namespace {
bool finiteState(const ArmMotionState& s) {
  return s.q.allFinite() && s.qdot.allFinite() && s.qddot.allFinite();
}
SharedRootArmTarget modelTarget(const ArmKinematicSample& s) {
  SharedRootArmTarget t;
  t.shoulder=s.shoulder_position;t.elbow=s.elbow_position;t.wrist=s.wrist_position;
  t.hand=s.tcp_pose.position;t.palm=s.tcp_pose;return t;
}
} // namespace

SharedRootGuidance::SharedRootGuidance(
    MujocoRobot& robot,const QpIkConfig& config,const std::string& urdf,
    const SharedRootOptions* options)
    :robot_(robot),kinematics_(urdf,{robot.tcpRelativeToLink7(ArmSide::kLeft),
                                  robot.tcpRelativeToLink7(ArmSide::kRight)}) {
  if(!options||!options->enabled)
    throw std::invalid_argument("shared-root mapping must be configured and enabled");
  ArmMotionState left,right;
  left.q=robot.armPosition(ArmSide::kLeft);
  right.q=robot.armPosition(ArmSide::kRight);
  if(!finiteState(left)||!finiteState(right))
    throw std::invalid_argument("cannot initialize shared-root guidance state");
  initializeSharedRoot(*options,urdf,config.shared_root_shape);
}

void SharedRootGuidance::initializeSharedRoot(
    const SharedRootOptions& options,const std::string& urdf,
    const SharedRootShapeConfig& shape) {
  // Reload frozen evidence at startup. No caller can silently swap the TCP,
  // protocol contract or fixed geometry after the startup options were loaded.
  auto verified=loadSharedRootOptions(options.profile_path);
  if(verified.profile_sha256!=options.profile_sha256 ||
     sharedRootSha256File(urdf)!=sharedRootSha256File(verified.urdf_path))
    throw std::invalid_argument("shared-root startup configuration changed");
  MujocoRobot reference(verified.mujoco_path);
  for(auto side:{ArmSide::kLeft,ArmSide::kRight}) {
    const auto actual=robot_.tcpRelativeToLink7(side),expected=reference.tcpRelativeToLink7(side);
    if((actual.position-expected.position).norm()>1e-10 ||
        rotationDistance(actual.rotation,expected.rotation)>1e-10)
      throw std::invalid_argument("shared-root runtime TCP mismatch");
    const auto& a=robot_.mapping(side).limits;const auto& e=reference.mapping(side).limits;
    if(!a.lower_position.isApprox(e.lower_position,1e-12)||
       !a.upper_position.isApprox(e.upper_position,1e-12))
      throw std::invalid_argument("shared-root runtime joint limits mismatch");
    for(double fraction:{0.,.35,.65}) {
      const Vec7 q=fraction*(e.lower_position+e.upper_position);
      const auto x=robot_.armKinematicsAt(side,q),y=reference.armKinematicsAt(side,q);
      if((x.shoulder_position-y.shoulder_position).norm()>1e-10||
         (x.elbow_position-y.elbow_position).norm()>1e-10||
         (x.wrist_position-y.wrist_position).norm()>1e-10||
         (x.tcp_pose.position-y.tcp_pose.position).norm()>1e-10||
         rotationDistance(x.tcp_pose.rotation,y.tcp_pose.rotation)>1e-10)
        throw std::invalid_argument("shared-root runtime geometry mismatch");
    }
  }
  verified.builder.shape_config=shape;
  shared_root_=std::make_unique<SharedRootPipeline>(verified);
}

bool SharedRootGuidance::reset(const ArmMotionState& l,const ArmMotionState& r) noexcept {
  if(!finiteState(l)||!finiteState(r))return false;
  shared_root_->resetSession();
  shared_ack_ready_=false;
  shared_output_={};latest_targets_={};
  latest_source_sequence_=last_accepted_sequence_=0;
  return true;
}

bool SharedRootGuidance::resetMappingSession(
    const ArmMotionState& l,const ArmMotionState& r) noexcept {
  return reset(l,r);
}

bool SharedRootGuidance::updateSharedRootFrame(const PicoTeleopFrame& f,std::int64_t now) {
  shared_ack_ready_=false;
  return shared_root_->observe(f,now);
}

SharedRootGuidanceDiagnostics SharedRootGuidance::stepSharedRoot(
    const ArmMotionState& l,const ArmMotionState& r,double dt,std::int64_t now,
    bool authorized,bool model_valid) {
  ++shared_cycle_;shared_ack_ready_=false;
  SharedRootGuidanceDiagnostics out;out.shared_root_cycle=shared_cycle_;
  if(!authorized||!model_valid||!finiteState(l)||!finiteState(r)||!std::isfinite(dt)||dt<=0) {
    (void)shared_root_->step(now,false,{});
    shared_output_={};latest_targets_={};
    out.detail="shared_root_execution_or_model_invalid";return out;
  }
  SharedRootTargets model;model.valid=true;
  model.left=modelTarget(kinematics_.sample(ArmSide::kLeft,l.q));
  model.right=modelTarget(kinematics_.sample(ArmSide::kRight,r.q));
  shared_output_=shared_root_->step(now,true,model);
  out.shared_root_state=shared_output_.state;
  if(shared_output_.reset_histories)last_accepted_sequence_=0;
  latest_targets_=shared_output_.target;
  // A held mapping is not fresh. Never assign a new source sequence or
  // freshness to the held target.
  latest_targets_.valid=shared_output_.valid &&
      (shared_output_.state==SharedRootState::kRecovering||shared_output_.state==SharedRootState::kTracking);
  if(latest_targets_.valid)latest_source_sequence_=shared_root_->candidate().sequence;
  out.reference_reset_required=shared_output_.reset_histories;
  out.target_valid=out.accepted=latest_targets_.valid;
  out.left.target=latest_targets_.left;out.right.target=latest_targets_.right;
  out.left.accepted=out.right.accepted=out.target_valid;
  out.cartesian_targets.left=latest_targets_.left.palm;
  out.cartesian_targets.right=latest_targets_.right.palm;
  out.cartesian_targets.left_stale=out.cartesian_targets.right_stale=!out.target_valid;
  out.detail=out.target_valid?"shared_root_dls_mapping_ready":"shared_root_dls_mapping_stale";
  shared_ack_ready_=out.target_valid&&latest_source_sequence_!=last_accepted_sequence_;
  return out;
}

bool SharedRootGuidance::confirmSharedRootReference(std::uint64_t cycle,bool accepted) noexcept {
  if(cycle!=shared_cycle_||!shared_ack_ready_)return false;
  shared_ack_ready_=false;
  const bool committed=accepted&&shared_root_->accept(
      shared_output_.sequence,shared_output_.epoch,shared_output_.generation);
  // accept() returns recovery-transition status, not history-commit status.
  if(accepted)last_accepted_sequence_=shared_output_.sequence;
  return committed;
}

void SharedRootGuidance::invalidateTarget(std::string_view detail) noexcept {
  shared_root_->resetSession();
  shared_ack_ready_=false;shared_output_={};
  latest_targets_.valid=false;latest_targets_.detail=detail;
}
} // namespace tianji_qp_ik
