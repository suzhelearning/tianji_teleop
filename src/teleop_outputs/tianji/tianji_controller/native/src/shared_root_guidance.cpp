#include "tianji_qp_ik/spark_guidance.hpp"
#include "tianji_qp_ik/so3.hpp"
#include <cmath>
#include <stdexcept>

namespace tianji_qp_ik {
namespace {
bool finiteState(const ArmMotionState& s) {
  return s.q.allFinite() && s.qdot.allFinite() && s.qddot.allFinite();
}
SparkUpperArmTarget modelTarget(const ArmKinematicSample& s) {
  SparkUpperArmTarget t;
  t.shoulder=s.shoulder_position;t.elbow=s.elbow_position;t.wrist=s.wrist_position;
  t.hand=s.tcp_pose.position;t.palm=s.tcp_pose;return t;
}
} // namespace
void DualArmSparkGuidance::initializeSharedRoot(const SharedRootOptions& options,
                                               const std::string& urdf) {
  if(!usesSharedRootDirectIk(config_.ik_algorithm) &&
     (posture_mode_!=SparkPostureGuideMode::kHeadroomFeedforwardJointReferenceVelocity ||
      !usesSparkHeadroomFeedforwardVelocityQp(config_.ik_algorithm) ||
      config_.control_level!=ControlLevel::kVelocity))
    throw std::invalid_argument("shared-root requires headroom/feedforward guidance");
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
  // Use the actual guidance's SPARK settings, not the isolated builder defaults.
  verified.builder.shape_config=config_.spark_upper_qpoases;
  shared_root_=std::make_unique<SharedRootPipeline>(verified);
}
bool DualArmSparkGuidance::updateSharedRootFrame(const PicoTeleopFrame& f,std::int64_t now) {
  if(!shared_root_) throw std::logic_error("shared-root not configured");
  shared_ack_ready_=false;
  return shared_root_->observe(f,now);
}
bool DualArmSparkGuidance::resetSharedRootControl(const ArmMotionState& l,const ArmMotionState& r) {
  last_feedforward_sequence_=last_palm_twist_sequence_=0;
  held_targets_={};blend_start_targets_={};blend_restart_pending_=false;
  blend_elapsed_seconds_=joint_reference_phase_=0;cancelJointSpaceTakeover();
  bool valid=true;
  for(auto side:{ArmSide::kLeft,ArmSide::kRight}) {
    auto& state=side==ArmSide::kLeft?left_:right_;
    const auto& model=side==ArmSide::kLeft?l:r;
    state.ik.reset();state.last_valid_q_ik=model.q;
    state.otg.reset(kinematics_.sample(side,model.q).tcp_pose);
    state.feedforward.reset(model,shared_output_.epoch);
    state.palm_twist.reset();state.motion_intent_twist.reset();state.headroom.reset();
    state.last_feedforward_target={};state.settled_hold_active=false;
    state.settled_hold_dwell_seconds=0;state.settled_hold_reason=SparkSettledHoldReason::kNone;
    // Shared-root is restricted at construction to headroom/feedforward QP.
    // The Ruckig posture reference is not consumed by that mode; its separate
    // velocity/acceleration envelope must not veto a moving recovery seed.
    // The outer controller retains ownership of reference validation/limits.
    valid=finiteState(model)&&valid;
  }
  return valid;
}
SparkGuidanceDiagnostics DualArmSparkGuidance::stepSharedRoot(
    const ArmMotionState& l,const ArmMotionState& r,double dt,std::int64_t now,
    bool authorized,bool model_valid) {
  if(!shared_root_) throw std::logic_error("shared-root not configured");
  ++shared_cycle_;shared_ack_ready_=false;
  SparkGuidanceDiagnostics out;out.shared_root_cycle=shared_cycle_;
  if(!authorized||!model_valid||!finiteState(l)||!finiteState(r)||!std::isfinite(dt)||dt<=0) {
    (void)shared_root_->step(now,false,{});
    shared_output_={};latest_targets_={};raw_targets_={};shared_suppress_hold_=false;
    out.detail="shared_root_execution_or_model_invalid";return out;
  }
  SparkUpperTargets model;model.valid=true;
  model.left=modelTarget(kinematics_.sample(ArmSide::kLeft,l.q));
  model.right=modelTarget(kinematics_.sample(ArmSide::kRight,r.q));
  shared_output_=shared_root_->step(now,true,model);
  out.shared_root_state=shared_output_.state;
  shared_suppress_hold_=shared_output_.suppress_stationary_hold;
  if(shared_output_.reset_histories && !resetSharedRootControl(l,r)) {
    (void)shared_root_->step(now,false,{});latest_targets_={};
    out.detail="shared_root_reference_reset_failed";return out;
  }
  latest_targets_=shared_output_.target;
  // A held mapping is not fresh. Use the existing feedforward stale/stop path;
  // never assign a new source sequence or freshness to the held target.
  latest_targets_.valid=shared_output_.valid &&
      (shared_output_.state==SharedRootState::kRecovering||shared_output_.state==SharedRootState::kTracking);
  raw_targets_=latest_targets_;
  const auto& candidate=shared_root_->candidate();
  if(latest_targets_.valid) {
    latest_source_sequence_=candidate.sequence;latest_source_timestamp_ns_=candidate.source_timestamp_ns;
    latest_source_epoch_=candidate.epoch;latest_source_discontinuity_=candidate.stream_discontinuity;
    latest_left_input_palm_=candidate.left_intent;latest_right_input_palm_=candidate.right_intent;
    shared_intent_valid_=candidate.intent_evidence_valid;
  }
  const auto previous_feedforward=last_feedforward_sequence_;
  if (usesSharedRootDirectIk(config_.ik_algorithm)) {
    out.reference_reset_required = shared_output_.reset_histories;
    // Mapping only: no SPARK IK, feedforward governor, or second QP. The
    // The direct IK + Ruckig controller's bilateral acceptance owns the outer ack.
    out.target_valid = out.accepted = latest_targets_.valid;
    out.left.target = latest_targets_.left; out.right.target = latest_targets_.right;
    out.cartesian_targets.left = latest_targets_.left.palm;
    out.cartesian_targets.right = latest_targets_.right.palm;
    out.cartesian_targets.left_stale = out.cartesian_targets.right_stale = !out.target_valid;
    out.detail = config_.ik_algorithm == IkAlgorithm::kPicoEeFrankaDls
        ? (out.target_valid ? "shared_root_dls_mapping_ready" : "shared_root_dls_mapping_stale")
        : (out.target_valid ? "shared_root_ceres_mapping_ready" : "shared_root_ceres_mapping_stale");
    shared_ack_ready_ = out.target_valid && latest_source_sequence_ != last_feedforward_sequence_;
    return out;
  }
  out=stepImpl(l,r,dt);out.shared_root_cycle=shared_cycle_;out.shared_root_state=shared_output_.state;
  if(blend_restart_pending_) {
    // stepImpl has detected corroborated release of an ordinary settled hold.
    // Its legacy blend flag must not be ignored or used to mix frame semantics.
    shared_root_->restartRecovery();blend_restart_pending_=false;
    return out; // this release cycle already holds the model via legacy safety
  }
  // Both references must consume this source in THIS cycle, during blend or
  // tracking. The outer bilateral ack commits elbow history; only alpha=1
  // transitions recovery. Cached per-arm success never commits a new branch.
  shared_ack_ready_=out.accepted && shared_output_.valid &&
      (shared_output_.state==SharedRootState::kRecovering||shared_output_.state==SharedRootState::kTracking) &&
      latest_source_sequence_!=previous_feedforward &&
      last_feedforward_sequence_==shared_output_.sequence &&
      out.left.ik.accepted&&out.right.ik.accepted&&
      out.left.feedforward_target.accepted&&out.right.feedforward_target.accepted;
  return out;
}
bool DualArmSparkGuidance::confirmSharedRootReference(std::uint64_t cycle,bool accepted) noexcept {
  if(!shared_root_||cycle!=shared_cycle_||!shared_ack_ready_) return false;
  shared_ack_ready_=false;
  const bool committed = accepted && shared_root_->accept(shared_output_.sequence,shared_output_.epoch,shared_output_.generation);
  // accept() returns recovery-transition status, not history-commit status.
  if (accepted && usesSharedRootDirectIk(config_.ik_algorithm))
    last_feedforward_sequence_ = shared_output_.sequence;
  return committed;
}
} // namespace tianji_qp_ik
