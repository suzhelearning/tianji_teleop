#include "tianji_qp_ik/shared_root_continuity.hpp"
#include "tianji_qp_ik/so3.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace tianji_qp_ik {
namespace {
bool validTargets(const SharedRootTargets& t) {
  if(!t.valid) return false;
  for(const auto* s:{&t.left,&t.right})
    if(!s->palm.position.allFinite()||!isProperRotation(s->palm.rotation)||
        !s->shoulder.allFinite()||!s->elbow.allFinite()||!s->wrist.allFinite()||!s->hand.allFinite())
      return false;
  return true;
}
} // namespace
SharedRootContinuity::SharedRootContinuity(SharedRootContinuityConfig c,
    std::optional<SharedRootClosureGeometry> geometry):config_(c),geometry_(std::move(geometry)) {
  if(c.recovery_frames<2||c.recovery_frames>128) throw std::invalid_argument("invalid recovery count");
  for(double v:{c.maximum_hold_s,c.freshness_s,c.minimum_receive_span_s,
      c.maximum_receive_gap_s,c.maximum_source_gap_s,c.blend_s,c.maximum_elbow_step_m})
    if(!std::isfinite(v)||v<=0) throw std::invalid_argument("invalid continuity time");
}
bool SharedRootContinuity::fresh(std::int64_t now,std::int64_t received,double seconds) const noexcept {
  return received>0 && now>=received && static_cast<double>(now-received)*1e-9<=seconds;
}
void SharedRootContinuity::resetSession() noexcept {
  state_=SharedRootState::kUninitialized; candidate_={}; held_={}; start_={}; pending_={};
  epoch_=generation_=sequence_=0; source_ns_=receive_ns_=held_receive_ns_=0;
  first_candidate_ns_=blend_start_ns_=last_now_ns_=0; candidate_count_=0;
  elbow_history_={};accepted_elbows_={};
}
void SharedRootContinuity::interrupt() noexcept {
  candidate_count_=0; first_candidate_ns_=0; candidate_.valid=false; pending_={};
  state_=held_.valid ? SharedRootState::kHoldLastMapped : SharedRootState::kInvalid;
}
void SharedRootContinuity::observe(const SharedRootBuiltTargets& f,std::int64_t now) {
  if(now<=0||(last_now_ns_&&now<last_now_ns_)) {interrupt();return;}
  last_now_ns_=now;
  if(f.epoch==0||f.source_timestamp_ns<=0||f.receive_monotonic_ns<=0) {interrupt();return;}
  if(epoch_!=f.epoch||generation_!=f.generation||
      (f.stream_discontinuity&&f.sequence>sequence_)) {
    resetSession(); epoch_=f.epoch;generation_=f.generation; last_now_ns_=now;
  }
  if(source_ns_&&(f.sequence<=sequence_||f.source_timestamp_ns<=source_ns_)) return;
  const bool gap=source_ns_ &&
      (static_cast<double>(f.source_timestamp_ns-source_ns_)*1e-9>config_.maximum_source_gap_s ||
       !fresh(f.receive_monotonic_ns,receive_ns_,config_.maximum_receive_gap_s));
  sequence_=f.sequence;source_ns_=f.source_timestamp_ns;receive_ns_=f.receive_monotonic_ns;
  if(gap) interrupt();
  if(!f.valid||!validTargets(f.filtered)||!fresh(now,f.receive_monotonic_ns,config_.freshness_s)) {
    interrupt();return;
  }
  if(candidate_count_==0) first_candidate_ns_=f.receive_monotonic_ns;
  candidate_count_=std::min(candidate_count_+1,config_.recovery_frames);
  candidate_=f;
  pending_={}; // old IK acknowledgements cannot complete a newer candidate
}
SharedRootContinuityOutput SharedRootContinuity::step(std::int64_t now,bool authorized,
                                                     const SharedRootTargets& model) {
  SharedRootContinuityOutput out;
  if(!authorized) {resetSession();return out;}
  if(now<=0||(last_now_ns_&&now<last_now_ns_)) {interrupt();out.state=state_;return out;}
  last_now_ns_=now;
  if(candidate_.valid&&!fresh(now,candidate_.receive_monotonic_ns,config_.freshness_s)) interrupt();
  // Model seed is read-only fallback, not an accepted target history.
  if(geometry_&&!elbow_history_[0]&&validTargets(model))
    elbow_history_={model.left.elbow,model.right.elbow};
  const bool ready=candidate_.valid && candidate_count_>=config_.recovery_frames &&
      static_cast<double>(candidate_.receive_monotonic_ns-first_candidate_ns_)*1e-9>=config_.minimum_receive_span_s;
  if(state_!=SharedRootState::kTracking&&state_!=SharedRootState::kRecovering&&ready&&validTargets(model)) {
    start_=model;blend_start_ns_=now;state_=SharedRootState::kRecovering;
    if(geometry_)elbow_history_={model.left.elbow,model.right.elbow};
    out.reset_histories=true;
  }
  if(state_==SharedRootState::kHoldLastMapped) {
    if(fresh(now,held_receive_ns_,std::min(config_.maximum_hold_s,config_.freshness_s))) {
      out.valid=true;out.target=held_;out.receive_monotonic_ns=held_receive_ns_;
    } else state_=SharedRootState::kInvalid;
  } else if(state_==SharedRootState::kTracking||state_==SharedRootState::kRecovering) {
    out.alpha=state_==SharedRootState::kTracking ? 1 :
        std::clamp(static_cast<double>(now-blend_start_ns_)*1e-9/config_.blend_s,0.,1.);
    out.target=state_==SharedRootState::kTracking ? candidate_.filtered :
        blendSharedRootTargets(start_,candidate_.filtered,out.alpha);
    if(geometry_) {
      // Interpolate only preference, then close from this cycle's palm pose.
      if(state_==SharedRootState::kRecovering) {
        out.target.left.elbow=(1-out.alpha)*start_.left.elbow+out.alpha*candidate_.filtered_preference.left.elbow;
        out.target.right.elbow=(1-out.alpha)*start_.right.elbow+out.alpha*candidate_.filtered_preference.right.elbow;
      } else {
        out.target.left.elbow=candidate_.filtered_preference.left.elbow;
        out.target.right.elbow=candidate_.filtered_preference.right.elbow;
      }
      if(!closeSharedRootTargets(out.target,*geometry_,elbow_history_)) {
        const auto reason=out.target.detail;
        interrupt();out.valid=false;out.target={};out.target.detail=reason;
        out.state=state_;return out;
      }
      if(elbow_history_[0]&&elbow_history_[1]&&
          ((out.target.left.elbow-*elbow_history_[0]).norm()>config_.maximum_elbow_step_m||
           (out.target.right.elbow-*elbow_history_[1]).norm()>config_.maximum_elbow_step_m)) {
        interrupt();out.valid=false;out.target={};out.target.detail="ElbowBranchDiscontinuity";
        out.state=state_;return out;
      }
    }
    out.valid=out.target.valid;out.suppress_stationary_hold=state_==SharedRootState::kRecovering;
    out.sequence=candidate_.sequence;out.epoch=candidate_.epoch;out.generation=candidate_.generation;
    out.receive_monotonic_ns=candidate_.receive_monotonic_ns;
    if(out.valid&&!geometry_) {held_=out.target;held_receive_ns_=candidate_.receive_monotonic_ns;}
  }
  out.state=state_;pending_=out;return out;
}
bool SharedRootContinuity::accept(std::uint64_t sequence,std::uint64_t epoch,std::uint64_t generation) noexcept {
  if(geometry_) {
    if((state_!=SharedRootState::kRecovering&&state_!=SharedRootState::kTracking)||!pending_.valid||
        sequence!=pending_.sequence||epoch!=pending_.epoch||generation!=pending_.generation)return false;
    // Called only after same-cycle bilateral outer reference acceptance.
    accepted_elbows_={pending_.target.left.elbow,pending_.target.right.elbow};
    elbow_history_=accepted_elbows_;held_=pending_.target;held_receive_ns_=pending_.receive_monotonic_ns;
    const bool transition=state_==SharedRootState::kRecovering&&pending_.alpha==1;
    if(transition)state_=SharedRootState::kTracking;
    pending_={};return transition; // preserve public recovery-confirmation meaning
  }
  if(state_!=SharedRootState::kRecovering||!pending_.valid||pending_.alpha!=1||
      sequence!=pending_.sequence||epoch!=pending_.epoch||generation!=pending_.generation) return false;
  state_=SharedRootState::kTracking;pending_={};return true;
}
} // namespace tianji_qp_ik
