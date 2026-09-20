#include "tianji_qp_ik/shared_root_pipeline.hpp"

namespace tianji_qp_ik {
SharedRootPipeline::SharedRootPipeline(const SharedRootOptions& o)
    : morphology_(o.morphology), builder_(o.builder), continuity_(o.continuity,o.builder.closure_geometry),
      timing_(o.continuity) {}

void SharedRootPipeline::clearAlgorithms() noexcept {
  morphology_.reset(); builder_.reset(); continuity_.resetSession();
  candidate_={}; estimate_={}; last_valid_receive_ns_=0;
  interrupted_=true; rebuild_scale_=await_confident_=false;
}
void SharedRootPipeline::resetSession() noexcept {
  clearAlgorithms(); sequence_=epoch_=generation_=0;
  source_ns_=receive_ns_=now_ns_=0;
}
void SharedRootPipeline::restartRecovery() noexcept {
  continuity_.resetSession();builder_.beginRecovery();candidate_={};
  interrupted_=true;last_valid_receive_ns_=0;
}
void SharedRootPipeline::interruptBuilder() noexcept {
  if(!interrupted_) builder_.beginRecovery();
  interrupted_=true;
}
bool SharedRootPipeline::observe(const PicoTeleopFrame& f,std::int64_t now) {
  if(now<=0||(now_ns_&&now<now_ns_)) {clearAlgorithms();return false;}
  now_ns_=now;
  if(f.tracking_epoch==0||f.source_timestamp_ns<=0||f.receive_monotonic_ns<=0) {
    candidate_={}; continuity_.observe(candidate_,now); interruptBuilder(); return false;
  }
  const bool context=epoch_!=f.tracking_epoch||generation_!=f.resynchronization_generation;
  if(!context&&source_ns_&&(f.sequence<=sequence_||f.source_timestamp_ns<=source_ns_))
    return false; // Even a duplicate discontinuity must not reset history.
  if(context||f.stream_discontinuity) {
    clearAlgorithms(); source_ns_=receive_ns_=0;
    epoch_=f.tracking_epoch; generation_=f.resynchronization_generation;
  }
  const bool gap=source_ns_&&
      (double(f.source_timestamp_ns-source_ns_)*1e-9>timing_.maximum_source_gap_s||
       double(f.receive_monotonic_ns-receive_ns_)*1e-9>timing_.maximum_receive_gap_s);
  const bool time_valid=f.receive_monotonic_ns>=receive_ns_&&now>=f.receive_monotonic_ns&&
      double(now-f.receive_monotonic_ns)*1e-9<=timing_.freshness_s;
  sequence_=f.sequence;source_ns_=f.source_timestamp_ns;
  // Never move the receiver-time fence backwards on a malformed candidate.
  if(f.receive_monotonic_ns>receive_ns_) receive_ns_=f.receive_monotonic_ns;
  if(gap) interruptBuilder();
  auto input=adapter_.adapt(f);
  if(!time_valid) input.valid=false;
  if(rebuild_scale_) {
    morphology_.reset(); estimate_={}; rebuild_scale_=false; await_confident_=true;
  }
  const auto old_morphology=morphology_;
  const auto old_estimate=estimate_;
  // Stale/replayed arrivals must not teach the estimator or refresh its timers.
  if(time_valid) estimate_=morphology_.update(input);
  else {estimate_.valid=false;estimate_.new_sample=false;}
  if(estimate_.state==MorphologyState::kInvalid) rebuild_scale_=true;
  if(await_confident_&&estimate_.state==MorphologyState::kConfident) await_confident_=false;
  auto build_estimate=estimate_;
  if(await_confident_) build_estimate.valid=false;
  builder_.setElbowHistory(continuity_.elbowHistory());
  candidate_=builder_.update(input,build_estimate);
  if(!candidate_.valid) {
    if(!await_confident_&&estimate_.valid&&estimate_.new_sample) {
      morphology_=old_morphology;estimate_=old_estimate;estimate_.new_sample=false;
    }
    interruptBuilder();
  } else {interrupted_=false;last_valid_receive_ns_=f.receive_monotonic_ns;}
  continuity_.observe(candidate_,now);
  return candidate_.valid;
}
SharedRootContinuityOutput SharedRootPipeline::step(std::int64_t now,bool authorized,
                                                    const SparkUpperTargets& model) {
  if(!authorized||now<=0||(now_ns_&&now<now_ns_)) {
    // Keep the unique-source fence on revocation: old data cannot rearm motion.
    clearAlgorithms(); return {};
  }
  now_ns_=now;
  if(last_valid_receive_ns_&&double(now-last_valid_receive_ns_)*1e-9>timing_.freshness_s)
    interruptBuilder();
  return continuity_.step(now,true,model);
}
bool SharedRootPipeline::accept(std::uint64_t seq,std::uint64_t epoch,std::uint64_t gen) noexcept {
  return continuity_.accept(seq,epoch,gen);
}
} // namespace tianji_qp_ik
