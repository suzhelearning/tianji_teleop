#include "tianji_qp_ik/shared_root_morphology.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace tianji_qp_ik {
namespace {
bool inRange(double v,const std::array<double,2>& range) noexcept {
  return std::isfinite(v) && v>=range[0] && v<=range[1];
}
double median(std::array<double,128> values,std::size_t count) noexcept {
  std::sort(values.begin(),values.begin()+static_cast<std::ptrdiff_t>(count));
  return count%2 ? values[count/2] : .5*(values[count/2-1]+values[count/2]);
}
double mad(const std::array<double,128>& values,std::size_t count,double center) noexcept {
  std::array<double,128> deviations{};
  for(std::size_t i=0;i<count;++i) deviations[i]=std::abs(values[i]-center);
  return median(deviations,count)/center;
}
} // namespace
SharedRootMorphologyEstimator::SharedRootMorphologyEstimator(SharedRootMorphologyConfig c):config_(c) {
  if(c.minimum_unique_samples<2 || c.minimum_unique_samples>c.window_frames || c.window_frames>128)
    throw std::invalid_argument("invalid bounded morphology window");
  for(double v:{c.mad_ratio_max,c.outlier_ratio,c.provisional_timeout_s,
      c.sustained_invalid_timeout_s,c.robot_width_m,c.robot_reach_m})
    if(!std::isfinite(v)||v<=0) throw std::invalid_argument("invalid morphology scalar");
  for(auto r:{c.shoulder_range,c.upper_range,c.forearm_range,c.wrist_palm_range,
      c.reach_scale_range,c.lateral_scale_range})
    if(!std::isfinite(r[0])||!std::isfinite(r[1])||r[0]<=0||r[1]<=r[0])
      throw std::invalid_argument("invalid morphology range");
}
void SharedRootMorphologyEstimator::reset() noexcept {
  cursor_=count_=0; epoch_=generation_=sequence_=0;
  source_ns_=receive_ns_=started_ns_=invalid_since_ns_=0; last_={};
}
MorphologyEstimate SharedRootMorphologyEstimator::update(const SharedRootInput& f) noexcept {
  last_.new_sample=false;
  if(f.tracking_epoch==0 || f.source_timestamp_ns<=0 || f.receive_monotonic_ns<=0) {
    auto out=last_; out.valid=false; out.detail="invalid_metadata"; return out;
  }
  const bool changed=epoch_!=f.tracking_epoch || generation_!=f.resynchronization_generation;
  // Repeated flagged frames must not repeatedly initialize a context.
  if(changed || (f.stream_discontinuity && f.sequence>sequence_)) {
    reset(); epoch_=f.tracking_epoch; generation_=f.resynchronization_generation;
  }
  if(source_ns_ && (f.sequence<=sequence_ || f.source_timestamp_ns<=source_ns_ ||
      f.receive_monotonic_ns<receive_ns_)) return last_;
  sequence_=f.sequence; source_ns_=f.source_timestamp_ns; receive_ns_=f.receive_monotonic_ns;
  if(!started_ns_) started_ns_=receive_ns_;
  const double width=(f.left.p_shoulder_root_Ct-f.right.p_shoulder_root_Ct).norm();
  bool good=f.valid && inRange(width,config_.shoulder_range);
  double reach=0;
  for(const auto* s:{&f.left,&f.right}) {
    const double upper=(s->p_elbow_root_Ct-s->p_shoulder_root_Ct).norm();
    const double forearm=(s->p_wrist_root_Ct-s->p_elbow_root_Ct).norm();
    const double palm=(s->p_control_root_Ct-s->p_wrist_root_Ct).norm();
    good=good && inRange(upper,config_.upper_range) && inRange(forearm,config_.forearm_range)
        && inRange(palm,config_.wrist_palm_range);
    reach+=.5*(upper+forearm+palm);
  }
  good=good && inRange(config_.robot_width_m/width,config_.lateral_scale_range)
      && inRange(config_.robot_reach_m/reach,config_.reach_scale_range);
  if(good && last_.state==MorphologyState::kConfident) {
    good=std::abs(width/median(widths_,count_)-1)<=config_.outlier_ratio
      && std::abs(reach/median(reaches_,count_)-1)<=config_.outlier_ratio;
  }
  if(last_.state==MorphologyState::kInvalid) {
    // Continuity manager must explicitly reset/rebuild a candidate estimator.
    last_.valid=false; last_.detail="requires_candidate_rebuild"; return last_;
  }
  if(!good) {
    if(!invalid_since_ns_) invalid_since_ns_=receive_ns_;
    if(static_cast<double>(receive_ns_-invalid_since_ns_)*1e-9>=config_.sustained_invalid_timeout_s) {
      last_.state=MorphologyState::kInvalid; last_.valid=false;
    }
    last_.detail="morphology_sample_rejected";
    return last_;
  }
  invalid_since_ns_=0;
  widths_[cursor_]=width; reaches_[cursor_]=reach;
  cursor_=(cursor_+1)%config_.window_frames;
  count_=std::min(count_+1,config_.window_frames);
  const double wm=median(widths_,count_), rm=median(reaches_,count_);
  last_.samples=count_; last_.new_sample=true;
  last_.lateral_scale=config_.robot_width_m/wm;
  last_.reach_scale=config_.robot_reach_m/rm;
  const bool confident=count_>=config_.minimum_unique_samples &&
      mad(widths_,count_,wm)<=config_.mad_ratio_max && mad(reaches_,count_,rm)<=config_.mad_ratio_max;
  if(last_.state!=MorphologyState::kConfident)
    last_.state=confident ? MorphologyState::kConfident : MorphologyState::kProvisional;
  if(last_.state==MorphologyState::kProvisional &&
      static_cast<double>(receive_ns_-started_ns_)*1e-9>=config_.provisional_timeout_s)
    last_.state=MorphologyState::kInvalid;
  last_.valid=last_.state!=MorphologyState::kInvalid;
  last_.detail=last_.valid ? "accepted" : "provisional_timeout";
  return last_;
}
} // namespace tianji_qp_ik
