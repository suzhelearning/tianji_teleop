#include "tianji_qp_ik/shared_root_target_builder.hpp"
#include "tianji_qp_ik/so3.hpp"
#include <cmath>
#include <stdexcept>

namespace tianji_qp_ik {
namespace {
// Project a single bilateral translation onto four outer-reach balls, a
// displacement ball and (with history) a speed ball. Fixed work/storage;
// Dykstra's iteration may fail to converge within budget: reject, never relax.
bool reachableTranslation(const SharedRootBuiltTargets& out,
    const SharedRootClosureGeometry& geometry,double maximum,
    const Eigen::Vector3d& previous,double step,bool has_previous,Eigen::Vector3d& delta) {
  std::array<Eigen::Vector3d,6> centers,duals;
  std::array<double,6> radii{};
  for(auto& v:duals)v.setZero();
  for(int layer=0;layer<2;++layer)for(int side=0;side<2;++side) {
    const auto& targets=layer?out.filtered:out.raw;
    const auto& arm=side?targets.right:targets.left;
    const auto& g=geometry[side];
    const int i=2*layer+side;
    centers[i]=g.shoulder_B-arm.palm.position-arm.palm.rotation*g.tcp_to_wrist_center.position;
    radii[i]=g.upper_length_m+g.forearm_length_m;
    if(!centers[i].allFinite()||!std::isfinite(radii[i])||radii[i]<=0)return false;
  }
  centers[4].setZero();radii[4]=maximum;
  centers[5]=previous;radii[5]=step;
  const int count=has_previous?6:5;
  delta.setZero();
  for(int iteration=0;iteration<64;++iteration) {
    for(int i=0;i<count;++i) {
      const Eigen::Vector3d y=delta+duals[i],v=y-centers[i];
      const double norm=v.norm();
      const Eigen::Vector3d projected=norm>radii[i]?Eigen::Vector3d(centers[i]+v*(radii[i]/norm)):y;
      duals[i]=y-projected;delta=projected;
    }
  }
  if(!delta.allFinite())return false;
  for(int i=0;i<count;++i)if((delta-centers[i]).norm()>radii[i]+1e-10)return false;
  // Inner reach, branch degeneracy and closure residuals are checked by the
  // existing exact closure below; their non-convex constraints are not relaxed.
  return true;
}
double alpha(double dt,double tau) { return -std::expm1(-dt/tau); }
Eigen::Matrix3d rotationBlend(const Eigen::Matrix3d& a,const Eigen::Matrix3d& b,double t) {
  return Eigen::Quaterniond(a).slerp(t,Eigen::Quaterniond(b)).normalized().toRotationMatrix();
}
bool validSide(const SharedRootSideInput& s) {
  return s.p_shoulder_root_Ct.allFinite() && s.p_elbow_root_Ct.allFinite() &&
      s.p_wrist_root_Ct.allFinite() && s.p_control_root_Ct.allFinite() &&
      s.p_shape_proxy_root_Ct.allFinite() && isProperRotation(s.R_palm_Ct) &&
      isProperRotation(s.R_shoulder_Ct) && isProperRotation(s.R_elbow_Ct) &&
      isProperRotation(s.R_wrist_Ct);
}
bool filterShape(SparkUpperArmTarget& out,const SparkUpperArmTarget& old,double t) {
  std::array<Eigen::Vector3d,3> now{out.elbow-out.shoulder,out.wrist-out.elbow,out.hand-out.wrist};
  std::array<Eigen::Vector3d,3> before{old.elbow-old.shoulder,old.wrist-old.elbow,old.hand-old.wrist};
  for(std::size_t i=0;i<3;++i) {
    const double n=now[i].norm();
    if(n<1e-9 || before[i].norm()<1e-9) return false;
    const Eigen::Vector3d u=before[i].normalized(), v=now[i]/n;
    // An antipodal direction has no unique interpolation plane. Reject instead
    // of selecting a hidden axis or normalizing a near-zero vector.
    if(u.dot(v)<-1+1e-8) return false;
    const Eigen::Quaterniond turn=Eigen::Quaterniond::FromTwoVectors(u,v);
    now[i]=n*(Eigen::Quaterniond::Identity().slerp(t,turn)*u);
  }
  out.elbow=out.shoulder+now[0]; out.wrist=out.elbow+now[1]; out.hand=out.wrist+now[2];
  return true;
}
} // namespace

SharedRootTargetBuilder::SharedRootTargetBuilder(SharedRootBuilderConfig c)
    :config_(std::move(c)),scaler_(config_.geometry,config_.shape_config) {
  if(!isProperRotation(config_.R_BCt)||!config_.o_B.allFinite())
    throw std::invalid_argument("invalid shared-root fixed transform");
  for(double x:{config_.center_tau_s,config_.relation_tau_s,config_.direction_tau_s,
      config_.orientation_tau_s,config_.scale_tau_s,config_.maximum_source_dt_s,
      config_.maximum_center_m,config_.maximum_correction_m})
    if(!std::isfinite(x)||x<=0) throw std::invalid_argument("invalid shared-root filter/gate");
  for(double x:{config_.projection_maximum_translation_m,config_.projection_maximum_speed_m_s})
    if(!std::isfinite(x)||x<=0)throw std::invalid_argument("invalid reachable projection limit");
  if(config_.reachable_projection_enabled&&(!config_.closure_geometry||config_.projection_maximum_translation_m>.10))
    throw std::invalid_argument("reachable projection requires geometry and bounded correction");
}
void SharedRootTargetBuilder::reset() noexcept {
  scaler_.reset(); previous_={}; initialized_=intent_upgraded_=intent_history_valid_=false;
  intent_scale_locked_=false;
  elbow_history_={};
  center_.setZero(); relation_.setZero(); filtered_scale_.setOnes(); intent_scale_.setOnes();
}
void SharedRootTargetBuilder::beginRecovery() noexcept {
  scaler_.reset(); initialized_=intent_history_valid_=false;
}
SharedRootBuiltTargets SharedRootTargetBuilder::update(const SharedRootInput& f,const MorphologyEstimate& m) {
  // All histories, including the direction selector, commit bilaterally only
  // after every validity gate succeeds. This bounded value copy allocates no heap.
  auto candidate=*this;
  auto out=candidate.updateCandidate(f,m);
  if(out.valid) *this=std::move(candidate);
  return out;
}
SharedRootBuiltTargets SharedRootTargetBuilder::updateCandidate(const SharedRootInput& f,const MorphologyEstimate& m) {
  SharedRootBuiltTargets out;
  out.sequence=f.sequence; out.epoch=f.tracking_epoch; out.generation=f.resynchronization_generation;
  out.source_timestamp_ns=f.source_timestamp_ns; out.receive_monotonic_ns=f.receive_monotonic_ns;
  out.stream_discontinuity=f.stream_discontinuity;
  out.detail="invalid_input_or_scale";
  if(!f.valid||!m.valid||!m.new_sample||!validSide(f.left)||!validSide(f.right)||
      f.tracking_epoch==0||f.source_timestamp_ns<=0||f.receive_monotonic_ns<=0||
      !std::isfinite(m.reach_scale)||!std::isfinite(m.lateral_scale)||
      m.reach_scale<=0||m.lateral_scale<=0) return out;
  const bool context_changed=previous_.valid &&
      (f.tracking_epoch!=previous_.epoch||f.resynchronization_generation!=previous_.generation);
  if(context_changed||(f.stream_discontinuity&&f.sequence>previous_.sequence)) reset();
  const double dt=initialized_ ? static_cast<double>(f.source_timestamp_ns-previous_.source_timestamp_ns)*1e-9 : 0;
  if(initialized_ && (f.sequence<=previous_.sequence||dt<=0||dt>config_.maximum_source_dt_s||
      f.receive_monotonic_ns<previous_.receive_monotonic_ns)) {
    out.detail="duplicate_or_source_gap"; return out;
  }
  const Eigen::Vector3d scale(m.reach_scale,m.lateral_scale,m.reach_scale);
  const Eigen::Vector3d center=.5*(f.left.p_control_root_Ct+f.right.p_control_root_Ct);
  const Eigen::Vector3d relation=f.right.p_control_root_Ct-f.left.p_control_root_Ct;
  const auto map=[&](const Eigen::Vector3d& p,const Eigen::Vector3d& s)->Eigen::Vector3d {
    return config_.o_B+config_.R_BCt*s.cwiseProduct(p);
  };
  const Eigen::Vector3d c_raw=map(center,scale);
  const Eigen::Vector3d d_raw=config_.R_BCt*scale.cwiseProduct(relation);
  if(!c_raw.allFinite()||!d_raw.allFinite()||(c_raw-config_.o_B).norm()>config_.maximum_center_m) {
    out.detail="target_gate"; return out;
  }
  Pose left,right;
  left.position=c_raw-.5*d_raw; right.position=c_raw+.5*d_raw;
  left.rotation=config_.R_BCt*f.left.R_palm_Ct; right.rotation=config_.R_BCt*f.right.R_palm_Ct;
  PicoUpperLimbSkeleton skeleton; skeleton.valid=skeleton.rotations_valid=true;
  for(std::size_t side=0;side<2;++side) {
    const auto& s=side==0 ? f.left : f.right;
    const std::array<Eigen::Vector3d,4> points{s.p_shoulder_root_Ct,s.p_elbow_root_Ct,s.p_wrist_root_Ct,s.p_shape_proxy_root_Ct};
    const std::array<Eigen::Matrix3d,4> rotations{s.R_shoulder_Ct,s.R_elbow_Ct,s.R_wrist_Ct,s.R_palm_Ct};
    for(std::size_t j=0;j<4;++j) {
      skeleton.points[side*4+j]=config_.o_B+config_.R_BCt*points[j]; // NO anisotropic scaling of shape
      skeleton.rotations[side*4+j]=Eigen::Quaterniond(config_.R_BCt*rotations[j]);
    }
  }
  out.raw=scaler_.update(skeleton,left,right);
  if(!out.raw.valid) {out.detail=out.raw.detail; return out;}
  if((left.position-out.raw.left.hand).norm()>config_.maximum_correction_m||
      (right.position-out.raw.right.hand).norm()>config_.maximum_correction_m) {
    out.detail="shape_correction_gate"; return out;
  }
  // Legacy shape construction copies hand into palm; the isolated adapter
  // explicitly replaces only the main palm task, preserving the proxy.
  out.raw.left.palm=left; out.raw.right.palm=right;
  out.filtered=out.raw;
  if(!initialized_) {
    center_=center; relation_=relation; filtered_scale_=scale;
    if(!intent_scale_locked_||(!intent_upgraded_&&m.state==MorphologyState::kConfident)) {
      intent_scale_=scale;intent_scale_locked_=true;
      intent_upgraded_=m.state==MorphologyState::kConfident;
    }
  } else {
    center_+=alpha(dt,config_.center_tau_s)*(center-center_);
    relation_+=alpha(dt,config_.relation_tau_s)*(relation-relation_);
    filtered_scale_+=alpha(dt,config_.scale_tau_s)*(scale-filtered_scale_);
    out.filtered.left.palm.rotation=rotationBlend(previous_.filtered.left.palm.rotation,left.rotation,alpha(dt,config_.orientation_tau_s));
    out.filtered.right.palm.rotation=rotationBlend(previous_.filtered.right.palm.rotation,right.rotation,alpha(dt,config_.orientation_tau_s));
    if(!filterShape(out.filtered.left,previous_.filtered_preference.left,alpha(dt,config_.direction_tau_s))||
        !filterShape(out.filtered.right,previous_.filtered_preference.right,alpha(dt,config_.direction_tau_s))) {
      out.detail="antipodal_shape_direction"; return out;
    }
    if(!intent_upgraded_&&m.state==MorphologyState::kConfident) {
      intent_scale_=scale; intent_upgraded_=true; intent_history_valid_=false;
    }
  }
  const Eigen::Vector3d cf=map(center_,filtered_scale_);
  const Eigen::Vector3d df=config_.R_BCt*filtered_scale_.cwiseProduct(relation_);
  out.filtered.left.palm.position=cf-.5*df; out.filtered.right.palm.position=cf+.5*df;
  if(!cf.allFinite()||!df.allFinite()||(cf-config_.o_B).norm()>config_.maximum_center_m ||
      (out.filtered.left.palm.position-out.filtered.left.hand).norm()>config_.maximum_correction_m ||
      (out.filtered.right.palm.position-out.filtered.right.hand).norm()>config_.maximum_correction_m) {
    out.detail="filtered_target_gate"; return out;
  }
  out.raw_preference=out.raw;out.filtered_preference=out.filtered;
  if(config_.reachable_projection_enabled) {
    if(!reachableTranslation(out,*config_.closure_geometry,config_.projection_maximum_translation_m,
        previous_.reachable_translation,config_.projection_maximum_speed_m_s*dt,initialized_,out.reachable_translation)) {
      out.detail="ReachableProjectionInfeasible";return out;
    }
    for(auto* layer:{&out.raw,&out.filtered}) {
      layer->left.palm.position+=out.reachable_translation;
      layer->right.palm.position+=out.reachable_translation;
      if((.5*(layer->left.palm.position+layer->right.palm.position)-config_.o_B).norm()>config_.maximum_center_m||
         (layer->left.palm.position-layer->left.hand).norm()>config_.maximum_correction_m||
         (layer->right.palm.position-layer->right.hand).norm()>config_.maximum_correction_m) {
        out.detail="ReachableProjectionTargetGate";return out;
      }
    }
  }
  if(config_.closure_geometry) {
    if(!closeSharedRootTargets(out.raw,*config_.closure_geometry,elbow_history_) ||
       !closeSharedRootTargets(out.filtered,*config_.closure_geometry,elbow_history_)) {
      out.detail=!out.raw.valid?out.raw.detail:out.filtered.detail;
      return out;
    }
  }
  out.left_intent=left; out.right_intent=right;
  out.left_intent.position=map(f.left.p_control_root_Ct,intent_scale_);
  out.right_intent.position=map(f.right.p_control_root_Ct,intent_scale_);
  out.intent_evidence_valid=initialized_&&intent_history_valid_&&
      out.reachable_translation.norm()<1e-12&&previous_.reachable_translation.norm()<1e-12;
  if(out.intent_evidence_valid) {
    out.left_intent_twist=poseErrorWorld(out.left_intent,previous_.left_intent)/dt;
    out.right_intent_twist=poseErrorWorld(out.right_intent,previous_.right_intent)/dt;
  }
  out.sequence=f.sequence; out.epoch=f.tracking_epoch; out.generation=f.resynchronization_generation;
  out.source_timestamp_ns=f.source_timestamp_ns; out.receive_monotonic_ns=f.receive_monotonic_ns;
  out.valid=true; out.detail="accepted";
  previous_=out; initialized_=intent_history_valid_=true;
  return out;
}

SparkUpperTargets blendSharedRootTargets(const SparkUpperTargets& start,const SparkUpperTargets& goal,double t) {
  SparkUpperTargets out=goal;
  if(!start.valid||!goal.valid||!std::isfinite(t)||t<0||t>1) {
    out.valid=false; out.detail="invalid_shared_blend"; return out;
  }
  for(int i=0;i<2;++i) {
    const auto& a=i==0?start.left:start.right; const auto& b=i==0?goal.left:goal.right;
    auto& o=i==0?out.left:out.right;
    o.palm.position=(1-t)*a.palm.position+t*b.palm.position;
    o.palm.rotation=rotationBlend(a.palm.rotation,b.palm.rotation,t);
    o.shoulder=(1-t)*a.shoulder+t*b.shoulder; o.elbow=(1-t)*a.elbow+t*b.elbow;
    o.wrist=(1-t)*a.wrist+t*b.wrist; o.hand=(1-t)*a.hand+t*b.hand;
  }
  return out;
}
} // namespace tianji_qp_ik
