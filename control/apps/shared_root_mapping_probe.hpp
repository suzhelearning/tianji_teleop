#pragma once
// Offline-only constraint reconstruction. Never used by the production builder.
#include "tianji_qp_ik/shared_root_target_builder.hpp"
#include <algorithm>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace tianji_qp_ik {
struct MappingProbeBalls {
  std::array<Eigen::Vector3d,6> center;
  std::array<double,6> radius{};
  int count{5};
  static constexpr const char* names[]={"raw_left","raw_right","filtered_left",
      "filtered_right","translation_bound","speed_bound"};
  Eigen::Vector3d solve(int iterations) const {
    std::array<Eigen::Vector3d,6> dual;
    for(auto& v:dual)v.setZero();
    Eigen::Vector3d delta=Eigen::Vector3d::Zero();
    for(int iteration=0;iteration<iterations;++iteration)for(int i=0;i<count;++i) {
      const Eigen::Vector3d y=delta+dual[i],v=y-center[i];
      const double norm=v.norm();
      const Eigen::Vector3d projected=norm>radius[i]?Eigen::Vector3d(center[i]+v*(radius[i]/norm)):y;
      dual[i]=y-projected;delta=projected;
    }
    return delta;
  }
  double residual(const Eigen::Vector3d& delta,int i) const {return (delta-center[i]).norm()-radius[i];}
  double maximumResidual(const Eigen::Vector3d& delta) const {
    double maximum=-std::numeric_limits<double>::infinity();
    for(int i=0;i<count;++i)maximum=std::max(maximum,residual(delta,i));
    return maximum;
  }
};

class MappingTransitionProbe {
 public:
  void observe(const SharedRootBuiltTargets& built,const SharedRootBuilderConfig& config,
               bool has_history,std::int64_t origin) {
    if(has_history&&(!previous_.valid||built.source_timestamp_ns<=previous_.source_timestamp_ns))
      throw std::runtime_error("mapping probe history mismatch");
    const double dt=has_history?double(built.source_timestamp_ns-previous_.source_timestamp_ns)*1e-9:0;
    if(config.reachable_projection_enabled&&built.raw_preference.valid&&built.filtered_preference.valid) {
      MappingProbeBalls balls;
      for(int layer=0;layer<2;++layer)for(int side=0;side<2;++side) {
        const auto& preference=layer?built.filtered_preference:built.raw_preference;
        const auto& arm=side?preference.right:preference.left;
        const auto& geometry=(*config.closure_geometry)[side];
        const int i=2*layer+side;
        balls.center[i]=geometry.shoulder_B-arm.palm.position-arm.palm.rotation*geometry.tcp_to_wrist_center.position;
        balls.radius[i]=geometry.upper_length_m+geometry.forearm_length_m;
      }
      balls.center[4].setZero();balls.radius[4]=config.projection_maximum_translation_m;
      balls.center[5]=previous_.reachable_translation;
      balls.radius[5]=config.projection_maximum_speed_m_s*dt;
      balls.count=has_history?6:5;
      const auto reconstructed=balls.solve(64);
      if((reconstructed-built.reachable_translation).norm()>1e-12)
        throw std::runtime_error("mapping projection reconstruction disagrees with builder");
      if(built.detail=="ReachableProjectionInfeasible") {
        ++rejected_;
        const auto extended=balls.solve(4096);
        auto no_speed=balls;no_speed.count=5;
        const auto without_speed=no_speed.solve(4096);
        double largest_gap=0;int first=-1,second=-1;
        for(int i=0;i<balls.count;++i)for(int j=i+1;j<balls.count;++j) {
          const double gap=(balls.center[i]-balls.center[j]).norm()-balls.radius[i]-balls.radius[j];
          if(gap>largest_gap){largest_gap=gap;first=i;second=j;}
        }
        std::cout<<"projection_rejection t_ns="<<built.receive_monotonic_ns-origin<<" sequence="<<built.sequence
          <<" history="<<has_history<<" source_dt_s="<<dt<<" step_bound_m="<<balls.radius[5]
          <<" residual_64_m="<<balls.maximumResidual(reconstructed)
          <<" residual_4096_m="<<balls.maximumResidual(extended)
          <<" no_speed_residual_4096_m="<<no_speed.maximumResidual(without_speed)
          <<" pair_disjoint_gap_m="<<largest_gap<<" pair_a="<<(first<0?"none":balls.names[first])
          <<" pair_b="<<(second<0?"none":balls.names[second])
          <<" classification="<<(largest_gap>1e-10?"ProvenDisjointConstraintBalls":
              balls.maximumResidual(extended)<=1e-10?"FiniteIterationBudget":"CauseUndetermined")
          <<" diagnostic_only=true\n";
        for(int i=0;i<balls.count;++i)
          std::cout<<"projection_constraint sequence="<<built.sequence<<" name="<<balls.names[i]
            <<" center_x="<<balls.center[i].x()<<" center_y="<<balls.center[i].y()<<" center_z="<<balls.center[i].z()
            <<" radius_m="<<balls.radius[i]<<" residual_64_m="<<balls.residual(reconstructed,i)<<'\n';
      }
    }
    if(!built.valid)return;
    if(!has_history)for(auto& branch:branches_)branch={};
    const bool projected=built.reachable_translation.norm()>1e-12;
    const bool was_projected=previous_.reachable_translation.norm()>1e-12;
    if(has_history) {
      ++continuous_pairs_;
      const double step=(built.reachable_translation-previous_.reachable_translation).norm();
      maximum_speed_=std::max(maximum_speed_,step/dt);
      if(step>config.projection_maximum_speed_m_s*dt+1e-10)
        throw std::runtime_error("projection transition violates correction speed bound");
      if((projected||was_projected)&&built.intent_evidence_valid)
        throw std::runtime_error("projection transition leaked uncorrected intent");
      if(projected!=was_projected) {
        projected?++entries_:++exits_;
        std::cout<<"projection_transition t_ns="<<built.receive_monotonic_ns-origin<<" sequence="<<built.sequence
          <<" event="<<(projected?"enter":"exit")<<" correction_step_m="<<step
          <<" source_dt_s="<<dt<<" intent_valid="<<built.intent_evidence_valid<<'\n';
      }
    } else ++history_resets_;
    if(previous_.valid)for(int layer=0;layer<2;++layer)for(int side=0;side<2;++side) {
      const auto& targets=layer?built.filtered:built.raw;
      const auto& prior=layer?previous_.filtered:previous_.raw;
      const auto& arm=side?targets.right:targets.left;
      const auto& old=side?prior.right:prior.left;
      const double palm_step=(arm.palm.position-old.palm.position).norm();
      const double elbow_step=(arm.elbow-old.elbow).norm();
      auto& metric=has_history?continuous_[2*layer+side]:reset_[2*layer+side];
      ++metric.pairs;metric.palm=std::max(metric.palm,palm_step);metric.elbow=std::max(metric.elbow,elbow_step);
      if(!has_history)continue; // Reset gaps are not silently called continuous.
      const auto& g=(*config.closure_geometry)[side];
      const Eigen::Vector3d sw=arm.wrist-arm.shoulder;
      const double d=sw.norm();
      if(d<1e-9){++metric.degenerate;continue;}
      const Eigen::Vector3d normal=sw/d;
      const double a=(g.upper_length_m*g.upper_length_m-g.forearm_length_m*g.forearm_length_m+d*d)/(2*d);
      const Eigen::Vector3d center=arm.shoulder+a*normal;
      const Eigen::Vector3d radial=arm.elbow-center;
      const Eigen::Vector3d old_sw=old.wrist-old.shoulder;
      const double old_d=old_sw.norm();
      if(old_d<1e-9){++metric.degenerate;continue;}
      const Eigen::Vector3d old_normal=old_sw/old_d;
      const double old_a=(g.upper_length_m*g.upper_length_m-g.forearm_length_m*g.forearm_length_m+old_d*old_d)/(2*old_d);
      const Eigen::Vector3d old_radial=old.elbow-old.shoulder-old_a*old_normal;
      // A diagnostic resolution only; not a new production branch threshold.
      if(radial.norm()<1e-6||old_radial.norm()<1e-6||old_normal.dot(normal)<-1+1e-8){++metric.degenerate;continue;}
      // Compare radial directions after minimal transport of the circle plane;
      // translating the old elbow into a new circle can invent a direction at
      // exact extension, where the old branch was actually undefined.
      const Eigen::Vector3d transported=Eigen::Quaterniond::FromTwoVectors(old_normal,normal)*old_radial;
      const double cosine=radial.normalized().dot(transported.normalized());
      metric.minimum_cosine=std::min(metric.minimum_cosine,cosine);
      if(cosine<0) {
        ++metric.opposed;
        std::cout<<"elbow_direction_event t_ns="<<built.receive_monotonic_ns-origin<<" sequence="<<built.sequence
          <<" layer="<<(layer?"filtered":"raw")<<" side="<<(side?"right":"left")
          <<" transported_previous_cosine="<<cosine<<" elbow_step_m="<<elbow_step
          <<" circle_radius_m="<<radial.norm()<<" previous_circle_radius_m="<<old_radial.norm()
          <<" classification=NeedsReviewNotProvenFlip\n";
      }
    }
    // Also compare the last defined radial direction before a degenerate run
    // with the first defined one after it. Never bridge a builder reset.
    for(int layer=0;layer<2;++layer)for(int side=0;side<2;++side) {
      const int i=2*layer+side;auto& branch=branches_[i];
      const auto& targets=layer?built.filtered:built.raw;
      const auto& arm=side?targets.right:targets.left;
      const auto& g=(*config.closure_geometry)[side];
      const Eigen::Vector3d sw=arm.wrist-arm.shoulder;const double d=sw.norm();
      if(d<1e-9){++branch.gap;continue;}
      const Eigen::Vector3d normal=sw/d;
      const double a=(g.upper_length_m*g.upper_length_m-g.forearm_length_m*g.forearm_length_m+d*d)/(2*d);
      const Eigen::Vector3d radial=arm.elbow-arm.shoulder-a*normal;
      if(radial.norm()<1e-6){++branch.gap;continue;}
      if(branch.defined&&branch.gap) {
        if(branch.normal.dot(normal)<-1+1e-8)++unresolved_spans_;
        else {
          const Eigen::Vector3d transported=Eigen::Quaterniond::FromTwoVectors(branch.normal,normal)*branch.radial;
          const double cosine=radial.normalized().dot(transported.normalized());
          ++degenerate_spans_;minimum_span_cosine_=std::min(minimum_span_cosine_,cosine);
          if(cosine<0) {
            ++opposed_spans_;
            std::cout<<"elbow_degenerate_span t_ns="<<built.receive_monotonic_ns-origin<<" sequence="<<built.sequence
              <<" layer="<<(layer?"filtered":"raw")<<" side="<<(side?"right":"left")
              <<" degenerate_frames="<<branch.gap<<" transported_cosine="<<cosine
              <<" classification=NeedsReviewNotProvenDiscontinuity\n";
          }
        }
      }
      branch.defined=true;branch.gap=0;branch.normal=normal;branch.radial=radial;
    }
    previous_=built;
  }
  void report() const {
    std::cout<<"projection_transition_summary continuous_pairs="<<continuous_pairs_<<" history_starts="<<history_resets_
      <<" entries="<<entries_<<" exits="<<exits_<<" maximum_correction_speed_m_s="<<maximum_speed_
      <<" projection_rejections="<<rejected_<<" scope=standalone_mapping_not_control_recovery\n";
    std::cout<<"elbow_degenerate_span_summary compared="<<degenerate_spans_<<" opposed="<<opposed_spans_
      <<" unresolved="<<unresolved_spans_<<" minimum_transported_cosine="<<minimum_span_cosine_
      <<" scope=defined_endpoints_across_degeneracy_not_motion_safety\n";
    for(int reset=0;reset<2;++reset)for(int i=0;i<4;++i) {
      const auto& m=reset?reset_[i]:continuous_[i];
      std::cout<<"mapping_continuity history="<<(reset?"reset_gap":"continuous")
        <<" layer="<<(i/2?"filtered":"raw")<<" side="<<(i%2?"right":"left")
        <<" pairs="<<m.pairs<<" max_palm_step_m="<<m.palm<<" max_elbow_step_m="<<m.elbow
        <<" degenerate_comparisons="<<m.degenerate<<" opposed_direction_events="<<m.opposed
        <<" minimum_transported_previous_cosine="<<m.minimum_cosine<<'\n';
    }
  }
 private:
  struct Metric {std::size_t pairs{0},degenerate{0},opposed{0};double palm{0},elbow{0},minimum_cosine{1};};
  std::array<Metric,4> continuous_,reset_;
  struct Branch {
    bool defined{false};std::size_t gap{0};
    Eigen::Vector3d normal{Eigen::Vector3d::Zero()},radial{Eigen::Vector3d::Zero()};
  };
  std::array<Branch,4> branches_;
  SharedRootBuiltTargets previous_;
  std::size_t continuous_pairs_{0},history_resets_{0},entries_{0},exits_{0},rejected_{0};
  double maximum_speed_{0};
  std::size_t degenerate_spans_{0},opposed_spans_{0},unresolved_spans_{0};
  double minimum_span_cosine_{1};
};
} // namespace tianji_qp_ik
