// Offline, per-source-frame IK diagnostic. No sockets, mj_step, or executor.
#include "tianji_qp_ik/spark_guidance.hpp"
#include "tianji_qp_ik/so3.hpp"
#include "tianji_qp_ik/controller.hpp"
#include "tianji_qp_ik/shared_root_closure.hpp"
#include "shared_root_coverage.hpp"
#include "shared_root_mapping_probe.hpp"
#include <chrono>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <limits>
#include <map>
#include <set>
#include <vector>

using namespace tianji_qp_ik;
#include "shared_root_ceres_probe.hpp"
namespace {
std::uint64_t le(const unsigned char* b,std::size_t n) {
  std::uint64_t v=0;for(std::size_t i=0;i<n;++i)v|=std::uint64_t(b[i])<<(8*i);return v;
}
std::vector<PicoTeleopFrame> read(const std::string& path) {
  std::ifstream in(path,std::ios::binary); unsigned char h[16]{};
  if(!in.read(reinterpret_cast<char*>(h),16)||std::string(reinterpret_cast<char*>(h),4)!="TJVT"||
     le(h+4,2)!=1||le(h+6,2)!=656)throw std::runtime_error("invalid TJVT v1/v4 header");
  const auto count=le(h+8,8);
  if(count==0||count>1000000)throw std::runtime_error("empty or excessive trace");
  std::vector<PicoTeleopFrame> frames;frames.reserve(static_cast<std::size_t>(count));
  std::set<std::pair<std::uint64_t,std::uint64_t>> source_ids;
  std::uint64_t last=0;
  for(std::uint64_t i=0;i<count;++i) {
    unsigned char b[664]{};
    if(!in.read(reinterpret_cast<char*>(b),664))throw std::runtime_error("truncated trace");
    auto decoded=decodePicoTeleopPacket(b+8,656);
    if(!decoded.frame)throw std::runtime_error("packet rejected by native protocol decoder");
    if(!source_ids.emplace(decoded.frame->tracking_epoch,decoded.frame->sequence).second)
      throw std::runtime_error("duplicate epoch/sequence in offline audit");
    const auto elapsed=le(b,8);
    if((i&&elapsed<last)||elapsed>86400000000000ULL)throw std::runtime_error("invalid receive timeline");
    last=elapsed;decoded.frame->receive_monotonic_ns=1000000000+static_cast<std::int64_t>(elapsed);
    frames.push_back(*decoded.frame);
  }
  if(in.peek()!=std::char_traits<char>::eof())throw std::runtime_error("trailing trace data");
  return frames;
}
double p90(std::vector<double> v) {
  if(v.empty())return std::numeric_limits<double>::infinity();
  std::sort(v.begin(),v.end());return v[static_cast<std::size_t>(std::ceil(.9*static_cast<double>(v.size())))-1];
}
const char* stateName(SharedRootState state) {
  switch(state) {
    case SharedRootState::kUninitialized:return "uninitialized";
    case SharedRootState::kTracking:return "tracking";
    case SharedRootState::kHoldLastMapped:return "hold_last_mapped";
    case SharedRootState::kRecovering:return "recovering";
    case SharedRootState::kInvalid:return "invalid";
  }
  throw std::runtime_error("unknown shared root state");
}
struct ConflictSample {
  SparkUpperArmTarget target;
  Vec7 seed;
  ArmSide side;
  std::uint64_t sequence;
  std::size_t cycle;
  std::int64_t t_ns;
  double original_error;
};
void taskConflictAblation(const std::vector<ConflictSample>& samples,
    MujocoRobot& robot,const SharedRootOptions& options,const QpIkConfig& config) {
  PinocchioArmKinematics kinematics(options.urdf_path,
    {robot.tcpRelativeToLink7(ArmSide::kLeft),robot.tcpRelativeToLink7(ArmSide::kRight)});
  const char* variants[]={"baseline","shape_soft","pose_only","position_only",
    "iterations_only","precision_only","precision_iterations"};
  for(const auto& sample:samples)for(int variant=0;variant<7;++variant) {
    auto cfg=config.spark_upper_qpoases;
    if(variant>=1&&variant<=3) {
      const double factor=variant==1?.01:0.;
      cfg.stage1_upper_direction_weight*=factor;cfg.stage1_forearm_direction_weight*=factor;
      cfg.stage2_upper_direction_weight*=factor;cfg.stage2_forearm_direction_weight*=factor;
      cfg.stage2_elbow_position_weight*=factor;cfg.stage2_wrist_position_weight*=factor;
    }
    if(variant==3)cfg.stage1_palm_orientation_weight=cfg.stage2_palm_orientation_weight=0;
    if(variant==4||variant==6)cfg.stage1_max_iterations=cfg.stage2_max_iterations=100;
    if(variant==5||variant==6)cfg.convergence_delta=1e-8;
    const auto& limits=robot.mapping(sample.side).limits;
    SparkUpperQpoasesIk7 solver(sample.side,kinematics,limits,cfg,config.qpoases);
    // Each variant starts cold from the SAME recorded solution, with the SAME
    // complete target and limits. This is not a controller replay or live retry.
    const auto r=solver.solve(sample.target,sample.seed);
    const auto pose=robot.armKinematicsAt(sample.side,r.q).tcp_pose;
    const double pe=(pose.position-sample.target.palm.position).norm();
    const double re=rotationDistance(pose.rotation,sample.target.palm.rotation);
    const double clearance=std::min((r.q-limits.lower_position).minCoeff(),
      (limits.upper_position-r.q).minCoeff())-cfg.joint_limit_margin_rad;
    if(!r.q.allFinite()||!std::isfinite(pe)||!std::isfinite(re)||clearance< -1e-9)
      throw std::runtime_error("task conflict diagnostic nonfinite or unsafe result");
    if(r.accepted&&std::abs(pe-r.palm_position_error)>1e-5)
      throw std::runtime_error("task conflict diagnostic target/result mismatch");
    std::cout<<"task_ablation mode=1 cycle="<<sample.cycle<<" t_ns="<<sample.t_ns
      <<" sequence="<<sample.sequence<<" side="<<(sample.side==ArmSide::kLeft?"left":"right")
      <<" variant="<<variants[variant]<<" original_error_m="<<sample.original_error
      <<" accepted="<<r.accepted<<" position_error_m="<<pe<<" orientation_error_rad="<<re
      <<" safe_limit_clearance_rad="<<clearance<<" stop="<<r.detail
      <<" stage1_iterations="<<r.stage1_iterations<<" stage2_iterations="<<r.stage2_iterations
      <<" stage1_limit="<<cfg.stage1_max_iterations<<" stage2_limit="<<cfg.stage2_max_iterations
      <<" convergence_delta="<<cfg.convergence_delta<<" budget_exhausted="<<r.budget_exhausted
      <<" q_distance_rad="<<(r.q-sample.seed).norm()
      <<" seed=recorded_solution cold_solver=true deadline=unbounded scope=isolated_target_not_control_replay\n";
  }
}
void scaleAblation(const std::vector<PicoTeleopFrame>& frames,
                   const SharedRootOptions& options,const QpIkConfig& config) {
  MujocoRobot robot(options.mujoco_path);
  PinocchioArmKinematics kinematics(options.urdf_path,
      {robot.tcpRelativeToLink7(ArmSide::kLeft),robot.tcpRelativeToLink7(ArmSide::kRight)});
  auto bc=options.builder;bc.shape_config=config.spark_upper_qpoases;
  SharedRootTargetBuilder anisotropic(bc),isotropic(bc);
  SharedRootMorphologyEstimator morphology(options.morphology);
  std::array<std::unique_ptr<SparkUpperQpoasesIk7>,4> solvers;
  std::array<Vec7,4> seeds;
  struct Metrics {
    std::vector<double> position,orientation,relation,shape;
    std::size_t rejected{0},budget{0},near_limit{0};
  } metrics[2];
  const auto reset=[&] {
    morphology.reset();anisotropic.reset();isotropic.reset();
    for(std::size_t i=0;i<4;++i) {
      const auto side=i%2==0?ArmSide::kLeft:ArmSide::kRight;
      const auto& limits=robot.mapping(side).limits;
      seeds[i]=.5*(limits.lower_position+limits.upper_position);
      solvers[i]=std::make_unique<SparkUpperQpoasesIk7>(side,kinematics,limits,
          config.spark_upper_qpoases,config.qpoases);
    }
  };
  reset();std::size_t excluded=0,paired=0;
  std::uint64_t epoch=0,generation=0;
  std::int64_t source=0;
  for(const auto& f:frames) {
    if(f.tracking_epoch!=epoch||f.resynchronization_generation!=generation||f.stream_discontinuity) {
      reset();source=0;epoch=f.tracking_epoch;generation=f.resynchronization_generation;
    }
    if(source&&f.source_timestamp_ns<=source)
      throw std::runtime_error("scale ablation requires unique ordered source timestamps");
    if(source&&double(f.source_timestamp_ns-source)*1e-9>bc.maximum_source_dt_s) {
      anisotropic.beginRecovery();isotropic.beginRecovery();
    }
    source=f.source_timestamp_ns;
    const auto input=TjvrSharedRootInputAdapter{}.adapt(f);
    auto m=morphology.update(input);auto uniform=m;uniform.lateral_scale=m.reach_scale;
    const std::array<SharedRootBuiltTargets,2> targets{
      anisotropic.update(input,m),isotropic.update(input,uniform)};
    if(!targets[0].valid||!targets[1].valid) {
      ++excluded;anisotropic.beginRecovery();isotropic.beginRecovery();continue;
    }
    std::array<SparkUpperIkResult,4> results;
    bool all=true;
    for(std::size_t mode=0;mode<2;++mode) {
      const auto& t=targets[mode].filtered;
      for(std::size_t side_index=0;side_index<2;++side_index) {
        const auto i=mode*2+side_index;
        auto& r=results[i];
        r=solvers[i]->solve(side_index==0?t.left:t.right,seeds[i],
            std::chrono::steady_clock::now()+std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(config.spark_upper_qpoases.ik_cycle_budget_seconds)));
        metrics[mode].budget+=r.budget_exhausted;
        metrics[mode].rejected+=!r.accepted;all=all&&r.accepted;
        if(!r.accepted)continue;
        const auto& limits=robot.mapping(side_index==0?ArmSide::kLeft:ArmSide::kRight).limits;
        const double margin=config.spark_upper_qpoases.joint_limit_margin_rad;
        if(!r.q.allFinite()||(r.q.array()<limits.lower_position.array()+margin-1e-9).any()||
            (r.q.array()>limits.upper_position.array()-margin+1e-9).any())
          throw std::runtime_error("scale ablation IK violated safe joint limits");
        seeds[i]=r.q;
      }
    }
    if(!all){++excluded;continue;} // Error comparisons use the same frame set.
    ++paired;
    for(std::size_t mode=0;mode<2;++mode) {
      Pose poses[2];const auto& t=targets[mode].filtered;auto& stat=metrics[mode];
      for(std::size_t j=0;j<2;++j) {
        const auto side=j==0?ArmSide::kLeft:ArmSide::kRight;
        const auto& r=results[mode*2+j];const auto& target=j==0?t.left:t.right;
        poses[j]=kinematics.sample(side,r.q).tcp_pose;
        stat.position.push_back((poses[j].position-target.palm.position).norm());
        stat.orientation.push_back(rotationDistance(poses[j].rotation,target.palm.rotation));
        stat.shape.push_back(std::hypot(r.upper_direction_error,r.forearm_direction_error));
        const auto& limits=robot.mapping(side).limits;
        stat.near_limit+=std::min((r.q-limits.lower_position).minCoeff(),
          (limits.upper_position-r.q).minCoeff())-config.spark_upper_qpoases.joint_limit_margin_rad<.01;
      }
      stat.relation.push_back(((poses[1].position-poses[0].position)-
          (t.right.palm.position-t.left.palm.position)).norm());
    }
  }
  for(int mode=0;mode<2;++mode) {
    const auto& m=metrics[mode];
    std::cout<<"scale_ablation scale="<<(mode?"isotropic":"anisotropic")<<" frames="<<frames.size()
      <<" paired_frames="<<paired<<" excluded_frames="<<excluded<<" rejected_sides="<<m.rejected
      <<" budget_exhausted_sides="<<m.budget<<" near_safe_limit_sides="<<m.near_limit
      <<" palm_p90_m="<<p90(m.position)<<" orientation_p90_rad="<<p90(m.orientation)
      <<" B_relation_p90_m="<<p90(m.relation)<<" shape_error_norm_p90="<<p90(m.shape)
      <<" scope=paired_filtered_target_pose_IK actual=unavailable classification=CauseUndetermined\n";
  }
}
}
int main(int argc,char** argv) {
  try {
    if(argc!=3 && !(argc==4&&(std::string(argv[3])=="--ceres-reference-loop"||std::string(argv[3])=="--reference-loop"||
        std::string(argv[3])=="--reference-loop-200hz"||std::string(argv[3])=="--mapping-transitions"||std::string(argv[3])=="--layer-diagnostics"||std::string(argv[3])=="--worst-ik-multiseed"||std::string(argv[3])=="--scale-ablation"||std::string(argv[3])=="--mapping-only"||std::string(argv[3])=="--mapping-frames")))
      throw std::runtime_error("usage: tianji_shared_root_trace_audit PROFILE TRACE [--reference-loop|--reference-loop-200hz|--mapping-transitions|--layer-diagnostics|--worst-ik-multiseed|--scale-ablation|--mapping-only|--mapping-frames]");
    const bool mapping_frames=argc==4&&std::string(argv[3])=="--mapping-frames";
    const bool multiseed=argc==4&&std::string(argv[3])=="--worst-ik-multiseed";
    const bool layers=argc==4&&std::string(argv[3])=="--layer-diagnostics";
    const bool transitions=argc==4&&std::string(argv[3])=="--mapping-transitions";
    if(transitions)std::cout<<std::setprecision(17);
    const bool fixed_ticks=layers||multiseed||(argc==4&&std::string(argv[3])=="--reference-loop-200hz");
    const bool reference_loop=fixed_ticks||(argc==4&&std::string(argv[3])=="--reference-loop");
    const auto fingerprint=sharedRootSha256File(argv[2]);
    const auto frames=read(argv[2]);const auto options=loadSharedRootOptions(argv[1]);
    const auto baseline=loadConfig(argv[1]);
    if(argc==4&&std::string(argv[3])=="--ceres-reference-loop") {
      ceresReferenceProbe(frames,options,baseline);
      if(sharedRootSha256File(argv[2])!=fingerprint||sharedRootSha256File(argv[1])!=options.profile_sha256)
        throw std::runtime_error("Ceres probe inputs changed during run");
      std::cout<<"trace_sha256="<<fingerprint<<" profile_sha256="<<options.profile_sha256<<'\n';
      return 0;
    }
    if(usesSharedRootDirectIk(baseline.ik_algorithm) && !mapping_frames &&
       !(argc==4&&std::string(argv[3])=="--mapping-only"))
      throw std::runtime_error("Direct IK profile requires online Viewer or mapping-only diagnostics; Ceres also supports --ceres-reference-loop");
    if(layers)std::cout<<std::setprecision(17)<<"layer_contract schema=1 control_dt_ns=5000000 duration_ns="
      <<frames.back().receive_monotonic_ns-frames.front().receive_monotonic_ns
      <<" source_frames="<<frames.size()<<" time_basis=nanoseconds_since_first_receive\n";
    if(argc==4&&std::string(argv[3])=="--scale-ablation") {
      scaleAblation(frames,options,baseline);
      if(sharedRootSha256File(argv[2])!=fingerprint||sharedRootSha256File(argv[1])!=options.profile_sha256)
        throw std::runtime_error("scale ablation inputs changed during run");
      std::cout<<"trace_sha256="<<fingerprint<<" profile_sha256="<<options.profile_sha256
        <<" motion_authorized=false phase_a_accepted=false\n";
      return 0;
    }
    {
      auto o=options;o.builder.shape_config=baseline.spark_upper_qpoases;
      SharedRootPipeline probe(o);std::map<std::string,std::size_t> reasons;
      // Independent observe-only replay of estimator/builder for this trace.
      // Abort on disagreement; never infer candidate scale from rolled-back state.
      SharedRootMorphologyEstimator estimator(o.morphology);
      SharedRootTargetBuilder builder(o.builder);
      MappingTransitionProbe transition_probe;
      std::size_t candidate_center_reject=0;
      double candidate_max_relation=0;
      bool interrupted=true;
      std::uint64_t probe_epoch=0,probe_generation=0,probe_sequence=0;
      std::int64_t probe_source=0,probe_receive=0;
      double max_center=0,max_relation=0;std::size_t center_reject=0;
      std::size_t mapping_valid=0;
      std::size_t projected_frames=0;
      double maximum_projection_m=0;
      double relation_residual=0,center_residual=0,bone_residual=0,shoulder_residual=0,orientation_residual=0;
      std::vector<double> palm_shape_gap,filtered_palm_shape_gap,filter_displacement;
      std::array<std::size_t,2> closure_pairs{},closure_workspace{},closure_branch{};
      double closure_bone_error=0,closure_endpoint_error=0;
      SharedRootCoverageTimeline coverage(static_cast<std::int64_t>(o.continuity.freshness_s*1e9));
      std::size_t outside_frames=0,closure_unevaluated=0;
      std::array<std::map<int,std::size_t>,4> side_closure_status;
      for(const auto& f:frames) {
        if(!probe.observe(f,f.receive_monotonic_ns))
          ++reasons[std::string(probe.candidate().detail)+"/"+std::string(probe.morphology().detail)];
        const auto input=TjvrSharedRootInputAdapter{}.adapt(f);
        if(f.tracking_epoch!=probe_epoch||f.resynchronization_generation!=probe_generation||f.stream_discontinuity) {
          estimator.reset();builder.reset();interrupted=true;probe_source=probe_receive=0;
        }
        if(probe_source&&(f.sequence<=probe_sequence||f.source_timestamp_ns<=probe_source||f.receive_monotonic_ns<probe_receive))
          throw std::runtime_error("candidate audit requires ordered unique source frames");
        if(probe_source&&(double(f.source_timestamp_ns-probe_source)*1e-9>o.continuity.maximum_source_gap_s||
            double(f.receive_monotonic_ns-probe_receive)*1e-9>o.continuity.maximum_receive_gap_s)) {
          if(!interrupted)builder.beginRecovery();
          interrupted=true;
        }
        probe_epoch=f.tracking_epoch;probe_generation=f.resynchronization_generation;
        probe_source=f.source_timestamp_ns;probe_receive=f.receive_monotonic_ns;probe_sequence=f.sequence;
        const auto old_estimator=estimator;
        const auto candidate_scale=estimator.update(input);
        if(!candidate_scale.valid||!candidate_scale.new_sample)
          throw std::runtime_error("candidate audit does not support morphology invalid/rebuild context");
        const auto built=builder.update(input,candidate_scale);
        if(transitions)transition_probe.observe(built,o.builder,!interrupted,frames.front().receive_monotonic_ns);
        if(layers) {
          std::cout<<"projection_frame t_ns="<<f.receive_monotonic_ns-frames.front().receive_monotonic_ns
            <<" sequence="<<f.sequence<<" valid="<<built.valid<<" shift_m=";
          if(built.valid)std::cout<<built.reachable_translation.norm();else std::cout<<"NA";
          std::cout<<" reason="<<built.detail<<'\n';
        }
        coverage.observe(f.receive_monotonic_ns,built.valid);
        // Evaluate both sides even when raw closure short-circuited the builder.
        // These counts are diagnostic; never commit a branch or alter targets.
        bool any_outside=false;
        if(built.raw_preference.valid&&built.filtered_preference.valid) {
          for(int side=0;side<2;++side) {
            for(int layer=0;layer<2;++layer) {
              const auto& preference=layer?built.filtered_preference:built.raw_preference;
              const auto& t=side?preference.right:preference.left;
              const auto c=closeSharedRootArm(o.closure_geometry[side],t.palm,t.elbow);
              ++side_closure_status[2*layer+side][static_cast<int>(c.status)];
              any_outside=any_outside||c.status==ClosureStatus::kOutsideWorkspace;
            }
          }
        } else ++closure_unevaluated;
        outside_frames+=any_outside;
        if(built.valid!=probe.candidate().valid||built.detail!=probe.candidate().detail)
          throw std::runtime_error("candidate audit disagrees with pipeline");
        if(mapping_frames) {
          // Read-only visualization of FILTERED mapping, before guidance blend or IK.
          std::cout<<std::setprecision(17)<<"mapping_frame "<<f.receive_monotonic_ns<<' '<<f.sequence<<' '<<built.valid;
          if(built.valid)for(const auto* t:{&built.filtered.left,&built.filtered.right}) {
            for(const auto& p:{t->shoulder,t->elbow,t->wrist,t->hand,t->palm.position})
              for(int j=0;j<3;++j)std::cout<<' '<<p[j];
            for(int i=0;i<3;++i)for(int j=0;j<3;++j)std::cout<<' '<<t->palm.rotation(i,j);
          }
          std::cout<<'\n';
        }
        if(built.valid) {
          ++mapping_valid;
          projected_frames+=built.reachable_translation.norm()>1e-12;
          maximum_projection_m=std::max(maximum_projection_m,built.reachable_translation.norm());
          for(int layer=0;layer<2;++layer) {
            const auto& goals=layer?built.filtered:built.raw;
            bool valid_pair=true,outside=false,branch=false;
            for(int side=0;side<2;++side) {
              const auto& goal=side?goals.right:goals.left;
              // Independent geometric coverage diagnostic: no accepted control
              // history is invented, and these results do not alter live targets.
              const auto closed=closeSharedRootArm(o.closure_geometry[side],goal.palm,goal.elbow);
              valid_pair=valid_pair&&closed.valid();
              outside=outside||closed.status==ClosureStatus::kOutsideWorkspace;
              branch=branch||closed.status==ClosureStatus::kBranchUndetermined;
              if(closed.valid()) {
                const auto& t=closed.target;const auto& cg=o.closure_geometry[side];
                closure_endpoint_error=std::max(closure_endpoint_error,(t.hand-t.palm.position).norm());
                closure_bone_error=std::max({closure_bone_error,
                  std::abs((t.elbow-t.shoulder).norm()-cg.upper_length_m),
                  std::abs((t.wrist-t.elbow).norm()-cg.forearm_length_m)});
              }
            }
            closure_pairs[layer]+=valid_pair;closure_workspace[layer]+=outside;closure_branch[layer]+=branch;
          }
          const Eigen::Vector3d scale(candidate_scale.reach_scale,candidate_scale.lateral_scale,candidate_scale.reach_scale);
          const auto& l=built.raw.left;const auto& r=built.raw.right;
          const Eigen::Vector3d expected_relation=o.builder.R_BCt*scale.cwiseProduct(input.right.p_control_root_Ct-input.left.p_control_root_Ct);
          const Eigen::Vector3d expected_center=o.builder.o_B+o.builder.R_BCt*scale.cwiseProduct(.5*(input.left.p_control_root_Ct+input.right.p_control_root_Ct));
          relation_residual=std::max(relation_residual,((r.palm.position-l.palm.position)-expected_relation).norm());
          center_residual=std::max(center_residual,(.5*(r.palm.position+l.palm.position)-built.reachable_translation-expected_center).norm());
          for(int side=0;side<2;++side) {
            const auto& raw=side?built.raw.right:built.raw.left;
            const auto& filtered=side?built.filtered.right:built.filtered.left;
            const auto& src=side?input.right:input.left;
            const auto& geo=o.builder.geometry;
            const Eigen::Vector3d shoulder=side?geo.right_shoulder:geo.left_shoulder;
            const double upper=(side?geo.right_upper_arm_local:geo.left_upper_arm_local).norm();
            const double forearm=(side?geo.right_forearm_local:geo.left_forearm_local).norm();
            const double hand=(side?geo.right_wrist_to_palm_local:geo.left_wrist_to_palm_local).norm();
            for(const auto* t:{&raw,&filtered}) {
              shoulder_residual=std::max(shoulder_residual,(t->shoulder-shoulder).norm());
              bone_residual=std::max({bone_residual,std::abs((t->elbow-t->shoulder).norm()-upper),
                  std::abs((t->wrist-t->elbow).norm()-forearm),std::abs((t->hand-t->wrist).norm()-hand)});
            }
            orientation_residual=std::max(orientation_residual,(raw.palm.rotation-o.builder.R_BCt*src.R_palm_Ct).norm());
            palm_shape_gap.push_back((raw.palm.position-raw.hand).norm());
            filtered_palm_shape_gap.push_back((filtered.palm.position-filtered.hand).norm());
            filter_displacement.push_back((filtered.palm.position-raw.palm.position).norm());
          }
        }
        if(built.detail=="target_gate") {
          const Eigen::Vector3d s(candidate_scale.reach_scale,candidate_scale.lateral_scale,candidate_scale.reach_scale);
          const double c=s.cwiseProduct(.5*(input.left.p_control_root_Ct+input.right.p_control_root_Ct)).norm();
          const double r=s.cwiseProduct(input.right.p_control_root_Ct-input.left.p_control_root_Ct).norm();
          candidate_center_reject+=c>o.builder.maximum_center_m;
          candidate_max_relation=std::max(candidate_max_relation,r);
        }
        if(!built.valid) {estimator=old_estimator;if(!interrupted)builder.beginRecovery();interrupted=true;}
        else interrupted=false;
        const auto& m=probe.morphology();
        if(input.valid&&m.valid) {
          const Eigen::Vector3d scale(m.reach_scale,m.lateral_scale,m.reach_scale);
          const double center=scale.cwiseProduct(.5*(input.left.p_control_root_Ct+input.right.p_control_root_Ct)).norm();
          const double relation=scale.cwiseProduct(input.right.p_control_root_Ct-input.left.p_control_root_Ct).norm();
          max_center=std::max(center,max_center);max_relation=std::max(relation,max_relation);
          center_reject+=center>o.builder.maximum_center_m;
        }
      }
      for(const auto& [reason,count]:reasons)
        std::cout<<"mapping_only_rejection reason="<<reason<<" frames="<<count<<'\n';
      std::cout<<"reachable_projection enabled="<<o.builder.reachable_projection_enabled
        <<" corrected_frames="<<projected_frames<<" maximum_translation_m="<<maximum_projection_m
        <<" original_affine_center_preserved="<<(!o.builder.reachable_projection_enabled)
        <<" scope=common_translation_before_closure_not_joint_feasibility\n";
      std::cout<<"committed_scale_gate_probe max_center_m="<<max_center<<" max_relation_m="<<max_relation
        <<" center_over_limit="<<center_reject<<" inter_palm_distance_gate=disabled"<<'\n';
      std::cout<<"candidate_gate_probe center_rejected="<<candidate_center_reject
        <<" max_rejected_relation_m="<<candidate_max_relation<<'\n';
      std::cout<<"mapping_geometry frames="<<frames.size()<<" valid="<<mapping_valid
        <<" rejected="<<(frames.size()-mapping_valid)<<" raw_relation_max_m="<<relation_residual
        <<" raw_center_max_m="<<center_residual<<" raw_filtered_bone_max_m="<<bone_residual
        <<" raw_filtered_shoulder_max_m="<<shoulder_residual<<" raw_orientation_matrix_max="<<orientation_residual
        <<" raw_palm_shape_gap_p90_m="<<p90(palm_shape_gap)
        <<" filtered_palm_shape_gap_p90_m="<<p90(filtered_palm_shape_gap)
        <<" filter_displacement_p90_m="<<p90(filter_displacement)<<'\n';
      const auto timeline=coverage.through(frames.back().receive_monotonic_ns);
      if(transitions)transition_probe.report();
      std::cout<<"mapping_coverage evaluated_unique_frames="<<frames.size()
        <<" geometric_target_valid_ratio="<<double(mapping_valid)/double(frames.size())
        <<" mapped_palm_outside_workspace_ratio="<<double(outside_frames)/double(frames.size())
        <<" closure_unevaluated="<<closure_unevaluated
        <<" duration_s="<<double(timeline.duration())*1e-9
        <<" invalid_duration_s="<<double(timeline.invalidDuration())*1e-9
        <<" maximum_continuous_invalid_duration_s="<<double(timeline.maximumInvalidDuration())*1e-9
        <<" freshness_s="<<o.continuity.freshness_s
        <<" action=unannotated per_action_acceptance=unavailable"
        <<" window=first_to_last_receive eof_tail=unobserved"
        <<" duplicate_policy=reject superseded=0 scope=standalone_mapping_not_control_recovery\n";
      for(int layer=0;layer<2;++layer)for(int side=0;side<2;++side)
        for(const auto& [status,count]:side_closure_status[2*layer+side]) {
          const auto code=static_cast<ClosureStatus>(status);
          const char* reason=code==ClosureStatus::kAccepted?"accepted":
            code==ClosureStatus::kOutsideWorkspace?"outside_workspace":
            code==ClosureStatus::kBranchUndetermined?"branch_undetermined":
            code==ClosureStatus::kResidualFailure?"residual_failure":"invalid_input";
          std::cout<<"closure_side layer="<<(layer?"filtered":"raw")<<" side="<<(side?"right":"left")
            <<" reason="<<reason<<" frames="<<count<<'\n';
        }
      for(int layer=0;layer<2;++layer)
        std::cout<<"closure_candidate layer="<<(layer?"filtered":"raw")<<" evaluated="<<mapping_valid
          <<" valid_pairs="<<closure_pairs[layer]<<" outside_pairs="<<closure_workspace[layer]
          <<" undetermined_pairs="<<closure_branch[layer]<<" max_bone_error_m="<<closure_bone_error
          <<" max_endpoint_error_m="<<closure_endpoint_error
          <<" accepted_history=none scope=standalone_mapping_not_control_replay\n";
      if(mapping_valid==0||relation_residual>1e-9||center_residual>1e-9||bone_residual>1e-9||
          shoulder_residual>1e-9||orientation_residual>1e-9)
        throw std::runtime_error("mapping geometry contract failed");
    }
    if(transitions||mapping_frames||(argc==4&&std::string(argv[3])=="--mapping-only")) {
      if(sharedRootSha256File(argv[2])!=fingerprint||sharedRootSha256File(argv[1])!=options.profile_sha256)
        throw std::runtime_error("mapping audit inputs changed during run");
      std::cout<<"trace_sha256="<<fingerprint<<" profile_sha256="<<options.profile_sha256
        <<" geometry_sha256="<<options.geometry_sha256<<" ik_executed=false motion_authorized=false phase_a_accepted=false\n";
      return 0;
    }
    for(int mode=0;mode<(reference_loop?2:6);++mode) {
      MujocoRobot robot(options.mujoco_path);
      ArmMotionState left,right;
      for(auto side:{ArmSide::kLeft,ArmSide::kRight}) {
        const auto& limits=robot.mapping(side).limits;
        auto& s=side==ArmSide::kLeft?left:right;
        s.q=.5*(limits.lower_position+limits.upper_position);
        robot.setArmPosition(side,s.q);
      }
      robot.forward();
      auto cfg=baseline;auto opt=options;opt.enabled=true;
      if(mode>=2)cfg.spark_upper_qpoases.stage2_elbow_position_weight=
          cfg.spark_upper_qpoases.stage2_wrist_position_weight=.25;
      if(mode>=3)cfg.spark_upper_qpoases.stage2_upper_direction_weight=
          cfg.spark_upper_qpoases.stage2_forearm_direction_weight=.05;
      if(mode>=4)cfg.spark_upper_qpoases.stage2_max_iterations=50;
      if(mode==5)cfg.spark_upper_qpoases.convergence_delta=1e-7;
      DualArmSparkGuidance g(robot,cfg,options.urdf_path,
          SparkPostureGuideMode::kHeadroomFeedforwardJointReferenceVelocity,mode?&opt:nullptr);
      std::unique_ptr<DualArmController> controller;
      if(reference_loop) {
        controller=std::make_unique<DualArmController>(robot,cfg);
        if(!controller->synchronizeReferencesToActual())
          throw std::runtime_error("offline controller initialization rejected");
      }
      std::size_t reference_accepted=0,reference_rejected=0,recovery_confirmed=0,tracking=0;
      std::size_t closed_control_samples=0;
      double closed_control_bone_error=0,closed_control_endpoint_error=0;
      double ik_residual_disagreement=0,ik_vs_command_target_max=0;
      std::map<int,std::size_t> reference_reasons;
      std::vector<double> reference_errors,mapping_times;
      std::vector<double> feedforward_errors,reference_to_feedforward;
      std::map<std::string,std::size_t> headroom_sources;
      std::size_t low_headroom_sides=0,headroom_feedback_cycles=0;
      std::vector<double> position,orientation,relation,relative_rotation;
      struct StopStats {
        std::size_t count{0},over_10mm{0},stage1_cap{0},stage2_cap{0};
        std::vector<double> errors;
      };
      std::map<std::string,StopStats> stops;
      std::map<std::string,std::size_t> exclusions;
      std::size_t large_error_near_limit=0;
      double worst_error=0;std::uint64_t worst_sequence=0;
      std::string worst_side;
      SparkUpperIkResult worst_ik;
      SparkUpperArmTarget worst_target;
      double worst_clearance=0;
      std::size_t worst_cycle=0;
      std::vector<ConflictSample> conflict_samples;
      std::size_t rejected_input=0,excluded=0,budgets=0,accepted=0;
      std::size_t position_over=0,orientation_over=0;
      std::int64_t previous=frames.front().receive_monotonic_ns;
      std::uint64_t epoch=frames.front().tracking_epoch;
      const std::size_t cycles=fixed_ticks?static_cast<std::size_t>(
          (frames.back().receive_monotonic_ns-previous+4999999)/5000000)+1:frames.size();
      std::size_t source_index=0,last_index=frames.size(),delivered=0;
      for(std::size_t cycle=0;cycle<cycles;++cycle) {
        const auto now=fixed_ticks?frames.front().receive_monotonic_ns+
            static_cast<std::int64_t>(cycle)*5000000:frames[cycle].receive_monotonic_ns;
        if(fixed_ticks) {
          while(source_index+1<frames.size()&&frames[source_index+1].receive_monotonic_ns<=now)++source_index;
        } else source_index=cycle;
        const auto f=frames[source_index];
        const bool new_frame=source_index!=last_index;last_index=source_index;
        delivered+=new_frame;
        if(f.tracking_epoch!=epoch) {
          if(!g.reset(left,right))throw std::runtime_error("failed epoch reset");
          if(controller&&!controller->synchronizeReferencesToActual())
            throw std::runtime_error("offline controller epoch reset rejected");
          epoch=f.tracking_epoch;
        }
        const double dt=fixed_ticks?.005:std::clamp(static_cast<double>(now-previous)*1e-9,.001,.1);
        previous=now;
        SparkGuidanceDiagnostics d;
        if(mode==0) {if(new_frame)g.updatePicoFrame(f);d=g.step(left,right,dt);}
        else {
          if(new_frame) {
            const auto mapping_start=std::chrono::steady_clock::now();
            if(!g.updateSharedRootFrame(f,now))++rejected_input;
            mapping_times.push_back(std::chrono::duration<double,std::micro>(
                std::chrono::steady_clock::now()-mapping_start).count());
          }
          d=g.stepSharedRoot(left,right,dt,now,true,true);
        }
        tracking+=mode&&d.shared_root_state==SharedRootState::kTracking;
        if(mode&&d.target_valid) {
          const auto& target=g.latestTargets();
          for(int side=0;side<2;++side) {
            const auto& t=side?target.right:target.left;const auto& geo=options.closure_geometry[side];
            ++closed_control_samples;
            closed_control_endpoint_error=std::max(closed_control_endpoint_error,(t.hand-t.palm.position).norm());
            closed_control_bone_error=std::max({closed_control_bone_error,
              std::abs((t.elbow-t.shoulder).norm()-geo.upper_length_m),
              std::abs((t.wrist-t.elbow).norm()-geo.forearm_length_m)});
          }
          if(closed_control_bone_error>1e-9||closed_control_endpoint_error>1e-9)
            throw std::runtime_error("closed control target invariant failed");
        }
        budgets+=d.left.ik.budget_exhausted||d.right.ik.budget_exhausted;
        if(!d.target_valid||!d.left.ik.accepted||!d.right.ik.accepted) {
          ++excluded;++exclusions[std::string(d.detail)+"/"+std::string(d.left.ik.detail)+"/"+std::string(d.right.ik.detail)];
        } else {
        ++accepted;
        // Settled HOLD may replace d.*.target with the model command target
        // after IK has solved latestTargets(). Never pair that command target
        // with the earlier pose-IK q when reporting IK residuals.
        const auto& ik_targets=g.latestTargets();
        const auto lp=robot.armKinematicsAt(ArmSide::kLeft,d.left.ik.q).tcp_pose;
        const auto rp=robot.armKinematicsAt(ArmSide::kRight,d.right.ik.q).tcp_pose;
        for(auto side:{ArmSide::kLeft,ArmSide::kRight}) {
          const auto& arm=side==ArmSide::kLeft?d.left:d.right;
          const auto& pose=side==ArmSide::kLeft?lp:rp;
          const auto& ik_target=side==ArmSide::kLeft?ik_targets.left:ik_targets.right;
          const double error=(pose.position-ik_target.palm.position).norm();
          if(layers&&mode==1&&error>.01)
            conflict_samples.push_back({ik_target,arm.ik.q,side,f.sequence,cycle,
              now-frames.front().receive_monotonic_ns,error});
          ik_residual_disagreement=std::max(ik_residual_disagreement,std::abs(error-arm.ik.palm_position_error));
          ik_vs_command_target_max=std::max(ik_vs_command_target_max,
              (ik_target.palm.position-arm.target.palm.position).norm());
          if(ik_residual_disagreement>1e-5)
            throw std::runtime_error("IK target/result mismatch beyond frozen cross-model tolerance");
          const auto& limits=robot.mapping(side).limits;
          const double clearance=std::min((arm.ik.q-limits.lower_position).minCoeff(),
              (limits.upper_position-arm.ik.q).minCoeff())-cfg.spark_upper_qpoases.joint_limit_margin_rad;
          large_error_near_limit+=error>.01&&clearance<.01;
          if(worst_side.empty()||error>worst_error) {worst_error=error;worst_sequence=f.sequence;
            worst_side=side==ArmSide::kLeft?"left":"right";
            worst_ik=arm.ik;worst_target=ik_target;worst_clearance=clearance;worst_cycle=cycle;}
          auto& s=stops[std::string(arm.ik.detail)];
          ++s.count;s.over_10mm+=error>.01;
          s.stage1_cap+=arm.ik.stage1_iterations>=cfg.spark_upper_qpoases.stage1_max_iterations;
          s.stage2_cap+=arm.ik.stage2_iterations>=cfg.spark_upper_qpoases.stage2_max_iterations;
          s.errors.push_back(error);
        }
        for(const auto& pair:{std::pair{lp,ik_targets.left.palm},std::pair{rp,ik_targets.right.palm}}) {
          const double pe=(pair.first.position-pair.second.position).norm();
          const double re=rotationDistance(pair.first.rotation,pair.second.rotation);
          if(!std::isfinite(pe)||!std::isfinite(re))throw std::runtime_error("nonfinite IK metric");
          position.push_back(pe);orientation.push_back(re);position_over+=pe>.01;orientation_over+=re>.1;
        }
        relation.push_back(((rp.position-lp.position)-(ik_targets.right.palm.position-ik_targets.left.palm.position)).norm());
        relative_rotation.push_back(rotationDistance(lp.rotation.transpose()*rp.rotation,
            ik_targets.left.palm.rotation.transpose()*ik_targets.right.palm.rotation));
        }
        // Accepted stale-stop output must also advance the bounded controller:
        // lack of a new pose IK sample is not a command to freeze qdot history.
        bool reference_attempted=false,reference_ok=false;
        if(controller&&d.accepted) {
          reference_attempted=true;
          // Kinematic command-model reference replay only. No physical feedback,
          // no mj_step, and no reference reported as measured actual.
          const auto result=controller->step(d.cartesian_references,{},d.posture_tasks,dt);
          const bool both=result.accepted&&result.left.accepted&&result.right.accepted;
          reference_ok=both;
          reference_accepted+=both;reference_rejected+=!both;
          if(!both)++reference_reasons[static_cast<int>(result.hold_reason)];
          if(mode)recovery_confirmed+=g.confirmSharedRootReference(d.shared_root_cycle,both);
          const auto feedback=[&](ArmSide side,const ArmControllerDiagnostics& arm) {
            SparkConstraintHeadroomFeedback value;
            value.accepted=d.accepted&&arm.accepted&&arm.ik.status==SolverStatus::kSolved&&
                double(now-f.receive_monotonic_ns)*1e-9<=options.continuity.freshness_s;
            value.qdot=arm.ik.qdot;
            value.qddot=controller->previousAcceleration(side);
            value.task_scale_position=arm.ik.task_scale_position;
            value.task_scale_orientation=arm.ik.task_scale_orientation;
            return value;
          };
          g.updateHeadroomFeedback(feedback(ArmSide::kLeft,result.left),
                                   feedback(ArmSide::kRight,result.right),dt);
          ++headroom_feedback_cycles;
          left=controller->referenceState(ArmSide::kLeft);
          right=controller->referenceState(ArmSide::kRight);
          for(auto side:{ArmSide::kLeft,ArmSide::kRight}) {
            const auto& state=side==ArmSide::kLeft?left:right;
            const auto& limits=robot.mapping(side).limits;
            if(!state.q.allFinite()||!state.qdot.allFinite()||!state.qddot.allFinite()||
                (state.q.array()<limits.lower_position.array()-1e-9).any()||
                (state.q.array()>limits.upper_position.array()+1e-9).any())
              throw std::runtime_error("offline reference violates finite/joint limits");
            if(both&&d.target_valid)reference_errors.push_back((robot.armKinematicsAt(side,state.q).tcp_pose.position-
                (side==ArmSide::kLeft?d.left:d.right).target.palm.position).norm());
            const auto& arm=side==ArmSide::kLeft?d.left:d.right;
            if(both&&d.target_valid&&arm.feedforward.valid) {
              const auto ff=robot.armKinematicsAt(side,arm.feedforward.q).tcp_pose;
              feedforward_errors.push_back((ff.position-arm.target.palm.position).norm());
              reference_to_feedforward.push_back((robot.armKinematicsAt(side,state.q).tcp_pose.position-ff.position).norm());
              ++headroom_sources[std::string(toString(arm.headroom.dominant_source))];
              low_headroom_sides+=arm.headroom.valid&&arm.headroom.scale<.5;
            }
          }
        }
        if(layers&&mode==1)for(auto side:{ArmSide::kLeft,ArmSide::kRight}) {
          const auto& arm=side==ArmSide::kLeft?d.left:d.right;
          const auto& target=side==ArmSide::kLeft?g.latestTargets().left:g.latestTargets().right;
          const auto& reference=side==ArmSide::kLeft?left:right;
          const bool ik_valid=d.target_valid&&d.left.ik.accepted&&d.right.ik.accepted;
          const bool ref_valid=reference_ok&&d.target_valid;
          const bool ff_valid=ref_valid&&arm.feedforward.valid;
          const auto ik_pose=robot.armKinematicsAt(side,arm.ik.q).tcp_pose;
          const auto ref_pose=robot.armKinematicsAt(side,reference.q).tcp_pose;
          const auto ff_pose=robot.armKinematicsAt(side,arm.feedforward.q).tcp_pose;
          std::cout<<"layer_sample mode=1 cycle="<<cycle<<" t_ns="<<now-frames.front().receive_monotonic_ns
            <<" source_t_ns="<<f.receive_monotonic_ns-frames.front().receive_monotonic_ns
            <<" sequence="<<f.sequence<<" side="<<(side==ArmSide::kLeft?"left":"right")
            <<" state="<<stateName(d.shared_root_state)<<" settled_hold="<<arm.settled_hold_active
            <<" blend="<<d.blend_active<<" guidance_accepted="<<d.accepted<<" target_valid="<<d.target_valid
            <<" ik_valid="<<ik_valid<<" reference_attempted="<<reference_attempted<<" reference_accepted="<<reference_ok
            <<" ff_valid="<<ff_valid<<" headroom_valid="<<(ff_valid&&arm.headroom.valid)
            <<" headroom_source="<<(ff_valid&&arm.headroom.valid?toString(arm.headroom.dominant_source):"unavailable")
            <<" detail="<<d.detail;
          const auto metric=[&](const char* name,bool valid,double value) {
            std::cout<<' '<<name<<'=';
            if(!valid)std::cout<<"NA";
            else {if(!std::isfinite(value))throw std::runtime_error("nonfinite layer diagnostic");std::cout<<value;}
          };
          metric("ik_m",ik_valid,(ik_pose.position-target.palm.position).norm());
          metric("ik_rad",ik_valid,rotationDistance(ik_pose.rotation,target.palm.rotation));
          metric("reference_command_m",ref_valid,(ref_pose.position-arm.target.palm.position).norm());
          metric("reference_ik_target_m",ref_valid,(ref_pose.position-target.palm.position).norm());
          metric("ik_command_m",ref_valid,(target.palm.position-arm.target.palm.position).norm());
          metric("ff_command_m",ff_valid,(ff_pose.position-arm.target.palm.position).norm());
          metric("reference_ff_m",ff_valid,(ref_pose.position-ff_pose.position).norm());
          metric("headroom_scale",ff_valid&&arm.headroom.valid,arm.headroom.scale);
          std::cout<<'\n';
        }
      }
      std::cout<<"mode="<<mode<<" frames="<<frames.size()<<" control_cycles="<<cycles
        <<" delivered_source_frames="<<delivered<<" superseded_source_frames="<<frames.size()-delivered
        <<" accepted="<<accepted<<" excluded="<<excluded
        <<" rejected_input="<<rejected_input<<" budget_exhausted="<<budgets
        <<" palm_p90_m="<<p90(position)<<" orientation_p90_rad="<<p90(orientation)
        <<" B_relation_p90_m="<<p90(relation)<<" relative_rotation_p90_rad="<<p90(relative_rotation)
        <<" side_samples="<<position.size()<<" position_over_10mm="<<position_over<<" orientation_over_0.1rad="<<orientation_over
        <<" classification=CauseUndetermined actual=unavailable scope="
        <<(fixed_ticks?"offline_200hz_reference_loop":reference_loop?"per_source_frame_reference_loop":"per_source_frame_IK_only")<<'\n';
      if(mode)std::cout<<"closed_control mode="<<mode<<" side_samples="<<closed_control_samples
        <<" max_bone_error_m="<<closed_control_bone_error<<" max_endpoint_error_m="<<closed_control_endpoint_error<<'\n';
      std::cout<<"ik_target_pairing mode="<<mode<<" max_residual_disagreement_m="<<ik_residual_disagreement
        <<" max_ik_vs_command_target_m="<<ik_vs_command_target_max
        <<" ik_target=solver_input reference_target=post_hold_command\n";
      if(reference_loop)std::cout<<"reference_loop mode="<<mode<<" accepted="<<reference_accepted
        <<" rejected="<<reference_rejected<<" recovery_confirmations="<<recovery_confirmed
        <<" tracking_frames="<<tracking<<" reference_palm_p90_m="<<p90(reference_errors)<<'\n';
      if(reference_loop)std::cout<<"reference_layers mode="<<mode<<" feedback_cycles="<<headroom_feedback_cycles
        <<" ff_target_p90_m="<<p90(feedforward_errors)<<" reference_ff_p90_m="<<p90(reference_to_feedforward)
        <<" samples="<<feedforward_errors.size()<<" headroom_scale_below_half="<<low_headroom_sides<<'\n';
      for(const auto& [source,count]:headroom_sources)
        std::cout<<"headroom_source mode="<<mode<<" source="<<source<<" side_samples="<<count<<'\n';
      for(const auto& [reason,count]:reference_reasons)
        std::cout<<"reference_rejection mode="<<mode<<" hold_reason="<<reason<<" frames="<<count<<'\n';
      if(!mapping_times.empty()) {
        std::sort(mapping_times.begin(),mapping_times.end());
        std::cout<<"mapping_timing mode="<<mode<<" samples="<<mapping_times.size()<<" p99_us="
          <<mapping_times[static_cast<std::size_t>(std::ceil(.99*static_cast<double>(mapping_times.size())))-1]
          <<" max_us="<<mapping_times.back()<<" real_time_qualified=false\n";
      }
      for(const auto& [reason,count]:exclusions)
        std::cout<<"excluded_reason mode="<<mode<<" frames="<<count<<" reason="<<reason<<'\n';
      for(const auto& [reason,s]:stops)
        std::cout<<"ik_stop mode="<<mode<<" reason="<<reason<<" side_samples="<<s.count
          <<" over_10mm="<<s.over_10mm<<" palm_p90_m="<<p90(s.errors)
          <<" stage1_at_iteration_cap="<<s.stage1_cap<<" stage2_at_iteration_cap="<<s.stage2_cap<<'\n';
      std::cout<<"residual_probe mode="<<mode<<" over_10mm_within_0.01rad_of_safe_limit="<<large_error_near_limit
        <<" worst_error_m="<<worst_error<<" worst_sequence="<<worst_sequence<<" worst_side="<<worst_side<<'\n';
      if(!position.empty())
        std::cout<<"worst_ik_sample mode="<<mode<<" cycle="<<worst_cycle<<" sequence="<<worst_sequence
          <<" side="<<worst_side<<" fk_position_error_m="<<worst_error
          <<" solver_position_error_m="<<worst_ik.palm_position_error
          <<" orientation_error_rad="<<worst_ik.palm_orientation_error
          <<" safe_limit_clearance_rad="<<worst_clearance<<" stop="<<worst_ik.detail
          <<" stage1_iterations="<<worst_ik.stage1_iterations
          <<" stage1_iteration_limit="<<cfg.spark_upper_qpoases.stage1_max_iterations
          <<" stage2_iterations="<<worst_ik.stage2_iterations
          <<" stage2_iteration_limit="<<cfg.spark_upper_qpoases.stage2_max_iterations
          <<" budget_exhausted="<<worst_ik.budget_exhausted
          <<" stage1_weighted_error="<<worst_ik.stage1_error
          <<" stage2_weighted_error="<<worst_ik.stage2_error
          <<" classification=CauseUndetermined\n";
      if(multiseed&&!position.empty()) {
        // Isolated cold solvers after replay: never feed a diagnostic result
        // back to guidance/controller or alter the recorded target and limits.
        const auto side=worst_side=="left"?ArmSide::kLeft:ArmSide::kRight;
        const auto& limits=robot.mapping(side).limits;
        const Vec7 lower=limits.lower_position.array()+cfg.spark_upper_qpoases.joint_limit_margin_rad;
        const Vec7 upper=limits.upper_position.array()-cfg.spark_upper_qpoases.joint_limit_margin_rad;
        const Vec7 midpoint=.5*(lower+upper);
        PinocchioArmKinematics kinematics(options.urdf_path,
          {robot.tcpRelativeToLink7(ArmSide::kLeft),robot.tcpRelativeToLink7(ArmSide::kRight)});
        for(int trial=0;trial<16;++trial) {
          Vec7 seed=trial==0?worst_ik.q:midpoint;
          if(trial>=2) {const int joint=(trial-2)/2;
            seed[joint]+=(trial%2?1.0:-1.0)*.25*(upper[joint]-lower[joint]);}
          SparkUpperQpoasesIk7 solver(side,kinematics,limits,cfg.spark_upper_qpoases,cfg.qpoases);
          const auto result=solver.solve(worst_target,seed);
          const auto pose=robot.armKinematicsAt(side,result.q).tcp_pose;
          const double error=(pose.position-worst_target.palm.position).norm();
          const double orientation_error=rotationDistance(pose.rotation,worst_target.palm.rotation);
          const double clearance=std::min((result.q-lower).minCoeff(),(upper-result.q).minCoeff());
          if(!result.q.allFinite()||!std::isfinite(error)||!std::isfinite(orientation_error)||clearance< -1e-9)
            throw std::runtime_error("multiseed diagnostic nonfinite or unsafe result");
          if(result.accepted&&std::abs(error-result.palm_position_error)>1e-5)
            throw std::runtime_error("multiseed diagnostic target/result mismatch");
          std::cout<<"ik_multiseed mode="<<mode<<" sequence="<<worst_sequence<<" side="<<worst_side
            <<" trial="<<trial<<" seed="<<(trial==0?"previous_solution":trial==1?"safe_midpoint":"axis_offset")
            <<" accepted="<<result.accepted<<" position_error_m="<<error<<" orientation_error_rad="<<orientation_error
            <<" safe_limit_clearance_rad="<<clearance<<" stop="<<result.detail
            <<" stage1_iterations="<<result.stage1_iterations<<" stage2_iterations="<<result.stage2_iterations
            <<" budget_exhausted="<<result.budget_exhausted
            <<" q_distance_from_replay_rad="<<(result.q-worst_ik.q).norm()
            <<" cold_solver=true scope=isolated_target_not_control_replay actual=unavailable\n";
        }
      }
      if(layers&&mode==1)taskConflictAblation(conflict_samples,robot,options,cfg);
    }
    if(sharedRootSha256File(argv[2])!=fingerprint||sharedRootSha256File(argv[1])!=options.profile_sha256)
      throw std::runtime_error("audit inputs changed during run");
    std::cout<<"trace_sha256="<<fingerprint<<" input_contract_sha256="<<options.input_sha256
        <<" profile_sha256="<<options.profile_sha256<<" motion_authorized=false phase_a_accepted=false\n";
    return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 2;}
}
