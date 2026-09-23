#include "tianji_qp_ik/shared_root_options.hpp"
#include "tianji_qp_ik/pinocchio_arm_kinematics.hpp"
#include "tianji_qp_ik/so3.hpp"
#include "tianji_qp_ik/resource_paths.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <cmath>
#include <stdexcept>
#include <set>

namespace tianji_qp_ik {
namespace {
using Node=YAML::Node;
namespace fs=std::filesystem;
void require(bool value,const std::string& message) {if(!value)throw std::runtime_error("shared-root: "+message);}
void keys(const Node& n,std::initializer_list<const char*> allowed,std::string_view optional={}) {
  require(n.IsMap(),"expected configuration map");std::set<std::string> names;
  for(auto name:allowed)names.insert(name);
  std::set<std::string> seen;
  for(const auto& entry:n) {
    const auto key=entry.first.as<std::string>();
    require(names.count(key)!=0,"unknown key: "+key);
    require(seen.insert(key).second,"duplicate key: "+key);
  }
  for(auto name:allowed)if(optional!=name)require(bool(n[name]),std::string("missing key: ")+name);
}
double pos(const Node& n,const char* key) {
  const double v=n[key].as<double>();require(std::isfinite(v)&&v>0,std::string("invalid ")+key);return v;
}
std::array<double,2> range(const Node& n,const char* key) {
  auto a=n[key];require(a.IsSequence()&&a.size()==2,std::string("invalid range ")+key);
  std::array<double,2> out{a[0].as<double>(),a[1].as<double>()};
  require(std::isfinite(out[0])&&std::isfinite(out[1])&&out[0]>0&&out[1]>out[0],key);return out;
}
Eigen::Vector3d vec(const Node& n,const char* key) {
  auto a=n[key];require(a.IsSequence()&&a.size()==3,std::string("invalid vector ")+key);
  Eigen::Vector3d v(a[0].as<double>(),a[1].as<double>(),a[2].as<double>());
  require(v.allFinite(),key);return v;
}
void fingerprint(const fs::path& p,const std::string& expected) {
  require(sharedRootSha256File(p.string())==expected,"fingerprint mismatch: "+p.string());
}
Pose transform(const Node& n) {
  keys(n,{"translation_m","quaternion_wxyz"});
  Pose p;p.position=vec(n,"translation_m");
  auto q=n["quaternion_wxyz"];
  require(q.IsSequence()&&q.size()==4,"invalid quaternion");
  Eigen::Quaterniond rotation(q[0].as<double>(),q[1].as<double>(),q[2].as<double>(),q[3].as<double>());
  require(rotation.coeffs().allFinite()&&std::abs(rotation.norm()-1)<1e-10,"nonunit quaternion");
  p.rotation=rotation.toRotationMatrix();return p;
}
} // namespace
SharedRootOptions loadSharedRootOptions(const std::string& path) {
  SharedRootOptions out;const fs::path profile=fs::canonical(path);
  out.profile_path=profile.string();out.profile_sha256=sharedRootSha256File(path);
  const Node root=YAML::LoadFile(path);
  require(root.IsMap(),"expected configuration map");
  std::set<std::string> root_keys;
  for(const auto& entry:root)
    require(root_keys.insert(entry.first.as<std::string>()).second,"duplicate root configuration key");
  const Node c=root["shared_root"];
  keys(c,{"enabled","input_contract_artifact","robot_geometry_artifact","input_mode",
      "control_point_semantic","shape_proxy_semantic","morphology","bridge_frame_filter","continuity","target_gate","reachable_projection"},"reachable_projection");
  out.enabled=c["enabled"].as<bool>();
  const auto algorithm=root["ik"]["algorithm"].as<std::string>();
  require(algorithm=="pico_ee_franka_dls","incompatible algorithm");
  require(c["input_mode"].as<std::string>()=="tjvr_bridge_shoulder_frame","unsupported input");
  require(c["control_point_semantic"].as<std::string>()=="reconstructed_palm"&&
      c["shape_proxy_semantic"].as<std::string>()=="reconstructed_palm_proxy","wrong point semantics");
  const auto input=fs::canonical(profile.parent_path()/c["input_contract_artifact"].as<std::string>());
  const auto geometry=fs::canonical(profile.parent_path()/c["robot_geometry_artifact"].as<std::string>());
  out.input_contract_path=input.string();out.geometry_path=geometry.string();
  out.input_sha256=sharedRootSha256File(input.string());out.geometry_sha256=sharedRootSha256File(geometry.string());
  // Adapter v1 has one source/basis contract; arbitrary valid rotations or new
  // field mappings must not silently reinterpret that implementation.
  require(out.input_sha256=="a408ab5102a7fe305abe37e7f28c21166ec34a7d6af206ace8bc551cfbb7e099","unsupported input contract revision");
  // The reviewed DLS geometry retains its frozen model hashes and runtime checks.
  require(out.geometry_sha256=="1a8f99f66b078c85e3776241421ded5da12d8b61ed34e796eb6cebbb39bec4f7",
      "unsupported robot geometry revision");
  const Node g=YAML::LoadFile(geometry.string())["robot_geometry"];
  require(g["contract_version"].as<int>()==2,"unsupported robot contract");
  require(g["transform_convention"].as<std::string>()=="parent_T_child"&&
      g["wrist_center_orientation_semantic"].as<std::string>()=="virtual_frame_axes_parallel_to_solver_tcp",
      "unsupported closure frame convention");
  require(g["tjvr_input_contract_sha256"].as<std::string>()==out.input_sha256,"artifact link mismatch");
  out.urdf_path=fs::canonical(controllerResource(geometry,g["urdf_path"].as<std::string>())).string();
  out.mujoco_path=fs::canonical(controllerResource(geometry,g["mujoco_xml_path"].as<std::string>())).string();
  fingerprint(out.urdf_path,g["urdf_sha256"].as<std::string>());
  fingerprint(out.mujoco_path,g["mujoco_xml_sha256"].as<std::string>());
  auto& b=out.builder;auto& m=out.morphology;auto& t=out.continuity;
  if(const auto p=c["reachable_projection"]) {
    keys(p,{"enabled","maximum_translation_m","maximum_speed_m_s"});
    b.reachable_projection_enabled=p["enabled"].as<bool>();
    b.projection_maximum_translation_m=pos(p,"maximum_translation_m");
    b.projection_maximum_speed_m_s=pos(p,"maximum_speed_m_s");
    require(b.projection_maximum_translation_m<=.10,"projection translation exceeds experimental bound");
  }
  auto r=g["R_BCt"];require(r.IsSequence()&&r.size()==3,"invalid R_BCt");
  for(std::size_t i=0;i<3;++i) {require(r[i].IsSequence()&&r[i].size()==3,"invalid rotation row");
    for(std::size_t j=0;j<3;++j)b.R_BCt(static_cast<int>(i),static_cast<int>(j))=r[i][j].as<double>();}
  require(isProperRotation(b.R_BCt),"R_BCt not SO(3)");b.o_B=vec(g,"o_B_m");
  const auto mg=c["morphology"];
  keys(mg,{"window_frames","minimum_unique_samples","mad_ratio_max","outlier_ratio","provisional_timeout_s",
    "sustained_invalid_timeout_s","shoulder_width_min_m","shoulder_width_max_m","upper_arm_range_m","forearm_range_m",
    "wrist_palm_proxy_range_m","reach_scale_range","lateral_scale_range"});
  const int window=mg["window_frames"].as<int>(),minimum=mg["minimum_unique_samples"].as<int>();
  require(minimum>=2&&minimum<=window&&window<=128,"invalid morphology count");
  m.window_frames=static_cast<std::size_t>(window);m.minimum_unique_samples=static_cast<std::size_t>(minimum);
  m.mad_ratio_max=pos(mg,"mad_ratio_max");m.outlier_ratio=pos(mg,"outlier_ratio");
  m.provisional_timeout_s=pos(mg,"provisional_timeout_s");m.sustained_invalid_timeout_s=pos(mg,"sustained_invalid_timeout_s");
  m.shoulder_range={pos(mg,"shoulder_width_min_m"),pos(mg,"shoulder_width_max_m")};
  m.upper_range=range(mg,"upper_arm_range_m");m.forearm_range=range(mg,"forearm_range_m");
  m.wrist_palm_range=range(mg,"wrist_palm_proxy_range_m");m.reach_scale_range=range(mg,"reach_scale_range");m.lateral_scale_range=range(mg,"lateral_scale_range");
  auto filter=c["bridge_frame_filter"];
  keys(filter,{"center_time_constant_s","relation_time_constant_s","direction_time_constant_s","orientation_time_constant_s","scale_time_constant_s"});
  b.center_tau_s=pos(filter,"center_time_constant_s");b.relation_tau_s=pos(filter,"relation_time_constant_s");
  b.direction_tau_s=pos(filter,"direction_time_constant_s");b.orientation_tau_s=pos(filter,"orientation_time_constant_s");b.scale_tau_s=pos(filter,"scale_time_constant_s");
  auto co=c["continuity"];
  keys(co,{"maximum_hold_s","freshness_s","recovery_unique_frames","recovery_min_receive_span_s","recovery_max_receive_gap_s","source_dt_max_s","blend_seconds"});
  const int frames=co["recovery_unique_frames"].as<int>();require(frames>=2&&frames<=128,"invalid recovery count");t.recovery_frames=static_cast<std::size_t>(frames);
  t.maximum_hold_s=pos(co,"maximum_hold_s");t.freshness_s=pos(co,"freshness_s");
  t.minimum_receive_span_s=pos(co,"recovery_min_receive_span_s");t.maximum_receive_gap_s=pos(co,"recovery_max_receive_gap_s");
  t.maximum_source_gap_s=b.maximum_source_dt_s=pos(co,"source_dt_max_s");t.blend_s=pos(co,"blend_seconds");
  t.maximum_elbow_step_m=pos(root["pico_teleop"],"max_position_jump_m");
  auto gate=c["target_gate"];
  keys(gate,{"maximum_center_offset_from_robot_shoulders_m","maximum_position_correction_m","maximum_orientation_correction_rad"});
  b.maximum_center_m=pos(gate,"maximum_center_offset_from_robot_shoulders_m");b.maximum_correction_m=pos(gate,"maximum_position_correction_m");
  // Palm and shape rotation have the same source; no orientation correction
  // is introduced by this builder. Retain schema validation rather than ignore NaN.
  require(pos(gate,"maximum_orientation_correction_rad")<=3.141592653589793,"invalid angular gate");
  // Startup FK only, no simulation stepping or device I/O.
  MujocoRobot robot(out.mujoco_path);
  PinocchioArmKinematics kin(out.urdf_path,{robot.tcpRelativeToLink7(ArmSide::kLeft),robot.tcpRelativeToLink7(ArmSide::kRight)});
  const auto qref=g["reference_q_rad"];require(qref.IsSequence()&&qref.size()==14,"invalid reference q");
  double reach_sum=0;
  for(std::size_t side=0;side<2;++side) {
    const auto arm=side==0?ArmSide::kLeft:ArmSide::kRight;Vec7 q;
    for(int j=0;j<7;++j)q[j]=qref[side*7+static_cast<std::size_t>(j)].as<double>();
    require(q.allFinite(),"nonfinite reference q");
    const auto p=kin.sample(arm,q),mj=robot.armKinematicsAt(arm,q);
    const double tol=pos(g,"cross_model_position_tolerance_m");
    require((p.tcp_pose.position-mj.tcp_pose.position).norm()<tol&&
        (p.wrist_position-mj.wrist_position).norm()<tol&&
        (p.shoulder_position-mj.shoulder_position).norm()<tol,"model FK mismatch");
    require(rotationDistance(p.tcp_pose.rotation,mj.tcp_pose.rotation)<pos(g,"cross_model_orientation_tolerance_rad"),"model TCP axes mismatch");
    const std::string prefix=side==0?"left_":"right_";
    const std::string suffix=side==0?"L":"R";
    const auto node=g[side==0?"left":"right"];
    auto& closure=out.closure_geometry[side];
    closure.shoulder_frame=node["shoulder_center_frame"].as<std::string>();
    closure.elbow_frame=node["elbow_center_frame"].as<std::string>();
    closure.wrist_frame=node["wrist_center_frame"].as<std::string>();
    closure.tcp_frame=node["solver_tcp_frame"].as<std::string>();
    require(closure.shoulder_frame=="Link1_"+suffix&&closure.elbow_frame=="Link4_"+suffix&&
        closure.wrist_frame=="Link5_"+suffix&&closure.tcp_frame=="tcp_"+suffix,"unsupported joint centers");
    closure.shoulder_B=vec(node,"shoulder_center_B_m");
    closure.upper_length_m=pos(node,"upper_arm_length_m");
    closure.forearm_length_m=pos(node,"forearm_length_m");
    closure.tcp_to_wrist_center=transform(node["T_solver_tcp_to_wrist_center"]);
    closure.link7_to_tcp=transform(node["T_link7_to_solver_tcp"]);
    const auto tcp=robot.tcpRelativeToLink7(arm);
    require((tcp.position-closure.link7_to_tcp.position).norm()<1e-10&&
        rotationDistance(tcp.rotation,closure.link7_to_tcp.rotation)<1e-10,"closure TCP extrinsic mismatch");
    require((mj.shoulder_position-closure.shoulder_B).norm()<1e-10&&
        (mj.tcp_pose.position+mj.tcp_pose.rotation*closure.tcp_to_wrist_center.position-mj.wrist_position).norm()<1e-10&&
        rotationDistance(closure.tcp_to_wrist_center.rotation,Eigen::Matrix3d::Identity())<1e-10,
        "closure wrist-center contract mismatch");
    require((p.shoulder_position-vec(g,(prefix+"shoulder_B_m").c_str())).norm()<tol,"shoulder artifact mismatch");
    // Artifact geometry is defined by the XML model. The URDF is independently
    // compared above at the frozen cross-model tolerance, not substituted for
    // the XML geometry (its rounded fixed transforms differ slightly).
    const Eigen::Vector3d upper=mj.shoulder_rotation.transpose()*(mj.elbow_position-mj.shoulder_position);
    const Eigen::Vector3d forearm=mj.elbow_rotation.transpose()*(mj.wrist_position-mj.elbow_position);
    const Eigen::Vector3d hand=mj.wrist_rotation.transpose()*(mj.tcp_pose.position-mj.wrist_position);
    const Eigen::Vector3d lengths(upper.norm(),forearm.norm(),hand.norm());
    require((lengths-vec(g,(prefix+"segment_lengths_m").c_str())).norm()<1e-10,"segment geometry mismatch");
    require(std::abs(upper.norm()-closure.upper_length_m)<1e-10&&
        std::abs(forearm.norm()-closure.forearm_length_m)<1e-10&&
        std::abs(hand.norm()-pos(node,"wrist_to_shape_proxy_length_m"))<1e-10,"closure lengths mismatch");
    if(side==0){b.geometry.left_shoulder=mj.shoulder_position;b.geometry.left_upper_arm_local=upper;b.geometry.left_forearm_local=forearm;b.geometry.left_wrist_to_palm_local=hand;}
    else{b.geometry.right_shoulder=mj.shoulder_position;b.geometry.right_upper_arm_local=upper;b.geometry.right_forearm_local=forearm;b.geometry.right_wrist_to_palm_local=hand;}
    reach_sum+=upper.norm()+forearm.norm()+hand.norm();
  }
  require((b.o_B-.5*(b.geometry.left_shoulder+b.geometry.right_shoulder)).norm()<1e-8,"robot root mismatch");
  // This artifact family fixes forward/left/up to the model base, not J1 axes.
  require((b.R_BCt-Eigen::Matrix3d::Identity()).norm()<1e-12,"unsupported model axis contract");
  m.robot_width_m=(b.geometry.left_shoulder-b.geometry.right_shoulder).norm();m.robot_reach_m=.5*reach_sum;
  b.closure_geometry=out.closure_geometry;
  (void)SharedRootMorphologyEstimator(m);(void)SharedRootContinuity(t);(void)SharedRootTargetBuilder(b);
  require(out.profile_sha256==sharedRootSha256File(path)&&out.input_sha256==sharedRootSha256File(input.string())&&
      out.geometry_sha256==sharedRootSha256File(geometry.string()),"configuration changed during load");
  return out;
}
} // namespace tianji_qp_ik
