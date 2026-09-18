#include "tianji_teleop/ik/dexhand_qp_arm_ik.hpp"
#include "tianji_v131/velocity_qp.hpp"
#include "tianji_v131/so3.hpp"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <array>
#include <stdexcept>
#include <filesystem>
#include <cstdlib>

namespace tianji_teleop {
namespace ref = tianji_v131;
namespace pin = pinocchio;
struct DexhandQpArmIk::Impl {
  pin::Model model;
  mutable std::unique_ptr<pin::Data> data;
  IkSettings settings;
  std::array<std::array<int, 7>, 2> qi{}, vi{};
  std::array<pin::FrameIndex, 2> base{}, tcp{};
  std::array<ref::ArmLimits, 2> limits;
  std::unique_ptr<ref::MujocoRobot> robot;
  std::array<Eigen::Isometry3d,2> world_from_base;
  std::array<std::unique_ptr<ref::VelocityQp>, 2> solvers;
  Impl(const std::string& urdf, const IkSettings& s, const std::string& model_path) : settings(s) {
    pin::urdf::buildModel(urdf, model);
    data = std::make_unique<pin::Data>(model);
    robot=std::make_unique<ref::MujocoRobot>(model_path);
    pin::forwardKinematics(model,*data,pin::neutral(model));
    pin::updateFramePlacements(model,*data);
    for (int side=0; side<2; ++side) {
      const std::string suffix = side == 0 ? "L" : "R";
      base[side] = model.getFrameId("Base_"+suffix);
      tcp[side] = model.getFrameId("TCP_Link_"+suffix);
      if(base[side]>=static_cast<pin::FrameIndex>(model.nframes) || tcp[side]>=static_cast<pin::FrameIndex>(model.nframes))
        throw std::invalid_argument("Dexhand QP missing Base/TCP frame");
      world_from_base[side]=data->oMf[base[side]].toHomogeneousMatrix();
      limits[side]=robot->mapping(side==0?ref::ArmSide::kLeft:ref::ArmSide::kRight).limits;
      for(int j=0;j<7;++j) {
        auto id=model.getJointId("Joint"+std::to_string(j+1)+"_"+suffix);
        if(id==0 || id>=model.joints.size() || model.joints[id].nq()!=1 || model.joints[id].nv()!=1)
          throw std::invalid_argument("Dexhand QP missing scalar arm joint");
        qi[side][j]=model.joints[id].idx_q(); vi[side][j]=model.joints[id].idx_v();
        // Original fast simulation profile, still honoring the public step
        // contract when an embedding caller explicitly requests a lower bound.
        limits[side].velocity[j]=std::min(4.0, s.maximum_joint_step_rad / s.control_period_s);
      }
      solvers[side]=std::make_unique<ref::VelocityQp>(limits[side]);
    }
    // Original initializePicoEeDlsKinematics compatibility gate: reject a
    // gradient URDF inconsistent with the active MuJoCo kinematic model.
    for(int side=0;side<2;++side) {
      for(int probe=0;probe<2;++probe) {
        ArmJointVector q=ArmJointVector::Zero();
        if(probe) for(int j=0;j<7;++j)
          q[j]=std::clamp((j%2==0?1.0:-1.0)*.031,
            limits[side].lower_position[j],limits[side].upper_position[j]);
        const auto p=gradient_sample(side,q);
        const auto m=robot->armKinematicsAt(side==0?ref::ArmSide::kLeft:ref::ArmSide::kRight,q);
        if(!p.tcp_pose.position.isApprox(m.tcp_pose.position,1e-5) ||
           ref::rotationDistance(p.tcp_pose.rotation,m.tcp_pose.rotation)>1e-5 ||
           !p.tcp_jacobian.isApprox(m.tcp_jacobian,1e-5))
          throw std::invalid_argument("v131 Pinocchio kinematics do not match MuJoCo");
      }
    }
  }
  ref::ArmKinematicSample gradient_sample(int side, const ArmJointVector& q) const {
    if(!q.allFinite()) throw std::invalid_argument("nonfinite FK joints");
    Eigen::VectorXd full=pin::neutral(model);
    for(int j=0;j<7;++j) full[qi[side][j]]=q[j];
    pin::computeJointJacobians(model,*data,full);
    pin::updateFramePlacements(model,*data);
    auto pose=data->oMf[tcp[side]];
    Eigen::Matrix<double,6,Eigen::Dynamic> jac(6,model.nv); jac.setZero();
    pin::getFrameJacobian(model,*data,tcp[side],pin::ReferenceFrame::LOCAL_WORLD_ALIGNED,jac);
    ref::ArmKinematicSample out;
    out.tcp_pose.position=pose.translation(); out.tcp_pose.rotation=pose.rotation();
    for(int j=0;j<7;++j) {
      out.tcp_jacobian.col(j)=jac.col(vi[side][j]);
    }
    return out;
  }
};
DexhandQpArmIk::DexhandQpArmIk(const std::string& u,const IkSettings& s,const std::string& model_path):impl_(std::make_unique<Impl>(u,s,model_path)) {}
DexhandQpArmIk::~DexhandQpArmIk()=default;
void DexhandQpArmIk::reset(ArmSide side) const {
  impl_->solvers[side==ArmSide::kLeft?0:1]->reset();
}
void DexhandQpArmIk::command_feedback(ArmSide side,const ArmJointVector& q) const {
  impl_->solvers[side==ArmSide::kLeft?0:1]->feedback(q,impl_->settings.control_period_s);
}
Eigen::Isometry3d DexhandQpArmIk::forward(ArmSide side,const ArmJointVector& q) const {
  const int i=side==ArmSide::kLeft?0:1;
  const auto sample=impl_->robot->armKinematicsAt(i==0?ref::ArmSide::kLeft:ref::ArmSide::kRight,q);
  Eigen::Isometry3d out=Eigen::Isometry3d::Identity();
  out.linear()=sample.tcp_pose.rotation; out.translation()=sample.tcp_pose.position;
  return impl_->world_from_base[i].inverse()*out;
}
IkResult DexhandQpArmIk::solve(ArmSide side,const Eigen::Isometry3d& target,
  const ArmJointVector& q,const Eigen::Vector3d&) const {
  return solve_timed(side,target,q,Eigen::Vector3d::Zero(),-1,-1,-1);
}
IkResult DexhandQpArmIk::solve_timed(ArmSide side,const Eigen::Isometry3d& target,
  const ArmJointVector& q,const Eigen::Vector3d&,
  double source_time,double receive_time,double now) const {
  const int i=side==ArmSide::kLeft?0:1;
  IkResult out;
  out.joints_rad=q.allFinite()?q:ArmJointVector::Zero();
  out.achieved_pose=forward(side,out.joints_rad);
  if(!q.allFinite()) { out.status="nonfinite_seed"; return out; }
  const auto world_target=impl_->world_from_base[i]*target;
  ref::Pose desired; desired.position=world_target.translation(); desired.rotation=world_target.rotation();
  const auto arm=i==0?ref::ArmSide::kLeft:ref::ArmSide::kRight;
  const auto r=impl_->solvers[i]->solve(i==0?ref::ArmSide::kLeft:ref::ArmSide::kRight,
    desired,q,[&](const ref::Vec7& v){return impl_->robot->armKinematicsAt(arm,v);},impl_->settings.control_period_s,
    source_time,receive_time,now,[&](const ref::Vec7& v){return impl_->gradient_sample(i,v);});
  // Host "accepted" means deliverable command, unlike the original core's
  // strict QP acceptance. Bounded fallback remains explicitly degraded.
  out.status=std::string(r.detail); out.accepted=r.accepted || r.fallback_applied;
  impl_->robot->setArmState(arm,r.q,r.qdot);
  out.joints_rad=r.q; out.achieved_pose=forward(side,r.q);
  if(!out.accepted) return out;
  out.model_state_only=true; out.reference_velocity_rad_s=r.qdot;
  out.joints_rad=r.q; out.achieved_pose=forward(side,r.q);
  out.maximum_joint_step_rad=(r.q-r.previous_q).cwiseAbs().maxCoeff();
  out.position_error_m=(out.achieved_pose.translation()-target.translation()).norm();
  out.orientation_error_rad=ref::rotationDistance(out.achieved_pose.rotation(),target.rotation());
  out.minimum_singular_value=r.minimum_singular_value;
  out.solver_iterations=r.active_set_iterations; out.active_joint_constraints=r.active_bound_count;
  out.solve_time_ms=r.solve_time_us/1000;
  out.converged=out.position_error_m<=impl_->settings.position_tolerance_m &&
    out.orientation_error_rad<=impl_->settings.orientation_tolerance_rad;
  return out;
}
}
