#include "tianji_teleop/ik/dexhand_qp_arm_ik.hpp"
#include "tianji_v131/mujoco_robot.hpp"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <iostream>
#include <iomanip>

int main(int argc,char**argv) {
  if(argc<3) return 2;
  const bool combined=argc>3;
  using namespace tianji_teleop;
  IkSettings settings; settings.maximum_joint_step_rad=.02;settings.control_period_s=.005;
  DexhandQpArmIk solver(argv[1],settings,argv[2]);
  tianji_v131::MujocoRobot robot(argv[2]);
  ArmJointVector ql,qr;
  ql<<1.1,-.6,-1.52,-1.1,0,0,0;qr<<-1.1,-.6,1.52,-1.1,0,0,0;
  if(combined) {ql<<55,-65,-70,-60,60,0,0;qr<<-55,-65,70,-60,-60,0,0;ql*=M_PI/180;qr*=M_PI/180;}
  robot.setArmState(tianji_v131::ArmSide::kLeft,ql,ArmJointVector::Zero());
  robot.setArmState(tianji_v131::ArmSide::kRight,qr,ArmJointVector::Zero());robot.forward();
  auto hl=robot.tcpPose(tianji_v131::ArmSide::kLeft),hr=robot.tcpPose(tianji_v131::ArmSide::kRight);
  pinocchio::Model model;pinocchio::urdf::buildModel(argv[1],model);pinocchio::Data data(model);
  pinocchio::forwardKinematics(model,data,pinocchio::neutral(model));pinocchio::updateFramePlacements(model,data);
  Eigen::Isometry3d bl(data.oMf[model.getFrameId("Base_L")].toHomogeneousMatrix());
  Eigen::Isometry3d br(data.oMf[model.getFrameId("Base_R")].toHomogeneousMatrix());
  std::cout<<std::setprecision(17);
  for(int tick=0;tick<600;++tick) {
    const double t=tick*.005,stamp=1+(tick/2)*.01;
    Eigen::Isometry3d l=Eigen::Isometry3d::Identity(),r=l;
    l.translation()=hl.position+Eigen::Vector3d(.025*std::sin(t),.02*std::sin(.7*t),.015*std::sin(1.3*t));
    r.translation()=hr.position+Eigen::Vector3d(-.02*std::sin(t),.015*std::sin(.8*t),.02*std::sin(.9*t));
    l.linear()=Eigen::AngleAxisd(.08*std::sin(t),Eigen::Vector3d(1,.4,.7).normalized()).toRotationMatrix()*hl.rotation;
    r.linear()=Eigen::AngleAxisd(.06*std::sin(t),Eigen::Vector3d(.3,1,.6).normalized()).toRotationMatrix()*hr.rotation;
    if(combined) {
      l.translation()=hl.position+Eigen::AngleAxisd(-1.5708,Eigen::Vector3d::UnitX())*Eigen::Vector3d(.02,-.02,.02);
      r.translation()=hr.position+Eigen::AngleAxisd(1.5708,Eigen::Vector3d::UnitX())*Eigen::Vector3d(.02,-.02,.02);
      l.linear()=hl.rotation*Eigen::AngleAxisd(8*M_PI/180,Eigen::Vector3d::UnitZ()).toRotationMatrix();
      r.linear()=hr.rotation*Eigen::AngleAxisd(8*M_PI/180,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    }
    // Deliberately hold the external seed at Home for every tick.
    auto a=solver.solve_timed(ArmSide::kLeft,bl.inverse()*l,ql,Eigen::Vector3d::Zero(),stamp,stamp,1+t);
    auto b=solver.solve_timed(ArmSide::kRight,br.inverse()*r,qr,Eigen::Vector3d::Zero(),stamp,stamp,1+t);
    std::cout<<tick<<' '<<a.accepted<<' '<<b.accepted;
    for(const auto& q:{a.joints_rad,b.joints_rad}) for(int j=0;j<7;++j) std::cout<<' '<<q[j];
    std::cout<<'\n';
  }
}
