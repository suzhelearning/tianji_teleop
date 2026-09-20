#include "tianji_qp_ik/pinocchio_arm_kinematics.hpp"
#include "tianji_qp_ik/so3.hpp"
#include "tianji_qp_ik/shared_root_options.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

#include <gtest/gtest.h>
#include <array>

namespace tianji_qp_ik {
namespace {
// Offline geometry audit only: no viewer, sockets, actuator, or mj_step.
TEST(SharedRootGeometry, PalmTcpExtensionIsIsolatedFromLegacyModel) {
  MujocoRobot legacy(TIANJI_PROJECT_SOURCE_DIR "/models/marvin_m6_wuji2.xml");
  MujocoRobot palm(TIANJI_PROJECT_SOURCE_DIR "/models/marvin_m6_wuji2_shared_root.xml");
  for (auto side : {ArmSide::kLeft, ArmSide::kRight}) {
    const auto a=legacy.tcpRelativeToLink7(side), b=palm.tcpRelativeToLink7(side);
    EXPECT_LT((a.position-Eigen::Vector3d(0,-.1315,0)).norm(),1e-10);
    EXPECT_LT((b.position-Eigen::Vector3d(0,-.1615,0)).norm(),1e-10);
    EXPECT_LT(rotationDistance(a.rotation,b.rotation),1e-10);
    EXPECT_NEAR((b.position-a.position).norm(),.03,1e-10);
  }
}
TEST(SharedRootGeometry, FixedShoulderRootAndCrossModelFrames) {
  MujocoRobot robot(TIANJI_PROJECT_SOURCE_DIR "/models/marvin_m6_wuji2_shared_root.xml");
  PinocchioArmKinematics kin(
      TIANJI_PROJECT_SOURCE_DIR "/models/marvin_m6_s_ccs_696_v4_local.urdf",
      {robot.tcpRelativeToLink7(ArmSide::kLeft),
       robot.tcpRelativeToLink7(ArmSide::kRight)});
  for (auto side : {ArmSide::kLeft, ArmSide::kRight}) {
    const bool left = side == ArmSide::kLeft;
    const double sign = left ? 1.0 : -1.0;
    Vec7 home;
    home << sign * 1.10, -1.52, -sign * 1.52, -1.10, 0, 0, 0;
    Vec7 a, b;
    a << sign * 0.4, -0.8, -sign * 0.6, -1.2, 0.2, 0.5, -0.3;
    b << -sign * 0.3, -1.1, -sign * 1.0, -0.7, -0.4, -0.5, 0.6;
    const std::array<Vec7, 4> poses{Vec7::Zero(), home, a, b};
    for (const auto& q : poses) {
      const auto p = kin.sample(side, q);
      const auto m = robot.armKinematicsAt(side, q);
      constexpr double tol = 1e-5;
      EXPECT_LT((p.shoulder_position - m.shoulder_position).norm(), tol);
      EXPECT_LT((p.elbow_position - m.elbow_position).norm(), tol);
      EXPECT_LT((p.wrist_position - m.wrist_position).norm(), tol);
      EXPECT_LT((p.tcp_pose.position - m.tcp_pose.position).norm(), tol);
      EXPECT_LT(rotationDistance(p.shoulder_rotation, m.shoulder_rotation), tol);
      EXPECT_LT(rotationDistance(p.wrist_rotation, m.wrist_rotation), tol);
      EXPECT_LT(rotationDistance(p.tcp_pose.rotation, m.tcp_pose.rotation), tol);
      EXPECT_LT((m.shoulder_position - Eigen::Vector3d(0, sign * .2115, 1.121)).norm(), 1e-10);
      EXPECT_NEAR((m.tcp_pose.position - m.wrist_position).norm(), .1615, 1e-10);
    }
  }
}

TEST(SharedRootGeometry, R3ClosureExtrinsicsAcrossModelsAndJointPoses) {
  for(const auto profile:{"/config/qp_ik_pico_shared_root.yaml", "/config/qp_ik_pico_shared_root_ceres.yaml"}) {
  SCOPED_TRACE(profile);
  const auto o=loadSharedRootOptions(std::string(TIANJI_PROJECT_SOURCE_DIR)+profile);
  MujocoRobot robot(o.mujoco_path);
  PinocchioArmKinematics kin(o.urdf_path,{robot.tcpRelativeToLink7(ArmSide::kLeft),robot.tcpRelativeToLink7(ArmSide::kRight)});
  for(int side=0;side<2;++side) {
    const auto arm=side?ArmSide::kRight:ArmSide::kLeft;
    const auto& g=o.closure_geometry[side];const auto& limits=robot.mapping(arm).limits;
    double max_vector=0,max_origin=0,max_position=0,max_rotation=0,max_length=0;
    for(int k=0;k<100;++k) {
      Vec7 q;
      for(int j=0;j<7;++j) {
        const double f=.5+.49*std::sin(double((k+1)*(j+1))*.6180339887498948);
        q[j]=limits.lower_position[j]+f*(limits.upper_position[j]-limits.lower_position[j]);
      }
      robot.setArmPosition(arm,q);robot.forward();
      const auto m=robot.armKinematicsAt(arm,q),p=kin.sample(arm,q);
      for(const auto* s:{&m,&p}) {
        EXPECT_LT((s->shoulder_position-g.shoulder_B).norm(),1e-5);
        const Eigen::Vector3d local=s->tcp_pose.rotation.transpose()*(s->wrist_position-s->tcp_pose.position);
        const double residual=(local-g.tcp_to_wrist_center.position).norm();
        EXPECT_LT(residual,1e-5);
        if(s==&m)max_vector=std::max(max_vector,residual);
        max_length=std::max({max_length,std::abs((s->elbow_position-s->shoulder_position).norm()-g.upper_length_m),
          std::abs((s->wrist_position-s->elbow_position).norm()-g.forearm_length_m)});
      }
      max_position=std::max({max_position,(m.shoulder_position-p.shoulder_position).norm(),
        (m.elbow_position-p.elbow_position).norm(),(m.wrist_position-p.wrist_position).norm(),(m.tcp_pose.position-p.tcp_pose.position).norm()});
      max_rotation=std::max(max_rotation,rotationDistance(m.tcp_pose.rotation,p.tcp_pose.rotation));
      const auto& ids=robot.mapping(arm).body_ids;
      for(int j:{5,6}) {
        const Eigen::Map<const Eigen::Vector3d> w(robot.data()->xpos+3*ids[4]);
        const Eigen::Map<const Eigen::Vector3d> b(robot.data()->xpos+3*ids[j]);
        max_origin=std::max(max_origin,(w-b).norm());
      }
      const auto t=robot.tcpRelativeToLink7(arm);
      EXPECT_LT((t.position-g.link7_to_tcp.position).norm(),1e-10);
      EXPECT_LT(rotationDistance(t.rotation,g.link7_to_tcp.rotation),1e-10);
      EXPECT_LT((g.link7_to_tcp.position+g.link7_to_tcp.rotation*g.tcp_to_wrist_center.position).norm(),1e-10);
    }
    EXPECT_LT(max_vector,1e-10);EXPECT_LT(max_origin,1e-10);
    EXPECT_LT(max_position,1e-5);EXPECT_LT(max_rotation,1e-5);EXPECT_LT(max_length,1e-5);
    std::cout<<std::setprecision(17)<<"R3_geometry profile="<<profile<<" side="<<side<<" samples=100 wrist_vector="<<max_vector
      <<" origin="<<max_origin<<" cross_position="<<max_position<<" cross_rotation="<<max_rotation
      <<" length="<<max_length<<'\n';
  }
  }
}

TEST(SharedRootGeometry, WristLocalVectorIsReferenceGeometryNotFixedExtrinsic) {
  MujocoRobot robot(TIANJI_PROJECT_SOURCE_DIR "/models/marvin_m6_wuji2_shared_root.xml");
  for (auto side : {ArmSide::kLeft, ArmSide::kRight}) {
    const auto zero = robot.armKinematicsAt(side, Vec7::Zero());
    Vec7 q = Vec7::Zero();
    q[5] = 0.5;
    const auto moved = robot.armKinematicsAt(side, q);
    const Eigen::Vector3d v0 = zero.wrist_rotation.transpose() *
        (zero.tcp_pose.position - zero.wrist_position);
    const Eigen::Vector3d v1 = moved.wrist_rotation.transpose() *
        (moved.tcp_pose.position - moved.wrist_position);
    EXPECT_GT((v1-v0).norm(), .05);
    EXPECT_NEAR(v0.norm(), v1.norm(), 1e-10);
  }
}
}  // namespace
}  // namespace tianji_qp_ik
