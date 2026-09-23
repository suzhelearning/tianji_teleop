#include "tianji_qp_ik/shared_root_closure.hpp"
#include "tianji_qp_ik/shared_root_options.hpp"
#include "tianji_qp_ik/pinocchio_arm_kinematics.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <iostream>
namespace tianji_qp_ik {
namespace {
SharedRootClosureSideGeometry simple() {
  SharedRootClosureSideGeometry g;g.upper_length_m=g.forearm_length_m=.5;
  g.tcp_to_wrist_center.position.setZero();return g;
}
TEST(SharedRootClosure, SphereCircleAndExactEndpoint) {
  auto g=simple();Pose p;p.position={.6,0,0};
  auto out=closeSharedRootArm(g,p,{.3,1,0});ASSERT_TRUE(out.valid());
  EXPECT_LT((out.target.elbow-Eigen::Vector3d(.3,.4,0)).norm(),1e-12);
  EXPECT_EQ(out.target.hand,p.position);EXPECT_EQ(out.target.palm.position,p.position);
}
TEST(SharedRootClosure, StraightAndFoldedBoundaries) {
  auto g=simple();Pose p;p.position={1,0,0};
  auto out=closeSharedRootArm(g,p,{0,0,0});ASSERT_TRUE(out.valid());
  EXPECT_LT((out.target.elbow-Eigen::Vector3d(.5,0,0)).norm(),1e-12);
  g.forearm_length_m=.3;p.position={.2,0,0};
  out=closeSharedRootArm(g,p,{0,1,0});ASSERT_TRUE(out.valid());
  EXPECT_NEAR((out.target.elbow-out.target.shoulder).norm(),.5,1e-9);
  EXPECT_NEAR((out.target.wrist-out.target.elbow).norm(),.3,1e-9);
}
TEST(SharedRootClosure, ProjectionAndCoincidentFallbackUseReadOnlyHistory) {
  auto g=simple();Pose p;p.position={.6,0,0};
  EXPECT_EQ(closeSharedRootArm(g,p,{.3,0,0}).status,ClosureStatus::kBranchUndetermined);
  const std::optional<Eigen::Vector3d> previous=Eigen::Vector3d(.3,-.4,0);
  const auto out=closeSharedRootArm(g,p,{.3,0,0},previous);ASSERT_TRUE(out.valid());
  EXPECT_LT(out.target.elbow.y(),0);EXPECT_EQ(*previous,Eigen::Vector3d(.3,-.4,0));
  p.position.setZero();
  EXPECT_EQ(closeSharedRootArm(g,p,{0,1,0}).status,ClosureStatus::kBranchUndetermined);
  EXPECT_TRUE(closeSharedRootArm(g,p,{0,1,0},previous).valid());
}
TEST(SharedRootClosure, NoProjectionNoStretchAndNonfiniteRejected) {
  auto g=simple();Pose p;p.position={1.01,0,0};
  EXPECT_EQ(closeSharedRootArm(g,p,{0,1,0}).status,ClosureStatus::kOutsideWorkspace);
  g.forearm_length_m=.2;p.position={.1,0,0};
  EXPECT_EQ(closeSharedRootArm(g,p,{0,1,0}).status,ClosureStatus::kOutsideWorkspace);
  p.position[0]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(closeSharedRootArm(g,p,{0,1,0}).status,ClosureStatus::kInvalidInput);
}
TEST(SharedRootClosure, FrozenRobotFkPosesReconstructBothSides) {
  const auto o=loadSharedRootOptions(TIANJI_PROJECT_SOURCE_DIR "/config/qp_ik_pico_shared_root_dls.yaml");
  MujocoRobot robot(o.mujoco_path);
  double max_elbow_projection=0;
  for(int side=0;side<2;++side) {
    auto arm=side?ArmSide::kRight:ArmSide::kLeft;
    const auto& limits=robot.mapping(arm).limits;
    for(int k=0;k<100;++k) {
      Vec7 q;
      for(int j=0;j<7;++j)q[j]=limits.lower_position[j]+(.5+.49*std::sin(double((k+1)*(j+1))*.6180339887498948))*(limits.upper_position[j]-limits.lower_position[j]);
      const auto s=robot.armKinematicsAt(arm,q);
      const auto out=closeSharedRootArm(o.closure_geometry[side],s.tcp_pose,s.elbow_position);
      ASSERT_TRUE(out.valid())<<"side="<<side<<" sample="<<k;
      // The FK elbow is a preference, not a hard constraint: fixed reference
      // lengths differ slightly from model samples, amplified near extension.
      // Assert the closed geometry below; report projection displacement rather
      // than replacing the strict bone/endpoint tolerance with a looser one.
      max_elbow_projection=std::max(max_elbow_projection,(out.target.elbow-s.elbow_position).norm());
      EXPECT_LT((out.target.wrist-s.wrist_position).norm(),1e-9);
      EXPECT_NEAR((out.target.elbow-out.target.shoulder).norm(),o.closure_geometry[side].upper_length_m,1e-9);
      EXPECT_NEAR((out.target.wrist-out.target.elbow).norm(),o.closure_geometry[side].forearm_length_m,1e-9);
      EXPECT_EQ(out.target.hand,s.tcp_pose.position);
    }
  }
  std::cout<<"R3_FK_preference_projection_max_m="<<max_elbow_projection<<'\n';
}
} // namespace
} // namespace tianji_qp_ik
