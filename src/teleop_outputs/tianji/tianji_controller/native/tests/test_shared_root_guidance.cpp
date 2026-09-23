#include "tianji_qp_ik/shared_root_guidance.hpp"
#include "tianji_qp_ik/so3.hpp"
#include "tianji_qp_ik/controller.hpp"
#include "tianji_qp_ik/pico_teleop_session.hpp"
#include "tianji_qp_ik/simulation_recovery.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
#include <Eigen/SVD>

namespace tianji_qp_ik {
namespace {
constexpr auto kXml=TIANJI_MODEL_DIR "/marvin_m6_wuji2_shared_root_ceres.xml";
constexpr auto kProfile=TIANJI_PROJECT_SOURCE_DIR "/config/qp_ik_pico_shared_root_dls.yaml";
class SharedRootGuidanceTest : public ::testing::Test {
 protected:
  SharedRootGuidanceTest():robot(kXml),config(loadConfig(kProfile)),options(loadSharedRootOptions(kProfile)) {
    options.enabled=true;
    for(auto side:{ArmSide::kLeft,ArmSide::kRight}) {
      const auto& limits=robot.mapping(side).limits;
      auto& state=side==ArmSide::kLeft?left:right;
      state.q=configuredInitialPosture(config.controller,limits,side);
      robot.setArmPosition(side,state.q);
    }
    robot.forward();
  }
  PicoTeleopFrame frame(std::uint64_t n,const ArmMotionState& l,const ArmMotionState& r) {
    PicoTeleopFrame f;f.sequence=n;f.tracking_epoch=1;
    f.source_timestamp_ns=static_cast<std::int64_t>(n)*10000000;
    f.receive_monotonic_ns=1000000000+f.source_timestamp_ns;
    auto& s=f.upper_limb_skeleton;s.valid=s.rotations_valid=true;
    for(std::size_t i=0;i<2;++i) {
      const auto side=i==0?ArmSide::kLeft:ArmSide::kRight;
      const auto sample=robot.armKinematicsAt(side,i==0?l.q:r.q);
      s.points[i*4]=sample.shoulder_position;s.points[i*4+1]=sample.elbow_position;
      s.points[i*4+2]=sample.wrist_position;s.points[i*4+3]=sample.tcp_pose.position;
      s.rotations[i*4]=Eigen::Quaterniond(sample.shoulder_rotation);
      s.rotations[i*4+1]=Eigen::Quaterniond(sample.elbow_rotation);
      s.rotations[i*4+2]=Eigen::Quaterniond(sample.wrist_rotation);
      Eigen::Matrix3d basis;
      basis.col(0)=(i==0?-1.:1.)*Eigen::Vector3d::UnitY();
      basis.col(1)=(i==0?-1.:1.)*Eigen::Vector3d::UnitZ();
      basis.col(2)=Eigen::Vector3d::UnitX();
      s.rotations[i*4+3]=Eigen::Quaterniond(sample.tcp_pose.rotation*basis.transpose());
    }
    // Header poses belong to another mapping and must never reach shared IK.
    f.left.position=f.right.position=Eigen::Vector3d::Constant(99);
    return f;
  }
  SharedRootGuidanceDiagnostics tick(SharedRootGuidance& g,std::uint64_t n) {
    auto f=frame(n,left,right);
    EXPECT_TRUE(g.updateSharedRootFrame(f,f.receive_monotonic_ns));
    const auto out=g.stepSharedRoot(left,right,.01,f.receive_monotonic_ns,true,true);
    if(out.target_valid)for(int side=0;side<2;++side) {
      // The candidate always obeys the frozen closed geometry. During settled
      // hold the per-arm diagnostic instead describes actual model FK.
      const auto& diagnostic=side?out.right.target:out.left.target;
      EXPECT_EQ(diagnostic.hand,diagnostic.palm.position);
      const auto& target=side?g.latestTargets().right:g.latestTargets().left;
      EXPECT_EQ(target.hand,target.palm.position);
      EXPECT_NEAR((target.elbow-target.shoulder).norm(),options.closure_geometry[side].upper_length_m,1e-9);
      EXPECT_NEAR((target.wrist-target.elbow).norm(),options.closure_geometry[side].forearm_length_m,1e-9);
    }
    return out;
  }
  MujocoRobot robot;QpIkConfig config;SharedRootOptions options;ArmMotionState left,right;
};
TEST_F(SharedRootGuidanceTest, ReachableFinalTargetRequiresCurrentOuterAcknowledgement) {
  SharedRootGuidance g(robot,config,options.urdf_path,&options);
  SharedRootGuidanceDiagnostics out;
  for(std::uint64_t n=1;n<=25;++n)out=tick(g,n);
  ASSERT_TRUE(out.target_valid);ASSERT_TRUE(out.accepted);
  EXPECT_EQ(out.shared_root_state,SharedRootState::kRecovering);
  EXPECT_LT((out.left.target.palm.position-robot.armKinematicsAt(ArmSide::kLeft,left.q).tcp_pose.position).norm(),1e-5);
  EXPECT_FALSE(g.confirmSharedRootReference(out.shared_root_cycle-1,true));
  EXPECT_FALSE(g.confirmSharedRootReference(out.shared_root_cycle,false));
  EXPECT_FALSE(g.confirmSharedRootReference(out.shared_root_cycle,true));
  out=tick(g,26);
  ASSERT_TRUE(g.confirmSharedRootReference(out.shared_root_cycle,true));
  EXPECT_FALSE(g.confirmSharedRootReference(out.shared_root_cycle,true));
  EXPECT_EQ(tick(g,27).shared_root_state,SharedRootState::kTracking);
}

TEST_F(SharedRootGuidanceTest, MappingKeepsBilateralAckAndAuthorization) {
  SharedRootGuidance g(robot,config,options.urdf_path,&options);
  SharedRootGuidanceDiagnostics out;
  for(std::uint64_t n=1;n<=25;++n)out=tick(g,n);
  ASSERT_TRUE(out.target_valid);
  EXPECT_EQ(out.cartesian_targets.left.position,g.latestTargets().left.palm.position);
  EXPECT_FALSE(g.confirmSharedRootReference(out.shared_root_cycle,false));
  out=tick(g,26);
  EXPECT_FALSE(g.confirmSharedRootReference(out.shared_root_cycle-1,true));
  EXPECT_TRUE(g.confirmSharedRootReference(out.shared_root_cycle,true));
  EXPECT_FALSE(g.confirmSharedRootReference(out.shared_root_cycle,true));
  out=g.stepSharedRoot(left,right,.01,1260000000,false,true);
  EXPECT_FALSE(out.accepted);EXPECT_FALSE(out.target_valid);
  EXPECT_FALSE(g.confirmSharedRootReference(out.shared_root_cycle,true));
}

TEST_F(SharedRootGuidanceTest, LostInputWithStationaryReturnCannotLatchOldHold) {
  SharedRootGuidance g(robot,config,options.urdf_path,&options);
  SharedRootGuidanceDiagnostics out;
  for(std::uint64_t n=1;n<=25;++n)out=tick(g,n);
  ASSERT_TRUE(g.confirmSharedRootReference(out.shared_root_cycle,true));
  out=g.stepSharedRoot(left,right,.01,1450000000,true,true);
  EXPECT_FALSE(out.target_valid);
  // Human changes pose while packets are absent, then remains stationary.
  // The robot/model snapshot is still the pre-gap pose.
  auto moved_left=left,moved_right=right;
  moved_left.q[3]+=.03;moved_right.q[3]+=.03;
  for(std::uint64_t n=50;n<=75;++n) {
    auto f=frame(n,moved_left,moved_right);
    ASSERT_TRUE(g.updateSharedRootFrame(f,f.receive_monotonic_ns));
    out=g.stepSharedRoot(left,right,.01,f.receive_monotonic_ns,true,true);
  }
  ASSERT_TRUE(out.target_valid);ASSERT_TRUE(out.left.accepted);
  ASSERT_TRUE(out.right.accepted);
  EXPECT_LT((out.left.target.palm.position-robot.armKinematicsAt(ArmSide::kLeft,moved_left.q).tcp_pose.position).norm(),1e-5);
  EXPECT_TRUE(g.confirmSharedRootReference(out.shared_root_cycle,true));
}

TEST_F(SharedRootGuidanceTest, DlsControllerAcceptsTheSameRecoveryCycle) {
  SharedRootGuidance g(robot,config,options.urdf_path,&options);
  DualArmController controller(robot,config);
  ASSERT_TRUE(controller.synchronizeReferencesToActual());
  bool completed=false;
  for(std::uint64_t n=1;n<=40;++n) {
    auto f=frame(n,left,right);ASSERT_TRUE(g.updateSharedRootFrame(f,f.receive_monotonic_ns));
    auto out=g.stepSharedRoot(controller.referenceState(ArmSide::kLeft),
        controller.referenceState(ArmSide::kRight),.01,f.receive_monotonic_ns,true,true);
    if(!out.target_valid)continue;
    ASSERT_TRUE(out.accepted);
    const auto accepted=controller.step(out.cartesian_targets,.005);
    ASSERT_TRUE(accepted.accepted) << static_cast<int>(accepted.hold_reason);
    const bool both=accepted.left.accepted&&accepted.right.accepted;
    completed=g.confirmSharedRootReference(out.shared_root_cycle,both)||completed;
  }
  EXPECT_TRUE(completed);
}

TEST_F(SharedRootGuidanceTest, ExistingSessionEpochAndPauseDoNotDeadlockRecovery) {
  SharedRootGuidance g(robot,config,options.urdf_path,&options);
  PicoTeleopSession session(.1);session.setEnabled(true);
  SharedRootGuidanceDiagnostics out;std::size_t resets=0;
  const auto receive=[&](std::uint64_t n) {
    auto f=frame(n,left,right);
    const auto classification=session.classify(f,f.receive_monotonic_ns);
    if(classification.action==PicoTeleopAction::kResetEpochAndApply) {
      ASSERT_TRUE(g.reset(left,right));++resets;
    }
    if(classification.action==PicoTeleopAction::kResetEpochAndApply||
        classification.action==PicoTeleopAction::kApply) {
      if(g.updateSharedRootFrame(f,f.receive_monotonic_ns))session.commitApplied(f);
    }
    out=g.stepSharedRoot(left,right,.01,f.receive_monotonic_ns,session.enabled(),true);
  };
  for(std::uint64_t n=1;n<=25;++n)receive(n);
  ASSERT_EQ(resets,1U);ASSERT_TRUE(out.target_valid);
  EXPECT_TRUE(g.confirmSharedRootReference(out.shared_root_cycle,true));
  session.setEnabled(false);
  for(std::uint64_t n=26;n<=30;++n) {receive(n);EXPECT_FALSE(out.target_valid);}
  session.setEnabled(true);
  for(std::uint64_t n=31;n<=55;++n)receive(n);
  EXPECT_EQ(resets,2U);EXPECT_TRUE(out.target_valid);
  EXPECT_TRUE(session.freshness(1550000000).live);
  EXPECT_TRUE(g.confirmSharedRootReference(out.shared_root_cycle,true));
}

TEST_F(SharedRootGuidanceTest, RecoveryStartUsesCurrentSnapshot) {
  SharedRootGuidance g(robot,config,options.urdf_path,&options);
  for(std::uint64_t n=1;n<=25;++n)tick(g,n);
  g.stepSharedRoot(left,right,.01,1450000000,true,true);
  auto new_left=left,new_right=right;new_left.q[3]+=.04;new_right.q[3]+=.04;
  SharedRootGuidanceDiagnostics out;
  for(std::uint64_t n=50;n<=54;++n) {
    auto f=frame(n,left,right);ASSERT_TRUE(g.updateSharedRootFrame(f,f.receive_monotonic_ns));
    out=g.stepSharedRoot(new_left,new_right,.01,f.receive_monotonic_ns,true,true);
  }
  ASSERT_TRUE(out.target_valid);
  EXPECT_LT((out.left.target.palm.position-robot.armKinematicsAt(ArmSide::kLeft,new_left.q).tcp_pose.position).norm(),1e-5);
}

TEST_F(SharedRootGuidanceTest, ExplicitModelValidityAndAuthorityAreMandatory) {
  SharedRootGuidance g(robot,config,options.urdf_path,&options);
  for(std::uint64_t n=1;n<=25;++n)tick(g,n);
  EXPECT_FALSE(g.stepSharedRoot(left,right,.01,1250000000,true,false).accepted);
  EXPECT_FALSE(g.stepSharedRoot(left,right,.01,1250000000,false,true).accepted);
  EXPECT_FALSE(g.stepSharedRoot(left,right,.01,1250000000,true,true).target_valid);
  auto nan=left;nan.qdot[0]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(g.stepSharedRoot(nan,right,.01,1250000000,true,true).accepted);
}

TEST_F(SharedRootGuidanceTest, RejectsWrongModelTcp) {
  MujocoRobot other(TIANJI_MODEL_DIR "/marvin_m6_qp_test.xml");
  EXPECT_THROW((SharedRootGuidance(other,config,options.urdf_path,&options)),std::invalid_argument);
}

} // namespace
} // namespace tianji_qp_ik
