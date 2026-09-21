#include "tianji_qp_ik/spark_guidance.hpp"
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
constexpr auto kXml=TIANJI_MODEL_DIR "/marvin_m6_wuji2_shared_root.xml";
constexpr auto kUrdf=TIANJI_MODEL_DIR "/marvin_m6_s_ccs_696_v4_local.urdf";
constexpr auto kProfile=TIANJI_PROJECT_SOURCE_DIR "/config/qp_ik_pico_shared_root.yaml";
constexpr auto kMode=SparkPostureGuideMode::kHeadroomFeedforwardJointReferenceVelocity;
double quantile(std::vector<double> values,double p) {
  if(values.empty())return std::numeric_limits<double>::infinity();
  std::sort(values.begin(),values.end());
  return values[static_cast<std::size_t>(std::ceil(p*double(values.size())))-1];
}
class SharedRootGuidance : public ::testing::Test {
 protected:
  SharedRootGuidance():robot(kXml),config(loadConfig(kProfile)),options(loadSharedRootOptions(kProfile)) {
    // Relax only wall-clock solve budget for offline correctness tests; this is
    // not the shipped profile nor a real-time performance qualification.
    config.spark_upper_qpoases.ik_cycle_budget_seconds=.2;
    options.enabled=true;
    for(auto side:{ArmSide::kLeft,ArmSide::kRight}) {
      const auto& limits=robot.mapping(side).limits;
      auto& state=side==ArmSide::kLeft?left:right;
      state.q=.5*(limits.lower_position+limits.upper_position);
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
  SparkGuidanceDiagnostics tick(DualArmSparkGuidance& g,std::uint64_t n) {
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
TEST_F(SharedRootGuidance, DirectIkHomeDoesNotUseUnusedSparkAccelerationEnvelope) {
  for (auto algorithm : {IkAlgorithm::kPicoEeFrankaDls, IkAlgorithm::kPicoEeFrankaCeresLm}) {
    config.ik_algorithm = algorithm;
    auto& cfg = algorithm == IkAlgorithm::kPicoEeFrankaDls
        ? config.pico_ee_franka_dls.post_smoothing
        : config.pico_ee_franka_ceres_lm.post_smoothing;
    cfg.velocity_scale = 1.;
    cfg.max_velocity_rad_s = Vec7::Constant(4.);
    cfg.max_acceleration_rad_s2 = Vec7::Constant(90.);
    cfg.max_jerk_rad_s3 = Vec7::Constant(4500.);
    config.dls_posture_ruckig.max_acceleration_rad_s2 = Vec7::Constant(15.708);
    DualArmSparkGuidance g(robot,config,kUrdf,kMode,&options);
    const SimulationRecovery::Pair home{left,right};
    auto moving = home;
    moving[1].qdot[3] = -.185651879613;
    moving[1].qddot[3] = 40.8762390211;
    EXPECT_FALSE(g.reset(moving[0],moving[1])); // Original failing path.
    SimulationRecovery recovery(config,{robot.mapping(ArmSide::kLeft).limits,
        robot.mapping(ArmSide::kRight).limits},home,.005);
    ASSERT_TRUE(recovery.stop(moving,true));
    ASSERT_TRUE(g.resetMappingSession(moving[0],moving[1]));
    EXPECT_FALSE(g.latestTargets().valid);
    for(int i=0;i<12000 && recovery.phase()!=SimulationRecovery::Phase::kHomeReached;++i) {
      moving=recovery.update(moving);
      ASSERT_NE(recovery.phase(),SimulationRecovery::Phase::kFault);
    }
    EXPECT_EQ(recovery.phase(),SimulationRecovery::Phase::kHomeReached);
    EXPECT_FALSE(recovery.teleop());
    moving[1].qddot[3]=100.;
    EXPECT_FALSE(recovery.stop(moving,true)); // DLS/Ceres safety remains authoritative.
    moving[1].q[0]=std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(g.resetMappingSession(moving[0],moving[1]));
  }
}
TEST_F(SharedRootGuidance, MappingOnlyResetCannotBypassSparkReferenceChecks) {
  DualArmSparkGuidance g(robot,config,kUrdf,kMode,&options);
  EXPECT_FALSE(g.resetMappingSession(left,right));
}
TEST_F(SharedRootGuidance, ReachableFinalTargetRequiresCurrentOuterAcknowledgement) {
  DualArmSparkGuidance g(robot,config,kUrdf,kMode,&options);
  SparkGuidanceDiagnostics out;
  for(std::uint64_t n=1;n<=25;++n)out=tick(g,n);
  ASSERT_TRUE(out.target_valid);ASSERT_TRUE(out.accepted);
  EXPECT_EQ(out.shared_root_state,SharedRootState::kRecovering);
  EXPECT_FALSE(out.left.stationary_joint_reference_held);
  EXPECT_FALSE(out.right.stationary_joint_reference_held);
  EXPECT_FALSE(out.left.settled_hold_active);
  EXPECT_FALSE(out.right.settled_hold_active);
  ASSERT_TRUE(out.left.ik.accepted);ASSERT_TRUE(out.right.ik.accepted);
  EXPECT_LT((out.left.target.palm.position-robot.armKinematicsAt(ArmSide::kLeft,left.q).tcp_pose.position).norm(),1e-5);
  EXPECT_FALSE(g.confirmSharedRootReference(out.shared_root_cycle-1,true));
  EXPECT_FALSE(g.confirmSharedRootReference(out.shared_root_cycle,false));
  EXPECT_FALSE(g.confirmSharedRootReference(out.shared_root_cycle,true));
  out=tick(g,26);
  ASSERT_TRUE(g.confirmSharedRootReference(out.shared_root_cycle,true));
  EXPECT_FALSE(g.confirmSharedRootReference(out.shared_root_cycle,true));
  EXPECT_EQ(tick(g,27).shared_root_state,SharedRootState::kTracking);
}
// The mapper cannot commit a Ceres candidate without this cycle's outer ack.
TEST_F(SharedRootGuidance, CeresMappingOnlyKeepsBilateralAckAndAuthorization) {
  config.ik_algorithm=IkAlgorithm::kPicoEeFrankaCeresLm;
  DualArmSparkGuidance g(robot,config,kUrdf,SparkPostureGuideMode::kRuckig,&options);
  SparkGuidanceDiagnostics out;
  for(std::uint64_t n=1;n<=25;++n)out=tick(g,n);
  ASSERT_TRUE(out.target_valid);
  EXPECT_FALSE(out.left.ik.accepted); // no hidden SPARK solve
  EXPECT_FALSE(out.feedforward_references_valid);
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
TEST_F(SharedRootGuidance, MovingRecoveryDoesNotUseUnusedRuckigEnvelope) {
  DualArmSparkGuidance g(robot,config,kUrdf,kMode,&options);
  left.qdot[0]=right.qdot[0]=config.dls_posture_ruckig.max_velocity_rad_s[0]+.01;
  ASSERT_LT(left.qdot[0],robot.mapping(ArmSide::kLeft).limits.velocity[0]);
  bool valid_target=false;
  for(std::uint64_t n=1;n<=25;++n) {
    const auto out=tick(g,n);
    EXPECT_NE(out.detail,"shared_root_reference_reset_failed");
    valid_target=valid_target||out.target_valid;
  }
  EXPECT_TRUE(valid_target);
}
TEST_F(SharedRootGuidance, JointBoundaryTargetsStayFiniteAndInsideSafeLimits) {
  // Known model poses near each joint boundary; not a claim that every palm
  // remains exactly reachable after the shared anisotropic map.
  std::size_t checked=0;
  for(int joint=0;joint<7;++joint)for(bool upper:{false,true}) {
    DualArmSparkGuidance g(robot,config,kUrdf,kMode,&options);
    auto target_left=left,target_right=right;
    for(auto side:{ArmSide::kLeft,ArmSide::kRight}) {
      const auto& limits=robot.mapping(side).limits;
      auto& q=side==ArmSide::kLeft?target_left.q:target_right.q;
      q[joint]=upper?limits.upper_position[joint]-.06:limits.lower_position[joint]+.06;
    }
    for(std::uint64_t n=1;n<=30;++n) {
      const auto f=frame(n,target_left,target_right);
      g.updateSharedRootFrame(f,f.receive_monotonic_ns);
      const auto out=g.stepSharedRoot(left,right,.01,f.receive_monotonic_ns,true,true);
      for(auto side:{ArmSide::kLeft,ArmSide::kRight}) {
        const auto& ik=side==ArmSide::kLeft?out.left.ik:out.right.ik;
        if(!ik.accepted)continue;
        ++checked;const auto& limits=robot.mapping(side).limits;
        ASSERT_TRUE(ik.q.allFinite());
        EXPECT_TRUE((ik.q.array()>=limits.lower_position.array()+config.spark_upper_qpoases.joint_limit_margin_rad-1e-9).all());
        EXPECT_TRUE((ik.q.array()<=limits.upper_position.array()-config.spark_upper_qpoases.joint_limit_margin_rad+1e-9).all());
      }
    }
  }
  EXPECT_GT(checked,0U);
}
TEST_F(SharedRootGuidance, IllConditionedStraightArmObservationRemainsBounded) {
  auto straight_left=left,straight_right=right;
  straight_left.q.setZero();straight_right.q.setZero();
  straight_left.q[1]=straight_right.q[1]=-std::acos(-1.)/2;
  for(auto side:{ArmSide::kLeft,ArmSide::kRight}) {
    const auto& q=side==ArmSide::kLeft?straight_left.q:straight_right.q;
    const auto sample=robot.armKinematicsAt(side,q);
    Eigen::MatrixXd scaled=sample.tcp_jacobian;
    scaled.topRows(3)/=options.morphology.robot_reach_m;
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(scaled);
    ASSERT_EQ(svd.info(),Eigen::Success);
    const double sigma_min=svd.singularValues().minCoeff();
    const double ratio=sigma_min/svd.singularValues().maxCoeff();
    std::cout<<"straight_arm side="<<(side==ArmSide::kLeft?"left":"right")
             <<" reach_normalized_sigma_min="<<sigma_min<<" sigma_ratio="<<ratio<<'\n';
    ASSERT_LT(ratio,.05); // Ill-conditioned, not claimed to be exactly rank deficient.
  }
  DualArmSparkGuidance g(robot,config,kUrdf,kMode,&options);
  std::size_t accepted=0;
  for(std::uint64_t n=1;n<=40;++n) {
    const auto f=frame(n,straight_left,straight_right);
    ASSERT_TRUE(g.updateSharedRootFrame(f,f.receive_monotonic_ns));
    const auto out=g.stepSharedRoot(left,right,.01,f.receive_monotonic_ns,true,true);
    for(auto side:{ArmSide::kLeft,ArmSide::kRight}) {
      const auto& ik=side==ArmSide::kLeft?out.left.ik:out.right.ik;
      if(!ik.accepted)continue;
      ++accepted;const auto& limits=robot.mapping(side).limits;
      ASSERT_TRUE(ik.q.allFinite());
      EXPECT_TRUE((ik.q.array()>=limits.lower_position.array()+config.spark_upper_qpoases.joint_limit_margin_rad-1e-9).all());
      EXPECT_TRUE((ik.q.array()<=limits.upper_position.array()-config.spark_upper_qpoases.joint_limit_margin_rad+1e-9).all());
    }
  }
  EXPECT_GT(accepted,0U);
}
TEST_F(SharedRootGuidance, LostInputWithStationaryReturnCannotLatchOldHold) {
  DualArmSparkGuidance g(robot,config,kUrdf,kMode,&options);
  SparkGuidanceDiagnostics out;
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
  ASSERT_TRUE(out.target_valid);ASSERT_TRUE(out.left.ik.accepted);
  ASSERT_TRUE(out.right.ik.accepted);EXPECT_FALSE(out.left.settled_hold_active);
  EXPECT_FALSE(out.left.stationary_joint_reference_held);
  EXPECT_LT((out.left.target.palm.position-robot.armKinematicsAt(ArmSide::kLeft,moved_left.q).tcp_pose.position).norm(),1e-5);
  EXPECT_TRUE(g.confirmSharedRootReference(out.shared_root_cycle,true));
}
TEST_F(SharedRootGuidance, OuterVelocityQpAcceptsTheSameRecoveryCycle) {
  DualArmSparkGuidance g(robot,config,kUrdf,kMode,&options);
  DualArmController controller(robot,config);
  ASSERT_TRUE(controller.synchronizeReferencesToActual());
  bool completed=false;
  for(std::uint64_t n=1;n<=40;++n) {
    auto f=frame(n,left,right);ASSERT_TRUE(g.updateSharedRootFrame(f,f.receive_monotonic_ns));
    auto out=g.stepSharedRoot(controller.referenceState(ArmSide::kLeft),
        controller.referenceState(ArmSide::kRight),.01,f.receive_monotonic_ns,true,true);
    if(!out.target_valid)continue;
    ASSERT_TRUE(out.accepted);
    const auto accepted=controller.step(out.cartesian_references,{},out.posture_tasks,.01);
    ASSERT_TRUE(accepted.accepted) << static_cast<int>(accepted.hold_reason);
    const bool both=accepted.left.accepted&&accepted.right.accepted;
    completed=g.confirmSharedRootReference(out.shared_root_cycle,both)||completed;
  }
  EXPECT_TRUE(completed);
}
TEST_F(SharedRootGuidance, ExistingSessionEpochAndPauseDoNotDeadlockRecovery) {
  DualArmSparkGuidance g(robot,config,kUrdf,kMode,&options);
  PicoTeleopSession session(.1);session.setEnabled(true);
  SparkGuidanceDiagnostics out;std::size_t resets=0;
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
TEST_F(SharedRootGuidance, RecoveryStartAndIkSeedUseCurrentSnapshot) {
  DualArmSparkGuidance g(robot,config,kUrdf,kMode,&options);
  for(std::uint64_t n=1;n<=25;++n)tick(g,n);
  g.stepSharedRoot(left,right,.01,1450000000,true,true);
  auto new_left=left,new_right=right;new_left.q[3]+=.04;new_right.q[3]+=.04;
  SparkGuidanceDiagnostics out;
  for(std::uint64_t n=50;n<=54;++n) {
    auto f=frame(n,left,right);ASSERT_TRUE(g.updateSharedRootFrame(f,f.receive_monotonic_ns));
    out=g.stepSharedRoot(new_left,new_right,.01,f.receive_monotonic_ns,true,true);
  }
  ASSERT_TRUE(out.target_valid);EXPECT_DOUBLE_EQ(out.blend_progress,0);
  EXPECT_LT((out.left.target.palm.position-robot.armKinematicsAt(ArmSide::kLeft,new_left.q).tcp_pose.position).norm(),1e-5);
  EXPECT_LT((out.left.ik.q-new_left.q).norm(),1e-4);
  EXPECT_LT((out.right.ik.q-new_right.q).norm(),1e-4);
}
TEST_F(SharedRootGuidance, ExplicitModelValidityAndAuthorityAreMandatory) {
  DualArmSparkGuidance g(robot,config,kUrdf,kMode,&options);
  for(std::uint64_t n=1;n<=25;++n)tick(g,n);
  EXPECT_FALSE(g.stepSharedRoot(left,right,.01,1250000000,true,false).accepted);
  EXPECT_FALSE(g.stepSharedRoot(left,right,.01,1250000000,false,true).accepted);
  EXPECT_FALSE(g.stepSharedRoot(left,right,.01,1250000000,true,true).target_valid);
  auto nan=left;nan.qdot[0]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(g.stepSharedRoot(nan,right,.01,1250000000,true,true).accepted);
  EXPECT_FALSE(g.step(left,right,.01).accepted);
  EXPECT_THROW(g.updatePicoFrame(frame(26,left,right)),std::logic_error);
}
TEST_F(SharedRootGuidance, DisabledIsDirectLegacyWithoutArtifactAccess) {
  SharedRootOptions disabled;disabled.profile_path="must_not_be_read";
  DualArmSparkGuidance a(robot,config,kUrdf,kMode),b(robot,config,kUrdf,kMode,&disabled);
  for(std::uint64_t n=1;n<=10;++n) {
    auto f=frame(n,left,right);f.left=robot.armKinematicsAt(ArmSide::kLeft,left.q).tcp_pose;
    f.right=robot.armKinematicsAt(ArmSide::kRight,right.q).tcp_pose;
    EXPECT_EQ(a.updatePicoFrame(f).left.palm.position,b.updatePicoFrame(f).left.palm.position);
    auto x=a.step(left,right,.01),y=b.step(left,right,.01);
    EXPECT_EQ(x.accepted,y.accepted);EXPECT_EQ(x.left.q_ik,y.left.q_ik);
    EXPECT_EQ(x.right.q_ik,y.right.q_ik);
    EXPECT_EQ(x.cartesian_targets.left.position,y.cartesian_targets.left.position);
    EXPECT_EQ(x.shared_root_cycle,0U);EXPECT_EQ(y.shared_root_cycle,0U);
  }
}
TEST_F(SharedRootGuidance, CannotMixWrongPostureModeOrWrongModelTcp) {
  EXPECT_THROW((DualArmSparkGuidance(robot,config,kUrdf,SparkPostureGuideMode::kRuckig,&options)),std::invalid_argument);
  MujocoRobot other(TIANJI_MODEL_DIR "/marvin_m6_qp_test.xml");
  EXPECT_THROW((DualArmSparkGuidance(other,config,kUrdf,kMode,&options)),std::invalid_argument);
}
TEST_F(SharedRootGuidance, SettledHoldReleaseUsesSharedRecoveryInsteadOfLegacyBlend) {
  config.spark_headroom_feedforward_velocity_qp.settled_hold_dwell_seconds=.01;
  DualArmSparkGuidance g(robot,config,kUrdf,kMode,&options);
  SparkGuidanceDiagnostics out;
  for(std::uint64_t n=1;n<=25;++n)out=tick(g,n);
  ASSERT_TRUE(g.confirmSharedRootReference(out.shared_root_cycle,true));
  SparkConstraintHeadroomFeedback exhausted;exhausted.accepted=true;
  exhausted.task_scale_position=exhausted.task_scale_orientation=.75;
  for(int i=0;i<200;++i)g.updateHeadroomFeedback(exhausted,exhausted,.005);
  for(std::uint64_t n=26;n<=30;++n)out=tick(g,n);
  ASSERT_TRUE(out.left.settled_hold_active);ASSERT_TRUE(out.right.settled_hold_active);
  bool recovering=false;
  auto moved_left=left,moved_right=right;
  for(std::uint64_t n=31;n<=70;++n) {
    const double delta=.02*double(std::min<std::uint64_t>(n-30,5));
    moved_left.q[3]=left.q[3]+delta;moved_right.q[3]=right.q[3]+delta;
    auto f=frame(n,moved_left,moved_right);
    ASSERT_TRUE(g.updateSharedRootFrame(f,f.receive_monotonic_ns));
    out=g.stepSharedRoot(left,right,.01,f.receive_monotonic_ns,true,true);
    recovering=recovering||out.shared_root_state==SharedRootState::kRecovering;
  }
  EXPECT_TRUE(recovering);EXPECT_FALSE(out.left.settled_hold_active);
  EXPECT_FALSE(out.right.stationary_joint_reference_held);
  EXPECT_TRUE(g.confirmSharedRootReference(out.shared_root_cycle,true));
}
TEST_F(SharedRootGuidance, SyntheticSameTraceThreeWayIkMetrics) {
  // Deterministic known-reachable FK trace, not a device recording or dynamics
  // simulation. No mj_step; actual-feedback metrics are unavailable.
  for(int mode=0;mode<3;++mode) {
    auto candidate_config=config;
    if(mode==2) {
      candidate_config.spark_upper_qpoases.stage2_elbow_position_weight=.25;
      candidate_config.spark_upper_qpoases.stage2_wrist_position_weight=.25;
    }
    DualArmSparkGuidance g(robot,candidate_config,kUrdf,kMode,mode==0?nullptr:&options);
    std::vector<double> position,orientation,relation,relative_rotation;
    std::size_t rejected=0,budgets=0;
    for(std::uint64_t n=1;n<=160;++n) {
      auto l=left,r=right;
      const double phase=double(n)*.035;
      l.q[0]+=.035*std::sin(phase);l.q[1]+=.025*std::sin(.7*phase);
      r.q[0]+=.025*std::sin(phase+.4);r.q[1]+=.035*std::sin(.7*phase-.3);
      auto f=frame(n,l,r);
      f.left=robot.armKinematicsAt(ArmSide::kLeft,l.q).tcp_pose;
      f.right=robot.armKinematicsAt(ArmSide::kRight,r.q).tcp_pose;
      SparkGuidanceDiagnostics d;
      if(mode==0) {g.updatePicoFrame(f);d=g.step(left,right,.01);}
      else {g.updateSharedRootFrame(f,f.receive_monotonic_ns);
        d=g.stepSharedRoot(left,right,.01,f.receive_monotonic_ns,true,true);}
      // Do not fabricate external controller acceptance in this IK-only test.
      // Shared remains RECOVERING, with alpha=1 and ordinary holds suppressed.
      if(n<=30)continue;
      budgets+=d.left.ik.budget_exhausted||d.right.ik.budget_exhausted;
      if(!d.target_valid||!d.left.ik.accepted||!d.right.ik.accepted) {++rejected;continue;}
      const auto lp=robot.armKinematicsAt(ArmSide::kLeft,d.left.ik.q).tcp_pose;
      const auto rp=robot.armKinematicsAt(ArmSide::kRight,d.right.ik.q).tcp_pose;
      for(const auto& pair:{std::pair{lp,d.left.target.palm},std::pair{rp,d.right.target.palm}}) {
        position.push_back((pair.first.position-pair.second.position).norm());
        orientation.push_back(rotationDistance(pair.first.rotation,pair.second.rotation));
      }
      relation.push_back(((rp.position-lp.position)-(d.right.target.palm.position-d.left.target.palm.position)).norm());
      relative_rotation.push_back(rotationDistance(lp.rotation.transpose()*rp.rotation,
          d.left.target.palm.rotation.transpose()*d.right.target.palm.rotation));
    }
    std::cout<<"shared_root_synthetic_metrics mode="<<mode<<" samples="<<relation.size()
      <<" excluded="<<rejected<<" budget_exhausted="<<budgets
      <<" palm_p90_m="<<quantile(position,.9)<<" orientation_p90_rad="<<quantile(orientation,.9)
      <<" B_relation_p90_m="<<quantile(relation,.9)<<" relative_rotation_p90_rad="<<quantile(relative_rotation,.9)
      <<" actual=unavailable\n";
    EXPECT_EQ(rejected,0U);EXPECT_EQ(budgets,0U);
    if(mode>0) {
      EXPECT_LE(quantile(position,.9),.01);EXPECT_LE(quantile(orientation,.9),.10);
      EXPECT_LE(quantile(relation,.9),.012);EXPECT_LE(quantile(relative_rotation,.9),.12);
    }
  }
}
TEST_F(SharedRootGuidance, BoundedCandidateChainTimingIsReportedSeparately) {
  auto o=options;o.builder.shape_config=config.spark_upper_qpoases;
  SharedRootPipeline pipeline(o);
  const auto base=frame(1,left,right);std::vector<double> micros;micros.reserve(2048);
  for(std::uint64_t n=1;n<=2048;++n) {
    auto f=base;f.sequence=n;f.source_timestamp_ns=static_cast<std::int64_t>(n)*10000000;
    f.receive_monotonic_ns=f.source_timestamp_ns+1000000000;
    const auto start=std::chrono::steady_clock::now();
    const bool valid=pipeline.observe(f,f.receive_monotonic_ns);
    const double us=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();
    ASSERT_TRUE(valid);if(n>32)micros.push_back(us);
  }
  std::cout<<"shared_root_candidate_timing samples="<<micros.size()
    <<" p99_us="<<quantile(micros,.99)<<" target_us=50 real_time_qualified=false\n";
}
TEST_F(SharedRootGuidance, ExhaustedBudgetPreservesBoundedCandidateAndExternalAckGate) {
  // Fault injection, not a performance threshold: duration rounds to zero.
  config.spark_upper_qpoases.ik_cycle_budget_seconds=1e-12;
  DualArmSparkGuidance g(robot,config,kUrdf,kMode,&options);
  std::size_t exhausted=0;
  for(std::uint64_t n=1;n<=30;++n) {
    const auto out=tick(g,n);
    if(!out.target_valid)continue;
    exhausted+=out.left.ik.budget_exhausted||out.right.ik.budget_exhausted;
    // Legacy solver deliberately accepts its best bounded iterate even when
    // the deadline expires. accepted is NOT a convergence certificate.
    EXPECT_FALSE(g.confirmSharedRootReference(out.shared_root_cycle,false));
    EXPECT_NE(out.shared_root_state,SharedRootState::kTracking);
    EXPECT_TRUE(out.left.ik.q.allFinite());EXPECT_TRUE(out.right.ik.q.allFinite());
    for(auto side:{ArmSide::kLeft,ArmSide::kRight}) {
      const auto& q=side==ArmSide::kLeft?out.left.ik.q:out.right.ik.q;
      const auto& limits=robot.mapping(side).limits;
      EXPECT_TRUE((q.array()>=limits.lower_position.array()-1e-10).all());
      EXPECT_TRUE((q.array()<=limits.upper_position.array()+1e-10).all());
    }
  }
  EXPECT_GT(exhausted,0U);
}
TEST_F(SharedRootGuidance, ShippedBudgetWithZhoujieGeometryReportsAllExcludedSamples) {
  config=loadConfig(kProfile); // No .2 second correctness-test override.
  for(const double budget:{config.spark_upper_qpoases.ik_cycle_budget_seconds,.2}) {
  config.spark_upper_qpoases.ik_cycle_budget_seconds=budget;
  DualArmSparkGuidance g(robot,config,kUrdf,kMode,&options);
  constexpr double lengths[]={.25252884000000003,.24776442,.05999994};
  std::vector<double> times,errors;
  std::size_t rejected=0,budgets=0,valid=0;
  for(std::uint64_t n=1;n<=130;++n) {
    auto l=left,r=right;
    const double phase=.035*static_cast<double>(n);
    l.q[0]+=.03*std::sin(phase);r.q[0]+=.03*std::sin(phase+.4);
    auto f=frame(n,l,r);
    // FK provides synthetic directions/rotations, while the three segment
    // lengths are the pinned zhoujie snapshot. This is NOT a device trace.
    const auto points=f.upper_limb_skeleton.points;
    for(std::size_t side=0;side<2;++side) {
      for(std::size_t j=0;j<3;++j) {
        const auto i=side*4+j;
        f.upper_limb_skeleton.points[i+1]=f.upper_limb_skeleton.points[i]+
            lengths[j]*(points[i+1]-points[i]).normalized();
      }
    }
    ASSERT_TRUE(g.updateSharedRootFrame(f,f.receive_monotonic_ns));
    const auto start=std::chrono::steady_clock::now();
    const auto out=g.stepSharedRoot(left,right,.01,f.receive_monotonic_ns,true,true);
    const double us=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();
    if(n<=30)continue;
    times.push_back(us);
    budgets+=out.left.ik.budget_exhausted||out.right.ik.budget_exhausted;
    if(!out.target_valid||!out.left.ik.accepted||!out.right.ik.accepted) {++rejected;continue;}
    ++valid;
    for(auto side:{ArmSide::kLeft,ArmSide::kRight}) {
      const auto& d=side==ArmSide::kLeft?out.left:out.right;
      const auto& limits=robot.mapping(side).limits;
      ASSERT_TRUE(d.ik.q.allFinite());
      EXPECT_TRUE((d.ik.q.array()>=limits.lower_position.array()-1e-10).all());
      EXPECT_TRUE((d.ik.q.array()<=limits.upper_position.array()+1e-10).all());
      errors.push_back((robot.armKinematicsAt(side,d.ik.q).tcp_pose.position-d.target.palm.position).norm());
    }
  }
  EXPECT_EQ(valid+rejected,100U);
  EXPECT_GT(valid,0U);
  std::cout<<"zhoujie_synthetic_budget samples=100 accepted="<<valid<<" excluded="<<rejected
      <<" budget_exhausted="<<budgets<<" configured_budget_s="<<config.spark_upper_qpoases.ik_cycle_budget_seconds
      <<" guidance_wall_p99_us="<<quantile(times,.99)<<" palm_p90_m="<<quantile(errors,.9)
      <<" failure_cause=CauseUndetermined actual=unavailable real_time_qualified=false\n";
  // Timing is observational and host-dependent, not a CI hard-real-time claim.
  }
}
TEST_F(SharedRootGuidance, ZhoujieStaticPalmOnlyVersusShapeMultiSeedDiagnostic) {
  // Diagnostic ablation only: never passed through the runtime config loader.
  // Zero shape weights here do not relax production configuration validation.
  auto builder_options=options;
  builder_options.builder.shape_config=config.spark_upper_qpoases;
  SharedRootPipeline pipeline(builder_options);
  constexpr double lengths[]={.25252884000000003,.24776442,.05999994};
  for(std::uint64_t n=1;n<=40;++n) {
    auto f=frame(n,left,right);
    const auto original=f.upper_limb_skeleton.points;
    for(std::size_t side=0;side<2;++side) {
      for(std::size_t j=0;j<3;++j) {
        const auto i=side*4+j;
        f.upper_limb_skeleton.points[i+1]=f.upper_limb_skeleton.points[i]+
            lengths[j]*(original[i+1]-original[i]).normalized();
      }
    }
    ASSERT_TRUE(pipeline.observe(f,f.receive_monotonic_ns));
  }
  ASSERT_TRUE(pipeline.candidate().valid);
  PinocchioArmKinematics kinematics(kUrdf,
      {robot.tcpRelativeToLink7(ArmSide::kLeft),robot.tcpRelativeToLink7(ArmSide::kRight)});
  for(auto side:{ArmSide::kLeft,ArmSide::kRight}) {
    const auto target=side==ArmSide::kLeft?pipeline.candidate().filtered.left:pipeline.candidate().filtered.right;
    const auto& limits=robot.mapping(side).limits;
    for(int mode=0;mode<4;++mode) {
      auto c=config.spark_upper_qpoases;
      c.stage1_max_iterations=c.stage2_max_iterations=100;
      c.convergence_delta=1e-10;
      if(mode==1) {
        c.stage2_elbow_position_weight=c.stage2_wrist_position_weight=.25;
      } else if(mode==2) {
        c.stage1_upper_direction_weight=c.stage1_forearm_direction_weight=0;
        c.stage2_upper_direction_weight=c.stage2_forearm_direction_weight=0;
        c.stage2_elbow_position_weight=c.stage2_wrist_position_weight=0;
      } else if(mode==3) {
        c.stage2_upper_direction_weight=c.stage2_forearm_direction_weight=.05;
        c.stage2_elbow_position_weight=c.stage2_wrist_position_weight=.25;
      }
      double best_score=std::numeric_limits<double>::infinity(),best_position=best_score,best_rotation=best_score;
      std::size_t accepted=0,budget_count=0,within_tolerance=0;
      for(double fraction:{.2,.4,.5,.6,.8}) {
        const Vec7 seed=(1-fraction)*limits.lower_position+fraction*limits.upper_position;
        SparkUpperQpoasesIk7 solver(side,kinematics,limits,c);
        const auto result=solver.solve(target,seed,std::chrono::steady_clock::now()+std::chrono::milliseconds(200));
        budget_count+=result.budget_exhausted;
        ASSERT_TRUE(result.q.allFinite());
        EXPECT_TRUE((result.q.array()>=limits.lower_position.array()+c.joint_limit_margin_rad-1e-10).all());
        EXPECT_TRUE((result.q.array()<=limits.upper_position.array()-c.joint_limit_margin_rad+1e-10).all());
        if(!result.accepted)continue;
        ++accepted;
        const auto pose=kinematics.sample(side,result.q).tcp_pose;
        const double position=(pose.position-target.palm.position).norm();
        const double rotation=rotationDistance(pose.rotation,target.palm.rotation);
        within_tolerance+=position<=.01&&rotation<=.1;
        // Select one actual candidate, not independently best position/rotation.
        const double score=position/.01+rotation/.1;
        if(score<best_score) {best_score=score;best_position=position;best_rotation=rotation;}
      }
      EXPECT_GT(accepted,0U);
      if(mode==2) {
        // Establish reachability for this exact static palm pose, not for
        // arbitrary human input or the complete shape-constrained task.
        EXPECT_GT(within_tolerance,0U);
        EXPECT_LT(best_position,.001);
        EXPECT_LT(best_rotation,.01);
      }
      std::cout<<"zhoujie_static_ablation side="<<(side==ArmSide::kLeft?"left":"right")
          <<" mode="<<mode<<" seeds=5 accepted="<<accepted<<" budget_exhausted="<<budget_count
          <<" within_pose_tolerance="<<within_tolerance<<" best_position_m="<<best_position
          <<" best_orientation_rad="<<best_rotation<<" classification=CauseUndetermined actual=unavailable\n";
    }
  }
}
} // namespace
} // namespace tianji_qp_ik
