#include "tianji_qp_ik/controller.hpp"
#include "tianji_qp_ik/so3.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <limits>

namespace tianji_qp_ik { namespace {
constexpr auto profile = TIANJI_PROJECT_SOURCE_DIR "/config/qp_ik_pico_shared_root_dls.yaml";
class FrankaDlsController : public ::testing::Test {
 protected:
  MujocoRobot robot{TIANJI_MODEL_DIR "/marvin_m6_wuji2_shared_root_ceres.xml"};
  QpIkConfig config{loadConfig(profile)};
  FrankaDlsController() {
    for (auto side : {ArmSide::kLeft, ArmSide::kRight})
      robot.setArmPosition(side, configuredInitialPosture(config.controller, robot.mapping(side).limits, side));
    robot.forward();
  }
  DualArmTargets target() { return {robot.tcpPose(ArmSide::kLeft),robot.tcpPose(ArmSide::kRight)}; }
  void useExecutionEnvelope() {
    std::array<ArmLimits, 2> limits{
        robot.mapping(ArmSide::kLeft).limits, robot.mapping(ArmSide::kRight).limits};
    limits[0].upper_position[2] = 0.0;
    limits[1].lower_position[2] = 0.0;
    limits[0].upper_position[3] = limits[1].upper_position[3] = 0.0;
    config.joint_limits.execution_limits = limits;
  }
};
TEST_F(FrankaDlsController, ExplicitPinocchioIsRequiredWithoutFallback) {
  auto c=config;c.controller.pico_ee_dls_kinematics_urdf_path.clear();
  EXPECT_THROW(DualArmController(robot,c),std::invalid_argument);
  c=config;c.controller.pico_ee_dls_kinematics_urdf_path="/nonexistent/dls.urdf";
  EXPECT_ANY_THROW(DualArmController(robot,c));
  c=config;c.controller.pico_ee_dls_kinematics_urdf_path=
      TIANJI_MODEL_DIR "/marvin_m6_s_ccs_696_v4_local.urdf";
  EXPECT_THROW(DualArmController(robot,c),std::invalid_argument);
}
TEST_F(FrankaDlsController, PinocchioTcpAndJacobianMatchBothModelAndFullPinocchio) {
  DlsPinocchioArmKinematics kin(config.controller.pico_ee_dls_kinematics_urdf_path,
      {robot.tcpRelativeToLink7(ArmSide::kLeft),robot.tcpRelativeToLink7(ArmSide::kRight)});
  for(auto side:{ArmSide::kLeft,ArmSide::kRight}) {
    const auto& limits=robot.mapping(side).limits;
    for(int n=0;n<100;++n) {
      Vec7 q;for(int j=0;j<7;++j)q[j]=limits.lower_position[j]+
          (.5+.49*std::sin(double((n+1)*(j+1))*.6180339887498948))*(limits.upper_position[j]-limits.lower_position[j]);
      const auto fast=kin.sampleTcp(side,q),full=kin.sample(side,q),mj=robot.armKinematicsAt(side,q);
      EXPECT_LT((fast.tcp_pose.position-full.tcp_pose.position).norm(),1e-12);
      EXPECT_LT((fast.tcp_jacobian-full.tcp_jacobian).norm(),1e-12);
      EXPECT_LT((fast.tcp_pose.position-mj.tcp_pose.position).norm(),1e-5);
      EXPECT_LT(rotationDistance(fast.tcp_pose.rotation,mj.tcp_pose.rotation),1e-5);
      EXPECT_LT((fast.tcp_jacobian-mj.tcp_jacobian).cwiseAbs().maxCoeff(),1e-5);
    }
  }
}
TEST_F(FrankaDlsController, ReportsPinocchioAndSeparatePipelineTiming) {
  DualArmController controller(robot,config);
  auto t=target();t.left.position.x()+=.01;
  const auto d=controller.step(t,.005);ASSERT_TRUE(d.accepted);
  for(const auto* arm:{&d.left,&d.right}) {
    EXPECT_TRUE(arm->ee_pinocchio_kinematics);EXPECT_TRUE(arm->ee_ruckig_invoked);
    EXPECT_GT(arm->ee_ik_wall_time_us,0);EXPECT_GT(arm->ee_ruckig_wall_time_us,0);
    EXPECT_GE(arm->ee_ik_to_ruckig_wall_time_us,arm->ee_ik_wall_time_us+arm->ee_ruckig_wall_time_us);
  }
}
TEST_F(FrankaDlsController, KinematicsOnlyMatchesFullModelAndDoesNotMutateLiveState) {
  for(auto side:{ArmSide::kLeft,ArmSide::kRight}) {
    const auto original=robot.armPosition(side);
    const auto limits=robot.mapping(side).limits;
    for(int n=0;n<32;++n) {
      Vec7 q;
      for(int j=0;j<7;++j)q[j]=limits.lower_position[j]+(.5+.45*std::sin(double(7*n+j)))*(limits.upper_position[j]-limits.lower_position[j]);
      const auto full=robot.armKinematicsAt(side,q), fast=robot.armKinematicsOnlyAt(side,q);
      EXPECT_LT((full.tcp_pose.position-fast.tcp_pose.position).norm(),1e-12);
      EXPECT_LT((full.tcp_pose.rotation-fast.tcp_pose.rotation).norm(),1e-12);
      EXPECT_LT((full.tcp_jacobian-fast.tcp_jacobian).norm(),1e-12);
      EXPECT_LT((full.shoulder_position-fast.shoulder_position).norm(),1e-12);
      EXPECT_LT((full.elbow_position-fast.elbow_position).norm(),1e-12);
      EXPECT_LT((full.wrist_position-fast.wrist_position).norm(),1e-12);
      EXPECT_LT((full.elbow_position_jacobian-fast.elbow_position_jacobian).norm(),1e-12);
      EXPECT_LT((full.wrist_position_jacobian-fast.wrist_position_jacobian).norm(),1e-12);
      EXPECT_EQ(robot.armPosition(side),original);
    }
  }
}
TEST_F(FrankaDlsController, RejectsHardwareDisabledAndRelaxedEnvelope) {
  auto c=config; c.controller.model_state_only=false;
  EXPECT_THROW(DualArmController(robot,c),std::invalid_argument);
  c=config; c.pico_ee_franka_dls.enabled=false;
  EXPECT_THROW(DualArmController(robot,c),std::invalid_argument);
  c=config; c.pico_ee_franka_dls.post_smoothing.max_jerk_rad_s3*=2;
  EXPECT_THROW(DualArmController(robot,c),std::invalid_argument);
  c=config; c.pico_ee_franka_dls.post_smoothing.max_velocity_rad_s[0]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(DualArmController(robot,c),std::invalid_argument);
}
TEST_F(FrankaDlsController, UnreachablePositiveElbowPoseNeverEscapesExecutionEnvelope) {
  std::array<Vec7, 2> initial{robot.armPosition(ArmSide::kLeft),
                             robot.armPosition(ArmSide::kRight)};
  initial[0][3] = initial[1][3] = -0.15;
  for (bool constrained : {false, true}) {
    if (constrained) useExecutionEnvelope();
    for (auto side : {ArmSide::kLeft, ArmSide::kRight})
      robot.setArmState(side, initial[side == ArmSide::kLeft ? 0 : 1], Vec7::Zero());
    robot.forward();
    DualArmController controller(robot, config);
    double maximum_elbow = -0.15;
    for (int step = 0; step < 600; ++step) {
      DualArmTargets requested;
      for (auto side : {ArmSide::kLeft, ArmSide::kRight}) {
        Vec7 q = initial[side == ArmSide::kLeft ? 0 : 1];
        q[3] += 0.001 * (step + 1);
        (side == ArmSide::kLeft ? requested.left : requested.right) =
            robot.armKinematicsAt(side, q).tcp_pose;
      }
      const auto result = controller.step(requested, 0.005);
      for (auto side : {ArmSide::kLeft, ArmSide::kRight}) {
        const auto& q = controller.reference(side);
        maximum_elbow = std::max(maximum_elbow, q[3]);
        if (!constrained) continue;
        const auto& limits = (*config.joint_limits.execution_limits)[side == ArmSide::kLeft ? 0 : 1];
        EXPECT_TRUE((q.array() >= limits.lower_position.array()).all());
        EXPECT_TRUE((q.array() <= limits.upper_position.array()).all());
        if (result.accepted) {
          const auto& goal = side == ArmSide::kLeft ? result.left.dls_posture_goal
                                                  : result.right.dls_posture_goal;
          EXPECT_TRUE((goal.array() >= limits.lower_position.array() + config.joint_limits.margin_rad - 1e-10).all());
          EXPECT_TRUE((goal.array() <= limits.upper_position.array() - config.joint_limits.margin_rad + 1e-10).all());
        }
      }
    }
    // Both trials must actually move; the wide-model control must reach the
    // forbidden branch, so an always-hold implementation cannot pass.
    EXPECT_GT(maximum_elbow, constrained ? -0.14 : 0.02);
  }
}

TEST_F(FrankaDlsController, RejectsInitialAndResetStatesOutsideEffectiveRuckigEnvelope) {
  useExecutionEnvelope();
  const auto initial = robot.armPosition(ArmSide::kLeft);
  for (double elbow : {0.1, -0.01}) {
    Vec7 q = initial;
    q[3] = elbow;
    robot.setArmPosition(ArmSide::kLeft, q);
    robot.forward();
    EXPECT_THROW(DualArmController(robot, config), std::invalid_argument);
  }
  robot.setArmPosition(ArmSide::kLeft, initial);
  robot.forward();
  DualArmController controller(robot, config);
  const auto before = controller.referenceState(ArmSide::kLeft);
  for (double elbow : {0.1, -0.01}) {
    auto invalid = before;
    invalid.q[3] = elbow;
    EXPECT_FALSE(controller.setReferenceState(ArmSide::kLeft, invalid));
    EXPECT_EQ(controller.reference(ArmSide::kLeft), before.q);
    EXPECT_EQ(robot.armPosition(ArmSide::kLeft), before.q);
    std::array<Vec7, 2> measured{before.q, controller.reference(ArmSide::kRight)};
    measured[0][0] += .02;
    measured[1][3] = elbow;
    DualArmTargets captured_tcp;
    EXPECT_FALSE(controller.resetEpisodeReference(measured, captured_tcp));
    EXPECT_EQ(controller.reference(ArmSide::kLeft), before.q);
    EXPECT_EQ(robot.armPosition(ArmSide::kLeft), before.q);
  }
  EXPECT_TRUE(controller.step(target(), 0.005).accepted);
}

TEST_F(FrankaDlsController, StationaryAndMovingPoseRemainBounded) {
  DualArmController controller(robot,config);
  auto t=target();
  ASSERT_TRUE(controller.step(t,.005).accepted);
  t.left.position.x()+=.02; t.right.position.x()+=.02;
  auto previous=controller.referenceState(ArmSide::kLeft);
  for(int n=0;n<150;++n) {
    const auto d=controller.step(t,.005);
    ASSERT_TRUE(d.accepted) << d.left.dls_posture_ruckig_detail << " " << d.right.dls_posture_ruckig_detail;
    const auto state=controller.referenceState(ArmSide::kLeft);
    EXPECT_TRUE((state.qdot.cwiseAbs().array()<=config.pico_ee_franka_dls.post_smoothing.max_velocity_rad_s.array()+1e-8).all());
    EXPECT_TRUE((state.qddot.cwiseAbs().array()<=config.joint_limits.max_acceleration_rad_s2.array()+1e-8).all());
    EXPECT_TRUE(((state.qddot-previous.qddot).cwiseAbs().array()/.005<=config.joint_limits.max_jerk_rad_s3.array()+1e-6).all());
    previous=state;
  }
  EXPECT_LT((robot.armKinematicsAt(ArmSide::kLeft,previous.q).tcp_pose.position-t.left.position).norm(),.003);
}
TEST_F(FrankaDlsController, ExplicitSimulationStartCapsBothArmsAndWaitsForLiveTarget) {
  DualArmController controller(robot,config);
  ASSERT_TRUE(controller.beginSimulationSoftStart());
  auto t=target();t.left_stale=t.right_stale=true;
  for(int n=0;n<60;++n) EXPECT_FALSE(controller.step(t,.005).accepted);
  t.left_stale=t.right_stale=false;
  t.left.position.x()+=.10;t.right.position.x()+=.10;
  for(int n=0;n<40;++n) {
    std::array<ArmMotionState,2> previous{controller.referenceState(ArmSide::kLeft),controller.referenceState(ArmSide::kRight)};
    auto result=controller.step(t,.005);
    ASSERT_TRUE(result.accepted)<<result.left.dls_posture_ruckig_detail;
    for(auto side:{ArmSide::kLeft,ArmSide::kRight}) {
      const auto state=controller.referenceState(side);
      EXPECT_LE(state.qdot.cwiseAbs().maxCoeff(),.35+1e-8);
      EXPECT_DOUBLE_EQ(controller.trajectorySampleLimits(side).max_velocity_rad_s[0],.35);
      EXPECT_LE(state.qddot.cwiseAbs().maxCoeff(),.5+1e-8);
      const auto& before=previous[side==ArmSide::kLeft?0:1];
      EXPECT_LE((state.qddot-before.qddot).cwiseAbs().maxCoeff()/.005,2.+1e-6);
    }
  }
  EXPECT_FALSE(controller.beginSimulationSoftStart()); // Cannot lower limits on a moving seed.
}
TEST_F(FrankaDlsController, StaleDoesNotChaseLastGoalAndRecovers) {
  DualArmController controller(robot,config);
  auto t=target();t.left.position.x()+=.10;t.right.position.x()+=.10;
  for(int n=0;n<15;++n) ASSERT_TRUE(controller.step(t,.005).accepted);
  t.left_stale=true;
  for(int n=0;n<100;++n) {
    const auto d=controller.step(t,.005);
    EXPECT_FALSE(d.accepted); EXPECT_FALSE(d.left.accepted); EXPECT_FALSE(d.right.accepted);
    EXPECT_EQ(d.hold_reason, HoldReason::kNone);
    EXPECT_TRUE(d.left.dls_posture_ruckig_accepted);
    EXPECT_TRUE(d.right.dls_posture_ruckig_accepted);
  }
  EXPECT_LT(controller.previousVelocity(ArmSide::kLeft).norm(),1e-6);
  t.left_stale=false;
  EXPECT_TRUE(controller.step(t,.005).accepted);
}
TEST_F(FrankaDlsController, Pico2ExplicitSoftStartCapsReachBothLimiters) {
  DualArmController controller(robot,config);
  EXPECT_FALSE(controller.beginSimulationSoftStart({1.4,-3.,12.}));
  ASSERT_TRUE(controller.beginSimulationSoftStart({1.4,3.,12.}));
  auto t=target();t.left.position.x()+=.10;t.right.position.x()+=.10;
  for(int n=0;n<30;++n) {
    std::array<ArmMotionState,2> previous{controller.referenceState(ArmSide::kLeft),controller.referenceState(ArmSide::kRight)};
    ASSERT_TRUE(controller.step(t,.005).accepted);
    for(auto side:{ArmSide::kLeft,ArmSide::kRight}) {
      const auto limits=controller.trajectorySampleLimits(side);
      EXPECT_DOUBLE_EQ(limits.max_velocity_rad_s[0],1.4);
      EXPECT_DOUBLE_EQ(limits.max_acceleration_rad_s2[0],3.);
      EXPECT_DOUBLE_EQ(limits.max_jerk_rad_s3[0],12.);
      const auto state=controller.referenceState(side);
      EXPECT_LE(state.qdot.cwiseAbs().maxCoeff(),1.4+1e-8);
      EXPECT_LE(state.qddot.cwiseAbs().maxCoeff(),3.+1e-8);
      EXPECT_LE((state.qddot-previous[side==ArmSide::kLeft?0:1].qddot).cwiseAbs().maxCoeff()/.005,12.+1e-6);
    }
  }
}
TEST_F(FrankaDlsController, BadDtCannotMoveEitherArm) {
  DualArmController controller(robot,config);
  const auto l=controller.reference(ArmSide::kLeft),r=controller.reference(ArmSide::kRight);
  EXPECT_FALSE(controller.step(target(),0).accepted);
  EXPECT_EQ(controller.reference(ArmSide::kLeft),l);
  EXPECT_EQ(controller.reference(ArmSide::kRight),r);
}
TEST_F(FrankaDlsController, OneInvalidResetCannotPartiallyCommitOtherArm) {
  DualArmController controller(robot,config);
  auto motion=controller.referenceState(ArmSide::kRight);
  motion.q=robot.mapping(ArmSide::kRight).limits.lower_position;
  ASSERT_TRUE(controller.setReferenceState(ArmSide::kRight,motion));
  const auto l=controller.reference(ArmSide::kLeft),r=controller.reference(ArmSide::kRight);
  auto t=target();t.left.position.x()+=.02;
  const auto failed=controller.step(t,.005);
  EXPECT_FALSE(failed.accepted);
  EXPECT_FALSE(failed.left.accepted);
  EXPECT_FALSE(failed.right.accepted);
  EXPECT_EQ(failed.hold_reason,HoldReason::kSolverFailure);
  EXPECT_EQ(controller.reference(ArmSide::kLeft),l);
  EXPECT_EQ(controller.reference(ArmSide::kRight),r);
}
TEST_F(FrankaDlsController, ResetDropsOldGoalSeed) {
  DualArmController controller(robot,config);
  auto t=target();t.left.position.x()+=.08;
  ASSERT_TRUE(controller.step(t,.005).accepted);
  controller.resetSolvers();
  const auto reset_q=controller.referenceState(ArmSide::kLeft).q;
  robot.forward(); const auto still=target();
  const auto d=controller.step(still,.005);
  ASSERT_TRUE(d.accepted);
  // Compare IK seed/goal to the reset state, not the subsequent moving Ruckig
  // reference: resetting solver history does not erase physical velocity.
  EXPECT_LT((d.left.dls_posture_goal-reset_q).norm(),1e-8);
}
TEST_F(FrankaDlsController, LivePipelineMatchesSourceKernelThenOnlineRuckig) {
  DualArmController controller(robot,config);
  DlsPinocchioArmKinematics kin(config.controller.pico_ee_dls_kinematics_urdf_path,
      {robot.tcpRelativeToLink7(ArmSide::kLeft),robot.tcpRelativeToLink7(ArmSide::kRight)});
  PicoEeFrankaDlsIk7 solver(config.iterative_dls,config.pico_ee_franka_dls,config.joint_limits.margin_rad);
  auto limits=robot.mapping(ArmSide::kLeft).limits, safe=limits;
  safe.lower_position.array()+=config.joint_limits.margin_rad;
  safe.upper_position.array()-=config.joint_limits.margin_rad;
  RuckigTrajectoryLimiter7 smoother(config.pico_ee_franka_dls.post_smoothing,safe,.005);
  auto current=controller.referenceState(ArmSide::kLeft);
  solver.reset(current);ASSERT_TRUE(smoother.reset(current));
  auto t=target();
  for(int n=0;n<100;++n) {
    t.left.position.x()+=.0003;
    PicoEeFrankaDlsInput input;
    input.target=t.left;input.target_valid=true;input.dt=.005;input.limits=limits;
    input.seed=current.q;input.seed_velocity=current.qdot;input.seed_acceleration=current.qddot;
    input.velocity_bounds.lower=-limits.velocity;input.velocity_bounds.upper=limits.velocity;
    input.home_reference=config.pico_ee_franka_dls.home_left_rad;
    input.evaluate=[&](const Vec7& q){return kin.sampleTcp(ArmSide::kLeft,q);};
    const auto ik=solver.solve(input);ASSERT_TRUE(ik.accepted);
    const auto trajectory=smoother.update(ik.goal,.005);ASSERT_TRUE(trajectory.accepted);
    const auto out=controller.step(t,.005);ASSERT_TRUE(out.accepted);
    EXPECT_LT((out.left.dls_posture_goal-ik.goal).norm(),1e-12);
    current=controller.referenceState(ArmSide::kLeft);
    EXPECT_LT((current.q-trajectory.state.q).norm(),1e-12);
    EXPECT_LT((current.qdot-trajectory.state.qdot).norm(),1e-12);
    EXPECT_LT((current.qddot-trajectory.state.qddot).norm(),1e-12);
  }
}
} }
