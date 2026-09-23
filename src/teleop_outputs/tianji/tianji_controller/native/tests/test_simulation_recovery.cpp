#include "tianji_qp_ik/simulation_recovery.hpp"
#include "tianji_qp_ik/joint_kinematics_plot.hpp"
#include <gtest/gtest.h>

namespace tianji_qp_ik {
namespace {
class RecoveryTest : public ::testing::Test {
 protected:
  QpIkConfig config;
  std::array<ArmLimits,2> limits;
  SimulationRecovery::Pair home{};
  const double dt=.005;
  void SetUp() override {
    config.joint_limits.margin_rad=.01;
    auto& smoothing=config.pico_ee_franka_dls.post_smoothing;
    smoothing.velocity_scale=1;
    smoothing.max_velocity_rad_s=Vec7::Constant(1);
    smoothing.max_acceleration_rad_s2=Vec7::Constant(2);
    smoothing.max_jerk_rad_s3=Vec7::Constant(10);
    for(auto& l:limits) {
      l.lower_position=Vec7::Constant(-2);
      l.upper_position=Vec7::Constant(2);
      l.velocity=Vec7::Constant(1);
    }
  }
};
TEST_F(RecoveryTest, RequiresFreshExplicitStart) {
  SimulationRecovery gate(config,limits,home,dt);
  EXPECT_EQ(gate.phase(),SimulationRecovery::Phase::kWaiting);
  EXPECT_FALSE(gate.start(false,home));
  auto moving=home; moving[1].qdot[0]=.1;
  EXPECT_FALSE(gate.start(true,moving));
  EXPECT_TRUE(gate.start(true,home));
  EXPECT_FALSE(gate.start(true,home));
}

TEST_F(RecoveryTest, HandsOnlyFollowDuringExplicitTeleop) {
  SimulationRecovery gate(config,limits,home,dt);
  EXPECT_TRUE(gate.handsPaused(false,false));
  ASSERT_TRUE(gate.start(true,home));
  EXPECT_FALSE(gate.handsPaused(false,false));
  EXPECT_TRUE(gate.handsPaused(true,false));
  EXPECT_TRUE(gate.handsPaused(false,true));
  ASSERT_TRUE(gate.stop(home,true));
  auto state=home;
  for(int i=0;i<2000&&gate.phase()!=SimulationRecovery::Phase::kHomeReached;++i) {
    EXPECT_TRUE(gate.handsPaused(false,false));
    state=gate.update(state);
  }
  ASSERT_EQ(gate.phase(),SimulationRecovery::Phase::kHomeReached);
  EXPECT_TRUE(gate.handsPaused(false,false));
  ASSERT_TRUE(gate.start(true,state));
  EXPECT_FALSE(gate.handsPaused(false,false));
  ASSERT_TRUE(gate.stop(state));
  for(int i=0;i<2000&&gate.phase()!=SimulationRecovery::Phase::kHold;++i) {
    EXPECT_TRUE(gate.handsPaused(false,false));
    state=gate.update(state);
  }
  ASSERT_EQ(gate.phase(),SimulationRecovery::Phase::kHold);
  EXPECT_TRUE(gate.handsPaused(false,false));
  gate.fault();
  EXPECT_TRUE(gate.handsPaused(false,false));
}

TEST_F(RecoveryTest, BrakesWithinConfiguredDlsVelocityLimit) {
  config.pico_ee_franka_dls.post_smoothing.max_velocity_rad_s=Vec7::Constant(.3);
  SimulationRecovery gate(config,limits,home,dt);
  auto state=home; state[0].qdot[0]=.2;
  ASSERT_TRUE(gate.stop(state));
  for(int i=0;i<2000;++i) {
    state=gate.update(state);
    for(const auto& arm:state)EXPECT_LE(arm.qdot.cwiseAbs().maxCoeff(),.3+1e-8);
  }
  EXPECT_EQ(gate.phase(),SimulationRecovery::Phase::kHold);
}
TEST_F(RecoveryTest, HomeIsSmoothBilateralAndDoesNotResume) {
  SimulationRecovery gate(config,limits,home,dt);
  auto state=home;
  state[0].q=Vec7::Constant(.6);state[1].q=Vec7::Constant(-.5);
  state[0].qdot[0]=.2;
  ASSERT_TRUE(gate.stop(state,true));
  EXPECT_FALSE(gate.stop(state,true));
  bool homing=false;
  JointKinematicsDifferentiator plot;
  bool valid_home_jerk=false;
  for(int i=0;i<6000&&gate.phase()!=SimulationRecovery::Phase::kHomeReached;++i) {
    auto previous=state;
    const bool home_sample=gate.phase()==SimulationRecovery::Phase::kHoming;
    homing|=gate.phase()==SimulationRecovery::Phase::kHoming;
    EXPECT_FALSE(gate.start(true,state));
    state=gate.update(state);
    ASSERT_NE(gate.phase(),SimulationRecovery::Phase::kFault);
    const auto derivatives=plot.update(state[0].qdot,state[0].qddot,
        ReferenceAccelerationSource::kRuckigOutput,state[0].qdot,dt,false);
    EXPECT_TRUE(derivatives.reference_acceleration_valid);
    EXPECT_TRUE(derivatives.reference_acceleration.isApprox(state[0].qddot));
    if(derivatives.reference_jerk_valid) {
      EXPECT_TRUE(derivatives.reference_jerk.isApprox(
          (state[0].qddot-previous[0].qddot)/dt));
      valid_home_jerk|=gate.phase()==SimulationRecovery::Phase::kHoming;
    }
    for(int arm=0;arm<2;++arm) {
      if(home_sample) {
        EXPECT_LE(state[arm].qdot.cwiseAbs().maxCoeff(),.5+1e-8);
        EXPECT_LE(state[arm].qddot.cwiseAbs().maxCoeff(),.8+1e-8);
        EXPECT_LE((state[arm].qddot-previous[arm].qddot).cwiseAbs().maxCoeff()/dt,3.+1e-6);
      }
      EXPECT_LE((state[arm].q-previous[arm].q).cwiseAbs().maxCoeff(),.00501);
      EXPECT_LE(state[arm].qdot.cwiseAbs().maxCoeff(),1.00001);
      EXPECT_LE(state[arm].qddot.cwiseAbs().maxCoeff(),2.00001);
      EXPECT_LE((state[arm].qddot-previous[arm].qddot).cwiseAbs().maxCoeff()/dt,10.0001);
    }
  }
  EXPECT_TRUE(homing);
  EXPECT_TRUE(valid_home_jerk);
  EXPECT_EQ(gate.phase(),SimulationRecovery::Phase::kHomeReached);
  for(const auto& s:state)EXPECT_TRUE(s.q.isZero(1e-6));
  EXPECT_FALSE(gate.teleop());
  EXPECT_TRUE(gate.start(true,state));
}
TEST_F(RecoveryTest, HoldCancelsHomeAndRequiresRestart) {
  SimulationRecovery gate(config,limits,home,dt);
  auto state=home;state[0].q[0]=.8;
  ASSERT_TRUE(gate.stop(state,true));
  for(int i=0;i<150;++i)state=gate.update(state);
  ASSERT_EQ(gate.phase(),SimulationRecovery::Phase::kHoming);
  ASSERT_TRUE(gate.stop(state));
  for(int i=0;i<3000;++i)state=gate.update(state);
  EXPECT_EQ(gate.phase(),SimulationRecovery::Phase::kHold);
  EXPECT_GT(state[0].q[0],.1);
  EXPECT_FALSE(gate.start(false,state));
  EXPECT_TRUE(gate.start(true,state));
}
TEST_F(RecoveryTest, InvalidSideFaultsWithoutPartialAdvance) {
  SimulationRecovery gate(config,limits,home,dt);
  auto invalid=home;invalid[1].q[0]=3;
  EXPECT_FALSE(gate.stop(invalid,true));
  EXPECT_EQ(gate.phase(),SimulationRecovery::Phase::kFault);
  EXPECT_FALSE(gate.start(true,home));
  EXPECT_TRUE(gate.update(home)[0].q.isZero());
}
} // namespace
} // namespace tianji_qp_ik
