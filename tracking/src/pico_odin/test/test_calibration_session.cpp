#include <cmath>

#include <gtest/gtest.h>

#include "pico_odin/calibration_session.hpp"
#include "pico_odin/ground_ready_gate.hpp"
#include "pico_odin/skeleton_frame_contract.hpp"

namespace po = pico_odin;

namespace {

po::HostTimedPose sample(double stamp, double pitch = 0.0, double yaw = 0.0) {
  return po::HostTimedPose{
    stamp,
    po::Pose3{
      Eigen::Quaterniond(
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())),
      Eigen::Vector3d(0.0, 0.0, 1.0)}};
}

}  // namespace

TEST(SkeletonFrameContract, AcceptsOnlyTheConfiguredCanonicalFrame) {
  EXPECT_TRUE(po::is_expected_skeleton_frame("pico_ground", "pico_ground"));
  EXPECT_FALSE(po::is_expected_skeleton_frame("pico", "pico_ground"));
  EXPECT_FALSE(po::is_expected_skeleton_frame("", "pico_ground"));
  EXPECT_FALSE(po::is_expected_skeleton_frame("map", "pico_ground"));
  EXPECT_FALSE(po::is_expected_skeleton_frame("pico_ground", ""));
}

TEST(CalibrationSession, WorldResetCancelsAndClearsActiveAttempt) {
  po::CalibrationSession session;
  session.observe_valid_inputs(0.0);
  ASSERT_EQ(session.phase(), po::CalibrationPhase::kReady);
  ASSERT_TRUE(session.start(0.0));
  session.add_pico(sample(0.1), true, 0.1);
  session.add_odin(sample(0.1));
  ASSERT_FALSE(session.pico_samples().empty());

  session.on_world_reset();

  EXPECT_EQ(session.phase(), po::CalibrationPhase::kCancelled);
  EXPECT_TRUE(session.pico_samples().empty());
  EXPECT_TRUE(session.odin_samples().empty());
  EXPECT_NE(session.message().find("world reset"), std::string::npos);
}

TEST(CalibrationSession, RequestedStartSurvivesInputPauseAndStartsWhenFresh) {
  po::CalibrationSession session;
  session.observe_valid_inputs(0.0);
  ASSERT_EQ(session.phase(), po::CalibrationPhase::kReady);
  ASSERT_TRUE(session.request_start());
  EXPECT_EQ(session.phase(), po::CalibrationPhase::kPendingStart);

  session.observe_invalid_inputs();
  EXPECT_EQ(session.phase(), po::CalibrationPhase::kPendingStart);
  EXPECT_TRUE(session.pico_samples().empty());
  EXPECT_TRUE(session.odin_samples().empty());

  session.observe_valid_inputs(0.8);
  EXPECT_EQ(session.phase(), po::CalibrationPhase::kNeutral);
  EXPECT_NE(session.message().find("stand upright"), std::string::npos);
}

TEST(GroundReadyGate, RequiresNotReadyThenReadyAfterArm) {
  po::GroundReadyGate gate;
  gate.arm();

  gate.observe(true);
  EXPECT_FALSE(gate.ready());
  gate.observe(false);
  EXPECT_FALSE(gate.ready());
  gate.observe(true);
  EXPECT_TRUE(gate.ready());
  EXPECT_TRUE(gate.consume());
  EXPECT_FALSE(gate.ready());
}

TEST(GroundReadyGate, ClaimsACompleteUnconsumedCycleObservedBeforeArm) {
  po::GroundReadyGate gate;
  gate.observe(false);
  gate.observe(true);

  gate.arm();
  EXPECT_TRUE(gate.ready());
  EXPECT_TRUE(gate.consume());
}

TEST(GroundReadyGate, DoesNotClaimAStaleTrueObservedBeforeArm) {
  po::GroundReadyGate gate;
  gate.observe(true);

  gate.arm();
  EXPECT_FALSE(gate.ready());
}

TEST(GroundReadyGate, RepeatedArmRequiresANewNotReadyTransition) {
  po::GroundReadyGate gate;
  gate.arm();
  gate.observe(false);
  gate.arm();

  gate.observe(true);
  EXPECT_FALSE(gate.ready());
  gate.observe(false);
  gate.observe(true);
  EXPECT_TRUE(gate.consume());
}

TEST(GroundReadyGate, RepeatedArmDoesNotReuseAClaimedPreArmCycle) {
  po::GroundReadyGate gate;
  gate.observe(false);
  gate.observe(true);
  gate.arm();
  EXPECT_TRUE(gate.ready());

  gate.arm();
  EXPECT_FALSE(gate.ready());
  gate.observe(false);
  gate.observe(true);
  EXPECT_TRUE(gate.ready());
}

TEST(GroundReadyGate, DefersASecondCycleUntilTheNextResetArm) {
  po::GroundReadyGate gate;
  gate.arm();
  gate.observe(false);
  gate.observe(true);
  ASSERT_TRUE(gate.ready());

  gate.observe(false);
  gate.observe(true);
  EXPECT_FALSE(gate.ready());

  gate.arm();
  EXPECT_TRUE(gate.ready());
}

TEST(CalibrationSession, RepeatedResetIsIdempotentWhileStartIsPending) {
  po::CalibrationSession session;
  session.observe_valid_inputs(0.0);
  ASSERT_TRUE(session.request_start());

  session.on_world_reset();

  EXPECT_EQ(session.phase(), po::CalibrationPhase::kPendingStart);
  session.observe_valid_inputs(1.0);
  ASSERT_EQ(session.phase(), po::CalibrationPhase::kNeutral);
  session.on_world_reset();
  EXPECT_EQ(session.phase(), po::CalibrationPhase::kCancelled);
}

TEST(CalibrationSession, OnePitchAndYawCycleReachesSolveAfterFinalNeutral) {
  po::CalibrationSessionOptions options;
  options.neutral_duration_sec = 0.2;
  options.final_neutral_duration_sec = 0.2;
  options.min_pitch_range_rad = 0.20;
  options.min_yaw_range_rad = 0.20;
  po::CalibrationSession session(options);
  session.observe_valid_inputs(0.0);
  ASSERT_TRUE(session.start(0.0));

  session.add_pico(sample(0.0), true, 0.0);
  session.add_odin(sample(0.0));
  session.add_pico(sample(0.25), true, 0.25);
  session.add_odin(sample(0.25));
  ASSERT_EQ(session.phase(), po::CalibrationPhase::kExcitation);

  session.add_pico(sample(0.35, 0.16, 0.0), false, 0.35);
  session.add_odin(sample(0.35));
  session.add_pico(sample(0.45, -0.16, 0.0), false, 0.45);
  session.add_odin(sample(0.45));
  session.add_pico(sample(0.55, 0.0, 0.16), false, 0.55);
  session.add_odin(sample(0.55));
  session.add_pico(sample(0.65, 0.0, -0.16), false, 0.65);
  session.add_odin(sample(0.65));
  session.add_pico(sample(0.75), true, 0.75);
  session.add_odin(sample(0.75));
  ASSERT_EQ(session.phase(), po::CalibrationPhase::kFinalNeutral);
  session.add_pico(sample(1.0), true, 1.0);
  session.add_odin(sample(1.0));

  EXPECT_EQ(session.phase(), po::CalibrationPhase::kSolving);
  EXPECT_GT(session.pitch_range_rad(), 0.20);
  EXPECT_GT(session.yaw_range_rad(), 0.20);
  EXPECT_FALSE(session.pico_samples().empty());
}

TEST(CalibrationSession, OdinRestartCancelsAttempt) {
  po::CalibrationSession session;
  session.observe_valid_inputs(0.0);
  ASSERT_TRUE(session.start(0.0));

  session.on_odin_restart();

  EXPECT_EQ(session.phase(), po::CalibrationPhase::kCancelled);
  EXPECT_NE(session.message().find("Odin restart"), std::string::npos);
}

TEST(CalibrationSession, OneSidedExcursionsDoNotCompleteTheRequiredCycles) {
  po::CalibrationSessionOptions options;
  options.neutral_duration_sec = 0.2;
  options.final_neutral_duration_sec = 0.2;
  options.min_pitch_range_rad = 0.20;
  options.min_yaw_range_rad = 0.20;
  po::CalibrationSession session(options);
  session.observe_valid_inputs(0.0);
  ASSERT_TRUE(session.start(0.0));
  session.add_pico(sample(0.0), true, 0.0);
  session.add_pico(sample(0.25), true, 0.25);

  session.add_pico(sample(0.35, 0.25, 0.25), false, 0.35);
  session.add_pico(sample(0.45), true, 0.45);

  EXPECT_EQ(session.phase(), po::CalibrationPhase::kExcitation);
}

TEST(CalibrationSession, MustReturnToInitialUprightPoseBeforeFinalNeutral) {
  po::CalibrationSessionOptions options;
  options.neutral_duration_sec = 0.2;
  options.final_neutral_duration_sec = 0.2;
  options.min_pitch_range_rad = 0.20;
  options.min_yaw_range_rad = 0.20;
  options.return_upright_tolerance_rad = 0.05;
  po::CalibrationSession session(options);
  session.observe_valid_inputs(0.0);
  ASSERT_TRUE(session.start(0.0));
  session.add_pico(sample(0.0), true, 0.0);
  session.add_pico(sample(0.25), true, 0.25);
  session.add_pico(sample(0.35, 0.16, 0.16), false, 0.35);
  session.add_pico(sample(0.45, -0.16, -0.16), false, 0.45);

  session.add_pico(sample(0.55, 0.10, 0.10), true, 0.55);

  EXPECT_EQ(session.phase(), po::CalibrationPhase::kExcitation);
}

TEST(CalibrationSession, RejectsInitialNeutralPoseThatIsNotUpright) {
  po::CalibrationSessionOptions options;
  options.neutral_duration_sec = 0.2;
  options.max_initial_tilt_rad = 0.20;
  po::CalibrationSession session(options);
  session.observe_valid_inputs(0.0);
  ASSERT_TRUE(session.start(0.0));

  session.add_pico(sample(0.0, 0.35), true, 0.0);
  session.add_pico(sample(0.25, 0.35), true, 0.25);

  EXPECT_EQ(session.phase(), po::CalibrationPhase::kRejected);
  EXPECT_NE(session.message().find("upright"), std::string::npos);
}

TEST(CalibrationSession, StaleInputRevokesReadyAndCancelsActiveAttempt) {
  po::CalibrationSession session;
  session.observe_valid_inputs(0.0);
  ASSERT_EQ(session.phase(), po::CalibrationPhase::kReady);

  session.observe_invalid_inputs();
  EXPECT_EQ(session.phase(), po::CalibrationPhase::kWaitingForInputs);
  EXPECT_FALSE(session.start(0.1));

  session.observe_valid_inputs(0.2);
  ASSERT_TRUE(session.start(0.2));
  session.add_pico(sample(0.3), true, 0.3);
  session.add_odin(sample(0.3));
  session.observe_invalid_inputs();

  EXPECT_EQ(session.phase(), po::CalibrationPhase::kCancelled);
  EXPECT_TRUE(session.pico_samples().empty());
  EXPECT_TRUE(session.odin_samples().empty());
}
