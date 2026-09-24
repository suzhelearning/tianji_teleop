#include "tianji_qp_ik/episode_relative.hpp"
#include "tianji_qp_ik/so3.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <limits>

namespace tianji_qp_ik {
namespace {
class EpisodeRelative : public ::testing::Test {
 protected:
  MujocoRobot robot{TIANJI_MODEL_DIR "/marvin_m6_wuji2_shared_root_ceres.xml"};
  QpIkConfig config{loadConfig(TIANJI_PROJECT_SOURCE_DIR "/config/qp_ik_pico_shared_root_dls.yaml")};
  std::unique_ptr<DualArmController> controller;
  EpisodeRelativeSession session{.1};
  PicoTeleopFrame frame;
  std::int64_t now{1000000000};
  std::uint64_t token{0};
  std::array<Vec7, 2> measured;
  EpisodeRelative() {
    for (std::size_t arm = 0; arm < 2; ++arm) {
      const auto side = arm == 0 ? ArmSide::kLeft : ArmSide::kRight;
      measured[arm] = configuredInitialPosture(config.controller, robot.mapping(side).limits, side);
      robot.setArmPosition(side, measured[arm]);
      // Actual hardware can differ from nominal. A prepare must use this snapshot.
      measured[arm][0] += arm == 0 ? .04 : -.04;
    }
    robot.forward(); controller = std::make_unique<DualArmController>(robot, config);
    frame.tracking_epoch = 1;
    frame.upper_limb_skeleton.valid = frame.upper_limb_skeleton.rotations_valid = true;
    frame.upper_limb_skeleton.points[kPicoLeftShoulderPoint] = {.0, .2, 1.5};
    frame.upper_limb_skeleton.points[kPicoRightShoulderPoint] = {.0, -.2, 1.5};
    frame.upper_limb_skeleton.points[kPicoLeftHandPoint] = {.15, .2, 1.0};
    frame.upper_limb_skeleton.points[kPicoRightHandPoint] = {.15, -.2, 1.0};
    // This branch deliberately contains an incompatible absolute/scaled target.
    frame.left.position = frame.right.position = Eigen::Vector3d::Constant(99.);
  }
  void sample() {
    now += 10000000; ++frame.sequence;
    frame.source_timestamp_ns = frame.bridge_send_monotonic_ns = frame.receive_monotonic_ns = now;
    (void)session.observe(frame, now);
    session.tick(now);
  }
  void stable() { for (int n = 0; n < 40; ++n) sample(); }
  TeleopReferenceCommand request(ReferenceOperation op, std::uint64_t id = 1) {
    TeleopReferenceCommand out;
    out.token = ++token; out.operation = op; out.reference_id = id;
    out.received_ns = out.measured_ns = now; out.measured_q = measured;
    out.waist_lower = {-.2, -.5, -.8}; out.waist_upper = {.5, .5, -.2};
    return out;
  }
  TeleopReferenceResult command(ReferenceOperation op, std::uint64_t id = 1) {
    return session.command(request(op, id), *controller, now);
  }
  void expectStationary() {
    for (std::size_t arm = 0; arm < 2; ++arm) {
      const auto state = controller->referenceState(arm == 0 ? ArmSide::kLeft : ArmSide::kRight);
      EXPECT_LT((state.q - measured[arm]).norm(), 1e-10);
      EXPECT_LT(state.qdot.norm(), 1e-10); EXPECT_LT(state.qddot.norm(), 1e-10);
    }
  }
};

TEST_F(EpisodeRelative, PreflightHoldsUntilFreshBilateralPrepare) {
  const auto initial = controller->reference(ArmSide::kLeft);
  EXPECT_FALSE(session.ready(now));
  sample(); EXPECT_TRUE(session.ready(now));
  EXPECT_EQ(session.referenceId(), 0U);
  EXPECT_FALSE(command(ReferenceOperation::kPrepare).success); // No waiting or capture.
  EXPECT_EQ(controller->reference(ArmSide::kLeft), initial);
  stable();
  const auto prepared = command(ReferenceOperation::kPrepare, 2);
  ASSERT_TRUE(prepared.success) << prepared.message;
  EXPECT_EQ(prepared.phase, ReferencePhase::kPrepared);
  expectStationary();
  sample();
  EXPECT_TRUE(session.ready(now)); EXPECT_FALSE(session.active());
  expectStationary();
}

TEST_F(EpisodeRelative, FirstAndStaticFrameDoNotDriftAcrossTwoMeasuredEpisodes) {
  for (std::uint64_t id = 1; id <= 2; ++id) {
    stable();
    const auto prepared = command(ReferenceOperation::kPrepare, id);
    ASSERT_TRUE(prepared.success) << prepared.message;
    ASSERT_TRUE(command(ReferenceOperation::kActivate, id).success);
    for (int cycle = 0; cycle < 100; ++cycle) {
      sample();
      const auto result = controller->step(session.targets(), .005);
      ASSERT_TRUE(result.accepted) << result.left.dls_posture_ruckig_detail << ' '
                                   << result.right.dls_posture_ruckig_detail;
      expectStationary();
      EXPECT_LT((session.targets().left.position - prepared.robot_reference[0].position).norm(), 1e-12);
    }
    const auto before_cancel = controller->reference(ArmSide::kLeft);
    ASSERT_TRUE(command(ReferenceOperation::kCancel, id).success);
    EXPECT_EQ(session.referenceId(), 0U);
    EXPECT_EQ(controller->reference(ArmSide::kLeft), before_cancel);
    measured[0][0] -= .025; measured[1][0] += .035;
    frame.upper_limb_skeleton.points[kPicoLeftHandPoint].x() += .025;
    frame.upper_limb_skeleton.points[kPicoRightHandPoint].y() += .03;
  }
}

TEST_F(EpisodeRelative, TranslationIsOneToOneAndOrientationIsRelativeWithoutRetargetScale) {
  stable(); const auto prepared = command(ReferenceOperation::kPrepare);
  ASSERT_TRUE(prepared.success); ASSERT_TRUE(command(ReferenceOperation::kActivate).success);
  const Eigen::Vector3d delta(.025, -.01, .02);
  const Eigen::Matrix3d rotation = Eigen::AngleAxisd(.13, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
  for (const auto hand : {kPicoLeftHandPoint, kPicoRightHandPoint}) {
    frame.upper_limb_skeleton.points[hand] += delta;
    frame.upper_limb_skeleton.rotations[hand] = Eigen::Quaterniond(rotation);
  }
  sample(); const auto target = session.targets();
  EXPECT_LT((target.left.position - prepared.robot_reference[0].position - delta).norm(), 1e-12);
  EXPECT_LT((target.right.position - prepared.robot_reference[1].position - delta).norm(), 1e-12);
  EXPECT_LT(rotationDistance(target.left.rotation, rotation * prepared.robot_reference[0].rotation), 1e-7);
  EXPECT_LT(rotationDistance(target.right.rotation, rotation * prepared.robot_reference[1].rotation), 1e-7);
}

TEST_F(EpisodeRelative, StaticMovedInputDoesNotAccumulatePostureIterations) {
  stable(); ASSERT_TRUE(command(ReferenceOperation::kPrepare).success);
  ASSERT_TRUE(command(ReferenceOperation::kActivate).success);
  frame.upper_limb_skeleton.points[kPicoLeftHandPoint].x() += .005;
  frame.upper_limb_skeleton.points[kPicoRightHandPoint].x() += .005;
  sample(); const auto first = controller->step(session.targets(), .005);
  ASSERT_TRUE(first.accepted);
  auto previous = controller->referenceState(ArmSide::kLeft);
  for (int n = 0; n < 100; ++n) {
    sample(); const auto out = controller->step(session.targets(), .005);
    ASSERT_TRUE(out.accepted) << out.left.dls_posture_ruckig_detail;
    EXPECT_LT((out.left.dls_posture_goal - first.left.dls_posture_goal).norm(), 1e-12);
    const auto current = controller->referenceState(ArmSide::kLeft);
    const auto limits = controller->trajectorySampleLimits(ArmSide::kLeft);
    EXPECT_TRUE((current.qdot.cwiseAbs().array() <= limits.max_velocity_rad_s.array() + 1e-8).all());
    EXPECT_TRUE((current.qddot.cwiseAbs().array() <= limits.max_acceleration_rad_s2.array() + 1e-8).all());
    EXPECT_TRUE(((current.qddot-previous.qddot).cwiseAbs().array() / .005 <= limits.max_jerk_rad_s3.array() + 1e-6).all());
    previous = current;
  }
}

TEST_F(EpisodeRelative, InvalidRightMeasurementCannotPartiallyCaptureLeft) {
  stable(); const auto before = controller->reference(ArmSide::kLeft);
  auto bad = request(ReferenceOperation::kPrepare);
  bad.measured_q[1][3] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(session.command(bad, *controller, now).success);
  EXPECT_EQ(controller->reference(ArmSide::kLeft), before);
  EXPECT_FALSE(command(ReferenceOperation::kPrepare).success); // Reference IDs are consumed on attempts.
  bad = request(ReferenceOperation::kPrepare, 2);
  bad.measured_q[1][3] = controller->motionLimits(ArmSide::kRight).upper_position[3] + .1;
  EXPECT_FALSE(session.command(bad, *controller, now).success);
  EXPECT_EQ(controller->reference(ArmSide::kLeft), before);
}

TEST_F(EpisodeRelative, PrepareMovementCannotBeUndoneByReturningToCapture) {
  stable(); ASSERT_TRUE(command(ReferenceOperation::kPrepare).success);
  frame.upper_limb_skeleton.points[kPicoLeftHandPoint].x() += .03; sample();
  frame.upper_limb_skeleton.points[kPicoLeftHandPoint].x() -= .03; stable();
  EXPECT_FALSE(command(ReferenceOperation::kActivate).success);
  EXPECT_TRUE(session.ready(now)); EXPECT_EQ(session.referenceId(), 1U);
  expectStationary();
  ASSERT_TRUE(command(ReferenceOperation::kPrepare, 2).success);
  EXPECT_TRUE(command(ReferenceOperation::kActivate, 2).success);
}

TEST_F(EpisodeRelative, ExpiredPrepareStaysHeldAndRequiresNewReference) {
  stable(); ASSERT_TRUE(command(ReferenceOperation::kPrepare).success);
  for (int n = 0; n < 510; ++n) sample();
  EXPECT_FALSE(command(ReferenceOperation::kActivate).success);
  EXPECT_TRUE(session.ready(now)); EXPECT_EQ(session.referenceId(), 1U);
  expectStationary();
  EXPECT_TRUE(command(ReferenceOperation::kCancel).success);
  EXPECT_TRUE(command(ReferenceOperation::kPrepare, 2).success);
}

TEST_F(EpisodeRelative, TrackingResetAndStalenessNeverRebaselineActiveReference) {
  stable(); ASSERT_TRUE(command(ReferenceOperation::kPrepare).success);
  ASSERT_TRUE(command(ReferenceOperation::kActivate).success);
  ++frame.tracking_epoch; sample();
  EXPECT_TRUE(session.faulted()); EXPECT_FALSE(session.ready(now));
  stable(); EXPECT_FALSE(command(ReferenceOperation::kPrepare, 2).success);
  EXPECT_TRUE(command(ReferenceOperation::kCancel).success);
  EXPECT_TRUE(session.faulted()); EXPECT_FALSE(command(ReferenceOperation::kPrepare, 3).success);
}

TEST_F(EpisodeRelative, ReorderedLateCommandsAndRobotMovementCannotActivate) {
  stable();
  auto late = request(ReferenceOperation::kPrepare);
  late.measured_ns -= 100000001;
  EXPECT_FALSE(session.command(late, *controller, now).success);
  ASSERT_TRUE(command(ReferenceOperation::kPrepare, 2).success);
  EXPECT_FALSE(command(ReferenceOperation::kActivate, 1).success);
  auto moved = request(ReferenceOperation::kActivate, 2);
  moved.measured_q[1][0] += .02;
  EXPECT_FALSE(session.command(moved, *controller, now).success);
  EXPECT_FALSE(command(ReferenceOperation::kActivate, 2).success);
}

TEST_F(EpisodeRelative, WaistAndFullWindowStabilityAreRequiredAtPrepare) {
  stable();
  frame.upper_limb_skeleton.points[kPicoRightHandPoint].z() = 1.6; stable();
  EXPECT_FALSE(command(ReferenceOperation::kPrepare).success);
  frame.upper_limb_skeleton.points[kPicoRightHandPoint].z() = 1.0; sample();
  EXPECT_FALSE(command(ReferenceOperation::kPrepare, 2).success);
  stable(); EXPECT_TRUE(command(ReferenceOperation::kPrepare, 3).success);
  now += 100000001; session.tick(now);
  EXPECT_TRUE(session.faulted()); EXPECT_FALSE(session.ready(now));
}
}  // namespace
}  // namespace tianji_qp_ik
