#include "tianji_qp_ik/pico_teleop_session.hpp"

#include <gtest/gtest.h>

#include <cstdint>

namespace tianji_qp_ik {
namespace {

PicoTeleopFrame frameAt(std::uint64_t epoch, std::uint64_t sequence,
                        std::int64_t receive_monotonic_ns) {
  PicoTeleopFrame frame;
  frame.tracking_epoch = epoch;
  frame.sequence = sequence;
  frame.receive_monotonic_ns = receive_monotonic_ns;
  return frame;
}

PicoTeleopFrame buttonFrame(std::uint64_t epoch,
                            std::uint64_t resynchronization_generation,
                            bool reported_state) {
  PicoTeleopFrame frame = frameAt(epoch, 1U, 1000000000LL);
  frame.resynchronization_generation = resynchronization_generation;
  frame.user_button_pressed = reported_state;
  return frame;
}

TEST(PicoTeleopSession, SelectsOneConsistentClockDomainForTargets) {
  constexpr std::int64_t kMonotonicNs = 987654321000LL;
  EXPECT_DOUBLE_EQ(selectTargetTimeSeconds(false, 1.25, kMonotonicNs), 1.25);
  EXPECT_DOUBLE_EQ(selectTargetTimeSeconds(true, 1.25, kMonotonicNs),
                   987.654321);
  EXPECT_DOUBLE_EQ(monotonicTimestampSeconds(kMonotonicNs), 987.654321);
  EXPECT_DOUBLE_EQ(monotonicTimestampSeconds(0), 0.0);
}

TEST(PicoTeleopSession, ClassifiesEnableFreshDuplicateAndStaleTransitions) {
  PicoTeleopSession session(0.050);
  session.setEnabled(true);
  const auto first =
      session.classify(frameAt(9U, 1U, 1000000000LL), 1000000000LL);
  EXPECT_EQ(first.action, PicoTeleopAction::kResetEpochAndApply);
  EXPECT_DOUBLE_EQ(first.frame_age_seconds, 0.0);
  EXPECT_FALSE(session.freshness(1000000000LL).live);
  session.commitApplied(frameAt(9U, 1U, 1000000000LL));
  EXPECT_TRUE(session.freshness(1000000000LL).live);

  const auto next =
      session.classify(frameAt(9U, 2U, 1014000000LL), 1015000000LL);
  EXPECT_EQ(next.action, PicoTeleopAction::kApply);
  EXPECT_DOUBLE_EQ(next.frame_age_seconds, 0.001);
  session.commitApplied(frameAt(9U, 2U, 1014000000LL));

  const auto duplicate =
      session.classify(frameAt(9U, 2U, 1016000000LL), 1017000000LL);
  EXPECT_EQ(duplicate.action, PicoTeleopAction::kIgnoreAlreadyApplied);

  session.setEnabled(false);
  EXPECT_EQ(session.classify(frameAt(9U, 3U, 1028000000LL), 1029000000LL).action,
            PicoTeleopAction::kIgnoreDisabled);
  const PicoTeleopFreshness disabled = session.freshness(1030000000LL);
  EXPECT_FALSE(disabled.live);
  EXPECT_FALSE(disabled.stale);

  session.setEnabled(true);
  const PicoTeleopFreshness awaiting = session.freshness(1030000000LL);
  EXPECT_FALSE(awaiting.live);
  EXPECT_FALSE(awaiting.stale);
  EXPECT_FALSE(awaiting.has_applied_frame);
  EXPECT_EQ(session.classify(frameAt(9U, 3U, 1028000000LL), 1080000000LL).action,
            PicoTeleopAction::kIgnoreStale);

  const PicoTeleopFreshness freshness = session.freshness(1080000000LL);
  EXPECT_FALSE(freshness.has_applied_frame);
  EXPECT_FALSE(freshness.live);
  EXPECT_FALSE(freshness.stale);
  EXPECT_DOUBLE_EQ(freshness.frame_age_seconds, 0.0);
}

TEST(PicoTeleopSession, ReenableStartsANewTakeoverEpochWithinSameStream) {
  PicoTeleopSession session(0.050);
  session.setEnabled(true);
  const PicoTeleopFrame first = frameAt(9U, 1U, 1000000000LL);
  ASSERT_EQ(session.classify(first, 1000000000LL).action,
            PicoTeleopAction::kResetEpochAndApply);
  session.commitApplied(first);

  session.setEnabled(false);
  session.setEnabled(true);
  const PicoTeleopFrame reenabled = frameAt(9U, 2U, 1010000000LL);
  EXPECT_EQ(session.classify(reenabled, 1011000000LL).action,
            PicoTeleopAction::kResetEpochAndApply);
}

TEST(PicoTeleopSession, AButtonTogglesOncePerReportedStateChange) {
  PicoTeleopSession session(0.050);
  session.setEnabled(true);

  EXPECT_EQ(session.observeButton(buttonFrame(9U, 0U, false)),
            PicoTeleopButtonAction::kNone);
  EXPECT_EQ(session.observeButton(buttonFrame(9U, 0U, true)),
            PicoTeleopButtonAction::kPause);
  EXPECT_FALSE(session.enabled());
  EXPECT_EQ(session.observeButton(buttonFrame(9U, 0U, true)),
            PicoTeleopButtonAction::kNone);
  EXPECT_EQ(session.observeButton(buttonFrame(9U, 0U, false)),
            PicoTeleopButtonAction::kResume);
  EXPECT_TRUE(session.enabled());
  EXPECT_EQ(session.observeButton(buttonFrame(9U, 0U, false)),
            PicoTeleopButtonAction::kNone);
  EXPECT_EQ(session.observeButton(buttonFrame(9U, 0U, true)),
            PicoTeleopButtonAction::kPause);
  EXPECT_FALSE(session.enabled());
}

TEST(PicoTeleopSession, InitialReportedStateDoesNotTriggerAnAction) {
  PicoTeleopSession session(0.050);
  session.setEnabled(false);

  EXPECT_EQ(session.observeButton(buttonFrame(9U, 0U, true)),
            PicoTeleopButtonAction::kNone);
  EXPECT_FALSE(session.enabled());
  EXPECT_EQ(session.observeButton(buttonFrame(9U, 0U, true)),
            PicoTeleopButtonAction::kNone);
  EXPECT_EQ(session.observeButton(buttonFrame(9U, 0U, false)),
            PicoTeleopButtonAction::kResume);
  EXPECT_TRUE(session.enabled());
}

TEST(PicoTeleopSession, NewTrackingEpochOnlyReinitializesButtonState) {
  PicoTeleopSession session(0.050);
  session.setEnabled(true);

  EXPECT_EQ(session.observeButton(buttonFrame(9U, 0U, false)),
            PicoTeleopButtonAction::kNone);
  EXPECT_EQ(session.observeButton(buttonFrame(9U, 0U, true)),
            PicoTeleopButtonAction::kPause);
  ASSERT_FALSE(session.enabled());

  EXPECT_EQ(session.observeButton(buttonFrame(10U, 0U, false)),
            PicoTeleopButtonAction::kNone);
  EXPECT_FALSE(session.enabled());
  EXPECT_EQ(session.observeButton(buttonFrame(10U, 0U, true)),
            PicoTeleopButtonAction::kResume);
  EXPECT_TRUE(session.enabled());
}

TEST(PicoTeleopSession, ResynchronizationOnlyReinitializesButtonState) {
  PicoTeleopSession session(0.050);
  session.setEnabled(true);

  EXPECT_EQ(session.observeButton(buttonFrame(9U, 0U, false)),
            PicoTeleopButtonAction::kNone);
  EXPECT_EQ(session.observeButton(buttonFrame(9U, 0U, true)),
            PicoTeleopButtonAction::kPause);
  ASSERT_FALSE(session.enabled());

  EXPECT_EQ(session.observeButton(buttonFrame(9U, 1U, false)),
            PicoTeleopButtonAction::kNone);
  EXPECT_FALSE(session.enabled());
  EXPECT_EQ(session.observeButton(buttonFrame(9U, 1U, true)),
            PicoTeleopButtonAction::kResume);
  EXPECT_TRUE(session.enabled());
}

TEST(PicoTeleopSession, TimeoutBoundaryIsStaleAndNewEpochResets) {
  PicoTeleopSession session(0.050);
  session.setEnabled(true);
  ASSERT_EQ(session.classify(frameAt(9U, 1U, 1000000000LL), 1000000000LL).action,
            PicoTeleopAction::kResetEpochAndApply);
  session.commitApplied(frameAt(9U, 1U, 1000000000LL));
  const PicoTeleopFreshness boundary = session.freshness(1050000000LL);
  EXPECT_FALSE(boundary.live);
  EXPECT_TRUE(boundary.stale);
  EXPECT_DOUBLE_EQ(boundary.frame_age_seconds, 0.050);

  EXPECT_EQ(session.classify(frameAt(10U, 1U, 1060000000LL), 1061000000LL).action,
            PicoTeleopAction::kResetEpochAndApply);
  EXPECT_FALSE(session.freshness(1061000000LL).live);
  session.commitApplied(frameAt(10U, 1U, 1060000000LL));
  EXPECT_TRUE(session.freshness(1061000000LL).live);
}

TEST(PicoTeleopSession, StreamDiscontinuityResetsWithinSameEpoch) {
  PicoTeleopSession session(0.050);
  session.setEnabled(true);
  const PicoTeleopFrame first = frameAt(9U, 1U, 1000000000LL);
  ASSERT_EQ(session.classify(first, 1000000000LL).action,
            PicoTeleopAction::kResetEpochAndApply);
  session.commitApplied(first);

  PicoTeleopFrame discontinuity = frameAt(9U, 4U, 1010000000LL);
  discontinuity.stream_discontinuity = true;
  EXPECT_EQ(session.classify(discontinuity, 1011000000LL).action,
            PicoTeleopAction::kResetEpochAndApply);
}

TEST(PicoTeleopSession, PersistedGenerationSurvivesEventFrameSupersession) {
  PicoTeleopSession session(0.050);
  session.setEnabled(true);
  const PicoTeleopFrame first = frameAt(9U, 1U, 1000000000LL);
  session.commitApplied(first);

  PicoTeleopFrame newest = frameAt(9U, 5U, 1010000000LL);
  newest.resynchronization_generation = 1U;
  EXPECT_EQ(session.classify(newest, 1011000000LL).action,
            PicoTeleopAction::kResetEpochAndApply);
  session.commitApplied(newest);

  PicoTeleopFrame following = frameAt(9U, 6U, 1020000000LL);
  following.resynchronization_generation = 1U;
  EXPECT_EQ(session.classify(following, 1021000000LL).action,
            PicoTeleopAction::kApply);

  PicoTeleopFrame older_generation = frameAt(9U, 7U, 1030000000LL);
  EXPECT_EQ(session.classify(older_generation, 1031000000LL).action,
            PicoTeleopAction::kApply);
  session.commitApplied(older_generation);

  PicoTeleopFrame current_generation = frameAt(9U, 8U, 1040000000LL);
  current_generation.resynchronization_generation = 1U;
  EXPECT_EQ(session.classify(current_generation, 1041000000LL).action,
            PicoTeleopAction::kApply);
}

TEST(PicoTeleopSession, RejectedApplicationDoesNotRefreshFreshness) {
  PicoTeleopSession session(0.050);
  session.setEnabled(true);
  const PicoTeleopFrame first = frameAt(4U, 10U, 1000000000LL);
  ASSERT_EQ(session.classify(first, 1000000000LL).action,
            PicoTeleopAction::kResetEpochAndApply);
  session.commitApplied(first);

  const PicoTeleopFrame candidate = frameAt(4U, 11U, 1040000000LL);
  ASSERT_EQ(session.classify(candidate, 1040000000LL).action,
            PicoTeleopAction::kApply);
  EXPECT_TRUE(session.freshness(1049000000LL).live);
  EXPECT_TRUE(session.freshness(1050000000LL).stale);
}

TEST(PicoTeleopSession, InvalidInputRevokesWithoutApplyingAZeroPose) {
  PicoTeleopSession session(0.050);
  session.setEnabled(true);
  session.commitApplied(frameAt(9U, 5U, 1000000000LL));
  ASSERT_TRUE(session.freshness(1001000000LL).live);

  auto invalid = frameAt(9U, 6U, 1002000000LL);
  invalid.valid = false;
  invalid.resynchronization_generation = 1U;
  EXPECT_EQ(session.classify(invalid, 1002000000LL).action,
            PicoTeleopAction::kIgnoreStale);
  session.invalidate();
  EXPECT_FALSE(session.freshness(1002000000LL).live);
  EXPECT_TRUE(session.enabled());
  EXPECT_EQ(session.classify(frameAt(9U, 5U, 1003000000LL), 1003000000LL).action,
            PicoTeleopAction::kIgnoreAlreadyApplied);

  auto recovery = frameAt(9U, 7U, 1004000000LL);
  recovery.resynchronization_generation = 1U;
  EXPECT_EQ(session.classify(recovery, 1004000000LL).action,
            PicoTeleopAction::kResetEpochAndApply);
  EXPECT_FALSE(session.freshness(1004000000LL).live);
  session.commitApplied(recovery);
  EXPECT_TRUE(session.freshness(1004000000LL).live);
}

TEST(PicoTeleopSession, PublisherRestartRequiresANewReceiverGeneration) {
  PicoTeleopSession session(0.050);
  session.setEnabled(true);
  session.commitApplied(frameAt(9U, 50U, 1000000000LL));
  auto restarted = frameAt(1U, 1U, 1002000000LL);
  EXPECT_EQ(session.classify(restarted, 1002000000LL).action,
            PicoTeleopAction::kIgnoreAlreadyApplied);
  // A typed ROS publisher restart can reset both epoch and sequence. The
  // receiver-local generation survives coalescing of the reset event itself.
  restarted.resynchronization_generation = 1U;
  EXPECT_EQ(session.classify(restarted, 1002000000LL).action,
            PicoTeleopAction::kResetEpochAndApply);
  session.commitApplied(restarted);
  EXPECT_EQ(session.classify(restarted, 1003000000LL).action,
            PicoTeleopAction::kIgnoreAlreadyApplied);
}


}  // namespace
}  // namespace tianji_qp_ik
