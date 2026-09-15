#include "tianji_qp_ik/joint_kinematics_plot.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace tianji_qp_ik {
namespace {

TEST(JointKinematicsDifferentiatorTest,
     DifferentiatesVelocityReferenceAndActualIndependently) {
  JointKinematicsDifferentiator differentiator;
  const Vec7 zero = Vec7::Zero();

  const JointKinematicsDerivatives initial = differentiator.update(
      zero, zero, ReferenceAccelerationSource::kDifferentiateVelocity, zero,
      0.005, false);
  EXPECT_FALSE(initial.reference_acceleration_valid);
  EXPECT_FALSE(initial.actual_acceleration_valid);

  const JointKinematicsDerivatives second = differentiator.update(
      Vec7::Constant(0.1), zero,
      ReferenceAccelerationSource::kDifferentiateVelocity,
      Vec7::Constant(0.2), 0.005, false);
  EXPECT_TRUE(second.reference_acceleration_valid);
  EXPECT_TRUE(second.actual_acceleration_valid);
  EXPECT_FALSE(second.reference_jerk_valid);
  EXPECT_FALSE(second.actual_jerk_valid);
  EXPECT_TRUE(
      second.reference_acceleration.isApprox(Vec7::Constant(20.0), 1e-12));
  EXPECT_TRUE(
      second.actual_acceleration.isApprox(Vec7::Constant(40.0), 1e-12));

  const JointKinematicsDerivatives third = differentiator.update(
      Vec7::Constant(0.3), zero,
      ReferenceAccelerationSource::kDifferentiateVelocity,
      Vec7::Constant(0.6), 0.005, false);
  EXPECT_TRUE(third.reference_jerk_valid);
  EXPECT_TRUE(third.actual_jerk_valid);
  EXPECT_TRUE(third.reference_jerk.isApprox(Vec7::Constant(4000.0), 1e-9));
  EXPECT_TRUE(third.actual_jerk.isApprox(Vec7::Constant(8000.0), 1e-9));
}

TEST(JointKinematicsDifferentiatorTest,
     UsesDirectQpAccelerationAndResetsOnSourceChange) {
  JointKinematicsDifferentiator differentiator;
  const Vec7 zero = Vec7::Zero();
  (void)differentiator.update(
      zero, zero, ReferenceAccelerationSource::kDifferentiateVelocity, zero,
      0.005, false);
  (void)differentiator.update(
      Vec7::Constant(0.1), zero,
      ReferenceAccelerationSource::kDifferentiateVelocity,
      Vec7::Constant(0.1), 0.005, false);

  const JointKinematicsDerivatives first_direct = differentiator.update(
      Vec7::Constant(0.1), Vec7::Constant(7.0),
      ReferenceAccelerationSource::kDirectQpOutput,
      Vec7::Constant(0.1), 0.005, false);
  EXPECT_TRUE(first_direct.reference_acceleration_valid);
  EXPECT_TRUE(first_direct.reference_acceleration.isApprox(Vec7::Constant(7.0)));
  EXPECT_FALSE(first_direct.reference_jerk_valid);
  EXPECT_FALSE(first_direct.actual_acceleration_valid);

  const JointKinematicsDerivatives second_direct = differentiator.update(
      Vec7::Constant(0.12), Vec7::Constant(8.0),
      ReferenceAccelerationSource::kDirectQpOutput,
      Vec7::Constant(0.13), 0.005, false);
  EXPECT_TRUE(second_direct.reference_jerk_valid);
  EXPECT_TRUE(second_direct.reference_jerk.isApprox(Vec7::Constant(200.0)));
  EXPECT_TRUE(second_direct.actual_acceleration_valid);
}

TEST(JointKinematicsDifferentiatorTest,
     ResetAndInvalidInputSuppressDerivativeSpikes) {
  JointKinematicsDifferentiator differentiator;
  const Vec7 zero = Vec7::Zero();
  (void)differentiator.update(
      zero, zero, ReferenceAccelerationSource::kDifferentiateVelocity, zero,
      0.005, false);
  (void)differentiator.update(
      Vec7::Ones(), zero,
      ReferenceAccelerationSource::kDifferentiateVelocity, Vec7::Ones(),
      0.005, false);

  const JointKinematicsDerivatives reset = differentiator.update(
      Vec7::Constant(-3.0), zero,
      ReferenceAccelerationSource::kDifferentiateVelocity,
      Vec7::Constant(4.0), 0.005, true);
  EXPECT_FALSE(reset.reference_acceleration_valid);
  EXPECT_FALSE(reset.actual_acceleration_valid);
  EXPECT_TRUE(reset.reference_acceleration.isZero());
  EXPECT_TRUE(reset.actual_jerk.isZero());

  Vec7 non_finite = Vec7::Zero();
  non_finite[3] = std::numeric_limits<double>::quiet_NaN();
  const JointKinematicsDerivatives invalid = differentiator.update(
      non_finite, zero, ReferenceAccelerationSource::kDirectQpOutput, zero,
      0.005, false);
  EXPECT_FALSE(invalid.reference_acceleration_valid);
  EXPECT_FALSE(invalid.actual_acceleration_valid);

  const JointKinematicsDerivatives invalid_dt = differentiator.update(
      zero, zero, ReferenceAccelerationSource::kDifferentiateVelocity, zero,
      0.0, false);
  EXPECT_FALSE(invalid_dt.reference_acceleration_valid);
}

TEST(JointKinematicsHistoryTest,
     PreservesChronologicalOrderAndSeparatesBothArms) {
  JointKinematicsHistory history(3U);
  for (std::uint64_t sequence = 1U; sequence <= 4U; ++sequence) {
    JointKinematicsSample sample;
    sample.sequence = sequence;
    sample.time_seconds = static_cast<double>(sequence);
    sample.left.reference.position.setConstant(static_cast<double>(sequence));
    sample.left.actual.position.setConstant(
        10.0 + static_cast<double>(sequence));
    sample.right.reference.position.setConstant(-static_cast<double>(sequence));
    history.push(sample);
  }

  EXPECT_EQ(history.size(), 3U);
  const JointPlotSeries left =
      history.series(ArmSide::kLeft, PlotMetric::kPosition, 0);
  const JointPlotSeries right =
      history.series(ArmSide::kRight, PlotMetric::kPosition, 0);
  ASSERT_EQ(left.time.size(), 3U);
  EXPECT_DOUBLE_EQ(left.time.front(), 2.0);
  EXPECT_DOUBLE_EQ(left.reference.front(), 2.0);
  EXPECT_DOUBLE_EQ(left.reference.back(), 4.0);
  EXPECT_DOUBLE_EQ(left.actual.front(), 12.0);
  EXPECT_DOUBLE_EQ(right.reference.front(), -2.0);
  EXPECT_DOUBLE_EQ(right.reference.back(), -4.0);
}

TEST(JointKinematicsHistoryTest,
     KeepsReferenceAndActualDerivativeValidityIndependent) {
  JointKinematicsHistory history(4U);
  JointKinematicsSample sample;
  sample.time_seconds = 1.0;
  sample.left.reference.acceleration.setConstant(7.0);
  sample.left.actual.acceleration.setConstant(13.0);
  sample.left.reference.jerk.setConstant(11.0);
  sample.left.actual.jerk.setConstant(17.0);
  sample.left.reference_acceleration_valid = true;
  sample.left.actual_acceleration_valid = false;
  sample.left.reference_jerk_valid = false;
  sample.left.actual_jerk_valid = true;
  history.push(sample);

  const JointPlotSeries acceleration =
      history.series(ArmSide::kLeft, PlotMetric::kAcceleration, 3);
  const JointPlotSeries jerk =
      history.series(ArmSide::kLeft, PlotMetric::kJerk, 3);
  ASSERT_EQ(acceleration.reference_valid.size(), 1U);
  EXPECT_TRUE(acceleration.reference_valid[0]);
  EXPECT_FALSE(acceleration.actual_valid[0]);
  EXPECT_FALSE(jerk.reference_valid[0]);
  EXPECT_TRUE(jerk.actual_valid[0]);
  EXPECT_DOUBLE_EQ(acceleration.reference[0], 7.0);
  EXPECT_DOUBLE_EQ(acceleration.actual[0], 13.0);
}

TEST(JointKinematicsHistoryTest, RejectsInvalidCapacityAndJointIndex) {
  EXPECT_THROW(JointKinematicsHistory(0U), std::invalid_argument);
  JointKinematicsHistory history(2U);
  EXPECT_THROW(history.series(ArmSide::kLeft, PlotMetric::kPosition, -1),
               std::out_of_range);
  EXPECT_THROW(history.series(ArmSide::kLeft, PlotMetric::kPosition, kArmDof),
               std::out_of_range);
}

TEST(JointKinematicsPlotMetricTest, CyclesNamesAndUnits) {
  EXPECT_STREQ(plotMetricName(PlotMetric::kPosition), "q");
  EXPECT_STREQ(plotMetricUnit(PlotMetric::kVelocity), "rad/s");
  EXPECT_EQ(nextPlotMetric(PlotMetric::kPosition), PlotMetric::kVelocity);
  EXPECT_EQ(nextPlotMetric(PlotMetric::kVelocity), PlotMetric::kAcceleration);
  EXPECT_EQ(nextPlotMetric(PlotMetric::kAcceleration), PlotMetric::kJerk);
  EXPECT_EQ(nextPlotMetric(PlotMetric::kJerk), PlotMetric::kPosition);
}

}  // namespace
}  // namespace tianji_qp_ik
