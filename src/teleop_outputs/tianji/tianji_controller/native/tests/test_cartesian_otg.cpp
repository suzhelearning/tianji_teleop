#include "tianji_qp_ik/cartesian_otg.hpp"

#include "tianji_qp_ik/so3.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <cmath>
#include <limits>

namespace tianji_qp_ik {
namespace {

constexpr double kDt = 0.005;

CartesianOtgConfig testConfig() {
  return {};
}

Pose identityPose() { return {}; }

TEST(CartesianOtg, ResetStartsAtMeasuredPoseWithZeroDerivatives) {
  CartesianReferenceGenerator generator(testConfig(), kDt);
  Pose measured;
  measured.position = {0.2, -0.1, 0.5};
  measured.rotation = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY()).toRotationMatrix();
  generator.reset(measured);

  EXPECT_TRUE(generator.state().pose.position.isApprox(measured.position));
  EXPECT_TRUE(generator.state().pose.rotation.isApprox(measured.rotation));
  EXPECT_TRUE(generator.state().twist.isZero());
  EXPECT_TRUE(generator.state().acceleration.isZero());
  EXPECT_TRUE(generator.state().valid);
}

TEST(CartesianOtg, TranslationRespectsVelocityAccelerationAndJerk) {
  const CartesianOtgConfig config = testConfig();
  CartesianReferenceGenerator generator(config, kDt);
  generator.reset(identityPose());
  Pose target;
  target.position.x() = 1.0;
  Vec6 target_twist = Vec6::Zero();
  target_twist.x() = 0.4;
  Eigen::Vector3d previous_acceleration = Eigen::Vector3d::Zero();

  for (int step = 0; step < 300; ++step) {
    const CartesianReference reference =
        generator.update(target, target_twist, false, kDt);
    ASSERT_TRUE(reference.valid) << "step=" << step;
    EXPECT_LE(reference.twist.head<3>().norm(),
              config.translation_velocity_max + 1e-9);
    EXPECT_LE(reference.acceleration.head<3>().norm(),
              config.translation_acceleration_max + 1e-9);
    EXPECT_LE((reference.acceleration.head<3>() - previous_acceleration).norm(),
              config.translation_jerk_max * kDt + 1e-8);
    previous_acceleration = reference.acceleration.head<3>();
  }
  EXPECT_GT(generator.state().pose.position.x(), 0.1);
}

TEST(CartesianOtg, OrientationUsesShortestWorldFrameRotation) {
  CartesianReferenceGenerator generator(testConfig(), kDt);
  generator.reset(identityPose());
  Pose target;
  target.rotation =
      Eigen::AngleAxisd(3.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  const CartesianReference reference =
      generator.update(target, Vec6::Zero(), false, kDt);
  ASSERT_TRUE(reference.valid);
  EXPECT_GT(reference.twist.tail<3>().z(), 0.0);
  EXPECT_LT(rotationDistance(reference.pose.rotation, target.rotation), 3.0);
}

TEST(CartesianOtg, OrientationRespectsAngularDerivativeLimits) {
  const CartesianOtgConfig config = testConfig();
  CartesianReferenceGenerator generator(config, kDt);
  generator.reset(identityPose());
  Pose target;
  target.rotation = Eigen::AngleAxisd(2.0, Eigen::Vector3d(1.0, 1.0, 1.0).normalized())
                        .toRotationMatrix();
  Eigen::Vector3d previous_acceleration = Eigen::Vector3d::Zero();

  for (int step = 0; step < 300; ++step) {
    const CartesianReference reference =
        generator.update(target, Vec6::Zero(), false, kDt);
    ASSERT_TRUE(reference.valid) << "step=" << step;
    EXPECT_LE(reference.twist.tail<3>().norm(),
              config.angular_velocity_max + 1e-9);
    EXPECT_LE(reference.acceleration.tail<3>().norm(),
              config.angular_acceleration_max + 1e-9);
    EXPECT_LE((reference.acceleration.tail<3>() - previous_acceleration).norm(),
              config.angular_jerk_max * kDt + 1e-9);
    previous_acceleration = reference.acceleration.tail<3>();
  }
}

TEST(CartesianOtg, StaleInputDeceleratesWithoutPoseJump) {
  CartesianReferenceGenerator generator(testConfig(), kDt);
  generator.reset(identityPose());
  Pose target;
  target.position.x() = 1.0;
  CartesianReference reference;
  for (int step = 0; step < 80; ++step) {
    reference = generator.update(target, Vec6::Zero(), false, kDt);
  }
  const double speed_before = reference.twist.head<3>().norm();
  const Pose pose_before = reference.pose;
  reference = generator.update(target, Vec6::Zero(), true, kDt);

  EXPECT_TRUE(reference.stale);
  EXPECT_TRUE(reference.valid);
  EXPECT_LT((reference.pose.position - pose_before.position).norm(),
            testConfig().translation_velocity_max * kDt + 1e-9);
  EXPECT_GT(reference.twist.head<3>().norm(), 0.0);
  EXPECT_LE(std::abs(reference.twist.head<3>().norm() - speed_before),
            testConfig().translation_acceleration_max * kDt + 1e-8);
}

TEST(CartesianOtg, StaleOrientationContinuesToTargetAndSettlesWithoutReversal) {
  CartesianReferenceGenerator generator(testConfig(), kDt);
  generator.reset(identityPose());
  Pose target;
  target.rotation =
      Eigen::AngleAxisd(0.8, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  ASSERT_TRUE(generator.update(target, Vec6::Zero(), false, kDt).valid);

  double previous_velocity = generator.state().twist.tail<3>().z();
  int reversals = 0;
  for (int step = 0; step < 2000; ++step) {
    const CartesianReference reference =
        generator.update(target, Vec6::Zero(), true, kDt);
    ASSERT_TRUE(reference.valid);
    const double velocity = reference.twist.tail<3>().z();
    if (previous_velocity > 1e-6 && velocity < -1e-6) {
      ++reversals;
    }
    previous_velocity = velocity;
  }

  EXPECT_EQ(reversals, 0);
  EXPECT_LT(rotationDistance(generator.state().pose.rotation,
                             target.rotation),
            1e-8);
  EXPECT_TRUE(generator.state().twist.tail<3>().isZero(1e-12));
  EXPECT_TRUE(generator.state().acceleration.tail<3>().isZero(1e-12));
}

TEST(CartesianOtg, ContinuouslyChangingOrientationRemainsValidAndBounded) {
  const CartesianOtgConfig config = testConfig();
  CartesianReferenceGenerator generator(config, kDt);
  generator.reset(identityPose());
  Eigen::Vector3d previous_acceleration = Eigen::Vector3d::Zero();

  for (int step = 0; step < 1000; ++step) {
    const double time = static_cast<double>(step) * kDt;
    const Eigen::Vector3d rotation_vector(
        0.45 * std::sin(1.1 * time),
        0.35 * std::sin(0.7 * time),
        0.30 * std::cos(0.9 * time));
    Pose target;
    const double angle = rotation_vector.norm();
    target.rotation =
        Eigen::AngleAxisd(angle, rotation_vector / angle).toRotationMatrix();
    const CartesianReference reference =
        generator.update(target, Vec6::Zero(), false, kDt);

    ASSERT_TRUE(reference.valid) << "step=" << step;
    EXPECT_LE(reference.twist.tail<3>().norm(),
              config.angular_velocity_max + 1e-9);
    EXPECT_LE(reference.acceleration.tail<3>().norm(),
              config.angular_acceleration_max + 1e-9);
    EXPECT_LE((reference.acceleration.tail<3>() - previous_acceleration).norm(),
              config.angular_jerk_max * kDt + 1e-8);
    previous_acceleration = reference.acceleration.tail<3>();
  }
}

TEST(CartesianOtg, ChangingRotationAxesThenStaleConvergesToFinalTarget) {
  CartesianOtgConfig config = testConfig();
  config.stationary_hold_enabled = false;
  CartesianReferenceGenerator generator(config, kDt);
  generator.reset(identityPose());

  Pose target;
  for (int step = 0; step < 1000; ++step) {
    const double time = static_cast<double>(step) * kDt;
    const Eigen::Vector3d rotation_vector(
        1.4 * std::sin(1.3 * time),
        1.1 * std::sin(0.9 * time + 0.4),
        1.2 * std::cos(1.1 * time));
    const double angle = rotation_vector.norm();
    target.rotation =
        Eigen::AngleAxisd(angle, rotation_vector / angle).toRotationMatrix();
    Vec6 target_twist = Vec6::Zero();
    target_twist.tail<3>() =
        Eigen::Vector3d(1.0, -0.7, 0.9);
    ASSERT_TRUE(generator.update(target, target_twist, false, kDt).valid);
  }

  for (int step = 0; step < 3000; ++step) {
    ASSERT_TRUE(generator.update(target, Vec6::Zero(), true, kDt).valid);
  }

  EXPECT_LT(rotationDistance(generator.state().pose.rotation,
                             target.rotation),
            1.0e-6);
  EXPECT_TRUE(generator.state().twist.tail<3>().isZero(1.0e-8));
  EXPECT_TRUE(generator.state().acceleration.tail<3>().isZero(1.0e-7));
}

TEST(CartesianOtg, VelocityTrackingReducesMovingCircleLagAndPreservesLimits) {
  CartesianOtgConfig position_config = testConfig();
  position_config.translation_velocity_max = 2.0;
  position_config.translation_acceleration_max = 12.0;
  position_config.translation_jerk_max = 120.0;
  position_config.translation_tracking_enabled = false;

  CartesianOtgConfig tracking_config = position_config;
  tracking_config.translation_tracking_enabled = true;
  tracking_config.translation_tracking_gain = 30.0;

  constexpr double kRadius = 0.075;
  constexpr double kPeriodSeconds = 4.0;
  constexpr double kAngularSpeed = 2.0 * M_PI / kPeriodSeconds;
  constexpr int kCircleSteps = static_cast<int>(kPeriodSeconds / kDt);
  Pose initial;
  initial.position = {kRadius, 0.0, 0.0};

  CartesianReferenceGenerator position_generator(position_config, kDt);
  CartesianReferenceGenerator tracking_generator(tracking_config, kDt);
  position_generator.reset(initial);
  tracking_generator.reset(initial);

  double position_error_sum = 0.0;
  double tracking_error_sum = 0.0;
  int evaluated_steps = 0;
  Eigen::Vector3d previous_tracking_acceleration = Eigen::Vector3d::Zero();

  for (int step = 1; step <= kCircleSteps; ++step) {
    const double time = static_cast<double>(step) * kDt;
    const double phase = kAngularSpeed * time;
    Pose target;
    target.position =
        {kRadius * std::cos(phase), kRadius * std::sin(phase), 0.0};
    Vec6 target_twist = Vec6::Zero();
    target_twist.head<3>() = Eigen::Vector3d(
        -kRadius * kAngularSpeed * std::sin(phase),
        kRadius * kAngularSpeed * std::cos(phase), 0.0);

    const CartesianReference position_reference =
        position_generator.update(target, target_twist, false, kDt);
    const CartesianReference tracking_reference =
        tracking_generator.update(target, target_twist, false, kDt);
    ASSERT_TRUE(position_reference.valid) << "step=" << step;
    ASSERT_TRUE(tracking_reference.valid) << "step=" << step;

    EXPECT_LE(tracking_reference.twist.head<3>().norm(),
              tracking_config.translation_velocity_max + 1e-9);
    EXPECT_LE(tracking_reference.acceleration.head<3>()
                  .cwiseAbs()
                  .maxCoeff(),
              tracking_config.translation_acceleration_max + 1e-9);
    EXPECT_LE((tracking_reference.acceleration.head<3>() -
               previous_tracking_acceleration)
                  .cwiseAbs()
                  .maxCoeff(),
              tracking_config.translation_jerk_max * kDt + 1e-8);
    previous_tracking_acceleration =
        tracking_reference.acceleration.head<3>();

    if (time >= 0.5) {
      position_error_sum +=
          (position_reference.pose.position - target.position).norm();
      tracking_error_sum +=
          (tracking_reference.pose.position - target.position).norm();
      ++evaluated_steps;
    }
  }

  ASSERT_GT(evaluated_steps, 0);
  const double position_mean_error =
      position_error_sum / static_cast<double>(evaluated_steps);
  const double tracking_mean_error =
      tracking_error_sum / static_cast<double>(evaluated_steps);
  EXPECT_LT(tracking_mean_error, 0.75 * position_mean_error)
      << "position_mean_error=" << position_mean_error
      << " tracking_mean_error=" << tracking_mean_error;
  EXPECT_LT(tracking_mean_error, 0.00075);
}

TEST(CartesianOtg, VelocityTrackingUsesPositionModeToSettleStationaryTarget) {
  CartesianOtgConfig config = testConfig();
  config.translation_tracking_enabled = true;
  config.translation_tracking_gain = 30.0;
  config.translation_stationary_velocity_threshold = 1.0e-4;
  config.translation_velocity_max = 2.0;
  config.translation_acceleration_max = 12.0;
  config.translation_jerk_max = 120.0;
  CartesianReferenceGenerator generator(config, kDt);
  generator.reset(identityPose());

  Pose moving_target;
  Vec6 moving_twist = Vec6::Zero();
  moving_twist.x() = 0.2;
  for (int step = 1; step <= 100; ++step) {
    moving_target.position.x() = 0.2 * static_cast<double>(step) * kDt;
    ASSERT_TRUE(generator.update(moving_target, moving_twist, false, kDt).valid);
  }

  for (int step = 0; step < 200; ++step) {
    ASSERT_TRUE(
        generator.update(moving_target, Vec6::Zero(), false, kDt).valid);
  }
  EXPECT_LT((generator.state().pose.position - moving_target.position).norm(),
            1e-6);
  EXPECT_LT(generator.state().twist.head<3>().norm(), 1e-6);
}

TEST(CartesianOtg, TranslationPositionModeIgnoresLinearVelocityFeedforward) {
  CartesianOtgConfig position_config = testConfig();
  position_config.translation_position_mode = true;
  position_config.translation_tracking_enabled = true;
  position_config.translation_tracking_gain = 30.0;
  position_config.translation_velocity_max = 2.0;
  position_config.translation_acceleration_max = 12.0;
  position_config.translation_jerk_max = 120.0;

  CartesianOtgConfig legacy_config = position_config;
  legacy_config.translation_position_mode = false;

  CartesianReferenceGenerator position_generator(position_config, kDt);
  CartesianReferenceGenerator legacy_generator(legacy_config, kDt);
  position_generator.reset(identityPose());
  legacy_generator.reset(identityPose());

  Vec6 noisy_twist = Vec6::Zero();
  noisy_twist.x() = 0.5;
  for (int step = 0; step < 40; ++step) {
    ASSERT_TRUE(position_generator
                    .update(identityPose(), noisy_twist, false, kDt)
                    .valid);
    ASSERT_TRUE(legacy_generator
                    .update(identityPose(), noisy_twist, false, kDt)
                    .valid);
  }

  EXPECT_NEAR(position_generator.state().pose.position.x(), 0.0, 1e-12);
  EXPECT_NEAR(position_generator.state().twist.x(), 0.0, 1e-12);
  EXPECT_GT(legacy_generator.state().pose.position.x(), 1e-3);
  EXPECT_GT(std::abs(legacy_generator.state().twist.x()), 1e-2);
}

CartesianOtgConfig stationaryHoldConfig() {
  CartesianOtgConfig config = testConfig();
  config.translation_tracking_enabled = true;
  config.translation_tracking_gain = 30.0;
  config.translation_stationary_velocity_threshold = 1.0e-4;
  config.stationary_hold_enabled = true;
  config.stationary_hold_dwell_seconds = 0.15;
  config.translation_hold_enter_velocity_m_s = 0.06;
  config.translation_hold_exit_velocity_m_s = 0.12;
  config.translation_hold_exit_position_error_m = 0.008;
  config.orientation_hold_enter_velocity_rad_s = 0.12;
  config.orientation_hold_exit_velocity_rad_s = 0.25;
  config.orientation_hold_exit_error_rad = 0.035;
  config.translation_velocity_max = 2.0;
  config.translation_acceleration_max = 12.0;
  config.translation_jerk_max = 120.0;
  config.angular_velocity_max = 8.0;
  config.angular_acceleration_max = 40.0;
  config.angular_jerk_max = 400.0;
  return config;
}

TEST(CartesianOtg, StationaryHoldRejectsSubThresholdTranslationJitter) {
  const CartesianOtgConfig config = stationaryHoldConfig();
  CartesianReferenceGenerator generator(config, kDt);
  generator.reset(identityPose());

  double minimum_steady_position = std::numeric_limits<double>::infinity();
  double maximum_steady_position = -std::numeric_limits<double>::infinity();
  double maximum_steady_speed = 0.0;
  for (int step = 0; step < 800; ++step) {
    const double time = static_cast<double>(step) * kDt;
    Pose target;
    target.position.x() = 0.003 * std::sin(2.0 * M_PI * time);
    Vec6 target_twist = Vec6::Zero();
    target_twist.x() = 0.04 * std::cos(2.0 * M_PI * time);
    const CartesianReference reference =
        generator.update(target, target_twist, false, kDt);
    ASSERT_TRUE(reference.valid) << "step=" << step;
    if (step >= 600) {
      minimum_steady_position =
          std::min(minimum_steady_position, reference.pose.position.x());
      maximum_steady_position =
          std::max(maximum_steady_position, reference.pose.position.x());
      maximum_steady_speed = std::max(
          maximum_steady_speed, reference.twist.head<3>().norm());
    }
  }

  EXPECT_LT(maximum_steady_position - minimum_steady_position, 1.0e-5);
  EXPECT_LT(maximum_steady_speed, 1.0e-5);
}

TEST(CartesianOtg, StationaryHoldRejectsSubThresholdOrientationJitter) {
  const CartesianOtgConfig config = stationaryHoldConfig();
  CartesianReferenceGenerator generator(config, kDt);
  generator.reset(identityPose());

  double maximum_steady_rotation_span = 0.0;
  double maximum_steady_angular_speed = 0.0;
  Eigen::Matrix3d first_steady_rotation = Eigen::Matrix3d::Identity();
  for (int step = 0; step < 800; ++step) {
    const double time = static_cast<double>(step) * kDt;
    Pose target;
    target.rotation =
        Eigen::AngleAxisd(0.012 * std::sin(2.0 * M_PI * time),
                          Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
    Vec6 target_twist = Vec6::Zero();
    target_twist.tail<3>().z() = 0.08 * std::cos(2.0 * M_PI * time);
    const CartesianReference reference =
        generator.update(target, target_twist, false, kDt);
    ASSERT_TRUE(reference.valid) << "step=" << step;
    if (step == 600) {
      first_steady_rotation = reference.pose.rotation;
    }
    if (step >= 600) {
      maximum_steady_rotation_span = std::max(
          maximum_steady_rotation_span,
          rotationDistance(first_steady_rotation, reference.pose.rotation));
      maximum_steady_angular_speed = std::max(
          maximum_steady_angular_speed, reference.twist.tail<3>().norm());
    }
  }

  EXPECT_LT(maximum_steady_rotation_span, 1.0e-5);
  EXPECT_LT(maximum_steady_angular_speed, 1.0e-5);
}

TEST(CartesianOtg, StationaryHoldExitsForIntentionalMotion) {
  const CartesianOtgConfig config = stationaryHoldConfig();
  CartesianReferenceGenerator generator(config, kDt);
  generator.reset(identityPose());

  for (int step = 0; step < 200; ++step) {
    Pose jitter_target;
    jitter_target.position.x() = (step % 2 == 0) ? 0.002 : -0.002;
    Vec6 jitter_twist = Vec6::Zero();
    jitter_twist.x() = (step % 2 == 0) ? 0.04 : -0.04;
    ASSERT_TRUE(
        generator.update(jitter_target, jitter_twist, false, kDt).valid);
  }
  const double held_position = generator.state().pose.position.x();

  Pose intentional_target;
  intentional_target.position.x() = held_position + 0.05;
  Vec6 intentional_twist = Vec6::Zero();
  intentional_twist.x() = 0.20;
  for (int step = 0; step < 40; ++step) {
    ASSERT_TRUE(generator
                    .update(intentional_target, intentional_twist, false, kDt)
                    .valid);
  }

  EXPECT_GT(generator.state().pose.position.x() - held_position, 1.0e-3);
  EXPECT_GT(generator.state().twist.x(), 0.0);
}

TEST(CartesianOtg, StationaryHoldDoesNotFreezeBeforeReachingLargeTargetStep) {
  const CartesianOtgConfig config = stationaryHoldConfig();
  CartesianReferenceGenerator generator(config, kDt);
  generator.reset(identityPose());

  Pose target;
  target.position.x() = 0.20;
  for (int step = 0; step < 1200; ++step) {
    ASSERT_TRUE(generator.update(target, Vec6::Zero(), false, kDt).valid);
  }

  EXPECT_NEAR(generator.state().pose.position.x(), target.position.x(), 1.0e-5);
  EXPECT_LT(generator.state().twist.head<3>().norm(), 1.0e-5);
}

TEST(CartesianOtg, ReconfigurePreservesRunningReferenceState) {
  CartesianOtgConfig config = testConfig();
  config.translation_acceleration_max = 30.0;
  config.translation_jerk_max = 1000.0;
  config.angular_acceleration_max = 100.0;
  config.angular_jerk_max = 3000.0;
  CartesianReferenceGenerator generator(config, kDt);
  generator.reset(identityPose());

  Pose target;
  target.position.x() = 0.20;
  target.rotation =
      Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  for (int step = 0; step < 20; ++step) {
    ASSERT_TRUE(generator.update(target, Vec6::Zero(), false, kDt).valid);
  }
  const CartesianReference before = generator.state();

  config.translation_acceleration_max = 12.0;
  config.translation_jerk_max = 120.0;
  config.angular_acceleration_max = 40.0;
  config.angular_jerk_max = 400.0;
  generator.reconfigure(config);

  EXPECT_TRUE(generator.state().pose.position.isApprox(before.pose.position));
  EXPECT_TRUE(generator.state().pose.rotation.isApprox(before.pose.rotation));
  EXPECT_TRUE(generator.state().twist.isApprox(before.twist));
  EXPECT_TRUE(generator.state().acceleration.isApprox(before.acceleration));
  EXPECT_TRUE(generator.update(target, Vec6::Zero(), false, kDt).valid);
}

TEST(CartesianOtg, RejectsNonFiniteInputWithoutMutatingState) {
  CartesianReferenceGenerator generator(testConfig(), kDt);
  generator.reset(identityPose());
  Pose target;
  target.position.x() = 0.2;
  ASSERT_TRUE(generator.update(target, Vec6::Zero(), false, kDt).valid);
  const CartesianReference before = generator.state();
  target.position.y() = std::numeric_limits<double>::quiet_NaN();

  const CartesianReference rejected =
      generator.update(target, Vec6::Zero(), false, kDt);
  EXPECT_FALSE(rejected.valid);
  EXPECT_TRUE(generator.state().pose.position.isApprox(before.pose.position));
  EXPECT_TRUE(generator.state().pose.rotation.isApprox(before.pose.rotation));
  EXPECT_TRUE(generator.state().twist.isApprox(before.twist));
  EXPECT_TRUE(generator.state().acceleration.isApprox(before.acceleration));
}

}  // namespace
}  // namespace tianji_qp_ik
