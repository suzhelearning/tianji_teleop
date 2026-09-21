#include "tianji_qp_ik/pico_ee_franka_dls.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace tianji_qp_ik {
namespace {

ArmLimits limits() {
  ArmLimits result;
  result.lower_position = Vec7::Constant(-3.0);
  result.upper_position = Vec7::Constant(3.0);
  result.velocity = Vec7::Constant(10.0);
  return result;
}

JointVelocityBounds velocityBounds() {
  JointVelocityBounds result;
  result.lower = Vec7::Constant(-10.0);
  result.upper = Vec7::Constant(10.0);
  return result;
}

ArmKinematicSample evaluate(const Vec7& q) {
  ArmKinematicSample result;
  result.tcp_pose.position = q.head<3>();
  result.tcp_pose.rotation.setIdentity();
  result.tcp_jacobian.setZero();
  result.tcp_jacobian.block<3, 3>(0, 0).setIdentity();
  return result;
}

IterativeDlsConfig dlsConfig() {
  IterativeDlsConfig result;
  result.posture_reference_enabled = true;
  result.max_iterations = 8;
  result.position_tolerance_m = 1.0e-6;
  result.orientation_tolerance_rad = 1.0e-6;
  result.position_gain = 1.0;
  result.orientation_gain = 1.0;
  result.minimum_damping = 1.0e-6;
  result.maximum_damping = 0.1;
  result.singular_value_threshold = 0.05;
  result.maximum_step_norm_rad = 0.35;
  result.maximum_joint_step_rad = 0.20;
  result.minimum_merit_improvement = 1.0e-12;
  result.nominal_posture_gain = 0.5;
  result.arm_angle_gain = 0.0;
  result.maximum_goal_step_rad = 0.20;
  result.posture_reference_margin_rad = 0.05;
  return result;
}

PicoEeFrankaDlsConfig plannerConfig() {
  PicoEeFrankaDlsConfig result;
  result.enabled = true;
  result.bandwidth_rad_s = Vec7::Constant(15.0);
  result.max_velocity_rad_s = Vec7::Constant(4.0);
  result.max_acceleration_rad_s2 = Vec7::Constant(60.0);
  result.max_jerk_rad_s3 = Vec7::Constant(3000.0);
  result.maximum_target_joint_step_rad = 0.20;
  result.maximum_target_step_norm_rad = 0.35;
  result.planner_validation_tolerance = 1.0e-8;
  result.home_left_rad[6] = 0.6;
  return result;
}

PicoEeFrankaDlsInput inputFor(const Pose& target) {
  PicoEeFrankaDlsInput result;
  result.target = target;
  result.target_valid = true;
  result.target_stale = false;
  result.seed.setZero();
  result.seed_velocity.setZero();
  result.seed_acceleration.setZero();
  result.limits = limits();
  result.velocity_bounds = velocityBounds();
  result.home_reference = plannerConfig().home_left_rad;
  result.evaluate = evaluate;
  result.dt = 0.005;
  return result;
}

}  // namespace

TEST(PicoEeFrankaDls, UsesHomeInDlsNullspaceAndPublishesSafePlannerState) {
  PicoEeFrankaDlsIk7 solver(dlsConfig(), plannerConfig(), 0.05);
  Pose target;
  target.position << 0.10, -0.05, 0.02;
  target.rotation.setIdentity();

  const PicoEeFrankaDlsResult result = solver.solve(inputFor(target));

  EXPECT_TRUE(result.accepted);
  EXPECT_TRUE(result.planner_accepted);
  EXPECT_GT(result.goal[6], 0.0);
  EXPECT_LT(result.dls.position_error_m,
            result.dls.initial_position_error_m);
  EXPECT_TRUE((result.planner_state.q.array() >= -2.95).all());
  EXPECT_TRUE((result.planner_state.q.array() <= 2.95).all());
  EXPECT_TRUE((result.planner_state.qdot.cwiseAbs().array() <= 4.0 + 1.0e-8)
                  .all());
}

TEST(PicoEeFrankaDls, UsesPreviousBestGoalAsIkSeedInsteadOfPlannerState) {
  PicoEeFrankaDlsIk7 solver(dlsConfig(), plannerConfig(), 0.05);
  Pose first_target;
  first_target.position << 0.10, -0.05, 0.02;
  first_target.rotation.setIdentity();
  auto first_input = inputFor(first_target);
  const auto first = solver.solve(first_input);
  ASSERT_TRUE(first.accepted) << first.detail;

  Pose second_target;
  second_target.position << -0.12, 0.08, -0.04;
  second_target.rotation.setIdentity();
  auto second_input = inputFor(second_target);
  // Deliberately move the model reference far from the previous DLS goal.
  // The planner still owns this state, but it must not become the next IK
  // seed.
  second_input.seed = Vec7::Constant(1.5);
  second_input.seed_velocity = Vec7::Zero();
  second_input.seed_acceleration = Vec7::Zero();

  const auto second = solver.solve(second_input);
  ASSERT_TRUE(second.accepted) << second.detail;
  const ArmKinematicSample previous_goal_sample = evaluate(first.goal);
  const double expected_initial_error =
      (second_target.position - previous_goal_sample.tcp_pose.position).norm();
  EXPECT_NEAR(second.dls.initial_position_error_m, expected_initial_error,
              1.0e-12);
  EXPECT_NEAR(second.dls.initial_position_error_m,
              (second_target.position - first.goal.head<3>()).norm(),
              1.0e-12);
}

TEST(PicoEeFrankaDls, HoldsTargetAndKeepsSevenIndependentStatesClamped) {
  PicoEeFrankaDlsIk7 solver(dlsConfig(), plannerConfig(), 0.05);
  Pose target;
  target.position << 2.8, 0.0, 0.0;
  target.rotation.setIdentity();
  PicoEeFrankaDlsInput input = inputFor(target);

  for (int cycle = 0; cycle < 5000; ++cycle) {
    const PicoEeFrankaDlsResult result = solver.solve(input);
    ASSERT_TRUE(result.accepted) << result.detail;
    ASSERT_TRUE(result.planner_state.q.allFinite());
    ASSERT_TRUE(result.planner_state.qdot.allFinite());
    ASSERT_TRUE(result.planner_state.qddot.allFinite());
    ASSERT_TRUE(result.planner_jerk.allFinite());
    EXPECT_TRUE((result.planner_state.q.array() >= -2.95 - 1.0e-8).all());
    EXPECT_TRUE((result.planner_state.q.array() <= 2.95 + 1.0e-8).all());
    EXPECT_TRUE((result.planner_state.qdot.cwiseAbs().array() <= 4.0 + 1.0e-8)
                    .all());
    EXPECT_TRUE(
        (result.planner_state.qddot.cwiseAbs().array() <= 60.0 + 1.0e-8)
            .all());
    EXPECT_TRUE((result.planner_jerk.cwiseAbs().array() <= 3000.0 + 1.0e-8)
                    .all());
  }

  input.target_valid = false;
  input.target_stale = true;
  const PicoEeFrankaDlsResult held = solver.solve(input);
  EXPECT_TRUE(held.accepted);
  EXPECT_TRUE(held.target_held);
  EXPECT_EQ(held.detail, "pico_ee_franka_dls_target_held");
}

TEST(PicoEeFrankaDls, PlannerPositionClampIsIndependentOfTargetMagnitude) {
  RealTimeConstrainedPlanner planner(15.0, 0.005, 4.0, 60.0, 3000.0,
                                     -2.95, 2.95);
  ASSERT_TRUE(planner.reset(0.0));
  planner.setTarget(100.0);
  for (int cycle = 0; cycle < 5000; ++cycle) {
    ASSERT_TRUE(planner.update(0.005));
    EXPECT_LE(planner.position(), 2.95);
    EXPECT_GE(planner.position(), -2.95);
    EXPECT_LE(std::abs(planner.velocity()), 4.0 + 1.0e-10);
    EXPECT_LE(std::abs(planner.acceleration()), 60.0 + 1.0e-10);
    EXPECT_LE(std::abs(planner.jerk()), 3000.0 + 1.0e-10);
  }
}

TEST(PicoEeFrankaDls, VelocityEnvelopeKeepsPublishedJerkBounded) {
  constexpr double kDt = 0.005;
  constexpr double kMaxJerk = 3000.0;
  RealTimeConstrainedPlanner planner(15.0, kDt, 4.0, 60.0, kMaxJerk,
                                     -2.95, 2.95);
  ASSERT_TRUE(planner.reset(0.0));
  planner.setTarget(2.5);

  double previous_velocity = planner.velocity();
  double previous_acceleration = planner.acceleration();
  for (int cycle = 0; cycle < 1800; ++cycle) {
    ASSERT_TRUE(planner.update(kDt));
    const double finite_difference_acceleration =
        (planner.velocity() - previous_velocity) / kDt;
    const double finite_difference_jerk =
        (planner.acceleration() - previous_acceleration) / kDt;
    EXPECT_NEAR(finite_difference_acceleration, planner.acceleration(),
                1.0e-8);
    EXPECT_LE(std::abs(finite_difference_jerk), kMaxJerk + 1.0e-8);
    EXPECT_LE(std::abs(planner.velocity()), 4.0 + 1.0e-10);
    previous_velocity = planner.velocity();
    previous_acceleration = planner.acceleration();
  }
}

TEST(PicoEeFrankaDls, DirectModePublishesDlsWithoutTargetOrVelocitySmoothing) {
  auto config = plannerConfig();
  config.planner_enabled = false;
  config.maximum_target_joint_step_rad = 0.001;
  PicoEeFrankaDlsIk7 solver(dlsConfig(), config, 0.05);
  Pose target;
  target.position << 0.10, -0.05, 0.02;
  target.rotation.setIdentity();
  auto input = inputFor(target);
  const auto result = solver.solve(input);
  ASSERT_TRUE(result.accepted) << result.detail;
  EXPECT_FALSE(result.planner_accepted);
  EXPECT_TRUE(result.planner_state.q.isApprox(result.dls.q));
  EXPECT_TRUE(result.planner_target.isApprox(result.goal));
  EXPECT_GT(result.qdot.cwiseAbs().maxCoeff(), 4.0);
  EXPECT_GT(result.goal[6], 0.0);
  EXPECT_TRUE((result.goal.array() >= -2.95).all());
  EXPECT_TRUE((result.goal.array() <= 2.95).all());

  input.seed = result.planner_state.q;
  input.seed_velocity = result.qdot;
  input.seed_acceleration = result.planner_state.qddot;
  input.target_stale = true;
  const auto held = solver.solve(input);
  ASSERT_TRUE(held.accepted);
  EXPECT_TRUE(held.target_held);
  EXPECT_TRUE(held.planner_state.q.isApprox(result.goal));
  EXPECT_TRUE(held.qdot.isZero());
}

TEST(PicoEeFrankaDls, DirectModeRejectsNonFiniteSeed) {
  auto config = plannerConfig();
  config.planner_enabled = false;
  PicoEeFrankaDlsIk7 solver(dlsConfig(), config, 0.05);
  Pose target;
  target.position.setZero();
  target.rotation.setIdentity();
  auto input = inputFor(target);
  input.seed[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(solver.solve(input).accepted);
}

TEST(PicoEeFrankaDls, RejectsInvalidTargetInsteadOfPublishingPreviousGoal) {
  auto config = plannerConfig();
  config.planner_enabled = false;
  PicoEeFrankaDlsIk7 solver(dlsConfig(), config, 0.05);
  Pose target;
  target.position << 0.10, -0.05, 0.02;
  target.rotation.setIdentity();
  auto input = inputFor(target);
  input.target.position[0] = std::numeric_limits<double>::quiet_NaN();

  const auto result = solver.solve(input);

  EXPECT_FALSE(result.accepted);
  EXPECT_TRUE(result.target_held);
  EXPECT_EQ(result.detail, "pico_ee_franka_dls_rejected_candidate");
  EXPECT_EQ(result.dls.status, PoseDlsStatus::kRejected);
}

TEST(PicoEeFrankaDls, PublishesBestCandidateEvenWithoutImprovement) {
  auto dls = dlsConfig();
  dls.max_iterations = 2;
  dls.nominal_posture_gain = 0.0;
  Pose target;
  target.position << 0.1, 0.0, 0.0;
  target.rotation.setIdentity();
  auto input = inputFor(target);
  // Deliberately adverse local direction: the final iterate is worse than
  // both the seed and the best candidate, making the output policy observable.
  input.evaluate = [](const Vec7& q) {
    auto sample = evaluate(q);
    sample.tcp_jacobian(0, 0) = -1.0;
    return sample;
  };
  FrankaPoseDlsInput baseline;
  baseline.target = target;
  baseline.seed = input.seed;
  baseline.limits = input.limits;
  baseline.evaluate = input.evaluate;
  FrankaIterativePoseDlsIk7 shared_solver(dls, 0.05);
  const auto best = shared_solver.solve(baseline);
  EXPECT_EQ(best.status, PoseDlsStatus::kNotConverged);
  EXPECT_TRUE(best.q.isZero());

  baseline.return_final_iterate = true;
  const auto final = shared_solver.solve(baseline);
  EXPECT_EQ(final.status, PoseDlsStatus::kNotConverged);
  EXPECT_EQ(final.iterations, 2);
  EXPECT_LT(final.q[0], -0.2);
  EXPECT_GT(final.position_error_m, final.initial_position_error_m);

  for (bool planner_enabled : {false, true}) {
    auto config = plannerConfig();
    config.planner_enabled = planner_enabled;
    PicoEeFrankaDlsIk7 solver(dls, config, 0.05);
    const auto result = solver.solve(input);
    ASSERT_TRUE(result.accepted) << result.detail;
    EXPECT_FALSE(result.target_held);
    EXPECT_EQ(result.dls.status, PoseDlsStatus::kNotConverged);
    EXPECT_TRUE(result.goal.isApprox(best.q));
    EXPECT_FALSE(result.goal.isApprox(final.q));
    EXPECT_EQ(result.planner_accepted, planner_enabled);
    if (!planner_enabled) {
      EXPECT_TRUE(result.planner_state.q.isApprox(best.q));
    }
  }
}

TEST(PicoEeFrankaDls, PublishesImprovedFinalIterateAtIterationLimit) {
  auto dls = dlsConfig();
  dls.max_iterations = 1;
  dls.nominal_posture_gain = 0.0;
  auto config = plannerConfig();
  config.planner_enabled = false;
  PicoEeFrankaDlsIk7 solver(dls, config, 0.05);
  Pose target;
  target.position << 1.0, 0.0, 0.0;
  target.rotation.setIdentity();
  const auto result = solver.solve(inputFor(target));
  ASSERT_TRUE(result.accepted);
  EXPECT_FALSE(result.target_held);
  EXPECT_EQ(result.dls.status, PoseDlsStatus::kImproved);
  EXPECT_EQ(result.dls.iterations, 1);
  EXPECT_NEAR(result.goal[0], 0.2, 1.0e-8);
  EXPECT_NEAR(result.dls.position_error_m, 0.8, 1.0e-8);
}

TEST(PicoEeFrankaDls, ArmPlaneBudgetIsSharedAcrossIterationsAndCycles) {
  auto config = plannerConfig();
  config.planner_enabled = false;
  config.max_arm_plane_rate_rad_s = 2.0;
  auto dls = dlsConfig();
  dls.nominal_posture_gain = 0.0;
  dls.max_iterations = 20;
  for (double dt : {0.0025, 0.005, 0.01}) {
    PicoEeFrankaDlsIk7 solver(dls, config, 0.05);
    Pose target;
    target.position << 0.5, 0.0, 0.0;
    auto input = inputFor(target);
    input.dt = dt;
    input.evaluate = [](const Vec7& q) {
      auto sample = evaluate(q);
      sample.shoulder_position.setZero();
      sample.wrist_position = Eigen::Vector3d(0, 0, 1);
      sample.elbow_position = Eigen::Vector3d(0.2 * std::cos(q[0]),
                                             0.2 * std::sin(q[0]), 0.5);
      return sample;
    };
    for (int cycle = 0; cycle < 10; ++cycle) {
      const auto result = solver.solve(input);
      ASSERT_TRUE(result.accepted) << result.detail;
      EXPECT_FALSE(result.target_held);
      EXPECT_GT(result.goal[0], input.seed[0]);
      EXPECT_LE(std::abs(result.goal[0] - input.seed[0]), 2.0 * dt + 1e-9);
      EXPECT_TRUE(result.goal.isApprox(result.dls.q));
      input.seed = result.goal;
      input.seed_velocity = result.qdot;
      input.seed_acceleration = result.planner_state.qddot;
    }
  }
}

TEST(PicoEeFrankaDls, ArmPlaneGuardRejectsDegenerateGeometry) {
  auto config = plannerConfig();
  config.planner_enabled = false;
  config.max_arm_plane_rate_rad_s = 2.0;
  PicoEeFrankaDlsIk7 solver(dlsConfig(), config, 0.05);
  Pose target;
  target.position << 0.1, 0, 0;
  EXPECT_FALSE(solver.solve(inputFor(target)).accepted);
}

TEST(PicoEeFrankaDls, GuidedDirectionUsesRedundancyToTrackAlongPlaneBoundary) {
  auto config = plannerConfig();
  config.planner_enabled = false;
  config.max_arm_plane_rate_rad_s = 2.0;
  auto dls = dlsConfig();
  dls.nominal_posture_gain = 0.0;
  Pose target;
  target.position << 0.1, 0, 0;
  auto input = inputFor(target);
  input.evaluate = [](const Vec7& q) {
    auto sample = evaluate(q);
    sample.wrist_position = Eigen::Vector3d(0, 0, 1);
    const double theta = q[0] + q[6];
    sample.elbow_position = Eigen::Vector3d(0.2 * std::cos(theta),
                                           0.2 * std::sin(theta), 0.5);
    return sample;
  };
  const auto baseline = PicoEeFrankaDlsIk7(dls, config, 0.05).solve(input);
  config.arm_plane_direction_guidance = true;
  const auto guided = PicoEeFrankaDlsIk7(dls, config, 0.05).solve(input);
  ASSERT_TRUE(baseline.accepted);
  ASSERT_TRUE(guided.accepted);
  EXPECT_LT(guided.dls.position_error_m, 1e-5);
  EXPECT_GT(baseline.dls.position_error_m, 0.08);
  EXPECT_LE(std::abs(guided.goal[0] + guided.goal[6]), 0.01 + 1e-9);
  EXPECT_FALSE(guided.target_held);
}

TEST(PicoEeFrankaDls, OptionalDirectVelocityCapPreservesDirection) {
  auto config = plannerConfig();
  config.planner_enabled = false;
  config.direct_velocity_limit_enabled = true;
  for (double dt : {0.0025, 0.005, 0.01}) {
    PicoEeFrankaDlsIk7 solver(dlsConfig(), config, 0.05);
    Pose target;
    target.position << 0.3, -0.1, 0.02;
    auto in = inputFor(target);
    in.dt = dt;
    in.limits.velocity[0] = 2.0;
    in.velocity_bounds.lower[0] = -0.5;
    in.velocity_bounds.upper[0] = 0.5;
    const auto result = solver.solve(in);
    ASSERT_TRUE(result.accepted);
    EXPECT_FALSE(result.planner_accepted);
    EXPECT_FALSE(result.target_held);
    EXPECT_LE(result.qdot.cwiseAbs().maxCoeff(), 4.0 + 1e-9);
    EXPECT_LE(std::abs(result.qdot[0]), 0.5 + 1e-9);
    const Vec7 delta = result.dls.q - in.seed;
    const Vec7 step = result.goal - in.seed;
    const double scale = step.dot(delta) / delta.squaredNorm();
    EXPECT_GT(scale, 0.0);
    EXPECT_LT(scale, 1.0);
    EXPECT_TRUE(step.isApprox(scale * delta, 1e-9));
    in.seed = result.goal;
    in.seed_velocity = result.qdot;
    in.seed_acceleration = result.planner_state.qddot;
    in.target_stale = true;
    const auto held = solver.solve(in);
    ASSERT_TRUE(held.accepted);
    EXPECT_TRUE(held.qdot.isZero());
  }
}

}  // namespace tianji_qp_ik
