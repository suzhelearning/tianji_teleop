#include "tianji_qp_ik/hierarchical_qp.hpp"
#include "tianji_qp_ik/hierarchical_qp_ik.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>

namespace tianji_qp_ik {
namespace {

HierarchicalQpConfig testConfig() {
  HierarchicalQpConfig config;
  config.lambda_reg = 0.1;
  config.posture_weight = 0.2;
  config.continuity_weight = 0.3;
  config.jerk_weight = 0.4;
  config.nominal_gain = 0.4;
  config.slack_weight_position = 100.0;
  config.slack_weight_orientation = 30.0;
  config.slack_position_scale = 0.5;
  config.slack_orientation_scale = 2.0;
  config.equality_tolerance = 1e-9;
  return config;
}

SafetyConfig safetyConfig() {
  SafetyConfig config;
  config.bound_tolerance = 1e-8;
  config.hessian_eigenvalue_tolerance = 1e-10;
  return config;
}

ArmIkInput deterministicInput() {
  ArmIkInput input;
  input.q_measured << -0.6, -0.4, -0.2, 0.0, 0.2, 0.4, 0.6;
  input.qdot_prev << 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1;
  input.qddot_prev << -0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.4;
  input.desired_twist << 0.1, -0.2, 0.3, -0.4, 0.5, -0.6;
  input.limits.lower_position = Vec7::Constant(-1.0);
  input.limits.upper_position = Vec7::Constant(1.0);
  input.bounds.lower << -1.0, -1.1, -1.2, -1.3, -1.4, -1.5, -1.6;
  input.bounds.upper << 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6;
  input.jacobian <<
      1.0, 0.1, 0.0, 0.2, 0.0, 0.3, 0.0,
      0.0, 1.0, 0.1, 0.0, 0.2, 0.0, 0.3,
      0.3, 0.0, 1.0, 0.1, 0.0, 0.2, 0.0,
      0.0, 0.3, 0.0, 1.0, 0.1, 0.0, 0.2,
      0.2, 0.0, 0.3, 0.0, 1.0, 0.1, 0.0,
      0.0, 0.2, 0.0, 0.3, 0.0, 1.0, 0.1;
  return input;
}

class KnownSolutionSolver final : public IHierarchicalQpSolver {
 public:
  bool initialize(const HierarchicalQpProblem&) override {
    initialized = true;
    return true;
  }

  HierarchicalQpSolution solve(const HierarchicalQpProblem& problem) override {
    HierarchicalQpSolution solution;
    solution.status = SolverStatus::kSolved;
    solution.x.head<7>().setZero();
    solution.x[0] = problem.upper[0];
    solution.x[1] = problem.lower[1];
    solution.x.segment<6>(kSlackStartIndex) =
        problem.equality - problem.A.leftCols<7>() * solution.x.head<7>();
    solution.iterations = 7;
    solution.solve_time_us = 12.5;
    solution.detail = "known_solution";
    return solution;
  }

  void reset() override { initialized = false; }

  bool initialized{false};
};

class ZeroPrimarySolver final : public IHierarchicalQpSolver {
 public:
  bool initialize(const HierarchicalQpProblem&) override { return true; }

  HierarchicalQpSolution solve(const HierarchicalQpProblem& problem) override {
    HierarchicalQpSolution solution;
    solution.status = SolverStatus::kSolved;
    solution.x.head<kArmDof>().setZero();
    solution.x.segment<6>(kSlackStartIndex) = problem.equality;
    solution.detail = "zero_primary";
    return solution;
  }

  void reset() override {}
};

class ScaledZeroPrimarySolver final : public IHierarchicalQpSolver {
 public:
  bool initialize(const HierarchicalQpProblem&) override { return true; }

  HierarchicalQpSolution solve(const HierarchicalQpProblem& problem) override {
    HierarchicalQpSolution solution;
    solution.status = SolverStatus::kSolved;
    solution.x.setZero();
    solution.x[kBetaPositionIndex] = 0.8;
    solution.x[kBetaOrientationIndex] = 0.9;
    solution.x.segment<3>(kSlackStartIndex) =
        0.8 * (-problem.A.block<3, 1>(0, kBetaPositionIndex));
    solution.x.segment<3>(kSlackStartIndex + 3) =
        0.9 * (-problem.A.block<3, 1>(3, kBetaOrientationIndex));
    solution.detail = "scaled_zero_primary";
    return solution;
  }

  void reset() override {}
};

class RetrySequenceSolver final : public IHierarchicalQpSolver {
 public:
  explicit RetrySequenceSolver(bool retry_succeeds)
      : retry_succeeds_(retry_succeeds) {}

  bool initialize(const HierarchicalQpProblem&) override {
    ++initialize_count;
    return true;
  }

  HierarchicalQpSolution solve(const HierarchicalQpProblem& problem) override {
    ++solve_count;
    HierarchicalQpSolution solution;
    solution.iterations = 7;
    solution.solve_time_us = 20.0;
    if (solve_count == 1 || !retry_succeeds_) {
      solution.status = SolverStatus::kMaxIterations;
      solution.detail = "forced_max_iterations";
      return solution;
    }
    solution.status = SolverStatus::kSolved;
    solution.x.head<kArmDof>().setZero();
    solution.x.segment<6>(kSlackStartIndex) = problem.equality;
    solution.detail = "cold_retry_solution";
    return solution;
  }

  void reset() override { ++reset_count; }

  int initialize_count{0};
  int solve_count{0};
  int reset_count{0};

 private:
  bool retry_succeeds_{false};
};

class InitializationRetrySolver final : public IHierarchicalQpSolver {
 public:
  bool initialize(const HierarchicalQpProblem&) override {
    ++initialize_count;
    return initialize_count > 1;
  }
  HierarchicalQpSolution solve(const HierarchicalQpProblem& problem) override {
    ++solve_count;
    HierarchicalQpSolution solution;
    solution.status = SolverStatus::kSolved;
    solution.x.head<kArmDof>().setZero();
    solution.x.segment<6>(kSlackStartIndex) = problem.equality;
    solution.iterations = 3;
    solution.solve_time_us = 5.0;
    return solution;
  }
  void reset() override { ++reset_count; }
  int initialize_count{0};
  int solve_count{0};
  int reset_count{0};
};

TEST(HierarchicalQpBuilder, ExpandsSlackPostureAndContinuityExactly) {
  const HierarchicalQpConfig config = testConfig();
  const ArmIkInput input = deterministicInput();
  const HierarchicalQpProblem problem = HierarchicalQpBuilder(config).build(input);

  EXPECT_TRUE((problem.H.topLeftCorner<7, 7>().isApprox(
      1.0 * Mat77::Identity(), 1e-14)));
  EXPECT_TRUE((problem.H.topRightCorner<7, 6>().isZero(0.0)));
  EXPECT_TRUE((problem.H.bottomLeftCorner<6, 7>().isZero(0.0)));
  const Vec6 slack_diagonal =
      (Vec6() << 400.0, 400.0, 400.0, 7.5, 7.5, 7.5).finished();
  EXPECT_TRUE((problem.H.block<6, 6>(kSlackStartIndex, kSlackStartIndex)
                   .diagonal().isApprox(slack_diagonal)));
  EXPECT_DOUBLE_EQ(problem.H(kBetaPositionIndex, kBetaPositionIndex), 1.0);
  EXPECT_DOUBLE_EQ(problem.H(kBetaOrientationIndex, kBetaOrientationIndex),
                   1.0);

  const Vec7 qdot_nominal = -config.nominal_gain * input.q_measured;
  const Vec7 expected_gradient =
      -config.posture_weight * qdot_nominal -
      config.continuity_weight * input.qdot_prev -
      config.jerk_weight *
          (input.qdot_prev + input.qddot_prev * input.dt);
  EXPECT_TRUE((problem.g.head<7>().isApprox(expected_gradient, 1e-14)));
  EXPECT_TRUE((problem.g.segment<6>(kSlackStartIndex).isZero(0.0)));
  EXPECT_DOUBLE_EQ(problem.g[kBetaPositionIndex], 0.0);
  EXPECT_DOUBLE_EQ(problem.g[kBetaOrientationIndex], 0.0);
  EXPECT_TRUE((problem.A.leftCols<7>().isApprox(input.jacobian)));
  EXPECT_TRUE((problem.A.block<6, 6>(0, kSlackStartIndex).isApprox(
      Eigen::Matrix<double, 6, 6>::Identity())));
  EXPECT_TRUE(problem.equality.isApprox(input.desired_twist));
  EXPECT_TRUE((problem.lower.head<7>().isApprox(input.bounds.lower)));
  EXPECT_TRUE((problem.upper.head<7>().isApprox(input.bounds.upper)));
  for (int index = kSlackStartIndex;
       index < kSlackStartIndex + kSlackVariables; ++index) {
    EXPECT_TRUE(std::isinf(problem.lower[index]));
    EXPECT_LT(problem.lower[index], 0.0);
    EXPECT_TRUE(std::isinf(problem.upper[index]));
    EXPECT_GT(problem.upper[index], 0.0);
  }
  EXPECT_DOUBLE_EQ(problem.lower[kBetaPositionIndex], 0.0);
  EXPECT_DOUBLE_EQ(problem.upper[kBetaPositionIndex], 0.0);
  EXPECT_DOUBLE_EQ(problem.lower[kBetaOrientationIndex], 0.0);
  EXPECT_DOUBLE_EQ(problem.upper[kBetaOrientationIndex], 0.0);
}

TEST(HierarchicalQpBuilder, ScalesOnlyWristPostureRegularization) {
  HierarchicalQpConfig baseline_config = testConfig();
  HierarchicalQpConfig wrist_config = baseline_config;
  wrist_config.wrist_posture_weight_scale = 10.0;
  const ArmIkInput input = deterministicInput();

  const HierarchicalQpProblem baseline =
      HierarchicalQpBuilder(baseline_config).build(input);
  const HierarchicalQpProblem wrist =
      HierarchicalQpBuilder(wrist_config).build(input);
  const Vec7 q_nominal =
      0.5 * (input.limits.lower_position + input.limits.upper_position);
  const Vec7 qdot_nominal =
      -baseline_config.nominal_gain * (input.q_measured - q_nominal);

  for (int joint = 0; joint < 4; ++joint) {
    EXPECT_DOUBLE_EQ(wrist.H(joint, joint), baseline.H(joint, joint));
    EXPECT_DOUBLE_EQ(wrist.g[joint], baseline.g[joint]);
  }
  const double extra_weight = baseline_config.posture_weight * 9.0;
  for (int joint = 4; joint < kArmDof; ++joint) {
    EXPECT_NEAR(wrist.H(joint, joint) - baseline.H(joint, joint),
                extra_weight, 1.0e-14);
    EXPECT_NEAR(wrist.g[joint] - baseline.g[joint],
                -extra_weight * qdot_nominal[joint], 1.0e-14);
  }
}

TEST(HierarchicalQpBuilder,
     AddsIndependentPositionAndOrientationTaskScaling) {
  HierarchicalQpConfig config = testConfig();
  config.task_scaling_enabled = true;
  config.task_scaling_min_position = 0.20;
  config.task_scaling_min_orientation = 0.30;
  config.task_scaling_weight_position = 120.0;
  config.task_scaling_weight_orientation = 80.0;
  const ArmIkInput input = deterministicInput();

  const HierarchicalQpProblem problem =
      HierarchicalQpBuilder(config).build(input);

  EXPECT_TRUE((problem.A.block<3, 1>(0, kBetaPositionIndex)
                   .isApprox(-input.desired_twist.head<3>())));
  EXPECT_TRUE((problem.A.block<3, 1>(3, kBetaOrientationIndex)
                   .isApprox(-input.desired_twist.tail<3>())));
  EXPECT_TRUE(problem.equality.isZero(0.0));
  EXPECT_DOUBLE_EQ(problem.H(kBetaPositionIndex, kBetaPositionIndex),
                   config.task_scaling_weight_position);
  EXPECT_DOUBLE_EQ(problem.H(kBetaOrientationIndex, kBetaOrientationIndex),
                   config.task_scaling_weight_orientation);
  EXPECT_DOUBLE_EQ(problem.g[kBetaPositionIndex],
                   -config.task_scaling_weight_position);
  EXPECT_DOUBLE_EQ(problem.g[kBetaOrientationIndex],
                   -config.task_scaling_weight_orientation);
  EXPECT_DOUBLE_EQ(problem.lower[kBetaPositionIndex],
                   config.task_scaling_min_position);
  EXPECT_DOUBLE_EQ(problem.upper[kBetaPositionIndex], 1.0);
  EXPECT_DOUBLE_EQ(problem.lower[kBetaOrientationIndex],
                   config.task_scaling_min_orientation);
  EXPECT_DOUBLE_EQ(problem.upper[kBetaOrientationIndex], 1.0);
}

TEST(HierarchicalQpBuilder, SlackMakesTrackingEqualityFeasibleForAnyJacobianRank) {
  for (int rank_case = 0; rank_case < 3; ++rank_case) {
    ArmIkInput input = deterministicInput();
    if (rank_case == 1) {
      input.jacobian.row(5) = input.jacobian.row(4);
    } else if (rank_case == 2) {
      input.jacobian.setZero();
    }
    const HierarchicalQpProblem problem = HierarchicalQpBuilder(testConfig()).build(input);
    VecVelocityQp candidate = VecVelocityQp::Zero();
    candidate.segment<6>(kSlackStartIndex) = input.desired_twist;
    EXPECT_TRUE((problem.A * candidate).isApprox(problem.equality, 1e-14));
  }
}

TEST(HierarchicalQpBuilder, ArmAngleTaskDoesNotModifyPrimaryProblem) {
  ArmIkInput data = deterministicInput();
  const HierarchicalQpProblem baseline =
      HierarchicalQpBuilder(testConfig()).build(data);
  data.arm_angle_task.active = true;
  data.arm_angle_task.jacobian << 1.0, -2.0, 0.5, 0.0, 0.2, -0.3, 0.7;
  data.arm_angle_task.target = 1.25;
  data.arm_angle_task.activation = 1.0;
  const HierarchicalQpProblem with_secondary =
      HierarchicalQpBuilder(testConfig()).build(data);

  EXPECT_TRUE(with_secondary.H.isApprox(baseline.H, 0.0));
  EXPECT_TRUE(with_secondary.g.isApprox(baseline.g, 0.0));
  EXPECT_TRUE(with_secondary.A.isApprox(baseline.A, 0.0));
  EXPECT_TRUE(with_secondary.equality.isApprox(baseline.equality, 0.0));
  EXPECT_TRUE((with_secondary.lower.array() == baseline.lower.array()).all());
  EXPECT_TRUE((with_secondary.upper.array() == baseline.upper.array()).all());
}

TEST(HierarchicalQpBuilder, AddsConfiguredArmAngleTaskWithoutNewVariables) {
  HierarchicalQpConfig config = testConfig();
  config.arm_angle_weight = 8.0;
  ArmIkInput input = deterministicInput();
  input.jacobian.setZero();
  input.jacobian.leftCols<6>() =
      Eigen::Matrix<double, 6, 6>::Identity();
  input.arm_angle_task.active = true;
  input.arm_angle_task.jacobian << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.7;
  input.arm_angle_task.target = 1.25;
  input.arm_angle_task.activation = 0.5;
  input.arm_angle_task.weight_scale = 3.0;
  const HierarchicalQpProblem baseline =
      HierarchicalQpBuilder(testConfig()).build(input);
  const HierarchicalQpProblem problem =
      HierarchicalQpBuilder(config).build(input);
  const double weight = config.arm_angle_weight *
                        input.arm_angle_task.activation *
                        input.arm_angle_task.weight_scale;
  Vec7 nullspace_jacobian = Vec7::Zero();
  nullspace_jacobian[6] = 0.7;
  const double nullspace_target =
      input.arm_angle_task.target - input.desired_twist[0];

  EXPECT_EQ(problem.H.rows(), kHierarchicalVariables);
  EXPECT_TRUE((problem.H.topLeftCorner<kArmDof, kArmDof>().isApprox(
      baseline.H.topLeftCorner<kArmDof, kArmDof>() +
          weight * nullspace_jacobian * nullspace_jacobian.transpose(),
      1.0e-14)));
  EXPECT_TRUE(problem.g.head<kArmDof>().isApprox(
      baseline.g.head<kArmDof>() -
          weight * nullspace_target * nullspace_jacobian,
      1.0e-14));
  EXPECT_TRUE(problem.A.isApprox(baseline.A, 0.0));
  EXPECT_TRUE(problem.equality.isApprox(baseline.equality, 0.0));
}

TEST(HierarchicalQpBuilder,
     TaskScalingUsesDirectArmAngleObjectiveWhenConfigured) {
  HierarchicalQpConfig config = testConfig();
  config.task_scaling_enabled = true;
  config.arm_angle_weight = 8.0;
  ArmIkInput input = deterministicInput();
  input.arm_angle_task.active = true;
  input.arm_angle_task.nullspace_only = false;
  input.arm_angle_task.jacobian << 1.0, 0.0, -0.5, 0.0, 0.2, 0.0, 0.7;
  input.arm_angle_task.target = 1.25;
  input.arm_angle_task.activation = 0.5;
  input.arm_angle_task.weight_scale = 3.0;
  HierarchicalQpConfig baseline_config = config;
  baseline_config.arm_angle_weight = 0.0;
  const HierarchicalQpProblem baseline =
      HierarchicalQpBuilder(baseline_config).build(input);

  const HierarchicalQpProblem problem =
      HierarchicalQpBuilder(config).build(input);

  const double weight = config.arm_angle_weight *
                        input.arm_angle_task.activation *
                        input.arm_angle_task.weight_scale;
  EXPECT_TRUE((problem.H.topLeftCorner<kArmDof, kArmDof>().isApprox(
      baseline.H.topLeftCorner<kArmDof, kArmDof>() +
          weight * input.arm_angle_task.jacobian *
              input.arm_angle_task.jacobian.transpose(),
      1.0e-14)));
  EXPECT_TRUE(problem.g.head<kArmDof>().isApprox(
      baseline.g.head<kArmDof>() -
          weight * input.arm_angle_task.target *
              input.arm_angle_task.jacobian,
      1.0e-14));
}

TEST(HierarchicalQpBuilder, CarriesOptionalJointLinearConstraint) {
  ArmIkInput data = deterministicInput();
  data.linear_constraint.active = true;
  data.linear_constraint.jacobian << 1.0, -2.0, 0.5, 0.0, 0.2, -0.3, 0.7;
  data.linear_constraint.lower = -0.25;
  data.linear_constraint.upper = 0.75;
  data.branch_lock_constraint.active = true;
  data.branch_lock_constraint.jacobian <<
      -0.2, 0.1, 0.3, -0.4, 0.5, -0.6, 0.7;
  data.branch_lock_constraint.lower = -0.05;
  data.branch_lock_constraint.upper =
      std::numeric_limits<double>::infinity();
  const HierarchicalQpProblem problem =
      HierarchicalQpBuilder(testConfig()).build(data);

  EXPECT_TRUE(problem.linear_constraint.active);
  EXPECT_TRUE(problem.linear_constraint.jacobian.isApprox(
      data.linear_constraint.jacobian));
  EXPECT_DOUBLE_EQ(problem.linear_constraint.lower, -0.25);
  EXPECT_DOUBLE_EQ(problem.linear_constraint.upper, 0.75);
  EXPECT_TRUE(problem.branch_lock_constraint.active);
  EXPECT_TRUE(problem.branch_lock_constraint.jacobian.isApprox(
      data.branch_lock_constraint.jacobian));
  EXPECT_DOUBLE_EQ(problem.branch_lock_constraint.lower, -0.05);
  EXPECT_TRUE(std::isinf(problem.branch_lock_constraint.upper));
}

TEST(HierarchicalQpValidation, AcceptsFiniteProblemWithUnboundedSlack) {
  const HierarchicalQpProblem problem =
      HierarchicalQpBuilder(testConfig()).build(deterministicInput());
  const SafetyDecision decision = validateHierarchicalProblem(problem, safetyConfig());
  EXPECT_TRUE(decision.accepted);
  EXPECT_EQ(decision.reason, HoldReason::kNone);
}

TEST(HierarchicalQpValidation, RejectsNonFinitePhysicalDataAndInvalidBounds) {
  HierarchicalQpProblem problem =
      HierarchicalQpBuilder(testConfig()).build(deterministicInput());
  problem.A(2, 3) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(validateHierarchicalProblem(problem, safetyConfig()).reason,
            HoldReason::kNonFiniteProblem);

  problem = HierarchicalQpBuilder(testConfig()).build(deterministicInput());
  problem.lower[4] = problem.upper[4] + 0.1;
  const SafetyDecision bounds = validateHierarchicalProblem(problem, safetyConfig());
  EXPECT_EQ(bounds.reason, HoldReason::kInfeasibleBounds);
  EXPECT_EQ(bounds.joint_index, 4);

  problem = HierarchicalQpBuilder(testConfig()).build(deterministicInput());
  problem.lower[8] = -1.0;
  EXPECT_EQ(validateHierarchicalProblem(problem, safetyConfig()).reason,
            HoldReason::kNonFiniteProblem);

  problem = HierarchicalQpBuilder(testConfig()).build(deterministicInput());
  problem.lower[kBetaPositionIndex] = 0.5;
  problem.upper[kBetaPositionIndex] = 0.4;
  EXPECT_EQ(validateHierarchicalProblem(problem, safetyConfig()).reason,
            HoldReason::kNonFiniteProblem);

  problem = HierarchicalQpBuilder(testConfig()).build(deterministicInput());
  problem.branch_lock_constraint.active = true;
  problem.branch_lock_constraint.jacobian[2] =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(validateHierarchicalProblem(problem, safetyConfig()).reason,
            HoldReason::kNonFiniteProblem);
}

TEST(HierarchicalQpValidation, RejectsInvalidHessian) {
  HierarchicalQpProblem problem =
      HierarchicalQpBuilder(testConfig()).build(deterministicInput());
  problem.H(0, 0) = -1.0;
  EXPECT_EQ(validateHierarchicalProblem(problem, safetyConfig()).reason,
            HoldReason::kInvalidHessian);
}

TEST(HierarchicalQpValidation, ValidatesStatusFiniteBoundsAndEquality) {
  const HierarchicalQpConfig config = testConfig();
  const HierarchicalQpProblem problem =
      HierarchicalQpBuilder(config).build(deterministicInput());
  HierarchicalQpSolution solution;
  solution.status = SolverStatus::kSolved;
  solution.x.setZero();
  solution.x.segment<6>(kSlackStartIndex) = problem.equality;
  EXPECT_TRUE(validateHierarchicalSolution(problem, solution, config, safetyConfig()).accepted);

  solution.status = SolverStatus::kMaxIterations;
  EXPECT_EQ(validateHierarchicalSolution(problem, solution, config, safetyConfig()).reason,
            HoldReason::kSolverFailure);

  solution.status = SolverStatus::kSolved;
  solution.x[7] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(validateHierarchicalSolution(problem, solution, config, safetyConfig()).reason,
            HoldReason::kNonFiniteSolution);

  solution.x.setZero();
  solution.x.segment<6>(kSlackStartIndex) = problem.equality;
  solution.x[3] = problem.upper[3] + 2.0 * safetyConfig().bound_tolerance;
  EXPECT_EQ(validateHierarchicalSolution(problem, solution, config, safetyConfig()).reason,
            HoldReason::kBoundViolation);

  solution.x.setZero();
  EXPECT_EQ(validateHierarchicalSolution(problem, solution, config, safetyConfig()).reason,
            HoldReason::kEqualityViolation);
}

TEST(HierarchicalQpIk, MapsSolutionAndCountsActiveBoundSources) {
  ArmIkInput input = deterministicInput();
  input.bounds.upper_source[0] = BoundSource::kPosition;
  input.bounds.lower_source[1] = BoundSource::kVelocity;
  input.limits.velocity = Vec7::Constant(2.0);
  auto fake = std::make_unique<KnownSolutionSolver>();
  HierarchicalQpIk7 backend(testConfig(), safetyConfig(), std::move(fake));

  const ArmIkResult result = backend.solve(input);

  ASSERT_EQ(result.status, SolverStatus::kSolved) << result.detail;
  EXPECT_DOUBLE_EQ(result.qdot[0], input.bounds.upper[0]);
  EXPECT_DOUBLE_EQ(result.qdot[1], input.bounds.lower[1]);
  EXPECT_TRUE(result.slack.isApprox(
      input.desired_twist - input.jacobian * result.qdot, 1e-12));
  EXPECT_LT(result.equality_residual, testConfig().equality_tolerance);
  EXPECT_EQ(result.active_position_bound_count, 1);
  EXPECT_EQ(result.active_velocity_bound_count, 1);
  EXPECT_EQ(result.iterations, 7);
  EXPECT_DOUBLE_EQ(result.solve_time_us, 12.5);
}

TEST(HierarchicalQpIk, ReportsTaskScalingAndScaledSlack) {
  ArmIkInput input = deterministicInput();
  input.bounds.lower = Vec7::Constant(-1.0);
  input.bounds.upper = Vec7::Constant(1.0);
  HierarchicalQpConfig config = testConfig();
  config.task_scaling_enabled = true;
  config.task_scaling_min_position = 0.75;
  config.task_scaling_min_orientation = 0.75;
  HierarchicalQpIk7 backend(
      config, safetyConfig(), std::make_unique<ScaledZeroPrimarySolver>());

  const ArmIkResult result = backend.solve(input);

  ASSERT_EQ(result.status, SolverStatus::kSolved) << result.detail;
  EXPECT_DOUBLE_EQ(result.task_scale_position, 0.8);
  EXPECT_DOUBLE_EQ(result.task_scale_orientation, 0.9);
  EXPECT_TRUE(result.slack.head<3>().isApprox(
      0.8 * input.desired_twist.head<3>(), 1.0e-14));
  EXPECT_TRUE(result.slack.tail<3>().isApprox(
      0.9 * input.desired_twist.tail<3>(), 1.0e-14));
  EXPECT_LT(result.equality_residual, config.equality_tolerance);
}

TEST(HierarchicalQpIk, RefinesArmAngleOnlyInCartesianNullspace) {
  ArmIkInput input;
  input.jacobian.leftCols<6>() =
      Eigen::Matrix<double, 6, 6>::Identity();
  input.desired_twist << 0.1, -0.2, 0.3, -0.4, 0.5, -0.6;
  input.bounds.lower = Vec7::Constant(-1.0);
  input.bounds.upper = Vec7::Constant(1.0);
  input.limits.velocity = Vec7::Constant(2.0);
  input.arm_angle_task.active = true;
  input.arm_angle_task.jacobian[6] = 1.0;
  input.arm_angle_task.target = 0.4;
  input.arm_angle_task.activation = 1.0;
  HierarchicalQpIk7 backend(testConfig(), safetyConfig(),
                            std::make_unique<ZeroPrimarySolver>());

  const ArmIkResult result = backend.solve(input);

  ASSERT_EQ(result.status, SolverStatus::kSolved) << result.detail;
  EXPECT_NEAR(result.qdot[6], 0.4, 1e-12);
  EXPECT_TRUE((input.jacobian * result.qdot + result.slack)
                  .isApprox(input.desired_twist, 1e-12));
  EXPECT_TRUE((result.qdot.array() >= input.bounds.lower.array() - 1e-12)
                  .all());
  EXPECT_TRUE((result.qdot.array() <= input.bounds.upper.array() + 1e-12)
                  .all());
}

TEST(HierarchicalQpIk, WeightedArmAngleModeDoesNotApplyASecondExactStep) {
  ArmIkInput input = deterministicInput();
  input.jacobian.setZero();
  input.jacobian.leftCols<6>() =
      Eigen::Matrix<double, 6, 6>::Identity();
  input.desired_twist.setZero();
  input.bounds.lower.setConstant(-1.0);
  input.bounds.upper.setConstant(1.0);
  input.arm_angle_task.active = true;
  input.arm_angle_task.jacobian[6] = 1.0;
  input.arm_angle_task.target = 0.8;
  input.arm_angle_task.activation = 1.0;
  HierarchicalQpConfig config = testConfig();
  config.arm_angle_weight = 10.0;
  HierarchicalQpIk7 ik(config, safetyConfig(),
                       std::make_unique<ZeroPrimarySolver>());

  const ArmIkResult result = ik.solve(input);

  ASSERT_EQ(result.status, SolverStatus::kSolved);
  EXPECT_TRUE(result.qdot.isZero(1.0e-12));
  EXPECT_TRUE(result.slack.isZero(1.0e-12));
}

TEST(HierarchicalQpIk,
     ArmAngleNullspaceRefinementPreservesOutwardHardConstraint) {
  ArmIkInput input;
  input.jacobian.leftCols<6>() =
      Eigen::Matrix<double, 6, 6>::Identity();
  input.bounds.lower = Vec7::Constant(-1.0);
  input.bounds.upper = Vec7::Constant(1.0);
  input.limits.velocity = Vec7::Constant(2.0);
  input.arm_angle_task.active = true;
  input.arm_angle_task.jacobian[6] = 1.0;
  input.arm_angle_task.target = -0.4;
  input.arm_angle_task.activation = 1.0;
  input.linear_constraint.active = true;
  input.linear_constraint.jacobian[6] = 1.0;
  input.linear_constraint.lower = -0.02;
  HierarchicalQpIk7 backend(testConfig(), safetyConfig(),
                            std::make_unique<ZeroPrimarySolver>());

  const ArmIkResult result = backend.solve(input);

  ASSERT_EQ(result.status, SolverStatus::kSolved) << result.detail;
  EXPECT_GE(input.linear_constraint.jacobian.dot(result.qdot),
            input.linear_constraint.lower - 1e-12);
  EXPECT_NEAR(result.qdot[6], input.linear_constraint.lower, 1e-12);
  EXPECT_TRUE((input.jacobian * result.qdot).isZero(1e-12));
}

TEST(HierarchicalQpIk,
     ArmAngleNullspaceRefinementPreservesBothSafetyConstraints) {
  ArmIkInput input;
  input.jacobian.leftCols<6>() =
      Eigen::Matrix<double, 6, 6>::Identity();
  input.bounds.lower = Vec7::Constant(-1.0);
  input.bounds.upper = Vec7::Constant(1.0);
  input.limits.velocity = Vec7::Constant(2.0);
  input.arm_angle_task.active = true;
  input.arm_angle_task.jacobian[6] = 1.0;
  input.arm_angle_task.target = 0.4;
  input.arm_angle_task.activation = 1.0;
  input.linear_constraint.active = true;
  input.linear_constraint.jacobian[6] = 1.0;
  input.linear_constraint.lower = -0.2;
  input.branch_lock_constraint.active = true;
  input.branch_lock_constraint.jacobian[6] = 1.0;
  input.branch_lock_constraint.upper = 0.05;
  HierarchicalQpIk7 backend(testConfig(), safetyConfig(),
                            std::make_unique<ZeroPrimarySolver>());

  const ArmIkResult result = backend.solve(input);

  ASSERT_EQ(result.status, SolverStatus::kSolved) << result.detail;
  EXPECT_GE(input.linear_constraint.jacobian.dot(result.qdot),
            input.linear_constraint.lower - 1.0e-12);
  EXPECT_LE(input.branch_lock_constraint.jacobian.dot(result.qdot),
            input.branch_lock_constraint.upper + 1.0e-12);
  EXPECT_NEAR(result.qdot[6], input.branch_lock_constraint.upper, 1.0e-12);
  EXPECT_TRUE((input.jacobian * result.qdot).isZero(1.0e-12));
}

TEST(HierarchicalQpIk,
     SmoothPostureReferenceOwnsRedundancyWithoutChangingCartesianTask) {
  ArmIkInput input;
  input.jacobian.leftCols<6>() =
      Eigen::Matrix<double, 6, 6>::Identity();
  input.desired_twist << 0.1, -0.2, 0.3, -0.4, 0.5, -0.6;
  input.bounds.lower = Vec7::Constant(-1.0);
  input.bounds.upper = Vec7::Constant(1.0);
  input.limits.velocity = Vec7::Constant(2.0);
  input.posture_task.active = true;
  input.posture_task.target[6] = 0.35;
  HierarchicalQpIk7 backend(testConfig(), safetyConfig(),
                            std::make_unique<ZeroPrimarySolver>());

  const ArmIkResult result = backend.solve(input);

  ASSERT_EQ(result.status, SolverStatus::kSolved) << result.detail;
  EXPECT_NEAR(result.qdot[6], 0.35, 1.0e-12);
  EXPECT_TRUE((input.jacobian * result.qdot + result.slack)
                  .isApprox(input.desired_twist, 1.0e-12));
}

TEST(HierarchicalQpBuilder, SparkPostureAddsExactSoftVelocityObjective) {
  HierarchicalQpConfig config = testConfig();
  ArmIkInput baseline_input = deterministicInput();
  const HierarchicalQpProblem baseline =
      HierarchicalQpBuilder(config).build(baseline_input);

  ArmIkInput spark_input = baseline_input;
  spark_input.posture_task.active = true;
  spark_input.posture_task.source =
      JointVelocityPostureSource::kSparkSoftQp;
  spark_input.posture_task.activation = 0.4;
  spark_input.posture_task.weight = 0.25;
  spark_input.posture_task.target << 0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7;
  const HierarchicalQpProblem spark =
      HierarchicalQpBuilder(config).build(spark_input);

  const double effective_weight = 0.4 * 0.25;
  EXPECT_TRUE((spark.H.topLeftCorner<kArmDof, kArmDof>() -
               baseline.H.topLeftCorner<kArmDof, kArmDof>())
                  .isApprox(effective_weight * Mat77::Identity(), 1.0e-14));
  EXPECT_TRUE((spark.g.head<kArmDof>() - baseline.g.head<kArmDof>())
                  .isApprox(-effective_weight * spark_input.posture_task.target,
                            1.0e-14));
  EXPECT_TRUE(spark.A.isApprox(baseline.A, 0.0));
  EXPECT_TRUE(spark.equality.isApprox(baseline.equality, 0.0));
  for (int index = 0; index < kHierarchicalVariables; ++index) {
    EXPECT_EQ(spark.lower[index], baseline.lower[index]);
    EXPECT_EQ(spark.upper[index], baseline.upper[index]);
  }
}

TEST(HierarchicalQpBuilder,
     SparkJointReferenceAddsDedicatedVelocityContinuity) {
  HierarchicalQpConfig config = testConfig();
  ArmIkInput baseline_input = deterministicInput();
  baseline_input.qdot_prev = Vec7::LinSpaced(-0.3, 0.3);
  baseline_input.qddot_prev = Vec7::LinSpaced(0.7, -0.5);
  baseline_input.dt = 0.005;
  const HierarchicalQpProblem baseline =
      HierarchicalQpBuilder(config).build(baseline_input);

  ArmIkInput input = baseline_input;
  input.posture_task.active = true;
  input.posture_task.source =
      JointVelocityPostureSource::kSparkJointReference;
  input.posture_task.target = Vec7::LinSpaced(0.1, 0.7);
  input.posture_task.weight = 0.0;
  input.posture_task.smoothness_weight = 2.5;
  const HierarchicalQpProblem problem =
      HierarchicalQpBuilder(config).build(input);

  EXPECT_TRUE((problem.H.topLeftCorner<kArmDof, kArmDof>() -
               baseline.H.topLeftCorner<kArmDof, kArmDof>())
                  .isApprox(2.5 * Mat77::Identity(), 1.0e-14));
  EXPECT_TRUE((problem.g.head<kArmDof>() - baseline.g.head<kArmDof>())
                  .isApprox(-2.5 * input.qdot_prev, 1.0e-14));
}

TEST(HierarchicalQpBuilder,
     SparkFeedforwardJointReferenceAddsSoftTargetAndContinuity) {
  HierarchicalQpConfig config = testConfig();
  ArmIkInput baseline_input = deterministicInput();
  baseline_input.qdot_prev = Vec7::LinSpaced(-0.3, 0.3);
  const HierarchicalQpProblem baseline =
      HierarchicalQpBuilder(config).build(baseline_input);

  ArmIkInput input = baseline_input;
  input.posture_task.active = true;
  input.posture_task.source =
      JointVelocityPostureSource::kSparkFeedforwardJointReference;
  input.posture_task.target = Vec7::LinSpaced(0.1, 0.7);
  input.posture_task.weight = 1.5;
  input.posture_task.smoothness_weight = 2.5;
  const HierarchicalQpProblem problem =
      HierarchicalQpBuilder(config).build(input);

  EXPECT_TRUE((problem.H.topLeftCorner<kArmDof, kArmDof>() -
               baseline.H.topLeftCorner<kArmDof, kArmDof>())
                  .isApprox(4.0 * Mat77::Identity(), 1.0e-14));
  EXPECT_TRUE((problem.g.head<kArmDof>() - baseline.g.head<kArmDof>())
                  .isApprox(-1.5 * input.posture_task.target -
                                2.5 * input.qdot_prev,
                            1.0e-14));
}

TEST(HierarchicalQpBuilder,
     SparkFeedforwardJointReferenceAddsJerkContinuity) {
  HierarchicalQpConfig config = testConfig();
  ArmIkInput baseline_input = deterministicInput();
  baseline_input.qdot_prev = Vec7::LinSpaced(-0.3, 0.3);
  baseline_input.qddot_prev = Vec7::LinSpaced(0.7, -0.5);
  baseline_input.dt = 0.005;
  const HierarchicalQpProblem baseline =
      HierarchicalQpBuilder(config).build(baseline_input);

  ArmIkInput input = baseline_input;
  input.posture_task.active = true;
  input.posture_task.source =
      JointVelocityPostureSource::kSparkFeedforwardJointReference;
  input.posture_task.weight = 0.0;
  input.posture_task.smoothness_weight = 0.0;
  input.posture_task.jerk_smoothness_weight = 2.5;
  const HierarchicalQpProblem problem =
      HierarchicalQpBuilder(config).build(input);

  const Vec7 predicted_qdot =
      input.qdot_prev + input.qddot_prev * input.dt;
  EXPECT_TRUE((problem.H.topLeftCorner<kArmDof, kArmDof>() -
               baseline.H.topLeftCorner<kArmDof, kArmDof>())
                  .isApprox(2.5 * Mat77::Identity(), 1.0e-14));
  EXPECT_TRUE((problem.g.head<kArmDof>() - baseline.g.head<kArmDof>())
                  .isApprox(-2.5 * predicted_qdot, 1.0e-14));
}

TEST(HierarchicalQpIk, ColdRetriesTransientFailureInTheSameCycle) {
  auto solver = std::make_unique<RetrySequenceSolver>(true);
  RetrySequenceSolver* observer = solver.get();
  HierarchicalQpIk7 backend(testConfig(), safetyConfig(), std::move(solver));

  const ArmIkResult result = backend.solve(deterministicInput());

  ASSERT_EQ(result.status, SolverStatus::kSolved) << result.detail;
  EXPECT_EQ(observer->initialize_count, 2);
  EXPECT_EQ(observer->solve_count, 2);
  EXPECT_EQ(observer->reset_count, 1);
  EXPECT_EQ(result.iterations, 14);
  EXPECT_DOUBLE_EQ(result.solve_time_us, 40.0);
}

TEST(HierarchicalQpIk, ColdRetriesTransientInitializationFailure) {
  auto solver = std::make_unique<InitializationRetrySolver>();
  InitializationRetrySolver* observer = solver.get();
  HierarchicalQpIk7 backend(testConfig(), safetyConfig(), std::move(solver));
  const ArmIkResult result = backend.solve(deterministicInput());
  ASSERT_EQ(result.status, SolverStatus::kSolved) << result.detail;
  EXPECT_EQ(observer->initialize_count, 2);
  EXPECT_EQ(observer->solve_count, 1);
  EXPECT_EQ(observer->reset_count, 1);
  EXPECT_EQ(result.iterations, 3);
  EXPECT_DOUBLE_EQ(result.solve_time_us, 5.0);
}

TEST(HierarchicalQpIk, PreservesFailedRetryTimingDiagnostics) {
  auto solver = std::make_unique<RetrySequenceSolver>(false);
  RetrySequenceSolver* observer = solver.get();
  HierarchicalQpIk7 backend(testConfig(), safetyConfig(), std::move(solver));

  const ArmIkResult result = backend.solve(deterministicInput());

  EXPECT_EQ(result.status, SolverStatus::kMaxIterations);
  EXPECT_EQ(observer->initialize_count, 2);
  EXPECT_EQ(observer->solve_count, 2);
  EXPECT_EQ(result.iterations, 14);
  EXPECT_DOUBLE_EQ(result.solve_time_us, 40.0);
}

}  // namespace
}  // namespace tianji_qp_ik
