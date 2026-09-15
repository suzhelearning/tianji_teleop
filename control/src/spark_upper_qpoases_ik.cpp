#include "tianji_qp_ik/spark_upper_qpoases_ik.hpp"

#include "tianji_qp_ik/so3.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <utility>

namespace tianji_qp_ik {
namespace {

constexpr double kMinimumDirectionSegmentLength = 1.0e-6;

bool finiteTarget(const SparkUpperArmTarget& target) {
  return target.palm.position.allFinite() &&
         isProperRotation(target.palm.rotation) && target.elbow.allFinite() &&
         target.shoulder.allFinite() && target.wrist.allFinite() &&
         target.hand.allFinite();
}

bool inside(const Vec7& q, const Vec7& lower, const Vec7& upper) {
  return q.allFinite() && (q.array() >= lower.array()).all() &&
         (q.array() <= upper.array()).all();
}

}  // namespace

std::optional<SparkDirectionTaskLinearization> linearizeSparkDirectionTask(
    const Eigen::Vector3d& target_direction,
    const Eigen::Vector3d& segment_start,
    const Eigen::Vector3d& segment_end,
    const Mat37& start_jacobian,
    const Mat37& end_jacobian) noexcept {
  if (!target_direction.allFinite() || !segment_start.allFinite() ||
      !segment_end.allFinite() || !start_jacobian.allFinite() ||
      !end_jacobian.allFinite()) {
    return std::nullopt;
  }
  const double target_norm = target_direction.norm();
  const Eigen::Vector3d segment = segment_end - segment_start;
  const double segment_length = segment.norm();
  if (!std::isfinite(target_norm) ||
      target_norm <= kMinimumDirectionSegmentLength ||
      !std::isfinite(segment_length) ||
      segment_length <= kMinimumDirectionSegmentLength) {
    return std::nullopt;
  }
  const Eigen::Vector3d target = target_direction / target_norm;
  const Eigen::Vector3d current = segment / segment_length;
  SparkDirectionTaskLinearization result;
  result.error = target - current;
  result.jacobian =
      (Eigen::Matrix3d::Identity() - current * current.transpose()) /
      segment_length * (end_jacobian - start_jacobian);
  if (!result.error.allFinite() || !result.jacobian.allFinite()) {
    return std::nullopt;
  }
  return result;
}

SparkUpperQpoasesIk7::SparkUpperQpoasesIk7(
    ArmSide side, PinocchioArmKinematics& kinematics, ArmLimits limits,
    SparkUpperQpoasesConfig config,
    QpoasesConfig solver_config)
    : side_(side),
      kinematics_(kinematics),
      config_(std::move(config)),
      limits_(std::move(limits)),
      stage1_solver_(solver_config),
      stage2_solver_(solver_config),
      otg_stage1_solver_(solver_config),
      otg_stage2_solver_(solver_config) {}

QpoasesSolver7& SparkUpperQpoasesIk7::solver(SparkQpoasesStage stage) noexcept {
  return stage == SparkQpoasesStage::kStage1 ? stage1_solver_ : stage2_solver_;
}

bool& SparkUpperQpoasesIk7::initialized(SparkQpoasesStage stage) noexcept {
  return stage == SparkQpoasesStage::kStage1 ? stage1_initialized_
                                             : stage2_initialized_;
}

void SparkUpperQpoasesIk7::reset() noexcept {
  stage1_solver_.reset();
  stage2_solver_.reset();
  stage1_initialized_ = false;
  stage2_initialized_ = false;
  otg_stage1_solver_.reset();
  otg_stage2_solver_.reset();
  otg_stage1_initialized_ = false;
}

SolverResult7 SparkUpperQpoasesIk7::solveOtgStage1Qp(
    const QpProblem7& problem) {
  if (!otg_stage1_initialized_) {
    if (!otg_stage1_solver_.initialize(problem)) {
      SolverResult7 failed;
      failed.status = SolverStatus::kNumericalError;
      failed.detail = "otg_stage1_initialization_failed";
      return failed;
    }
    otg_stage1_initialized_ = true;
  }
  SolverResult7 result = otg_stage1_solver_.solve(problem);
  if (result.status == SolverStatus::kSolved) {
    return result;
  }
  otg_stage1_solver_.reset();
  otg_stage1_initialized_ = false;
  if (!otg_stage1_solver_.initialize(problem)) {
    return result;
  }
  otg_stage1_initialized_ = true;
  return otg_stage1_solver_.solve(problem);
}

bool SparkUpperQpoasesIk7::buildProblem(
    SparkQpoasesStage stage, const SparkUpperArmTarget& target, const Vec7& q,
    QpProblem7* problem, StageMetrics* metrics) {
  if (problem == nullptr || metrics == nullptr || !finiteTarget(target) ||
      !q.allFinite()) {
    return false;
  }
  const Vec7 safe_lower =
      limits_.lower_position.array() + config_.joint_limit_margin_rad;
  const Vec7 safe_upper =
      limits_.upper_position.array() - config_.joint_limit_margin_rad;
  if (!inside(q, safe_lower, safe_upper) ||
      (safe_lower.array() > safe_upper.array()).any()) {
    return false;
  }
  const ArmKinematicSample sample = kinematics_.sample(side_, q);
  if (!sample.tcp_pose.position.allFinite() ||
      !isProperRotation(sample.tcp_pose.rotation) ||
      !sample.tcp_jacobian.allFinite() ||
      !sample.shoulder_position.allFinite() ||
      !sample.elbow_position.allFinite() ||
      !sample.wrist_position.allFinite() ||
      !sample.shoulder_position_jacobian.allFinite() ||
      !sample.elbow_position_jacobian.allFinite() ||
      !sample.wrist_position_jacobian.allFinite()) {
    return false;
  }

  const Vec6 pose_error = poseErrorWorld(target.palm, sample.tcp_pose);
  const auto upper_direction = linearizeSparkDirectionTask(
      target.elbow - target.shoulder, sample.shoulder_position,
      sample.elbow_position, sample.shoulder_position_jacobian,
      sample.elbow_position_jacobian);
  const auto forearm_direction = linearizeSparkDirectionTask(
      target.wrist - target.elbow, sample.elbow_position,
      sample.wrist_position, sample.elbow_position_jacobian,
      sample.wrist_position_jacobian);
  if (!pose_error.allFinite() || !upper_direction.has_value() ||
      !forearm_direction.has_value()) {
    return false;
  }

  const int rows = stage == SparkQpoasesStage::kStage1 ? 9 : 18;
  Eigen::MatrixXd weighted_jacobian = Eigen::MatrixXd::Zero(rows, kArmDof);
  Eigen::VectorXd weighted_error = Eigen::VectorXd::Zero(rows);
  int row = 0;
  const auto append = [&](const Eigen::MatrixXd& jacobian,
                          const Eigen::VectorXd& error, double weight) {
    const double scale = std::sqrt(weight);
    weighted_jacobian.block(row, 0, jacobian.rows(), kArmDof) = scale * jacobian;
    weighted_error.segment(row, error.size()) = scale * error;
    row += static_cast<int>(error.size());
  };

  append(upper_direction->jacobian, upper_direction->error,
         stage == SparkQpoasesStage::kStage1
             ? config_.stage1_upper_direction_weight
             : config_.stage2_upper_direction_weight);
  append(forearm_direction->jacobian, forearm_direction->error,
         stage == SparkQpoasesStage::kStage1
             ? config_.stage1_forearm_direction_weight
             : config_.stage2_forearm_direction_weight);
  append(sample.tcp_jacobian.bottomRows<3>(), pose_error.tail<3>(),
         stage == SparkQpoasesStage::kStage1
             ? config_.stage1_palm_orientation_weight
             : config_.stage2_palm_orientation_weight);
  if (stage == SparkQpoasesStage::kStage2) {
    append(sample.elbow_position_jacobian,
           target.elbow - sample.elbow_position,
           config_.stage2_elbow_position_weight);
    append(sample.wrist_position_jacobian,
           target.wrist - sample.wrist_position,
           config_.stage2_wrist_position_weight);
    append(sample.tcp_jacobian.topRows<3>(), pose_error.head<3>(),
           config_.stage2_palm_position_weight);
  }
  if (row != rows) {
    return false;
  }

  const Mat77 hessian =
      weighted_jacobian.transpose() * weighted_jacobian +
      config_.damping * Mat77::Identity();
  const Vec7 gradient = -weighted_jacobian.transpose() * weighted_error;
  if (!hessian.allFinite() || !gradient.allFinite()) {
    return false;
  }
  problem->H = hessian;
  problem->g = gradient;
  problem->lower =
      (safe_lower - q).array().max(-config_.trust_region_rad).matrix();
  problem->upper =
      (safe_upper - q).array().min(config_.trust_region_rad).matrix();
  if (!problem->lower.allFinite() || !problem->upper.allFinite() ||
      (problem->lower.array() > problem->upper.array()).any()) {
    return false;
  }
  metrics->weighted_error = weighted_error.norm();
  metrics->upper_direction_error = upper_direction->error.norm();
  metrics->forearm_direction_error = forearm_direction->error.norm();
  metrics->palm_position_error = pose_error.head<3>().norm();
  metrics->palm_orientation_error = pose_error.tail<3>().norm();
  metrics->palm_primary_error = std::sqrt(
      config_.stage2_palm_position_weight *
          metrics->palm_position_error * metrics->palm_position_error +
      config_.stage2_palm_orientation_weight *
          metrics->palm_orientation_error * metrics->palm_orientation_error);
  return std::isfinite(metrics->weighted_error) &&
         std::isfinite(metrics->palm_primary_error);
}

SolverResult7 SparkUpperQpoasesIk7::solveQp(
    SparkQpoasesStage stage, const QpProblem7& problem,
    bool* used_hotstart) {
  SolverResult7 result;
  if (used_hotstart == nullptr) {
    result.detail = "null_hotstart_output";
    return result;
  }
  QpoasesSolver7& active_solver = solver(stage);
  bool& active_initialized = initialized(stage);
  *used_hotstart = active_initialized;
  if (!active_initialized) {
    if (!active_solver.initialize(problem)) {
      result.status = SolverStatus::kNumericalError;
      result.detail = "qpOASES_initialization_failed";
      return result;
    }
    active_initialized = true;
    result = active_solver.solve(problem);
    *used_hotstart = false;
    return result;
  }

  result = active_solver.solve(problem);
  if (result.status == SolverStatus::kSolved) {
    return result;
  }

  active_solver.reset();
  active_initialized = false;
  if (!active_solver.initialize(problem)) {
    result.status = SolverStatus::kNumericalError;
    result.detail = "qpOASES_hotstart_and_reinitialization_failed";
    *used_hotstart = true;
    return result;
  }
  active_initialized = true;
  result = active_solver.solve(problem);
  *used_hotstart = false;
  return result;
}

SparkUpperQpoasesIk7::StageResult SparkUpperQpoasesIk7::solveStage(
    SparkQpoasesStage stage, const SparkUpperArmTarget& target,
    const Vec7& seed, std::chrono::steady_clock::time_point deadline) {
  StageResult result;
  result.q = seed;
  if (!finiteTarget(target) || !seed.allFinite()) {
    result.detail = "invalid_spark_upper_ik_input";
    return result;
  }

  Vec7 current = seed;
  StageMetrics current_metrics;
  QpProblem7 current_problem;
  if (!buildProblem(stage, target, current, &current_problem,
                    &current_metrics)) {
    result.detail = "invalid_spark_upper_ik_problem";
    result.status = SolverStatus::kInvalidInput;
    return result;
  }
  const int max_iterations =
      stage == SparkQpoasesStage::kStage1
          ? config_.stage1_max_iterations
          : config_.stage2_max_iterations;
  constexpr double kLineSearchScales[] = {1.0, 0.5, 0.25, 0.125};
  constexpr double kStrictImprovement = 1.0e-12;
  for (int iteration = 0; iteration < max_iterations; ++iteration) {
    if (std::chrono::steady_clock::now() >= deadline) {
      result.budget_exhausted = true;
      result.detail = "spark_upper_ik_budget_exhausted";
      break;
    }
    bool used_hotstart = false;
    const SolverResult7 qp_result =
        solveQp(stage, current_problem, &used_hotstart);
    result.solve_time_us += qp_result.solve_time_us;
    result.hotstart = result.hotstart || used_hotstart;
    result.iterations = iteration + 1;
    if (qp_result.status != SolverStatus::kSolved ||
        !qp_result.qdot.allFinite()) {
      result.status = qp_result.status;
      result.detail = qp_result.detail;
      result.q = current;
      result.error = current_metrics.weighted_error;
      return result;
    }

    const Vec7 safe_lower =
        limits_.lower_position.array() + config_.joint_limit_margin_rad;
    const Vec7 safe_upper =
        limits_.upper_position.array() - config_.joint_limit_margin_rad;
    bool improved = false;
    Vec7 accepted_candidate = current;
    QpProblem7 accepted_problem = current_problem;
    StageMetrics accepted_metrics = current_metrics;
    for (std::size_t scale_index = 0;
         scale_index < std::size(kLineSearchScales); ++scale_index) {
      const double scale = kLineSearchScales[scale_index];
      // qpOASES can return a solution a few ulps beyond an active bound. Clamp
      // that residue after applying the configured fast-retarget integration
      // step; the QP bounds already enforce the trust region.
      const Vec7 candidate =
          (current + scale * config_.integration_step * qp_result.qdot)
              .array()
              .max(safe_lower.array())
              .min(safe_upper.array())
              .matrix();
      if (!inside(candidate, safe_lower, safe_upper)) {
        continue;
      }
      QpProblem7 candidate_problem;
      StageMetrics candidate_metrics;
      if (!buildProblem(stage, target, candidate, &candidate_problem,
                        &candidate_metrics)) {
        continue;
      }
      const bool primary_improved =
          stage == SparkQpoasesStage::kStage1
              ? candidate_metrics.weighted_error + kStrictImprovement <
                    current_metrics.weighted_error
              : candidate_metrics.palm_primary_error + kStrictImprovement <
                    current_metrics.palm_primary_error;
      if (!primary_improved) {
        continue;
      }
      accepted_candidate = candidate;
      accepted_problem = candidate_problem;
      accepted_metrics = candidate_metrics;
      result.backtracking_steps += static_cast<int>(scale_index);
      improved = true;
      break;
    }
    if (!improved) {
      result.detail = "spark_upper_ik_no_improvement";
      break;
    }
    const double error_delta =
        std::abs(current_metrics.weighted_error -
                 accepted_metrics.weighted_error);
    current = accepted_candidate;
    current_problem = accepted_problem;
    current_metrics = accepted_metrics;
    if (current_metrics.weighted_error <= 1.0e-6 ||
        error_delta < config_.convergence_delta) {
      result.detail = "spark_upper_ik_converged";
      break;
    }
  }

  result.accepted = true;
  result.status = SolverStatus::kSolved;
  result.q = current;
  result.error = current_metrics.weighted_error;
  result.upper_direction_error = current_metrics.upper_direction_error;
  result.forearm_direction_error = current_metrics.forearm_direction_error;
  result.palm_position_error = current_metrics.palm_position_error;
  result.palm_orientation_error = current_metrics.palm_orientation_error;
  if (result.detail == "not_solved") {
    result.detail = "spark_upper_ik_stage_solved";
  }
  return result;
}

SparkUpperIkResult SparkUpperQpoasesIk7::solve(
    const SparkUpperArmTarget& target, const Vec7& seed,
    std::chrono::steady_clock::time_point deadline) {
  SparkUpperIkResult result;
  result.q = seed;
  const Vec7 safe_lower =
      limits_.lower_position.array() + config_.joint_limit_margin_rad;
  const Vec7 safe_upper =
      limits_.upper_position.array() - config_.joint_limit_margin_rad;
  if (!finiteTarget(target)) {
    result.detail = "invalid_spark_upper_ik_target";
    return result;
  }
  if ((safe_lower.array() > safe_upper.array()).any()) {
    result.detail = "invalid_spark_upper_ik_limits";
    return result;
  }
  if (!seed.allFinite()) {
    result.detail = "invalid_spark_upper_ik_seed";
    return result;
  }
  const Vec7 clamped_seed =
      seed.array().max(safe_lower.array()).min(safe_upper.array()).matrix();

  result.q = clamped_seed;
  const StageResult stage1 =
      solveStage(SparkQpoasesStage::kStage1, target, clamped_seed, deadline);
  result.stage1_iterations = stage1.iterations;
  result.stage1_error = stage1.error;
  result.stage1_solve_time_us = stage1.solve_time_us;
  result.stage1_hotstart = stage1.hotstart;
  result.budget_exhausted = stage1.budget_exhausted;
  result.stage1_backtracking_steps = stage1.backtracking_steps;
  result.stage1_q = stage1.q;
  if (!stage1.accepted) {
    result.status = stage1.status;
    result.detail = stage1.detail;
    result.q = stage1.q;
    return result;
  }

  const StageResult stage2 =
      solveStage(SparkQpoasesStage::kStage2, target, stage1.q, deadline);
  result.stage2_iterations = stage2.iterations;
  result.stage2_error = stage2.error;
  result.stage2_solve_time_us = stage2.solve_time_us;
  result.stage2_hotstart = stage2.hotstart;
  result.budget_exhausted =
      result.budget_exhausted || stage2.budget_exhausted;
  result.stage2_backtracking_steps = stage2.backtracking_steps;
  result.upper_direction_error = stage2.upper_direction_error;
  result.forearm_direction_error = stage2.forearm_direction_error;
  result.palm_position_error = stage2.palm_position_error;
  result.palm_orientation_error = stage2.palm_orientation_error;
  result.q = stage2.q;
  result.status = stage2.status;
  result.detail = stage2.detail;
  result.accepted = stage2.accepted;
  return result;
}

SparkUpperIkResult SparkUpperQpoasesIk7::solveOtgConsistent(
    const SparkUpperArmTarget& shape_target, const Pose& otg_pose,
    const Vec7& seed, std::chrono::steady_clock::time_point deadline) {
  SparkUpperIkResult result;
  result.q = seed;
  if (!finiteTarget(shape_target) || !otg_pose.position.allFinite() ||
      !isProperRotation(otg_pose.rotation) || !seed.allFinite()) {
    result.detail = "invalid_otg_consistent_ik_input";
    return result;
  }

  const Vec7 safe_lower =
      limits_.lower_position.array() + config_.joint_limit_margin_rad;
  const Vec7 safe_upper =
      limits_.upper_position.array() - config_.joint_limit_margin_rad;
  if ((safe_lower.array() > safe_upper.array()).any()) {
    result.detail = "invalid_otg_consistent_ik_limits";
    return result;
  }
  const Vec7 clamped_seed =
      seed.array().max(safe_lower.array()).min(safe_upper.array()).matrix();
  Vec7 current = clamped_seed;

  const auto poseMerit = [&](const Vec6& error) {
    return error.head<3>().squaredNorm() /
               std::pow(config_.otg_position_tolerance_m, 2) +
           error.tail<3>().squaredNorm() /
               std::pow(config_.otg_orientation_tolerance_rad, 2);
  };
  const auto withinPoseTolerance = [&](const Vec6& error) {
    return error.head<3>().norm() <= config_.otg_position_tolerance_m &&
           error.tail<3>().norm() <=
               config_.otg_orientation_tolerance_rad;
  };

  ArmKinematicSample sample = kinematics_.sample(side_, current);
  Vec6 pose_error = poseErrorWorld(otg_pose, sample.tcp_pose);
  if (!pose_error.allFinite() || !sample.tcp_jacobian.allFinite()) {
    result.detail = "invalid_otg_consistent_stage1_sample";
    return result;
  }

  constexpr double kLineSearchScales[] = {1.0, 0.5, 0.25, 0.125};
  for (int iteration = 0;
       iteration < config_.stage1_max_iterations &&
       !withinPoseTolerance(pose_error);
       ++iteration) {
    if (std::chrono::steady_clock::now() >= deadline) {
      result.budget_exhausted = true;
      result.detail = "otg_consistent_stage1_budget_exhausted";
      break;
    }
    QpProblem7 problem;
    Mat67 weighted_tcp_jacobian = sample.tcp_jacobian;
    Vec6 weighted_pose_error = pose_error;
    const double position_scale =
        1.0 / config_.otg_position_tolerance_m;
    const double orientation_scale =
        1.0 / config_.otg_orientation_tolerance_rad;
    weighted_tcp_jacobian.topRows<3>() *= position_scale;
    weighted_tcp_jacobian.bottomRows<3>() *= orientation_scale;
    weighted_pose_error.head<3>() *= position_scale;
    weighted_pose_error.tail<3>() *= orientation_scale;
    problem.H = weighted_tcp_jacobian.transpose() * weighted_tcp_jacobian +
                (config_.damping + config_.otg_continuity_weight) *
                    Mat77::Identity();
    problem.g = -weighted_tcp_jacobian.transpose() * weighted_pose_error +
                config_.otg_continuity_weight * (current - clamped_seed);
    problem.lower =
        (safe_lower - current).array().max(-config_.trust_region_rad).matrix();
    problem.upper =
        (safe_upper - current).array().min(config_.trust_region_rad).matrix();
    const SolverResult7 qp_result = solveOtgStage1Qp(problem);
    result.stage1_solve_time_us += qp_result.solve_time_us;
    result.stage1_iterations = iteration + 1;
    if (qp_result.status != SolverStatus::kSolved ||
        !qp_result.qdot.allFinite()) {
      result.status = qp_result.status;
      result.detail = "otg_consistent_stage1_qp_failed";
      return result;
    }

    const double current_merit = poseMerit(pose_error);
    bool improved = false;
    for (std::size_t scale_index = 0;
         scale_index < std::size(kLineSearchScales); ++scale_index) {
      const Vec7 candidate =
          (current + kLineSearchScales[scale_index] *
                         config_.integration_step * qp_result.qdot)
              .array()
              .max(safe_lower.array())
              .min(safe_upper.array())
              .matrix();
      const ArmKinematicSample candidate_sample =
          kinematics_.sample(side_, candidate);
      const Vec6 candidate_error =
          poseErrorWorld(otg_pose, candidate_sample.tcp_pose);
      if (!candidate_error.allFinite() ||
          !candidate_sample.tcp_jacobian.allFinite() ||
          poseMerit(candidate_error) + 1.0e-12 >= current_merit) {
        continue;
      }
      current = candidate;
      sample = candidate_sample;
      pose_error = candidate_error;
      result.stage1_backtracking_steps += static_cast<int>(scale_index);
      improved = true;
      break;
    }
    if (!improved) {
      result.detail = "otg_consistent_stage1_no_improvement";
      break;
    }
  }

  result.stage1_q = current;
  result.stage1_error = poseMerit(pose_error);
  result.palm_position_error = pose_error.head<3>().norm();
  result.palm_orientation_error = pose_error.tail<3>().norm();
  if (!withinPoseTolerance(pose_error)) {
    result.q = current;
    result.status = SolverStatus::kMaxIterations;
    if (result.detail == "not_solved") {
      result.detail = "otg_consistent_stage1_tolerance_not_met";
    }
    return result;
  }

  // Stage 1 is always a valid fallback. Stage 2 may only replace it when the
  // nonlinear FK still satisfies the exact OTG TCP tolerances.
  result.accepted = true;
  result.status = SolverStatus::kSolved;
  result.q = current;
  result.detail = "otg_consistent_stage1_accepted";

  const auto shapeLinearization = [&](const ArmKinematicSample& current_sample,
                                      Eigen::MatrixXd* weighted_jacobian,
                                      Eigen::VectorXd* weighted_error,
                                      double* shape_error) {
    const auto upper = linearizeSparkDirectionTask(
        shape_target.elbow - shape_target.shoulder,
        current_sample.shoulder_position, current_sample.elbow_position,
        current_sample.shoulder_position_jacobian,
        current_sample.elbow_position_jacobian);
    const auto forearm = linearizeSparkDirectionTask(
        shape_target.wrist - shape_target.elbow,
        current_sample.elbow_position, current_sample.wrist_position,
        current_sample.elbow_position_jacobian,
        current_sample.wrist_position_jacobian);
    if (!upper.has_value() || !forearm.has_value()) {
      return false;
    }
    weighted_jacobian->resize(9, kArmDof);
    weighted_error->resize(9);
    const double upper_scale =
        std::sqrt(config_.stage2_upper_direction_weight);
    const double forearm_scale =
        std::sqrt(config_.stage2_forearm_direction_weight);
    const double elbow_scale =
        std::sqrt(config_.stage2_elbow_position_weight);
    weighted_jacobian->topRows<3>() = upper_scale * upper->jacobian;
    weighted_error->head<3>() = upper_scale * upper->error;
    weighted_jacobian->middleRows<3>(3) =
        forearm_scale * forearm->jacobian;
    weighted_error->segment<3>(3) = forearm_scale * forearm->error;
    weighted_jacobian->bottomRows<3>() =
        elbow_scale * current_sample.elbow_position_jacobian;
    weighted_error->tail<3>() =
        elbow_scale * (shape_target.elbow - current_sample.elbow_position);
    *shape_error = weighted_error->squaredNorm();
    return weighted_jacobian->allFinite() && weighted_error->allFinite() &&
           std::isfinite(*shape_error);
  };

  Eigen::MatrixXd weighted_jacobian;
  Eigen::VectorXd weighted_error;
  double current_shape_error = 0.0;
  if (!shapeLinearization(sample, &weighted_jacobian, &weighted_error,
                          &current_shape_error)) {
    return result;
  }
  for (int iteration = 0; iteration < config_.stage2_max_iterations;
       ++iteration) {
    if (std::chrono::steady_clock::now() >= deadline) {
      result.budget_exhausted = true;
      break;
    }
    pose_error = poseErrorWorld(otg_pose, sample.tcp_pose);
    EqualityConstrainedQpProblem7 problem;
    problem.H = weighted_jacobian.transpose() * weighted_jacobian +
                (config_.damping + config_.otg_continuity_weight) *
                    Mat77::Identity();
    problem.g = -weighted_jacobian.transpose() * weighted_error +
                config_.otg_continuity_weight * (current - clamped_seed);
    problem.A = sample.tcp_jacobian;
    problem.equality = pose_error;
    problem.lower =
        (safe_lower - current).array().max(-config_.trust_region_rad).matrix();
    problem.upper =
        (safe_upper - current).array().min(config_.trust_region_rad).matrix();
    const int prior_hotstarts = otg_stage2_solver_.hotstartCount();
    const SolverResult7 qp_result = otg_stage2_solver_.solve(problem);
    result.stage2_solve_time_us += qp_result.solve_time_us;
    result.stage2_iterations = iteration + 1;
    result.stage2_hotstart =
        result.stage2_hotstart ||
        otg_stage2_solver_.hotstartCount() > prior_hotstarts;
    if (qp_result.status != SolverStatus::kSolved ||
        !qp_result.qdot.allFinite()) {
      break;
    }
    const Vec7 candidate =
        (current + qp_result.qdot)
            .array()
            .max(safe_lower.array())
            .min(safe_upper.array())
            .matrix();
    const ArmKinematicSample candidate_sample =
        kinematics_.sample(side_, candidate);
    const Vec6 candidate_pose_error =
        poseErrorWorld(otg_pose, candidate_sample.tcp_pose);
    Eigen::MatrixXd candidate_jacobian;
    Eigen::VectorXd candidate_error;
    double candidate_shape_error = 0.0;
    if (!withinPoseTolerance(candidate_pose_error) ||
        !shapeLinearization(candidate_sample, &candidate_jacobian,
                            &candidate_error, &candidate_shape_error) ||
        candidate_shape_error + 1.0e-12 >= current_shape_error) {
      break;
    }
    current = candidate;
    sample = candidate_sample;
    pose_error = candidate_pose_error;
    weighted_jacobian = candidate_jacobian;
    weighted_error = candidate_error;
    current_shape_error = candidate_shape_error;
    result.q = current;
    result.stage2_error = std::sqrt(current_shape_error);
    result.palm_position_error = pose_error.head<3>().norm();
    result.palm_orientation_error = pose_error.tail<3>().norm();
    result.detail = "otg_consistent_stage2_accepted";
  }

  const auto upper = linearizeSparkDirectionTask(
      shape_target.elbow - shape_target.shoulder, sample.shoulder_position,
      sample.elbow_position, sample.shoulder_position_jacobian,
      sample.elbow_position_jacobian);
  const auto forearm = linearizeSparkDirectionTask(
      shape_target.wrist - shape_target.elbow, sample.elbow_position,
      sample.wrist_position, sample.elbow_position_jacobian,
      sample.wrist_position_jacobian);
  if (upper.has_value()) {
    result.upper_direction_error = upper->error.norm();
  }
  if (forearm.has_value()) {
    result.forearm_direction_error = forearm->error.norm();
  }
  return result;
}

}  // namespace tianji_qp_ik
