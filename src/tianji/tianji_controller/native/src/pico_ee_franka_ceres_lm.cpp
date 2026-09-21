#include "tianji_qp_ik/pico_ee_franka_ceres_lm.hpp"
#include "tianji_qp_ik/so3.hpp"
#include <Eigen/SVD>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#ifdef TIANJI_HAS_CERES
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <ceres/sized_cost_function.h>
#endif

namespace tianji_qp_ik {
#ifdef TIANJI_HAS_CERES
namespace {
using Clock = std::chrono::steady_clock;
bool validPose(const Pose& p) {
  return p.position.allFinite() && isProperRotation(p.rotation);
}
double seconds(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}
}
#endif

PicoEeFrankaCeresLmIk7::PicoEeFrankaCeresLmIk7(
    PicoEeFrankaCeresLmConfig config, double margin)
    : config_(std::move(config)), margin_(margin) {
  if (!available())
    throw std::runtime_error("Ceres IK is not built; enable TIANJI_ENABLE_CERES");
  const auto positive = [](double x) { return std::isfinite(x) && x > 0.0; };
  if (!std::isfinite(margin_) || margin_ < 0.0 || config_.max_iterations < 1 ||
      config_.nullspace_attempts < 0 || config_.nullspace_correction_iterations < 1 ||
      !positive(config_.max_solver_time_seconds) ||
      !positive(config_.initial_trust_region_radius) ||
      !std::isfinite(config_.maximum_joint_displacement_rad) ||
      config_.maximum_joint_displacement_rad < 0.0 ||
      (config_.joint_displacement_limit_enabled &&
       config_.maximum_joint_displacement_rad <= 0.0) ||
      !positive(config_.position_weight) || !positive(config_.orientation_weight) ||
      !positive(config_.position_tolerance_m) || !positive(config_.orientation_tolerance_rad) ||
      !positive(config_.nullspace_gain) || !positive(config_.nullspace_max_step_rad))
    throw std::invalid_argument("Invalid Ceres LM configuration");
}

bool PicoEeFrankaCeresLmIk7::available() noexcept {
#ifdef TIANJI_HAS_CERES
  return true;
#else
  return false;
#endif
}

Mat67 PicoEeFrankaCeresLmIk7::residualJacobian(
    const Vec6& error, const Mat67& geometric) {
  const Eigen::Vector3d phi = error.tail<3>();
  const double theta2 = phi.squaredNorm();
  const double theta = std::sqrt(theta2);
  Eigen::Matrix3d cross;
  cross << 0.0, -phi.z(), phi.y(), phi.z(), 0.0, -phi.x(),
           -phi.y(), phi.x(), 0.0;
  const double coefficient = theta2 < 1e-8
      ? 1.0 / 12.0 + theta2 / 720.0
      : (1.0 - 0.5 * theta / std::tan(0.5 * theta)) / theta2;
  const Eigen::Matrix3d right_inverse =
      Eigen::Matrix3d::Identity() + 0.5 * cross + coefficient * cross * cross;
  Mat67 jacobian;
  jacobian.topRows<3>() = -geometric.topRows<3>();
  jacobian.bottomRows<3>() = -right_inverse * geometric.bottomRows<3>();
  return jacobian;
}

#ifdef TIANJI_HAS_CERES
namespace {
struct CandidateTracker {
  const PicoEeFrankaDlsInput& input;
  const PicoEeFrankaCeresLmConfig& config;
  Vec7 lower, upper, home;
  Vec7 best{Vec7::Zero()};
  Vec6 best_error{Vec6::Zero()};
  double best_cost{std::numeric_limits<double>::infinity()};
  double best_home{std::numeric_limits<double>::infinity()};
  bool valid{false};
  int evaluations{0};
  bool home_gate{false};
  double home_ceiling{0.0}, position_ceiling{0.0}, orientation_ceiling{0.0};

  double cost(const Vec6& error) const {
    return config.position_weight * error.head<3>().squaredNorm() +
           config.orientation_weight * error.tail<3>().squaredNorm();
  }
  bool evaluate(const Vec7& q, Vec6& error, Mat67* jacobian) {
    ++evaluations;
    if (!q.allFinite() || (q.array() < lower.array()).any() ||
        (q.array() > upper.array()).any()) return false;
    ArmKinematicSample sample;
    try {
      sample = input.evaluate(q);
      if (!validPose(sample.tcp_pose) || !sample.tcp_jacobian.allFinite())
        return false;
      error = poseErrorWorld(input.target, sample.tcp_pose);
    } catch (...) { return false; }
    if (!error.allFinite()) return false;
    const double value = cost(error);
    const double home_value = (q - home).squaredNorm();
    const bool eligible = !home_gate ||
        (home_value < home_ceiling &&
         error.head<3>().norm() <= position_ceiling &&
         error.tail<3>().norm() <= orientation_ceiling);
    // Pose merit always wins, including during nullspace correction.
    // Home is only a tie-breaker: never hide a lower-error candidate.
    if (!valid || value < best_cost ||
        (eligible && value == best_cost && home_value < best_home)) {
      best = q;
      best_error = error;
      best_cost = value;
      best_home = home_value;
      valid = true;
    }
    if (jacobian)
      *jacobian = PicoEeFrankaCeresLmIk7::residualJacobian(
          error, sample.tcp_jacobian);
    return true;
  }
};

class PoseCost final : public ceres::SizedCostFunction<6, 7> {
 public:
  explicit PoseCost(CandidateTracker& tracker) : tracker_(tracker) {}
  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override {
    const Vec7 q = Eigen::Map<const Vec7>(parameters[0]);
    Vec6 error;
    Mat67 jacobian;
    const bool need_jacobian = jacobians && jacobians[0];
    if (!tracker_.evaluate(q, error, need_jacobian ? &jacobian : nullptr))
      return false;
    Vec6 scale;
    scale.head<3>().setConstant(std::sqrt(tracker_.config.position_weight));
    scale.tail<3>().setConstant(std::sqrt(tracker_.config.orientation_weight));
    Eigen::Map<Vec6> residual_map(residuals);
    residual_map = scale.cwiseProduct(error);
    if (need_jacobian) {
      Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> output(jacobians[0]);
      output = scale.asDiagonal() * jacobian;
    }
    return true;
  }
 private:
  CandidateTracker& tracker_;
};

int solveLm(CandidateTracker& tracker, Vec7& q, int max_iterations,
            double remaining_time) {
  if (remaining_time <= 0.0) return 0;
  ceres::Problem problem;
  problem.AddResidualBlock(new PoseCost(tracker), nullptr, q.data());
  for (int j = 0; j < kArmDof; ++j) {
    problem.SetParameterLowerBound(q.data(), j, tracker.lower[j]);
    problem.SetParameterUpperBound(q.data(), j, tracker.upper[j]);
  }
  ceres::Solver::Options options;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  options.linear_solver_type = ceres::DENSE_QR;
  options.num_threads = 1;
  options.max_num_iterations = max_iterations;
  options.max_solver_time_in_seconds = remaining_time;
  options.initial_trust_region_radius = tracker.config.initial_trust_region_radius;
  options.function_tolerance = 1e-12;
  options.gradient_tolerance = 1e-12;
  options.parameter_tolerance = 1e-12;
  options.logging_type = ceres::SILENT;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  // Termination type is deliberately not an acceptance gate. Tracker includes
  // all finite, bounded trials, even rejected LM steps, in this solve only.
  return std::max(0, summary.num_successful_steps + summary.num_unsuccessful_steps - 1);
}
}
#endif

PicoEeFrankaCeresLmResult PicoEeFrankaCeresLmIk7::solve(
    const PicoEeFrankaDlsInput& input) const {
  PicoEeFrankaCeresLmResult result;
#ifndef TIANJI_HAS_CERES
  (void)input;
  result.detail = "ceres_not_built";
  return result;
#else
  const auto start = Clock::now();
  if (!input.seed.allFinite() || !input.seed_velocity.allFinite() ||
      !input.seed_acceleration.allFinite() || !input.home_reference.allFinite() ||
      !input.limits.lower_position.allFinite() ||
      !input.limits.upper_position.allFinite() || !std::isfinite(input.dt) ||
      input.dt <= 0.0 || !std::isfinite(margin_) || margin_ < 0.0 ||
      (input.seed.array() < input.limits.lower_position.array()).any() ||
      (input.seed.array() > input.limits.upper_position.array()).any()) {
    result.detail = "ceres_invalid_input";
    return result;
  }
  const Vec7 interior_lower = input.limits.lower_position.array() + margin_;
  const Vec7 interior_upper = input.limits.upper_position.array() - margin_;
  if ((interior_lower.array() >= interior_upper.array()).any()) {
    result.detail = "ceres_invalid_limits";
    return result;
  }
  const Vec7 seed = input.seed.cwiseMax(interior_lower).cwiseMin(interior_upper);
  const auto publish = [&](const Vec7& q) {
    result.goal = q;
    result.planner_target = q;
    result.planner_state.q = q;
    result.qdot = (q - input.seed) / input.dt;
    result.planner_state.qdot = result.qdot;
    result.planner_state.qddot = (result.qdot - input.seed_velocity) / input.dt;
    result.planner_jerk =
        (result.planner_state.qddot - input.seed_acceleration) / input.dt;
    result.accepted = result.qdot.allFinite() &&
        result.planner_state.qddot.allFinite() && result.planner_jerk.allFinite();
    result.dls.solve_time_us = seconds(start) * 1e6;
  };
  if (!input.target_valid || input.target_stale) {
    result.target_held = true;
    result.detail = "ceres_stale_or_missing_target";
    publish(seed);
    return result;
  }
  if (!validPose(input.target) || !input.evaluate) {
    result.detail = "ceres_invalid_target_or_evaluator";
    return result;
  }
  const double step = config_.maximum_joint_displacement_rad;
  const Vec7 lower = config_.joint_displacement_limit_enabled
      ? interior_lower.cwiseMax((seed.array() - step).matrix())
      : interior_lower;
  const Vec7 upper = config_.joint_displacement_limit_enabled
      ? interior_upper.cwiseMin((seed.array() + step).matrix())
      : interior_upper;
  CandidateTracker tracker{input, config_,
      lower,
      upper,
      input.home_reference.cwiseMax(interior_lower).cwiseMin(interior_upper)};
  Vec6 initial_error;
  if (!tracker.evaluate(seed, initial_error, nullptr)) {
    result.detail = "ceres_invalid_seed_kinematics";
    return result;
  }
  result.dls.initial_position_error_m = initial_error.head<3>().norm();
  result.dls.initial_orientation_error_rad = initial_error.tail<3>().norm();
  const auto converged = [&](const Vec6& error) {
    return error.head<3>().norm() <= config_.position_tolerance_m &&
           error.tail<3>().norm() <= config_.orientation_tolerance_rad;
  };
  const double initial_cost = tracker.best_cost;
  if (!converged(initial_error)) {
    Vec7 q = seed;
    result.dls.iterations = solveLm(tracker, q, config_.max_iterations,
        config_.max_solver_time_seconds - seconds(start));
    const Vec7 primary = tracker.best;
    if (config_.nullspace_enabled) {
      tracker.home_gate = true;
      tracker.home_ceiling = tracker.best_home;
      tracker.position_ceiling = std::max(config_.position_tolerance_m,
                                          tracker.best_error.head<3>().norm());
      tracker.orientation_ceiling = std::max(config_.orientation_tolerance_rad,
                                             tracker.best_error.tail<3>().norm());
      ArmKinematicSample sample;
      bool usable = false;
      try {
        sample = input.evaluate(primary);
        usable = validPose(sample.tcp_pose) && sample.tcp_jacobian.allFinite();
      } catch (...) {}
      if (usable) {
        Eigen::JacobiSVD<Mat67> svd(sample.tcp_jacobian,
                                  Eigen::ComputeFullV | Eigen::ComputeFullU);
        if (svd.info() == Eigen::Success) {
          result.dls.minimum_singular_value = svd.singularValues().minCoeff();
          Mat77 projection = Mat77::Zero();
          const double threshold = 7.0 * std::numeric_limits<double>::epsilon() *
              std::max(1.0, svd.singularValues().maxCoeff());
          for (int j = 0; j < kArmDof; ++j) {
            if (j == 6 || svd.singularValues()[j] <= threshold)
              projection += svd.matrixV().col(j) * svd.matrixV().col(j).transpose();
          }
          Vec7 direction = config_.nullspace_gain * projection * (tracker.home - primary);
          const double length = direction.norm();
          if (length > config_.nullspace_max_step_rad)
            direction *= config_.nullspace_max_step_rad / length;
          double scale = 1.0;
          for (int j = 0; j < kArmDof; ++j) {
            if (direction[j] > 0.0)
              scale = std::min(scale, (tracker.upper[j] - primary[j]) / direction[j]);
            else if (direction[j] < 0.0)
              scale = std::min(scale, (tracker.lower[j] - primary[j]) / direction[j]);
          }
          for (int attempt = 0; attempt < config_.nullspace_attempts; ++attempt) {
            const double remaining = config_.max_solver_time_seconds - seconds(start);
            if (remaining <= 0.0 || scale * direction.norm() < 1e-12) break;
            Vec7 candidate = primary + std::max(0.0, scale) * direction;
            ++result.nullspace_trials;
            result.dls.iterations += solveLm(tracker, candidate,
                config_.nullspace_correction_iterations, remaining);
            scale *= 0.5;
          }
          result.nullspace_accepted = (tracker.best - primary).norm() > 1e-12 &&
              tracker.best_home < tracker.home_ceiling;
        }
      }
    }
  }
  result.evaluations = tracker.evaluations;
  result.dls.q = tracker.best;
  result.dls.position_error_m = tracker.best_error.head<3>().norm();
  result.dls.orientation_error_rad = tracker.best_error.tail<3>().norm();
  result.dls.status = converged(tracker.best_error) ? PoseDlsStatus::kConverged
      : (tracker.best_cost < initial_cost ? PoseDlsStatus::kImproved
                                         : PoseDlsStatus::kNotConverged);
  result.detail = converged(tracker.best_error)
      ? "ceres_converged" : "ceres_current_solve_best_effort";
  result.dls.detail = result.detail;
  publish(tracker.best);
  return result;
#endif
}
}  // namespace tianji_qp_ik
