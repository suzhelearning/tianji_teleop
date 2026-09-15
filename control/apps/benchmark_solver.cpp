#include "tianji_qp_ik/benchmark_dataset.hpp"
#include "tianji_qp_ik/cartesian_servo.hpp"
#include "tianji_qp_ik/controller.hpp"
#include "tianji_qp_ik/mujoco_robot.hpp"
#include "tianji_qp_ik/qp_builder.hpp"
#include "tianji_qp_ik/safety.hpp"
#include "tianji_qp_ik/target_manager.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace tianji_qp_ik {
namespace {

constexpr std::size_t kMinimumQualifyingSamples = 20000U;

struct Options {
  std::string config_path{"config/qp_ik.yaml"};
  std::string model_path{"models/marvin_m6_qp_test.xml"};
  std::string output_path{"benchmark_results.csv"};
  std::size_t samples{20000U};
  std::uint32_t seed{20260809U};
};

struct Statistics {
  double mean{0.0};
  double p50{0.0};
  double p95{0.0};
  double p99{0.0};
  double maximum{0.0};
};

struct BackendResult {
  SolverBackend backend{SolverBackend::kOsqp};
  std::vector<std::array<Vec7, 2>> solutions;
  std::vector<double> dual_total_us;
  std::vector<double> update_us;
  std::vector<double> solve_us;
  std::vector<double> per_arm_solver_total_us;
  std::vector<double> single_arm_cycle_us;
  std::vector<double> iterations;
  std::vector<double> initialization_us;
  std::vector<double> full_cycle_us;
  std::size_t solver_failures{0U};
  std::size_t bound_failures{0U};
  std::size_t cycle_failures{0U};
  std::size_t single_arm_failures{0U};
  double maximum_kkt_residual{0.0};
  double maximum_bound_violation{0.0};
  std::array<std::size_t, kBenchmarkCategoryCount> category_solver_failures{};
  std::array<std::size_t, kBenchmarkCategoryCount> category_bound_failures{};
  std::array<std::size_t, 5U> status_counts{};
};

double elapsedUs(std::chrono::steady_clock::time_point start,
                 std::chrono::steady_clock::time_point end) {
  return std::chrono::duration<double, std::micro>(end - start).count();
}

Options parseOptions(int argc, char** argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--help") {
      std::cout << "Usage: tianji_qp_ik_benchmark [--config FILE] [--model FILE] "
                   "[--output FILE] [--samples N>=20000] [--seed N]\n";
      std::exit(0);
    }
    if (index + 1 >= argc) {
      throw std::invalid_argument("missing value after " + argument);
    }
    const std::string value(argv[++index]);
    if (argument == "--config") {
      options.config_path = value;
    } else if (argument == "--model") {
      options.model_path = value;
    } else if (argument == "--output") {
      options.output_path = value;
    } else if (argument == "--samples") {
      options.samples = static_cast<std::size_t>(std::stoull(value));
    } else if (argument == "--seed") {
      options.seed = static_cast<std::uint32_t>(std::stoul(value));
    } else {
      throw std::invalid_argument("unknown option: " + argument);
    }
  }
  if (options.samples < kMinimumQualifyingSamples) {
    throw std::invalid_argument("--samples must be at least 20000 for a qualifying comparison");
  }
  return options;
}

double quantile(const std::vector<double>& sorted, double probability) {
  if (sorted.empty()) {
    return 0.0;
  }
  const double position = probability * static_cast<double>(sorted.size() - 1U);
  const std::size_t lower = static_cast<std::size_t>(std::floor(position));
  const std::size_t upper = static_cast<std::size_t>(std::ceil(position));
  const double fraction = position - static_cast<double>(lower);
  return sorted[lower] + fraction * (sorted[upper] - sorted[lower]);
}

Statistics summarize(const std::vector<double>& values) {
  Statistics result;
  if (values.empty()) {
    return result;
  }
  std::vector<double> sorted = values;
  std::sort(sorted.begin(), sorted.end());
  for (const double value : sorted) {
    result.mean += value;
  }
  result.mean /= static_cast<double>(sorted.size());
  result.p50 = quantile(sorted, 0.50);
  result.p95 = quantile(sorted, 0.95);
  result.p99 = quantile(sorted, 0.99);
  result.maximum = sorted.back();
  return result;
}

double objective(const QpProblem7& problem, const Vec7& solution) {
  return 0.5 * solution.dot(problem.H * solution) + problem.g.dot(solution);
}

double projectedKktResidual(const QpProblem7& problem, const Vec7& solution) {
  constexpr double kActiveTolerance = 1e-6;
  const Vec7 gradient = problem.H * solution + problem.g;
  double residual = 0.0;
  for (int joint = 0; joint < kArmDof; ++joint) {
    if (solution[joint] <= problem.lower[joint] + kActiveTolerance) {
      residual = std::max(residual, std::max(0.0, -gradient[joint]));
    } else if (solution[joint] >= problem.upper[joint] - kActiveTolerance) {
      residual = std::max(residual, std::max(0.0, gradient[joint]));
    } else {
      residual = std::max(residual, std::abs(gradient[joint]));
    }
  }
  return residual;
}

void consumeSolve(const QpProblem7& problem, const SolverResult7& solution,
                  BenchmarkCategory category, const SafetyGuard& safety,
                  BackendResult& result) {
  result.update_us.push_back(solution.update_time_us);
  result.solve_us.push_back(solution.solve_time_us);
  result.per_arm_solver_total_us.push_back(solution.update_time_us + solution.solve_time_us);
  result.iterations.push_back(static_cast<double>(solution.iterations));
  ++result.status_counts.at(static_cast<std::size_t>(solution.status));
  if (solution.status != SolverStatus::kSolved) {
    ++result.solver_failures;
    ++result.category_solver_failures.at(static_cast<std::size_t>(category));
  } else if (!safety.validateResult(problem, solution).accepted) {
    ++result.bound_failures;
    ++result.category_bound_failures.at(static_cast<std::size_t>(category));
  }
  if (solution.status == SolverStatus::kSolved) {
    result.maximum_kkt_residual =
        std::max(result.maximum_kkt_residual, projectedKktResidual(problem, solution.qdot));
    result.maximum_bound_violation =
        std::max(result.maximum_bound_violation,
                 std::max(0.0, (problem.lower - solution.qdot).maxCoeff()));
    result.maximum_bound_violation =
        std::max(result.maximum_bound_violation,
                 std::max(0.0, (solution.qdot - problem.upper).maxCoeff()));
  }
}

BackendResult benchmarkProblems(SolverBackend backend, const QpIkConfig& config,
                                const std::vector<BenchmarkSample>& samples) {
  BackendResult result;
  result.backend = backend;
  result.solutions.reserve(samples.size());
  result.dual_total_us.reserve(samples.size());
  result.update_us.reserve(2U * samples.size());
  result.solve_us.reserve(2U * samples.size());
  result.per_arm_solver_total_us.reserve(2U * samples.size());
  result.iterations.reserve(2U * samples.size());
  SafetyGuard safety(config.safety);
  std::unique_ptr<IQpSolver7> left = makeSolver(backend, config);
  std::unique_ptr<IQpSolver7> right = makeSolver(backend, config);
  auto initialization_start = std::chrono::steady_clock::now();
  const bool left_initialized = left->initialize(samples.front().left);
  result.initialization_us.push_back(
      elapsedUs(initialization_start, std::chrono::steady_clock::now()));
  initialization_start = std::chrono::steady_clock::now();
  const bool right_initialized = right->initialize(samples.front().right);
  result.initialization_us.push_back(
      elapsedUs(initialization_start, std::chrono::steady_clock::now()));
  if (!left_initialized || !right_initialized) {
    throw std::runtime_error(toString(backend) + " initialization failed");
  }

  constexpr std::size_t kWarmupCount = 1000U;
  for (std::size_t index = 0; index < kWarmupCount; ++index) {
    const BenchmarkSample& sample = samples[index % samples.size()];
    (void)left->solve(sample.left);
    (void)right->solve(sample.right);
  }
  left->reset();
  right->reset();
  if (!left->initialize(samples.front().left) || !right->initialize(samples.front().right)) {
    throw std::runtime_error(toString(backend) + " reinitialization failed");
  }

  for (const BenchmarkSample& sample : samples) {
    const auto start = std::chrono::steady_clock::now();
    const SolverResult7 left_result = left->solve(sample.left);
    const SolverResult7 right_result = right->solve(sample.right);
    result.dual_total_us.push_back(elapsedUs(start, std::chrono::steady_clock::now()));
    consumeSolve(sample.left, left_result, sample.category, safety, result);
    consumeSolve(sample.right, right_result, sample.category, safety, result);
    result.solutions.push_back({left_result.qdot, right_result.qdot});
  }
  return result;
}

void setNominalConfiguration(MujocoRobot& robot) {
  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    const ArmLimits& limits = robot.mapping(side).limits;
    robot.setArmPosition(side, 0.5 * (limits.lower_position + limits.upper_position));
  }
  robot.forward();
}

std::vector<double> benchmarkFullCycle(SolverBackend backend, const QpIkConfig& config,
                                       const std::string& model_path, std::size_t samples,
                                       std::size_t& failures) {
  MujocoRobot robot(model_path);
  setNominalConfiguration(robot);
  const DualArmTargets initial{robot.tcpPose(ArmSide::kLeft), robot.tcpPose(ArmSide::kRight)};
  TargetManager targets(config, initial);
  targets.setMode(TargetMode::kCombined, 0.0);
  DualArmController controller(robot, config, makeSolver(backend, config),
                               makeSolver(backend, config));
  constexpr std::size_t kWarmupCount = 1000U;
  const double dt = 1.0 / config.controller.rate_hz;
  for (std::size_t index = 0; index < kWarmupCount; ++index) {
    const double time = static_cast<double>(index) * dt;
    if (!controller.step(targets.sample(time), dt).accepted) {
      ++failures;
    }
  }

  std::vector<double> timings;
  timings.reserve(samples);
  for (std::size_t index = 0; index < samples; ++index) {
    const double time = static_cast<double>(index + kWarmupCount) * dt;
    const auto start = std::chrono::steady_clock::now();
    const DualArmTargets desired = targets.sample(time);
    const ControllerDiagnostics diagnostics = controller.step(desired, dt);
    timings.push_back(elapsedUs(start, std::chrono::steady_clock::now()));
    if (!diagnostics.accepted) {
      ++failures;
    }
  }
  return timings;
}

std::vector<double> benchmarkSingleArmCycle(SolverBackend backend, ArmSide side,
                                            const QpIkConfig& config,
                                            const std::string& model_path,
                                            std::size_t samples, std::size_t& failures) {
  MujocoRobot robot(model_path);
  setNominalConfiguration(robot);
  TargetManager targets(config,
                        {robot.tcpPose(ArmSide::kLeft), robot.tcpPose(ArmSide::kRight)});
  targets.setMode(TargetMode::kCombined, 0.0);
  QpBuilder builder(config.qp, config.joint_limits);
  SafetyGuard safety(config.safety);
  std::unique_ptr<IQpSolver7> solver = makeSolver(backend, config);
  bool initialized = false;
  const double dt = 1.0 / config.controller.rate_hz;
  constexpr std::size_t kWarmupCount = 1000U;
  std::vector<double> timings;
  timings.reserve(samples);

  for (std::size_t index = 0; index < kWarmupCount + samples; ++index) {
    const auto start = std::chrono::steady_clock::now();
    const double time = static_cast<double>(index) * dt;
    const DualArmTargets desired = targets.sample(time);
    robot.forward();
    const Pose current = robot.tcpPose(side);
    const Mat67 jacobian = robot.tcpJacobianWorld(side);
    const Vec7 q = robot.armPosition(side);
    const Pose& target = side == ArmSide::kLeft ? desired.left : desired.right;
    const QpProblem7 problem = builder.build(
        jacobian, cartesianServoTwist(config.cartesian_servo, target, current), q,
        robot.mapping(side).limits, dt);
    SafetyDecision decision = safety.validateProblem(problem);
    if (decision.accepted && !initialized) {
      initialized = solver->initialize(problem);
      decision.accepted = initialized;
      decision.reason = initialized ? HoldReason::kNone : HoldReason::kSolverFailure;
    }
    if (decision.accepted) {
      const SolverResult7 solution = solver->solve(problem);
      decision = safety.validateResult(problem, solution);
      if (decision.accepted) {
        robot.setArmPosition(side, q + dt * solution.qdot);
      }
    }
    if (!decision.accepted) {
      ++failures;
    }
    if (index >= kWarmupCount) {
      timings.push_back(elapsedUs(start, std::chrono::steady_clock::now()));
    }
  }
  return timings;
}

void printStatistics(const std::string& backend, const std::string& metric,
                     const Statistics& stats) {
  std::cout << std::left << std::setw(9) << backend << " " << std::setw(16) << metric
            << " mean=" << std::fixed << std::setprecision(3) << stats.mean
            << " us p50=" << stats.p50 << " p95=" << stats.p95 << " p99=" << stats.p99
            << " max=" << stats.maximum << '\n';
}

void writeMetric(std::ofstream& output, SolverBackend backend, const std::string& metric,
                 const std::string& unit, const Statistics& stats, std::size_t samples,
                 std::size_t failures) {
  output << toString(backend) << ',' << metric << ',' << unit << ',' << samples << ','
         << failures << ','
         << std::setprecision(12) << stats.mean << ',' << stats.p50 << ',' << stats.p95 << ','
         << stats.p99 << ',' << stats.maximum << '\n';
}

int run(int argc, char** argv) {
  const Options options = parseOptions(argc, argv);
  const QpIkConfig config = loadConfig(options.config_path);
  const std::vector<BenchmarkSample> dataset =
      generateBenchmarkDataset(options.model_path, config, options.samples, options.seed);
  const std::uint64_t dataset_hash = hashBenchmarkDataset(dataset);
  std::cout << "dataset_samples=" << dataset.size() << " seed=" << options.seed
            << " hash=0x" << std::hex << dataset_hash << std::dec << '\n';

  BackendResult osqp = benchmarkProblems(SolverBackend::kOsqp, config, dataset);
  BackendResult qpoases = benchmarkProblems(SolverBackend::kQpoases, config, dataset);
  const std::size_t cycle_samples = std::max<std::size_t>(5000U, options.samples);
  osqp.full_cycle_us = benchmarkFullCycle(SolverBackend::kOsqp, config, options.model_path,
                                          cycle_samples, osqp.cycle_failures);
  qpoases.full_cycle_us = benchmarkFullCycle(SolverBackend::kQpoases, config, options.model_path,
                                             cycle_samples, qpoases.cycle_failures);
  const std::size_t single_arm_samples_per_side =
      std::max<std::size_t>(2500U, options.samples / 2U);
  for (BackendResult* result : {&osqp, &qpoases}) {
    std::vector<double> left = benchmarkSingleArmCycle(
        result->backend, ArmSide::kLeft, config, options.model_path,
        single_arm_samples_per_side, result->single_arm_failures);
    std::vector<double> right = benchmarkSingleArmCycle(
        result->backend, ArmSide::kRight, config, options.model_path,
        single_arm_samples_per_side, result->single_arm_failures);
    result->single_arm_cycle_us.reserve(left.size() + right.size());
    result->single_arm_cycle_us.insert(result->single_arm_cycle_us.end(), left.begin(),
                                       left.end());
    result->single_arm_cycle_us.insert(result->single_arm_cycle_us.end(), right.begin(),
                                       right.end());
  }

  double maximum_solution_difference = 0.0;
  double maximum_objective_difference = 0.0;
  for (std::size_t index = 0; index < dataset.size(); ++index) {
    for (std::size_t arm = 0; arm < 2U; ++arm) {
      const QpProblem7& problem = arm == 0U ? dataset[index].left : dataset[index].right;
      maximum_solution_difference =
          std::max(maximum_solution_difference,
                   (osqp.solutions[index][arm] - qpoases.solutions[index][arm]).lpNorm<Eigen::Infinity>());
      maximum_objective_difference =
          std::max(maximum_objective_difference,
                   std::abs(objective(problem, osqp.solutions[index][arm]) -
                            objective(problem, qpoases.solutions[index][arm])));
    }
  }

  const std::array<BackendResult*, 2> results{&osqp, &qpoases};
  std::ofstream output(options.output_path);
  if (!output) {
    throw std::runtime_error("cannot open benchmark output: " + options.output_path);
  }
  output << "backend,metric,unit,samples,failures,mean,p50,p95,p99,max\n";
  for (const BackendResult* result : results) {
    const Statistics total = summarize(result->dual_total_us);
    const Statistics update = summarize(result->update_us);
    const Statistics solve = summarize(result->solve_us);
    const Statistics per_arm_solver_total = summarize(result->per_arm_solver_total_us);
    const Statistics single_arm_cycle = summarize(result->single_arm_cycle_us);
    const Statistics iterations = summarize(result->iterations);
    const Statistics initialization = summarize(result->initialization_us);
    const Statistics cycle = summarize(result->full_cycle_us);
    const std::size_t problem_failures = result->solver_failures + result->bound_failures;
    const std::size_t over_budget_samples = static_cast<std::size_t>(std::count_if(
        result->full_cycle_us.begin(), result->full_cycle_us.end(),
        [](double duration_us) { return duration_us >= 1000.0; }));
    printStatistics(toString(result->backend), "dual_qp_total", total);
    printStatistics(toString(result->backend), "per_arm_update", update);
    printStatistics(toString(result->backend), "per_arm_solve", solve);
    printStatistics(toString(result->backend), "per_arm_solver", per_arm_solver_total);
    printStatistics(toString(result->backend), "single_arm_cycle", single_arm_cycle);
    printStatistics(toString(result->backend), "cold_initialize", initialization);
    printStatistics(toString(result->backend), "full_cycle", cycle);
    std::cout << std::left << std::setw(9) << toString(result->backend) << " "
              << std::setw(16) << "iterations" << " mean=" << std::fixed
              << std::setprecision(3) << iterations.mean << " p50=" << iterations.p50
              << " p95=" << iterations.p95 << " p99=" << iterations.p99
              << " max=" << iterations.maximum << '\n';
    std::cout << "  failures: solver=" << result->solver_failures
              << " bounds=" << result->bound_failures
              << " cycle=" << result->cycle_failures
              << " single_arm=" << result->single_arm_failures
              << " over_budget_samples=" << over_budget_samples << '\n';
    std::cout << "  correctness: max_kkt_residual=" << std::scientific
              << result->maximum_kkt_residual
              << " max_bound_violation=" << result->maximum_bound_violation << std::fixed
              << '\n';
    if (problem_failures != 0U) {
      std::cout << "  failure_categories:";
      for (std::size_t category = 0; category < kBenchmarkCategoryCount; ++category) {
        std::cout << ' '
                  << benchmarkCategoryName(static_cast<BenchmarkCategory>(category)) << '='
                  << result->category_solver_failures[category] << '/'
                  << result->category_bound_failures[category];
      }
      std::cout << " (solver/bound)\n";
      std::cout << "  statuses: solved="
                << result->status_counts[static_cast<std::size_t>(SolverStatus::kSolved)]
                << " max_iterations="
                << result->status_counts[static_cast<std::size_t>(SolverStatus::kMaxIterations)]
                << " infeasible="
                << result->status_counts[static_cast<std::size_t>(SolverStatus::kInfeasible)]
                << " numerical="
                << result->status_counts[static_cast<std::size_t>(SolverStatus::kNumericalError)]
                << " invalid="
                << result->status_counts[static_cast<std::size_t>(SolverStatus::kInvalidInput)]
                << '\n';
    }
    writeMetric(output, result->backend, "dual_qp_total", "us", total, dataset.size(),
                problem_failures);
    writeMetric(output, result->backend, "per_arm_update", "us", update,
                2U * dataset.size(),
                problem_failures);
    writeMetric(output, result->backend, "per_arm_solve", "us", solve,
                2U * dataset.size(),
                problem_failures);
    writeMetric(output, result->backend, "per_arm_solver_total", "us",
                per_arm_solver_total,
                2U * dataset.size(), problem_failures);
    writeMetric(output, result->backend, "single_arm_cycle", "us", single_arm_cycle,
                result->single_arm_cycle_us.size(), result->single_arm_failures);
    writeMetric(output, result->backend, "iterations", "count", iterations,
                2U * dataset.size(), problem_failures);
    writeMetric(output, result->backend, "cold_initialize", "us", initialization,
                result->initialization_us.size(), 0U);
    writeMetric(output, result->backend, "full_cycle", "us", cycle, cycle_samples,
                result->cycle_failures);
    const Statistics kkt{result->maximum_kkt_residual, result->maximum_kkt_residual,
                         result->maximum_kkt_residual, result->maximum_kkt_residual,
                         result->maximum_kkt_residual};
    const Statistics bounds{result->maximum_bound_violation,
                            result->maximum_bound_violation,
                            result->maximum_bound_violation,
                            result->maximum_bound_violation,
                            result->maximum_bound_violation};
    writeMetric(output, result->backend, "max_kkt_residual", "absolute", kkt,
                2U * dataset.size(), problem_failures);
    writeMetric(output, result->backend, "max_bound_violation", "absolute", bounds,
                2U * dataset.size(), problem_failures);
    output << toString(result->backend) << ",over_budget_samples,count," << cycle_samples << ','
           << over_budget_samples << ',' << over_budget_samples << ',' << over_budget_samples
           << ',' << over_budget_samples << ',' << over_budget_samples << ','
           << over_budget_samples << '\n';
  }

  const Statistics osqp_cycle = summarize(osqp.full_cycle_us);
  const Statistics qpoases_cycle = summarize(qpoases.full_cycle_us);
  const Statistics osqp_dual_qp = summarize(osqp.dual_total_us);
  const Statistics qpoases_dual_qp = summarize(qpoases.dual_total_us);
  const bool osqp_correct =
      osqp.solver_failures == 0U && osqp.bound_failures == 0U &&
      osqp.cycle_failures == 0U && osqp.single_arm_failures == 0U &&
      osqp.maximum_kkt_residual <= 1e-5 &&
      osqp.maximum_bound_violation <= config.safety.bound_tolerance;
  const bool qpoases_correct =
      qpoases.solver_failures == 0U && qpoases.bound_failures == 0U &&
      qpoases.cycle_failures == 0U && qpoases.single_arm_failures == 0U &&
      qpoases.maximum_kkt_residual <= 1e-5 &&
      qpoases.maximum_bound_violation <= config.safety.bound_tolerance;
  const bool crosscheck_correct = maximum_solution_difference <= 1e-5 &&
                                  maximum_objective_difference <= 1e-7;
  const bool correct = osqp_correct && qpoases_correct && crosscheck_correct;
  constexpr double kP99TieFraction = 0.01;
  const bool p99_tie =
      std::abs(osqp_dual_qp.p99 - qpoases_dual_qp.p99) <=
      kP99TieFraction * std::min(osqp_dual_qp.p99, qpoases_dual_qp.p99);
  const SolverBackend selected = p99_tie
                                     ? (osqp_dual_qp.mean < qpoases_dual_qp.mean
                                            ? SolverBackend::kOsqp
                                            : SolverBackend::kQpoases)
                                     : (osqp_dual_qp.p99 < qpoases_dual_qp.p99
                                            ? SolverBackend::kOsqp
                                            : SolverBackend::kQpoases);
  std::cout << "crosscheck_max_solution_difference=" << std::setprecision(12)
            << maximum_solution_difference
            << " crosscheck_max_objective_difference=" << maximum_objective_difference << '\n';
  std::cout << "selected_backend=" << (correct ? toString(selected) : "none") << '\n';
  std::cout << "selection_metric=identical_qp_dual_total_p99_us"
            << " selection_p99_tie_band_percent=1 p99_tie=" << p99_tie << '\n';
  std::cout << "csv=" << std::filesystem::absolute(options.output_path).string() << '\n';

  const bool realtime_budget = osqp_cycle.p99 < 1000.0 && qpoases_cycle.p99 < 1000.0;
  if (!correct || !realtime_budget) {
    std::cerr << "benchmark acceptance failed: correct=" << correct
              << " both_p99_under_1000us=" << realtime_budget << '\n';
    return 2;
  }
  return 0;
}

}  // namespace
}  // namespace tianji_qp_ik

int main(int argc, char** argv) {
  try {
    return tianji_qp_ik::run(argc, argv);
  } catch (const std::exception& error) {
    std::cerr << "benchmark error: " << error.what() << '\n';
    return 1;
  }
}
