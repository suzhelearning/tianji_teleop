#pragma once

#include "tianji_qp_ik/config.hpp"
#include "tianji_qp_ik/equality_constrained_qpoases_solver.hpp"
#include "tianji_qp_ik/pinocchio_arm_kinematics.hpp"
#include "tianji_qp_ik/spark_upper_retarget.hpp"
#include "tianji_qp_ik/qpoases_solver.hpp"

#include <chrono>
#include <optional>
#include <string_view>

namespace tianji_qp_ik {

enum class SparkQpoasesStage { kStage1, kStage2 };

struct SparkDirectionTaskLinearization {
  Eigen::Vector3d error{Eigen::Vector3d::Zero()};
  Mat37 jacobian{Mat37::Zero()};
};

std::optional<SparkDirectionTaskLinearization> linearizeSparkDirectionTask(
    const Eigen::Vector3d& target_direction,
    const Eigen::Vector3d& segment_start,
    const Eigen::Vector3d& segment_end,
    const Mat37& start_jacobian,
    const Mat37& end_jacobian) noexcept;

struct SparkUpperIkResult {
  bool accepted{false};
  SolverStatus status{SolverStatus::kInvalidInput};
  Vec7 q{Vec7::Zero()};
  Vec7 stage1_q{Vec7::Zero()};
  int stage1_iterations{0};
  int stage2_iterations{0};
  double stage1_error{0.0};
  double stage2_error{0.0};
  double stage1_solve_time_us{0.0};
  double stage2_solve_time_us{0.0};
  bool stage1_hotstart{false};
  bool stage2_hotstart{false};
  bool budget_exhausted{false};
  int stage1_backtracking_steps{0};
  int stage2_backtracking_steps{0};
  double upper_direction_error{0.0};
  double forearm_direction_error{0.0};
  double palm_position_error{0.0};
  double palm_orientation_error{0.0};
  std::string_view detail{"not_solved"};
};

class SparkUpperQpoasesIk7 {
 public:
  SparkUpperQpoasesIk7(ArmSide side,
                       PinocchioArmKinematics& kinematics,
                       ArmLimits limits,
                       SparkUpperQpoasesConfig config,
                       QpoasesConfig solver_config = {});

  SparkUpperIkResult solve(const SparkUpperArmTarget& target,
                           const Vec7& seed,
                           std::chrono::steady_clock::time_point deadline =
                               std::chrono::steady_clock::time_point::max());
  SparkUpperIkResult solveOtgConsistent(
      const SparkUpperArmTarget& shape_target, const Pose& otg_pose,
      const Vec7& seed,
      std::chrono::steady_clock::time_point deadline =
          std::chrono::steady_clock::time_point::max());
  void reset() noexcept;

 private:
  struct StageResult {
    bool accepted{false};
    SolverStatus status{SolverStatus::kInvalidInput};
    Vec7 q{Vec7::Zero()};
    int iterations{0};
    double error{0.0};
    double solve_time_us{0.0};
    bool hotstart{false};
    bool budget_exhausted{false};
    int backtracking_steps{0};
    double upper_direction_error{0.0};
    double forearm_direction_error{0.0};
    double palm_position_error{0.0};
    double palm_orientation_error{0.0};
    std::string_view detail{"not_solved"};
  };

  struct StageMetrics {
    double weighted_error{0.0};
    double palm_primary_error{0.0};
    double upper_direction_error{0.0};
    double forearm_direction_error{0.0};
    double palm_position_error{0.0};
    double palm_orientation_error{0.0};
  };

  StageResult solveStage(SparkQpoasesStage stage,
                         const SparkUpperArmTarget& target,
                         const Vec7& seed,
                         std::chrono::steady_clock::time_point deadline);
  bool buildProblem(SparkQpoasesStage stage,
                    const SparkUpperArmTarget& target, const Vec7& q,
                    QpProblem7* problem, StageMetrics* metrics);
  SolverResult7 solveQp(SparkQpoasesStage stage, const QpProblem7& problem,
                        bool* used_hotstart);
  QpoasesSolver7& solver(SparkQpoasesStage stage) noexcept;
  bool& initialized(SparkQpoasesStage stage) noexcept;
  SolverResult7 solveOtgStage1Qp(const QpProblem7& problem);

  ArmSide side_;
  PinocchioArmKinematics& kinematics_;
  SparkUpperQpoasesConfig config_;
  ArmLimits limits_;
  QpoasesSolver7 stage1_solver_;
  QpoasesSolver7 stage2_solver_;
  bool stage1_initialized_{false};
  bool stage2_initialized_{false};
  QpoasesSolver7 otg_stage1_solver_;
  EqualityConstrainedQpoasesSolver7 otg_stage2_solver_;
  bool otg_stage1_initialized_{false};
};

}  // namespace tianji_qp_ik
