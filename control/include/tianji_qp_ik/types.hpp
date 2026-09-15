#pragma once

#include <Eigen/Core>

#include <limits>
#include <string_view>

namespace tianji_qp_ik {

inline constexpr int kArmDof = 7;
inline constexpr int kHandDof = 20;

enum class ArmSide { kLeft, kRight };

using Vec6 = Eigen::Matrix<double, 6, 1>;
using Vec7 = Eigen::Matrix<double, 7, 1>;
using Vec20 = Eigen::Matrix<double, 20, 1>;
using Mat37 = Eigen::Matrix<double, 3, 7>;
using Mat67 = Eigen::Matrix<double, 6, 7>;
using Mat77 = Eigen::Matrix<double, 7, 7>;

struct Pose {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
};

struct ArmLimits {
  Vec7 lower_position{Vec7::Zero()};
  Vec7 upper_position{Vec7::Zero()};
  Vec7 velocity{Vec7::Zero()};
};

struct ArmMotionState {
  Vec7 q{Vec7::Zero()};
  Vec7 qdot{Vec7::Zero()};
  Vec7 qddot{Vec7::Zero()};
};

struct ScalarJointTask {
  bool active{false};
  bool nullspace_only{false};
  Vec7 jacobian{Vec7::Zero()};
  double target{0.0};
  double activation{0.0};
  double weight_scale{1.0};
};

struct JointPositionGuard {
  bool active{false};
  Vec7 position{Vec7::Zero()};
  Vec7 lower{Vec7::Zero()};
  Vec7 upper{Vec7::Zero()};
  double soft_margin_rad{0.0};
  // Null-space recovery speed toward the center of the safe joint range.
  // This does not alter the Cartesian primary or any hard joint bound.
  double recovery_gain_rad_s_per_rad{0.0};
};

struct LinearJointConstraint {
  bool active{false};
  Vec7 jacobian{Vec7::Zero()};
  double lower{-std::numeric_limits<double>::infinity()};
  double upper{std::numeric_limits<double>::infinity()};
  double requested_lower{-std::numeric_limits<double>::infinity()};
  double requested_upper{std::numeric_limits<double>::infinity()};
  bool feasibility_clipped{false};
  bool viability_clipped{false};
};

struct QpProblem7 {
  Mat77 H{Mat77::Identity()};
  Vec7 g{Vec7::Zero()};
  Vec7 lower{Vec7::Zero()};
  Vec7 upper{Vec7::Zero()};
};

enum class SolverStatus {
  kSolved,
  kMaxIterations,
  kInfeasible,
  kNumericalError,
  kInvalidInput,
};

struct SolverResult7 {
  SolverStatus status{SolverStatus::kInvalidInput};
  Vec7 qdot{Vec7::Zero()};
  int iterations{0};
  int native_status_code{0};
  double update_time_us{0.0};
  double solve_time_us{0.0};
  std::string_view detail{"not_initialized"};
};

}  // namespace tianji_qp_ik
