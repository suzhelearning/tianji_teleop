#pragma once

#include <Eigen/Core>


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

enum class SolverStatus {
  kSolved,
  kMaxIterations,
  kInfeasible,
  kNumericalError,
  kInvalidInput,
};

}  // namespace tianji_qp_ik
