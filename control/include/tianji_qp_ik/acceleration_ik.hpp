#pragma once

#include "tianji_qp_ik/types.hpp"

#include <string_view>

namespace tianji_qp_ik {

struct JointAccelerationBounds {
  Vec7 lower{Vec7::Zero()};
  Vec7 upper{Vec7::Zero()};
};

struct ArmAccelerationInput {
  Vec7 q_model{Vec7::Zero()};
  Vec7 qdot_model{Vec7::Zero()};
  Vec7 qddot_previous{Vec7::Zero()};
  Mat67 jacobian{Mat67::Zero()};
  Vec6 jdot_qdot{Vec6::Zero()};
  Vec6 desired_acceleration{Vec6::Zero()};
  ArmLimits limits;
  JointAccelerationBounds bounds;
  bool posture_reference_active{false};
  Vec7 posture_reference{Vec7::Zero()};
  Vec7 posture_velocity_reference{Vec7::Zero()};
  Vec7 posture_acceleration_reference{Vec7::Zero()};
  ScalarJointTask arm_angle_task;
  LinearJointConstraint linear_constraint;
  double dt{0.005};
};

struct ArmAccelerationResult {
  SolverStatus status{SolverStatus::kInvalidInput};
  Vec7 qddot{Vec7::Zero()};
  Vec6 slack{Vec6::Zero()};
  double equality_residual{0.0};
  double solve_time_us{0.0};
  int iterations{0};
  int active_bound_count{0};
  double qddot_max_ratio{0.0};
  double task_scale_position{1.0};
  double task_scale_orientation{1.0};
  std::string_view detail{"not_initialized"};
};

}  // namespace tianji_qp_ik
