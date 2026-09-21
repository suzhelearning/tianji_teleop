#pragma once
#include "tianji_qp_ik/iterative_pose_dls.hpp"
#include "tianji_qp_ik/velocity_ik.hpp"
namespace tianji_qp_ik {
struct PicoEeFrankaDlsInput {
  Pose target;
  bool target_valid{false};
  bool target_stale{false};
  Vec7 seed{Vec7::Zero()};
  Vec7 seed_velocity{Vec7::Zero()};
  Vec7 seed_acceleration{Vec7::Zero()};
  ArmLimits limits;
  JointVelocityBounds velocity_bounds;
  Vec7 home_reference{Vec7::Zero()};
  KinematicsEvaluator evaluate;
  double dt{0.005};
};

struct PicoEeFrankaDlsResult {
  bool accepted{false};
  bool target_held{false};
  bool planner_accepted{false};
  PoseDlsResult dls;
  ArmMotionState planner_state;
  Vec7 planner_jerk{Vec7::Zero()};
  Vec7 goal{Vec7::Zero()};
  Vec7 planner_target{Vec7::Zero()};
  Vec7 qdot{Vec7::Zero()};
  double velocity_ratio{0.0};
  double acceleration_ratio{0.0};
  double jerk_ratio{0.0};
  std::string_view detail{"not_solved"};
};


}
