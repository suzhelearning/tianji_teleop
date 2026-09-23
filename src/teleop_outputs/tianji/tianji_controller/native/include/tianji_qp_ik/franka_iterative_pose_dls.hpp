#pragma once

#include "tianji_qp_ik/iterative_pose_dls.hpp"
#include "tianji_qp_ik/mujoco_robot.hpp"

#include <functional>
#include <string_view>

namespace tianji_qp_ik {

struct FrankaPoseDlsInput {
  Pose target;
  Vec7 seed{Vec7::Zero()};
  ArmLimits limits;
  bool fixed_posture_reference_valid{false};
  Vec7 fixed_posture_reference{Vec7::Zero()};
  ScalarJointTask secondary_task;
  KinematicsEvaluator evaluate;
  KinematicsFeasibility candidate_feasible;
  // If true, publish the last valid iterate.  The default publishes the
  // lowest-merit feasible candidate evaluated during this solve.
  bool return_final_iterate{false};
  // Optional search-only correction. Exact feasibility still gates candidates.
  std::function<Vec7(const Vec7&, const ArmKinematicSample&, double,
                    const Vec7&)> constrain_step;
};

Vec7 frankaClampPostureReferenceToInterior(const Vec7& requested,
                                     const ArmLimits& limits,
                                     double margin_rad) noexcept;

class FrankaIterativePoseDlsIk7 {
 public:
  FrankaIterativePoseDlsIk7(IterativeDlsConfig config, double margin_rad);
  PoseDlsResult solve(const FrankaPoseDlsInput& input) const;

 private:
  IterativeDlsConfig config_;
  double margin_rad_{0.0};
};

}  // namespace tianji_qp_ik
