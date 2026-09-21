#pragma once
#include "tianji_v131/mujoco_robot.hpp"
#include <functional>
namespace tianji_v131 {
// Callback facade for the original singularity-gradient Pinocchio evaluator.
struct PinocchioArmKinematics {
  std::function<ArmKinematicSample(const Vec7&)> evaluate;
  ArmKinematicSample sampleTcp(ArmSide, const Vec7& q) { return evaluate(q); }
};
}
