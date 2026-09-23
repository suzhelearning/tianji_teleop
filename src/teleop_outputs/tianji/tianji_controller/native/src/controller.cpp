#include "tianji_qp_ik/controller.hpp"
#include "tianji_qp_ik/so3.hpp"
#include <cmath>
#include <utility>
namespace tianji_qp_ik {
DualArmController::DualArmController(MujocoRobot& robot, QpIkConfig config)
    : robot_(robot), config_(std::move(config)) {
  left_state_.q_ref = robot_.armPosition(ArmSide::kLeft);
  right_state_.q_ref = robot_.armPosition(ArmSide::kRight);
  initializeDls();
}
ControllerDiagnostics DualArmController::step(const DualArmTargets& targets, double dt) {
  return stepDls(targets, nullptr, dt);
}
ControllerDiagnostics DualArmController::step(
    const DualArmReferences& references, double dt) {
  DualArmTargets targets;
  targets.left = references.left.pose;
  targets.right = references.right.pose;
  targets.left_twist = references.left.twist;
  targets.right_twist = references.right.twist;
  targets.left_stale = references.left.stale;
  targets.right_stale = references.right.stale;
  return stepDls(targets, &references, dt);
}
void DualArmController::resetSolvers() {
  left_dls_.valid = right_dls_.valid = false;
}
bool DualArmController::synchronizeReferencesToActual() {
  // DLS owns model references; feedback never overwrites the command model.
  clearHistory();
  return true;
}
const Vec7& DualArmController::reference(ArmSide side) const noexcept {
  return state(side).q_ref;
}

const Vec7& DualArmController::previousVelocity(ArmSide side) const noexcept {
  return state(side).qdot_prev;
}

const Vec7& DualArmController::previousAcceleration(
    ArmSide side) const noexcept {
  return state(side).qddot_prev;
}

ArmMotionState DualArmController::referenceState(
    ArmSide side) const noexcept {
  const ArmReferenceState& arm_state = state(side);
  return {arm_state.q_ref, arm_state.qdot_prev, arm_state.qddot_prev};
}

bool DualArmController::setReferenceState(
    ArmSide side, const ArmMotionState& motion) {
  left_dls_.valid = right_dls_.valid = false;
  const ArmLimits& limits = robot_.mapping(side).limits;
  if (!motion.q.allFinite() || !motion.qdot.allFinite() ||
      !motion.qddot.allFinite()) {
    return false;
  }
  for (int joint = 0; joint < kArmDof; ++joint) {
    if (motion.q[joint] <
            limits.lower_position[joint] - config_.safety.bound_tolerance ||
        motion.q[joint] >
            limits.upper_position[joint] + config_.safety.bound_tolerance ||
        std::abs(motion.qdot[joint]) >
            limits.velocity[joint] + config_.safety.bound_tolerance ||
        std::abs(motion.qddot[joint]) >
            config_.joint_limits.max_acceleration_rad_s2[joint] +
                config_.safety.bound_tolerance) {
      return false;
    }
  }
  ArmReferenceState& arm_state = state(side);
  arm_state.q_ref = motion.q;
  arm_state.qdot_prev = motion.qdot;
  arm_state.qddot_prev = motion.qddot;
  robot_.setArmState(side, motion.q, motion.qdot);
  return true;
}

ArmReferenceState& DualArmController::state(ArmSide side) noexcept {
  return side == ArmSide::kLeft ? left_state_ : right_state_;
}

const ArmReferenceState& DualArmController::state(ArmSide side) const noexcept {
  return side == ArmSide::kLeft ? left_state_ : right_state_;
}

void DualArmController::clearHistory() {
  clearHistory(ArmSide::kLeft);
  clearHistory(ArmSide::kRight);
}

void DualArmController::clearHistory(ArmSide side) {
  (side == ArmSide::kLeft ? left_dls_ : right_dls_).valid = false;
  ArmReferenceState& arm_state = state(side);
  arm_state.qdot_prev.setZero();
  arm_state.qddot_prev.setZero();
  if (arm_state.q_ref.allFinite()) {
    robot_.setArmState(side, arm_state.q_ref, Vec7::Zero());
  }
}

bool DualArmController::targetsAreFinite(const DualArmTargets& targets) const {
  return targets.left.position.allFinite() &&
         targets.left.rotation.allFinite() &&
         targets.left_twist.allFinite() &&
         targets.right.position.allFinite() &&
         targets.right.rotation.allFinite() &&
         targets.right_twist.allFinite() &&
         isProperRotation(targets.left.rotation) &&
         isProperRotation(targets.right.rotation);
}

bool DualArmController::referenceIsFinite(
    const CartesianReference& reference) const {
  return reference.valid && reference.pose.position.allFinite() &&
         reference.pose.rotation.allFinite() && reference.twist.allFinite() &&
         reference.acceleration.allFinite() &&
         isProperRotation(reference.pose.rotation);
}

}  // namespace tianji_qp_ik
