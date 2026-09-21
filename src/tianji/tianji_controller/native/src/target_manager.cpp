#include "tianji_qp_ik/target_manager.hpp"

#include "tianji_qp_ik/so3.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace tianji_qp_ik {

DualArmReferences directReferences(const DualArmTargets& targets) {
  DualArmReferences references;
  references.left.pose = targets.left;
  references.left.twist = targets.left_twist;
  references.left.acceleration.setZero();
  references.left.stale = targets.left_stale;
  references.right.pose = targets.right;
  references.right.twist = targets.right_twist;
  references.right.acceleration.setZero();
  references.right.stale = targets.right_stale;
  return references;
}
namespace {

constexpr double kTwoPi = 6.283185307179586476925286766559;

bool isValidPose(const Pose& pose) {
  return pose.position.allFinite() && isProperRotation(pose.rotation);
}

}  // namespace

TargetManager::TargetManager(QpIkConfig config, DualArmTargets initial_targets)
    : config_(config),
      initial_(initial_targets),
      manual_(initial_targets),
      hold_(initial_targets),
      last_output_(initial_targets) {}

void TargetManager::setMode(TargetMode mode, double start_time_seconds) {
  if (mode == mode_) {
    return;
  }
  if (mode == TargetMode::kHold) {
    hold_ = last_output_;
  } else if (mode == TargetMode::kManual) {
    manual_ = last_output_;
  }
  mode_ = mode;
  mode_start_time_seconds_ = start_time_seconds;
  filtered_left_twist_.setZero();
  filtered_right_twist_.setZero();
  left_manual_state_ = ManualTargetState{};
  right_manual_state_ = ManualTargetState{};
  has_sample_time_ = false;
}

void TargetManager::setManualTarget(ArmSide side, const Pose& requested) {
  Pose& target = side == ArmSide::kLeft ? manual_.left : manual_.right;
  if (!isValidPose(requested) || !isValidPose(target)) {
    target = requested;
    return;
  }
  target = limitManualIncrement(target, requested);
  (side == ArmSide::kLeft ? left_manual_state_ : right_manual_state_) =
      ManualTargetState{};
}

bool TargetManager::setManualTarget(ArmSide side, const Pose& requested,
                                    double source_timestamp_seconds,
                                    double receive_time_seconds) {
  if (!std::isfinite(source_timestamp_seconds) ||
      !std::isfinite(receive_time_seconds)) {
    return false;
  }
  Pose& target = side == ArmSide::kLeft ? manual_.left : manual_.right;
  ManualTargetState& state =
      side == ArmSide::kLeft ? left_manual_state_ : right_manual_state_;
  if (state.has_frame &&
      source_timestamp_seconds <= state.source_timestamp_seconds) {
    return false;
  }
  if (!isValidPose(requested) || !isValidPose(target)) {
    target = requested;
    state = ManualTargetState{};
    return true;
  }

  const Pose accepted = limitManualIncrement(target, requested);
  if (state.has_frame) {
    const double dt = source_timestamp_seconds - state.source_timestamp_seconds;
    const Vec6 raw_twist = poseErrorWorld(accepted, target) / dt;
    updateManualKinematics(state, raw_twist, dt);
  } else {
    state.filtered_twist.setZero();
    state.filtered_acceleration.setZero();
  }
  target = accepted;
  state.source_timestamp_seconds = source_timestamp_seconds;
  state.receive_time_seconds = receive_time_seconds;
  state.has_frame = true;
  return true;
}

bool TargetManager::setManualTargets(const Pose& left_requested,
                                     const Pose& right_requested,
                                     double source_timestamp_seconds,
                                     double receive_time_seconds) {
  if (!std::isfinite(source_timestamp_seconds) ||
      !std::isfinite(receive_time_seconds) ||
      !isValidPose(left_requested) || !isValidPose(right_requested)) {
    return false;
  }

  DualArmTargets manual_candidate = manual_;
  ManualTargetState left_state_candidate = left_manual_state_;
  ManualTargetState right_state_candidate = right_manual_state_;
  if ((left_state_candidate.has_frame &&
       source_timestamp_seconds <= left_state_candidate.source_timestamp_seconds) ||
      (right_state_candidate.has_frame &&
       source_timestamp_seconds <= right_state_candidate.source_timestamp_seconds)) {
    return false;
  }

  const bool paired_history =
      left_state_candidate.has_frame && right_state_candidate.has_frame &&
      left_state_candidate.source_timestamp_seconds ==
          right_state_candidate.source_timestamp_seconds &&
      isValidPose(manual_candidate.left) && isValidPose(manual_candidate.right);
  if (paired_history) {
    const double dt =
        source_timestamp_seconds - left_state_candidate.source_timestamp_seconds;
    const Vec6 raw_left = poseErrorWorld(left_requested, manual_candidate.left) / dt;
    const Vec6 raw_right = poseErrorWorld(right_requested, manual_candidate.right) / dt;
    updateManualKinematics(left_state_candidate, raw_left, dt);
    updateManualKinematics(right_state_candidate, raw_right, dt);
  } else {
    left_state_candidate.filtered_twist.setZero();
    right_state_candidate.filtered_twist.setZero();
    left_state_candidate.filtered_acceleration.setZero();
    right_state_candidate.filtered_acceleration.setZero();
  }

  manual_candidate.left = left_requested;
  manual_candidate.right = right_requested;
  left_state_candidate.source_timestamp_seconds = source_timestamp_seconds;
  right_state_candidate.source_timestamp_seconds = source_timestamp_seconds;
  left_state_candidate.receive_time_seconds = receive_time_seconds;
  right_state_candidate.receive_time_seconds = receive_time_seconds;
  left_state_candidate.has_frame = true;
  right_state_candidate.has_frame = true;

  manual_ = manual_candidate;
  left_manual_state_ = left_state_candidate;
  right_manual_state_ = right_state_candidate;
  return true;
}

DualArmTargets TargetManager::sample(double time_seconds) {
  DualArmTargets requested;
  bool left_manual_stale = false;
  bool right_manual_stale = false;
  if (mode_ == TargetMode::kManual) {
    left_manual_stale =
        manualTargetStale(left_manual_state_, time_seconds);
    right_manual_stale =
        manualTargetStale(right_manual_state_, time_seconds);
    const auto staleTarget = [this](const Pose& published,
                                    const Pose& received) {
      const Pose one_step = limitManualIncrement(published, received);
      const bool fully_publishable =
          (one_step.position - received.position).norm() <= 1.0e-12 &&
          rotationDistance(one_step.rotation, received.rotation) <= 1.0e-12;
      return fully_publishable ? received : published;
    };
    requested.left = left_manual_stale
                         ? staleTarget(last_output_.left, manual_.left)
                         : predictedManualPose(
                               manual_.left, left_manual_state_, time_seconds);
    requested.right = right_manual_stale
                          ? staleTarget(last_output_.right, manual_.right)
                          : predictedManualPose(
                                manual_.right, right_manual_state_,
                                time_seconds);
  } else if (mode_ == TargetMode::kHold) {
    requested = hold_;
  } else {
    const double elapsed = std::max(0.0, time_seconds - mode_start_time_seconds_);
    requested = {scriptedPose(initial_.left, elapsed), scriptedPose(initial_.right, elapsed)};
  }
  if (!isValidPose(requested.left) || !isValidPose(requested.right)) {
    return requested;
  }
  DualArmTargets output;
  output.left = limitManualIncrement(last_output_.left, requested.left);
  output.right = limitManualIncrement(last_output_.right, requested.right);
  if (mode_ == TargetMode::kManual) {
    filtered_left_twist_ = manualTwist(left_manual_state_, time_seconds);
    filtered_right_twist_ = manualTwist(right_manual_state_, time_seconds);
  } else if (has_sample_time_ && std::isfinite(time_seconds) &&
      time_seconds > last_sample_time_seconds_) {
    const double dt = time_seconds - last_sample_time_seconds_;
    const double cutoff = config_.cartesian_servo.feedforward_filter_cutoff_hz;
    const double alpha = 1.0 - std::exp(-kTwoPi * cutoff * dt);
    const Vec6 raw_left = poseErrorWorld(output.left, last_output_.left) / dt;
    const Vec6 raw_right = poseErrorWorld(output.right, last_output_.right) / dt;
    filtered_left_twist_ += alpha * (raw_left - filtered_left_twist_);
    filtered_right_twist_ += alpha * (raw_right - filtered_right_twist_);
  } else {
    filtered_left_twist_.setZero();
    filtered_right_twist_.setZero();
  }
  if (mode_ == TargetMode::kHold) {
    filtered_left_twist_.setZero();
    filtered_right_twist_.setZero();
  }
  output.left_twist = filtered_left_twist_;
  output.right_twist = filtered_right_twist_;
  if (mode_ == TargetMode::kManual) {
    output.left_stale = left_manual_stale;
    output.right_stale = right_manual_stale;
  }
  last_output_ = output;
  last_sample_time_seconds_ = time_seconds;
  has_sample_time_ = std::isfinite(time_seconds);
  return last_output_;
}

Pose TargetManager::predictedManualPose(const Pose& target,
                                        const ManualTargetState& state,
                                        double time_seconds) const {
  const bool second_order_otg_prediction =
      config_.cartesian_otg.enabled &&
      !config_.cartesian_otg.translation_position_mode &&
      config_.cartesian_otg.translation_prediction_enabled;
  if (config_.cartesian_otg.enabled && !second_order_otg_prediction) {
    return target;
  }
  if (!second_order_otg_prediction &&
      config_.cartesian_servo.kff_linear <= 0.0 &&
      config_.cartesian_servo.kff_angular <= 0.0) {
    return target;
  }
  if (!state.has_frame || !std::isfinite(time_seconds)) {
    return target;
  }
  const double age = std::max(0.0, time_seconds - state.receive_time_seconds);
  if (age >= config_.cartesian_servo.target_timeout_seconds) {
    return target;
  }
  const double horizon = std::min(
      age, second_order_otg_prediction
               ? config_.cartesian_otg.translation_prediction_horizon_seconds
               : config_.cartesian_servo.prediction_horizon_seconds);
  if (horizon <= 0.0) {
    return target;
  }
  Pose predicted = target;
  predicted.position += state.filtered_twist.head<3>() * horizon;
  if (second_order_otg_prediction) {
    predicted.position +=
        0.5 * state.filtered_acceleration.head<3>() * horizon * horizon;
    return predicted;
  }
  const Eigen::Vector3d rotation_vector =
      state.filtered_twist.tail<3>() * horizon;
  if (rotation_vector.norm() > 0.0) {
    predicted.rotation =
        Eigen::AngleAxisd(rotation_vector.norm(), rotation_vector.normalized())
            .toRotationMatrix() *
        target.rotation;
  }
  return predicted;
}

Vec6 TargetManager::manualTwist(const ManualTargetState& state,
                                double time_seconds) const {
  if (!config_.cartesian_otg.enabled &&
      config_.cartesian_servo.kff_linear <= 0.0 &&
      config_.cartesian_servo.kff_angular <= 0.0) {
    return Vec6::Zero();
  }
  if (!state.has_frame || !std::isfinite(time_seconds)) {
    return Vec6::Zero();
  }
  const double age = time_seconds - state.receive_time_seconds;
  const bool second_order_otg_prediction =
      config_.cartesian_otg.enabled &&
      !config_.cartesian_otg.translation_position_mode &&
      config_.cartesian_otg.translation_prediction_enabled;
  if (age < 0.0 || age >= config_.cartesian_servo.target_timeout_seconds) {
    return Vec6::Zero();
  }
  Vec6 predicted_twist = state.filtered_twist;
  if (second_order_otg_prediction) {
    const double horizon = std::min(
        age, config_.cartesian_otg.translation_prediction_horizon_seconds);
    predicted_twist.head<3>() +=
        state.filtered_acceleration.head<3>() * horizon;
  }
  return predicted_twist;
}

void TargetManager::updateManualKinematics(ManualTargetState& state,
                                           const Vec6& raw_twist,
                                           double dt_seconds) const {
  const double cutoff = config_.cartesian_servo.feedforward_filter_cutoff_hz;
  const double alpha = 1.0 - std::exp(-kTwoPi * cutoff * dt_seconds);
  const Vec6 previous_twist = state.filtered_twist;
  state.filtered_twist += alpha * (raw_twist - state.filtered_twist);
  const Vec6 raw_acceleration =
      (state.filtered_twist - previous_twist) / dt_seconds;
  state.filtered_acceleration +=
      alpha * (raw_acceleration - state.filtered_acceleration);
  const double linear_acceleration_norm =
      state.filtered_acceleration.head<3>().norm();
  if (linear_acceleration_norm >
      config_.cartesian_otg.translation_acceleration_max) {
    state.filtered_acceleration.head<3>() *=
        config_.cartesian_otg.translation_acceleration_max /
        linear_acceleration_norm;
  }
  state.filtered_acceleration.tail<3>().setZero();
}

bool TargetManager::manualTargetStale(const ManualTargetState& state,
                                      double time_seconds) const {
  return state.has_frame && std::isfinite(time_seconds) &&
         time_seconds - state.receive_time_seconds >=
             config_.cartesian_servo.target_timeout_seconds;
}

Pose TargetManager::scriptedPose(const Pose& initial, double elapsed_seconds) const {
  Pose result = initial;
  const double phase = kTwoPi * config_.trajectories.frequency_hz * elapsed_seconds;
  if (mode_ == TargetMode::kCircle || mode_ == TargetMode::kCombined) {
    result.position.x() += config_.trajectories.circle_radius * (std::cos(phase) - 1.0);
    result.position.z() += config_.trajectories.circle_radius * std::sin(phase);
  } else if (mode_ == TargetMode::kFigureEight) {
    result.position.x() += config_.trajectories.figure_eight_width * std::sin(phase);
    result.position.z() += config_.trajectories.figure_eight_height * std::sin(2.0 * phase);
  }
  if (mode_ == TargetMode::kOrientationOnly || mode_ == TargetMode::kCombined) {
    const double angle = config_.trajectories.angular_amplitude * std::sin(phase);
    result.rotation = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                      initial.rotation;
  }
  return result;
}

Pose TargetManager::limitManualIncrement(const Pose& current, const Pose& requested) const {
  Pose result = requested;
  Eigen::Vector3d translation = requested.position - current.position;
  if (translation.norm() > config_.safety.max_target_position_step) {
    translation *= config_.safety.max_target_position_step / translation.norm();
  }
  result.position = current.position + translation;

  Eigen::Vector3d rotation_vector = so3Log(requested.rotation * current.rotation.transpose());
  if (rotation_vector.norm() > config_.safety.max_target_orientation_step) {
    rotation_vector *= config_.safety.max_target_orientation_step / rotation_vector.norm();
  }
  if (rotation_vector.norm() > 0.0) {
    result.rotation =
        Eigen::AngleAxisd(rotation_vector.norm(), rotation_vector.normalized()).toRotationMatrix() *
        current.rotation;
  } else {
    result.rotation = current.rotation;
  }
  return result;
}

std::string toString(TargetMode mode) {
  switch (mode) {
    case TargetMode::kManual:
      return "manual";
    case TargetMode::kHold:
      return "hold";
    case TargetMode::kCircle:
      return "circle";
    case TargetMode::kFigureEight:
      return "figure_eight";
    case TargetMode::kOrientationOnly:
      return "orientation";
    case TargetMode::kCombined:
      return "combined";
  }
  return "unknown";
}

}  // namespace tianji_qp_ik
