#pragma once

#include "tianji_mapped_palm/types.hpp"

namespace tianji_mapped_palm {

enum class PicoMappedVectorDisableReason {
  kNone,
  kInactive,
  kInvalidDt,
  kNonFiniteInput,
  kInvalidSvd,
  kLowObservability,
  kCartesianLeakage,
  kInfeasibleScalarInterval,
  kStaleInput,
  kEpochReset,
};

struct PicoMappedArmAngleVectorGovernorConfig {
  double observability_min{1.0e-3};
  double maximum_cartesian_leakage{1.0e-8};
  double maximum_velocity_rad_s{2.5};
  double maximum_acceleration_rad_s2{8.0};
  double maximum_jerk_rad_s3{80.0};
  double reference_weight{3.0};
  double continuity_weight{0.1};
  double acceleration_weight{5.0e-5};
  double jerk_weight{5.0e-10};
  double basis_rotation_full_health_rad_s{2.0};
  double basis_rotation_zero_health_rad_s{8.0};
  // Optional transport of the previous achieved nullspace coefficient into
  // the current SVD basis. Disabled by default to preserve the qualified
  // mapped-EE behavior while the candidate is evaluated.
  bool basis_transport_enabled{false};
  double basis_transport_weight{1.0};
  // Hold a command at zero while it crosses the sign deadband. This is an
  // upstream reference shaping aid; it never changes QP constraints.
  double reversal_deadband_rad_s{0.0};
};

struct PicoMappedArmAngleVectorInput {
  Mat67 cartesian_jacobian{Mat67::Zero()};
  Vec7 arm_angle_jacobian{Vec7::Zero()};
  Vec6 desired_twist{Vec6::Zero()};
  Vec7 qdot_previous{Vec7::Zero()};
  Vec7 qddot_previous{Vec7::Zero()};
  Vec7 lower_velocity_bound{Vec7::Zero()};
  Vec7 upper_velocity_bound{Vec7::Zero()};
  double requested_arm_rate{0.0};
  double minimum_predicted_task_scale{1.0};
  double task_health{1.0};
  double dt{0.0};
  bool active{false};
  bool stale{false};
  bool epoch_reset{false};
};

struct PicoMappedArmAngleVectorState {
  bool valid{false};
  bool active{false};
  PicoMappedVectorDisableReason disable_reason{
      PicoMappedVectorDisableReason::kInactive};
  Vec7 basis{Vec7::Zero()};
  Vec7 qdot_task{Vec7::Zero()};
  bool basis_flipped{false};
  double cartesian_leakage{0.0};
  double arm_angle_observability{0.0};
  double basis_rotation_rate_rad_s{0.0};
  double health{0.0};
  double command_tracking_weight{0.0};
  bool exact_prediction_feasible{false};
  double predicted_task_scale{1.0};
  double alpha_raw{0.0};
  double alpha_command{0.0};
  double alpha_previous{0.0};
  double alpha_trend{0.0};
  double alpha_transport{0.0};
  bool basis_transport_applied{false};
  bool reversal_deadband_applied{false};
  double alpha_feasible_lower{0.0};
  double alpha_feasible_upper{0.0};
  double alpha_acceleration_command{0.0};
  double achieved_vector_velocity_norm{0.0};
  double achieved_vector_acceleration_norm{0.0};
  double achieved_vector_jerk_norm{0.0};
};

Vec7 orientPicoMappedNullspaceBasis(const Vec7& candidate,
                                    const Vec7& previous,
                                    bool previous_valid,
                                    bool* flipped) noexcept;

class PicoMappedArmAngleVectorGovernor7 final {
 public:
  explicit PicoMappedArmAngleVectorGovernor7(
      PicoMappedArmAngleVectorGovernorConfig config);

  PicoMappedArmAngleVectorState update(
      const PicoMappedArmAngleVectorInput& input);
  void reset() noexcept;

 private:
  PicoMappedArmAngleVectorState disabled(
      PicoMappedVectorDisableReason reason) noexcept;

  PicoMappedArmAngleVectorGovernorConfig config_;
  Vec7 previous_basis_{Vec7::Zero()};
  bool previous_basis_valid_{false};
  Vec7 previous_achieved_vector_{Vec7::Zero()};
  Vec7 previous_achieved_acceleration_{Vec7::Zero()};
  bool vector_history_valid_{false};
  double alpha_command_{0.0};
  double alpha_acceleration_command_{0.0};
  bool governor_valid_{false};
};

}  // namespace tianji_mapped_palm
