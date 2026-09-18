// Ported from TJ_arm_control_pico_ee_ik; see PORTING.md.
#include "tianji_v131/pico_ee_v131_singularity.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace tianji_v131 {
double picoEeV131SingularityActivation(double sigma_min, double threshold,
                                       double critical) noexcept {
  if (!std::isfinite(sigma_min) || !std::isfinite(threshold) ||
      !std::isfinite(critical) || threshold <= critical) {
    return 0.0;
  }
  return std::clamp((threshold - sigma_min) / (threshold - critical), 0.0,
                    1.0);
}

PicoEeV131SingularityEscape7::PicoEeV131SingularityEscape7(
    PicoEeV131VelocityQpConfig config, PinocchioArmKinematics& kinematics,
    ArmSide side)
    : config_(std::move(config)), kinematics_(kinematics), side_(side) {}

PicoEeV131SingularityState PicoEeV131SingularityEscape7::update(
    const Vec7& q, const PicoEeV131JacobianSvdResult& svd,
    double now_seconds) {
  state_.gradient_refreshed = false;
  state_.gradient_sweep_joint = -1;
  state_.sigma_min = svd.valid
                         ? svd.sigma_min
                         : std::numeric_limits<double>::quiet_NaN();
  state_.activation = picoEeV131SingularityActivation(
      state_.sigma_min, config_.singular_value_threshold,
      config_.singularity_critical_threshold);
  state_.requested_sigma_dot =
      state_.activation * config_.singularity_escape_speed_rad_s;
  if (!std::isfinite(now_seconds) || !q.allFinite() ||
      !std::isfinite(state_.sigma_min) || state_.activation <= 0.0) {
    gradient_sweep_active_ = false;
    gradient_sweep_next_joint_ = 0;
    state_.escape_active = false;
    state_.gradient_valid = false;
    state_.gradient.setZero();
    state_.gradient_age_seconds = 0.0;
    state_.gradient_sweep_active = false;
    state_.gradient_source_distance_rad = 0.0;
    return state_;
  }

  const double refresh_period =
      1.0 / std::max(config_.gradient_update_rate_hz, 1.0e-9);
  const bool refresh_due =
      !std::isfinite(last_gradient_update_seconds_) ||
      now_seconds - last_gradient_update_seconds_ >= refresh_period;
  if (!gradient_sweep_active_ && refresh_due) {
    gradient_sweep_active_ = true;
    gradient_sweep_q_ = q;
    gradient_sweep_components_.setZero();
    gradient_sweep_next_joint_ = 0;
    last_gradient_update_seconds_ = now_seconds;
  }

  if (gradient_sweep_active_) {
    const double h = config_.finite_difference_rad;
    const int joint = gradient_sweep_next_joint_;
    state_.gradient_sweep_joint = joint;
    bool component_valid = std::isfinite(h) && h > 0.0 && joint >= 0 &&
                           joint < kArmDof;
    if (component_valid) {
      Vec7 q_plus = gradient_sweep_q_;
      Vec7 q_minus = gradient_sweep_q_;
      q_plus[joint] += h;
      q_minus[joint] -= h;
      double sigma_plus = std::numeric_limits<double>::quiet_NaN();
      double sigma_minus = std::numeric_limits<double>::quiet_NaN();
      try {
        sigma_plus = picoEeV131MinimumSingularValue(
            kinematics_.sampleTcp(side_, q_plus).tcp_jacobian);
        sigma_minus = picoEeV131MinimumSingularValue(
            kinematics_.sampleTcp(side_, q_minus).tcp_jacobian);
      } catch (...) {
        component_valid = false;
      }
      if (component_valid && std::isfinite(sigma_plus) &&
          std::isfinite(sigma_minus)) {
        gradient_sweep_components_[joint] =
            (sigma_plus - sigma_minus) / (2.0 * h);
      } else {
        component_valid = false;
      }
    }

    if (!component_valid) {
      gradient_sweep_active_ = false;
      gradient_sweep_next_joint_ = 0;
      if (!state_.gradient_valid) {
        state_.gradient.setZero();
        gradient_timestamp_seconds_ =
            -std::numeric_limits<double>::infinity();
      }
    } else {
      ++gradient_sweep_next_joint_;
      if (gradient_sweep_next_joint_ >= kArmDof) {
        gradient_sweep_active_ = false;
        gradient_sweep_next_joint_ = 0;
        state_.gradient_source_distance_rad =
            (q - gradient_sweep_q_).norm();
        const double gradient_norm = gradient_sweep_components_.norm();
        const bool source_valid =
            std::isfinite(state_.gradient_source_distance_rad) &&
            state_.gradient_source_distance_rad <=
                config_.gradient_configuration_tolerance_rad;
        if (source_valid && gradient_sweep_components_.allFinite() &&
            std::isfinite(gradient_norm) && gradient_norm > 1.0e-9) {
          state_.gradient = gradient_sweep_components_ / gradient_norm;
          state_.gradient_valid = true;
          state_.gradient_refreshed = true;
          gradient_timestamp_seconds_ = now_seconds;
        }
      } else {
        state_.gradient_source_distance_rad =
            (q - gradient_sweep_q_).norm();
      }
    }
  }

  state_.gradient_sweep_active = gradient_sweep_active_;

  state_.gradient_age_seconds =
      std::isfinite(gradient_timestamp_seconds_)
          ? std::max(0.0, now_seconds - gradient_timestamp_seconds_)
          : 0.0;
  state_.escape_active =
      state_.gradient_valid &&
      std::isfinite(state_.gradient_age_seconds) &&
      state_.gradient_age_seconds <= config_.gradient_max_age_seconds;
  return state_;
}

void PicoEeV131SingularityEscape7::reset() noexcept {
  state_ = PicoEeV131SingularityState{};
  last_gradient_update_seconds_ =
      -std::numeric_limits<double>::infinity();
  gradient_timestamp_seconds_ = -std::numeric_limits<double>::infinity();
  gradient_sweep_active_ = false;
  gradient_sweep_q_.setZero();
  gradient_sweep_components_.setZero();
  gradient_sweep_next_joint_ = 0;
}

}  // namespace tianji_v131
