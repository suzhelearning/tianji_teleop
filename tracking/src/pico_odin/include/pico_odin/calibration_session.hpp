#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "pico_odin/extrinsic_solver.hpp"

namespace pico_odin {

enum class CalibrationPhase {
  kWaitingForInputs,
  kReady,
  kPendingStart,
  kNeutral,
  kExcitation,
  kFinalNeutral,
  kSolving,
  kSaved,
  kRejected,
  kCancelled,
};

struct CalibrationSessionOptions {
  double neutral_duration_sec{2.0};
  double final_neutral_duration_sec{2.0};
  double attempt_timeout_sec{30.0};
  double min_pitch_range_rad{12.0 * M_PI / 180.0};
  double min_yaw_range_rad{12.0 * M_PI / 180.0};
  double min_directional_excursion_ratio{0.20};
  double return_upright_tolerance_rad{8.0 * M_PI / 180.0};
  double max_initial_tilt_rad{25.0 * M_PI / 180.0};
};

class CalibrationSession {
 public:
  explicit CalibrationSession(CalibrationSessionOptions options = {})
  : options_(options) {}

  void observe_valid_inputs(double now_sec) {
    if (phase_ == CalibrationPhase::kPendingStart) {
      begin_attempt(now_sec);
      return;
    }
    if (phase_ == CalibrationPhase::kWaitingForInputs ||
        phase_ == CalibrationPhase::kCancelled ||
        phase_ == CalibrationPhase::kRejected ||
        phase_ == CalibrationPhase::kSaved) {
      phase_ = CalibrationPhase::kReady;
      message_ = "ready; press PICO controller A (keyboard a is a fallback) to start";
    }
  }

  void observe_invalid_inputs() {
    if (phase_ == CalibrationPhase::kPendingStart) return;
    if (active() || phase_ == CalibrationPhase::kSolving) {
      cancel("PICO or Odin input became stale or invalid");
    } else if (phase_ == CalibrationPhase::kReady) {
      phase_ = CalibrationPhase::kWaitingForInputs;
      message_ = "waiting for fresh PICO and Odin inputs";
    }
  }

  bool start(double now_sec) {
    if (phase_ != CalibrationPhase::kReady) return false;
    begin_attempt(now_sec);
    return true;
  }

  bool request_start() {
    if (phase_ == CalibrationPhase::kPendingStart) return true;
    if (phase_ != CalibrationPhase::kReady) return false;
    clear_samples();
    phase_ = CalibrationPhase::kPendingStart;
    message_ = "waiting for fresh ground-aligned PICO and Odin inputs";
    return true;
  }

  void add_pico(const HostTimedPose & sample, bool stationary, double now_sec) {
    if (!active()) return;
    pico_samples_.push_back(sample);
    if (now_sec - session_started_sec_ > options_.attempt_timeout_sec) {
      reject("calibration attempt timed out");
      return;
    }

    if (phase_ == CalibrationPhase::kNeutral) {
      if (!stationary) {
        phase_started_sec_ = now_sec;
        return;
      }
      if (now_sec - phase_started_sec_ >= options_.neutral_duration_sec) {
        const Eigen::Vector3d up = sample.pose.rotation * Eigen::Vector3d::UnitZ();
        const double tilt = std::acos(std::clamp(up.z(), -1.0, 1.0));
        if (tilt > options_.max_initial_tilt_rad) {
          reject("initial neutral pose is not upright");
          return;
        }
        phase_ = CalibrationPhase::kExcitation;
        neutral_rotation_ = sample.pose.rotation;
        reset_ranges(sample.pose.rotation);
        message_ = "lean forward/back once, then rotate left/right once";
      }
      return;
    }

    if (phase_ == CalibrationPhase::kExcitation) {
      update_ranges(sample.pose.rotation);
      if (stationary && excitation_complete() && returned_upright(sample.pose.rotation)) {
        phase_ = CalibrationPhase::kFinalNeutral;
        phase_started_sec_ = now_sec;
        message_ = "return upright and remain still";
      }
      return;
    }

    if (phase_ == CalibrationPhase::kFinalNeutral) {
      if (!stationary || !returned_upright(sample.pose.rotation)) {
        phase_ = CalibrationPhase::kExcitation;
        message_ = "motion resumed; finish the excitation and return upright";
        return;
      }
      if (now_sec - phase_started_sec_ >= options_.final_neutral_duration_sec) {
        phase_ = CalibrationPhase::kSolving;
        message_ = "solving extrinsics";
      }
    }
  }

  void add_odin(const HostTimedPose & sample) {
    if (active()) odin_samples_.push_back(sample);
  }

  void on_world_reset() { cancel("PICO world reset received during calibration"); }
  void on_odin_restart() { cancel("Odin restart detected during calibration"); }

  void mark_saved(const std::string & message) {
    phase_ = CalibrationPhase::kSaved;
    message_ = message;
  }

  void reject(const std::string & message) {
    phase_ = CalibrationPhase::kRejected;
    message_ = message;
  }

  CalibrationPhase phase() const { return phase_; }
  const std::string & message() const { return message_; }
  const std::vector<HostTimedPose> & pico_samples() const { return pico_samples_; }
  const std::vector<HostTimedPose> & odin_samples() const { return odin_samples_; }
  double pitch_range_rad() const { return max_pitch_ - min_pitch_; }
  double yaw_range_rad() const { return max_yaw_ - min_yaw_; }

 private:
  void begin_attempt(double now_sec) {
    clear_samples();
    phase_ = CalibrationPhase::kNeutral;
    session_started_sec_ = now_sec;
    phase_started_sec_ = now_sec;
    message_ = "stand upright and still";
  }

  bool active() const {
    return phase_ == CalibrationPhase::kNeutral ||
           phase_ == CalibrationPhase::kExcitation ||
           phase_ == CalibrationPhase::kFinalNeutral;
  }

  static std::pair<double, double> pitch_yaw(const Eigen::Quaterniond & rotation) {
    const Eigen::Vector3d forward = rotation * Eigen::Vector3d::UnitX();
    return {
      std::atan2(-forward.z(), std::hypot(forward.x(), forward.y())),
      std::atan2(forward.y(), forward.x())};
  }

  void reset_ranges(const Eigen::Quaterniond & rotation) {
    const auto [pitch, yaw] = pitch_yaw(rotation);
    neutral_pitch_ = pitch;
    neutral_yaw_ = yaw;
    min_pitch_ = max_pitch_ = pitch;
    min_yaw_ = max_yaw_ = previous_yaw_ = yaw;
  }

  bool excitation_complete() const {
    const double pitch_direction =
      options_.min_directional_excursion_ratio * options_.min_pitch_range_rad;
    const double yaw_direction =
      options_.min_directional_excursion_ratio * options_.min_yaw_range_rad;
    return pitch_range_rad() >= options_.min_pitch_range_rad &&
           yaw_range_rad() >= options_.min_yaw_range_rad &&
           max_pitch_ - neutral_pitch_ >= pitch_direction &&
           neutral_pitch_ - min_pitch_ >= pitch_direction &&
           max_yaw_ - neutral_yaw_ >= yaw_direction &&
           neutral_yaw_ - min_yaw_ >= yaw_direction;
  }

  bool returned_upright(const Eigen::Quaterniond & rotation) const {
    return angular_distance(rotation, neutral_rotation_) <=
           options_.return_upright_tolerance_rad;
  }

  void update_ranges(const Eigen::Quaterniond & rotation) {
    auto [pitch, yaw] = pitch_yaw(rotation);
    while (yaw - previous_yaw_ > M_PI) yaw -= 2.0 * M_PI;
    while (yaw - previous_yaw_ < -M_PI) yaw += 2.0 * M_PI;
    previous_yaw_ = yaw;
    min_pitch_ = std::min(min_pitch_, pitch);
    max_pitch_ = std::max(max_pitch_, pitch);
    min_yaw_ = std::min(min_yaw_, yaw);
    max_yaw_ = std::max(max_yaw_, yaw);
  }

  void cancel(const std::string & message) {
    if (!active() && phase_ != CalibrationPhase::kSolving) return;
    clear_samples();
    phase_ = CalibrationPhase::kCancelled;
    message_ = message;
  }

  void clear_samples() {
    pico_samples_.clear();
    odin_samples_.clear();
    min_pitch_ = min_yaw_ = std::numeric_limits<double>::infinity();
    max_pitch_ = max_yaw_ = -std::numeric_limits<double>::infinity();
    previous_yaw_ = 0.0;
    neutral_pitch_ = neutral_yaw_ = 0.0;
    neutral_rotation_ = Eigen::Quaterniond::Identity();
  }

  CalibrationSessionOptions options_;
  CalibrationPhase phase_{CalibrationPhase::kWaitingForInputs};
  std::string message_{"waiting for PICO and Odin inputs"};
  double session_started_sec_{0.0};
  double phase_started_sec_{0.0};
  double min_pitch_{std::numeric_limits<double>::infinity()};
  double max_pitch_{-std::numeric_limits<double>::infinity()};
  double min_yaw_{std::numeric_limits<double>::infinity()};
  double max_yaw_{-std::numeric_limits<double>::infinity()};
  double previous_yaw_{0.0};
  double neutral_pitch_{0.0};
  double neutral_yaw_{0.0};
  Eigen::Quaterniond neutral_rotation_{Eigen::Quaterniond::Identity()};
  std::vector<HostTimedPose> pico_samples_;
  std::vector<HostTimedPose> odin_samples_;
};

}  // namespace pico_odin
