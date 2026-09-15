#ifndef PICO_BRIDGE__FOOT_IMU_INPUT_STATE_HPP_
#define PICO_BRIDGE__FOOT_IMU_INPUT_STATE_HPP_

#include <chrono>
#include <cmath>
#include <optional>

#include "pico_bridge/foot_imu_fusion_math.hpp"

namespace pico_bridge::fusion {

class FootImuInputState {
public:
  using TimePoint = std::chrono::steady_clock::time_point;

  void accept(const Quaternion &orientation, TimePoint received_at) {
    orientation_ = orientation;
    received_at_ = received_at;
  }

  void reject() {
    orientation_.reset();
    received_at_ = TimePoint{};
  }

  const std::optional<Quaternion> &orientation() const {
    return orientation_;
  }

  bool is_fresh(TimePoint now, double max_age_sec) const {
    if (!orientation_ || max_age_sec < 0.0) return false;
    const double age = std::chrono::duration<double>(now - received_at_).count();
    return age >= 0.0 && age <= max_age_sec;
  }

private:
  std::optional<Quaternion> orientation_;
  TimePoint received_at_{};
};

inline bool is_finite_position(const Vector3 &position) {
  return std::isfinite(position[0]) && std::isfinite(position[1]) &&
         std::isfinite(position[2]);
}

}  // namespace pico_bridge::fusion

#endif  // PICO_BRIDGE__FOOT_IMU_INPUT_STATE_HPP_
