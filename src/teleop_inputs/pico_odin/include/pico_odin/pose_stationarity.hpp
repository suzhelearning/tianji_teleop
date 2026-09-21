#pragma once

#include <cmath>
#include <optional>

#include "pico_odin/se3.hpp"

namespace pico_odin {

struct PoseStationarityOptions {
  double maximum_linear_speed_mps{0.04};
  double maximum_angular_speed_radps{0.08};
  double minimum_interval_sec{1e-4};
  double maximum_interval_sec{0.5};
};

struct PoseStationarityObservation {
  bool valid_interval{false};
  bool stationary{false};
  double rate_hz{0.0};
};

class PoseStationarity {
 public:
  explicit PoseStationarity(PoseStationarityOptions options = {})
  : options_(options) {}

  PoseStationarityObservation observe(double stamp_sec, const Pose3 & pose) {
    PoseStationarityObservation observation;
    if (!std::isfinite(stamp_sec) || !finite(pose)) {
      reset();
      return observation;
    }
    if (previous_) {
      const double interval = stamp_sec - previous_->stamp_sec;
      if (interval > options_.minimum_interval_sec &&
          interval <= options_.maximum_interval_sec) {
        const double linear_speed =
          (pose.translation - previous_->pose.translation).norm() / interval;
        const double angular_speed =
          angular_distance(pose.rotation, previous_->pose.rotation) / interval;
        observation.valid_interval = true;
        observation.stationary =
          linear_speed <= options_.maximum_linear_speed_mps &&
          angular_speed <= options_.maximum_angular_speed_radps;
        observation.rate_hz = 1.0 / interval;
      }
    }
    previous_ = Sample{stamp_sec, pose};
    last_observation_ = observation;
    return observation;
  }

  const PoseStationarityObservation & last_observation() const {
    return last_observation_;
  }

  void reset() {
    previous_.reset();
    last_observation_ = {};
  }

 private:
  struct Sample {
    double stamp_sec;
    Pose3 pose;
  };

  PoseStationarityOptions options_;
  std::optional<Sample> previous_;
  PoseStationarityObservation last_observation_;
};

}  // namespace pico_odin
