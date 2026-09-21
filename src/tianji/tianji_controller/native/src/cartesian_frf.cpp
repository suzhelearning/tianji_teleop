#include "tianji_qp_ik/cartesian_frf.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace tianji_qp_ik {
namespace {

Eigen::Vector3d axisFor(FrfChannel channel) {
  switch (channel) {
    case FrfChannel::kX:
    case FrfChannel::kRx:
      return Eigen::Vector3d::UnitX();
    case FrfChannel::kY:
    case FrfChannel::kRy:
      return Eigen::Vector3d::UnitY();
    case FrfChannel::kZ:
    case FrfChannel::kRz:
      return Eigen::Vector3d::UnitZ();
  }
  throw std::invalid_argument("invalid FRF channel");
}

bool rotational(FrfChannel channel) {
  return channel == FrfChannel::kRx || channel == FrfChannel::kRy ||
         channel == FrfChannel::kRz;
}

}  // namespace

double logChirpInstantaneousFrequency(const FrfExcitationConfig& config,
                                      double chirp_time_seconds) {
  if (!(config.start_hz > 0.0) || !(config.end_hz > config.start_hz) ||
      !(config.chirp_seconds > 0.0)) {
    throw std::invalid_argument("invalid logarithmic chirp configuration");
  }
  const double clamped =
      std::clamp(chirp_time_seconds, 0.0, config.chirp_seconds);
  return config.start_hz *
         std::pow(config.end_hz / config.start_hz,
                  clamped / config.chirp_seconds);
}

double logChirpSample(const FrfExcitationConfig& config,
                      std::int64_t sample_index) {
  if (!(config.dt > 0.0) || sample_index < 0) {
    throw std::invalid_argument("invalid FRF sample time");
  }
  const double absolute_time = static_cast<double>(sample_index) * config.dt;
  const double chirp_time = absolute_time - config.warmup_seconds;
  if (chirp_time < 0.0 || chirp_time >= config.chirp_seconds) {
    return 0.0;
  }
  const double ratio_log = std::log(config.end_hz / config.start_hz);
  const double phase =
      2.0 * M_PI * config.start_hz * config.chirp_seconds / ratio_log *
      (std::exp(ratio_log * chirp_time / config.chirp_seconds) - 1.0);
  return config.amplitude * std::sin(phase);
}

Pose perturbPose(const Pose& center, FrfChannel channel, double value) {
  Pose target = center;
  const Eigen::Vector3d axis = axisFor(channel);
  if (rotational(channel)) {
    target.rotation = Eigen::AngleAxisd(value, axis).toRotationMatrix() *
                      center.rotation;
  } else {
    target.position += value * axis;
  }
  return target;
}

FrfChannel parseFrfChannel(std::string_view value) {
  if (value == "x") return FrfChannel::kX;
  if (value == "y") return FrfChannel::kY;
  if (value == "z") return FrfChannel::kZ;
  if (value == "rx") return FrfChannel::kRx;
  if (value == "ry") return FrfChannel::kRy;
  if (value == "rz") return FrfChannel::kRz;
  throw std::invalid_argument("unknown FRF channel");
}

std::string_view toString(FrfChannel channel) noexcept {
  switch (channel) {
    case FrfChannel::kX: return "x";
    case FrfChannel::kY: return "y";
    case FrfChannel::kZ: return "z";
    case FrfChannel::kRx: return "rx";
    case FrfChannel::kRy: return "ry";
    case FrfChannel::kRz: return "rz";
  }
  return "unknown";
}

}  // namespace tianji_qp_ik
