#pragma once

#include "tianji_qp_ik/types.hpp"

#include <cstdint>
#include <string_view>

namespace tianji_qp_ik {

enum class FrfChannel { kX, kY, kZ, kRx, kRy, kRz };

struct FrfExcitationConfig {
  double dt{0.005};
  double warmup_seconds{2.0};
  double chirp_seconds{20.0};
  double settle_seconds{2.0};
  double start_hz{0.2};
  double end_hz{15.0};
  double amplitude{0.005};
};

double logChirpInstantaneousFrequency(const FrfExcitationConfig& config,
                                      double chirp_time_seconds);
double logChirpSample(const FrfExcitationConfig& config,
                      std::int64_t sample_index);
Pose perturbPose(const Pose& center, FrfChannel channel, double value);
FrfChannel parseFrfChannel(std::string_view value);
std::string_view toString(FrfChannel channel) noexcept;

}  // namespace tianji_qp_ik
