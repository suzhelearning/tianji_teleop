#pragma once

#include "tianji_mapped_palm/config.hpp"
#include "tianji_mapped_palm/types.hpp"

#include <array>
#include <cstdint>
#include <string_view>

namespace tianji_mapped_palm {

enum class CausalSe3TwistFailure {
  kNone,
  kInsufficientHistory,
  kNonFiniteInput,
  kNonMonotonicSequence,
  kNonMonotonicTimestamp,
  kEpochReset,
  kStreamDiscontinuity,
  kSourceDtOutlier,
  kFitIllConditioned,
  kPositionFitResidual,
  kOrientationFitResidual,
  kObservationAge,
};

struct CausalSe3TwistInput {
  Pose pose;
  std::uint64_t sequence{0U};
  std::uint64_t tracking_epoch{0U};
  std::int64_t source_timestamp_ns{0};
  bool stream_discontinuity{false};
  double observation_age_s{0.0};
  double control_dt_s{0.005};
};

struct CausalSe3TwistResult {
  bool valid{false};
  double confidence{0.0};
  int fit_window{0};
  double condition_number{0.0};
  double position_fit_rms_m{0.0};
  double orientation_fit_rms_rad{0.0};
  double observation_age_s{0.0};
  Vec6 source_twist{Vec6::Zero()};
  Vec6 source_acceleration{Vec6::Zero()};
  Vec6 age_compensation{Vec6::Zero()};
  Vec6 aligned_twist{Vec6::Zero()};
  CausalSe3TwistFailure failure{
      CausalSe3TwistFailure::kInsufficientHistory};
};

std::string_view toString(CausalSe3TwistFailure failure) noexcept;

class CausalSe3TwistEstimator {
 public:
  explicit CausalSe3TwistEstimator(CausalSe3TwistEstimatorConfig config);
  CausalSe3TwistResult update(const CausalSe3TwistInput& input) noexcept;
  void reset() noexcept;

 private:
  struct Sample {
    Pose pose;
    std::uint64_t sequence{0U};
    std::uint64_t tracking_epoch{0U};
    std::int64_t source_timestamp_ns{0};
  };

  template <int Window>
  CausalSe3TwistResult fit(const CausalSe3TwistInput& input) noexcept;
  CausalSe3TwistResult fail(CausalSe3TwistFailure failure) noexcept;
  void append(const CausalSe3TwistInput& input) noexcept;
  bool sourceDtIsOutlier(std::int64_t timestamp_ns) const noexcept;

  CausalSe3TwistEstimatorConfig config_;
  std::array<Sample, 7> samples_{};
  int sample_count_{0};
  double filtered_confidence_{0.0};
};

}  // namespace tianji_mapped_palm
