#pragma once

#include "tianji_mapped_palm/config.hpp"
#include "tianji_mapped_palm/causal_se3_twist_estimator.hpp"
#include "tianji_mapped_palm/spark_palm_twist_estimator.hpp"

#include <cstdint>

namespace tianji_mapped_palm {

struct PicoEeReferenceSample {
  // The accepted mapped-corrected-palm pose remains authoritative. Filtering
  // is applied only to the differentiated Cartesian feedforward below.
  Pose pose;
  bool pose_valid{false};
  Vec6 low_frequency_feedforward_twist{Vec6::Zero()};
  Vec6 high_frequency_feedforward_twist{Vec6::Zero()};
  Vec6 twist{Vec6::Zero()};
  Vec6 legacy_low_frequency_feedforward_twist{Vec6::Zero()};
  Vec6 legacy_high_frequency_feedforward_twist{Vec6::Zero()};
  Vec6 causal_low_frequency_feedforward_twist{Vec6::Zero()};
  Vec6 causal_high_frequency_feedforward_twist{Vec6::Zero()};
  CausalSe3TwistResult causal;
  PicoEeTwistEstimatorMode estimator_mode{PicoEeTwistEstimatorMode::kLegacy};
  bool causal_selected{false};
  bool valid{false};
  bool stale{false};
};

// Converts the accepted mapped-corrected-palm stream into a world-frame
// Cartesian feedforward twist. This is a reference-only component; it never
// changes a target pose or invokes an IK/QP solver.
class PicoEeReferenceGenerator {
 public:
  explicit PicoEeReferenceGenerator(double cutoff_hz);
  PicoEeReferenceGenerator(
      const SparkFeedforwardVelocityQpConfig& feedforward_config,
      const CartesianServoConfig& servo_config);
  PicoEeReferenceGenerator(
      const SparkFeedforwardVelocityQpConfig& feedforward_config,
      const CartesianServoConfig& servo_config,
      const CausalSe3TwistEstimatorConfig& estimator_config);

  void reconfigure(double cutoff_hz) noexcept;
  PicoEeReferenceSample update(const Pose& mapped_pose,
                               std::uint64_t sequence,
                               std::uint64_t tracking_epoch,
                               std::int64_t source_timestamp_ns,
                               bool stream_discontinuity,
                               double dt_seconds,
                               double observation_age_s = 0.0) noexcept;
  void reset() noexcept;

 private:
  SparkPalmTwistEstimatorConfig twist_config_;
  SparkPalmTwistEstimator twist_estimator_;
  CausalSe3TwistEstimatorConfig causal_config_;
  CausalSe3TwistEstimator causal_estimator_;
  Vec6 causal_lowpass_twist_{Vec6::Zero()};
  std::int64_t causal_previous_timestamp_ns_{0};
  double cutoff_hz_{1.0};
  double position_feedforward_gain_{1.0};
  double orientation_feedforward_gain_{1.0};
  double position_high_frequency_feedforward_gain_{0.0};
  double orientation_high_frequency_feedforward_gain_{0.0};
  bool initialized_{false};
  std::uint64_t previous_epoch_{0U};
};

}  // namespace tianji_mapped_palm
