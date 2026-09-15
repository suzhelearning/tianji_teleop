#pragma once

#include "tianji_qp_ik/types.hpp"

#include <cstdint>
#include <deque>
#include <string_view>

namespace tianji_qp_ik {

struct SparkPalmTwistEstimatorConfig {
  double filter_alpha{0.70};
  int dt_median_window{31};
  double dt_min_ratio{0.5};
  double dt_max_ratio{1.5};
  double linear_stationary_threshold_m_s{0.002};
  double angular_stationary_threshold_rad_s{0.01};
  double stationary_decay{0.9025};
  double reversal_decay{0.7225};
  double maximum_linear_velocity_m_s{3.0};
  double maximum_angular_velocity_rad_s{12.0};
  double lowpass_cutoff_hz{0.8};
};

struct SparkPalmTwistDecision {
  bool accepted{false};
  bool dt_valid{false};
  bool reset{false};
  double source_dt_seconds{0.0};
  double median_dt_seconds{0.0};
  Vec6 twist{Vec6::Zero()};
  Vec6 low_frequency_twist{Vec6::Zero()};
  Vec6 high_frequency_twist{Vec6::Zero()};
  std::string_view detail{"palm_twist_not_updated"};
};

class SparkPalmTwistEstimator {
 public:
  explicit SparkPalmTwistEstimator(SparkPalmTwistEstimatorConfig config);

  void reset() noexcept;
  SparkPalmTwistDecision update(const Pose& pose, std::uint64_t sequence,
                                std::int64_t source_timestamp_ns,
                                std::uint64_t tracking_epoch,
                                bool stream_discontinuity) noexcept;
  const Vec6& twist() const noexcept { return twist_; }
  const Vec6& lowFrequencyTwist() const noexcept {
    return low_frequency_twist_;
  }
  Vec6 highFrequencyTwist() const noexcept {
    return twist_ - low_frequency_twist_;
  }

 private:
  double medianSourceDt() const noexcept;
  static void filterComponent(Eigen::Vector3d raw, double stationary_threshold,
                              double filter_alpha, double stationary_decay,
                              double reversal_decay,
                              Eigen::Vector3d& filtered) noexcept;
  static void limitNorm(double maximum_norm,
                        Eigen::Vector3d& value) noexcept;

  SparkPalmTwistEstimatorConfig config_;
  bool initialized_{false};
  Pose previous_pose_;
  std::uint64_t previous_sequence_{0U};
  std::int64_t previous_timestamp_ns_{0};
  std::uint64_t tracking_epoch_{0U};
  std::deque<double> valid_source_dts_;
  Vec6 twist_{Vec6::Zero()};
  Vec6 low_frequency_twist_{Vec6::Zero()};
};

}  // namespace tianji_qp_ik
