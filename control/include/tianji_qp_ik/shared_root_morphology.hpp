#pragma once
#include "tianji_qp_ik/shared_root_input.hpp"
#include <array>

namespace tianji_qp_ik {
enum class MorphologyState { kUninitialized, kProvisional, kConfident, kInvalid };
struct SharedRootMorphologyConfig {
  std::size_t window_frames{31}, minimum_unique_samples{15};
  double mad_ratio_max{.05}, outlier_ratio{.15};
  double provisional_timeout_s{2}, sustained_invalid_timeout_s{.25};
  double robot_width_m{.423};
  double robot_reach_m{.2875639059409229+.3145155004129367+.1315};
  std::array<double,2> shoulder_range{.10,.60}, upper_range{.10,.50};
  std::array<double,2> forearm_range{.10,.50}, wrist_palm_range{.005,.25};
  std::array<double,2> reach_scale_range{.5,2}, lateral_scale_range{.5,2};
};
struct MorphologyEstimate {
  bool valid{false}, new_sample{false};
  MorphologyState state{MorphologyState::kUninitialized};
  std::size_t samples{0};
  double reach_scale{1}, lateral_scale{1};
  std::string_view detail{"uninitialized"};
};
class SharedRootMorphologyEstimator {
 public:
  explicit SharedRootMorphologyEstimator(SharedRootMorphologyConfig config = {});
  MorphologyEstimate update(const SharedRootInput& input) noexcept;
  void reset() noexcept;
 private:
  SharedRootMorphologyConfig config_;
  std::array<double,128> reaches_{}, widths_{};
  std::size_t cursor_{0}, count_{0};
  std::uint64_t epoch_{0}, generation_{0}, sequence_{0};
  std::int64_t source_ns_{0}, receive_ns_{0}, started_ns_{0}, invalid_since_ns_{0};
  MorphologyEstimate last_;
};
} // namespace tianji_qp_ik
