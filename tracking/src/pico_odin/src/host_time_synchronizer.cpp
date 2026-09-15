#include "pico_odin/host_time_synchronizer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace pico_odin {

HostTimeSynchronizer::HostTimeSynchronizer(HostTimeSynchronizerOptions options)
: options_(std::move(options))
{
  const double lag_steps = 2.0 * options_.solver_options.max_receive_lag_sec /
    options_.solver_options.receive_lag_step_sec;
  if (!std::isfinite(options_.history_sec) || !(options_.history_sec > 0.0) ||
      !std::isfinite(options_.max_sync_gap_sec) ||
      !(options_.max_sync_gap_sec > 0.0) ||
      !std::isfinite(options_.lag_smoothing_alpha) ||
      !(options_.lag_smoothing_alpha > 0.0) ||
      options_.lag_smoothing_alpha > 1.0 ||
      !std::isfinite(options_.solver_options.max_receive_lag_sec) ||
      !(options_.solver_options.max_receive_lag_sec > 0.0) ||
      !std::isfinite(options_.solver_options.receive_lag_step_sec) ||
      !(options_.solver_options.receive_lag_step_sec > 0.0) ||
      !std::isfinite(lag_steps) || lag_steps > 1e6 ||
      !std::isfinite(options_.solver_options.min_receive_lag_correlation) ||
      options_.solver_options.min_receive_lag_correlation < -1.0 ||
      options_.solver_options.min_receive_lag_correlation > 1.0 ||
      options_.solver_options.min_receive_lag_pairs < 3) {
    throw std::invalid_argument("invalid host-time synchronizer options");
  }
  options_.solver_options.max_interpolation_gap_sec = options_.max_sync_gap_sec;
}

void HostTimeSynchronizer::append_and_trim(
  std::deque<HostTimedPose> & samples,
  const HostTimedPose & sample,
  double history_sec)
{
  if (!std::isfinite(sample.receipt_sec) || !finite(sample.pose)) {
    throw std::invalid_argument("host-timed pose is invalid");
  }
  if (!samples.empty() && sample.receipt_sec < samples.back().receipt_sec) {
    throw std::invalid_argument("host receipt timestamps must be monotonic");
  }
  samples.push_back(sample);
  while (samples.size() > 2 &&
         sample.receipt_sec - samples.front().receipt_sec > history_sec) {
    samples.pop_front();
  }
}

void HostTimeSynchronizer::add_pico(const HostTimedPose & sample) {
  append_and_trim(pico_samples_, sample, options_.history_sec);
}

void HostTimeSynchronizer::add_odin(const HostTimedPose & sample) {
  append_and_trim(odin_samples_, sample, options_.history_sec);
}

std::optional<Pose3> HostTimeSynchronizer::interpolate_for_pico(
  double pico_receipt_sec) const
{
  if (!std::isfinite(pico_receipt_sec) || odin_samples_.size() < 2) {
    return std::nullopt;
  }
  const double target = target_odin_receipt(pico_receipt_sec);
  if (target < odin_samples_.front().receipt_sec ||
      target > odin_samples_.back().receipt_sec) {
    return std::nullopt;
  }
  const auto upper = std::lower_bound(
    odin_samples_.begin(), odin_samples_.end(), target,
    [](const HostTimedPose & sample, double value) {
      return sample.receipt_sec < value;
    });
  if (upper == odin_samples_.begin()) return upper->pose;
  if (upper == odin_samples_.end()) return std::nullopt;
  const auto lower = std::prev(upper);
  const double duration = upper->receipt_sec - lower->receipt_sec;
  if (duration <= 0.0 || duration > options_.max_sync_gap_sec) {
    return std::nullopt;
  }
  return interpolate(
    lower->pose, upper->pose,
    (target - lower->receipt_sec) / duration);
}

bool HostTimeSynchronizer::update_receive_lag() {
  const std::vector<HostTimedPose> pico(pico_samples_.begin(), pico_samples_.end());
  const std::vector<HostTimedPose> odin(odin_samples_.begin(), odin_samples_.end());
  const auto estimate = estimate_receive_lag(pico, odin, options_.solver_options);
  if (!estimate.valid || estimate.at_boundary) return false;
  if (!receive_lag_valid_) {
    receive_lag_sec_ = estimate.lag_sec;
  } else {
    receive_lag_sec_ =
      (1.0 - options_.lag_smoothing_alpha) * receive_lag_sec_ +
      options_.lag_smoothing_alpha * estimate.lag_sec;
  }
  receive_lag_correlation_ = estimate.correlation;
  receive_lag_valid_ = true;
  return true;
}

void HostTimeSynchronizer::reset() {
  pico_samples_.clear();
  odin_samples_.clear();
  receive_lag_valid_ = false;
  receive_lag_sec_ = 0.0;
  receive_lag_correlation_ = 0.0;
}

std::optional<double> HostTimeSynchronizer::latest_odin_receipt() const {
  if (odin_samples_.empty()) return std::nullopt;
  return odin_samples_.back().receipt_sec;
}

}  // namespace pico_odin
