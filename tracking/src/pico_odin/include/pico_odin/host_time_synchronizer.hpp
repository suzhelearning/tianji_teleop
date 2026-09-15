#pragma once

#include <deque>
#include <optional>

#include "pico_odin/extrinsic_solver.hpp"

namespace pico_odin {

struct HostTimeSynchronizerOptions {
  double history_sec{6.0};
  double max_sync_gap_sec{0.05};
  double lag_smoothing_alpha{0.20};
  SolverOptions solver_options;
};

class HostTimeSynchronizer {
 public:
  explicit HostTimeSynchronizer(HostTimeSynchronizerOptions options = {});

  void add_pico(const HostTimedPose & sample);
  void add_odin(const HostTimedPose & sample);
  std::optional<Pose3> interpolate_for_pico(double pico_receipt_sec) const;
  bool update_receive_lag();
  void reset();

  bool receive_lag_valid() const { return receive_lag_valid_; }
  double receive_lag_sec() const { return receive_lag_sec_; }
  double receive_lag_correlation() const { return receive_lag_correlation_; }
  double target_odin_receipt(double pico_receipt_sec) const {
    return pico_receipt_sec + receive_lag_sec_;
  }
  std::optional<double> latest_odin_receipt() const;

 private:
  static void append_and_trim(
    std::deque<HostTimedPose> & samples,
    const HostTimedPose & sample,
    double history_sec);

  HostTimeSynchronizerOptions options_;
  std::deque<HostTimedPose> pico_samples_;
  std::deque<HostTimedPose> odin_samples_;
  bool receive_lag_valid_{false};
  double receive_lag_sec_{0.0};
  double receive_lag_correlation_{0.0};
};

}  // namespace pico_odin
