#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "pico_odin/extrinsics_file.hpp"

namespace pico_odin {

struct HostTimedPose {
  double receipt_sec{0.0};
  Pose3 pose;
};

struct ReceiveLagEstimate {
  bool valid{false};
  double lag_sec{0.0};
  double correlation{-1.0};
  std::size_t pair_count{0};
  bool at_boundary{false};
};

struct SolverOptions {
  double min_pitch_range_rad{12.0 * M_PI / 180.0};
  double min_yaw_range_rad{12.0 * M_PI / 180.0};
  std::size_t min_pairs{80};
  double max_receive_lag_sec{0.30};
  double receive_lag_step_sec{0.002};
  double min_receive_lag_correlation{0.25};
  std::size_t min_receive_lag_pairs{10};
  bool reject_receive_lag_at_boundary{true};
  double max_interpolation_gap_sec{0.10};
  std::size_t relative_motion_stride{10};
  double y_prior_sigma_m{0.05};
  double max_rotation_rms_rad{0.10};
  double max_translation_rms_m{0.08};
  double max_condition_number{1e6};
  double max_final_neutral_rotation_error_rad{0.10};
  double max_final_neutral_tilt_rad{25.0 * M_PI / 180.0};
  int max_refinement_iterations{8};
  double refinement_huber_delta{1.0};
  double rotation_prior_sigma_rad{10.0};
  double mount_x_min_m{-0.65};
  double mount_x_max_m{0.10};
  double mount_abs_y_max_m{0.20};
  double mount_abs_z_max_m{0.50};
  double mount_rear_angle_max_rad{1.60};
};

struct SolverResult {
  bool success{false};
  std::string message;
  Pose3 pelvis_T_odin;
  CalibrationMetrics metrics;
  double receive_lag_sec{0.0};
};

SolverResult solve_extrinsics(
  const std::vector<HostTimedPose> & pico,
  const std::vector<HostTimedPose> & odin,
  const SolverOptions & options);

ReceiveLagEstimate estimate_receive_lag(
  const std::vector<HostTimedPose> & pico,
  const std::vector<HostTimedPose> & odin,
  const SolverOptions & options);

}  // namespace pico_odin
