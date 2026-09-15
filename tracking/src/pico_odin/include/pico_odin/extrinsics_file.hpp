#pragma once

#include <filesystem>
#include <string>

#include "pico_odin/se3.hpp"

namespace pico_odin {

struct CalibrationMetrics {
  int sample_count{0};
  double time_correlation{0.0};
  double pitch_range_rad{0.0};
  double yaw_range_rad{0.0};
  double rotation_rms_rad{0.0};
  double translation_rms_m{0.0};
  double condition_number{0.0};
  int refinement_iterations{0};
};

struct ExtrinsicsDocument {
  int schema_version{3};
  bool valid{false};
  Pose3 pelvis_T_odin;
  CalibrationMetrics metrics;
  std::string calibrated_at;
  std::string pico_topic;
  std::string odin_topic;
};

std::filesystem::path default_extrinsics_path();
ExtrinsicsDocument load_extrinsics(const std::filesystem::path & path);
void save_extrinsics_atomic(
  const std::filesystem::path & path,
  const ExtrinsicsDocument & document);

}  // namespace pico_odin
