#pragma once

#include <algorithm>
#include <optional>
#include <string>

#include "pico_odin/se3.hpp"

namespace pico_odin {

enum class Discontinuity {
  kNone,
  kTimestampRollback,
  kFrameChanged,
  kReceiptGap,
  kPoseJump,
};

inline const char * discontinuity_name(Discontinuity reason) {
  switch (reason) {
    case Discontinuity::kNone: return "none";
    case Discontinuity::kTimestampRollback: return "timestamp rollback";
    case Discontinuity::kFrameChanged: return "frame changed";
    case Discontinuity::kReceiptGap: return "input gap";
    case Discontinuity::kPoseJump: return "pose discontinuity";
  }
  return "unknown";
}

struct PoseStreamContinuityOptions {
  double timestamp_rollback_tolerance_sec{0.5};
  double restart_receipt_gap_sec{1.0};
  double minimum_translation_jump_m{0.35};
  double minimum_rotation_jump_rad{0.60};
  double maximum_linear_speed_mps{4.0};
  double maximum_angular_speed_radps{6.0};
};

class PoseStreamContinuity {
 public:
  explicit PoseStreamContinuity(PoseStreamContinuityOptions options = {})
  : options_(options) {}

  Discontinuity observe(
    double stamp_sec,
    double receipt_sec,
    const std::string & frame_id,
    const Pose3 & pose)
  {
    Discontinuity result = Discontinuity::kNone;
    if (previous_) {
      if (stamp_sec + options_.timestamp_rollback_tolerance_sec < previous_->stamp_sec) {
        result = Discontinuity::kTimestampRollback;
      } else if (!frame_id.empty() && !previous_->frame_id.empty() &&
                 frame_id != previous_->frame_id) {
        result = Discontinuity::kFrameChanged;
      } else if (receipt_sec - previous_->receipt_sec >
                 options_.restart_receipt_gap_sec) {
        result = Discontinuity::kReceiptGap;
      } else {
        const double dt = stamp_sec - previous_->stamp_sec;
        if (dt > 1e-6) {
          const double translation_limit = std::max(
            options_.minimum_translation_jump_m,
            options_.maximum_linear_speed_mps * dt);
          const double rotation_limit = std::max(
            options_.minimum_rotation_jump_rad,
            options_.maximum_angular_speed_radps * dt);
          if ((pose.translation - previous_->pose.translation).norm() > translation_limit ||
              angular_distance(pose.rotation, previous_->pose.rotation) > rotation_limit) {
            result = Discontinuity::kPoseJump;
          }
        }
      }
    }
    previous_ = Sample{stamp_sec, receipt_sec, frame_id, pose};
    return result;
  }

  void reset() { previous_.reset(); }

 private:
  struct Sample {
    double stamp_sec;
    double receipt_sec;
    std::string frame_id;
    Pose3 pose;
  };

  PoseStreamContinuityOptions options_;
  std::optional<Sample> previous_;
};

}  // namespace pico_odin
