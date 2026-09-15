#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

namespace pico_bridge {

struct SmplVector3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct SmplQuaternion {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{0.0};

  bool operator==(const SmplQuaternion & other) const {
    return x == other.x && y == other.y && z == other.z && w == other.w;
  }
};

struct SmplPose {
  SmplVector3 position;
  SmplQuaternion orientation;
};

struct GroundAlignmentOptions {
  std::array<double, 3> foot_half_extents{0.115, 0.050, 0.022};
  std::size_t stable_window_frames{30};
  double stability_tolerance_m{0.02};
  bool require_world_reset{true};
};

inline bool finite(const SmplPose & pose) {
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

inline bool valid(const SmplPose & pose) {
  if (!finite(pose)) return false;
  const auto & q = pose.orientation;
  const double squared_norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
  return std::isfinite(squared_norm) && squared_norm > 1e-18;
}

inline double foot_sole_height(
  const SmplPose & pose, const std::array<double, 3> & half_extents)
{
  if (!finite(pose)) throw std::invalid_argument("foot pose is not finite");
  for (const double extent : half_extents) {
    if (!std::isfinite(extent) || !(extent > 0.0)) {
      throw std::invalid_argument("foot half extents must be positive and finite");
    }
  }

  const auto & q = pose.orientation;
  const double norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
  if (!std::isfinite(norm) || norm <= 1e-9) {
    throw std::invalid_argument("foot orientation is invalid");
  }
  const double x = q.x / norm;
  const double y = q.y / norm;
  const double z = q.z / norm;
  const double w = q.w / norm;
  const double r20 = 2.0 * (x*z - y*w);
  const double r21 = 2.0 * (y*z + x*w);
  const double r22 = 1.0 - 2.0 * (x*x + y*y);
  const double vertical_radius =
    std::abs(r20) * half_extents[0] +
    std::abs(r21) * half_extents[1] +
    std::abs(r22) * half_extents[2];
  return pose.position.z - vertical_radius;
}

class SmplGroundAlignment {
 public:
  explicit SmplGroundAlignment(GroundAlignmentOptions options)
  : options_(options), armed_(!options.require_world_reset)
  {
    if (options_.stable_window_frames == 0) {
      throw std::invalid_argument("stable window must be positive");
    }
    if (!std::isfinite(options_.stability_tolerance_m) ||
        options_.stability_tolerance_m < 0.0) {
      throw std::invalid_argument("stability tolerance must be non-negative and finite");
    }
    for (const double extent : options_.foot_half_extents) {
      if (!std::isfinite(extent) || !(extent > 0.0)) {
        throw std::invalid_argument("foot half extents must be positive and finite");
      }
    }
  }

  void reset() {
    samples_.clear();
    floor_height_.reset();
    armed_ = true;
  }

  bool observe(const std::vector<SmplPose> & poses) {
    if (poses.size() != 24) {
      if (!floor_height_) samples_.clear();
      return false;
    }
    for (const auto & pose : poses) {
      if (!valid(pose)) {
        if (!floor_height_) samples_.clear();
        return false;
      }
    }
    if (floor_height_) return true;
    if (!armed_) return false;

    double left = 0.0;
    double right = 0.0;
    try {
      left = foot_sole_height(poses[10], options_.foot_half_extents);
      right = foot_sole_height(poses[11], options_.foot_half_extents);
    } catch (const std::invalid_argument &) {
      samples_.clear();
      return false;
    }
    samples_.push_back({left, right});
    while (samples_.size() > options_.stable_window_frames) samples_.pop_front();
    if (samples_.size() < options_.stable_window_frames) return false;

    std::vector<double> heights;
    heights.reserve(samples_.size() * 2);
    for (const auto & sample : samples_) {
      heights.push_back(sample[0]);
      heights.push_back(sample[1]);
    }
    const auto limits = std::minmax_element(heights.begin(), heights.end());
    if (*limits.second - *limits.first > options_.stability_tolerance_m) return false;

    const std::size_t middle = heights.size() / 2;
    std::nth_element(heights.begin(), heights.begin() + middle, heights.end());
    const double upper = heights[middle];
    std::nth_element(heights.begin(), heights.begin() + middle - 1, heights.end());
    floor_height_ = 0.5 * (heights[middle - 1] + upper);
    samples_.clear();
    armed_ = false;
    return true;
  }

  bool locked() const { return floor_height_.has_value(); }
  const std::optional<double> & floor_height() const { return floor_height_; }

  std::vector<SmplPose> transform(const std::vector<SmplPose> & poses) const {
    if (!floor_height_) throw std::logic_error("ground alignment is not locked");
    std::vector<SmplPose> output = poses;
    for (auto & pose : output) pose.position.z -= *floor_height_;
    return output;
  }

 private:
  GroundAlignmentOptions options_;
  bool armed_{false};
  std::deque<std::array<double, 2>> samples_;
  std::optional<double> floor_height_;
};

}  // namespace pico_bridge
