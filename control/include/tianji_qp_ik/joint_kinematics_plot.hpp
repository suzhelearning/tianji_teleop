#pragma once

#include "tianji_qp_ik/types.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace tianji_qp_ik {

enum class PlotMetric { kPosition, kVelocity, kAcceleration, kJerk };

enum class ReferenceAccelerationSource {
  kDifferentiateVelocity,
  kDirectQpOutput,
};

struct JointKinematicsState {
  Vec7 position{Vec7::Zero()};
  Vec7 velocity{Vec7::Zero()};
  Vec7 acceleration{Vec7::Zero()};
  Vec7 jerk{Vec7::Zero()};
};

struct JointKinematicsBounds {
  Vec7 position_lower{Vec7::Constant(-1.0)};
  Vec7 position_upper{Vec7::Constant(1.0)};
  Vec7 velocity_lower{Vec7::Constant(-1.0)};
  Vec7 velocity_upper{Vec7::Constant(1.0)};
  Vec7 acceleration_lower{Vec7::Constant(-1.0)};
  Vec7 acceleration_upper{Vec7::Constant(1.0)};
  Vec7 jerk_lower{Vec7::Constant(-1.0)};
  Vec7 jerk_upper{Vec7::Constant(1.0)};
};

struct ArmJointKinematicsSample {
  JointKinematicsState reference;
  JointKinematicsState actual;
  JointKinematicsBounds bounds;
  bool reference_acceleration_valid{false};
  bool reference_jerk_valid{false};
  bool actual_acceleration_valid{false};
  bool actual_jerk_valid{false};
};

struct JointKinematicsSample {
  std::uint64_t sequence{0U};
  double time_seconds{0.0};
  ArmJointKinematicsSample left;
  ArmJointKinematicsSample right;
  bool reset{false};
};

struct JointKinematicsDerivatives {
  Vec7 reference_acceleration{Vec7::Zero()};
  Vec7 reference_jerk{Vec7::Zero()};
  Vec7 actual_acceleration{Vec7::Zero()};
  Vec7 actual_jerk{Vec7::Zero()};
  bool reference_acceleration_valid{false};
  bool reference_jerk_valid{false};
  bool actual_acceleration_valid{false};
  bool actual_jerk_valid{false};
};

class JointKinematicsDifferentiator {
 public:
  JointKinematicsDerivatives update(
      const Vec7& reference_velocity,
      const Vec7& direct_reference_acceleration,
      ReferenceAccelerationSource source, const Vec7& actual_velocity,
      double fixed_dt, bool reset_requested) noexcept;
  void reset() noexcept;

 private:
  bool source_initialized_{false};
  bool reference_velocity_initialized_{false};
  bool reference_acceleration_initialized_{false};
  bool actual_velocity_initialized_{false};
  bool actual_acceleration_initialized_{false};
  ReferenceAccelerationSource previous_source_{
      ReferenceAccelerationSource::kDifferentiateVelocity};
  Vec7 previous_reference_velocity_{Vec7::Zero()};
  Vec7 previous_reference_acceleration_{Vec7::Zero()};
  Vec7 previous_actual_velocity_{Vec7::Zero()};
  Vec7 previous_actual_acceleration_{Vec7::Zero()};
};

struct JointPlotSeries {
  std::vector<double> time;
  std::vector<double> reference;
  std::vector<double> actual;
  std::vector<double> lower;
  std::vector<double> upper;
  std::vector<bool> reference_valid;
  std::vector<bool> actual_valid;
};

class JointKinematicsHistory {
 public:
  explicit JointKinematicsHistory(std::size_t capacity);

  void push(const JointKinematicsSample& sample) noexcept;
  void clear() noexcept;
  std::size_t size() const noexcept { return size_; }
  std::size_t capacity() const noexcept { return samples_.size(); }
  JointPlotSeries series(ArmSide side, PlotMetric metric, int joint) const;

 private:
  const JointKinematicsSample& chronological(std::size_t index) const noexcept;

  std::vector<JointKinematicsSample> samples_;
  std::size_t begin_{0U};
  std::size_t size_{0U};
};

const char* plotMetricName(PlotMetric metric) noexcept;
const char* plotMetricUnit(PlotMetric metric) noexcept;
PlotMetric nextPlotMetric(PlotMetric metric) noexcept;

}  // namespace tianji_qp_ik
