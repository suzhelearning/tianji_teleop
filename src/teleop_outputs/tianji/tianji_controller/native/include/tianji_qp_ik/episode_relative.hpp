#pragma once

#include "tianji_qp_ik/controller.hpp"
#include "tianji_qp_ik/pico_teleop_protocol.hpp"
#include <array>
#include <cstdint>

namespace tianji_qp_ik {

enum class ReferenceOperation { kPrepare, kActivate, kCancel };
enum class ReferencePhase { kHeld, kPrepared, kActive, kFault };
const char* referencePhaseName(ReferencePhase phase) noexcept;

// Fixed-size ROS/control-thread exchange. Transport owns identity and ordering.
struct TeleopReferenceCommand {
  std::uint64_t token{0}, reference_id{0};
  ReferenceOperation operation{ReferenceOperation::kCancel};
  std::int64_t received_ns{0}, measured_ns{0};
  std::array<Vec7, 2> measured_q{Vec7::Zero(), Vec7::Zero()};
  double stable_time_s{0.3}, position_tolerance_m{0.01}, orientation_tolerance_rad{0.06};
  Eigen::Vector3d waist_lower{-0.4, -0.7, -0.85};
  Eigen::Vector3d waist_upper{0.7, 0.7, -0.05};
};
struct TeleopReferenceResult {
  std::uint64_t token{0}, reference_id{0};
  bool success{false};
  ReferencePhase phase{ReferencePhase::kHeld};
  const char* message{"rejected"};
  std::int64_t input_ns{0};
  std::uint64_t tracking_epoch{0};
  std::array<Vec7, 2> measured_q{Vec7::Zero(), Vec7::Zero()};
  std::array<Pose, 2> human_reference, robot_reference;
};

class EpisodeRelativeSession {
 public:
  explicit EpisodeRelativeSession(double freshness_s);
  // Uses calibrated skeleton palms, not absolute frame.left/right retargets.
  bool observe(const PicoTeleopFrame& frame, std::int64_t now) noexcept;
  void tick(std::int64_t now) noexcept;
  TeleopReferenceResult command(const TeleopReferenceCommand& request,
                               DualArmController& controller, std::int64_t now);
  bool ready(std::int64_t now) const noexcept;
  bool active() const noexcept { return phase_ == ReferencePhase::kActive; }
  bool faulted() const noexcept { return phase_ == ReferencePhase::kFault; }
  ReferencePhase phase() const noexcept { return phase_; }
  std::uint64_t referenceId() const noexcept { return reference_id_; }
  DualArmTargets targets() const noexcept;
  void fault() noexcept;
 private:
  struct Sample {
    std::array<Pose, 2> palms;
    Eigen::Vector3d shoulder_midpoint{Eigen::Vector3d::Zero()};
    std::int64_t input_ns{0};
  };
  static bool validPolicy(const TeleopReferenceCommand& policy) noexcept;
  bool fresh(std::int64_t now) const noexcept;
  bool waist(const Sample& sample, const TeleopReferenceCommand& policy) const noexcept;
  bool stable(const TeleopReferenceCommand& policy, std::int64_t now) const noexcept;
  bool atCapture(const Sample& sample) const noexcept;
  TeleopReferenceResult result(std::uint64_t token, bool success, const char* message) const;
  // At most two seconds at 500 Hz. Overflow fails readiness, never shortens a window.
  static constexpr std::size_t kHistoryCapacity = 1024;
  std::array<Sample, kHistoryCapacity> history_;
  std::size_t history_begin_{0}, history_size_{0};
  Sample latest_, captured_;
  std::array<Vec7, 2> captured_q_{Vec7::Zero(), Vec7::Zero()};
  std::array<Pose, 2> robot_reference_;
  TeleopReferenceCommand policy_;
  ReferencePhase phase_{ReferencePhase::kHeld};
  std::int64_t freshness_ns_, prepared_ns_{0};
  std::uint64_t epoch_{0}, generation_{0}, sequence_{0}, reference_id_{0}, highest_reference_{0};
  bool source_valid_{false}, prepared_valid_{false};
};
}  // namespace tianji_qp_ik
