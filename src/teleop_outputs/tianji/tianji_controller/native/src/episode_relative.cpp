#include "tianji_qp_ik/episode_relative.hpp"
#include "tianji_qp_ik/shared_root_input.hpp"
#include "tianji_qp_ik/so3.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace tianji_qp_ik {
namespace {
bool recent(std::int64_t stamp, std::int64_t now, std::int64_t limit) noexcept {
  return stamp > 0 && stamp <= now && now - stamp <= limit;
}
bool near(const Pose& a, const Pose& b, double position, double orientation) noexcept {
  return (a.position - b.position).norm() <= position &&
         rotationDistance(a.rotation, b.rotation) <= orientation;
}
}
const char* referencePhaseName(ReferencePhase phase) noexcept {
  switch (phase) {
    case ReferencePhase::kHeld: return "held";
    case ReferencePhase::kPrepared: return "prepared";
    case ReferencePhase::kActive: return "active";
    case ReferencePhase::kFault: return "fault";
  }
  return "fault";
}
EpisodeRelativeSession::EpisodeRelativeSession(double freshness_s)
    : freshness_ns_(0) {
  if (!std::isfinite(freshness_s) || freshness_s <= 0)
    throw std::invalid_argument("episode reference requires positive finite freshness");
  freshness_ns_ = static_cast<std::int64_t>(std::clamp(freshness_s, .001, .1) * 1e9);
}
void EpisodeRelativeSession::fault() noexcept {
  phase_ = ReferencePhase::kFault;
  prepared_valid_ = false;
}
bool EpisodeRelativeSession::observe(const PicoTeleopFrame& frame, std::int64_t now) noexcept {
  const bool changed = epoch_ != 0 && (frame.tracking_epoch != epoch_ ||
      frame.resynchronization_generation != generation_ || frame.stream_discontinuity);
  const auto input = TjvrSharedRootInputAdapter{}.adapt(frame);
  if (!frame.valid || !input.valid || frame.user_button_pressed ||
      !recent(frame.bridge_send_monotonic_ns, now, freshness_ns_) ||
      !recent(frame.receive_monotonic_ns, now, freshness_ns_) ||
      (!changed && sequence_ != 0 && frame.sequence <= sequence_)) {
    source_valid_ = false;
    history_size_ = 0;
    if (reference_id_ != 0) fault();
    return false;
  }
  if (changed || (source_valid_ && !recent(latest_.input_ns, now, freshness_ns_))) {
    history_size_ = 0;
    if (reference_id_ != 0) fault();
  }
  epoch_ = frame.tracking_epoch;
  generation_ = frame.resynchronization_generation;
  sequence_ = frame.sequence;
  source_valid_ = true;
  latest_.input_ns = frame.bridge_send_monotonic_ns;
  latest_.palms[0] = {input.left.p_control_root_Ct, input.left.R_palm_Ct};
  latest_.palms[1] = {input.right.p_control_root_Ct, input.right.R_palm_Ct};
  latest_.shoulder_midpoint = .5 * (input.left.p_shoulder_root_Ct + input.right.p_shoulder_root_Ct);
  if (history_size_ == kHistoryCapacity) {
    history_begin_ = (history_begin_ + 1) % kHistoryCapacity;
    --history_size_;
  }
  history_[(history_begin_ + history_size_) % kHistoryCapacity] = latest_;
  ++history_size_;
  if (phase_ == ReferencePhase::kPrepared &&
      (!waist(latest_, policy_) || !atCapture(latest_))) prepared_valid_ = false;
  return !faulted();
}
bool EpisodeRelativeSession::fresh(std::int64_t now) const noexcept {
  return source_valid_ && recent(latest_.input_ns, now, freshness_ns_);
}
void EpisodeRelativeSession::tick(std::int64_t now) noexcept {
  if (reference_id_ != 0 && !fresh(now)) fault();
  if (phase_ == ReferencePhase::kPrepared && now - prepared_ns_ > 5000000000LL)
    prepared_valid_ = false; // Frozen q/id remains published until explicit cancellation.
}
bool EpisodeRelativeSession::validPolicy(const TeleopReferenceCommand& p) noexcept {
  return std::isfinite(p.stable_time_s) && p.stable_time_s >= .05 && p.stable_time_s <= 2. &&
      std::isfinite(p.position_tolerance_m) && p.position_tolerance_m > 0 && p.position_tolerance_m <= .1 &&
      std::isfinite(p.orientation_tolerance_rad) && p.orientation_tolerance_rad > 0 && p.orientation_tolerance_rad <= .5 &&
      p.waist_lower.allFinite() && p.waist_upper.allFinite() &&
      (p.waist_lower.array() < p.waist_upper.array()).all() &&
      p.waist_lower.cwiseAbs().maxCoeff() <= 2. && p.waist_upper.cwiseAbs().maxCoeff() <= 2.;
}
bool EpisodeRelativeSession::waist(const Sample& sample, const TeleopReferenceCommand& p) const noexcept {
  for (const auto& palm : sample.palms) {
    const Eigen::Vector3d relative = palm.position - sample.shoulder_midpoint;
    if ((relative.array() < p.waist_lower.array()).any() ||
        (relative.array() > p.waist_upper.array()).any()) return false;
  }
  return true;
}
bool EpisodeRelativeSession::stable(const TeleopReferenceCommand& p, std::int64_t now) const noexcept {
  if (!fresh(now) || history_size_ < 2 || !waist(latest_, p)) return false;
  const auto boundary = latest_.input_ns - static_cast<std::int64_t>(p.stable_time_s * 1e9);
  std::int64_t previous_ns = latest_.input_ns;
  for (std::size_t n = history_size_; n > 0; --n) {
    const auto& sample = history_[(history_begin_ + n - 1) % kHistoryCapacity];
    if (previous_ns < sample.input_ns || previous_ns - sample.input_ns > freshness_ns_ ||
        !waist(sample, p)) return false;
    for (std::size_t arm = 0; arm < 2; ++arm)
      if (!near(sample.palms[arm], latest_.palms[arm], p.position_tolerance_m,
                p.orientation_tolerance_rad)) return false;
    if (sample.input_ns <= boundary) return true;
    previous_ns = sample.input_ns;
  }
  return false;
}
bool EpisodeRelativeSession::atCapture(const Sample& sample) const noexcept {
  for (std::size_t arm = 0; arm < 2; ++arm)
    if (!near(sample.palms[arm], captured_.palms[arm], policy_.position_tolerance_m,
              policy_.orientation_tolerance_rad)) return false;
  return true;
}
bool EpisodeRelativeSession::ready(std::int64_t now) const noexcept {
  // Preflight has no capture policy yet: never solve or move merely for readiness.
  // The exact caller-supplied waist/stability policy is checked synchronously on r.
  return !faulted() && fresh(now);
}
TeleopReferenceResult EpisodeRelativeSession::result(std::uint64_t token, bool success,
                                                     const char* message) const {
  TeleopReferenceResult out;
  out.token = token; out.reference_id = reference_id_; out.success = success;
  out.phase = phase_; out.message = message;
  out.input_ns = captured_.input_ns; out.tracking_epoch = epoch_;
  out.measured_q = captured_q_; out.human_reference = captured_.palms;
  out.robot_reference = robot_reference_;
  return out;
}
TeleopReferenceResult EpisodeRelativeSession::command(const TeleopReferenceCommand& request,
    DualArmController& controller, std::int64_t now) {
  tick(now);
  if (!recent(request.received_ns, now, 250000000) ||
      !recent(request.measured_ns, now, 100000000))
    return result(request.token, false, "command_expired");
  if (request.operation == ReferenceOperation::kCancel) {
    if (request.reference_id != reference_id_)
      return result(request.token, false, "reference_id_mismatch");
    reference_id_ = 0; prepared_valid_ = false;
    if (!faulted()) phase_ = ReferencePhase::kHeld;
    // No solver/reset/reference write: retain the last committed model q.
    return result(request.token, true, "cancelled");
  }
  if (faulted()) return result(request.token, false, "fault_requires_restart");
  if (request.operation == ReferenceOperation::kPrepare) {
    if (active() || request.reference_id == 0 || request.reference_id <= highest_reference_)
      return result(request.token, false, "invalid_reference_order_or_active");
    highest_reference_ = request.reference_id;
    // Every prepare attempt revokes the preceding prepared reference; never retry automatically.
    reference_id_ = 0; prepared_valid_ = false; phase_ = ReferencePhase::kHeld;
    if (!validPolicy(request)) return result(request.token, false, "invalid_readiness_policy");
    if (!stable(request, now)) return result(request.token, false, "palms_not_fresh_stable_at_waist");
    DualArmTargets tcp;
    if (!controller.resetEpisodeReference(request.measured_q, tcp))
      return result(request.token, false, "measured_q_outside_stationary_execution_envelope");
    captured_q_ = request.measured_q;
    robot_reference_ = {tcp.left, tcp.right};
    captured_ = latest_; policy_ = request; reference_id_ = request.reference_id;
    phase_ = ReferencePhase::kPrepared; prepared_ns_ = now; prepared_valid_ = true;
    return result(request.token, true, "prepared");
  }
  if (phase_ != ReferencePhase::kPrepared || request.reference_id != reference_id_)
    return result(request.token, false, "no_matching_prepared_reference");
  if (!prepared_valid_ || !stable(policy_, now) || !atCapture(latest_)) {
    prepared_valid_ = false;
    return result(request.token, false, "prepare_expired_or_palms_moved_new_r_required");
  }
  for (std::size_t arm = 0; arm < 2; ++arm)
    if (!request.measured_q[arm].allFinite() ||
        (request.measured_q[arm] - captured_q_[arm]).cwiseAbs().maxCoeff() > .01) {
      prepared_valid_ = false;
      return result(request.token, false, "robot_moved_during_prepare");
    }
  phase_ = ReferencePhase::kActive;
  return result(request.token, true, "active");
}
DualArmTargets EpisodeRelativeSession::targets() const noexcept {
  DualArmTargets out;
  Pose* poses[] = {&out.left, &out.right};
  for (std::size_t arm = 0; arm < 2; ++arm) {
    poses[arm]->position = robot_reference_[arm].position + latest_.palms[arm].position - captured_.palms[arm].position;
    poses[arm]->rotation = latest_.palms[arm].rotation * captured_.palms[arm].rotation.transpose() * robot_reference_[arm].rotation;
  }
  out.left_stale = out.right_stale = !active();
  return out;
}
}  // namespace tianji_qp_ik
