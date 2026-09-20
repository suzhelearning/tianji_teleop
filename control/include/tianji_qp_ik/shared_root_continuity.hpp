#pragma once
#include "tianji_qp_ik/shared_root_target_builder.hpp"
namespace tianji_qp_ik {
enum class SharedRootState { kUninitialized, kTracking, kHoldLastMapped, kRecovering, kInvalid };
struct SharedRootContinuityConfig {
  std::size_t recovery_frames{5};
  double maximum_hold_s{.1}, freshness_s{.1}, minimum_receive_span_s{.02};
  double maximum_receive_gap_s{.1}, maximum_source_gap_s{.1}, blend_s{.15};
  double maximum_elbow_step_m{.15}; // loaded from existing PICO position jump bound
};
struct SharedRootContinuityOutput {
  bool valid{false}, reset_histories{false}, suppress_stationary_hold{false};
  SharedRootState state{SharedRootState::kUninitialized};
  double alpha{0};
  SparkUpperTargets target;
  std::uint64_t sequence{0},epoch{0},generation{0};
  std::int64_t receive_monotonic_ns{0};
};
// No hardware authority and no own clock/thread: caller supplies monotonic now,
// existing execution authorization and one coherent FK/model snapshot.
class SharedRootContinuity {
 public:
  explicit SharedRootContinuity(SharedRootContinuityConfig config={},
      std::optional<SharedRootClosureGeometry> geometry=std::nullopt);
  void resetSession() noexcept;
  void observe(const SharedRootBuiltTargets&,std::int64_t now_ns);
  SharedRootContinuityOutput step(std::int64_t now_ns,bool authorized,
                                  const SparkUpperTargets& model_snapshot);
  bool accept(std::uint64_t sequence,std::uint64_t epoch,std::uint64_t generation) noexcept;
  const SharedRootElbowHistory& elbowHistory() const noexcept { return elbow_history_; }
  const SharedRootElbowHistory& acceptedElbows() const noexcept { return accepted_elbows_; }
 private:
  bool fresh(std::int64_t now,std::int64_t received,double max_s) const noexcept;
  void interrupt() noexcept;
  SharedRootContinuityConfig config_;
  std::optional<SharedRootClosureGeometry> geometry_;
  SharedRootElbowHistory elbow_history_,accepted_elbows_;
  SharedRootState state_{SharedRootState::kUninitialized};
  SharedRootBuiltTargets candidate_;
  SparkUpperTargets held_,start_;
  SharedRootContinuityOutput pending_;
  std::uint64_t epoch_{0},generation_{0},sequence_{0};
  std::int64_t source_ns_{0},receive_ns_{0},held_receive_ns_{0};
  std::int64_t first_candidate_ns_{0},blend_start_ns_{0},last_now_ns_{0};
  std::size_t candidate_count_{0};
};
} // namespace tianji_qp_ik
