#pragma once
#include "tianji_qp_ik/shared_root_options.hpp"

namespace tianji_qp_ik {
// Single control-thread owner. Pure post-receiver processing: no I/O, clock,
// executor authority, or internal thread.
class SharedRootPipeline {
 public:
  explicit SharedRootPipeline(const SharedRootOptions&);
  void resetSession() noexcept;
  bool observe(const PicoTeleopFrame&, std::int64_t now_ns);
  SharedRootContinuityOutput step(std::int64_t now_ns, bool authorized,
                                  const SharedRootTargets& recovery_model);
  bool accept(std::uint64_t sequence, std::uint64_t epoch,
              std::uint64_t generation) noexcept;
  const SharedRootBuiltTargets& candidate() const noexcept { return candidate_; }
  const MorphologyEstimate& morphology() const noexcept { return estimate_; }

 private:
  void clearAlgorithms() noexcept;
  void interruptBuilder() noexcept;
  TjvrSharedRootInputAdapter adapter_;
  SharedRootMorphologyEstimator morphology_;
  SharedRootTargetBuilder builder_;
  SharedRootContinuity continuity_;
  SharedRootContinuityConfig timing_;
  SharedRootBuiltTargets candidate_;
  MorphologyEstimate estimate_;
  std::uint64_t sequence_{0}, epoch_{0}, generation_{0};
  std::int64_t source_ns_{0}, receive_ns_{0}, last_valid_receive_ns_{0}, now_ns_{0};
  bool interrupted_{true}, rebuild_scale_{false}, await_confident_{false};
};
} // namespace tianji_qp_ik
