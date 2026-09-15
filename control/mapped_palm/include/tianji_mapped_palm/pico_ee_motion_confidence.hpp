#pragma once

#include "tianji_mapped_palm/config.hpp"
#include "tianji_mapped_palm/types.hpp"

#include <string_view>

namespace tianji_mapped_palm {

enum class MotionConfidenceReason {
  kNone,
  kDisabled,
  kInvalidInput,
  kStale,
  kReset,
};

struct PicoEeMotionEvidenceInput {
  Vec6 low_frequency_twist{Vec6::Zero()};
  double freshness{1.0};
  double estimator_confidence{1.0};
  bool stale{false};
  bool reset{false};
  double dt{0.005};
};

struct PicoEeMotionEvidence {
  bool valid{false};
  double linear_raw{0.0};
  double angular_raw{0.0};
  double linear_filtered{0.0};
  double angular_filtered{0.0};
  double raw{0.0};
  double filtered{0.0};
  MotionConfidenceReason reason{MotionConfidenceReason::kDisabled};
};

class PicoEeMotionConfidence {
 public:
  explicit PicoEeMotionConfidence(PicoEeMotionConfidenceConfig config);
  PicoEeMotionEvidence update(const PicoEeMotionEvidenceInput& input) noexcept;
  void reset() noexcept;

 private:
  PicoEeMotionConfidenceConfig config_;
  double linear_filtered_{0.0};
  double angular_filtered_{0.0};
};

std::string_view toString(MotionConfidenceReason reason) noexcept;

}  // namespace tianji_mapped_palm
