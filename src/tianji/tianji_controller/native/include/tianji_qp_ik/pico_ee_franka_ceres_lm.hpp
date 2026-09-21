#pragma once

#include "tianji_qp_ik/ceres_ik_types.hpp"

namespace tianji_qp_ik {

// Stateless across control frames: never caches a previous goal.
struct PicoEeFrankaCeresLmResult : PicoEeFrankaDlsResult {
  int evaluations{0};
  int nullspace_trials{0};
  bool nullspace_accepted{false};
};

class PicoEeFrankaCeresLmIk7 {
 public:
  PicoEeFrankaCeresLmIk7(PicoEeFrankaCeresLmConfig config, double margin);
  PicoEeFrankaCeresLmResult solve(const PicoEeFrankaDlsInput& input) const;
  static bool available() noexcept;
  // Derivative of [p_target-p; Log(R_target R^T)] in world coordinates.
  static Mat67 residualJacobian(const Vec6& error, const Mat67& geometric);

 private:
  PicoEeFrankaCeresLmConfig config_;
  double margin_;
};

}  // namespace tianji_qp_ik
