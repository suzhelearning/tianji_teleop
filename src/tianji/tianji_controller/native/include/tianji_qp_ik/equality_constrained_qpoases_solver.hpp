#pragma once

#include "tianji_qp_ik/config.hpp"
#include "tianji_qp_ik/types.hpp"

#include <qpOASES.hpp>

#include <array>
#include <memory>

namespace tianji_qp_ik {

inline constexpr int kEqualityConstrainedQpRows = 6;

struct EqualityConstrainedQpProblem7 {
  Mat77 H{Mat77::Identity()};
  Vec7 g{Vec7::Zero()};
  Mat67 A{Mat67::Zero()};
  Vec6 equality{Vec6::Zero()};
  Vec7 lower{Vec7::Constant(-1.0)};
  Vec7 upper{Vec7::Constant(1.0)};
};

class EqualityConstrainedQpoasesSolver7 {
 public:
  explicit EqualityConstrainedQpoasesSolver7(QpoasesConfig config);

  EqualityConstrainedQpoasesSolver7(
      const EqualityConstrainedQpoasesSolver7&) = delete;
  EqualityConstrainedQpoasesSolver7& operator=(
      const EqualityConstrainedQpoasesSolver7&) = delete;

  SolverResult7 solve(const EqualityConstrainedQpProblem7& problem);
  void reset() noexcept;

  int setupCount() const noexcept { return setup_count_; }
  int hotstartCount() const noexcept { return hotstart_count_; }

 private:
  bool valid(const EqualityConstrainedQpProblem7& problem) const noexcept;
  void copyProblemData(const EqualityConstrainedQpProblem7& problem);
  qpOASES::returnValue initialize();
  SolverStatus mapStatus(qpOASES::returnValue status) const noexcept;

  QpoasesConfig config_;
  std::unique_ptr<qpOASES::SQProblem> solver_;
  std::array<qpOASES::real_t, kArmDof * kArmDof> hessian_{};
  std::array<qpOASES::real_t, kArmDof> gradient_{};
  std::array<qpOASES::real_t, kEqualityConstrainedQpRows * kArmDof>
      constraints_{};
  std::array<qpOASES::real_t, kArmDof> lower_{};
  std::array<qpOASES::real_t, kArmDof> upper_{};
  std::array<qpOASES::real_t, kEqualityConstrainedQpRows> equality_lower_{};
  std::array<qpOASES::real_t, kEqualityConstrainedQpRows> equality_upper_{};
  bool initialized_{false};
  int setup_count_{0};
  int hotstart_count_{0};
};

}  // namespace tianji_qp_ik
