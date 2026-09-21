#pragma once

#include "tianji_qp_ik/hierarchical_qp.hpp"
#include "tianji_qp_ik/hierarchical_qpoases_solver.hpp"
#include "tianji_qp_ik/velocity_ik.hpp"

#include <memory>

namespace tianji_qp_ik {

class HierarchicalQpIk7 final : public IArmVelocityIk {
 public:
  HierarchicalQpIk7(HierarchicalQpConfig config,
                    QpoasesConfig qpoases_config,
                    SafetyConfig safety_config);
  HierarchicalQpIk7(HierarchicalQpConfig config,
                    SafetyConfig safety_config,
                    std::unique_ptr<IHierarchicalQpSolver> solver);

  ArmIkResult solve(const ArmIkInput& input) override;
  void reset() override;
  IkAlgorithm algorithm() const noexcept override {
    return IkAlgorithm::kHierarchicalQp;
  }

 private:
  HierarchicalQpConfig config_;
  SafetyConfig safety_config_;
  HierarchicalQpBuilder builder_;
  std::unique_ptr<IHierarchicalQpSolver> solver_;
  bool initialized_{false};
};

}  // namespace tianji_qp_ik
