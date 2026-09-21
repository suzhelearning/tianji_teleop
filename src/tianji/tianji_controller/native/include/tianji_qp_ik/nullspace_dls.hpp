#pragma once

#include "tianji_qp_ik/velocity_ik.hpp"

namespace tianji_qp_ik {

class NullspaceDlsIk7 final : public IArmVelocityIk {
 public:
  explicit NullspaceDlsIk7(DlsConfig config);

  ArmIkResult solve(const ArmIkInput& input) override;
  void reset() override {}
  IkAlgorithm algorithm() const noexcept override {
    return IkAlgorithm::kNullspaceDls;
  }

 private:
  DlsConfig config_;
};

}  // namespace tianji_qp_ik
