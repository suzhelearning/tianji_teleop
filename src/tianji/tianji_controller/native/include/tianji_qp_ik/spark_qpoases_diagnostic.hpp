#pragma once

#include "tianji_qp_ik/spark_guidance.hpp"

#include <string_view>

namespace tianji_qp_ik {

struct SparkQpoasesDirectCommand {
  bool accepted{false};
  Vec7 left{Vec7::Zero()};
  Vec7 right{Vec7::Zero()};
  std::string_view detail{"not_evaluated"};
};

struct SparkPostureVelocityResult {
  bool accepted{false};
  Vec7 value{Vec7::Zero()};
  std::string_view detail{"not_evaluated"};
};

SparkQpoasesDirectCommand makeDirectCommand(
    const SparkGuidanceDiagnostics& diagnostics) noexcept;

SparkPostureVelocityResult makePostureVelocity(
    const Vec7& q_ik, const Vec7& q_model, double position_gain,
    const Vec7& velocity_limit) noexcept;

}  // namespace tianji_qp_ik
