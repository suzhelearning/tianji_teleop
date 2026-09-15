#include "tianji_qp_ik/spark_qpoases_diagnostic.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace tianji_qp_ik {
namespace {

TEST(SparkQpoasesDiagnostic, ReturnsAcceptedBilateralIkWithoutModification) {
  SparkGuidanceDiagnostics diagnostics;
  diagnostics.accepted = true;
  diagnostics.left.accepted = true;
  diagnostics.right.accepted = true;
  diagnostics.left.ik.accepted = true;
  diagnostics.right.ik.accepted = true;
  diagnostics.left.q_ik = Vec7::LinSpaced(-0.3, 0.3);
  diagnostics.right.q_ik = Vec7::LinSpaced(0.4, -0.2);

  const SparkQpoasesDirectCommand command = makeDirectCommand(diagnostics);

  ASSERT_TRUE(command.accepted) << command.detail;
  EXPECT_TRUE(command.left.isApprox(diagnostics.left.q_ik));
  EXPECT_TRUE(command.right.isApprox(diagnostics.right.q_ik));
}

TEST(SparkQpoasesDiagnostic, RejectsEitherArmFailureAndNonFiniteIk) {
  SparkGuidanceDiagnostics diagnostics;
  diagnostics.accepted = true;
  diagnostics.left.accepted = true;
  diagnostics.right.accepted = false;
  diagnostics.left.ik.accepted = true;
  diagnostics.right.ik.accepted = false;
  EXPECT_FALSE(makeDirectCommand(diagnostics).accepted);

  diagnostics.right.accepted = true;
  diagnostics.right.ik.accepted = true;
  diagnostics.left.q_ik.setZero();
  diagnostics.right.q_ik.setZero();
  diagnostics.right.q_ik[3] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(makeDirectCommand(diagnostics).accepted);
}

TEST(SparkQpoasesDiagnostic, ConvertsPositionErrorToBoundedPostureVelocity) {
  const Vec7 q_ik = Vec7::LinSpaced(-2.0, 2.0);
  const Vec7 q_model = Vec7::Zero();
  const Vec7 velocity_limit = Vec7::Constant(1.5);

  const SparkPostureVelocityResult result =
      makePostureVelocity(q_ik, q_model, 2.0, velocity_limit);

  ASSERT_TRUE(result.accepted) << result.detail;
  const Vec7 expected =
      (2.0 * q_ik).cwiseMax(-velocity_limit).cwiseMin(velocity_limit);
  EXPECT_TRUE(result.value.isApprox(expected));
  EXPECT_DOUBLE_EQ(result.value[0], -1.5);
  EXPECT_DOUBLE_EQ(result.value[kArmDof - 1], 1.5);
}

TEST(SparkQpoasesDiagnostic, RejectsInvalidPostureVelocityInputs) {
  Vec7 q_ik = Vec7::Zero();
  const Vec7 q_model = Vec7::Zero();
  const Vec7 velocity_limit = Vec7::Ones();
  EXPECT_FALSE(makePostureVelocity(q_ik, q_model, 0.0, velocity_limit).accepted);

  q_ik[0] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(makePostureVelocity(q_ik, q_model, 1.0, velocity_limit).accepted);

  q_ik.setZero();
  Vec7 invalid_limit = velocity_limit;
  invalid_limit[1] = -1.0;
  EXPECT_FALSE(makePostureVelocity(q_ik, q_model, 1.0, invalid_limit).accepted);
}

}  // namespace
}  // namespace tianji_qp_ik
