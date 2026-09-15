#include "tianji_qp_ik/cartesian_otg.hpp"
#include "tianji_qp_ik/controller.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <memory>

namespace tianji_qp_ik {
namespace {

std::string modelPath() {
  return (std::filesystem::path(TIANJI_PROJECT_SOURCE_DIR) / "models" /
          "marvin_m6_qp_test.xml")
      .string();
}

QpIkConfig testConfig() {
  QpIkConfig config = loadConfig(
      (std::filesystem::path(TIANJI_PROJECT_SOURCE_DIR) / "config" /
       "qp_ik_cartesian_otg_velocity.yaml")
          .string());
  config.safety.reference_tracking_warn_rad = 10.0;
  config.safety.reference_tracking_stop_rad = 20.0;
  return config;
}

void initializeAtMidpoint(MujocoRobot& robot) {
  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    const ArmLimits& limits = robot.mapping(side).limits;
    robot.setArmPosition(
        side, 0.5 * (limits.lower_position + limits.upper_position));
  }
  robot.forward();
}

class CapturingIk final : public IArmVelocityIk {
 public:
  ArmIkResult solve(const ArmIkInput& input) override {
    ++solve_count;
    last_input = input;
    ArmIkResult result;
    result.status = SolverStatus::kSolved;
    result.qdot.setZero();
    result.slack = input.desired_twist;
    result.equality_residual = 0.0;
    return result;
  }
  void reset() override {}
  IkAlgorithm algorithm() const noexcept override {
    return IkAlgorithm::kHierarchicalQp;
  }

  ArmIkInput last_input;
  int solve_count{0};
};

DualArmReferences referencesAtRobot(MujocoRobot& robot) {
  robot.forward();
  DualArmReferences references;
  references.left.pose = robot.tcpPose(ArmSide::kLeft);
  references.right.pose = robot.tcpPose(ArmSide::kRight);
  return references;
}

TEST(OtgController, SendsReferenceTwistExactlyOnceToVelocityQp) {
  MujocoRobot robot(modelPath());
  initializeAtMidpoint(robot);
  auto left = std::make_unique<CapturingIk>();
  auto right = std::make_unique<CapturingIk>();
  CapturingIk* left_observer = left.get();
  DualArmController controller(robot, testConfig(),
                               IkAlgorithm::kHierarchicalQp,
                               std::move(left), std::move(right));
  DualArmReferences references = referencesAtRobot(robot);
  references.left.twist << 0.2, -0.1, 0.05, 0.0, 0.0, 0.3;

  const ControllerDiagnostics diagnostics = controller.step(references, 0.005);

  ASSERT_TRUE(diagnostics.accepted);
  EXPECT_TRUE(left_observer->last_input.desired_twist.isApprox(
      references.left.twist, 1e-12));
  EXPECT_TRUE(diagnostics.left.reference.twist.isApprox(
      references.left.twist));
}

TEST(OtgController, InvalidReferenceStopsOnlyAffectedArm) {
  MujocoRobot robot(modelPath());
  initializeAtMidpoint(robot);
  auto left = std::make_unique<CapturingIk>();
  auto right = std::make_unique<CapturingIk>();
  CapturingIk* left_observer = left.get();
  CapturingIk* right_observer = right.get();
  DualArmController controller(robot, testConfig(),
                               IkAlgorithm::kHierarchicalQp,
                               std::move(left), std::move(right));
  DualArmReferences references = referencesAtRobot(robot);
  references.left.valid = false;
  references.right.twist.x() = 0.1;

  const ControllerDiagnostics diagnostics = controller.step(references, 0.005);

  EXPECT_FALSE(diagnostics.accepted);
  EXPECT_FALSE(diagnostics.left.accepted);
  EXPECT_TRUE(diagnostics.right.accepted);
  EXPECT_EQ(left_observer->solve_count, 0);
  EXPECT_EQ(right_observer->solve_count, 1);
}

TEST(OtgController, GeneratedReferenceRunsThroughUnchangedQp) {
  MujocoRobot robot(modelPath());
  initializeAtMidpoint(robot);
  const QpIkConfig config = testConfig();
  DualArmController controller(robot, config);
  DualArmReferences references = referencesAtRobot(robot);
  CartesianReferenceGenerator left_generator(config.cartesian_otg, 0.005);
  CartesianReferenceGenerator right_generator(config.cartesian_otg, 0.005);
  left_generator.reset(references.left.pose);
  right_generator.reset(references.right.pose);
  const Eigen::Vector3d left_start = references.left.pose.position;
  Pose left_target = references.left.pose;
  left_target.position.x() += 0.05;

  for (int step = 0; step < 200; ++step) {
    references.left =
        left_generator.update(left_target, Vec6::Zero(), false, 0.005);
    references.right = right_generator.update(
        references.right.pose, Vec6::Zero(), false, 0.005);
    ASSERT_TRUE(controller.step(references, 0.005).left.accepted)
        << "step=" << step;
  }
  robot.forward();
  EXPECT_GT((robot.tcpPose(ArmSide::kLeft).position - left_start).norm(),
            0.005);
}

}  // namespace
}  // namespace tianji_qp_ik
