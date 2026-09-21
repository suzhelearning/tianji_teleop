#include "tianji_qp_ik/arm_angle.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>

namespace tianji_qp_ik {
namespace {

TEST(ArmAngleReferenceModeTest, TogglesBothDirections) {
  EXPECT_EQ(toggleArmAngleReferenceMode(ArmAngleReferenceMode::kPico),
            ArmAngleReferenceMode::kDefaultDown);
  EXPECT_EQ(toggleArmAngleReferenceMode(ArmAngleReferenceMode::kDefaultDown),
            ArmAngleReferenceMode::kOutwardOnly);
  EXPECT_EQ(toggleArmAngleReferenceMode(ArmAngleReferenceMode::kOutwardOnly),
            ArmAngleReferenceMode::kPicoOutward);
  EXPECT_EQ(toggleArmAngleReferenceMode(ArmAngleReferenceMode::kPicoOutward),
            ArmAngleReferenceMode::kPico);
  EXPECT_EQ(toString(ArmAngleReferenceMode::kOutwardOnly), "outward_only");
  EXPECT_EQ(toString(ArmAngleReferenceMode::kPicoOutward), "pico_outward");
}

TEST(ArmAngleReferenceModeTest, ParsesCliNames) {
  EXPECT_EQ(armAngleReferenceModeFromString("pico"),
            ArmAngleReferenceMode::kPico);
  EXPECT_EQ(armAngleReferenceModeFromString("default_down"),
            ArmAngleReferenceMode::kDefaultDown);
  EXPECT_EQ(armAngleReferenceModeFromString("outward_only"),
            ArmAngleReferenceMode::kOutwardOnly);
  EXPECT_EQ(armAngleReferenceModeFromString("pico_outward"),
            ArmAngleReferenceMode::kPicoOutward);
  EXPECT_THROW(armAngleReferenceModeFromString("unknown"),
               std::invalid_argument);
}

TEST(ArmAngleReferenceModeTest, PicoOutwardCombinesPicoReferenceAndBarrier) {
  EXPECT_TRUE(usesPicoArmDirection(ArmAngleReferenceMode::kPicoOutward));
  EXPECT_TRUE(usesOutwardArmBarrier(ArmAngleReferenceMode::kPicoOutward));
  EXPECT_FALSE(
      usesContinuityArmDirection(ArmAngleReferenceMode::kPicoOutward));
  EXPECT_FALSE(usesOutwardArmBarrier(ArmAngleReferenceMode::kPico));
  EXPECT_TRUE(
      usesContinuityArmDirection(ArmAngleReferenceMode::kOutwardOnly));

  DualArmDirectionReferences pico;
  pico.left = {true, Eigen::Vector3d::UnitX(),
               ArmDirectionReferenceSource::kPico};
  pico.right = pico.left;
  const auto selected = selectArmDirectionReferences(
      ArmAngleReferenceMode::kPicoOutward, true, pico);
  EXPECT_EQ(selected.left.source, ArmDirectionReferenceSource::kPico);
  EXPECT_TRUE(selected.left.direction.isApprox(Eigen::Vector3d::UnitX()));
}

TEST(ArmAngleReferenceModeTest, KeepsLastValidPicoDirectionWhenStreamIsStale) {
  DualArmDirectionReferences pico;
  pico.left = {true, Eigen::Vector3d::UnitX(),
               ArmDirectionReferenceSource::kPico};
  pico.right = pico.left;

  const DualArmDirectionReferences selected_pico =
      selectArmDirectionReferences(ArmAngleReferenceMode::kPico, true, pico);
  EXPECT_EQ(selected_pico.left.source, ArmDirectionReferenceSource::kPico);
  EXPECT_TRUE(selected_pico.left.direction.isApprox(Eigen::Vector3d::UnitX()));

  const DualArmDirectionReferences selected_default =
      selectArmDirectionReferences(ArmAngleReferenceMode::kDefaultDown, true,
                                   pico);
  EXPECT_EQ(selected_default.left.source,
            ArmDirectionReferenceSource::kDefaultDown);
  EXPECT_TRUE(selected_default.left.direction.isApprox(
      -Eigen::Vector3d::UnitZ()));

  const DualArmDirectionReferences selected_stale =
      selectArmDirectionReferences(ArmAngleReferenceMode::kPico, false, pico);
  EXPECT_EQ(selected_stale.left.source, ArmDirectionReferenceSource::kPico);
  EXPECT_TRUE(selected_stale.left.direction.isApprox(
      Eigen::Vector3d::UnitX()));
}

TEST(ArmAngleReferenceModeTest, OutwardOnlyDoesNotConsumePicoDirection) {
  DualArmDirectionReferences pico;
  pico.left = {true, Eigen::Vector3d::UnitX(),
               ArmDirectionReferenceSource::kPico};
  pico.right = pico.left;
  const auto selected = selectArmDirectionReferences(
      ArmAngleReferenceMode::kOutwardOnly, true, pico);
  EXPECT_EQ(selected.left.source, ArmDirectionReferenceSource::kDefaultDown);
  EXPECT_EQ(selected.right.source, ArmDirectionReferenceSource::kDefaultDown);
}

TEST(ArmDirectionReferenceManager, DefaultsToWorldDownWithoutPico) {
  ArmDirectionReferenceManager manager(8.0);
  const DualArmDirectionReferences output = manager.update({}, 0.005);

  ASSERT_TRUE(output.left.valid);
  ASSERT_TRUE(output.right.valid);
  EXPECT_TRUE(output.left.direction.isApprox(-Eigen::Vector3d::UnitZ()));
  EXPECT_TRUE(output.right.direction.isApprox(-Eigen::Vector3d::UnitZ()));
  EXPECT_EQ(output.left.source, ArmDirectionReferenceSource::kDefaultDown);
}

TEST(ArmDirectionReferenceManager, LimitsPicoDirectionAngularRate) {
  ArmDirectionReferenceManager manager(2.0);
  ASSERT_TRUE(manager.update({}, 0.1).left.valid);
  DualArmDirectionReferences requested;
  requested.left = {true, Eigen::Vector3d::UnitX(),
                    ArmDirectionReferenceSource::kPico};
  requested.right = requested.left;

  const DualArmDirectionReferences output = manager.update(requested, 0.1);
  const double moved = std::acos(std::clamp(
      output.left.direction.dot(-Eigen::Vector3d::UnitZ()), -1.0, 1.0));
  EXPECT_NEAR(moved, 0.2, 1e-12);
  EXPECT_TRUE(output.left.valid);
  EXPECT_EQ(output.left.source, ArmDirectionReferenceSource::kPico);
}

TEST(ArmDirectionReferenceManager, HoldsPreviousWhenLivePicoBecomesInvalid) {
  ArmDirectionReferenceManager manager(100.0);
  DualArmDirectionReferences requested;
  requested.left = {true, Eigen::Vector3d::UnitY(),
                    ArmDirectionReferenceSource::kPico};
  requested.right = requested.left;
  ASSERT_TRUE(manager.update(requested, 0.1).left.direction.isApprox(
      Eigen::Vector3d::UnitY()));

  const DualArmDirectionReferences output = manager.update({}, 0.1);
  EXPECT_TRUE(output.left.direction.isApprox(Eigen::Vector3d::UnitY()));
  EXPECT_EQ(output.left.source, ArmDirectionReferenceSource::kPrevious);
}

TEST(ArmDirectionReferenceManager, HoldsLastPicoWhenPicoIsStale) {
  ArmDirectionReferenceManager manager(100.0);
  DualArmDirectionReferences requested;
  requested.left = {true, Eigen::Vector3d::UnitY(),
                    ArmDirectionReferenceSource::kPico};
  requested.right = requested.left;
  ASSERT_TRUE(manager.update(requested, 0.1).left.direction.isApprox(
      Eigen::Vector3d::UnitY()));

  const DualArmDirectionReferences selected = selectArmDirectionReferences(
      ArmAngleReferenceMode::kPico, false, requested);
  const DualArmDirectionReferences output = manager.update(selected, 0.1);

  EXPECT_TRUE(output.left.direction.isApprox(Eigen::Vector3d::UnitY()));
  EXPECT_EQ(output.left.source, ArmDirectionReferenceSource::kPico);
}

TEST(ArmDirectionReferenceManager, ResetRestartsContinuouslyFromWorldDown) {
  ArmDirectionReferenceManager manager(2.0);
  DualArmDirectionReferences requested;
  requested.left = {true, Eigen::Vector3d::UnitY(),
                    ArmDirectionReferenceSource::kPico};
  requested.right = requested.left;
  ASSERT_TRUE(manager.update(requested, 0.1).left.valid);

  manager.reset();
  requested.left.direction = Eigen::Vector3d::UnitX();
  requested.right.direction = Eigen::Vector3d::UnitX();
  const DualArmDirectionReferences output = manager.update(requested, 0.1);
  const double moved_from_down = std::acos(std::clamp(
      output.left.direction.dot(-Eigen::Vector3d::UnitZ()), -1.0, 1.0));
  EXPECT_LE(moved_from_down, 0.2 + 1e-12);
}

ArmAngleGeometryInput bentGeometry() {
  ArmAngleGeometryInput input;
  input.shoulder_position = Eigen::Vector3d::Zero();
  input.elbow_position = Eigen::Vector3d(0.0, 1.0, 0.0);
  input.wrist_position = Eigen::Vector3d(1.0, 0.0, 0.0);
  input.elbow_position_jacobian.setZero();
  input.elbow_position_jacobian(2, 6) = 1.0;
  return input;
}

double signedArmAngleError(const ArmAngleGeometryInput& geometry,
                           const Eigen::Vector3d& world_reference) {
  const Eigen::Vector3d axis =
      (geometry.wrist_position - geometry.shoulder_position).normalized();
  Eigen::Vector3d current =
      geometry.elbow_position - geometry.shoulder_position;
  current -= axis * axis.dot(current);
  current.normalize();
  Eigen::Vector3d projected =
      world_reference - axis * axis.dot(world_reference);
  projected.normalize();
  return std::atan2(axis.dot(current.cross(projected)),
                    current.dot(projected));
}

ArmAngleGeometryInput differentialGeometry() {
  ArmAngleGeometryInput input;
  input.shoulder_position = Eigen::Vector3d(0.10, -0.20, 0.30);
  input.elbow_position = Eigen::Vector3d(0.35, 0.25, 0.05);
  input.wrist_position = Eigen::Vector3d(0.75, -0.05, 0.45);
  for (int column = 0; column < kArmDof; ++column) {
    const double scale = static_cast<double>(column + 1);
    input.shoulder_position_jacobian.col(column) =
        Eigen::Vector3d(0.003 * scale, -0.002 * scale, 0.001 * scale);
    input.elbow_position_jacobian.col(column) = Eigen::Vector3d(
        0.010 * scale, 0.006 * (8.0 - scale), -0.004 * scale);
    input.wrist_position_jacobian.col(column) = Eigen::Vector3d(
        -0.005 * scale, 0.008 * scale, 0.003 * (8.0 - scale));
  }
  return input;
}

TEST(ArmAngleTaskBuilder, ComputesSignedErrorAndExactJacobian) {
  ArmAngleTaskBuilder builder(ArmSide::kLeft, 0.015, 0.050, 8.0);
  ArmDirectionReference reference{
      true, Eigen::Vector3d(0.0, 0.0, -1.0),
      ArmDirectionReferenceSource::kPico};

  const ArmAngleTask task = builder.compute(bentGeometry(), reference, 0.005);

  ASSERT_TRUE(task.active);
  EXPECT_NEAR(task.error_rad, -0.5 * 3.14159265358979323846, 1e-12);
  EXPECT_NEAR(task.radius_m, 1.0, 1e-12);
  EXPECT_NEAR(task.activation, 1.0, 1e-12);
  EXPECT_NEAR(task.jacobian[6], 1.0, 1e-12);
}

TEST(ArmAngleTaskBuilder, StandardPlaneZeroTracksEqualRobotAndPicoSwivel) {
  ArmAngleTaskBuilder builder(ArmSide::kLeft, 0.015, 0.050, 8.0);
  ArmDirectionReference reference{
      true, Eigen::Vector3d::UnitY(),
      ArmDirectionReferenceSource::kPico};
  reference.shoulder_to_wrist_axis_valid = true;
  reference.shoulder_to_wrist_axis = Eigen::Vector3d::UnitX();

  const ArmAngleTask task = builder.compute(bentGeometry(), reference, 0.005);

  ASSERT_TRUE(task.active);
  EXPECT_NEAR(task.robot_angle_rad, 0.5 * 3.14159265358979323846, 1e-12);
  EXPECT_NEAR(task.target_angle_rad, task.robot_angle_rad, 1e-12);
  EXPECT_NEAR(task.error_rad, 0.0, 1e-12);
}

TEST(ArmAngleTaskBuilder, StandardPlaneUsesThePicoTargetAxis) {
  ArmAngleTaskBuilder builder(ArmSide::kRight, 0.015, 0.050, 8.0);
  ArmAngleGeometryInput geometry = bentGeometry();
  ArmDirectionReference reference{
      true, (Eigen::Vector3d::UnitX() - Eigen::Vector3d::UnitZ()).normalized(),
      ArmDirectionReferenceSource::kPico};
  reference.shoulder_to_wrist_axis_valid = true;
  reference.shoulder_to_wrist_axis = Eigen::Vector3d::UnitY();

  const ArmAngleTask task = builder.compute(geometry, reference, 0.005);

  ASSERT_TRUE(task.active);
  EXPECT_NEAR(task.robot_angle_rad, 0.5 * 3.14159265358979323846, 1e-12);
  EXPECT_NEAR(task.target_angle_rad, -0.25 * 3.14159265358979323846,
              1e-12);
  EXPECT_NEAR(task.error_rad, -0.75 * 3.14159265358979323846, 1e-12);
}

TEST(ArmAngleTaskBuilder, ContinuityCapturesCurrentElbowPlane) {
  ArmAngleTaskBuilder builder(ArmSide::kLeft, 0.015, 0.050, 8.0);

  const ArmAngleTask task = builder.computeContinuity(bentGeometry(), 0.005);

  ASSERT_TRUE(task.active);
  EXPECT_EQ(task.reference_source, ArmDirectionReferenceSource::kPrevious);
  EXPECT_NEAR(task.error_rad, 0.0, 1e-12);
  EXPECT_TRUE(task.projected_reference.isApprox(Eigen::Vector3d::UnitY()));
}

TEST(ArmAngleTaskBuilder, ContinuityOpposesElbowPlaneRotation) {
  ArmAngleTaskBuilder builder(ArmSide::kLeft, 0.015, 0.050, 8.0);
  ASSERT_TRUE(builder.computeContinuity(bentGeometry(), 0.005).active);

  ArmAngleGeometryInput rotated = bentGeometry();
  rotated.elbow_position = Eigen::Vector3d(0.0, 0.0, 1.0);
  rotated.elbow_position_jacobian.setZero();
  rotated.elbow_position_jacobian(1, 6) = 1.0;
  const ArmAngleTask task = builder.computeContinuity(rotated, 0.005);

  ASSERT_TRUE(task.active);
  EXPECT_EQ(task.reference_source, ArmDirectionReferenceSource::kPrevious);
  EXPECT_NEAR(task.error_rad, -0.5 * 3.14159265358979323846, 1e-12);
}

TEST(ArmAngleTaskBuilder, ContinuityResetRecapturesCurrentElbowPlane) {
  ArmAngleTaskBuilder builder(ArmSide::kLeft, 0.015, 0.050, 8.0);
  ASSERT_TRUE(builder.computeContinuity(bentGeometry(), 0.005).active);
  builder.reset();

  ArmAngleGeometryInput rotated = bentGeometry();
  rotated.elbow_position = Eigen::Vector3d(0.0, 0.0, 1.0);
  rotated.elbow_position_jacobian.setZero();
  rotated.elbow_position_jacobian(1, 6) = 1.0;
  const ArmAngleTask task = builder.computeContinuity(rotated, 0.005);

  ASSERT_TRUE(task.active);
  EXPECT_NEAR(task.error_rad, 0.0, 1e-12);
  EXPECT_TRUE(task.projected_reference.isApprox(Eigen::Vector3d::UnitZ()));
}

TEST(ArmAngleTaskBuilder, HoldsControlDirectionAcrossTheS1BranchCut) {
  ArmAngleTaskBuilder builder(ArmSide::kLeft, 0.015, 0.050, 8.0);
  const ArmDirectionReference reference{
      true, Eigen::Vector3d::UnitY(), ArmDirectionReferenceSource::kPico};
  const auto geometry_at = [](double angle) {
    ArmAngleGeometryInput geometry;
    geometry.shoulder_position.setZero();
    geometry.wrist_position = Eigen::Vector3d::UnitX();
    geometry.elbow_position =
        Eigen::Vector3d(0.0, std::cos(angle), std::sin(angle));
    geometry.elbow_position_jacobian.col(6) =
        Eigen::Vector3d(0.0, -std::sin(angle), std::cos(angle));
    return geometry;
  };

  const ArmAngleTask before =
      builder.compute(geometry_at(-3.13), reference, 0.005);
  const ArmAngleTask after =
      builder.compute(geometry_at(-3.1531853071795863), reference, 0.005);
  const ArmAngleTask near_equivalent_alignment =
      builder.compute(geometry_at(-6.20), reference, 0.005);

  ASSERT_TRUE(before.active);
  ASSERT_TRUE(after.active);
  ASSERT_TRUE(near_equivalent_alignment.active);
  EXPECT_GT(before.error_rad, 3.0);
  EXPECT_LT(after.error_rad, -3.0);
  EXPECT_GT(before.control_error_rad, 3.0);
  // The physical diagnostic remains wrapped, but the controller must not
  // reverse direction on every sample while noise straddles +/-pi.
  EXPECT_GT(after.control_error_rad, 3.0);
  EXPECT_LE(std::abs(before.control_error_rad),
            3.14159265358979323846);
  EXPECT_LE(std::abs(after.control_error_rad),
            3.14159265358979323846);
  EXPECT_NEAR(before.control_error_rad, before.error_rad, 1.0e-12);
  EXPECT_NEAR(std::abs(after.control_error_rad), std::abs(after.error_rad),
              1.0e-12);
  // A full 2*pi-equivalent alignment returns directly to zero instead of
  // commanding another elbow revolution.
  EXPECT_NEAR(near_equivalent_alignment.control_error_rad,
              near_equivalent_alignment.error_rad, 1.0e-12);
  EXPECT_LT(std::abs(near_equivalent_alignment.control_error_rad), 0.10);
}

TEST(ArmAngleAcceleration, EstimatesJacobianDerivativeContribution) {
  Vec7 previous = Vec7::Zero();
  Vec7 current = Vec7::Zero();
  Vec7 qdot = Vec7::Zero();
  current[2] = 0.20;
  previous[2] = 0.10;
  qdot[2] = 2.0;

  EXPECT_NEAR(estimateArmAngleJacobianDotTimesVelocity(
                  current, previous, qdot, 0.01, true),
              20.0, 1e-12);
  EXPECT_DOUBLE_EQ(estimateArmAngleJacobianDotTimesVelocity(
                       current, previous, qdot, 0.01, false),
                   0.0);
}

TEST(ArmAngleTaskBuilder, JacobianMatchesCompleteSignedAngleDifferential) {
  ArmAngleTaskBuilder builder(ArmSide::kLeft, 0.015, 0.050, 8.0);
  const ArmAngleGeometryInput geometry = differentialGeometry();
  const Eigen::Vector3d world_reference =
      Eigen::Vector3d(0.20, 0.40, -0.70).normalized();
  const ArmDirectionReference reference{
      true, world_reference, ArmDirectionReferenceSource::kPico};
  const ArmAngleTask task = builder.compute(geometry, reference, 0.005);
  ASSERT_TRUE(task.active);

  constexpr double kStep = 1e-6;
  for (int column = 0; column < kArmDof; ++column) {
    ArmAngleGeometryInput plus = geometry;
    ArmAngleGeometryInput minus = geometry;
    plus.shoulder_position +=
        kStep * geometry.shoulder_position_jacobian.col(column);
    plus.elbow_position +=
        kStep * geometry.elbow_position_jacobian.col(column);
    plus.wrist_position +=
        kStep * geometry.wrist_position_jacobian.col(column);
    minus.shoulder_position -=
        kStep * geometry.shoulder_position_jacobian.col(column);
    minus.elbow_position -=
        kStep * geometry.elbow_position_jacobian.col(column);
    minus.wrist_position -=
        kStep * geometry.wrist_position_jacobian.col(column);
    double error_delta = signedArmAngleError(plus, world_reference) -
                         signedArmAngleError(minus, world_reference);
    if (error_delta > 3.14159265358979323846) {
      error_delta -= 2.0 * 3.14159265358979323846;
    } else if (error_delta < -3.14159265358979323846) {
      error_delta += 2.0 * 3.14159265358979323846;
    }
    const double expected_current_rate =
        -error_delta / (2.0 * kStep);
    EXPECT_NEAR(task.jacobian[column], expected_current_rate, 1e-8)
        << "column=" << column;
  }
}

TEST(ArmAngleTaskBuilder, FadesNearStraightAndDisablesBelowMinimum) {
  ArmAngleTaskBuilder builder(ArmSide::kLeft, 0.015, 0.050, 8.0);
  ArmDirectionReference reference{
      true, -Eigen::Vector3d::UnitZ(),
      ArmDirectionReferenceSource::kDefaultDown};
  ArmAngleGeometryInput input = bentGeometry();
  input.elbow_position = Eigen::Vector3d(0.5, 0.030, 0.0);
  const ArmAngleTask faded = builder.compute(input, reference, 0.005);
  EXPECT_TRUE(faded.active);
  EXPECT_NEAR(faded.activation, (0.030 - 0.015) / (0.050 - 0.015),
              1e-12);

  input.elbow_position.y() = 0.010;
  const ArmAngleTask disabled = builder.compute(input, reference, 0.005);
  EXPECT_FALSE(disabled.active);
  EXPECT_DOUBLE_EQ(disabled.activation, 0.0);
}

TEST(ArmAngleTaskBuilder, LocksElbowBranchWhenPlaneRadiusCollapses) {
  ArmAngleTaskBuilder builder(ArmSide::kLeft, 0.015, 0.050, 8.0);
  ArmDirectionReference reference{
      true, Eigen::Vector3d::UnitY(), ArmDirectionReferenceSource::kPico};

  ArmAngleGeometryInput reliable = bentGeometry();
  reliable.elbow_position = Eigen::Vector3d(0.5, 0.08, 0.0);
  reliable.elbow_position_jacobian.setZero();
  reliable.elbow_position_jacobian(1, 6) = 1.0;
  reliable.elbow_position_jacobian(2, 5) = 1.0;
  static_cast<void>(builder.compute(reliable, reference, 0.005));

  ArmAngleGeometryInput collapsed = reliable;
  collapsed.elbow_position.y() = -0.002;
  const ArmAngleTask task = builder.compute(collapsed, reference, 0.005);

  EXPECT_FALSE(task.active);
  EXPECT_TRUE(task.branch_lock_active);
  EXPECT_TRUE(task.branch_lock_jacobian.allFinite());
  EXPECT_GT(task.branch_lock_jacobian.norm(), 1e-9);
  EXPECT_NEAR(task.branch_lock_jacobian[6], 1.0, 1e-12);
  EXPECT_NEAR(task.branch_lock_distance_m, -0.017, 1e-12);
}

TEST(ArmAngleTaskBuilder, BranchLockCreatesFiniteVelocityAndAccelerationBounds) {
  ArmAngleTaskBuilder builder(ArmSide::kLeft, 0.015, 0.050, 8.0);
  ArmDirectionReference reference{
      true, Eigen::Vector3d::UnitY(), ArmDirectionReferenceSource::kPico};
  ArmAngleGeometryInput reliable = bentGeometry();
  reliable.elbow_position = Eigen::Vector3d(0.5, 0.08, 0.0);
  reliable.elbow_position_jacobian.setZero();
  reliable.elbow_position_jacobian(1, 6) = 1.0;
  reliable.elbow_position_jacobian(2, 5) = 1.0;
  static_cast<void>(builder.compute(reliable, reference, 0.005));

  ArmAngleGeometryInput collapsed = reliable;
  collapsed.elbow_position.y() = -0.002;
  const ArmAngleTask task = builder.compute(collapsed, reference, 0.005);
  const Vec7 lower = Vec7::Constant(-10.0);
  const Vec7 upper = Vec7::Constant(10.0);
  const auto velocity = makeArmAngleBranchLockVelocityConstraint(
      task, lower, upper, 8.0);
  const auto acceleration = makeArmAngleBranchLockAccelerationConstraint(
      task, Vec7::Zero(), 0.0, lower, upper, 100.0, 20.0);

  ASSERT_TRUE(velocity.active);
  ASSERT_TRUE(acceleration.active);
  EXPECT_NEAR(velocity.requested_lower, 0.136, 1e-12);
  EXPECT_NEAR(acceleration.requested_lower, 1.7, 1e-12);
  EXPECT_TRUE(velocity.jacobian.isApprox(task.branch_lock_jacobian));
  EXPECT_TRUE(acceleration.jacobian.isApprox(task.branch_lock_jacobian));

  const auto clipped = makeArmAngleBranchLockAccelerationConstraint(
      task, Vec7::Zero(), 0.0, Vec7::Constant(-0.01),
      Vec7::Constant(0.01), 100.0, 20.0);
  EXPECT_TRUE(clipped.active);
  EXPECT_TRUE(clipped.feasibility_clipped);
  EXPECT_NEAR(clipped.lower, 0.009, 1e-12);
}

TEST(ArmAngleTaskBuilder,
     ConfiguredBranchLockRadiusPreservesAnObservableElbowPlane) {
  ArmAngleTaskBuilder builder(ArmSide::kLeft, 0.015, 0.050, 8.0,
                              0.05, 0.10, 0.20, 0.040);
  ArmDirectionReference reference{
      true, Eigen::Vector3d::UnitY(), ArmDirectionReferenceSource::kPico};
  ArmAngleGeometryInput reliable = bentGeometry();
  reliable.elbow_position = Eigen::Vector3d(0.5, 0.08, 0.0);
  reliable.elbow_position_jacobian.setZero();
  reliable.elbow_position_jacobian(1, 6) = 1.0;
  reliable.elbow_position_jacobian(2, 5) = 1.0;
  static_cast<void>(builder.compute(reliable, reference, 0.005));

  ArmAngleGeometryInput approaching_straight = reliable;
  approaching_straight.elbow_position.y() = 0.030;
  const ArmAngleTask task =
      builder.compute(approaching_straight, reference, 0.005);
  const LinearJointConstraint constraint =
      makeArmAngleBranchLockVelocityConstraint(
          task, Vec7::Constant(-10.0), Vec7::Constant(10.0), 8.0);

  EXPECT_TRUE(task.active);
  EXPECT_TRUE(task.branch_lock_active);
  EXPECT_NEAR(task.branch_lock_distance_m, -0.010, 1.0e-12);
  ASSERT_TRUE(constraint.active);
  EXPECT_NEAR(constraint.requested_lower, 0.080, 1.0e-12);
}

TEST(ArmAngleTaskBuilder, UsesDeterministicOutwardDirectionWhenProjectionDegenerates) {
  ArmAngleTaskBuilder left(ArmSide::kLeft, 0.015, 0.050, 8.0);
  ArmAngleTaskBuilder right(ArmSide::kRight, 0.015, 0.050, 8.0);
  const ArmDirectionReference parallel{
      true, Eigen::Vector3d::UnitX(), ArmDirectionReferenceSource::kPico};

  const ArmAngleTask left_task = left.compute(bentGeometry(), parallel, 0.005);
  const ArmAngleTask right_task = right.compute(bentGeometry(), parallel, 0.005);
  EXPECT_EQ(left_task.reference_source,
            ArmDirectionReferenceSource::kDegenerate);
  EXPECT_EQ(right_task.reference_source,
            ArmDirectionReferenceSource::kDegenerate);
  EXPECT_TRUE(left_task.projected_reference.isApprox(
      Eigen::Vector3d::UnitY()));
  EXPECT_TRUE(right_task.projected_reference.isApprox(
      -Eigen::Vector3d::UnitY()));
}

TEST(ArmAngleTaskBuilder, RateLimitsProjectionAcrossAxisSingularity) {
  ArmAngleTaskBuilder builder(ArmSide::kLeft, 0.015, 0.050, 2.0);
  const ArmAngleGeometryInput geometry = bentGeometry();
  const ArmDirectionReference initial{
      true, Eigen::Vector3d::UnitY(), ArmDirectionReferenceSource::kPico};
  const ArmAngleTask before = builder.compute(geometry, initial, 0.05);
  ASSERT_TRUE(before.active);

  const ArmDirectionReference across_axis{
      true, (Eigen::Vector3d::UnitX() -
             1e-2 * Eigen::Vector3d::UnitY()).normalized(),
      ArmDirectionReferenceSource::kPico};
  const ArmAngleTask after = builder.compute(geometry, across_axis, 0.05);
  ASSERT_TRUE(after.active);
  const double projected_change = std::acos(std::clamp(
      before.projected_reference.dot(after.projected_reference), -1.0, 1.0));
  EXPECT_LE(projected_change, 0.1 + 1e-12);
  EXPECT_DOUBLE_EQ(after.reference_rate_rad_s, 0.0);
}

TEST(ArmAngleTaskBuilder, ReportsRateOfFinalProjectedReference) {
  ArmAngleTaskBuilder builder(ArmSide::kLeft, 0.015, 0.050, 2.0);
  const ArmAngleGeometryInput geometry = bentGeometry();
  const ArmDirectionReference initial{
      true, Eigen::Vector3d::UnitY(), ArmDirectionReferenceSource::kPico};
  ASSERT_TRUE(builder.compute(geometry, initial, 0.05).active);

  const ArmDirectionReference rotating{
      true, Eigen::Vector3d::UnitZ(), ArmDirectionReferenceSource::kPico};
  const ArmAngleTask after = builder.compute(geometry, rotating, 0.05);

  ASSERT_TRUE(after.active);
  EXPECT_NEAR(after.reference_rate_rad_s, 2.0, 1e-12);
}

TEST(ArmAngleTrackingEnvelope, BoundsRateWithoutChangingReference) {
  ArmAngleTask task;
  task.active = true;
  task.jacobian[6] = 1.0;
  task.control_error_rad = -1.40;
  task.reference_rate_rad_s = 2.0;

  const LinearJointConstraint constraint =
      makeArmAngleTrackingEnvelopeVelocityConstraint(
          task, Vec7::Constant(-10.0), Vec7::Constant(10.0), 1.50, 8.0);

  ASSERT_TRUE(constraint.active);
  EXPECT_TRUE(constraint.jacobian.isApprox(task.jacobian));
  EXPECT_NEAR(constraint.requested_lower, -21.2, 1e-12);
  EXPECT_NEAR(constraint.requested_upper, 2.8, 1e-12);
  EXPECT_NEAR(constraint.lower, -21.2, 1e-12);
  EXPECT_NEAR(constraint.upper, 2.8, 1e-12);
  EXPECT_FALSE(constraint.feasibility_clipped);
}

TEST(ArmAngleTrackingEnvelope, ClipsUnreachableRecoveryIntoJointBoxInterior) {
  ArmAngleTask task;
  task.active = true;
  task.jacobian[6] = 1.0;
  task.control_error_rad = 2.0;
  task.reference_rate_rad_s = 0.0;

  const LinearJointConstraint constraint =
      makeArmAngleTrackingEnvelopeVelocityConstraint(
          task, Vec7::Constant(-0.10), Vec7::Constant(0.10), 1.50, 8.0);

  ASSERT_TRUE(constraint.active);
  EXPECT_TRUE(constraint.feasibility_clipped);
  EXPECT_NEAR(constraint.requested_lower, 4.0, 1e-12);
  EXPECT_NEAR(constraint.requested_upper, 28.0, 1e-12);
  EXPECT_NEAR(constraint.lower, 0.09, 1e-12);
  EXPECT_TRUE(std::isinf(constraint.upper));
}

TEST(ArmAngleTaskBuilder, HoldsDegenerateProjectionWithHysteresis) {
  ArmAngleTaskBuilder builder(ArmSide::kLeft, 0.015, 0.050, 2.0, 0.05,
                              0.10);
  const ArmAngleGeometryInput geometry = bentGeometry();
  const ArmDirectionReference initial{
      true, Eigen::Vector3d::UnitY(), ArmDirectionReferenceSource::kPico};
  const ArmAngleTask before = builder.compute(geometry, initial, 0.05);
  ASSERT_TRUE(before.active);

  const ArmDirectionReference below_enter{
      true,
      (Eigen::Vector3d::UnitX() +
       0.03 * Eigen::Vector3d::UnitZ()).normalized(),
      ArmDirectionReferenceSource::kPico};
  const ArmAngleTask held = builder.compute(geometry, below_enter, 0.05);
  ASSERT_TRUE(held.active);
  EXPECT_TRUE(held.reference_projection_held);
  EXPECT_EQ(held.reference_source, ArmDirectionReferenceSource::kPrevious);
  EXPECT_TRUE(held.projected_reference.isApprox(before.projected_reference,
                                                1e-12));
  EXPECT_LT(held.requested_reference_projection_norm, 0.05);
  EXPECT_NEAR(held.jacobian_norm, held.jacobian.norm(), 1e-12);
  EXPECT_LT(held.jacobian_norm, 100.0);

  const ArmDirectionReference between_thresholds{
      true,
      (Eigen::Vector3d::UnitX() +
       0.07 * Eigen::Vector3d::UnitZ()).normalized(),
      ArmDirectionReferenceSource::kPico};
  const ArmAngleTask still_held =
      builder.compute(geometry, between_thresholds, 0.05);
  EXPECT_TRUE(still_held.reference_projection_held);
  EXPECT_TRUE(still_held.projected_reference.isApprox(
      before.projected_reference, 1e-12));

  const ArmDirectionReference above_exit{
      true,
      (Eigen::Vector3d::UnitX() +
       0.20 * Eigen::Vector3d::UnitZ()).normalized(),
      ArmDirectionReferenceSource::kPico};
  const ArmAngleTask recovered = builder.compute(geometry, above_exit, 0.05);
  EXPECT_FALSE(recovered.reference_projection_held);
  const double recovered_change = std::acos(std::clamp(
      still_held.projected_reference.dot(recovered.projected_reference),
      -1.0, 1.0));
  EXPECT_LE(recovered_change, 0.1 + 1e-12);
  EXPECT_LT(recovered.jacobian_norm, 100.0);
}

TEST(ArmAngleTaskBuilder,
     ReferenceGovernorPreventsAnUntrackablePicoDirectionFromReachingPi) {
  constexpr double kGovernorEnterError = 0.80;
  constexpr double kGovernorExitError = 0.40;
  constexpr double kGovernorTrackingError = 0.15;
  ArmAngleTaskBuilder builder(
      ArmSide::kRight, 0.015, 0.050, 2.0, 0.05, 0.10, 0.20, 0.015,
      true, kGovernorEnterError, kGovernorExitError,
      kGovernorTrackingError);
  ArmAngleGeometryInput geometry = bentGeometry();
  const ArmDirectionReference aligned{
      true, Eigen::Vector3d::UnitY(), ArmDirectionReferenceSource::kPico};
  ASSERT_TRUE(builder.compute(geometry, aligned, 0.005).active);

  const ArmDirectionReference unreachable{
      true, -Eigen::Vector3d::UnitY(), ArmDirectionReferenceSource::kPico};
  ArmAngleTask governed;
  for (int cycle = 0; cycle < 1000; ++cycle) {
    // The model is deliberately held fixed: the effective reference must not
    // keep integrating the raw PICO request all the way to the S1 antipode.
    governed = builder.compute(geometry, unreachable, 0.005);
  }

  ASSERT_TRUE(governed.active);
  EXPECT_TRUE(governed.reference_governor_held);
  EXPECT_LE(std::abs(governed.control_error_rad),
            kGovernorTrackingError + 1.0e-12);
  EXPECT_LT(std::abs(governed.control_error_rad),
            0.5 * 3.14159265358979323846);

  // Once the modeled elbow catches the governed reference, the Schmitt exit
  // threshold releases it and tracking of the original PICO direction
  // resumes without a reference jump.
  geometry.elbow_position = governed.projected_reference;
  const ArmDirectionReference caught_up{
      true, geometry.elbow_position.normalized(),
      ArmDirectionReferenceSource::kPico};
  const ArmAngleTask recovered =
      builder.compute(geometry, caught_up, 0.005);
  ASSERT_TRUE(recovered.active);
  EXPECT_FALSE(recovered.reference_governor_held);
  EXPECT_LE(std::abs(recovered.control_error_rad), 2.0 * 0.005 + 1.0e-12);
}

TEST(ArmAngleNullspace, TracksSecondaryWithoutChangingCartesianPrimary) {
  Mat67 jacobian = Mat67::Zero();
  jacobian.leftCols<6>() = Eigen::Matrix<double, 6, 6>::Identity();
  ScalarJointTask task;
  task.active = true;
  task.jacobian[6] = 1.0;
  task.target = 0.4;
  task.activation = 1.0;

  const ArmAngleNullspaceResult result = refineArmAngleInNullspace(
      Vec7::Zero(), jacobian, task, Vec7::Constant(-1.0),
      Vec7::Constant(1.0), 1e-10);

  ASSERT_TRUE(result.active);
  EXPECT_NEAR(result.value[6], 0.4, 1e-12);
  EXPECT_TRUE((jacobian * result.value).isZero(1e-12));
  EXPECT_NEAR(result.cartesian_residual, 0.0, 1e-12);
  EXPECT_NEAR(result.after, task.target, 1e-12);
}

TEST(ArmAngleNullspace, ClampsScalarCorrectionToJointBounds) {
  Mat67 jacobian = Mat67::Zero();
  jacobian.leftCols<6>() = Eigen::Matrix<double, 6, 6>::Identity();
  ScalarJointTask task;
  task.active = true;
  task.jacobian[6] = 1.0;
  task.target = 2.0;
  task.activation = 1.0;
  Vec7 upper = Vec7::Constant(1.0);
  upper[6] = 0.25;

  const ArmAngleNullspaceResult result = refineArmAngleInNullspace(
      Vec7::Zero(), jacobian, task, Vec7::Constant(-1.0), upper, 1e-10);

  ASSERT_TRUE(result.active);
  EXPECT_NEAR(result.value[6], 0.25, 1e-12);
  EXPECT_LE(result.value[6], upper[6]);
  EXPECT_TRUE((jacobian * result.value).isZero(1e-12));
}

TEST(ArmAngleNullspace, RecoversTowardJointCenterNearASoftLimit) {
  Mat67 jacobian = Mat67::Zero();
  jacobian.leftCols<6>() = Eigen::Matrix<double, 6, 6>::Identity();
  ScalarJointTask task;
  task.active = true;
  task.jacobian[6] = 1.0;
  task.target = 0.4;
  task.activation = 1.0;
  JointPositionGuard guard;
  guard.active = true;
  guard.position.setZero();
  guard.position[6] = 0.95;
  guard.lower.setConstant(-1.0);
  guard.upper.setConstant(1.0);
  guard.soft_margin_rad = 0.20;
  guard.recovery_gain_rad_s_per_rad = 1.0;

  const ArmAngleNullspaceResult result = refineArmAngleInNullspace(
      Vec7::Zero(), jacobian, task, Vec7::Constant(-1.0),
      Vec7::Constant(1.0), 1e-10, {}, guard);

  ASSERT_TRUE(result.active);
  // Tracking asks for +0.4 rad/s deeper into the upper limit.  Inside the
  // soft buffer, the strict-nullspace secondary objective must turn around
  // and recover toward the joint-range center instead of merely stopping at
  // the boundary.
  EXPECT_LT(result.value[6], 0.0);
  EXPECT_GE(result.value[6], -1.0);
  EXPECT_TRUE((jacobian * result.value).isZero(1.0e-12));
}

TEST(ArmAngleTask, WeightedTaskRecoversTowardJointCenterNearASoftLimit) {
  Mat67 jacobian = Mat67::Zero();
  jacobian.leftCols<6>() =
      Eigen::Matrix<double, 6, 6>::Identity();
  ScalarJointTask task;
  task.active = true;
  task.jacobian[6] = 1.0;
  task.target = 1.0;
  task.activation = 1.0;
  JointPositionGuard guard;
  guard.active = true;
  guard.position.setZero();
  guard.position[6] = 0.95;
  guard.lower.setConstant(-1.0);
  guard.upper.setConstant(1.0);
  guard.soft_margin_rad = 0.20;
  guard.recovery_gain_rad_s_per_rad = 2.0;

  const ScalarJointTask recovered = applyArmAngleJointLimitRecovery(
      task, jacobian, guard, 2.5);

  EXPECT_TRUE(recovered.active);
  EXPECT_LT(recovered.target, 0.0);
}

TEST(ArmAngleTask, WeightedRecoveryIgnoresLimitsOutsideArmAngleNullspace) {
  Mat67 jacobian = Mat67::Zero();
  jacobian.leftCols<6>() =
      Eigen::Matrix<double, 6, 6>::Identity();
  ScalarJointTask task;
  task.active = true;
  task.jacobian[6] = 1.0;
  task.target = 1.0;
  task.activation = 1.0;
  JointPositionGuard guard;
  guard.active = true;
  guard.position.setZero();
  guard.position[0] = 0.95;
  guard.lower.setConstant(-1.0);
  guard.upper.setConstant(1.0);
  guard.soft_margin_rad = 0.20;
  guard.recovery_gain_rad_s_per_rad = 2.0;

  const ScalarJointTask recovered = applyArmAngleJointLimitRecovery(
      task, jacobian, guard, 2.5);

  EXPECT_TRUE(recovered.active);
  EXPECT_DOUBLE_EQ(recovered.target, task.target);
}


TEST(ArmAngleNullspace, SelectsTaskEffectiveDirectionWhenJacobianIsRankDeficient) {
  Mat67 jacobian = Mat67::Zero();
  jacobian.topLeftCorner<5, 5>() =
      Eigen::Matrix<double, 5, 5>::Identity();
  ScalarJointTask task;
  task.active = true;
  task.jacobian[5] = 1.0;
  task.target = 0.3;
  task.activation = 1.0;

  const ArmAngleNullspaceResult result = refineArmAngleInNullspace(
      Vec7::Zero(), jacobian, task, Vec7::Constant(-1.0),
      Vec7::Constant(1.0), 1e-10);

  ASSERT_TRUE(result.active);
  EXPECT_NEAR(result.value[5], 0.3, 1e-12);
  EXPECT_TRUE((jacobian * result.value).isZero(1e-12));
}

TEST(ArmAngleNullspace, LeavesPrimaryUnchangedForDegenerateOrInvalidTask) {
  Mat67 jacobian = Mat67::Zero();
  jacobian.leftCols<6>() = Eigen::Matrix<double, 6, 6>::Identity();
  const Vec7 primary = Vec7::LinSpaced(-0.3, 0.3);
  ScalarJointTask task;
  task.active = true;
  task.jacobian[0] = 1.0;
  task.target = 1.0;
  task.activation = 1.0;

  const ArmAngleNullspaceResult degenerate = refineArmAngleInNullspace(
      primary, jacobian, task, Vec7::Constant(-1.0),
      Vec7::Constant(1.0), 1e-10);
  EXPECT_FALSE(degenerate.active);
  EXPECT_TRUE(degenerate.value.isApprox(primary, 0.0));

  task.target = std::numeric_limits<double>::quiet_NaN();
  const ArmAngleNullspaceResult invalid = refineArmAngleInNullspace(
      primary, jacobian, task, Vec7::Constant(-1.0),
      Vec7::Constant(1.0), 1e-10);
  EXPECT_FALSE(invalid.active);
  EXPECT_TRUE(invalid.value.isApprox(primary, 0.0));
}

}  // namespace
}  // namespace tianji_qp_ik
