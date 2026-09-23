#include "tianji_qp_ik/mujoco_joint_plot.hpp"

#include <gtest/gtest.h>

#include <string>

namespace tianji_qp_ik {
namespace {

TEST(MujocoJointPlotLayoutTest, BuildsTwoByFourPanelAndSeparateScene) {
  const JointPlotLayout layout = computeJointPlotLayout(1600, 900, true);
  ASSERT_TRUE(layout.visible);
  EXPECT_EQ(layout.scene.left, 0);
  EXPECT_EQ(layout.scene.bottom, 0);
  EXPECT_GT(layout.scene.width, 0);
  EXPECT_LT(layout.scene.width, 1600);
  EXPECT_EQ(layout.scene.width + layout.panel.width, 1600);
  EXPECT_EQ(layout.panels[0].bottom, layout.panels[1].bottom);
  EXPECT_EQ(layout.panels[0].width, layout.panels[1].width);
  EXPECT_GT(layout.panels[0].bottom, layout.panels[2].bottom);
  EXPECT_EQ(layout.status.left,
            layout.panels[6].left + layout.panels[6].width + 6);
}

TEST(MujocoJointPlotLayoutTest, HidesPanelWhenWindowIsTooSmallOrDisabled) {
  const JointPlotLayout small = computeJointPlotLayout(900, 600, true);
  EXPECT_FALSE(small.visible);
  EXPECT_EQ(small.scene.width, 900);
  EXPECT_EQ(small.scene.height, 600);

  const JointPlotLayout disabled = computeJointPlotLayout(1600, 900, false);
  EXPECT_FALSE(disabled.visible);
  EXPECT_EQ(disabled.scene.width, 1600);
}

TEST(MujocoJointPlotLayoutTest, HitTestOnlyConsumesVisiblePanel) {
  const JointPlotLayout layout = computeJointPlotLayout(1600, 900, true);
  EXPECT_FALSE(pointInJointPlotPanel(layout, layout.scene.width - 1, 100));
  EXPECT_TRUE(pointInJointPlotPanel(layout, layout.scene.width + 1, 100));
  EXPECT_FALSE(pointInJointPlotPanel(layout, 1700, 100));
}

TEST(MujocoJointPlotTest,
     RendersReferenceActualAndEffectiveBoundsForAllSevenJoints) {
  JointKinematicsHistory history(16U);
  for (int index = 0; index < 4; ++index) {
    JointKinematicsSample sample;
    sample.time_seconds = 0.005 * static_cast<double>(index);
    const double value = static_cast<double>(index);
    sample.left.reference.velocity.setConstant(1.0 + value);
    sample.left.actual.velocity.setConstant(0.5 + value);
    sample.right.reference.velocity.setConstant(-1.0 - value);
    sample.right.actual.velocity.setConstant(-0.5 - value);
    sample.left.bounds.velocity_lower.setConstant(-2.0 - value);
    sample.left.bounds.velocity_upper.setConstant(2.0 + value);
    history.push(sample);
  }

  MujocoJointPlot plot;
  plot.update(history, ArmSide::kLeft, PlotMetric::kVelocity, 5.0);
  for (int joint = 0; joint < kArmDof; ++joint) {
    const mjvFigure& figure = plot.figure(joint);
    EXPECT_EQ(figure.linepnt[0], 4);
    EXPECT_EQ(figure.linepnt[1], 4);
    EXPECT_EQ(figure.linepnt[2], 4);
    EXPECT_EQ(figure.linepnt[3], 4);
    EXPECT_FLOAT_EQ(figure.linedata[0][7], 4.0F);
    EXPECT_FLOAT_EQ(figure.linedata[1][7], 3.5F);
    EXPECT_FLOAT_EQ(figure.linedata[2][7], -5.0F);
    EXPECT_FLOAT_EQ(figure.linedata[3][7], 5.0F);
  }

  plot.update(history, ArmSide::kRight, PlotMetric::kVelocity, 5.0);
  EXPECT_FLOAT_EQ(plot.figure(0).linedata[0][7], -4.0F);
  EXPECT_FLOAT_EQ(plot.figure(0).linedata[1][7], -3.5F);
}

TEST(MujocoJointPlotTest, AppliesIndependentDerivativeValidity) {
  JointKinematicsHistory history(4U);
  JointKinematicsSample first;
  first.time_seconds = 0.0;
  first.left.reference.acceleration.setConstant(1.0);
  first.left.actual.acceleration.setConstant(2.0);
  first.left.reference_acceleration_valid = true;
  first.left.actual_acceleration_valid = false;
  history.push(first);

  JointKinematicsSample second = first;
  second.time_seconds = 0.005;
  second.left.reference_acceleration_valid = false;
  second.left.actual_acceleration_valid = true;
  history.push(second);

  MujocoJointPlot plot;
  plot.update(history, ArmSide::kLeft, PlotMetric::kAcceleration, 5.0);
  const mjvFigure& figure = plot.figure(0);
  EXPECT_EQ(figure.linepnt[0], 1);
  EXPECT_EQ(figure.linepnt[1], 1);
  EXPECT_EQ(figure.linepnt[2], 2);
  EXPECT_EQ(figure.linepnt[3], 2);
}


}  // namespace
}  // namespace tianji_qp_ik
