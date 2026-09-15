#pragma once

#include "tianji_qp_ik/joint_kinematics_plot.hpp"

#include <mujoco/mujoco.h>

#include <array>

namespace tianji_qp_ik {

struct JointPlotLayout {
  bool visible{false};
  mjrRect scene{0, 0, 1, 1};
  mjrRect panel{0, 0, 0, 0};
  std::array<mjrRect, kArmDof> panels{};
  mjrRect status{0, 0, 0, 0};
};

JointPlotLayout computeJointPlotLayout(int framebuffer_width,
                                       int framebuffer_height,
                                       bool requested) noexcept;
bool pointInJointPlotPanel(const JointPlotLayout& layout, int x,
                           int y) noexcept;

class MujocoJointPlot {
 public:
  MujocoJointPlot() noexcept;

  void update(const JointKinematicsHistory& history, ArmSide side,
              PlotMetric metric, double window_seconds) noexcept;
  void render(const JointPlotLayout& layout,
              const mjrContext& context) noexcept;
  const mjvFigure& figure(int joint) const noexcept;

 private:
  std::array<mjvFigure, kArmDof> figures_{};
};

}  // namespace tianji_qp_ik
