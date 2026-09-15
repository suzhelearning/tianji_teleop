#include "tianji_qp_ik/mujoco_joint_plot.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace tianji_qp_ik {
namespace {

constexpr int kPanelGap = 6;
constexpr int kMinimumWindowWidth = 1200;
constexpr int kMinimumWindowHeight = 700;
constexpr int kMinimumSceneWidth = 640;

void setColor(float color[3], float red, float green, float blue) noexcept {
  color[0] = red;
  color[1] = green;
  color[2] = blue;
}

void configureFigure(mjvFigure& figure) noexcept {
  mjv_defaultFigure(&figure);
  figure.flg_legend = 0;
  figure.flg_ticklabel[0] = 1;
  figure.flg_ticklabel[1] = 1;
  figure.flg_extend = 1;
  figure.linewidth = 1.5F;
  figure.gridwidth = 0.5F;
  figure.gridsize[0] = 4;
  figure.gridsize[1] = 4;
  figure.figurergba[0] = 0.035F;
  figure.figurergba[1] = 0.055F;
  figure.figurergba[2] = 0.080F;
  figure.figurergba[3] = 1.0F;
  figure.panergba[0] = 0.055F;
  figure.panergba[1] = 0.080F;
  figure.panergba[2] = 0.110F;
  figure.panergba[3] = 1.0F;
  setColor(figure.linergb[0], 0.20F, 0.90F, 0.82F);
  setColor(figure.linergb[1], 0.35F, 0.65F, 1.00F);
  setColor(figure.linergb[2], 1.00F, 0.38F, 0.48F);
  setColor(figure.linergb[3], 1.00F, 0.38F, 0.48F);
  std::snprintf(figure.linename[0], sizeof(figure.linename[0]),
                "QP reference");
  std::snprintf(figure.linename[1], sizeof(figure.linename[1]),
                "MuJoCo actual");
  std::snprintf(figure.linename[2], sizeof(figure.linename[2]), "lower");
  std::snprintf(figure.linename[3], sizeof(figure.linename[3]), "upper");
  std::snprintf(figure.xlabel, sizeof(figure.xlabel), "time [s]");
}

void appendPoint(mjvFigure& figure, int line, float x, float y) noexcept {
  int& count = figure.linepnt[line];
  if (count >= mjMAXLINEPNT || !std::isfinite(x) || !std::isfinite(y)) {
    return;
  }
  figure.linedata[line][2 * count] = x;
  figure.linedata[line][2 * count + 1] = y;
  ++count;
}

}  // namespace

JointPlotLayout computeJointPlotLayout(int framebuffer_width,
                                       int framebuffer_height,
                                       bool requested) noexcept {
  JointPlotLayout layout;
  const int width = std::max(framebuffer_width, 1);
  const int height = std::max(framebuffer_height, 1);
  layout.scene = {0, 0, width, height};
  if (!requested || width < kMinimumWindowWidth ||
      height < kMinimumWindowHeight) {
    return layout;
  }

  const int desired_panel_width = std::max(600, 45 * width / 100);
  const int panel_width =
      std::min(desired_panel_width, width - kMinimumSceneWidth);
  if (panel_width <= 2 * kPanelGap) {
    return layout;
  }
  layout.visible = true;
  layout.scene.width = width - panel_width;
  layout.panel = {layout.scene.width, 0, panel_width, height};
  const int cell_width = (panel_width - 3 * kPanelGap) / 2;
  const int cell_height = (height - 5 * kPanelGap) / 4;
  for (int joint = 0; joint < kArmDof; ++joint) {
    const int row = joint / 2;
    const int column = joint % 2;
    layout.panels[static_cast<std::size_t>(joint)] = mjrRect{
        layout.panel.left + kPanelGap + column * (cell_width + kPanelGap),
        height - kPanelGap - (row + 1) * cell_height - row * kPanelGap,
        cell_width, cell_height};
  }
  layout.status = mjrRect{
      layout.panel.left + kPanelGap + cell_width + kPanelGap, kPanelGap,
      cell_width, cell_height};
  return layout;
}

bool pointInJointPlotPanel(const JointPlotLayout& layout, int x,
                           int y) noexcept {
  return layout.visible && x >= layout.panel.left &&
         x < layout.panel.left + layout.panel.width &&
         y >= layout.panel.bottom &&
         y < layout.panel.bottom + layout.panel.height;
}

MujocoJointPlot::MujocoJointPlot() noexcept {
  for (mjvFigure& figure : figures_) {
    configureFigure(figure);
  }
}

void MujocoJointPlot::update(const JointKinematicsHistory& history,
                             ArmSide side, PlotMetric metric,
                             double window_seconds) noexcept {
  const double safe_window =
      std::isfinite(window_seconds) && window_seconds > 0.0
          ? window_seconds
          : 5.0;
  for (int joint = 0; joint < kArmDof; ++joint) {
    mjvFigure& current = figures_[static_cast<std::size_t>(joint)];
    for (int line = 0; line < 4; ++line) {
      current.linepnt[line] = 0;
    }
    current.range[0][0] = static_cast<float>(-safe_window);
    current.range[0][1] = 0.0F;
    current.range[1][0] = 0.0F;
    current.range[1][1] = 0.0F;
    current.flg_symmetric = metric == PlotMetric::kPosition ? 0 : 1;
    std::snprintf(current.title, sizeof(current.title), "%s J%d %s [%s]",
                  side == ArmSide::kLeft ? "Left" : "Right", joint + 1,
                  plotMetricName(metric), plotMetricUnit(metric));

    const JointPlotSeries values = history.series(side, metric, joint);
    if (values.time.empty()) {
      continue;
    }
    const double newest = values.time.back();
    const std::size_t first =
        values.time.size() > static_cast<std::size_t>(mjMAXLINEPNT)
            ? values.time.size() - static_cast<std::size_t>(mjMAXLINEPNT)
            : 0U;
    for (std::size_t index = first; index < values.time.size(); ++index) {
      const double relative_time = values.time[index] - newest;
      if (relative_time < -safe_window) {
        continue;
      }
      const float x = static_cast<float>(relative_time);
      if (values.reference_valid[index]) {
        appendPoint(current, 0, x,
                    static_cast<float>(values.reference[index]));
      }
      if (values.actual_valid[index]) {
        appendPoint(current, 1, x, static_cast<float>(values.actual[index]));
      }
      appendPoint(current, 2, x, static_cast<float>(values.lower[index]));
      appendPoint(current, 3, x, static_cast<float>(values.upper[index]));
    }
  }
}

void MujocoJointPlot::render(const JointPlotLayout& layout,
                             const mjrContext& context) noexcept {
  if (!layout.visible) {
    return;
  }
  for (int joint = 0; joint < kArmDof; ++joint) {
    mjr_figure(layout.panels[static_cast<std::size_t>(joint)],
               &figures_[static_cast<std::size_t>(joint)], &context);
  }
}

const mjvFigure& MujocoJointPlot::figure(int joint) const noexcept {
  const int bounded = std::clamp(joint, 0, kArmDof - 1);
  return figures_[static_cast<std::size_t>(bounded)];
}

}  // namespace tianji_qp_ik
