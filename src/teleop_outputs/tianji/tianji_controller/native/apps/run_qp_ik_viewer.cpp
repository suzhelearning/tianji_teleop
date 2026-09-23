// Display-only helper window of the bare-hand SPD route.
//
// `--external-display` renders the 54-joint model state that the parent
// simulation process writes to a private stdin pipe:
//
//   <timestamp_ns> <54 joint radians> <status text>\n
//
// The process owns no control loop, no network input, no recording, no joint
// export and no second solver. The SPD DLS worker stays the only motion
// authority; this window only shows the joints it is told to show.
// `--headless` keeps the same pipe contract without opening a window, so the
// display path can be exercised where no display server exists.
#include "tianji_qp_ik/config.hpp"
#include "tianji_qp_ik/joint_kinematics_plot.hpp"
#include "tianji_qp_ik/mujoco_joint_plot.hpp"
#include "tianji_qp_ik/mujoco_robot.hpp"
#include "tianji_qp_ik/resource_paths.hpp"
#include "tianji_qp_ik/shared_root_options.hpp"

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <fcntl.h>
#include <unistd.h>

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>

namespace tianji_qp_ik {
namespace {

struct Options {
  std::string config_path;
  std::string model_path;
  bool help_requested{false};
  bool external_display{false};
  bool continuous_follow{false};
  bool headless{false};
};

Options parseOptions(int argc, char** argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--help") {
      std::cout << "Usage: tianji_qp_ik_viewer --external-display "
                   "[--model FILE] [--config FILE] "
                   "[--continuous-follow] [--headless (private stdin pipe only)]\n"
                   "Private stdin frames: <timestamp_ns> <54 joint radians> "
                   "<status text>. No control, network input, recording or "
                   "joint export is available in this display-only helper.\n";
      options.help_requested = true;
      return options;
    }
    if (argument == "--external-display") {
      options.external_display = true;
      continue;
    }
    if (argument == "--continuous-follow") {
      options.continuous_follow = true;
      continue;
    }
    if (argument == "--headless") {
      options.headless = true;
      continue;
    }
    if (index + 1 >= argc) {
      throw std::invalid_argument("missing value after " + argument);
    }
    const std::string value(argv[++index]);
    if (argument == "--config") {
      options.config_path = value;
    } else if (argument == "--model") {
      options.model_path = value;
    } else {
      throw std::invalid_argument("unknown option: " + argument);
    }
  }
  if (!options.external_display) {
    throw std::invalid_argument(
        "--external-display is required: this helper only displays the "
        "54-joint simulation state supplied by its parent pipe");
  }
  return options;
}

struct ViewerApplication {
  explicit ViewerApplication(MujocoRobot& robot_in) : robot(robot_in) {}

  MujocoRobot& robot;
  bool continuous_follow{false};
  bool external_quit_sent{false};
  std::string external_status{"Waiting for simulation joint frames"};
  Vec7 left_q{Vec7::Zero()};
  Vec7 right_q{Vec7::Zero()};
  Vec20 left_hand_q{Vec20::Zero()};
  Vec20 right_hand_q{Vec20::Zero()};
  std::uint64_t frames{0U};
  mjvCamera camera{};
  mjvOption visual_options{};
  mjvScene scene{};
  mjvPerturb perturb{};
  mjrContext context{};
  JointKinematicsHistory joint_plot_history{2001U};
  MujocoJointPlot joint_plot;
  JointPlotLayout joint_plot_layout;
  PlotMetric joint_plot_metric{PlotMetric::kPosition};
  ArmSide joint_plot_arm{ArmSide::kLeft};
  bool show_joint_plots{true};
  bool joint_plot_arm_locked{false};
  ArmSide selected_arm{ArmSide::kLeft};
  int target_left_body{-1};
  int target_right_body{-1};
  bool left_button{false};
  bool middle_button{false};
  bool right_button{false};
  bool show_help{true};
  double previous_x{0.0};
  double previous_y{0.0};
};

void setConfiguredPlotPositionBounds(ArmSide side, const MujocoRobot& robot,
                                     const QpIkConfig& config,
                                     JointKinematicsBounds& bounds) {
  const ArmLimits& limits = robot.mapping(side).limits;
  const double margin = config.joint_limits.margin_rad;
  bounds.position_lower = limits.lower_position.array() + margin;
  bounds.position_upper = limits.upper_position.array() - margin;
}

void renderSnapshot(ViewerApplication& application) {
  application.robot.setArmPosition(ArmSide::kLeft, application.left_q);
  application.robot.setArmPosition(ArmSide::kRight, application.right_q);
  if (application.robot.hasHandMappings()) {
    // Display the supplied positions exactly; the controller's hand setter
    // intentionally clamps, but a display must not modify its input state.
    for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
      const HandMapping& mapping = application.robot.handMapping(side);
      const Vec20& position = side == ArmSide::kLeft
          ? application.left_hand_q : application.right_hand_q;
      for (int joint = 0; joint < kHandDof; ++joint) {
        application.robot.data()->qpos[
            mapping.qpos_addresses[static_cast<std::size_t>(joint)]] = position[joint];
      }
    }
  }
  application.robot.forward();
}

void drawOverlay(const ViewerApplication& application, mjrRect viewport) {
  char status[5120]{};
  const char* help = application.continuous_follow
      ? "R calibrate / recalibrate | S follow\n"
        "P/Space hold | H arms Home | Q/Escape/close exit\n"
        "F1 help | F2 plots | F3 metric\n"
        "F4 arm lock | F5 follow selected\n"
        "Mouse: rotate / pan / zoom"
      : "C calibrate | S follow | P/Space hold\n"
        "H arms Home | Q Home then quit\n"
        "Escape/close: parent finishes Home\n"
        "F1 help | F2 plots | F3 metric\n"
        "F4 arm lock | F5 follow L/R\n"
        "Mouse: rotate / pan / zoom";
  std::snprintf(
      status, sizeof(status),
      "Simulation joints | no hardware feedback\n%s\n%s",
      application.external_status.c_str(),
      application.show_help ? help : "");
  mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, status, nullptr,
              &application.context);
}

void selectTarget(ViewerApplication& application, ArmSide side) {
  application.selected_arm = side;
  if (!application.joint_plot_arm_locked) {
    application.joint_plot_arm = side;
  }
}

void keyboardCallback(GLFWwindow* window, int key, int, int action, int) {
  if (action != GLFW_PRESS) {
    return;
  }
  auto& application = *static_cast<ViewerApplication*>(glfwGetWindowUserPointer(window));
  if (key == GLFW_KEY_ESCAPE) {
    if (!application.external_quit_sent) {
      std::cout << "DISPLAY_KEY " << GLFW_KEY_Q << std::endl;
      application.external_quit_sent = true;
    }
    glfwSetWindowShouldClose(window, GLFW_TRUE);
    return;
  }
  // Control keys belong to the parent state machine; forward them verbatim.
  if (application.continuous_follow) {
    if (key == GLFW_KEY_R || key == GLFW_KEY_S || key == GLFW_KEY_P ||
        key == GLFW_KEY_H || key == GLFW_KEY_Q || key == GLFW_KEY_SPACE) {
      std::cout << "DISPLAY_KEY " << key << std::endl;
      return;
    }
  } else if (key == GLFW_KEY_C || key == GLFW_KEY_S || key == GLFW_KEY_P ||
             key == GLFW_KEY_H || key == GLFW_KEY_Q || key == GLFW_KEY_SPACE) {
    std::cout << "DISPLAY_KEY " << key << std::endl;
    return;
  }
  switch (key) {
    case GLFW_KEY_L:
      selectTarget(application, ArmSide::kLeft);
      break;
    case GLFW_KEY_R:
      selectTarget(application, ArmSide::kRight);
      break;
    case GLFW_KEY_F1:
      application.show_help = !application.show_help;
      break;
    case GLFW_KEY_F2:
      application.show_joint_plots = !application.show_joint_plots;
      break;
    case GLFW_KEY_F3:
      application.joint_plot_metric =
          nextPlotMetric(application.joint_plot_metric);
      break;
    case GLFW_KEY_F4:
      application.joint_plot_arm =
          application.joint_plot_arm == ArmSide::kLeft ? ArmSide::kRight
                                                        : ArmSide::kLeft;
      application.joint_plot_arm_locked = true;
      break;
    case GLFW_KEY_F5:
      application.joint_plot_arm_locked = false;
      application.joint_plot_arm = application.selected_arm;
      break;
    default:
      break;
  }
}

struct ViewportCursor {
  int width{1};
  int height{1};
  Eigen::Vector2d pixel{Eigen::Vector2d::Zero()};
};

ViewportCursor viewportCursor(GLFWwindow* window, double cursor_x, double cursor_y) {
  int window_width = 1;
  int window_height = 1;
  int framebuffer_width = 1;
  int framebuffer_height = 1;
  glfwGetWindowSize(window, &window_width, &window_height);
  glfwGetFramebufferSize(window, &framebuffer_width, &framebuffer_height);
  window_width = std::max(window_width, 1);
  window_height = std::max(window_height, 1);
  framebuffer_width = std::max(framebuffer_width, 1);
  framebuffer_height = std::max(framebuffer_height, 1);
  const double scale_x = static_cast<double>(framebuffer_width) / window_width;
  const double scale_y = static_cast<double>(framebuffer_height) / window_height;
  auto& application =
      *static_cast<ViewerApplication*>(glfwGetWindowUserPointer(window));
  const int interaction_width = application.joint_plot_layout.visible
                                    ? application.joint_plot_layout.scene.width
                                    : framebuffer_width;
  return ViewportCursor{
      interaction_width,
      framebuffer_height,
      Eigen::Vector2d(cursor_x * scale_x,
                      (static_cast<double>(window_height) - cursor_y) * scale_y)};
}

void mouseButtonCallback(GLFWwindow* window, int button, int action, int) {
  auto& application = *static_cast<ViewerApplication*>(glfwGetWindowUserPointer(window));
  application.left_button =
      button == GLFW_MOUSE_BUTTON_LEFT ? action == GLFW_PRESS : application.left_button;
  application.middle_button =
      button == GLFW_MOUSE_BUTTON_MIDDLE ? action == GLFW_PRESS : application.middle_button;
  application.right_button =
      button == GLFW_MOUSE_BUTTON_RIGHT ? action == GLFW_PRESS : application.right_button;
  glfwGetCursorPos(window, &application.previous_x, &application.previous_y);

  const ViewportCursor panel_cursor =
      viewportCursor(window, application.previous_x, application.previous_y);
  if (pointInJointPlotPanel(
          application.joint_plot_layout,
          static_cast<int>(panel_cursor.pixel.x()),
          static_cast<int>(panel_cursor.pixel.y()))) {
    application.left_button = false;
    application.middle_button = false;
    application.right_button = false;
  }
}

void cursorPositionCallback(GLFWwindow* window, double x, double y) {
  auto& application = *static_cast<ViewerApplication*>(glfwGetWindowUserPointer(window));
  const ViewportCursor cursor = viewportCursor(window, x, y);
  if (pointInJointPlotPanel(application.joint_plot_layout,
                            static_cast<int>(cursor.pixel.x()),
                            static_cast<int>(cursor.pixel.y()))) {
    application.previous_x = x;
    application.previous_y = y;
    return;
  }
  if (!application.left_button && !application.middle_button && !application.right_button) {
    application.previous_x = x;
    application.previous_y = y;
    return;
  }
  int width = 1;
  int height = 1;
  glfwGetWindowSize(window, &width, &height);
  const double dx = (x - application.previous_x) / static_cast<double>(height);
  const double dy = (y - application.previous_y) / static_cast<double>(height);
  application.previous_x = x;
  application.previous_y = y;
  const bool shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                     glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
  int mouse_action = mjMOUSE_ROTATE_V;
  if (application.right_button) {
    mouse_action = shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (application.middle_button) {
    mouse_action = mjMOUSE_ZOOM;
  } else if (shift) {
    mouse_action = mjMOUSE_ROTATE_H;
  }
  mjv_moveCamera(application.robot.model(), mouse_action, dx, dy, &application.scene,
                 &application.camera);
}

void windowFocusCallback(GLFWwindow* window, int focused) {
  if (focused == GLFW_TRUE) {
    return;
  }
  auto& application = *static_cast<ViewerApplication*>(glfwGetWindowUserPointer(window));
  application.left_button = false;
  application.middle_button = false;
  application.right_button = false;
}

void scrollCallback(GLFWwindow* window, double, double y_offset) {
  auto& application = *static_cast<ViewerApplication*>(glfwGetWindowUserPointer(window));
  double x = 0.0;
  double y = 0.0;
  glfwGetCursorPos(window, &x, &y);
  const ViewportCursor cursor = viewportCursor(window, x, y);
  if (pointInJointPlotPanel(application.joint_plot_layout,
                            static_cast<int>(cursor.pixel.x()),
                            static_cast<int>(cursor.pixel.y()))) {
    return;
  }
  mjv_moveCamera(application.robot.model(), mjMOUSE_ZOOM, 0.0, -0.05 * y_offset,
                 &application.scene, &application.camera);
}

// Private parent pipe: no control worker or network receiver exists in this
// process. Bounded reads keep the window responsive even if the producer
// outruns rendering.
class ExternalDisplayInput {
 public:
  ExternalDisplayInput(const MujocoRobot& robot, const QpIkConfig& config) {
    setConfiguredPlotPositionBounds(ArmSide::kLeft, robot, config, left_bounds_);
    setConfiguredPlotPositionBounds(ArmSide::kRight, robot, config, right_bounds_);
    {
      const auto& smoothing = config.pico_ee_franka_dls.post_smoothing;
      for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
        JointKinematicsBounds& bounds =
            side == ArmSide::kLeft ? left_bounds_ : right_bounds_;
        const Vec7 velocity =
            (smoothing.velocity_scale * robot.mapping(side).limits.velocity)
                .cwiseMin(smoothing.max_velocity_rad_s);
        bounds.velocity_lower = -velocity;
        bounds.velocity_upper = velocity;
        bounds.acceleration_lower = -smoothing.max_acceleration_rad_s2;
        bounds.acceleration_upper = smoothing.max_acceleration_rad_s2;
        bounds.jerk_lower = -smoothing.max_jerk_rad_s3;
        bounds.jerk_upper = smoothing.max_jerk_rad_s3;
      }
    }
    flags_ = ::fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags_ < 0 || ::fcntl(STDIN_FILENO, F_SETFL, flags_ | O_NONBLOCK) < 0) {
      throw std::runtime_error("could not set private display stdin nonblocking");
    }
  }
  ExternalDisplayInput(const ExternalDisplayInput&) = delete;
  ExternalDisplayInput& operator=(const ExternalDisplayInput&) = delete;

  ~ExternalDisplayInput() {
    if (flags_ >= 0) {
      (void)::fcntl(STDIN_FILENO, F_SETFL, flags_);
    }
  }

  bool drain(ViewerApplication& application) {
    std::array<char, 4096> chunk{};
    for (unsigned batch = 0; batch < 16U; ++batch) {
      const ssize_t count = ::read(STDIN_FILENO, chunk.data(), chunk.size());
      if (count == 0) {
        return false;
      }
      if (count < 0) {
        if (errno == EINTR) {
          continue;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          return true;
        }
        throw std::runtime_error("private display stdin read failed");
      }
      for (ssize_t index = 0; index < count; ++index) {
        const char value = chunk[static_cast<std::size_t>(index)];
        if (value == '\n') {
          if (!discard_line_) {
            acceptLine(application);
          }
          line_size_ = 0U;
          discard_line_ = false;
        } else if (line_size_ < line_.size() && value != '\0') {
          line_[line_size_++] = value;
        } else {
          discard_line_ = true;
        }
      }
    }
    return true;
  }

 private:
  void acceptLine(ViewerApplication& application) {
    const char* cursor = line_.data();
    const char* end = cursor + line_size_;
    const auto skip_space = [&cursor, end] {
      while (cursor != end && (*cursor == ' ' || *cursor == '\t' || *cursor == '\r')) {
        ++cursor;
      }
    };
    skip_space();
    std::uint64_t timestamp = 0U;
    const auto time_result = std::from_chars(cursor, end, timestamp);
    if (time_result.ec != std::errc{} || time_result.ptr == end ||
        (sequence_ != 0U && timestamp <= previous_timestamp_)) {
      return;
    }
    cursor = time_result.ptr;
    std::array<double, 54> joints{};
    for (double& joint : joints) {
      const char* separator = cursor;
      skip_space();
      if (cursor == separator || cursor == end) {
        return;
      }
      const auto result = std::from_chars(cursor, end, joint);
      if (result.ec != std::errc{} || !std::isfinite(joint)) {
        return;
      }
      cursor = result.ptr;
    }
    if (cursor != end && *cursor != ' ' && *cursor != '\t' && *cursor != '\r') {
      return;
    }
    skip_space();
    application.external_status.assign(cursor, end);
    std::replace(application.external_status.begin(),
                 application.external_status.end(), '|', '\n');
    if (application.external_status.empty()) {
      application.external_status = "Receiving simulation joint frames";
    }
    application.left_q = Eigen::Map<const Vec7>(joints.data());
    application.right_q = Eigen::Map<const Vec7>(joints.data() + 7);
    application.left_hand_q = Eigen::Map<const Vec20>(joints.data() + 14);
    application.right_hand_q = Eigen::Map<const Vec20>(joints.data() + 34);
    if (sequence_ == 0U) {
      first_timestamp_ = timestamp;
    }
    const double dt = sequence_ == 0U
        ? 0.0 : static_cast<double>(timestamp - previous_timestamp_) * 1e-9;
    JointKinematicsSample sample;
    sample.sequence = ++sequence_;
    ++application.frames;
    sample.time_seconds = static_cast<double>(timestamp - first_timestamp_) * 1e-9;
    sample.left.bounds = left_bounds_;
    sample.right.bounds = right_bounds_;
    setState(sample.left, application.left_q, previous_left_,
             left_differentiator_, dt);
    setState(sample.right, application.right_q, previous_right_,
             right_differentiator_, dt);
    application.joint_plot_history.push(sample);
    previous_timestamp_ = timestamp;
  }

  static void setState(ArmJointKinematicsSample& sample, const Vec7& position,
                       Vec7& previous_position,
                       JointKinematicsDifferentiator& differentiator, double dt) {
    const double absent = std::numeric_limits<double>::quiet_NaN();
    sample.reference.position.setConstant(absent);
    sample.reference.velocity.setConstant(absent);
    sample.reference.acceleration.setConstant(absent);
    sample.reference.jerk.setConstant(absent);
    sample.actual.position = position;
    sample.actual.velocity.setConstant(absent);
    if (dt > 0.0) {
      sample.actual.velocity = (position - previous_position) / dt;
      const JointKinematicsDerivatives derivatives = differentiator.update(
          Vec7::Zero(), Vec7::Zero(),
          ReferenceAccelerationSource::kDifferentiateVelocity,
          sample.actual.velocity, dt, false);
      sample.actual.acceleration = derivatives.actual_acceleration;
      sample.actual.jerk = derivatives.actual_jerk;
      sample.actual_acceleration_valid = derivatives.actual_acceleration_valid;
      sample.actual_jerk_valid = derivatives.actual_jerk_valid;
    }
    previous_position = position;
  }

  int flags_{-1};
  std::array<char, 4096> line_{};
  std::size_t line_size_{0U};
  bool discard_line_{false};
  std::uint64_t sequence_{0U};
  std::uint64_t first_timestamp_{0U};
  std::uint64_t previous_timestamp_{0U};
  Vec7 previous_left_{Vec7::Zero()};
  Vec7 previous_right_{Vec7::Zero()};
  JointKinematicsBounds left_bounds_;
  JointKinematicsBounds right_bounds_;
  JointKinematicsDifferentiator left_differentiator_;
  JointKinematicsDifferentiator right_differentiator_;
};

// Pipe-only companion of the window mode: identical frame contract and state
// bookkeeping, no GLFW context. Used by the parent's self-test and by hosts
// without a display server.
int runPipeOnly(const Options& options, const QpIkConfig& config) {
  MujocoRobot robot(options.model_path);
  if (!robot.hasHandMappings()) {
    throw std::runtime_error("external display requires a mapped 54-joint model");
  }
  ViewerApplication application(robot);
  application.continuous_follow = options.continuous_follow;
  ExternalDisplayInput input(robot, config);
  std::cout << "DISPLAY_READY" << std::endl;
  while (input.drain(application)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  std::cout << "display_complete frames=" << application.frames << std::endl;
  return 0;
}

int runWindow(const Options& options, const QpIkConfig& config) {
  if (glfwInit() == GLFW_FALSE) {
    throw std::runtime_error(
        "GLFW initialization failed; use --headless for the pipe-only display");
  }
  GLFWwindow* window = glfwCreateWindow(1280, 800, "Tianji dual-arm DLS", nullptr, nullptr);
  if (window == nullptr) {
    glfwTerminate();
    throw std::runtime_error("GLFW window creation failed");
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);
  MujocoRobot robot(options.model_path);
  if (!robot.hasHandMappings()) {
    glfwDestroyWindow(window);
    glfwTerminate();
    throw std::runtime_error("external display requires a mapped 54-joint model");
  }
  ViewerApplication application(robot);
  application.continuous_follow = options.continuous_follow;
  ExternalDisplayInput input(robot, config);
  glfwSetWindowTitle(window, "Tianji simulation joint display | no hardware output");
  application.target_left_body = mj_name2id(robot.model(), mjOBJ_BODY, "target_L");
  application.target_right_body = mj_name2id(robot.model(), mjOBJ_BODY, "target_R");
  mjv_defaultFreeCamera(robot.model(), &application.camera);
  application.camera.distance *= 1.5;
  mjv_defaultOption(&application.visual_options);
  application.visual_options.frame = mjFRAME_NONE;
  mjv_defaultPerturb(&application.perturb);
  mjv_defaultScene(&application.scene);
  mjr_defaultContext(&application.context);
  mjv_makeScene(robot.model(), &application.scene, 2000);
  mjr_makeContext(robot.model(), &application.context, mjFONTSCALE_150);
  glfwSetWindowUserPointer(window, &application);
  glfwSetKeyCallback(window, keyboardCallback);
  glfwSetMouseButtonCallback(window, mouseButtonCallback);
  glfwSetCursorPosCallback(window, cursorPositionCallback);
  glfwSetScrollCallback(window, scrollCallback);
  glfwSetWindowFocusCallback(window, windowFocusCallback);
  glfwSetWindowCloseCallback(window, [](GLFWwindow* closing_window) {
    auto& app = *static_cast<ViewerApplication*>(glfwGetWindowUserPointer(closing_window));
    if (!app.external_quit_sent) {
      std::cout << "DISPLAY_KEY " << GLFW_KEY_Q << std::endl;
      app.external_quit_sent = true;
    }
  });
  std::cout << "DISPLAY_READY" << std::endl;

  while (glfwWindowShouldClose(window) == GLFW_FALSE) {
    const std::uint64_t previous_frames = application.frames;
    if (!input.drain(application)) {
      glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
    if (application.frames != previous_frames) {
      renderSnapshot(application);
    }
    int width = 1;
    int height = 1;
    glfwGetFramebufferSize(window, &width, &height);
    application.joint_plot_layout = computeJointPlotLayout(
        width, height, application.show_joint_plots);
    const mjrRect viewport = application.joint_plot_layout.scene;
    mjv_updateScene(robot.model(), robot.data(), &application.visual_options,
                    &application.perturb, &application.camera, mjCAT_ALL, &application.scene);
    // Static target markers are not part of the supplied simulation state.
    const mjModel* model = robot.model();
    for (int index = 0; index < application.scene.ngeom; ++index) {
      mjvGeom& geom = application.scene.geoms[index];
      int body = -1;
      if (geom.objtype == mjOBJ_GEOM && geom.objid >= 0) {
        body = model->geom_bodyid[geom.objid];
      } else if (geom.objtype == mjOBJ_SITE && geom.objid >= 0) {
        body = model->site_bodyid[geom.objid];
      }
      while (body > 0) {
        if (body == application.target_left_body ||
            body == application.target_right_body) {
          geom.rgba[3] = 0.0F;
          break;
        }
        body = model->body_parentid[body];
      }
    }
    mjr_render(viewport, &application.scene, &application.context);
    drawOverlay(application, viewport);
    if (application.joint_plot_layout.visible) {
      application.joint_plot.updateSimulation(
          application.joint_plot_history, application.joint_plot_arm,
          application.joint_plot_metric, 5.0);
      application.joint_plot.render(application.joint_plot_layout,
                                    application.context);
      char plot_status[640]{};
      std::snprintf(
          plot_status, sizeof(plot_status),
          "%s arm | %s [%s] | last 5 s\n"
          "blue: simulation\n"
          "auto Y (zoomed)\n"
          "red limits may be hidden\n"
          "derivatives: timestamps\n"
          "F3 metric | F4 arm\n"
          "history %zu/%zu",
          application.joint_plot_arm == ArmSide::kLeft ? "Left" : "Right",
          plotMetricName(application.joint_plot_metric),
          plotMetricUnit(application.joint_plot_metric),
          application.joint_plot_history.size(),
          application.joint_plot_history.capacity());
      mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT,
                  application.joint_plot_layout.status, plot_status, nullptr,
                  &application.context);
    }
    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  std::cout << "display_complete frames=" << application.frames << std::endl;
  mjr_freeContext(&application.context);
  mjv_freeScene(&application.scene);
  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}

int run(int argc, char** argv) {
  Options options = parseOptions(argc, argv);
  if (options.help_requested) {
    return 0;
  }
  if (options.config_path.empty()) {
    options.config_path = (runtimeWorkspace() /
        "install/spd/share/tianji_controller/config/qp_ik_pico_shared_root_dls.yaml").string();
  }
  const QpIkConfig config = loadConfig(options.config_path, ConfigConsumer::kSharedRootAware);
  if (options.model_path.empty()) {
    if (config.shared_root_profile_path.empty()) {
      throw std::invalid_argument(
          "--model is required when the profile has no shared-root model");
    }
    options.model_path = loadSharedRootOptions(config.shared_root_profile_path).mujoco_path;
  }
  return options.headless ? runPipeOnly(options, config)
                          : runWindow(options, config);
}

}  // namespace
}  // namespace tianji_qp_ik

int main(int argc, char** argv) {
  try {
    return tianji_qp_ik::run(argc, argv);
  } catch (const std::exception& error) {
    std::cerr << "viewer error: " << error.what() << '\n';
    return 1;
  }
}
