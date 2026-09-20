#include "tianji_qp_ik/controller.hpp"
#include "tianji_qp_ik/cartesian_otg.hpp"
#include "tianji_qp_ik/acceleration_controller.hpp"
#include "tianji_qp_ik/interactive_marker.hpp"
#include "tianji_qp_ik/joint_kinematics_plot.hpp"
#include "tianji_qp_ik/joint_command.hpp"
#include "tianji_qp_ik/mujoco_marker_adapter.hpp"
#include "tianji_qp_ik/mujoco_joint_plot.hpp"
#include "tianji_qp_ik/mujoco_robot.hpp"
#include "tianji_qp_ik/pico_teleop_session.hpp"
#include "tianji_qp_ik/pico_skeleton_overlay.hpp"
#include "tianji_qp_ik/pico_udp_receiver.hpp"
#include "tianji_qp_ik/so3.hpp"
#include "tianji_qp_ik/spark_guidance.hpp"
#include "tianji_qp_ik/spark_qpoases_diagnostic.hpp"
#include "tianji_qp_ik/target_manager.hpp"
#include "tianji_qp_ik/telemetry.hpp"
#include "tianji_qp_ik/wuji_hand_udp_receiver.hpp"

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <arpa/inet.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iomanip>
#include "tianji_qp_ik/simulation_recovery.hpp"
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <time.h>

namespace tianji_qp_ik {
namespace {

volatile std::sig_atomic_t stop_requested = 0;

void requestStop(int) { stop_requested = 1; }

struct Options {
  std::string config_path{"config/qp_ik_pico_teleop.yaml"};
  std::string model_path{"models/marvin_m6_wuji2.xml"};
  bool model_path_explicit{false};
  std::string telemetry_path;
  std::string joint_telemetry_path;
  std::string joint_command_host{"127.0.0.1"};
  std::uint16_t joint_command_port{0U};
  bool continuous{false};
  bool headless{false};
  bool simulation_recovery{false};
  bool sim_allow_pico_jumps{false};
  double duration_seconds{0.0};
  bool pico_teleop{true};
  bool pico_skeleton_overlay{true};
  bool hand_teleop{false};
  std::string pico_bind{"127.0.0.1"};
  std::uint16_t pico_port{15000U};
  std::string hand_bind{"127.0.0.1"};
  std::uint16_t hand_port{16000U};
  double hand_stale_timeout_seconds{0.100};
  std::string pico_record_path;
  std::optional<ControlLevel> control_level_override;
  std::optional<IkAlgorithm> algorithm_override;
  std::optional<bool> model_state_only_override;
  ArmAngleReferenceMode arm_angle_reference_mode{
      ArmAngleReferenceMode::kPico};
};

struct ViewerApplication {
  ViewerApplication(MujocoRobot& robot_in, BoundedSpscQueue<ViewerCommand>& commands_in)
      : robot(robot_in), commands(commands_in) {}

  MujocoRobot& robot;
  BoundedSpscQueue<ViewerCommand>& commands;
  ViewerSnapshot snapshot;
  mjvCamera camera{};
  mjvOption visual_options{};
  mjvScene scene{};
  mjvPerturb perturb{};
  mjrContext context{};
  InteractiveMarker6D marker;
  ManualTargetPreview preview;
  JointKinematicsHistory joint_plot_history{2001U};
  MujocoJointPlot joint_plot;
  JointPlotLayout joint_plot_layout;
  PlotMetric joint_plot_metric{PlotMetric::kPosition};
  ArmSide joint_plot_arm{ArmSide::kLeft};
  bool show_joint_plots{true};
  bool show_pico_skeleton{false};
  bool joint_plot_arm_locked{false};
  ArmSide selected_arm{ArmSide::kLeft};
  MarkerHandle hovered_handle{MarkerHandle::kNone};
  int target_left_body{-1};
  int target_right_body{-1};
  bool left_button{false};
  bool middle_button{false};
  bool right_button{false};
  bool show_help{true};
  std::uint64_t next_command_id{1U};
  std::uint64_t command_drops{0U};
  double previous_x{0.0};
  double previous_y{0.0};
};

Options parseOptions(int argc, char** argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--help") {
      std::cout << "Usage: tianji_qp_ik_viewer [--config FILE] [--model FILE] "
                   "[--solver qpoases] [--headless] [--duration SECONDS|--continuous] "
                   "[--telemetry FILE] [--joint-telemetry FILE] "
                   "[--joint-command-host LOOPBACK_IPV4] [--joint-command-port PORT (0=disabled)] "
                   "[--pico-teleop|--no-pico-teleop] "
                   "[--simulation-recovery (Ceres model-only S/H/P gate)] "
                   "[--sim-allow-pico-jumps (requires simulation recovery)] "
                   "[--pico-skeleton-overlay|--no-pico-skeleton-overlay] "
                   "[--pico-bind IPV4] [--pico-port PORT] "
                   "[--pico-record FILE.tjvr] "
                   "[--hand-teleop|--no-hand-teleop] "
                   "[--hand-bind IPV4] [--hand-port PORT] "
                   "[--hand-stale-timeout SECONDS] "
                   "[--control-level velocity|acceleration] "
                   "[--algorithm hierarchical_qp|nullspace_dls|"
                   "spark_guided_velocity_qp|spark_direct_velocity_qp|"
                   "spark_pose_velocity_qp|spark_upper_qpoases_direct|"
                   "spark_upper_qpoases_velocity_qp|"
                   "spark_upper_qpoases_cartesian_otg_velocity_qp|"
                   "spark_upper_qpoases_feedforward_velocity_qp|"
                   "spark_upper_qpoases_headroom_feedforward_velocity_qp] "
                   "[--model-state-only|--actual-feedback-control] "
                   "[--arm-angle-mode pico|default_down|outward_only|pico_outward]\n";
      std::exit(0);
    }
    if (argument == "--sim-allow-pico-jumps") {
      options.sim_allow_pico_jumps = true;
      continue;
    }
    if (argument == "--simulation-recovery") {
      options.simulation_recovery = true;
      continue;
    }
    if (argument == "--continuous") {
      options.continuous = true;
      continue;
    }
    if (argument == "--headless") {
      options.headless = true;
      continue;
    }
    if (argument == "--pico-teleop") {
      options.pico_teleop = true;
      continue;
    }
    if (argument == "--no-pico-teleop") {
      options.pico_teleop = false;
      continue;
    }
    if (argument == "--hand-teleop") {
      options.hand_teleop = true;
      continue;
    }
    if (argument == "--no-hand-teleop") {
      options.hand_teleop = false;
      continue;
    }
    if (argument == "--pico-skeleton-overlay") {
      options.pico_skeleton_overlay = true;
      continue;
    }
    if (argument == "--no-pico-skeleton-overlay") {
      options.pico_skeleton_overlay = false;
      continue;
    }
    if (argument == "--model-state-only") {
      options.model_state_only_override = true;
      continue;
    }
    if (argument == "--actual-feedback-control") {
      options.model_state_only_override = false;
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
      options.model_path_explicit = true;
    } else if (argument == "--duration") {
      options.duration_seconds = std::stod(value);
    } else if (argument == "--telemetry") {
      options.telemetry_path = value;
    } else if (argument == "--joint-telemetry") {
      options.joint_telemetry_path = value;
    } else if (argument == "--joint-command-host") {
      options.joint_command_host = value;
    } else if (argument == "--joint-command-port") {
      std::size_t parsed_characters = 0U;
      const unsigned long parsed = std::stoul(value, &parsed_characters);
      if (parsed_characters != value.size() || parsed > 65535UL) {
        throw std::invalid_argument("--joint-command-port must be in [0,65535]");
      }
      options.joint_command_port = static_cast<std::uint16_t>(parsed);
    } else if (argument == "--pico-bind") {
      options.pico_bind = value;
    } else if (argument == "--pico-record") {
      options.pico_record_path = value;
    } else if (argument == "--pico-port") {
      std::size_t parsed_characters = 0U;
      const unsigned long parsed = std::stoul(value, &parsed_characters);
      if (parsed_characters != value.size() || parsed < 1UL || parsed > 65535UL) {
        throw std::invalid_argument("--pico-port must be in [1,65535]");
      }
      options.pico_port = static_cast<std::uint16_t>(parsed);
    } else if (argument == "--hand-bind") {
      options.hand_bind = value;
    } else if (argument == "--hand-port") {
      std::size_t parsed_characters = 0U;
      const unsigned long parsed = std::stoul(value, &parsed_characters);
      if (parsed_characters != value.size() || parsed < 1UL ||
          parsed > 65535UL) {
        throw std::invalid_argument("--hand-port must be in [1,65535]");
      }
      options.hand_port = static_cast<std::uint16_t>(parsed);
    } else if (argument == "--hand-stale-timeout") {
      options.hand_stale_timeout_seconds = std::stod(value);
    } else if (argument == "--control-level") {
      if (value == "velocity") {
        options.control_level_override = ControlLevel::kVelocity;
      } else if (value == "acceleration") {
        options.control_level_override = ControlLevel::kAcceleration;
      } else {
        throw std::invalid_argument(
            "--control-level must be velocity or acceleration");
      }
    } else if (argument == "--algorithm") {
      if (value == "hierarchical_qp") {
        options.algorithm_override = IkAlgorithm::kHierarchicalQp;
      } else if (value == "nullspace_dls") {
        options.algorithm_override = IkAlgorithm::kNullspaceDls;
      } else if (value == "pico_ee_franka_ceres_lm") {
        options.algorithm_override = IkAlgorithm::kPicoEeFrankaCeresLm;
      } else if (value == "pico_ee_franka_dls") {
        options.algorithm_override = IkAlgorithm::kPicoEeFrankaDls;
      } else if (value == "spark_guided_velocity_qp") {
        options.algorithm_override = IkAlgorithm::kSparkGuidedVelocityQp;
      } else if (value == "spark_direct_velocity_qp") {
        options.algorithm_override = IkAlgorithm::kSparkDirectVelocityQp;
      } else if (value == "spark_pose_velocity_qp") {
        options.algorithm_override = IkAlgorithm::kSparkPoseVelocityQp;
      } else if (value == "spark_upper_qpoases_direct") {
        options.algorithm_override = IkAlgorithm::kSparkUpperQpoasesDirect;
      } else if (value == "spark_upper_qpoases_velocity_qp") {
        options.algorithm_override =
            IkAlgorithm::kSparkUpperQpoasesVelocityQp;
      } else if (value ==
                 "spark_upper_qpoases_cartesian_otg_velocity_qp") {
        options.algorithm_override =
            IkAlgorithm::kSparkUpperQpoasesCartesianOtgVelocityQp;
      } else if (value ==
                 "spark_upper_qpoases_feedforward_velocity_qp") {
        options.algorithm_override =
            IkAlgorithm::kSparkUpperQpoasesFeedforwardVelocityQp;
      } else if (value ==
                 "spark_upper_qpoases_headroom_feedforward_velocity_qp") {
        options.algorithm_override =
            IkAlgorithm::kSparkUpperQpoasesHeadroomFeedforwardVelocityQp;
      } else {
        throw std::invalid_argument(
            "--algorithm is not a supported IK algorithm");
      }
    } else if (argument == "--arm-angle-mode") {
      options.arm_angle_reference_mode =
          armAngleReferenceModeFromString(value);
    } else if (argument == "--solver") {
      if (value == "osqp") {
        throw std::invalid_argument("hierarchical QP supports qpOASES only");
      } else if (value == "qpoases") {
        // Accepted for compatibility with existing launch commands.
      } else {
        throw std::invalid_argument("--solver must be osqp or qpoases");
      }
    } else {
      throw std::invalid_argument("unknown option: " + argument);
    }
  }
  if (!std::isfinite(options.duration_seconds) || options.duration_seconds < 0.0) {
    throw std::invalid_argument("--duration must be finite and non-negative");
  }
  if (!std::isfinite(options.hand_stale_timeout_seconds) ||
      options.hand_stale_timeout_seconds <= 0.0) {
    throw std::invalid_argument(
        "--hand-stale-timeout must be finite and positive");
  }
  if (options.continuous && (!options.headless || options.duration_seconds != 0.0)) {
    throw std::invalid_argument("--continuous requires --headless and no nonzero --duration");
  }
  if (options.headless && !options.continuous && options.duration_seconds == 0.0) {
    options.duration_seconds = 2.0;
  }
  if (!options.pico_record_path.empty() && !options.pico_teleop) {
    throw std::invalid_argument("--pico-record requires --pico-teleop");
  }
  in_addr parsed_address{};
  if (inet_pton(AF_INET, options.pico_bind.c_str(), &parsed_address) != 1) {
    throw std::invalid_argument("--pico-bind must be a valid IPv4 address");
  }
  if (inet_pton(AF_INET, options.hand_bind.c_str(), &parsed_address) != 1) {
    throw std::invalid_argument("--hand-bind must be a valid IPv4 address");
  }
  if (inet_pton(AF_INET, options.joint_command_host.c_str(), &parsed_address) != 1 ||
      (ntohl(parsed_address.s_addr) >> 24U) != 127U) {
    throw std::invalid_argument("--joint-command-host must be a loopback IPv4 address");
  }
  return options;
}

std::int64_t monotonicNowNs() {
  timespec now{};
  if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
    throw std::runtime_error("clock_gettime(CLOCK_MONOTONIC) failed");
  }
  return static_cast<std::int64_t>(now.tv_sec) * 1000000000LL + now.tv_nsec;
}

timespec addNanoseconds(timespec value, std::int64_t nanoseconds) {
  constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;
  value.tv_nsec += static_cast<long>(nanoseconds);
  while (value.tv_nsec >= kNanosecondsPerSecond) {
    value.tv_nsec -= static_cast<long>(kNanosecondsPerSecond);
    ++value.tv_sec;
  }
  return value;
}

bool laterThan(const timespec& left, const timespec& right) {
  return left.tv_sec > right.tv_sec ||
         (left.tv_sec == right.tv_sec && left.tv_nsec > right.tv_nsec);
}

void sleepUntil(const timespec& deadline) {
  int result = 0;
  do {
    result = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, nullptr);
  } while (result == EINTR);
}

void setInitialConfiguration(MujocoRobot& robot,
                             const QpIkConfig& config) {
  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    const ArmLimits& limits = robot.mapping(side).limits;
    robot.setArmState(
        side,
        configuredInitialPosture(config.controller, limits, side),
        Vec7::Zero());
  }
  robot.forward();
}

Vec20 handVector(
    const std::array<double, kWujiHandJointDof>& values) noexcept {
  Vec20 result;
  for (int index = 0; index < kHandDof; ++index) {
    result[index] = values[static_cast<std::size_t>(index)];
  }
  return result;
}

DualArmTargets currentTargets(MujocoRobot& robot) {
  robot.forward();
  return {robot.tcpPose(ArmSide::kLeft), robot.tcpPose(ArmSide::kRight)};
}

std::unique_ptr<DualArmController> makeController(MujocoRobot& robot,
                                                  const QpIkConfig& config) {
  return std::make_unique<DualArmController>(robot, config);
}

struct DualArmMotionState {
  ArmMotionState left;
  ArmMotionState right;
};

DualArmMotionState captureMotionState(
    ControlLevel control_level, const DualArmController& controller,
    const DualArmAccelerationController& acceleration_controller) {
  if (control_level == ControlLevel::kAcceleration) {
    return {acceleration_controller.referenceState(ArmSide::kLeft),
            acceleration_controller.referenceState(ArmSide::kRight)};
  }
  return {controller.referenceState(ArmSide::kLeft),
          controller.referenceState(ArmSide::kRight)};
}

void seedControllers(
    const DualArmMotionState& motion,
    DualArmController& controller,
    DualArmAccelerationController& acceleration_controller) {
  const bool velocity_seeded =
      controller.setReferenceState(ArmSide::kLeft, motion.left) &&
      controller.setReferenceState(ArmSide::kRight, motion.right);
  const bool acceleration_seeded =
      acceleration_controller.setReferenceState(ArmSide::kLeft, motion.left) &&
      acceleration_controller.setReferenceState(ArmSide::kRight, motion.right);
  if (!velocity_seeded || !acceleration_seeded) {
    throw std::runtime_error("failed to transfer controller motion state");
  }
}

void rebuildControllersPreservingMotion(
    MujocoRobot& robot, const QpIkConfig& config,
    const DualArmMotionState& motion,
    std::unique_ptr<DualArmController>& controller,
    std::unique_ptr<DualArmAccelerationController>& acceleration_controller) {
  auto replacement_controller = makeController(robot, config);
  auto replacement_acceleration =
      std::make_unique<DualArmAccelerationController>(robot, config);
  seedControllers(motion, *replacement_controller, *replacement_acceleration);
  controller = std::move(replacement_controller);
  acceleration_controller = std::move(replacement_acceleration);
}

bool resumeFromPause(
    bool& paused, ControlLevel control_level, DualArmController& controller,
    DualArmAccelerationController& acceleration_controller) {
  if (!paused) {
    return true;
  }
  const bool synchronized =
      control_level == ControlLevel::kAcceleration
          ? acceleration_controller.synchronizeReferencesToActual()
          : controller.synchronizeReferencesToActual();
  if (!synchronized) {
    return false;
  }
  paused = false;
  return true;
}

void processCommand(const ViewerCommand& command, MujocoRobot& robot,
                    QpIkConfig& config, double target_time,
                    TargetManager& targets, PicoTeleopSession& pico_session,
                    bool pico_configured,
                    ArmAngleReferenceMode& arm_angle_reference_mode,
                    bool& paused,
                    bool& pico_paused,
                    std::unique_ptr<DualArmController>& controller,
                    std::unique_ptr<DualArmAccelerationController>&
                        acceleration_controller) {
  switch (command.type) {
    case ViewerCommandType::kSimulationStart:
    case ViewerCommandType::kSimulationHome:
    case ViewerCommandType::kSimulationHold:
      break; // Owned exclusively by the opt-in simulation state machine.
    case ViewerCommandType::kSetMode:
      if (!resumeFromPause(paused, config.control_level, *controller,
                           *acceleration_controller)) {
        break;
      }
      targets.setMode(command.mode, target_time);
      break;
    case ViewerCommandType::kSetBackend:
      if (command.backend != SolverBackend::kQpoases) {
        throw std::invalid_argument("hierarchical QP supports qpOASES only");
      }
      controller->resetSolvers();
      break;
    case ViewerCommandType::kSetIkAlgorithm:
      if (command.algorithm == IkAlgorithm::kNullspaceDls &&
          config.control_level == ControlLevel::kAcceleration) {
        const DualArmMotionState motion = captureMotionState(
            config.control_level, *controller, *acceleration_controller);
        config.control_level = ControlLevel::kVelocity;
        config.ik_algorithm = command.algorithm;
        rebuildControllersPreservingMotion(
            robot, config, motion, controller, acceleration_controller);
      } else {
        config.ik_algorithm = command.algorithm;
        controller->setAlgorithm(command.algorithm);
      }
      break;
    case ViewerCommandType::kSetControlLevel:
      if (config.control_level != command.control_level) {
        const DualArmMotionState motion = captureMotionState(
            config.control_level, *controller, *acceleration_controller);
        config.control_level = command.control_level;
        if (config.control_level == ControlLevel::kAcceleration) {
          config.ik_algorithm = IkAlgorithm::kHierarchicalQp;
        }
        rebuildControllersPreservingMotion(
            robot, config, motion, controller, acceleration_controller);
      }
      break;
    case ViewerCommandType::kResetNominal:
      setInitialConfiguration(robot, config);
      targets = TargetManager(config, currentTargets(robot));
      targets.setMode(TargetMode::kHold, target_time);
      {
        const IkAlgorithm algorithm = controller->algorithm();
        config.ik_algorithm = algorithm;
        controller = makeController(robot, config);
        acceleration_controller =
            std::make_unique<DualArmAccelerationController>(robot, config);
        if (controller->algorithm() != algorithm) {
          controller->setAlgorithm(algorithm);
        }
      }
      paused = false;
      break;
    case ViewerCommandType::kSetManualTarget:
      if (pico_session.enabled()) {
        break;
      }
      if (!resumeFromPause(paused, config.control_level, *controller,
                           *acceleration_controller)) {
        break;
      }
      targets.setMode(TargetMode::kManual, target_time);
      // External VR sources should provide their own monotonic timestamp. Use
      // receive time as a safe compatibility fallback for local/headless
      // producers that do not carry one yet.
      (void)targets.setManualTarget(command.side, command.target,
                                    std::isfinite(command.target_timestamp_seconds)
                                        ? command.target_timestamp_seconds
                                        : target_time,
                                    target_time);
      break;
    case ViewerCommandType::kSetPaused:
      if (paused && !command.paused) {
        if (!resumeFromPause(paused, config.control_level, *controller,
                             *acceleration_controller)) {
          break;
        }
        targets.setMode(TargetMode::kHold, target_time);
      } else {
        paused = command.paused;
      }
      break;
    case ViewerCommandType::kTogglePicoTeleop:
      if (!pico_configured) {
        break;
      }
      {
        const bool was_enabled = pico_session.enabled();
        pico_session.setEnabled(!was_enabled);
        pico_paused = was_enabled;
        if (was_enabled) {
          targets.setMode(TargetMode::kHold, target_time);
        }
      }
      break;
    case ViewerCommandType::kTogglePicoArmAngleSource:
      if (!pico_configured) {
        arm_angle_reference_mode =
            toggleArmAngleReferenceMode(arm_angle_reference_mode);
      }
      break;
  }
}

void setConfiguredPlotBounds(ArmSide side, const MujocoRobot& robot,
                             const QpIkConfig& config,
                             JointKinematicsBounds& bounds) {
  const ArmLimits& limits = robot.mapping(side).limits;
  const bool acceleration_level =
      config.control_level == ControlLevel::kAcceleration;
  const double margin = acceleration_level
                            ? config.joint_acceleration_limits.margin_rad
                            : config.joint_limits.margin_rad;
  const double velocity_scale =
      acceleration_level
          ? config.joint_acceleration_limits.velocity_scale
          : config.joint_limits.velocity_scale;
  const Vec7 acceleration_limit =
      acceleration_level
          ? config.joint_acceleration_limits.max_acceleration_rad_s2
          : config.joint_limits.max_acceleration_rad_s2;
  const bool hard_jerk_enabled =
      acceleration_level
          ? config.joint_acceleration_limits.hard_jerk_enabled
          : config.joint_limits.hard_jerk_enabled;
  const Vec7 jerk_limit =
      acceleration_level
          ? config.joint_acceleration_limits.max_jerk_rad_s3
          : config.joint_limits.max_jerk_rad_s3;
  bounds.position_lower = limits.lower_position.array() + margin;
  bounds.position_upper = limits.upper_position.array() - margin;
  bounds.velocity_lower = -velocity_scale * limits.velocity;
  bounds.velocity_upper = velocity_scale * limits.velocity;
  bounds.acceleration_lower = -acceleration_limit;
  bounds.acceleration_upper = acceleration_limit;
  if (hard_jerk_enabled) {
    bounds.jerk_lower = -jerk_limit;
    bounds.jerk_upper = jerk_limit;
  } else {
    const double unconstrained =
        std::numeric_limits<double>::quiet_NaN();
    bounds.jerk_lower.setConstant(unconstrained);
    bounds.jerk_upper.setConstant(unconstrained);
  }
}

void setPlotDerivatives(
    ArmJointKinematicsSample& sample,
    const JointKinematicsDerivatives& derivatives) noexcept {
  sample.reference.acceleration = derivatives.reference_acceleration;
  sample.reference.jerk = derivatives.reference_jerk;
  sample.actual.acceleration = derivatives.actual_acceleration;
  sample.actual.jerk = derivatives.actual_jerk;
  sample.reference_acceleration_valid =
      derivatives.reference_acceleration_valid;
  sample.reference_jerk_valid = derivatives.reference_jerk_valid;
  sample.actual_acceleration_valid = derivatives.actual_acceleration_valid;
  sample.actual_jerk_valid = derivatives.actual_jerk_valid;
}

void controlLoop(MujocoRobot& robot, QpIkConfig config,
                 BoundedSpscQueue<ViewerCommand>& commands,
                 LatestSnapshotExchange<ViewerSnapshot>& snapshots,
                 BoundedSpscQueue<JointKinematicsSample>& joint_plot_queue,
                 BoundedSpscQueue<JointKinematicsSample>* joint_telemetry,
                 TelemetryBuffer* telemetry,
                 LatestSpscExchange<PicoTeleopFrame>* pico_frames,
                 PicoUdpReceiver* pico_receiver,
                 LatestSpscExchange<WujiHandTeleopFrame>* hand_frames,
                 WujiHandUdpReceiver* hand_receiver,
                 JointCommandExporter* joint_command_exporter,
                 bool pico_initially_enabled,
                 bool simulation_recovery,
                 ArmAngleReferenceMode initial_arm_angle_reference_mode,
                 std::atomic<bool>& running) {
  setInitialConfiguration(robot, config);
  const DualArmTargets initial_targets = currentTargets(robot);
  TargetManager targets(config, initial_targets);
  std::unique_ptr<DualArmController> controller = makeController(robot, config);
  const double dt = 1.0 / config.controller.rate_hz;
  std::optional<SimulationRecovery> recovery;
  const auto motionPair = [&] {
    return SimulationRecovery::Pair{controller->referenceState(ArmSide::kLeft),
                                    controller->referenceState(ArmSide::kRight)};
  };
  if(simulation_recovery)
    recovery.emplace(config,std::array<ArmLimits,2>{robot.mapping(ArmSide::kLeft).limits,
        robot.mapping(ArmSide::kRight).limits},motionPair(),dt);
  int reported_phase=-1;
  const CartesianOtgConfig initial_otg_config =
      cartesianOtgConfigForControlLevel(config, config.control_level);
  CartesianReferenceGenerator left_otg(initial_otg_config, dt);
  CartesianReferenceGenerator right_otg(initial_otg_config, dt);
  left_otg.reset(initial_targets.left);
  right_otg.reset(initial_targets.right);
  std::unique_ptr<DualArmAccelerationController> acceleration_controller =
      std::make_unique<DualArmAccelerationController>(robot, config);
  std::unique_ptr<DualArmSparkGuidance> spark_guidance;
  const bool shared_root_mode = !config.shared_root_profile_path.empty();
  std::optional<SharedRootOptions> shared_root_options;
  if (shared_root_mode) {
    shared_root_options = loadSharedRootOptions(config.shared_root_profile_path);
    if (!shared_root_options->enabled)
      throw std::runtime_error("shared-root profile changed after startup validation");
  }
  if (usesSparkGuidance(config.ik_algorithm)) {
    SparkPostureGuideMode posture_mode = SparkPostureGuideMode::kRuckig;
    if (usesSparkOtgConsistentVelocityQp(config.ik_algorithm)) {
      posture_mode =
          SparkPostureGuideMode::kOtgConsistentJointReferenceVelocity;
    } else if (usesSparkHeadroomFeedforwardVelocityQp(
                   config.ik_algorithm)) {
      posture_mode =
          SparkPostureGuideMode::kHeadroomFeedforwardJointReferenceVelocity;
    } else if (usesSparkFeedforwardVelocityQp(config.ik_algorithm)) {
      posture_mode =
          SparkPostureGuideMode::kFeedforwardJointReferenceVelocity;
    } else if (config.ik_algorithm ==
        IkAlgorithm::kSparkUpperQpoasesVelocityQp) {
      posture_mode = SparkPostureGuideMode::kJointReferenceVelocity;
    } else if (config.ik_algorithm == IkAlgorithm::kSparkDirectVelocityQp ||
               usesSparkUpperQpoasesDirect(config.ik_algorithm)) {
      posture_mode = SparkPostureGuideMode::kDirect;
    } else if (config.ik_algorithm == IkAlgorithm::kSparkPoseVelocityQp) {
      posture_mode = SparkPostureGuideMode::kDisabled;
    }
    spark_guidance = std::make_unique<DualArmSparkGuidance>(
        robot, config, shared_root_options ? shared_root_options->urdf_path
                                          : "models/marvin_m6_s_ccs_696_v4_local.urdf",
        posture_mode, shared_root_options ? &*shared_root_options : nullptr);
  }
  DualArmTargets last_spark_targets = initial_targets;
  DualArmReferences last_spark_references = directReferences(initial_targets);
  SparkGuidanceDiagnostics spark_diagnostics;
  bool joint_takeover_active = false;
  bool pico_paused = false;
  ArmMotionState direct_left_state;
  direct_left_state.q = robot.armPosition(ArmSide::kLeft);
  ArmMotionState direct_right_state;
  direct_right_state.q = robot.armPosition(ArmSide::kRight);
  const bool pico_configured = pico_frames != nullptr && pico_receiver != nullptr;
  const bool hand_configured = hand_frames != nullptr && hand_receiver != nullptr;
  PicoTeleopSession pico_session(
      config.cartesian_servo.target_timeout_seconds);
  ArmDirectionReferenceManager arm_direction_manager(
      config.arm_angle.reference_rate_limit_rad_s);
  ArmAngleReferenceMode arm_angle_reference_mode{
      initial_arm_angle_reference_mode};
  pico_session.setEnabled(pico_configured && pico_initially_enabled);
  DualArmDirectionReferences latest_pico_arm_directions;
  PicoUpperLimbSkeleton latest_pico_upper_limb_skeleton;
  std::uint64_t pico_applied_epoch = 0U;
  std::uint64_t pico_applied_sequence = 0U;
  std::uint64_t pico_reset_applies = 0U;
  std::uint64_t joint_command_epoch = 0U;
  std::uint64_t joint_command_sequence =
      joint_command_exporter != nullptr ? static_cast<std::uint64_t>(monotonicNowNs()) : 0U;
  HandCommandFreshness left_hand_freshness(ArmSide::kLeft);
  HandCommandFreshness right_hand_freshness(ArmSide::kRight);
  WujiHandHistory hand_history;
  bool hand_was_paused = false;
  std::int64_t hand_input_after_ns = 0;
  JointCommandArmReadiness joint_command_arm_readiness;
  std::uint64_t joint_command_resynchronization_generation = 0U;
  std::int64_t pico_left_source_timestamp_ns = 0;
  std::int64_t pico_right_source_timestamp_ns = 0;
  std::int64_t pico_applied_bridge_send_monotonic_ns = 0;
  double pico_receive_to_control_us = 0.0;
  double pico_bridge_to_control_us = 0.0;
  bool paused = false;
  const std::int64_t period_nanoseconds =
      static_cast<std::int64_t>(std::llround(1000000000.0 / config.controller.rate_hz));
  CycleTimeWindow cycle_times(1000U);
  double cycle_p99_us = 0.0;
  std::uint64_t sequence = 0U;
  std::uint64_t last_processed_command_id = 0U;
  std::uint64_t deadline_misses = 0U;
  std::uint64_t control_failures = 0U;
  std::uint64_t snapshot_drops = 0U;
  std::uint64_t telemetry_drops = 0U;
  std::uint64_t joint_plot_samples = 0U;
  std::uint64_t joint_plot_drops = 0U;
  JointKinematicsDifferentiator left_plot_differentiator;
  JointKinematicsDifferentiator right_plot_differentiator;
  timespec deadline{};
  clock_gettime(CLOCK_MONOTONIC, &deadline);
  deadline = addNanoseconds(deadline, period_nanoseconds);

  while (running.load(std::memory_order_acquire)) {
    const auto cycle_start = std::chrono::steady_clock::now();
    const std::int64_t monotonic_now_ns = monotonicNowNs();
    const double control_time = static_cast<double>(sequence) * dt;
    const double target_time = selectTargetTimeSeconds(
        pico_configured, control_time, monotonic_now_ns);
    bool plot_reset_requested = false;
    bool joint_command_stream_reset = false;
    bool hand_reset_requested = false;
    ViewerCommand command;
    while (commands.tryPop(command)) {
      if(recovery) {
        bool changed=false;
        if(command.type==ViewerCommandType::kSimulationStart) {
          changed=recovery->start(pico_session.freshness(monotonic_now_ns).live,motionPair());
          std::cout<<(config.ik_algorithm == IkAlgorithm::kPicoEeFrankaDls ? "DLS_SIM: S " : "CERES_SIM: S ")
                   <<(changed?"accepted":"rejected: fresh input and stationary WAIT/HOLD/Home required")<<std::endl;
        } else if(command.type==ViewerCommandType::kSimulationHome) {
          changed=recovery->stop(motionPair(),true);
          std::cout << "SIM: H " << (changed ? "accepted: braking then Home" :
              "rejected: fault, recovery already active, or invalid motion state")
                    << "; phase=" << recovery->name() << std::endl;
        } else if(command.type==ViewerCommandType::kSimulationHold) {
          changed=recovery->stop(motionPair());
        }
        if(changed) {
          hand_reset_requested = true; // Reject samples collected before this ownership change.
          controller->resetSolvers();
          if(!spark_guidance->resetMappingSession(controller->referenceState(ArmSide::kLeft),
                                   controller->referenceState(ArmSide::kRight))) {
            recovery->fault();
            std::cerr << "SIM: mapping session reset failed; recovery FAULT" << std::endl;
          }
          if(command.type==ViewerCommandType::kSimulationStart && recovery->teleop()) {
            if(!controller->beginSimulationSoftStart()) recovery->fault();
            else std::cout << "SIM: soft start: slow approach, then gradual normal-speed tracking" << std::endl;
          }
        }
        last_processed_command_id=std::max(last_processed_command_id,command.id);
        continue; // Legacy toggles/reset cannot bypass simulation ownership.
      }
      if (shared_root_mode &&
          (command.type == ViewerCommandType::kSetIkAlgorithm ||
           command.type == ViewerCommandType::kSetControlLevel)) {
        // Switching backend requires a new session with another explicit
        // profile. Never reinterpret an existing shared-root stream as legacy.
        last_processed_command_id = std::max(last_processed_command_id, command.id);
        continue;
      }
      const ControlLevel previous_control_level = config.control_level;
      const IkAlgorithm previous_algorithm = controller->algorithm();
      const bool previous_paused = paused;
      const bool previous_pico_paused = pico_paused;
      processCommand(command, robot, config, target_time, targets,
                     pico_session, pico_configured, arm_angle_reference_mode,
                     paused, pico_paused,
                     controller, acceleration_controller);
      if (shared_root_mode && command.type == ViewerCommandType::kResetNominal) {
        pico_session.setEnabled(false);
        pico_paused = true; // Reset/Home never automatically grants takeover.
      }
      if (shared_root_mode && command.type != ViewerCommandType::kResetNominal &&
          (paused != previous_paused || pico_paused != previous_pico_paused)) {
        // Process revocation even when pause/resume commands share one tick.
        if (!spark_guidance->reset(controller->referenceState(ArmSide::kLeft),
                                   controller->referenceState(ArmSide::kRight)))
          throw std::runtime_error("shared-root pause/rearm reset failed");
      }
      hand_reset_requested = hand_reset_requested ||
                             pico_paused != previous_pico_paused;
      if (pico_paused && joint_takeover_active) {
        if (spark_guidance != nullptr) {
          spark_guidance->cancelJointSpaceTakeover();
        }
        joint_takeover_active = false;
      }
      if (config.control_level != previous_control_level) {
        const CartesianOtgConfig selected_otg_config =
            cartesianOtgConfigForControlLevel(config, config.control_level);
        left_otg.reconfigure(selected_otg_config);
        right_otg.reconfigure(selected_otg_config);
      }
      if (config.control_level != previous_control_level ||
          controller->algorithm() != previous_algorithm ||
          paused != previous_paused ||
          command.type == ViewerCommandType::kResetNominal) {
        plot_reset_requested = true;
      }
      if (command.type == ViewerCommandType::kResetNominal) {
        robot.forward();
        direct_left_state = ArmMotionState{};
        direct_left_state.q = robot.armPosition(ArmSide::kLeft);
        direct_right_state = ArmMotionState{};
        direct_right_state.q = robot.armPosition(ArmSide::kRight);
        left_otg.reset(robot.tcpPose(ArmSide::kLeft));
        right_otg.reset(robot.tcpPose(ArmSide::kRight));
        last_spark_targets = currentTargets(robot);
        last_spark_references = directReferences(last_spark_targets);
        spark_diagnostics = {};
        if (spark_guidance != nullptr) {
          if (!spark_guidance->reset(direct_left_state,
                                     direct_right_state)) {
            throw std::runtime_error(
                "failed to reset SPARK guidance to initial posture");
          }
        }
      }
      last_processed_command_id = std::max(last_processed_command_id, command.id);
    }

    PicoTeleopFrame pico_frame;
    if (pico_frames != nullptr && pico_frames->tryReadLatest(pico_frame)) {
      joint_command_epoch = pico_frame.tracking_epoch;
      joint_command_stream_reset =
          pico_frame.stream_discontinuity ||
          pico_frame.resynchronization_generation !=
              joint_command_resynchronization_generation;
      joint_command_resynchronization_generation =
          pico_frame.resynchronization_generation;
      const PicoTeleopButtonAction button_action =
          pico_session.observeButton(pico_frame);
      if(recovery&&button_action==PicoTeleopButtonAction::kPause)
        recovery->stop(motionPair());
      if (button_action == PicoTeleopButtonAction::kPause) {
        pico_paused = true;
        if (spark_guidance != nullptr) {
          spark_guidance->cancelJointSpaceTakeover();
        }
        joint_takeover_active = false;
        targets.setMode(TargetMode::kHold, target_time);
      } else {
        if (button_action == PicoTeleopButtonAction::kResume) {
          pico_paused = false;
        }
        const PicoTeleopClassification classification =
            pico_session.classify(pico_frame, monotonic_now_ns);
        if (classification.action == PicoTeleopAction::kApply ||
            classification.action == PicoTeleopAction::kResetEpochAndApply) {
          const double source_seconds =
              static_cast<double>(pico_frame.source_timestamp_ns) * 1.0e-9;
          const bool reset_epoch =
              classification.action == PicoTeleopAction::kResetEpochAndApply;
          if(recovery&&recovery->teleop()&&reset_epoch&&pico_applied_epoch!=0)
            recovery->stop(motionPair()); // Reset never grants automatic re-entry.
          const DualArmTargets current = currentTargets(robot);
          bool target_accepted = false;
          TargetManager candidate = reset_epoch
                                        ? TargetManager(config, current)
                                        : targets;
          if (spark_guidance != nullptr &&
              usesSparkGuidance(controller->algorithm())) {
            const bool spark_direct_qpos =
                usesSparkUpperQpoasesDirect(controller->algorithm());
            if (reset_epoch) {
              joint_takeover_active = false;
              spark_guidance->cancelJointSpaceTakeover();
              (void)spark_guidance->reset(
                  spark_direct_qpos
                      ? direct_left_state
                      : controller->referenceState(ArmSide::kLeft),
                  spark_direct_qpos
                      ? direct_right_state
                      : controller->referenceState(ArmSide::kRight));
            }
            SparkUpperTargets spark_targets;
            if (shared_root_mode) {
              target_accepted = spark_guidance->updateSharedRootFrame(pico_frame, monotonic_now_ns);
            } else {
              spark_targets = spark_guidance->updatePicoFrame(pico_frame);
              target_accepted = spark_targets.valid;
            }
            if (!shared_root_mode && target_accepted && reset_epoch && !spark_direct_qpos &&
                controller->algorithm() != IkAlgorithm::kSparkPoseVelocityQp) {
              const ArmMotionState left_alignment_model =
                  spark_direct_qpos
                      ? direct_left_state
                      : controller->referenceState(ArmSide::kLeft);
              const ArmMotionState right_alignment_model =
                  spark_direct_qpos
                      ? direct_right_state
                      : controller->referenceState(ArmSide::kRight);
              joint_takeover_active = spark_guidance->startJointSpaceTakeover(
                  spark_targets, left_alignment_model, right_alignment_model);
            }
          } else {
            candidate.setMode(TargetMode::kManual, target_time);
            target_accepted = candidate.setManualTargets(
                pico_frame.left, pico_frame.right, source_seconds,
                monotonicTimestampSeconds(pico_frame.receive_monotonic_ns));
          }
          if (target_accepted) {
            if (reset_epoch && !config.controller.model_state_only) {
              const bool synchronized =
                  config.control_level == ControlLevel::kAcceleration
                      ? acceleration_controller->synchronizeReferencesToActual()
                      : controller->synchronizeReferencesToActual();
              if (!synchronized) {
                continue;
              }
            }
            if (spark_guidance == nullptr ||
                !usesSparkGuidance(controller->algorithm())) {
              targets = std::move(candidate);
            }
            if (reset_epoch) {
              // A PICO epoch reset changes the target stream, not the robot's
              // commanded Cartesian state.  Keep the OTG state continuous and
              // let it move toward the newly accepted target under its normal
              // velocity, acceleration, and jerk limits.  Resetting from the
              // MuJoCo feedback pose here teleported the reference whenever
              // the model reference and simulated feedback had separated.
              plot_reset_requested = true;
              ++pico_reset_applies;
            }
            pico_session.commitApplied(pico_frame);
            pico_applied_bridge_send_monotonic_ns =
                pico_frame.bridge_send_monotonic_ns;
            latest_pico_upper_limb_skeleton = pico_frame.upper_limb_skeleton;
            if (pico_frame.left_arm_direction.valid) {
              latest_pico_arm_directions.left = pico_frame.left_arm_direction;
            }
            if (pico_frame.right_arm_direction.valid) {
              latest_pico_arm_directions.right = pico_frame.right_arm_direction;
            }
            pico_applied_epoch = pico_frame.tracking_epoch;
            pico_applied_sequence = pico_frame.sequence;
            pico_left_source_timestamp_ns = pico_frame.source_timestamp_ns;
            pico_right_source_timestamp_ns = pico_frame.source_timestamp_ns;
            pico_receive_to_control_us = 1.0e-3 * static_cast<double>(
                std::max<std::int64_t>(
                    0, monotonic_now_ns - pico_frame.receive_monotonic_ns));
            pico_bridge_to_control_us = 1.0e-3 * static_cast<double>(
                std::max<std::int64_t>(
                    0, monotonic_now_ns -
                           pico_frame.bridge_send_monotonic_ns));
          }
        }
      }
    }

    const PicoTeleopFreshness pico_freshness =
        pico_session.freshness(monotonic_now_ns);
    if(recovery&&recovery->teleop()&&!pico_freshness.live)
      recovery->stop(motionPair());
    const PicoReceiverStats pico_stats =
        pico_receiver != nullptr ? pico_receiver->stats() : PicoReceiverStats{};
    if (spark_guidance != nullptr &&
        usesSparkGuidance(controller->algorithm())) {
      arm_angle_reference_mode = ArmAngleReferenceMode::kOutwardOnly;
    }
    const DualArmDirectionReferences requested_arm_directions =
        selectArmDirectionReferences(arm_angle_reference_mode,
                                     pico_freshness.live,
                                     latest_pico_arm_directions);
    const DualArmDirectionReferences arm_directions =
        arm_direction_manager.update(requested_arm_directions, dt);

    DualArmTargets desired = targets.sample(target_time);
    const bool spark_mode = spark_guidance != nullptr &&
                            usesSparkGuidance(controller->algorithm());
    bool joint_takeover_cycle = false;
    if (spark_mode) {
      if (!pico_freshness.live) {
        if (joint_takeover_active) {
          spark_guidance->cancelJointSpaceTakeover();
          joint_takeover_active = false;
        }
        if (!shared_root_mode) spark_guidance->invalidateTarget("spark_pico_stale");
      }
      joint_takeover_cycle = joint_takeover_active;
      const bool spark_direct_qpos =
          usesSparkUpperQpoasesDirect(controller->algorithm());
      const ArmMotionState left_spark_model =
          spark_direct_qpos ? direct_left_state
                            : controller->referenceState(ArmSide::kLeft);
      const ArmMotionState right_spark_model =
          spark_direct_qpos ? direct_right_state
                            : controller->referenceState(ArmSide::kRight);
      spark_diagnostics = shared_root_mode
                              ? spark_guidance->stepSharedRoot(
                                    left_spark_model, right_spark_model, dt,
                                    monotonic_now_ns,
                                    !paused && !pico_paused && pico_session.enabled() &&
                                        (!recovery || recovery->teleop()),
                                    left_spark_model.q.allFinite() && left_spark_model.qdot.allFinite() &&
                                    left_spark_model.qddot.allFinite() && right_spark_model.q.allFinite() &&
                                    right_spark_model.qdot.allFinite() && right_spark_model.qddot.allFinite())
                              : joint_takeover_active
                              ? spark_guidance->stepJointSpaceTakeover(
                                    left_spark_model, right_spark_model, dt)
                              : spark_guidance->step(
                                    left_spark_model, right_spark_model, dt);
      if (spark_diagnostics.accepted) {
        last_spark_targets = spark_diagnostics.cartesian_targets;
        if (spark_diagnostics.cartesian_references_valid) {
          last_spark_references = spark_diagnostics.cartesian_references;
        }
      }
      if (joint_takeover_cycle &&
          (!spark_diagnostics.accepted ||
           spark_diagnostics.joint_takeover_finished)) {
        joint_takeover_active = false;
      }
      desired = last_spark_targets;
      desired.left_stale = !pico_freshness.live ||
                           !spark_diagnostics.accepted ||
                           (shared_root_mode && !spark_diagnostics.target_valid);
      desired.right_stale = desired.left_stale;
    }
    const bool spark_internal_otg =
        usesSparkOtgConsistentVelocityQp(controller->algorithm());
    const bool spark_internal_feedforward =
        usesSparkFeedforwardVelocityQp(controller->algorithm());
    const bool spark_internal_reference =
        spark_internal_otg || spark_internal_feedforward;
    DualArmReferences references =
        (joint_takeover_cycle && spark_diagnostics.accepted &&
         spark_diagnostics.cartesian_references_valid)
            ? spark_diagnostics.cartesian_references
            : (spark_internal_reference ? last_spark_references
                                        : directReferences(desired));
    const bool spark_direct_qpos =
        usesSparkUpperQpoasesDirect(controller->algorithm());
    if (shared_root_mode && desired.left_stale) {
      // Cached references may have been fresh when accepted. They must not
      // mask the current mapping's stale/invalid/disabled state.
      references.left.stale = references.right.stale = true;
      references.left.twist.setZero();references.right.twist.setZero();
    }
    const bool spark_joint_reference_velocity =
        controller->algorithm() ==
        IkAlgorithm::kSparkUpperQpoasesVelocityQp;
    if (config.cartesian_otg.enabled && !usesSharedRootDirectIk(controller->algorithm()) && !spark_direct_qpos &&
        !spark_joint_reference_velocity && !spark_internal_reference) {
      references.left = left_otg.update(
          desired.left, desired.left_twist, desired.left_stale, dt);
      references.right = right_otg.update(
          desired.right, desired.right_twist, desired.right_stale, dt);
    }
    ControllerDiagnostics diagnostics;
    AccelerationControllerDiagnostics acceleration_diagnostics;
    controller->setArmAngleReferenceMode(arm_angle_reference_mode);
    acceleration_controller->setArmAngleReferenceMode(
        arm_angle_reference_mode);
    bool recovery_plot_valid = false;
    const bool recovery_home_sample = recovery &&
        recovery->phase() == SimulationRecovery::Phase::kHoming;
    if(recovery&&!recovery->teleop()) {
      const auto next=recovery->update(motionPair());
      // Recovery validates a bilateral candidate before either arm is applied.
      if(!controller->setReferenceState(ArmSide::kLeft,next[0]) ||
         !controller->setReferenceState(ArmSide::kRight,next[1]))
        throw std::runtime_error("Simulation recovery reference validation failed");
      robot.forward();
      recovery_plot_valid = recovery->phase() != SimulationRecovery::Phase::kFault;
      // Waiting/braking/Home are intentional non-tracking states, not NaNs.
      diagnostics.hold_reason = diagnostics.left.hold_reason = diagnostics.right.hold_reason =
          recovery_plot_valid ? HoldReason::kNone : HoldReason::kSolverFailure;
      diagnostics.left.safety = diagnostics.right.safety = {false, diagnostics.hold_reason, -1};
      diagnostics.left.q_ref=next[0].q;diagnostics.right.q_ref=next[1].q;
      diagnostics.left.current=robot.tcpPose(ArmSide::kLeft);
      diagnostics.right.current=robot.tcpPose(ArmSide::kRight);
      diagnostics.left.q_actual=robot.armPosition(ArmSide::kLeft);
      diagnostics.right.q_actual=robot.armPosition(ArmSide::kRight);
      diagnostics.left.tcp_actual=diagnostics.left.current;
      diagnostics.right.tcp_actual=diagnostics.right.current;
      diagnostics.left.target=diagnostics.left.current;
      diagnostics.right.target=diagnostics.right.current;
      diagnostics.left.reference.pose=diagnostics.left.current;
      diagnostics.right.reference.pose=diagnostics.right.current;
    } else if (!paused && !pico_paused) {
      if (spark_diagnostics.reference_reset_required) controller->resetSolvers();
      if (spark_direct_qpos) {
        const SparkQpoasesDirectCommand direct_command =
            makeDirectCommand(spark_diagnostics);
        const auto update_direct_state = [dt](const Vec7& joint_command,
                                               ArmMotionState& state) {
          const Vec7 velocity = (joint_command - state.q) / dt;
          const Vec7 acceleration = (velocity - state.qdot) / dt;
          state.q = joint_command;
          state.qdot = velocity;
          state.qddot = acceleration;
        };
        if (direct_command.accepted) {
          update_direct_state(direct_command.left, direct_left_state);
          update_direct_state(direct_command.right, direct_right_state);
        } else {
          update_direct_state(direct_left_state.q, direct_left_state);
          update_direct_state(direct_right_state.q, direct_right_state);
        }
        robot.setArmState(ArmSide::kLeft, direct_left_state.q,
                          direct_left_state.qdot);
        robot.setArmState(ArmSide::kRight, direct_right_state.q,
                          direct_right_state.qdot);
        robot.forward();

        const auto fill_direct_diagnostics =
            [&robot](ArmSide side, const ArmMotionState& state,
                     const CartesianReference& reference,
                     ArmControllerDiagnostics& arm) {
              arm.accepted = true;
              arm.hold_reason = HoldReason::kNone;
              arm.reference = reference;
              arm.target = reference.pose;
              arm.q_ref = state.q;
              arm.q_actual = robot.armPosition(side);
              arm.current = robot.tcpPose(side);
              arm.tcp_actual = arm.current;
              arm.pose_error = poseErrorWorld(arm.target, arm.current);
              arm.actual_pose_error = arm.pose_error;
              arm.ik.status = SolverStatus::kSolved;
              arm.ik.detail = "spark_qpoases_direct_qpos";
              arm.ik.qdot = state.qdot;
            };
        fill_direct_diagnostics(ArmSide::kLeft, direct_left_state,
                                references.left, diagnostics.left);
        fill_direct_diagnostics(ArmSide::kRight, direct_right_state,
                                references.right, diagnostics.right);
        diagnostics.accepted = true;
        diagnostics.hold_reason = HoldReason::kNone;
      } else if (config.control_level == ControlLevel::kAcceleration) {
        acceleration_diagnostics = acceleration_controller->step(
            references, arm_directions, dt);
      } else {
        if (spark_mode && spark_diagnostics.accepted) {
          diagnostics = controller->step(
              references, arm_directions, spark_diagnostics.posture_tasks,
              dt);
        } else {
          diagnostics = config.cartesian_otg.enabled
                            ? controller->step(references, arm_directions, dt)
                            : controller->step(desired, arm_directions, dt);
        }
      }
      const bool accepted = config.control_level == ControlLevel::kAcceleration
                                ? acceleration_diagnostics.accepted
                                : diagnostics.accepted;
      if (!accepted) {
        ++control_failures;
        if(recovery&&pico_freshness.live&&!desired.left_stale&&!desired.right_stale)
          recovery->stop(motionPair());
      }
      if (shared_root_mode) {
        (void)spark_guidance->confirmSharedRootReference(
            spark_diagnostics.shared_root_cycle,
            accepted && diagnostics.left.accepted && diagnostics.right.accepted &&
            pico_freshness.live && spark_diagnostics.target_valid);
      }
      if (spark_guidance != nullptr &&
          usesSparkHeadroomFeedforwardVelocityQp(controller->algorithm()) &&
          config.control_level == ControlLevel::kVelocity &&
          !joint_takeover_cycle) {
        const bool pico_headroom_feedback_valid =
            pico_freshness.live && spark_diagnostics.accepted;
        const auto feedback = [&controller, pico_headroom_feedback_valid](
                                  ArmSide side,
                                  const ArmControllerDiagnostics& arm) {
          SparkConstraintHeadroomFeedback value;
          value.accepted = pico_headroom_feedback_valid && arm.accepted &&
                           arm.ik.status == SolverStatus::kSolved;
          value.qdot = arm.ik.qdot;
          value.qddot = controller->previousAcceleration(side);
          value.task_scale_position = arm.ik.task_scale_position;
          value.task_scale_orientation = arm.ik.task_scale_orientation;
          return value;
        };
        spark_guidance->updateHeadroomFeedback(
            feedback(ArmSide::kLeft, diagnostics.left),
            feedback(ArmSide::kRight, diagnostics.right), dt);
      }
    } else {
      robot.forward();
      diagnostics.left.q_ref =
          config.control_level == ControlLevel::kAcceleration
              ? acceleration_controller->positionReference(ArmSide::kLeft)
              : controller->reference(ArmSide::kLeft);
      diagnostics.right.q_ref =
          config.control_level == ControlLevel::kAcceleration
              ? acceleration_controller->positionReference(ArmSide::kRight)
              : controller->reference(ArmSide::kRight);
      diagnostics.left.q_actual = robot.armPosition(ArmSide::kLeft);
      diagnostics.right.q_actual = robot.armPosition(ArmSide::kRight);
      diagnostics.left.tcp_actual = robot.tcpPose(ArmSide::kLeft);
      diagnostics.right.tcp_actual = robot.tcpPose(ArmSide::kRight);
      diagnostics.left.current = robot.armKinematicsAt(
          ArmSide::kLeft, diagnostics.left.q_ref).tcp_pose;
      diagnostics.right.current = robot.armKinematicsAt(
          ArmSide::kRight, diagnostics.right.q_ref).tcp_pose;
      diagnostics.left.reference = config.cartesian_otg.enabled
                                       ? references.left
                                       : CartesianReference{desired.left};
      diagnostics.right.reference = config.cartesian_otg.enabled
                                        ? references.right
                                        : CartesianReference{desired.right};
      diagnostics.left.target = diagnostics.left.reference.pose;
      diagnostics.right.target = diagnostics.right.reference.pose;
      diagnostics.left.pose_error = poseErrorWorld(
          diagnostics.left.target, diagnostics.left.current);
      diagnostics.right.pose_error = poseErrorWorld(
          diagnostics.right.target, diagnostics.right.current);
      diagnostics.left.actual_pose_error =
          poseErrorWorld(diagnostics.left.target, diagnostics.left.tcp_actual);
      diagnostics.right.actual_pose_error =
          poseErrorWorld(diagnostics.right.target, diagnostics.right.tcp_actual);
      diagnostics.hold_reason = HoldReason::kNone;
    }

    const bool hands_paused = recovery ? recovery->handsPaused(paused, pico_paused)
                                       : paused || pico_paused;
    if (hands_paused || hand_was_paused || plot_reset_requested ||
        joint_command_stream_reset || hand_reset_requested) {
      hand_history.clear();
      left_hand_freshness.reset();
      right_hand_freshness.reset();
      hand_input_after_ns = monotonic_now_ns;
    }
    hand_was_paused = hands_paused;
    if (hand_frames != nullptr) {
      WujiHandTeleopFrame hand_frame;
      // Drain while paused; never replay a queued pre-resume source sample.
      if (hand_frames->tryReadLatest(hand_frame) && !hands_paused) {
        if (hand_frame.left_source_timestamp_ns <= hand_input_after_ns) {
          hand_frame.left_valid = false;
          hand_frame.left_source_timestamp_ns = 0;
        }
        if (hand_frame.right_source_timestamp_ns <= hand_input_after_ns) {
          hand_frame.right_valid = false;
          hand_frame.right_source_timestamp_ns = 0;
        }
        if (hand_history.observe(hand_frame)) {
          left_hand_freshness.observe(hand_frame);
          right_hand_freshness.observe(hand_frame);
        }
      }
    }
    const std::int64_t hand_sample_time_ns = monotonicNowNs();
    const WujiHandSample hand_sample = hand_history.sample(hand_sample_time_ns);
    const bool left_hand_applied = !hands_paused && hand_sample.left_valid &&
                                  left_hand_freshness.live(hand_sample_time_ns);
    const bool right_hand_applied = !hands_paused && hand_sample.right_valid &&
                                   right_hand_freshness.live(hand_sample_time_ns);
    if (left_hand_applied) {
      robot.setHandPosition(ArmSide::kLeft, handVector(hand_sample.left));
    }
    if (right_hand_applied) {
      robot.setHandPosition(ArmSide::kRight, handVector(hand_sample.right));
    }
    if (left_hand_applied || right_hand_applied) {
      robot.forward();
    }

    const WujiHandReceiverStats hand_stats =
        hand_receiver != nullptr ? hand_receiver->stats()
                                  : WujiHandReceiverStats{};

    ViewerSnapshot snapshot;
    snapshot.sequence = sequence;
    if(recovery) {
      snapshot.simulation_phase=static_cast<int>(recovery->phase());
      if(snapshot.simulation_phase!=reported_phase) {
        reported_phase=snapshot.simulation_phase;
        std::cout<<(config.ik_algorithm == IkAlgorithm::kPicoEeFrankaDls ? "DLS_SIM: " : "CERES_SIM: ")
                 <<recovery->name()<<std::endl;
      }
    }
    snapshot.last_processed_command_id = last_processed_command_id;
    snapshot.control_time_seconds = control_time;
    snapshot.left_q = robot.armPosition(ArmSide::kLeft);
    snapshot.right_q = robot.armPosition(ArmSide::kRight);
    if (robot.hasHandMappings()) {
      snapshot.left_hand_q = robot.handPosition(ArmSide::kLeft);
      snapshot.right_hand_q = robot.handPosition(ArmSide::kRight);
    }
    const Vec7 left_nominal =
        0.5 * (robot.mapping(ArmSide::kLeft).limits.lower_position +
               robot.mapping(ArmSide::kLeft).limits.upper_position);
    const Vec7 right_nominal =
        0.5 * (robot.mapping(ArmSide::kRight).limits.lower_position +
               robot.mapping(ArmSide::kRight).limits.upper_position);
    snapshot.at_nominal_configuration =
        (snapshot.left_q - left_nominal).cwiseAbs().maxCoeff() <= 1e-10 &&
        (snapshot.right_q - right_nominal).cwiseAbs().maxCoeff() <= 1e-10;
    snapshot.targets = desired;
    snapshot.algorithm = controller->algorithm();
    snapshot.control_level = config.control_level;
    snapshot.backend = SolverBackend::kQpoases;
    snapshot.mode = targets.mode();
    snapshot.paused = paused;
    snapshot.accepted = config.control_level == ControlLevel::kAcceleration
                            ? acceleration_diagnostics.accepted
                            : diagnostics.accepted;
    snapshot.hold_reason = config.control_level == ControlLevel::kAcceleration
                               ? acceleration_diagnostics.hold_reason
                               : diagnostics.hold_reason;
    snapshot.left_target_stale = desired.left_stale;
    snapshot.right_target_stale = desired.right_stale;
    snapshot.otg_enabled =
        spark_internal_otg ||
        (config.cartesian_otg.enabled && !spark_direct_qpos &&
         !spark_joint_reference_velocity);
    snapshot.hand_configured = hand_configured;
    snapshot.hand_stale = hand_stats.stale;
    snapshot.hand_live = hand_configured && !hand_stats.stale;
    snapshot.hand_sequence = hand_stats.sequence;
    snapshot.hand_datagrams = hand_stats.datagrams;
    snapshot.hand_accepted = hand_stats.accepted;
    snapshot.hand_malformed = hand_stats.malformed;
    snapshot.hand_crc_failures = hand_stats.crc_failures;
    snapshot.hand_reordered = hand_stats.reordered;
    snapshot.left_position_error = diagnostics.left.pose_error.head<3>().norm();
    snapshot.left_orientation_error = diagnostics.left.pose_error.tail<3>().norm();
    snapshot.right_position_error = diagnostics.right.pose_error.head<3>().norm();
    snapshot.right_orientation_error = diagnostics.right.pose_error.tail<3>().norm();
    snapshot.left_solver_status = diagnostics.left.ik.status;
    snapshot.right_solver_status = diagnostics.right.ik.status;
    snapshot.left_ik.accepted = diagnostics.left.accepted;
    snapshot.left_ik.fallback_applied = diagnostics.left.fallback_applied;
    snapshot.left_ik.hold_reason = diagnostics.left.hold_reason;
    snapshot.left_ik.position_error = snapshot.left_position_error;
    snapshot.left_ik.orientation_error = snapshot.left_orientation_error;
    snapshot.left_ik.actual_position_error =
        diagnostics.left.actual_pose_error.head<3>().norm();
    snapshot.left_ik.actual_orientation_error =
        diagnostics.left.actual_pose_error.tail<3>().norm();
    snapshot.left_ik.slack_position_norm = diagnostics.left.ik.slack.head<3>().norm();
    snapshot.left_ik.slack_orientation_norm = diagnostics.left.ik.slack.tail<3>().norm();
    snapshot.left_ik.equality_residual = diagnostics.left.ik.equality_residual;
    snapshot.left_ik.reference_error_max_abs = diagnostics.left.reference_error_max_abs;
    snapshot.left_ik.reference_scale = diagnostics.left.reference_scale;
    snapshot.left_ik.reference_frozen = diagnostics.left.reference_frozen;
    snapshot.left_ik.qdot_max_ratio = diagnostics.left.ik.qdot_max_ratio;
    snapshot.left_ik.qddot_max_ratio =
        (controller->previousAcceleration(ArmSide::kLeft).cwiseAbs().array() /
         config.joint_limits.max_acceleration_rad_s2.array())
            .maxCoeff();
    snapshot.left_ik.task_scale_position =
        diagnostics.left.ik.task_scale_position;
    snapshot.left_ik.task_scale_orientation =
        diagnostics.left.ik.task_scale_orientation;
    snapshot.left_ik.solve_time_us = diagnostics.left.ik.solve_time_us;
    snapshot.left_ik.active_position_bounds =
        diagnostics.left.ik.active_position_bound_count;
    snapshot.left_ik.active_velocity_bounds =
        diagnostics.left.ik.active_velocity_bound_count;
    snapshot.left_ik.active_acceleration_bounds =
        diagnostics.left.ik.active_acceleration_bound_count;
    snapshot.left_ik.active_braking_bounds =
        diagnostics.left.ik.active_braking_bound_count;
    snapshot.left_ik.iterations = diagnostics.left.ik.iterations;
    snapshot.left_ik.status = diagnostics.left.ik.status;
    snapshot.left_ik.otg_valid = diagnostics.left.reference.valid;
    snapshot.left_ik.otg_stale = diagnostics.left.reference.stale;
    snapshot.left_ik.reference_linear_velocity =
        diagnostics.left.reference.twist.head<3>().norm();
    snapshot.left_ik.reference_angular_velocity =
        diagnostics.left.reference.twist.tail<3>().norm();
    snapshot.left_ik.reference_linear_acceleration =
        diagnostics.left.reference.acceleration.head<3>().norm();
    snapshot.left_ik.reference_angular_acceleration =
        diagnostics.left.reference.acceleration.tail<3>().norm();
    snapshot.left_ik.arm_angle_reference_source =
        diagnostics.left.arm_angle.reference_source;
    snapshot.left_ik.arm_angle_active =
        diagnostics.left.arm_angle_task_active;
    snapshot.left_ik.arm_angle_error_rad = diagnostics.left.arm_angle.error_rad;
    snapshot.left_ik.arm_angle_robot_rad =
        diagnostics.left.arm_angle.robot_angle_rad;
    snapshot.left_ik.arm_angle_target_rad =
        diagnostics.left.arm_angle.target_angle_rad;
    snapshot.left_ik.arm_angle_control_error_rad =
        diagnostics.left.arm_angle.control_error_rad;
    snapshot.left_ik.arm_angle_current_rate_rad_s =
        diagnostics.left.arm_angle_current_rate;
    snapshot.left_ik.arm_angle_requested_velocity_rad_s =
        diagnostics.left.arm_angle_requested_rate;
    snapshot.left_ik.arm_angle_radius_m = diagnostics.left.arm_angle.radius_m;
    snapshot.left_ik.arm_angle_reference_projection_norm =
        diagnostics.left.arm_angle.requested_reference_projection_norm;
    snapshot.left_ik.arm_angle_jacobian_norm =
        diagnostics.left.arm_angle.jacobian_norm;
    snapshot.left_ik.arm_angle_projection_held =
        diagnostics.left.arm_angle.reference_projection_held;
    snapshot.left_ik.arm_angle_reference_governor_held =
        diagnostics.left.arm_angle.reference_governor_held;
    snapshot.left_ik.arm_angle_branch_lock_active =
        diagnostics.left.arm_angle.branch_lock_active;
    snapshot.left_ik.arm_angle_branch_lock_distance_m =
        diagnostics.left.arm_angle.branch_lock_distance_m;
    snapshot.left_ik.arm_angle_branch_lock_constraint_active =
        diagnostics.left.arm_angle.branch_lock_constraint_active;
    snapshot.left_ik.arm_angle_branch_lock_requested_lower =
        diagnostics.left.arm_angle.branch_lock_requested_lower;
    snapshot.left_ik.arm_angle_branch_lock_effective_lower =
        diagnostics.left.arm_angle.branch_lock_effective_lower;
    snapshot.left_ik.arm_angle_branch_lock_feasibility_clipped =
        diagnostics.left.arm_angle.branch_lock_feasibility_clipped;
    snapshot.left_ik.elbow_world_z = diagnostics.left.arm_angle.elbow_world_z;
    snapshot.left_ik.shoulder_world_z =
        diagnostics.left.arm_angle.shoulder_world_z;
    snapshot.left_ik.upper_arm_outward_active =
        diagnostics.left.upper_arm_outward.constraint_active;
    snapshot.left_ik.upper_arm_outward_distance_m =
        diagnostics.left.upper_arm_outward.state.distance_m;
    snapshot.left_ik.upper_arm_outward_requested_lower =
        diagnostics.left.upper_arm_outward.requested_lower;
    snapshot.left_ik.upper_arm_outward_effective_lower =
        diagnostics.left.upper_arm_outward.effective_lower;
    snapshot.left_ik.upper_arm_outward_achieved =
        diagnostics.left.upper_arm_outward.achieved;
    snapshot.left_ik.upper_arm_outward_residual =
        diagnostics.left.upper_arm_outward.residual;
    snapshot.left_ik.upper_arm_outward_feasibility_clipped =
        diagnostics.left.upper_arm_outward.feasibility_clipped;
    snapshot.left_ik.dls_posture_reference_active =
        diagnostics.left.dls_posture_reference_active;
    snapshot.left_ik.dls_posture_status =
        static_cast<int>(diagnostics.left.dls_posture_status);
    snapshot.left_ik.dls_posture_iterations =
        diagnostics.left.dls_posture_iterations;
    snapshot.left_ik.dls_posture_joint_projection_count =
        diagnostics.left.dls_posture_joint_projection_count;
    snapshot.left_ik.dls_posture_ruckig_accepted =
        diagnostics.left.dls_posture_ruckig_accepted;
    snapshot.left_ik.dls_posture_solve_time_us =
        diagnostics.left.dls_posture_solve_time_us;
    snapshot.left_ik.ee_ik_wall_time_us = diagnostics.left.ee_ik_wall_time_us;
    snapshot.left_ik.ee_ruckig_wall_time_us = diagnostics.left.ee_ruckig_wall_time_us;
    snapshot.left_ik.ee_ik_to_ruckig_wall_time_us = diagnostics.left.ee_ik_to_ruckig_wall_time_us;
    snapshot.left_ik.ee_ruckig_invoked = diagnostics.left.ee_ruckig_invoked;
    snapshot.left_ik.ee_pinocchio_kinematics = diagnostics.left.ee_pinocchio_kinematics;
    snapshot.left_ik.dls_posture_initial_position_error_m =
        diagnostics.left.dls_posture_initial_position_error_m;
    snapshot.left_ik.dls_posture_initial_orientation_error_rad =
        diagnostics.left.dls_posture_initial_orientation_error_rad;
    snapshot.left_ik.dls_posture_final_position_error_m =
        diagnostics.left.dls_posture_final_position_error_m;
    snapshot.left_ik.dls_posture_final_orientation_error_rad =
        diagnostics.left.dls_posture_final_orientation_error_rad;
    snapshot.left_ik.dls_posture_goal_error_max_abs =
        (diagnostics.left.dls_posture_goal - diagnostics.left.q_ref)
            .cwiseAbs().maxCoeff();
    snapshot.left_ik.dls_posture_reference_error_max_abs =
        (diagnostics.left.dls_posture_reference - diagnostics.left.q_ref)
            .cwiseAbs().maxCoeff();
    snapshot.left_ik.dls_posture_velocity_target_max_abs =
        diagnostics.left.dls_posture_velocity_target.cwiseAbs().maxCoeff();
    snapshot.left_ik.dls_posture_qdot_error_max_abs =
        (diagnostics.left.dls_posture_velocity_target - diagnostics.left.ik.qdot)
            .cwiseAbs().maxCoeff();
    snapshot.left_ik.dls_posture_goal_limit_margin_rad =
        diagnostics.left.dls_posture_goal_limit_margin_rad;
    snapshot.left_ik.spark_posture_active =
        spark_mode && !spark_direct_qpos &&
        spark_diagnostics.posture_tasks.left.active;
    snapshot.left_ik.spark_ik_accepted = spark_diagnostics.left.ik.accepted;
    snapshot.left_ik.spark_stage1_iterations =
        spark_diagnostics.left.ik.stage1_iterations;
    snapshot.left_ik.spark_stage2_iterations =
        spark_diagnostics.left.ik.stage2_iterations;
    snapshot.left_ik.spark_solve_time_us =
        spark_diagnostics.left.ik.stage1_solve_time_us +
        spark_diagnostics.left.ik.stage2_solve_time_us;
    snapshot.left_ik.spark_palm_position_error_m =
        spark_diagnostics.left.ik.palm_position_error;
    snapshot.left_ik.spark_palm_orientation_error_rad =
        spark_diagnostics.left.ik.palm_orientation_error;
    snapshot.left_ik.spark_reference_velocity_ratio =
        spark_diagnostics.left.reference.velocity_ratio;
    snapshot.left_ik.spark_reference_acceleration_ratio =
        spark_diagnostics.left.reference.acceleration_ratio;
    snapshot.left_ik.spark_reference_jerk_ratio =
        spark_diagnostics.left.reference.jerk_ratio;
    snapshot.left_ik.spark_q_ik_error_max_abs =
        spark_mode && spark_diagnostics.left.ik.accepted
            ? (spark_diagnostics.left.q_ik - diagnostics.left.q_ref)
                  .cwiseAbs().maxCoeff()
            : 0.0;
    snapshot.left_ik.spark_q_ref_error_max_abs =
        spark_mode && spark_diagnostics.left.reference.accepted
            ? (spark_diagnostics.left.reference.state.q - diagnostics.left.q_ref)
                  .cwiseAbs().maxCoeff()
            : 0.0;
    snapshot.left_ik.spark_posture_velocity_max_abs =
        spark_mode && !spark_direct_qpos
            ? spark_diagnostics.posture_tasks.left.target.cwiseAbs().maxCoeff()
            : 0.0;
    snapshot.right_ik.accepted = diagnostics.right.accepted;
    snapshot.right_ik.fallback_applied = diagnostics.right.fallback_applied;
    snapshot.right_ik.hold_reason = diagnostics.right.hold_reason;
    snapshot.right_ik.position_error = snapshot.right_position_error;
    snapshot.right_ik.orientation_error = snapshot.right_orientation_error;
    snapshot.right_ik.actual_position_error =
        diagnostics.right.actual_pose_error.head<3>().norm();
    snapshot.right_ik.actual_orientation_error =
        diagnostics.right.actual_pose_error.tail<3>().norm();
    snapshot.right_ik.slack_position_norm = diagnostics.right.ik.slack.head<3>().norm();
    snapshot.right_ik.slack_orientation_norm = diagnostics.right.ik.slack.tail<3>().norm();
    snapshot.right_ik.equality_residual = diagnostics.right.ik.equality_residual;
    snapshot.right_ik.reference_error_max_abs = diagnostics.right.reference_error_max_abs;
    snapshot.right_ik.reference_scale = diagnostics.right.reference_scale;
    snapshot.right_ik.reference_frozen = diagnostics.right.reference_frozen;
    snapshot.right_ik.qdot_max_ratio = diagnostics.right.ik.qdot_max_ratio;
    snapshot.right_ik.qddot_max_ratio =
        (controller->previousAcceleration(ArmSide::kRight).cwiseAbs().array() /
         config.joint_limits.max_acceleration_rad_s2.array())
            .maxCoeff();
    snapshot.right_ik.task_scale_position =
        diagnostics.right.ik.task_scale_position;
    snapshot.right_ik.task_scale_orientation =
        diagnostics.right.ik.task_scale_orientation;
    snapshot.right_ik.solve_time_us = diagnostics.right.ik.solve_time_us;
    snapshot.right_ik.active_position_bounds =
        diagnostics.right.ik.active_position_bound_count;
    snapshot.right_ik.active_velocity_bounds =
        diagnostics.right.ik.active_velocity_bound_count;
    snapshot.right_ik.active_acceleration_bounds =
        diagnostics.right.ik.active_acceleration_bound_count;
    snapshot.right_ik.active_braking_bounds =
        diagnostics.right.ik.active_braking_bound_count;
    snapshot.right_ik.iterations = diagnostics.right.ik.iterations;
    snapshot.right_ik.status = diagnostics.right.ik.status;
    snapshot.right_ik.otg_valid = diagnostics.right.reference.valid;
    snapshot.right_ik.otg_stale = diagnostics.right.reference.stale;
    snapshot.right_ik.reference_linear_velocity =
        diagnostics.right.reference.twist.head<3>().norm();
    snapshot.right_ik.reference_angular_velocity =
        diagnostics.right.reference.twist.tail<3>().norm();
    snapshot.right_ik.reference_linear_acceleration =
        diagnostics.right.reference.acceleration.head<3>().norm();
    snapshot.right_ik.reference_angular_acceleration =
        diagnostics.right.reference.acceleration.tail<3>().norm();
    snapshot.right_ik.arm_angle_reference_source =
        diagnostics.right.arm_angle.reference_source;
    snapshot.right_ik.arm_angle_active =
        diagnostics.right.arm_angle_task_active;
    snapshot.right_ik.arm_angle_error_rad =
        diagnostics.right.arm_angle.error_rad;
    snapshot.right_ik.arm_angle_robot_rad =
        diagnostics.right.arm_angle.robot_angle_rad;
    snapshot.right_ik.arm_angle_target_rad =
        diagnostics.right.arm_angle.target_angle_rad;
    snapshot.right_ik.arm_angle_control_error_rad =
        diagnostics.right.arm_angle.control_error_rad;
    snapshot.right_ik.arm_angle_current_rate_rad_s =
        diagnostics.right.arm_angle_current_rate;
    snapshot.right_ik.arm_angle_requested_velocity_rad_s =
        diagnostics.right.arm_angle_requested_rate;
    snapshot.right_ik.arm_angle_radius_m =
        diagnostics.right.arm_angle.radius_m;
    snapshot.right_ik.arm_angle_reference_projection_norm =
        diagnostics.right.arm_angle.requested_reference_projection_norm;
    snapshot.right_ik.arm_angle_jacobian_norm =
        diagnostics.right.arm_angle.jacobian_norm;
    snapshot.right_ik.arm_angle_projection_held =
        diagnostics.right.arm_angle.reference_projection_held;
    snapshot.right_ik.arm_angle_reference_governor_held =
        diagnostics.right.arm_angle.reference_governor_held;
    snapshot.right_ik.arm_angle_branch_lock_active =
        diagnostics.right.arm_angle.branch_lock_active;
    snapshot.right_ik.arm_angle_branch_lock_distance_m =
        diagnostics.right.arm_angle.branch_lock_distance_m;
    snapshot.right_ik.arm_angle_branch_lock_constraint_active =
        diagnostics.right.arm_angle.branch_lock_constraint_active;
    snapshot.right_ik.arm_angle_branch_lock_requested_lower =
        diagnostics.right.arm_angle.branch_lock_requested_lower;
    snapshot.right_ik.arm_angle_branch_lock_effective_lower =
        diagnostics.right.arm_angle.branch_lock_effective_lower;
    snapshot.right_ik.arm_angle_branch_lock_feasibility_clipped =
        diagnostics.right.arm_angle.branch_lock_feasibility_clipped;
    snapshot.right_ik.elbow_world_z = diagnostics.right.arm_angle.elbow_world_z;
    snapshot.right_ik.shoulder_world_z =
        diagnostics.right.arm_angle.shoulder_world_z;
    snapshot.right_ik.upper_arm_outward_active =
        diagnostics.right.upper_arm_outward.constraint_active;
    snapshot.right_ik.upper_arm_outward_distance_m =
        diagnostics.right.upper_arm_outward.state.distance_m;
    snapshot.right_ik.upper_arm_outward_requested_lower =
        diagnostics.right.upper_arm_outward.requested_lower;
    snapshot.right_ik.upper_arm_outward_effective_lower =
        diagnostics.right.upper_arm_outward.effective_lower;
    snapshot.right_ik.upper_arm_outward_achieved =
        diagnostics.right.upper_arm_outward.achieved;
    snapshot.right_ik.upper_arm_outward_residual =
        diagnostics.right.upper_arm_outward.residual;
    snapshot.right_ik.upper_arm_outward_feasibility_clipped =
        diagnostics.right.upper_arm_outward.feasibility_clipped;
    snapshot.right_ik.dls_posture_reference_active =
        diagnostics.right.dls_posture_reference_active;
    snapshot.right_ik.dls_posture_status =
        static_cast<int>(diagnostics.right.dls_posture_status);
    snapshot.right_ik.dls_posture_iterations =
        diagnostics.right.dls_posture_iterations;
    snapshot.right_ik.dls_posture_joint_projection_count =
        diagnostics.right.dls_posture_joint_projection_count;
    snapshot.right_ik.dls_posture_ruckig_accepted =
        diagnostics.right.dls_posture_ruckig_accepted;
    snapshot.right_ik.dls_posture_solve_time_us =
        diagnostics.right.dls_posture_solve_time_us;
    snapshot.right_ik.ee_ik_wall_time_us = diagnostics.right.ee_ik_wall_time_us;
    snapshot.right_ik.ee_ruckig_wall_time_us = diagnostics.right.ee_ruckig_wall_time_us;
    snapshot.right_ik.ee_ik_to_ruckig_wall_time_us = diagnostics.right.ee_ik_to_ruckig_wall_time_us;
    snapshot.right_ik.ee_ruckig_invoked = diagnostics.right.ee_ruckig_invoked;
    snapshot.right_ik.ee_pinocchio_kinematics = diagnostics.right.ee_pinocchio_kinematics;
    snapshot.right_ik.dls_posture_initial_position_error_m =
        diagnostics.right.dls_posture_initial_position_error_m;
    snapshot.right_ik.dls_posture_initial_orientation_error_rad =
        diagnostics.right.dls_posture_initial_orientation_error_rad;
    snapshot.right_ik.dls_posture_final_position_error_m =
        diagnostics.right.dls_posture_final_position_error_m;
    snapshot.right_ik.dls_posture_final_orientation_error_rad =
        diagnostics.right.dls_posture_final_orientation_error_rad;
    snapshot.right_ik.dls_posture_goal_error_max_abs =
        (diagnostics.right.dls_posture_goal - diagnostics.right.q_ref)
            .cwiseAbs().maxCoeff();
    snapshot.right_ik.dls_posture_reference_error_max_abs =
        (diagnostics.right.dls_posture_reference - diagnostics.right.q_ref)
            .cwiseAbs().maxCoeff();
    snapshot.right_ik.dls_posture_velocity_target_max_abs =
        diagnostics.right.dls_posture_velocity_target.cwiseAbs().maxCoeff();
    snapshot.right_ik.dls_posture_qdot_error_max_abs =
        (diagnostics.right.dls_posture_velocity_target - diagnostics.right.ik.qdot)
            .cwiseAbs().maxCoeff();
    snapshot.right_ik.dls_posture_goal_limit_margin_rad =
        diagnostics.right.dls_posture_goal_limit_margin_rad;
    snapshot.right_ik.spark_posture_active =
        spark_mode && !spark_direct_qpos &&
        spark_diagnostics.posture_tasks.right.active;
    snapshot.right_ik.spark_ik_accepted = spark_diagnostics.right.ik.accepted;
    snapshot.right_ik.spark_stage1_iterations =
        spark_diagnostics.right.ik.stage1_iterations;
    snapshot.right_ik.spark_stage2_iterations =
        spark_diagnostics.right.ik.stage2_iterations;
    snapshot.right_ik.spark_solve_time_us =
        spark_diagnostics.right.ik.stage1_solve_time_us +
        spark_diagnostics.right.ik.stage2_solve_time_us;
    snapshot.right_ik.spark_palm_position_error_m =
        spark_diagnostics.right.ik.palm_position_error;
    snapshot.right_ik.spark_palm_orientation_error_rad =
        spark_diagnostics.right.ik.palm_orientation_error;
    snapshot.right_ik.spark_reference_velocity_ratio =
        spark_diagnostics.right.reference.velocity_ratio;
    snapshot.right_ik.spark_reference_acceleration_ratio =
        spark_diagnostics.right.reference.acceleration_ratio;
    snapshot.right_ik.spark_reference_jerk_ratio =
        spark_diagnostics.right.reference.jerk_ratio;
    snapshot.right_ik.spark_q_ik_error_max_abs =
        spark_mode && spark_diagnostics.right.ik.accepted
            ? (spark_diagnostics.right.q_ik - diagnostics.right.q_ref)
                  .cwiseAbs().maxCoeff()
            : 0.0;
    snapshot.right_ik.spark_q_ref_error_max_abs =
        spark_mode && spark_diagnostics.right.reference.accepted
            ? (spark_diagnostics.right.reference.state.q - diagnostics.right.q_ref)
                  .cwiseAbs().maxCoeff()
            : 0.0;
    snapshot.right_ik.spark_posture_velocity_max_abs =
        spark_mode && !spark_direct_qpos
            ? spark_diagnostics.posture_tasks.right.target.cwiseAbs().maxCoeff()
            : 0.0;
    const auto fill_feedforward_snapshot = [](
                                               const auto& arm_diagnostics,
                                               const Vec6& cartesian_twist,
                                               ArmIkSnapshot& arm_snapshot) {
      arm_snapshot.spark_feedforward_valid =
          arm_diagnostics.feedforward.valid;
      arm_snapshot.spark_feedforward_state =
          static_cast<int>(arm_diagnostics.feedforward.state);
      arm_snapshot.spark_feedforward_dt_valid =
          arm_diagnostics.feedforward_target.dt_valid;
      arm_snapshot.spark_feedforward_jump_rejected =
          arm_diagnostics.feedforward_target.jump_rejected;
      arm_snapshot.spark_feedforward_epoch_reset =
          arm_diagnostics.feedforward_target.epoch_reset;
      arm_snapshot.spark_feedforward_source_dt_seconds =
          arm_diagnostics.feedforward_target.source_dt_seconds;
      arm_snapshot.spark_feedforward_median_dt_seconds =
          arm_diagnostics.feedforward_target.median_dt_seconds;
      arm_snapshot.spark_feedforward_activation =
          arm_diagnostics.feedforward.activation;
      arm_snapshot.spark_feedforward_linear_velocity =
          cartesian_twist.head<3>().norm();
      arm_snapshot.spark_feedforward_angular_velocity =
          cartesian_twist.tail<3>().norm();
      arm_snapshot.spark_motion_intent_linear_velocity =
          arm_diagnostics.motion_intent_twist.twist.template head<3>().norm();
      arm_snapshot.spark_motion_intent_angular_velocity =
          arm_diagnostics.motion_intent_twist.twist.template tail<3>().norm();
      arm_snapshot.spark_stationary_joint_reference_held =
          arm_diagnostics.stationary_joint_reference_held;
      arm_snapshot.spark_settled_hold_active =
          arm_diagnostics.settled_hold_active;
      arm_snapshot.spark_settled_hold_dwell_seconds =
          arm_diagnostics.settled_hold_dwell_seconds;
      arm_snapshot.spark_settled_hold_reason =
          static_cast<int>(arm_diagnostics.settled_hold_reason);
      arm_snapshot.spark_feedforward_q_ik = arm_diagnostics.q_ik;
      arm_snapshot.spark_feedforward_q = arm_diagnostics.feedforward.q;
      arm_snapshot.spark_feedforward_qdot = arm_diagnostics.feedforward.qdot;
      arm_snapshot.spark_feedforward_qddot = arm_diagnostics.feedforward.qddot;
      arm_snapshot.spark_feedforward_jerk = arm_diagnostics.feedforward.jerk;
      arm_snapshot.headroom_valid = arm_diagnostics.headroom.valid;
      arm_snapshot.headroom_derivative_history_valid =
          arm_diagnostics.headroom.derivative_history_valid;
      arm_snapshot.headroom_velocity =
          arm_diagnostics.headroom.velocity_headroom;
      arm_snapshot.headroom_acceleration =
          arm_diagnostics.headroom.acceleration_headroom;
      arm_snapshot.headroom_jerk = arm_diagnostics.headroom.jerk_headroom;
      arm_snapshot.headroom_task = arm_diagnostics.headroom.task_headroom;
      arm_snapshot.headroom_raw = arm_diagnostics.headroom.raw_headroom;
      arm_snapshot.headroom_filtered =
          arm_diagnostics.headroom.filtered_headroom;
      arm_snapshot.headroom_scale = arm_diagnostics.headroom.scale;
      arm_snapshot.headroom_state =
          static_cast<int>(arm_diagnostics.headroom.state);
      arm_snapshot.headroom_dominant_source =
          static_cast<int>(arm_diagnostics.headroom.dominant_source);
    };
    if (spark_internal_feedforward) {
      fill_feedforward_snapshot(
          spark_diagnostics.left, spark_diagnostics.cartesian_targets.left_twist,
          snapshot.left_ik);
      fill_feedforward_snapshot(
          spark_diagnostics.right,
          spark_diagnostics.cartesian_targets.right_twist, snapshot.right_ik);
    }
    if (config.control_level == ControlLevel::kAcceleration && !paused) {
      snapshot.left_position_error =
          acceleration_diagnostics.left.pose_error.head<3>().norm();
      snapshot.left_orientation_error =
          acceleration_diagnostics.left.pose_error.tail<3>().norm();
      snapshot.right_position_error =
          acceleration_diagnostics.right.pose_error.head<3>().norm();
      snapshot.right_orientation_error =
          acceleration_diagnostics.right.pose_error.tail<3>().norm();
      snapshot.left_solver_status = acceleration_diagnostics.left.qp.status;
      snapshot.right_solver_status = acceleration_diagnostics.right.qp.status;
      snapshot.left_ik.accepted = acceleration_diagnostics.left.accepted;
      snapshot.left_ik.fallback_applied =
          acceleration_diagnostics.left.fallback_applied;
      snapshot.left_ik.hold_reason = acceleration_diagnostics.left.hold_reason;
      snapshot.left_ik.position_error = snapshot.left_position_error;
      snapshot.left_ik.orientation_error = snapshot.left_orientation_error;
      snapshot.left_ik.actual_position_error =
          acceleration_diagnostics.left.actual_pose_error.head<3>().norm();
      snapshot.left_ik.actual_orientation_error =
          acceleration_diagnostics.left.actual_pose_error.tail<3>().norm();
      snapshot.left_ik.slack_position_norm =
          acceleration_diagnostics.left.qp.slack.head<3>().norm();
      snapshot.left_ik.slack_orientation_norm =
          acceleration_diagnostics.left.qp.slack.tail<3>().norm();
      snapshot.left_ik.equality_residual =
          acceleration_diagnostics.left.qp.equality_residual;
      snapshot.left_ik.reference_error_max_abs =
          acceleration_diagnostics.left.position_reference_error;
      snapshot.left_ik.qdot_reference_error_max_abs =
          acceleration_diagnostics.left.velocity_reference_error;
      snapshot.left_ik.reference_scale =
          acceleration_diagnostics.left.reference_scale;
      snapshot.left_ik.reference_frozen =
          acceleration_diagnostics.left.reference_frozen;
      snapshot.left_ik.qddot_max_ratio =
          acceleration_diagnostics.left.qp.qddot_max_ratio;
      snapshot.left_ik.active_qddot_bounds =
          acceleration_diagnostics.left.qp.active_bound_count;
      snapshot.left_ik.task_scale_position =
          acceleration_diagnostics.left.qp.task_scale_position;
      snapshot.left_ik.task_scale_orientation =
          acceleration_diagnostics.left.qp.task_scale_orientation;
      snapshot.left_ik.solve_time_us =
          acceleration_diagnostics.left.qp.solve_time_us;
      snapshot.left_ik.iterations = acceleration_diagnostics.left.qp.iterations;
      snapshot.left_ik.status = acceleration_diagnostics.left.qp.status;
      snapshot.left_ik.otg_valid =
          acceleration_diagnostics.left.reference.valid;
      snapshot.left_ik.otg_stale =
          acceleration_diagnostics.left.reference.stale;
      snapshot.left_ik.reference_linear_velocity =
          acceleration_diagnostics.left.reference.twist.head<3>().norm();
      snapshot.left_ik.reference_angular_velocity =
          acceleration_diagnostics.left.reference.twist.tail<3>().norm();
      snapshot.left_ik.reference_linear_acceleration =
          acceleration_diagnostics.left.reference.acceleration.head<3>().norm();
      snapshot.left_ik.reference_angular_acceleration =
          acceleration_diagnostics.left.reference.acceleration.tail<3>().norm();
      snapshot.left_ik.arm_angle_reference_source =
          acceleration_diagnostics.left.arm_angle.reference_source;
      snapshot.left_ik.arm_angle_active =
          acceleration_diagnostics.left.arm_angle_task_active;
      snapshot.left_ik.arm_angle_error_rad =
          acceleration_diagnostics.left.arm_angle.error_rad;
      snapshot.left_ik.arm_angle_robot_rad =
          acceleration_diagnostics.left.arm_angle.robot_angle_rad;
      snapshot.left_ik.arm_angle_target_rad =
          acceleration_diagnostics.left.arm_angle.target_angle_rad;
      snapshot.left_ik.arm_angle_current_rate_rad_s =
          acceleration_diagnostics.left.arm_angle_current_rate;
      snapshot.left_ik.arm_angle_requested_acceleration_rad_s2 =
          acceleration_diagnostics.left.arm_angle_requested_acceleration;
      snapshot.left_ik.arm_angle_radius_m =
          acceleration_diagnostics.left.arm_angle.radius_m;
      snapshot.left_ik.arm_angle_reference_projection_norm =
          acceleration_diagnostics.left.arm_angle
              .requested_reference_projection_norm;
      snapshot.left_ik.arm_angle_jacobian_norm =
          acceleration_diagnostics.left.arm_angle.jacobian_norm;
      snapshot.left_ik.arm_angle_projection_held =
          acceleration_diagnostics.left.arm_angle.reference_projection_held;
      snapshot.left_ik.arm_angle_reference_governor_held =
          acceleration_diagnostics.left.arm_angle.reference_governor_held;
      snapshot.left_ik.arm_angle_branch_lock_active =
          acceleration_diagnostics.left.arm_angle.branch_lock_active;
      snapshot.left_ik.arm_angle_branch_lock_distance_m =
          acceleration_diagnostics.left.arm_angle.branch_lock_distance_m;
      snapshot.left_ik.arm_angle_branch_lock_constraint_active =
          acceleration_diagnostics.left.arm_angle.branch_lock_constraint_active;
      snapshot.left_ik.arm_angle_branch_lock_requested_lower =
          acceleration_diagnostics.left.arm_angle.branch_lock_requested_lower;
      snapshot.left_ik.arm_angle_branch_lock_effective_lower =
          acceleration_diagnostics.left.arm_angle.branch_lock_effective_lower;
      snapshot.left_ik.arm_angle_branch_lock_feasibility_clipped =
          acceleration_diagnostics.left.arm_angle.branch_lock_feasibility_clipped;
      snapshot.left_ik.arm_angle_achieved_acceleration_rad_s2 =
          acceleration_diagnostics.left.arm_angle_achieved_acceleration;
      snapshot.left_ik.arm_angle_acceleration_residual_rad_s2 =
          acceleration_diagnostics.left.arm_angle_acceleration_residual;
      snapshot.left_ik.elbow_world_z =
          acceleration_diagnostics.left.arm_angle.elbow_world_z;
      snapshot.left_ik.shoulder_world_z =
          acceleration_diagnostics.left.arm_angle.shoulder_world_z;
      snapshot.left_ik.upper_arm_outward_active =
          acceleration_diagnostics.left.upper_arm_outward.constraint_active;
      snapshot.left_ik.upper_arm_outward_distance_m =
          acceleration_diagnostics.left.upper_arm_outward.state.distance_m;
      snapshot.left_ik.upper_arm_outward_requested_lower =
          acceleration_diagnostics.left.upper_arm_outward.requested_lower;
      snapshot.left_ik.upper_arm_outward_effective_lower =
          acceleration_diagnostics.left.upper_arm_outward.effective_lower;
      snapshot.left_ik.upper_arm_outward_achieved =
          acceleration_diagnostics.left.upper_arm_outward.achieved;
      snapshot.left_ik.upper_arm_outward_residual =
          acceleration_diagnostics.left.upper_arm_outward.residual;
      snapshot.left_ik.upper_arm_outward_feasibility_clipped =
          acceleration_diagnostics.left.upper_arm_outward.feasibility_clipped;
      snapshot.right_ik.accepted = acceleration_diagnostics.right.accepted;
      snapshot.right_ik.fallback_applied =
          acceleration_diagnostics.right.fallback_applied;
      snapshot.right_ik.hold_reason = acceleration_diagnostics.right.hold_reason;
      snapshot.right_ik.position_error = snapshot.right_position_error;
      snapshot.right_ik.orientation_error = snapshot.right_orientation_error;
      snapshot.right_ik.actual_position_error =
          acceleration_diagnostics.right.actual_pose_error.head<3>().norm();
      snapshot.right_ik.actual_orientation_error =
          acceleration_diagnostics.right.actual_pose_error.tail<3>().norm();
      snapshot.right_ik.slack_position_norm =
          acceleration_diagnostics.right.qp.slack.head<3>().norm();
      snapshot.right_ik.slack_orientation_norm =
          acceleration_diagnostics.right.qp.slack.tail<3>().norm();
      snapshot.right_ik.equality_residual =
          acceleration_diagnostics.right.qp.equality_residual;
      snapshot.right_ik.reference_error_max_abs =
          acceleration_diagnostics.right.position_reference_error;
      snapshot.right_ik.qdot_reference_error_max_abs =
          acceleration_diagnostics.right.velocity_reference_error;
      snapshot.right_ik.reference_scale =
          acceleration_diagnostics.right.reference_scale;
      snapshot.right_ik.reference_frozen =
          acceleration_diagnostics.right.reference_frozen;
      snapshot.right_ik.qddot_max_ratio =
          acceleration_diagnostics.right.qp.qddot_max_ratio;
      snapshot.right_ik.active_qddot_bounds =
          acceleration_diagnostics.right.qp.active_bound_count;
      snapshot.right_ik.task_scale_position =
          acceleration_diagnostics.right.qp.task_scale_position;
      snapshot.right_ik.task_scale_orientation =
          acceleration_diagnostics.right.qp.task_scale_orientation;
      snapshot.right_ik.solve_time_us =
          acceleration_diagnostics.right.qp.solve_time_us;
      snapshot.right_ik.iterations = acceleration_diagnostics.right.qp.iterations;
      snapshot.right_ik.status = acceleration_diagnostics.right.qp.status;
      snapshot.right_ik.otg_valid =
          acceleration_diagnostics.right.reference.valid;
      snapshot.right_ik.otg_stale =
          acceleration_diagnostics.right.reference.stale;
      snapshot.right_ik.reference_linear_velocity =
          acceleration_diagnostics.right.reference.twist.head<3>().norm();
      snapshot.right_ik.reference_angular_velocity =
          acceleration_diagnostics.right.reference.twist.tail<3>().norm();
      snapshot.right_ik.reference_linear_acceleration =
          acceleration_diagnostics.right.reference.acceleration.head<3>().norm();
      snapshot.right_ik.reference_angular_acceleration =
          acceleration_diagnostics.right.reference.acceleration.tail<3>().norm();
      snapshot.right_ik.arm_angle_reference_source =
          acceleration_diagnostics.right.arm_angle.reference_source;
      snapshot.right_ik.arm_angle_active =
          acceleration_diagnostics.right.arm_angle_task_active;
      snapshot.right_ik.arm_angle_error_rad =
          acceleration_diagnostics.right.arm_angle.error_rad;
      snapshot.right_ik.arm_angle_robot_rad =
          acceleration_diagnostics.right.arm_angle.robot_angle_rad;
      snapshot.right_ik.arm_angle_target_rad =
          acceleration_diagnostics.right.arm_angle.target_angle_rad;
      snapshot.right_ik.arm_angle_current_rate_rad_s =
          acceleration_diagnostics.right.arm_angle_current_rate;
      snapshot.right_ik.arm_angle_requested_acceleration_rad_s2 =
          acceleration_diagnostics.right.arm_angle_requested_acceleration;
      snapshot.right_ik.arm_angle_radius_m =
          acceleration_diagnostics.right.arm_angle.radius_m;
      snapshot.right_ik.arm_angle_reference_projection_norm =
          acceleration_diagnostics.right.arm_angle
              .requested_reference_projection_norm;
      snapshot.right_ik.arm_angle_jacobian_norm =
          acceleration_diagnostics.right.arm_angle.jacobian_norm;
      snapshot.right_ik.arm_angle_projection_held =
          acceleration_diagnostics.right.arm_angle.reference_projection_held;
      snapshot.right_ik.arm_angle_reference_governor_held =
          acceleration_diagnostics.right.arm_angle.reference_governor_held;
      snapshot.right_ik.arm_angle_branch_lock_active =
          acceleration_diagnostics.right.arm_angle.branch_lock_active;
      snapshot.right_ik.arm_angle_branch_lock_distance_m =
          acceleration_diagnostics.right.arm_angle.branch_lock_distance_m;
      snapshot.right_ik.arm_angle_branch_lock_constraint_active =
          acceleration_diagnostics.right.arm_angle.branch_lock_constraint_active;
      snapshot.right_ik.arm_angle_branch_lock_requested_lower =
          acceleration_diagnostics.right.arm_angle.branch_lock_requested_lower;
      snapshot.right_ik.arm_angle_branch_lock_effective_lower =
          acceleration_diagnostics.right.arm_angle.branch_lock_effective_lower;
      snapshot.right_ik.arm_angle_branch_lock_feasibility_clipped =
          acceleration_diagnostics.right.arm_angle.branch_lock_feasibility_clipped;
      snapshot.right_ik.arm_angle_achieved_acceleration_rad_s2 =
          acceleration_diagnostics.right.arm_angle_achieved_acceleration;
      snapshot.right_ik.arm_angle_acceleration_residual_rad_s2 =
          acceleration_diagnostics.right.arm_angle_acceleration_residual;
      snapshot.right_ik.elbow_world_z =
          acceleration_diagnostics.right.arm_angle.elbow_world_z;
      snapshot.right_ik.shoulder_world_z =
          acceleration_diagnostics.right.arm_angle.shoulder_world_z;
      snapshot.right_ik.upper_arm_outward_active =
          acceleration_diagnostics.right.upper_arm_outward.constraint_active;
      snapshot.right_ik.upper_arm_outward_distance_m =
          acceleration_diagnostics.right.upper_arm_outward.state.distance_m;
      snapshot.right_ik.upper_arm_outward_requested_lower =
          acceleration_diagnostics.right.upper_arm_outward.requested_lower;
      snapshot.right_ik.upper_arm_outward_effective_lower =
          acceleration_diagnostics.right.upper_arm_outward.effective_lower;
      snapshot.right_ik.upper_arm_outward_achieved =
          acceleration_diagnostics.right.upper_arm_outward.achieved;
      snapshot.right_ik.upper_arm_outward_residual =
          acceleration_diagnostics.right.upper_arm_outward.residual;
      snapshot.right_ik.upper_arm_outward_feasibility_clipped =
          acceleration_diagnostics.right.upper_arm_outward.feasibility_clipped;
    }

    JointKinematicsSample plot_sample;
    plot_sample.sequence = sequence;
    plot_sample.time_seconds = control_time;
    plot_sample.reset = plot_reset_requested;
    setConfiguredPlotBounds(ArmSide::kLeft, robot, config,
                            plot_sample.left.bounds);
    setConfiguredPlotBounds(ArmSide::kRight, robot, config,
                            plot_sample.right.bounds);

    const ArmMotionState left_reference_state =
        spark_direct_qpos
            ? direct_left_state
            : config.control_level == ControlLevel::kAcceleration
            ? acceleration_controller->referenceState(ArmSide::kLeft)
            : controller->referenceState(ArmSide::kLeft);
    const ArmMotionState right_reference_state =
        spark_direct_qpos
            ? direct_right_state
            : config.control_level == ControlLevel::kAcceleration
            ? acceleration_controller->referenceState(ArmSide::kRight)
            : controller->referenceState(ArmSide::kRight);
    plot_sample.left.reference.position = left_reference_state.q;
    plot_sample.left.reference.velocity = left_reference_state.qdot;
    plot_sample.right.reference.position = right_reference_state.q;
    plot_sample.right.reference.velocity = right_reference_state.qdot;
    plot_sample.left.actual.position = robot.armPosition(ArmSide::kLeft);
    plot_sample.left.actual.velocity = robot.armVelocity(ArmSide::kLeft);
    plot_sample.right.actual.position = robot.armPosition(ArmSide::kRight);
    plot_sample.right.actual.velocity = robot.armVelocity(ArmSide::kRight);

    const bool acceleration_level =
        config.control_level == ControlLevel::kAcceleration;
    const bool ruckig_plot = usesSharedRootDirectIk(controller->algorithm());
    const bool ruckig_step_valid = ruckig_plot &&
        diagnostics.left.dls_posture_ruckig_accepted &&
        diagnostics.right.dls_posture_ruckig_accepted;
    plot_sample.left.ruckig_output = plot_sample.right.ruckig_output = ruckig_plot;
    const bool left_output_valid =
        !paused && (ruckig_plot ? (recovery_plot_valid || ruckig_step_valid) : acceleration_level
                        ? acceleration_diagnostics.left.accepted
                        : diagnostics.left.accepted);
    const bool right_output_valid =
        !paused && (ruckig_plot ? (recovery_plot_valid || ruckig_step_valid) : acceleration_level
                        ? acceleration_diagnostics.right.accepted
                        : diagnostics.right.accepted);
    const ReferenceAccelerationSource acceleration_source =
        ruckig_plot ? ReferenceAccelerationSource::kRuckigOutput : acceleration_level
            ? ReferenceAccelerationSource::kDirectQpOutput
            : ReferenceAccelerationSource::kDifferentiateVelocity;
    if (acceleration_level) {
      if (left_output_valid) {
        plot_sample.left.bounds.acceleration_lower =
            acceleration_diagnostics.left.bounds.lower;
        plot_sample.left.bounds.acceleration_upper =
            acceleration_diagnostics.left.bounds.upper;
      }
      if (right_output_valid) {
        plot_sample.right.bounds.acceleration_lower =
            acceleration_diagnostics.right.bounds.lower;
        plot_sample.right.bounds.acceleration_upper =
            acceleration_diagnostics.right.bounds.upper;
      }
    } else if (!ruckig_plot) {
      if (left_output_valid) {
        plot_sample.left.bounds.velocity_lower = diagnostics.left.bounds.lower;
        plot_sample.left.bounds.velocity_upper = diagnostics.left.bounds.upper;
      }
      if (right_output_valid) {
        plot_sample.right.bounds.velocity_lower =
            diagnostics.right.bounds.lower;
        plot_sample.right.bounds.velocity_upper =
            diagnostics.right.bounds.upper;
      }
    }

    if (ruckig_plot) {
      const auto& normal_smoothing = controller->algorithm() == IkAlgorithm::kPicoEeFrankaDls
          ? config.pico_ee_franka_dls.post_smoothing : config.pico_ee_franka_ceres_lm.post_smoothing;
      for (auto* arm : {&plot_sample.left, &plot_sample.right}) {
        const ArmSide side = arm == &plot_sample.left ? ArmSide::kLeft : ArmSide::kRight;
        const auto smoothing = recovery && !recovery->teleop()
            ? normal_smoothing : controller->trajectorySampleLimits(side);
        Vec7 velocity = (smoothing.velocity_scale * robot.mapping(side).limits.velocity)
                            .cwiseMin(smoothing.max_velocity_rad_s);
        Vec7 acceleration = smoothing.max_acceleration_rad_s2;
        Vec7 jerk = smoothing.max_jerk_rad_s3;
        if (recovery_home_sample) {
          velocity = velocity.cwiseMin(Vec7::Constant(SimulationRecovery::kHomeVelocity));
          acceleration = acceleration.cwiseMin(Vec7::Constant(SimulationRecovery::kHomeAcceleration));
          jerk = jerk.cwiseMin(Vec7::Constant(SimulationRecovery::kHomeJerk));
        }
        arm->bounds.velocity_lower = -velocity; arm->bounds.velocity_upper = velocity;
        arm->bounds.acceleration_lower = -acceleration; arm->bounds.acceleration_upper = acceleration;
        arm->bounds.jerk_lower = -jerk; arm->bounds.jerk_upper = jerk;
      }
    }

    const JointKinematicsDerivatives left_plot_derivatives =
        left_plot_differentiator.update(
            left_reference_state.qdot, left_reference_state.qddot,
            acceleration_source, plot_sample.left.actual.velocity, dt,
            plot_reset_requested || !left_output_valid);
    const JointKinematicsDerivatives right_plot_derivatives =
        right_plot_differentiator.update(
            right_reference_state.qdot, right_reference_state.qddot,
            acceleration_source, plot_sample.right.actual.velocity, dt,
            plot_reset_requested || !right_output_valid);
    setPlotDerivatives(plot_sample.left, left_plot_derivatives);
    setPlotDerivatives(plot_sample.right, right_plot_derivatives);
    if (!left_output_valid) {
      plot_sample.left.reference_acceleration_valid = false;
      plot_sample.left.reference_jerk_valid = false;
      plot_sample.left.actual_acceleration_valid = false;
      plot_sample.left.actual_jerk_valid = false;
    }
    if (!right_output_valid) {
      plot_sample.right.reference_acceleration_valid = false;
      plot_sample.right.reference_jerk_valid = false;
      plot_sample.right.actual_acceleration_valid = false;
      plot_sample.right.actual_jerk_valid = false;
    }
    plot_sample.reset = plot_sample.reset || !left_output_valid ||
                        !right_output_valid;
    if (joint_plot_queue.tryPush(plot_sample)) {
      ++joint_plot_samples;
    } else {
      ++joint_plot_drops;
    }
    if (joint_telemetry != nullptr) {
      joint_telemetry->tryPush(plot_sample);
    }
    snapshot.joint_plot_samples = joint_plot_samples;
    snapshot.joint_plot_drops = joint_plot_drops;
    snapshot.joint_plot_reference_derivatives_valid =
        plot_sample.left.reference_jerk_valid &&
        plot_sample.right.reference_jerk_valid;
    snapshot.joint_plot_actual_derivatives_valid =
        plot_sample.left.actual_jerk_valid &&
        plot_sample.right.actual_jerk_valid;

    snapshot.cycle_time_us =
        std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - cycle_start)
            .count();
    cycle_times.add(snapshot.cycle_time_us);
    if ((sequence % 100U) == 0U) {
      cycle_p99_us = cycle_times.percentile99();
    }
    snapshot.cycle_p99_us = cycle_p99_us;

    timespec now{};
    clock_gettime(CLOCK_MONOTONIC, &now);
    if (laterThan(now, deadline)) {
      ++deadline_misses;
    }
    snapshot.deadline_misses = deadline_misses;
    snapshot.control_failures = control_failures;
    snapshot.snapshot_drops = snapshot_drops;
    snapshot.telemetry_drops = telemetry_drops;
    snapshot.pico_configured = pico_configured;
    snapshot.pico_enabled = pico_session.enabled();
    snapshot.pico_live = pico_freshness.live;
    snapshot.pico_stale = pico_freshness.stale;
    snapshot.arm_angle_reference_mode = arm_angle_reference_mode;
    snapshot.pico_tracking_epoch = pico_applied_epoch;
    snapshot.pico_sequence = pico_applied_sequence;
    snapshot.pico_datagrams = pico_stats.datagrams;
    snapshot.pico_accepted = pico_stats.accepted;
    snapshot.pico_malformed = pico_stats.malformed;
    snapshot.pico_crc_failures = pico_stats.crc_failures;
    snapshot.pico_reordered = pico_stats.reordered;
    snapshot.pico_jump_rejections = pico_stats.jump_rejections;
    snapshot.pico_superseded = pico_stats.superseded;
    snapshot.pico_epoch_resets = pico_stats.epoch_resets;
    snapshot.pico_resynchronizations = pico_stats.resynchronizations;
    snapshot.pico_reset_applies = pico_reset_applies;
    snapshot.pico_left_source_timestamp_ns = pico_left_source_timestamp_ns;
    snapshot.pico_right_source_timestamp_ns = pico_right_source_timestamp_ns;
    snapshot.pico_input_frequency_hz = pico_stats.input_frequency_hz;
    snapshot.pico_frame_age_ms = 1.0e3 * pico_freshness.frame_age_seconds;
    snapshot.pico_receive_to_control_us = pico_receive_to_control_us;
    snapshot.pico_bridge_to_control_us = pico_bridge_to_control_us;
    snapshot.pico_upper_limb_skeleton = latest_pico_upper_limb_skeleton;
    snapshot.pico_upper_limb_skeleton.valid =
        snapshot.pico_upper_limb_skeleton.valid && pico_session.enabled() &&
        pico_freshness.live;
    if (spark_mode) {
      snapshot.spark_upper_limb_skeleton =
          sparkUpperLimbSkeleton(spark_guidance->latestTargets());
      snapshot.spark_upper_limb_skeleton.valid =
          snapshot.spark_upper_limb_skeleton.valid && pico_session.enabled() &&
          pico_freshness.live;
    }

    if (joint_command_exporter != nullptr) {
      JointCommandFrame output;
      output.sequence = ++joint_command_sequence;
      output.source_timestamp_ns = monotonicNowNs();
      output.pico_tracking_epoch = joint_command_epoch;
      // Recheck freshness after the solve, not at tick start: a slow solve must
      // never make an already expired PICO/hand input look live to hardware.
      const bool arms_ready =
          !paused && !pico_paused && !plot_reset_requested &&
          pico_configured && pico_session.enabled() &&
          pico_session.freshness(output.source_timestamp_ns).live &&
          jointCommandPicoBridgeFresh(pico_applied_bridge_send_monotonic_ns,
                                      output.source_timestamp_ns) &&
          joint_command_epoch == pico_applied_epoch &&
          snapshot.accepted && snapshot.hold_reason == HoldReason::kNone &&
          left_output_valid && right_output_valid &&
          snapshot.left_ik.status == SolverStatus::kSolved &&
          snapshot.right_ik.status == SolverStatus::kSolved &&
          !desired.left_stale && !desired.right_stale &&
          (!spark_mode || spark_diagnostics.accepted);
      const bool hands_allowed =
          !paused && !pico_paused && !plot_reset_requested &&
          hand_configured && robot.hasHandMappings();
      const bool left_hand_ready =
          hands_allowed && left_hand_applied &&
          left_hand_freshness.live(output.source_timestamp_ns);
      const bool right_hand_ready =
          hands_allowed && right_hand_applied &&
          right_hand_freshness.live(output.source_timestamp_ns);
      output.flags = static_cast<std::uint8_t>(
          (joint_command_arm_readiness.update(
               arms_ready, plot_reset_requested || joint_command_stream_reset)
               ? kJointCommandArmsReadyFlag : 0U) |
          (left_hand_ready ? kJointCommandLeftHandReadyFlag : 0U) |
          (right_hand_ready ? kJointCommandRightHandReadyFlag : 0U));
      // These are the committed controller references, not pre-QP SPARK goals
      // or the rendering snapshot's simulated/actual-feedback arm positions.
      for (int index = 0; index < kArmDof; ++index) {
        output.position_rad[static_cast<std::size_t>(index)] = left_reference_state.q[index];
        output.position_rad[7U + static_cast<std::size_t>(index)] = right_reference_state.q[index];
      }
      if (robot.hasHandMappings()) {
        for (int index = 0; index < kHandDof; ++index) {
          output.position_rad[14U + static_cast<std::size_t>(index)] = snapshot.left_hand_q[index];
          output.position_rad[34U + static_cast<std::size_t>(index)] = snapshot.right_hand_q[index];
        }
      }
      joint_command_exporter->send(output);
    }

    if (telemetry != nullptr) {
      TelemetrySample sample;
      sample.simulation_phase = snapshot.simulation_phase;
      sample.sequence = snapshot.sequence;
      sample.control_time_seconds = snapshot.control_time_seconds;
      sample.algorithm = snapshot.algorithm;
      sample.control_level = snapshot.control_level;
      sample.backend = snapshot.backend;
      sample.mode = snapshot.mode;
      sample.arm_angle_reference_mode = snapshot.arm_angle_reference_mode;
      sample.paused = snapshot.paused;
      sample.accepted = snapshot.accepted;
      sample.hold_reason = snapshot.hold_reason;
      sample.left_target_stale = snapshot.left_target_stale;
      sample.right_target_stale = snapshot.right_target_stale;
      sample.otg_enabled = snapshot.otg_enabled;
      sample.left_position_error = snapshot.left_position_error;
      sample.left_orientation_error = snapshot.left_orientation_error;
      sample.right_position_error = snapshot.right_position_error;
      sample.right_orientation_error = snapshot.right_orientation_error;
      sample.left_ik = snapshot.left_ik;
      sample.right_ik = snapshot.right_ik;
      sample.cycle_time_us = snapshot.cycle_time_us;
      sample.cycle_p99_us = snapshot.cycle_p99_us;
      sample.deadline_misses = snapshot.deadline_misses;
      sample.control_failures = snapshot.control_failures;
      sample.pico_configured = snapshot.pico_configured;
      sample.pico_enabled = snapshot.pico_enabled;
      sample.pico_live = snapshot.pico_live;
      sample.pico_stale = snapshot.pico_stale;
      sample.pico_tracking_epoch = snapshot.pico_tracking_epoch;
      sample.pico_sequence = snapshot.pico_sequence;
      sample.pico_datagrams = snapshot.pico_datagrams;
      sample.pico_accepted = snapshot.pico_accepted;
      sample.pico_malformed = snapshot.pico_malformed;
      sample.pico_crc_failures = snapshot.pico_crc_failures;
      sample.pico_reordered = snapshot.pico_reordered;
      sample.pico_jump_rejections = snapshot.pico_jump_rejections;
      sample.pico_superseded = snapshot.pico_superseded;
      sample.pico_epoch_resets = snapshot.pico_epoch_resets;
      sample.pico_resynchronizations = snapshot.pico_resynchronizations;
      sample.pico_reset_applies = snapshot.pico_reset_applies;
      sample.pico_left_source_timestamp_ns =
          snapshot.pico_left_source_timestamp_ns;
      sample.pico_right_source_timestamp_ns =
          snapshot.pico_right_source_timestamp_ns;
      sample.pico_input_frequency_hz = snapshot.pico_input_frequency_hz;
      sample.pico_frame_age_ms = snapshot.pico_frame_age_ms;
      sample.pico_receive_to_control_us = snapshot.pico_receive_to_control_us;
      sample.pico_bridge_to_control_us = snapshot.pico_bridge_to_control_us;
      sample.left_target_pose = desired.left;
      sample.right_target_pose = desired.right;
      sample.left_reference_pose = config.cartesian_otg.enabled
                                       ? references.left.pose
                                       : desired.left;
      sample.right_reference_pose = config.cartesian_otg.enabled
                                        ? references.right.pose
                                        : desired.right;
      sample.left_actual_pose = robot.tcpPose(ArmSide::kLeft);
      sample.right_actual_pose = robot.tcpPose(ArmSide::kRight);
      if (!telemetry->tryPush(sample)) {
        ++telemetry_drops;
      }
    }
    if (!snapshots.tryPublish(snapshot)) {
      ++snapshot_drops;
    }

    sleepUntil(deadline);
    deadline = addNanoseconds(deadline, period_nanoseconds);
    ++sequence;
  }
}

std::string solverStatusName(SolverStatus status);

void writeTelemetryCsv(const std::string& path, TelemetryBuffer& telemetry,
                       const std::atomic<bool>& control_finished) {
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("cannot open telemetry output: " + path);
  }
  output << "sequence,control_time_seconds,algorithm,control_level,mode,arm_angle_mode,paused,accepted,hold_reason,"
            "left_target_stale,right_target_stale,otg_enabled,"
            "left_position_error_m,left_orientation_error_rad,left_slack_position_norm,"
            "left_slack_orientation_norm,left_equality_residual,"
            "left_accepted,left_fallback_applied,left_hold_reason,left_reference_error_max_abs,"
            "left_reference_scale,left_reference_frozen,left_qdot_max_ratio,left_solve_time_us,"
            "left_active_position_bounds,left_active_velocity_bounds,"
            "left_active_acceleration_bounds,left_active_braking_bounds,left_iterations,"
            "left_status,left_otg_valid,left_otg_stale,left_v_ref,left_w_ref,left_a_ref,left_alpha_ref,"
            "left_qdot_reference_error,left_qddot_max_ratio,left_active_qddot_bounds,"
            "left_task_scale_position,left_task_scale_orientation,"
            "right_position_error_m,right_orientation_error_rad,"
            "right_slack_position_norm,right_slack_orientation_norm,"
            "right_equality_residual,right_accepted,right_fallback_applied,right_hold_reason,"
            "right_reference_error_max_abs,right_reference_scale,right_reference_frozen,"
            "right_qdot_max_ratio,right_solve_time_us,right_active_position_bounds,"
            "right_active_velocity_bounds,right_active_acceleration_bounds,"
            "right_active_braking_bounds,right_iterations,right_status,"
            "right_otg_valid,right_otg_stale,right_v_ref,right_w_ref,right_a_ref,right_alpha_ref,"
            "right_qdot_reference_error,right_qddot_max_ratio,right_active_qddot_bounds,"
            "right_task_scale_position,right_task_scale_orientation,"
            "cycle_time_us,"
            "cycle_p99_us,deadline_misses,control_failures,"
            "pico_configured,pico_enabled,pico_live,pico_stale,"
            "pico_tracking_epoch,pico_sequence,pico_datagrams,pico_accepted,"
            "pico_malformed,pico_crc_failures,pico_reordered,"
            "pico_jump_rejections,pico_superseded,pico_epoch_resets,"
            "pico_resynchronizations,pico_reset_applies,"
            "pico_left_source_timestamp_ns,pico_right_source_timestamp_ns,"
            "pico_input_frequency_hz,pico_frame_age_ms,"
            "pico_receive_to_control_us,pico_bridge_to_control_us,"
            "left_actual_position_error_m,left_actual_orientation_error_rad,"
            "right_actual_position_error_m,right_actual_orientation_error_rad,"
            "left_target_px,left_target_py,left_target_pz,left_target_qx,left_target_qy,left_target_qz,left_target_qw,"
            "left_reference_px,left_reference_py,left_reference_pz,left_reference_qx,left_reference_qy,left_reference_qz,left_reference_qw,"
            "left_actual_px,left_actual_py,left_actual_pz,left_actual_qx,left_actual_qy,left_actual_qz,left_actual_qw,"
            "right_target_px,right_target_py,right_target_pz,right_target_qx,right_target_qy,right_target_qz,right_target_qw,"
            "right_reference_px,right_reference_py,right_reference_pz,right_reference_qx,right_reference_qy,right_reference_qz,right_reference_qw,"
            "right_actual_px,right_actual_py,right_actual_pz,right_actual_qx,right_actual_qy,right_actual_qz,right_actual_qw,"
            "left_arm_angle_source,left_arm_angle_active,left_arm_angle_error_rad,"
            "left_arm_angle_robot_rad,left_arm_angle_target_rad,"
            "left_arm_angle_current_rate_rad_s,"
            "left_arm_angle_requested_velocity_rad_s,"
            "left_arm_angle_requested_acceleration_rad_s2,"
            "left_arm_angle_radius_m,left_arm_angle_reference_projection_norm,"
            "left_arm_angle_jacobian_norm,left_arm_angle_projection_held,"
            "left_arm_angle_reference_governor_held,"
            "left_arm_angle_branch_lock_active,left_arm_angle_branch_lock_distance_m,"
            "left_arm_angle_branch_lock_constraint_active,"
            "left_arm_angle_branch_lock_requested_lower,"
            "left_arm_angle_branch_lock_effective_lower,"
            "left_arm_angle_branch_lock_feasibility_clipped,"
            "left_arm_angle_achieved_acceleration_rad_s2,"
            "left_arm_angle_acceleration_residual_rad_s2,"
            "left_elbow_world_z,left_shoulder_world_z,"
            "left_upper_arm_outward_active,left_upper_arm_outward_distance_m,"
            "left_upper_arm_outward_requested_lower,left_upper_arm_outward_effective_lower,"
            "left_upper_arm_outward_achieved,left_upper_arm_outward_residual,"
            "left_upper_arm_outward_feasibility_clipped,"
            "right_arm_angle_source,right_arm_angle_active,right_arm_angle_error_rad,"
            "right_arm_angle_robot_rad,right_arm_angle_target_rad,"
            "right_arm_angle_current_rate_rad_s,"
            "right_arm_angle_requested_velocity_rad_s,"
            "right_arm_angle_requested_acceleration_rad_s2,"
            "right_arm_angle_radius_m,right_arm_angle_reference_projection_norm,"
            "right_arm_angle_jacobian_norm,right_arm_angle_projection_held,"
            "right_arm_angle_reference_governor_held,"
            "right_arm_angle_branch_lock_active,right_arm_angle_branch_lock_distance_m,"
            "right_arm_angle_branch_lock_constraint_active,"
            "right_arm_angle_branch_lock_requested_lower,"
            "right_arm_angle_branch_lock_effective_lower,"
            "right_arm_angle_branch_lock_feasibility_clipped,"
            "right_arm_angle_achieved_acceleration_rad_s2,"
            "right_arm_angle_acceleration_residual_rad_s2,"
            "right_elbow_world_z,right_shoulder_world_z,"
            "right_upper_arm_outward_active,right_upper_arm_outward_distance_m,"
            "right_upper_arm_outward_requested_lower,right_upper_arm_outward_effective_lower,"
            "right_upper_arm_outward_achieved,right_upper_arm_outward_residual,"
            "right_upper_arm_outward_feasibility_clipped,"
            "left_arm_angle_control_error_rad,left_dls_posture_reference_active,"
            "left_dls_posture_status,left_dls_posture_iterations,"
            "left_dls_posture_joint_projection_count,left_dls_posture_ruckig_accepted,"
            "left_dls_posture_solve_time_us,"
            "left_dls_posture_initial_position_error_m,"
            "left_dls_posture_initial_orientation_error_rad,"
            "left_dls_posture_final_position_error_m,"
            "left_dls_posture_final_orientation_error_rad,"
            "left_dls_posture_goal_error_max_abs,"
            "left_dls_posture_reference_error_max_abs,"
            "left_dls_posture_velocity_target_max_abs,"
            "left_dls_posture_qdot_error_max_abs,"
            "left_dls_posture_goal_limit_margin_rad,"
            "right_arm_angle_control_error_rad,right_dls_posture_reference_active,"
            "right_dls_posture_status,right_dls_posture_iterations,"
            "right_dls_posture_joint_projection_count,right_dls_posture_ruckig_accepted,"
            "right_dls_posture_solve_time_us,"
            "right_dls_posture_initial_position_error_m,"
            "right_dls_posture_initial_orientation_error_rad,"
            "right_dls_posture_final_position_error_m,"
            "right_dls_posture_final_orientation_error_rad,"
            "right_dls_posture_goal_error_max_abs,"
            "right_dls_posture_reference_error_max_abs,"
            "right_dls_posture_velocity_target_max_abs,"
            "right_dls_posture_qdot_error_max_abs,"
            "right_dls_posture_goal_limit_margin_rad,"
            "left_spark_posture_active,left_spark_ik_accepted,"
            "left_spark_stage1_iterations,left_spark_stage2_iterations,"
            "left_spark_solve_time_us,left_spark_palm_position_error_m,"
            "left_spark_palm_orientation_error_rad,"
            "left_spark_reference_velocity_ratio,"
            "left_spark_reference_acceleration_ratio,"
            "left_spark_reference_jerk_ratio,left_spark_q_ik_error_max_abs,"
            "left_spark_q_ref_error_max_abs,"
            "left_spark_posture_velocity_max_abs,"
            "right_spark_posture_active,right_spark_ik_accepted,"
            "right_spark_stage1_iterations,right_spark_stage2_iterations,"
            "right_spark_solve_time_us,right_spark_palm_position_error_m,"
            "right_spark_palm_orientation_error_rad,"
            "right_spark_reference_velocity_ratio,"
            "right_spark_reference_acceleration_ratio,"
            "right_spark_reference_jerk_ratio,right_spark_q_ik_error_max_abs,"
            "right_spark_q_ref_error_max_abs,"
            "right_spark_posture_velocity_max_abs";
  for (const std::string_view side : {"left", "right"}) {
    output << ',' << side << "_spark_feedforward_valid"
           << ',' << side << "_spark_feedforward_state"
           << ',' << side << "_spark_feedforward_dt_valid"
           << ',' << side << "_spark_feedforward_jump_rejected"
           << ',' << side << "_spark_feedforward_epoch_reset"
           << ',' << side << "_spark_feedforward_source_dt_seconds"
           << ',' << side << "_spark_feedforward_median_dt_seconds"
           << ',' << side << "_spark_feedforward_activation"
           << ',' << side << "_spark_feedforward_linear_velocity"
           << ',' << side << "_spark_feedforward_angular_velocity"
           << ',' << side << "_spark_motion_intent_linear_velocity"
           << ',' << side << "_spark_motion_intent_angular_velocity"
           << ',' << side << "_spark_stationary_joint_reference_held"
           << ',' << side << "_spark_settled_hold_active"
           << ',' << side << "_spark_settled_hold_dwell_seconds"
           << ',' << side << "_spark_settled_hold_reason";
    for (const std::string_view quantity : {"q_ik", "q", "qdot", "qddot",
                                            "jerk"}) {
      for (int joint = 1; joint <= kArmDof; ++joint) {
        output << ',' << side << "_spark_feedforward_" << quantity << "_j"
               << joint;
      }
    }
    output << ',' << side << "_headroom_valid"
           << ',' << side << "_headroom_derivative_history_valid"
           << ',' << side << "_headroom_velocity"
           << ',' << side << "_headroom_acceleration"
           << ',' << side << "_headroom_jerk"
           << ',' << side << "_headroom_task"
           << ',' << side << "_headroom_raw"
           << ',' << side << "_headroom_filtered"
           << ',' << side << "_headroom_scale"
           << ',' << side << "_headroom_state"
           << ',' << side << "_headroom_dominant_source";
  }
  // Append-only CSV extension; existing column positions remain unchanged.
  for (const char* side : {"left", "right"})
    output << ',' << side << "_ee_ik_wall_time_us"
           << ',' << side << "_ee_ruckig_wall_time_us"
           << ',' << side << "_ee_ik_to_ruckig_wall_time_us"
           << ',' << side << "_ee_ruckig_invoked"
           << ',' << side << "_ee_pinocchio_kinematics";
  output << ",simulation_phase\n";
  output << std::setprecision(12);

  const auto write_feedforward = [&output](const ArmIkSnapshot& arm) {
    output << ',' << arm.spark_feedforward_valid
           << ',' << arm.spark_feedforward_state
           << ',' << arm.spark_feedforward_dt_valid
           << ',' << arm.spark_feedforward_jump_rejected
           << ',' << arm.spark_feedforward_epoch_reset
           << ',' << arm.spark_feedforward_source_dt_seconds
           << ',' << arm.spark_feedforward_median_dt_seconds
           << ',' << arm.spark_feedforward_activation
           << ',' << arm.spark_feedforward_linear_velocity
           << ',' << arm.spark_feedforward_angular_velocity
           << ',' << arm.spark_motion_intent_linear_velocity
           << ',' << arm.spark_motion_intent_angular_velocity
           << ',' << (arm.spark_stationary_joint_reference_held ? 1 : 0)
           << ',' << (arm.spark_settled_hold_active ? 1 : 0)
           << ',' << arm.spark_settled_hold_dwell_seconds
           << ',' << toString(static_cast<SparkSettledHoldReason>(
                         arm.spark_settled_hold_reason));
    for (const Vec7* values : {&arm.spark_feedforward_q_ik,
                               &arm.spark_feedforward_q,
                               &arm.spark_feedforward_qdot,
                               &arm.spark_feedforward_qddot,
                               &arm.spark_feedforward_jerk}) {
      for (int joint = 0; joint < kArmDof; ++joint) {
        output << ',' << (*values)[joint];
      }
    }
    output << ',' << arm.headroom_valid
           << ',' << arm.headroom_derivative_history_valid
           << ',' << arm.headroom_velocity
           << ',' << arm.headroom_acceleration
           << ',' << arm.headroom_jerk
           << ',' << arm.headroom_task
           << ',' << arm.headroom_raw
           << ',' << arm.headroom_filtered
           << ',' << arm.headroom_scale
           << ',' << arm.headroom_state
           << ',' << arm.headroom_dominant_source;
  };

  TelemetrySample sample;
  for (;;) {
    if (telemetry.tryPop(sample)) {
      output << sample.sequence << ',' << sample.control_time_seconds << ','
             << toString(sample.algorithm) << ','
             << toString(sample.control_level) << ',' << toString(sample.mode) << ','
             << toString(sample.arm_angle_reference_mode) << ','
             << sample.paused << ',' << sample.accepted << ','
             << toString(sample.hold_reason) << ',' << sample.left_target_stale << ','
             << sample.right_target_stale << ',' << sample.otg_enabled << ','
             << sample.left_position_error << ','
             << sample.left_orientation_error << ','
             << sample.left_ik.slack_position_norm << ','
             << sample.left_ik.slack_orientation_norm << ','
             << sample.left_ik.equality_residual << ','
             << sample.left_ik.accepted << ','
             << sample.left_ik.fallback_applied << ','
             << toString(sample.left_ik.hold_reason) << ','
             << sample.left_ik.reference_error_max_abs << ','
             << sample.left_ik.reference_scale << ','
             << sample.left_ik.reference_frozen << ','
             << sample.left_ik.qdot_max_ratio << ','
             << sample.left_ik.solve_time_us << ','
             << sample.left_ik.active_position_bounds << ','
             << sample.left_ik.active_velocity_bounds << ','
             << sample.left_ik.active_acceleration_bounds << ','
             << sample.left_ik.active_braking_bounds << ','
             << sample.left_ik.iterations << ','
             << solverStatusName(sample.left_ik.status) << ','
             << sample.left_ik.otg_valid << ','
             << sample.left_ik.otg_stale << ','
             << sample.left_ik.reference_linear_velocity << ','
             << sample.left_ik.reference_angular_velocity << ','
             << sample.left_ik.reference_linear_acceleration << ','
             << sample.left_ik.reference_angular_acceleration << ','
             << sample.left_ik.qdot_reference_error_max_abs << ','
             << sample.left_ik.qddot_max_ratio << ','
             << sample.left_ik.active_qddot_bounds << ','
             << sample.left_ik.task_scale_position << ','
             << sample.left_ik.task_scale_orientation << ','
             << sample.right_position_error << ','
             << sample.right_orientation_error << ','
             << sample.right_ik.slack_position_norm << ','
             << sample.right_ik.slack_orientation_norm << ','
             << sample.right_ik.equality_residual << ','
             << sample.right_ik.accepted << ','
             << sample.right_ik.fallback_applied << ','
             << toString(sample.right_ik.hold_reason) << ','
             << sample.right_ik.reference_error_max_abs << ','
             << sample.right_ik.reference_scale << ','
             << sample.right_ik.reference_frozen << ','
             << sample.right_ik.qdot_max_ratio << ','
             << sample.right_ik.solve_time_us << ','
             << sample.right_ik.active_position_bounds << ','
             << sample.right_ik.active_velocity_bounds << ','
             << sample.right_ik.active_acceleration_bounds << ','
             << sample.right_ik.active_braking_bounds << ','
             << sample.right_ik.iterations << ','
             << solverStatusName(sample.right_ik.status) << ','
             << sample.right_ik.otg_valid << ','
             << sample.right_ik.otg_stale << ','
             << sample.right_ik.reference_linear_velocity << ','
             << sample.right_ik.reference_angular_velocity << ','
             << sample.right_ik.reference_linear_acceleration << ','
             << sample.right_ik.reference_angular_acceleration << ','
             << sample.right_ik.qdot_reference_error_max_abs << ','
             << sample.right_ik.qddot_max_ratio << ','
             << sample.right_ik.active_qddot_bounds << ','
             << sample.right_ik.task_scale_position << ','
             << sample.right_ik.task_scale_orientation << ','
             << sample.cycle_time_us << ',' << sample.cycle_p99_us << ','
             << sample.deadline_misses << ',' << sample.control_failures << ','
             << sample.pico_configured << ',' << sample.pico_enabled << ','
             << sample.pico_live << ',' << sample.pico_stale << ','
             << sample.pico_tracking_epoch << ',' << sample.pico_sequence << ','
             << sample.pico_datagrams << ',' << sample.pico_accepted << ','
             << sample.pico_malformed << ',' << sample.pico_crc_failures << ','
             << sample.pico_reordered << ',' << sample.pico_jump_rejections << ','
             << sample.pico_superseded << ',' << sample.pico_epoch_resets << ','
             << sample.pico_resynchronizations << ','
             << sample.pico_reset_applies << ','
             << sample.pico_left_source_timestamp_ns << ','
             << sample.pico_right_source_timestamp_ns << ','
             << sample.pico_input_frequency_hz << ','
             << sample.pico_frame_age_ms << ','
             << sample.pico_receive_to_control_us << ','
             << sample.pico_bridge_to_control_us << ','
             << sample.left_ik.actual_position_error << ','
             << sample.left_ik.actual_orientation_error << ','
             << sample.right_ik.actual_position_error << ','
             << sample.right_ik.actual_orientation_error << ','
             << sample.left_target_pose.position.x() << ','
             << sample.left_target_pose.position.y() << ','
             << sample.left_target_pose.position.z() << ',';
      const Eigen::Quaterniond left_target_q(sample.left_target_pose.rotation);
      const Eigen::Quaterniond left_reference_q(sample.left_reference_pose.rotation);
      const Eigen::Quaterniond left_actual_q(sample.left_actual_pose.rotation);
      const Eigen::Quaterniond right_target_q(sample.right_target_pose.rotation);
      const Eigen::Quaterniond right_reference_q(sample.right_reference_pose.rotation);
      const Eigen::Quaterniond right_actual_q(sample.right_actual_pose.rotation);
      output << left_target_q.x() << ',' << left_target_q.y() << ','
             << left_target_q.z() << ',' << left_target_q.w() << ','
             << sample.left_reference_pose.position.x() << ','
             << sample.left_reference_pose.position.y() << ','
             << sample.left_reference_pose.position.z() << ','
             << left_reference_q.x() << ',' << left_reference_q.y() << ','
             << left_reference_q.z() << ',' << left_reference_q.w() << ','
             << sample.left_actual_pose.position.x() << ','
             << sample.left_actual_pose.position.y() << ','
             << sample.left_actual_pose.position.z() << ','
             << left_actual_q.x() << ',' << left_actual_q.y() << ','
             << left_actual_q.z() << ',' << left_actual_q.w() << ','
             << sample.right_target_pose.position.x() << ','
             << sample.right_target_pose.position.y() << ','
             << sample.right_target_pose.position.z() << ','
             << right_target_q.x() << ',' << right_target_q.y() << ','
             << right_target_q.z() << ',' << right_target_q.w() << ','
             << sample.right_reference_pose.position.x() << ','
             << sample.right_reference_pose.position.y() << ','
             << sample.right_reference_pose.position.z() << ','
             << right_reference_q.x() << ',' << right_reference_q.y() << ','
             << right_reference_q.z() << ',' << right_reference_q.w() << ','
             << sample.right_actual_pose.position.x() << ','
             << sample.right_actual_pose.position.y() << ','
             << sample.right_actual_pose.position.z() << ','
             << right_actual_q.x() << ',' << right_actual_q.y() << ','
             << right_actual_q.z() << ',' << right_actual_q.w() << ','
             << toString(sample.left_ik.arm_angle_reference_source) << ','
             << sample.left_ik.arm_angle_active << ','
             << sample.left_ik.arm_angle_error_rad << ','
             << sample.left_ik.arm_angle_robot_rad << ','
             << sample.left_ik.arm_angle_target_rad << ','
             << sample.left_ik.arm_angle_current_rate_rad_s << ','
             << sample.left_ik.arm_angle_requested_velocity_rad_s << ','
             << sample.left_ik.arm_angle_requested_acceleration_rad_s2 << ','
             << sample.left_ik.arm_angle_radius_m << ','
             << sample.left_ik.arm_angle_reference_projection_norm << ','
             << sample.left_ik.arm_angle_jacobian_norm << ','
             << sample.left_ik.arm_angle_projection_held << ','
             << sample.left_ik.arm_angle_reference_governor_held << ','
             << sample.left_ik.arm_angle_branch_lock_active << ','
             << sample.left_ik.arm_angle_branch_lock_distance_m << ','
             << sample.left_ik.arm_angle_branch_lock_constraint_active << ','
             << sample.left_ik.arm_angle_branch_lock_requested_lower << ','
             << sample.left_ik.arm_angle_branch_lock_effective_lower << ','
             << sample.left_ik.arm_angle_branch_lock_feasibility_clipped << ','
             << sample.left_ik.arm_angle_achieved_acceleration_rad_s2 << ','
             << sample.left_ik.arm_angle_acceleration_residual_rad_s2 << ','
             << sample.left_ik.elbow_world_z << ','
             << sample.left_ik.shoulder_world_z << ','
             << sample.left_ik.upper_arm_outward_active << ','
             << sample.left_ik.upper_arm_outward_distance_m << ','
             << sample.left_ik.upper_arm_outward_requested_lower << ','
             << sample.left_ik.upper_arm_outward_effective_lower << ','
             << sample.left_ik.upper_arm_outward_achieved << ','
             << sample.left_ik.upper_arm_outward_residual << ','
             << sample.left_ik.upper_arm_outward_feasibility_clipped << ','
             << toString(sample.right_ik.arm_angle_reference_source) << ','
             << sample.right_ik.arm_angle_active << ','
             << sample.right_ik.arm_angle_error_rad << ','
             << sample.right_ik.arm_angle_robot_rad << ','
             << sample.right_ik.arm_angle_target_rad << ','
             << sample.right_ik.arm_angle_current_rate_rad_s << ','
             << sample.right_ik.arm_angle_requested_velocity_rad_s << ','
             << sample.right_ik.arm_angle_requested_acceleration_rad_s2 << ','
             << sample.right_ik.arm_angle_radius_m << ','
             << sample.right_ik.arm_angle_reference_projection_norm << ','
             << sample.right_ik.arm_angle_jacobian_norm << ','
             << sample.right_ik.arm_angle_projection_held << ','
             << sample.right_ik.arm_angle_reference_governor_held << ','
             << sample.right_ik.arm_angle_branch_lock_active << ','
             << sample.right_ik.arm_angle_branch_lock_distance_m << ','
             << sample.right_ik.arm_angle_branch_lock_constraint_active << ','
             << sample.right_ik.arm_angle_branch_lock_requested_lower << ','
             << sample.right_ik.arm_angle_branch_lock_effective_lower << ','
             << sample.right_ik.arm_angle_branch_lock_feasibility_clipped << ','
             << sample.right_ik.arm_angle_achieved_acceleration_rad_s2 << ','
             << sample.right_ik.arm_angle_acceleration_residual_rad_s2 << ','
             << sample.right_ik.elbow_world_z << ','
             << sample.right_ik.shoulder_world_z << ','
             << sample.right_ik.upper_arm_outward_active << ','
             << sample.right_ik.upper_arm_outward_distance_m << ','
             << sample.right_ik.upper_arm_outward_requested_lower << ','
             << sample.right_ik.upper_arm_outward_effective_lower << ','
             << sample.right_ik.upper_arm_outward_achieved << ','
             << sample.right_ik.upper_arm_outward_residual << ','
             << sample.right_ik.upper_arm_outward_feasibility_clipped << ','
             << sample.left_ik.arm_angle_control_error_rad << ','
             << sample.left_ik.dls_posture_reference_active << ','
             << sample.left_ik.dls_posture_status << ','
             << sample.left_ik.dls_posture_iterations << ','
             << sample.left_ik.dls_posture_joint_projection_count << ','
             << sample.left_ik.dls_posture_ruckig_accepted << ','
             << sample.left_ik.dls_posture_solve_time_us << ','
             << sample.left_ik.dls_posture_initial_position_error_m << ','
             << sample.left_ik.dls_posture_initial_orientation_error_rad << ','
             << sample.left_ik.dls_posture_final_position_error_m << ','
             << sample.left_ik.dls_posture_final_orientation_error_rad << ','
             << sample.left_ik.dls_posture_goal_error_max_abs << ','
             << sample.left_ik.dls_posture_reference_error_max_abs << ','
             << sample.left_ik.dls_posture_velocity_target_max_abs << ','
             << sample.left_ik.dls_posture_qdot_error_max_abs << ','
             << sample.left_ik.dls_posture_goal_limit_margin_rad << ','
             << sample.right_ik.arm_angle_control_error_rad << ','
             << sample.right_ik.dls_posture_reference_active << ','
             << sample.right_ik.dls_posture_status << ','
             << sample.right_ik.dls_posture_iterations << ','
             << sample.right_ik.dls_posture_joint_projection_count << ','
             << sample.right_ik.dls_posture_ruckig_accepted << ','
             << sample.right_ik.dls_posture_solve_time_us << ','
             << sample.right_ik.dls_posture_initial_position_error_m << ','
             << sample.right_ik.dls_posture_initial_orientation_error_rad << ','
             << sample.right_ik.dls_posture_final_position_error_m << ','
             << sample.right_ik.dls_posture_final_orientation_error_rad << ','
             << sample.right_ik.dls_posture_goal_error_max_abs << ','
             << sample.right_ik.dls_posture_reference_error_max_abs << ','
             << sample.right_ik.dls_posture_velocity_target_max_abs << ','
             << sample.right_ik.dls_posture_qdot_error_max_abs << ','
             << sample.right_ik.dls_posture_goal_limit_margin_rad << ','
             << sample.left_ik.spark_posture_active << ','
             << sample.left_ik.spark_ik_accepted << ','
             << sample.left_ik.spark_stage1_iterations << ','
             << sample.left_ik.spark_stage2_iterations << ','
             << sample.left_ik.spark_solve_time_us << ','
             << sample.left_ik.spark_palm_position_error_m << ','
             << sample.left_ik.spark_palm_orientation_error_rad << ','
             << sample.left_ik.spark_reference_velocity_ratio << ','
             << sample.left_ik.spark_reference_acceleration_ratio << ','
             << sample.left_ik.spark_reference_jerk_ratio << ','
             << sample.left_ik.spark_q_ik_error_max_abs << ','
             << sample.left_ik.spark_q_ref_error_max_abs << ','
             << sample.left_ik.spark_posture_velocity_max_abs << ','
             << sample.right_ik.spark_posture_active << ','
             << sample.right_ik.spark_ik_accepted << ','
             << sample.right_ik.spark_stage1_iterations << ','
             << sample.right_ik.spark_stage2_iterations << ','
             << sample.right_ik.spark_solve_time_us << ','
             << sample.right_ik.spark_palm_position_error_m << ','
             << sample.right_ik.spark_palm_orientation_error_rad << ','
             << sample.right_ik.spark_reference_velocity_ratio << ','
             << sample.right_ik.spark_reference_acceleration_ratio << ','
             << sample.right_ik.spark_reference_jerk_ratio << ','
             << sample.right_ik.spark_q_ik_error_max_abs << ','
             << sample.right_ik.spark_q_ref_error_max_abs << ','
             << sample.right_ik.spark_posture_velocity_max_abs;
      write_feedforward(sample.left_ik);
      write_feedforward(sample.right_ik);
      for (const auto* arm : {&sample.left_ik, &sample.right_ik})
        output << ',' << arm->ee_ik_wall_time_us << ',' << arm->ee_ruckig_wall_time_us
               << ',' << arm->ee_ik_to_ruckig_wall_time_us << ',' << arm->ee_ruckig_invoked
               << ',' << arm->ee_pinocchio_kinematics;
      output << ',' << sample.simulation_phase << '\n';
      continue;
    }
    if (control_finished.load(std::memory_order_acquire)) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  output.flush();
  if (!output) {
    throw std::runtime_error("failed while writing telemetry output: " + path);
  }
}

void writeJointTelemetryCsv(
    const std::string& path,
    BoundedSpscQueue<JointKinematicsSample>& telemetry,
    const std::atomic<bool>& control_finished) {
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("cannot open joint telemetry output: " + path);
  }
  output << "sequence,control_time_seconds,reset";
  constexpr std::array<const char*, 16> kFields{
      "reference_q", "reference_qdot", "reference_qddot", "reference_jerk",
      "actual_q", "actual_qdot", "actual_qddot", "actual_jerk",
      "position_lower", "position_upper", "velocity_lower", "velocity_upper",
      "acceleration_lower", "acceleration_upper", "jerk_lower", "jerk_upper"};
  for (const char* side : {"left", "right"}) {
    for (int joint = 0; joint < kArmDof; ++joint) {
      for (const char* field : kFields) {
        output << ',' << side << "_j" << (joint + 1) << '_' << field;
      }
    }
  }
  output << ",left_reference_acceleration_valid,left_reference_jerk_valid,"
            "left_actual_acceleration_valid,left_actual_jerk_valid,"
            "right_reference_acceleration_valid,right_reference_jerk_valid,"
            "right_actual_acceleration_valid,right_actual_jerk_valid,"
            "left_ruckig_output,right_ruckig_output\n";
  output << std::setprecision(12);

  const auto writeArm = [&output](const ArmJointKinematicsSample& arm) {
    for (int joint = 0; joint < kArmDof; ++joint) {
      output << ',' << arm.reference.position[joint]
             << ',' << arm.reference.velocity[joint]
             << ',' << arm.reference.acceleration[joint]
             << ',' << arm.reference.jerk[joint]
             << ',' << arm.actual.position[joint]
             << ',' << arm.actual.velocity[joint]
             << ',' << arm.actual.acceleration[joint]
             << ',' << arm.actual.jerk[joint]
             << ',' << arm.bounds.position_lower[joint]
             << ',' << arm.bounds.position_upper[joint]
             << ',' << arm.bounds.velocity_lower[joint]
             << ',' << arm.bounds.velocity_upper[joint]
             << ',' << arm.bounds.acceleration_lower[joint]
             << ',' << arm.bounds.acceleration_upper[joint]
             << ',' << arm.bounds.jerk_lower[joint]
             << ',' << arm.bounds.jerk_upper[joint];
    }
  };

  JointKinematicsSample sample;
  for (;;) {
    if (telemetry.tryPop(sample)) {
      output << sample.sequence << ',' << sample.time_seconds << ','
             << sample.reset;
      writeArm(sample.left);
      writeArm(sample.right);
      output << ',' << sample.left.reference_acceleration_valid
             << ',' << sample.left.reference_jerk_valid
             << ',' << sample.left.actual_acceleration_valid
             << ',' << sample.left.actual_jerk_valid
             << ',' << sample.right.reference_acceleration_valid
             << ',' << sample.right.reference_jerk_valid
             << ',' << sample.right.actual_acceleration_valid
             << ',' << sample.right.actual_jerk_valid
             << ',' << sample.left.ruckig_output
             << ',' << sample.right.ruckig_output << '\n';
      continue;
    }
    if (control_finished.load(std::memory_order_acquire)) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  output.flush();
  if (!output) {
    throw std::runtime_error("failed while writing joint telemetry output: " +
                             path);
  }
}

std::string solverStatusName(SolverStatus status) {
  switch (status) {
    case SolverStatus::kSolved:
      return "solved";
    case SolverStatus::kMaxIterations:
      return "max_iterations";
    case SolverStatus::kInfeasible:
      return "infeasible";
    case SolverStatus::kNumericalError:
      return "numerical_error";
    case SolverStatus::kInvalidInput:
      return "invalid_input";
  }
  return "unknown";
}

std::optional<std::uint64_t> pushCommand(ViewerApplication& application,
                                         ViewerCommand command) {
  command.id = application.next_command_id++;
  if (command.type == ViewerCommandType::kSetManualTarget &&
      !std::isfinite(command.target_timestamp_seconds)) {
    command.target_timestamp_seconds =
        std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count();
  }
  const bool accepted = application.commands.tryPush(command);
  if (!accepted) {
    ++application.command_drops;
    return std::nullopt;
  }
  return command.id;
}

void setMocapPose(MujocoRobot& robot, const std::string& body_name, const Pose& pose) {
  const mjModel* model = robot.model();
  mjData* data = robot.data();
  const int body_id = mj_name2id(model, mjOBJ_BODY, body_name.c_str());
  if (body_id < 0 || model->body_mocapid[body_id] < 0) {
    throw std::runtime_error("missing mocap body: " + body_name);
  }
  const int mocap_id = model->body_mocapid[body_id];
  for (int axis = 0; axis < 3; ++axis) {
    data->mocap_pos[3 * mocap_id + axis] = pose.position[axis];
  }
  const Eigen::Quaterniond quaternion(pose.rotation);
  data->mocap_quat[4 * mocap_id] = quaternion.w();
  data->mocap_quat[4 * mocap_id + 1] = quaternion.x();
  data->mocap_quat[4 * mocap_id + 2] = quaternion.y();
  data->mocap_quat[4 * mocap_id + 3] = quaternion.z();
}

Pose mocapPose(const ViewerApplication& application, ArmSide side) {
  const mjModel* model = application.robot.model();
  const mjData* data = application.robot.data();
  const int body_id = side == ArmSide::kLeft ? application.target_left_body
                                             : application.target_right_body;
  const int mocap_id = model->body_mocapid[body_id];
  Pose pose;
  pose.position = Eigen::Map<const Eigen::Vector3d>(&data->mocap_pos[3 * mocap_id]);
  const Eigen::Quaterniond quaternion(data->mocap_quat[4 * mocap_id],
                                     data->mocap_quat[4 * mocap_id + 1],
                                     data->mocap_quat[4 * mocap_id + 2],
                                     data->mocap_quat[4 * mocap_id + 3]);
  pose.rotation = quaternion.normalized().toRotationMatrix();
  return pose;
}

void selectTarget(ViewerApplication& application, ArmSide side) {
  application.marker.cancelDrag();
  application.hovered_handle = MarkerHandle::kNone;
  application.selected_arm = side;
  if (!application.joint_plot_arm_locked) {
    application.joint_plot_arm = side;
  }
}

void sendMode(ViewerApplication& application, TargetMode mode) {
  application.marker.cancelDrag();
  if (mode != TargetMode::kManual) {
    application.preview.cancelAll();
  }
  ViewerCommand command;
  command.type = ViewerCommandType::kSetMode;
  command.mode = mode;
  (void)pushCommand(application, command);
}

void keyboardCallback(GLFWwindow* window, int key, int, int action, int) {
  if (action != GLFW_PRESS) {
    return;
  }
  auto& application = *static_cast<ViewerApplication*>(glfwGetWindowUserPointer(window));
  ViewerCommand command;
  if(application.snapshot.simulation_phase>=0&&key!=GLFW_KEY_ESCAPE&&
      key!=GLFW_KEY_F1&&key!=GLFW_KEY_F2&&key!=GLFW_KEY_F3&&
      key!=GLFW_KEY_F4&&key!=GLFW_KEY_F5&&key!=GLFW_KEY_K) {
    if(key==GLFW_KEY_S)command.type=ViewerCommandType::kSimulationStart;
    else if(key==GLFW_KEY_H)command.type=ViewerCommandType::kSimulationHome;
    else if(key==GLFW_KEY_P||key==GLFW_KEY_SPACE)command.type=ViewerCommandType::kSimulationHold;
    else return;
    (void)pushCommand(application,command);
    return;
  }
  switch (key) {
    case GLFW_KEY_ESCAPE:
      glfwSetWindowShouldClose(window, GLFW_TRUE);
      break;
    case GLFW_KEY_L:
      selectTarget(application, ArmSide::kLeft);
      break;
    case GLFW_KEY_R:
      selectTarget(application, ArmSide::kRight);
      break;
    case GLFW_KEY_0:
    case GLFW_KEY_M:
      sendMode(application, TargetMode::kManual);
      break;
    case GLFW_KEY_1:
      sendMode(application, TargetMode::kCircle);
      break;
    case GLFW_KEY_2:
      sendMode(application, TargetMode::kFigureEight);
      break;
    case GLFW_KEY_3:
      sendMode(application, TargetMode::kOrientationOnly);
      break;
    case GLFW_KEY_4:
      sendMode(application, TargetMode::kCombined);
      break;
    case GLFW_KEY_H:
      sendMode(application, TargetMode::kHold);
      break;
    case GLFW_KEY_W:
      application.marker.cancelDrag();
      application.marker.setFrame(application.marker.frame() == MarkerFrame::kWorld
                                      ? MarkerFrame::kLocal
                                      : MarkerFrame::kWorld);
      break;
    case GLFW_KEY_SPACE:
      application.marker.cancelDrag();
      command.type = ViewerCommandType::kSetPaused;
      command.paused = !application.snapshot.paused;
      (void)pushCommand(application, command);
      break;
    case GLFW_KEY_N:
      application.marker.cancelDrag();
      application.preview.cancelAll();
      command.type = ViewerCommandType::kResetNominal;
      (void)pushCommand(application, command);
      break;
    case GLFW_KEY_Q:
      application.marker.cancelDrag();
      command.type = ViewerCommandType::kSetIkAlgorithm;
      command.algorithm = IkAlgorithm::kHierarchicalQp;
      (void)pushCommand(application, command);
      break;
    case GLFW_KEY_D:
      application.marker.cancelDrag();
      command.type = ViewerCommandType::kSetIkAlgorithm;
      command.algorithm = IkAlgorithm::kNullspaceDls;
      (void)pushCommand(application, command);
      break;
    case GLFW_KEY_V:
      application.marker.cancelDrag();
      command.type = ViewerCommandType::kSetControlLevel;
      command.control_level = ControlLevel::kVelocity;
      (void)pushCommand(application, command);
      break;
    case GLFW_KEY_A:
      application.marker.cancelDrag();
      command.type = ViewerCommandType::kSetControlLevel;
      command.control_level = ControlLevel::kAcceleration;
      (void)pushCommand(application, command);
      break;
    case GLFW_KEY_P:
      application.marker.cancelDrag();
      application.preview.cancelAll();
      command.type = ViewerCommandType::kTogglePicoTeleop;
      (void)pushCommand(application, command);
      break;
    case GLFW_KEY_G:
      application.marker.cancelDrag();
      command.type = ViewerCommandType::kTogglePicoArmAngleSource;
      (void)pushCommand(application, command);
      break;
    case GLFW_KEY_K:
      application.show_pico_skeleton = !application.show_pico_skeleton;
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

MarkerPointer currentMarkerPointer(const ViewerApplication& application,
                                   const ViewportCursor& cursor) {
  const double relx =
      std::clamp(cursor.pixel.x() / static_cast<double>(cursor.width), 0.0, 1.0);
  const double rely =
      std::clamp(cursor.pixel.y() / static_cast<double>(cursor.height), 0.0, 1.0);
  const Pose target = mocapPose(application, application.selected_arm);
  return markerPointerFromScene(application.scene,
                                static_cast<double>(cursor.width) /
                                    static_cast<double>(cursor.height),
                                relx, rely, target.position);
}

MarkerCamera currentMarkerCamera(const ViewerApplication& application) {
  const Pose target = mocapPose(application, application.selected_arm);
  return markerCameraFromScene(application.scene, target.position);
}

MarkerHandle currentMarkerHandle(const ViewerApplication& application,
                                 const ViewportCursor& cursor) {
  const Pose target = mocapPose(application, application.selected_arm);
  const MarkerScreenPick pick = pickMarkerHandle(
      application.marker.geometry(target), application.scene,
      cursor.width, cursor.height, cursor.pixel);
  return pick.handle;
}

void beginMarkerDrag(ViewerApplication& application, const ViewportCursor& cursor) {
  const Pose target = mocapPose(application, application.selected_arm);
  const MarkerPointer pointer = currentMarkerPointer(application, cursor);
  const MarkerHandle handle = currentMarkerHandle(application, cursor);
  application.hovered_handle = handle;
  if (handle != MarkerHandle::kNone) {
    (void)application.marker.beginDrag(handle, target, pointer,
                                       currentMarkerCamera(application));
  }
}

void selectTargetAtCursor(ViewerApplication& application,
                          const ViewportCursor& cursor) {
  mjtNum selected_point[3]{};
  int geom_id = -1;
  int flex_id = -1;
  int skin_id = -1;
  const int selected_body = mjv_select(
      application.robot.model(), application.robot.data(),
      &application.visual_options,
      static_cast<mjtNum>(cursor.width) / static_cast<mjtNum>(cursor.height),
      static_cast<mjtNum>(cursor.pixel.x()) / static_cast<mjtNum>(cursor.width),
      static_cast<mjtNum>(cursor.pixel.y()) / static_cast<mjtNum>(cursor.height),
      &application.scene, selected_point, &geom_id, &flex_id, &skin_id);
  if (selected_body == application.target_left_body) {
    selectTarget(application, ArmSide::kLeft);
  } else if (selected_body == application.target_right_body) {
    selectTarget(application, ArmSide::kRight);
  }
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
    application.marker.cancelDrag();
    return;
  }

  if (button != GLFW_MOUSE_BUTTON_LEFT || action != GLFW_PRESS) {
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
      application.marker.endDrag();
    }
    return;
  }

  const ViewportCursor cursor =
      viewportCursor(window, application.previous_x, application.previous_y);
  selectTargetAtCursor(application, cursor);
  beginMarkerDrag(application, cursor);
  if (application.marker.activeHandle() != MarkerHandle::kNone) {
    return;
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
    application.hovered_handle = MarkerHandle::kNone;
    application.marker.cancelDrag();
    return;
  }
  if (!application.left_button && !application.middle_button && !application.right_button) {
    application.previous_x = x;
    application.previous_y = y;
    application.hovered_handle =
        currentMarkerHandle(application, viewportCursor(window, x, y));
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
  if (application.left_button &&
      application.marker.activeHandle() != MarkerHandle::kNone) {
    const MarkerPointer pointer =
        currentMarkerPointer(application, viewportCursor(window, x, y));
    const std::optional<Pose> target = application.marker.updateDrag(pointer);
    if (target.has_value()) {
      ViewerCommand command;
      command.type = ViewerCommandType::kSetManualTarget;
      command.side = application.selected_arm;
      command.target = *target;
      const std::optional<std::uint64_t> command_id = pushCommand(application, command);
      if (command_id.has_value()) {
        application.preview.record(command.side, command.target, *command_id);
      }
    }
    return;
  }

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
  application.marker.cancelDrag();
  application.hovered_handle = MarkerHandle::kNone;
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

void renderSnapshot(ViewerApplication& application) {
  application.robot.setArmPosition(ArmSide::kLeft, application.snapshot.left_q);
  application.robot.setArmPosition(ArmSide::kRight, application.snapshot.right_q);
  if (application.robot.hasHandMappings()) {
    application.robot.setHandPosition(ArmSide::kLeft,
                                      application.snapshot.left_hand_q);
    application.robot.setHandPosition(ArmSide::kRight,
                                      application.snapshot.right_hand_q);
  }
  const Pose left_target = application.preview.resolve(
      ArmSide::kLeft, application.snapshot.targets.left,
      application.snapshot.last_processed_command_id);
  const Pose right_target = application.preview.resolve(
      ArmSide::kRight, application.snapshot.targets.right,
      application.snapshot.last_processed_command_id);
  setMocapPose(application.robot, "target_L", left_target);
  setMocapPose(application.robot, "target_R", right_target);
  application.robot.forward();
}

void drawOverlay(const ViewerApplication& application, mjrRect viewport) {
  char status[2048]{};
  char help[1024]{};
  const char* selected = application.selected_arm == ArmSide::kLeft ? "left" : "right";
  const char* frame = application.marker.frame() == MarkerFrame::kWorld ? "world" : "local";
  std::snprintf(
      status, sizeof(status),
      "IK: %s / %s | mode: %s | selected: %s | marker: %s%s\n"
      "OTG: %s | left v/w/a/alpha %.2f/%.2f/%.2f/%.2f | right %.2f/%.2f/%.2f/%.2f\n"
      "qddot ratio/bounds/qdot-ref: left %.2f/%d/%.3f | right %.2f/%d/%.3f\n"
      "left  err %.4f m / %.3f rad | slack %.4f / %.4f | eq %.1e | "
      "bounds p/v/a/b %d/%d/%d/%d | ref %.2f | %.1f us | %s\n"
      "right err %.4f m / %.3f rad | slack %.4f / %.4f | eq %.1e | "
      "bounds p/v/a/b %d/%d/%d/%d | ref %.2f | %.1f us | %s\n"
      "cycle: %.1f us | rolling p99: %.1f us | deadline misses: %llu\n"
      "control failures: %llu | snapshot/telemetry drops: %llu / %llu | command drops: %llu\n"
      "PICO cfg/en/live/stale %d/%d/%d/%d | epoch/seq %llu/%llu | %.1f Hz | age %.1f ms\n"
      "PICO rx accepted/datagrams %llu/%llu | bad/crc/order/jump/super/epoch/resync/reset %llu/%llu/%llu/%llu/%llu/%llu/%llu/%llu | latency recv/bridge %.1f/%.1f us\n"
      "arm angle mode: %s | effective left/right: %s/%s\n"
      "skeleton overlay: %s | PICO %s | Spark %s\n"
      "safety: %s",
      toString(application.snapshot.algorithm).c_str(),
      toString(application.snapshot.control_level).c_str(),
      toString(application.snapshot.mode).c_str(),
      selected, frame, application.snapshot.paused ? " | PAUSED" : "",
      application.snapshot.otg_enabled ? "enabled" : "direct",
      application.snapshot.left_ik.reference_linear_velocity,
      application.snapshot.left_ik.reference_angular_velocity,
      application.snapshot.left_ik.reference_linear_acceleration,
      application.snapshot.left_ik.reference_angular_acceleration,
      application.snapshot.right_ik.reference_linear_velocity,
      application.snapshot.right_ik.reference_angular_velocity,
      application.snapshot.right_ik.reference_linear_acceleration,
      application.snapshot.right_ik.reference_angular_acceleration,
      application.snapshot.left_ik.qddot_max_ratio,
      application.snapshot.left_ik.active_qddot_bounds,
      application.snapshot.left_ik.qdot_reference_error_max_abs,
      application.snapshot.right_ik.qddot_max_ratio,
      application.snapshot.right_ik.active_qddot_bounds,
      application.snapshot.right_ik.qdot_reference_error_max_abs,
      application.snapshot.left_position_error, application.snapshot.left_orientation_error,
      application.snapshot.left_ik.slack_position_norm,
      application.snapshot.left_ik.slack_orientation_norm,
      application.snapshot.left_ik.equality_residual,
      application.snapshot.left_ik.active_position_bounds,
      application.snapshot.left_ik.active_velocity_bounds,
      application.snapshot.left_ik.active_acceleration_bounds,
      application.snapshot.left_ik.active_braking_bounds,
      application.snapshot.left_ik.reference_scale,
      application.snapshot.left_ik.solve_time_us,
      solverStatusName(application.snapshot.left_solver_status).c_str(),
      application.snapshot.right_position_error, application.snapshot.right_orientation_error,
      application.snapshot.right_ik.slack_position_norm,
      application.snapshot.right_ik.slack_orientation_norm,
      application.snapshot.right_ik.equality_residual,
      application.snapshot.right_ik.active_position_bounds,
      application.snapshot.right_ik.active_velocity_bounds,
      application.snapshot.right_ik.active_acceleration_bounds,
      application.snapshot.right_ik.active_braking_bounds,
      application.snapshot.right_ik.reference_scale,
      application.snapshot.right_ik.solve_time_us,
      solverStatusName(application.snapshot.right_solver_status).c_str(),
      application.snapshot.cycle_time_us, application.snapshot.cycle_p99_us,
      static_cast<unsigned long long>(application.snapshot.deadline_misses),
      static_cast<unsigned long long>(application.snapshot.control_failures),
      static_cast<unsigned long long>(application.snapshot.snapshot_drops),
      static_cast<unsigned long long>(application.snapshot.telemetry_drops),
      static_cast<unsigned long long>(application.command_drops),
      application.snapshot.pico_configured,
      application.snapshot.pico_enabled,
      application.snapshot.pico_live,
      application.snapshot.pico_stale,
      static_cast<unsigned long long>(application.snapshot.pico_tracking_epoch),
      static_cast<unsigned long long>(application.snapshot.pico_sequence),
      application.snapshot.pico_input_frequency_hz,
      application.snapshot.pico_frame_age_ms,
      static_cast<unsigned long long>(application.snapshot.pico_accepted),
      static_cast<unsigned long long>(application.snapshot.pico_datagrams),
      static_cast<unsigned long long>(application.snapshot.pico_malformed),
      static_cast<unsigned long long>(application.snapshot.pico_crc_failures),
      static_cast<unsigned long long>(application.snapshot.pico_reordered),
      static_cast<unsigned long long>(application.snapshot.pico_jump_rejections),
      static_cast<unsigned long long>(application.snapshot.pico_superseded),
      static_cast<unsigned long long>(application.snapshot.pico_epoch_resets),
      static_cast<unsigned long long>(
          application.snapshot.pico_resynchronizations),
      static_cast<unsigned long long>(application.snapshot.pico_reset_applies),
      application.snapshot.pico_receive_to_control_us,
      application.snapshot.pico_bridge_to_control_us,
      toString(application.snapshot.arm_angle_reference_mode).data(),
      toString(application.snapshot.left_ik.arm_angle_reference_source).data(),
      toString(application.snapshot.right_ik.arm_angle_reference_source).data(),
      application.show_pico_skeleton ? "on" : "off",
      application.snapshot.pico_upper_limb_skeleton.valid ? "valid" : "hidden",
      application.snapshot.spark_upper_limb_skeleton.valid ? "valid" : "hidden",
      toString(application.snapshot.hold_reason).c_str());
  if (application.show_help) {
    std::snprintf(help, sizeof(help),
                  "L/R select | drag XYZ arrow: translate | drag XYZ ring: rotate | "
                  "drag center: view plane | W world/local\n"
                  "0/M manual | 1 circle | 2 figure-8 | 3 orientation | 4 combined | H hold\n"
                  "V velocity QP | A acceleration QP | Q hierarchical QP | D null-space DLS\n"
                  "P PICO teleop | G arm-angle source (non-PICO) | K PICO skeleton | Space pause | N nominal reset\n"
                  "F1 help | F2 plots | F3 q/dq/ddq/jerk | "
                  "F4 plot L/R lock | F5 plot follows selected\n"
                  "Esc quit | "
                  "Right drag pan | middle drag/wheel zoom");
  }
  if (application.show_help && application.snapshot.simulation_phase >= 0) {
    std::snprintf(help, sizeof(help),
        "Direct IK + Ruckig model-reference simulation (no hardware output)\n"
        "S: start with fresh PICO | H: smooth Home, then wait\n"
        "P / Space: bounded stop | stale input: stop, S required again\n"
        "F1: help | F2-F5: plots | K: skeleton | Esc: quit");
  }
  mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, status,
              application.show_help ? help : nullptr, &application.context);
}

ViewerCommand headlessCommandForStage(int stage, const ViewerSnapshot& latest) {
  ViewerCommand command;
  switch (stage) {
    case 0:
      command.type = ViewerCommandType::kSetIkAlgorithm;
      command.algorithm = IkAlgorithm::kHierarchicalQp;
      break;
    case 1:
      command.type = ViewerCommandType::kSetManualTarget;
      command.side = ArmSide::kLeft;
      command.target = latest.targets.left;
      command.target.position.x() += 0.02;
      break;
    case 2:
      command.type = ViewerCommandType::kSetMode;
      command.mode = TargetMode::kCircle;
      break;
    case 3:
      command.type = ViewerCommandType::kSetMode;
      command.mode = TargetMode::kFigureEight;
      break;
    case 4:
      command.type = ViewerCommandType::kSetMode;
      command.mode = TargetMode::kOrientationOnly;
      break;
    case 5:
      command.type = ViewerCommandType::kSetMode;
      command.mode = TargetMode::kCombined;
      break;
    case 6:
      command.type = ViewerCommandType::kSetControlLevel;
      command.control_level = ControlLevel::kVelocity;
      break;
    case 7:
      command.type = ViewerCommandType::kSetIkAlgorithm;
      command.algorithm = IkAlgorithm::kNullspaceDls;
      break;
    case 8:
      command.type = ViewerCommandType::kSetIkAlgorithm;
      command.algorithm = IkAlgorithm::kHierarchicalQp;
      break;
    case 9:
      command.type = ViewerCommandType::kSetControlLevel;
      command.control_level = ControlLevel::kAcceleration;
      break;
    case 10:
      command.type = ViewerCommandType::kSetPaused;
      command.paused = true;
      break;
    case 11:
      command.type = ViewerCommandType::kSetPaused;
      command.paused = false;
      break;
    case 12:
      command.type = ViewerCommandType::kResetNominal;
      break;
    default:
      throw std::invalid_argument("invalid headless stage");
  }
  return command;
}

bool headlessStageObserved(int stage, const ViewerSnapshot& snapshot) {
  switch (stage) {
    case 0:
      return snapshot.algorithm == IkAlgorithm::kHierarchicalQp;
    case 1:
      return snapshot.mode == TargetMode::kManual &&
             snapshot.left_position_error > 1e-6;
    case 2:
      return snapshot.mode == TargetMode::kCircle;
    case 3:
      return snapshot.mode == TargetMode::kFigureEight;
    case 4:
      return snapshot.mode == TargetMode::kOrientationOnly;
    case 5:
      return snapshot.mode == TargetMode::kCombined;
    case 6:
      return snapshot.control_level == ControlLevel::kVelocity;
    case 7:
      return snapshot.algorithm == IkAlgorithm::kNullspaceDls;
    case 8:
      return snapshot.algorithm == IkAlgorithm::kHierarchicalQp;
    case 9:
      return snapshot.control_level == ControlLevel::kAcceleration;
    case 10:
      return snapshot.paused;
    case 11:
      return !snapshot.paused && snapshot.mode == TargetMode::kHold;
    case 12:
      return !snapshot.paused && snapshot.mode == TargetMode::kHold &&
             snapshot.at_nominal_configuration;
    default:
      return false;
  }
}

struct JointPlotObservation {
  std::uint64_t drained{0U};
  bool both_arms_finite{false};
  bool reference_jerk_valid_seen{false};
  bool actual_jerk_valid_seen{false};
};

bool armJointPlotStateFinite(const ArmJointKinematicsSample& arm) noexcept {
  return arm.reference.position.allFinite() &&
         arm.reference.velocity.allFinite() &&
         arm.reference.acceleration.allFinite() &&
         arm.reference.jerk.allFinite() && arm.actual.position.allFinite() &&
         arm.actual.velocity.allFinite() &&
         arm.actual.acceleration.allFinite() && arm.actual.jerk.allFinite();
}

void drainJointPlotQueue(
    BoundedSpscQueue<JointKinematicsSample>& joint_plot_queue,
    JointPlotObservation& observation,
    JointKinematicsHistory* history = nullptr) {
  JointKinematicsSample sample;
  while (joint_plot_queue.tryPop(sample)) {
    ++observation.drained;
    observation.both_arms_finite =
        observation.both_arms_finite ||
        (armJointPlotStateFinite(sample.left) &&
         armJointPlotStateFinite(sample.right));
    observation.reference_jerk_valid_seen =
        observation.reference_jerk_valid_seen ||
        (sample.left.reference_jerk_valid &&
         sample.right.reference_jerk_valid);
    observation.actual_jerk_valid_seen =
        observation.actual_jerk_valid_seen ||
        (sample.left.actual_jerk_valid && sample.right.actual_jerk_valid);
    if (history != nullptr) {
      history->push(sample);
    }
  }
}

int runHeadless(const Options& options, BoundedSpscQueue<ViewerCommand>& commands,
                LatestSnapshotExchange<ViewerSnapshot>& snapshots,
                BoundedSpscQueue<JointKinematicsSample>& joint_plot_queue,
                std::atomic<bool>& running, std::thread& control_thread) {
  ViewerSnapshot latest;
  JointPlotObservation plot_observation;
  std::uint64_t command_failures = 0U;
  std::uint64_t next_command_id = 1U;
  std::uint64_t pending_command_id = 0U;
  int pending_stage = 0;
  int completed_stage = -1;
  ViewerCommand initial = headlessCommandForStage(pending_stage, latest);
  initial.id = next_command_id++;
  if (commands.tryPush(initial)) {
    pending_command_id = initial.id;
  } else {
    ++command_failures;
  }
  const auto finish = std::chrono::steady_clock::now() +
                      std::chrono::duration<double>(options.duration_seconds);
  const auto start = std::chrono::steady_clock::now();
  while (running.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < finish) {
    (void)snapshots.tryReadLatest(latest);
    drainJointPlotQueue(joint_plot_queue, plot_observation);
    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    if (pending_command_id != 0U &&
        latest.last_processed_command_id >= pending_command_id &&
        headlessStageObserved(pending_stage, latest)) {
      completed_stage = pending_stage;
      pending_command_id = 0U;
    }

    constexpr int kFinalHeadlessStage = 12;
    const int allowed_stage = std::min(
        kFinalHeadlessStage,
        static_cast<int>(static_cast<double>(kFinalHeadlessStage + 1) *
                         elapsed / options.duration_seconds));
    if (pending_command_id == 0U && completed_stage < allowed_stage &&
        latest.sequence > 0U) {
      pending_stage = completed_stage + 1;
      ViewerCommand scripted = headlessCommandForStage(pending_stage, latest);
      scripted.id = next_command_id++;
      if (commands.tryPush(scripted)) {
        pending_command_id = scripted.id;
      } else {
        ++command_failures;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  running.store(false, std::memory_order_release);
  control_thread.join();
  (void)snapshots.tryReadLatest(latest);
  drainJointPlotQueue(joint_plot_queue, plot_observation);
  if (pending_command_id != 0U &&
      latest.last_processed_command_id >= pending_command_id &&
      headlessStageObserved(pending_stage, latest)) {
    completed_stage = pending_stage;
    pending_command_id = 0U;
  }
  std::cout << "headless_complete sequence=" << latest.sequence
            << " algorithm=" << toString(latest.algorithm)
            << " control_level=" << toString(latest.control_level)
            << " mode=" << toString(latest.mode)
            << " accepted=" << latest.accepted
            << " hand_configured=" << latest.hand_configured
            << " hand_live=" << latest.hand_live
            << " hand_stale=" << latest.hand_stale
            << " hand_sequence=" << latest.hand_sequence
            << " hand_datagrams=" << latest.hand_datagrams
            << " hand_accepted=" << latest.hand_accepted
            << " hand_malformed=" << latest.hand_malformed
            << " hand_crc_failures=" << latest.hand_crc_failures
            << " hand_reordered=" << latest.hand_reordered
            << " hand_left_q0=" << latest.left_hand_q[0]
            << " hand_right_q0=" << latest.right_hand_q[0]
            << " cycle_p99_us=" << latest.cycle_p99_us
            << " deadline_misses=" << latest.deadline_misses
            << " control_failures=" << latest.control_failures
            << " command_failures=" << command_failures
            << " completed_stage=" << completed_stage
            << " last_command_id=" << latest.last_processed_command_id
            << " snapshot_drops=" << latest.snapshot_drops
            << " telemetry_drops=" << latest.telemetry_drops
            << " joint_plot_drained=" << plot_observation.drained
            << " joint_plot_both_arms_finite="
            << plot_observation.both_arms_finite
            << " joint_plot_reference_jerk_valid_seen="
            << plot_observation.reference_jerk_valid_seen
            << " joint_plot_actual_jerk_valid_seen="
            << plot_observation.actual_jerk_valid_seen << '\n';
  return latest.sequence > 0U && latest.accepted && latest.hold_reason == HoldReason::kNone &&
                 latest.control_failures == 0U && command_failures == 0U &&
                 latest.algorithm == IkAlgorithm::kHierarchicalQp &&
                 completed_stage == 12 && pending_command_id == 0U
             ? 0
             : 2;
}

int runPicoHeadless(const Options& options,
                    LatestSnapshotExchange<ViewerSnapshot>& snapshots,
                    BoundedSpscQueue<JointKinematicsSample>& joint_plot_queue,
                    std::atomic<bool>& running,
                    std::thread& control_thread) {
  ViewerSnapshot latest;
  JointPlotObservation plot_observation;
  bool pico_live_seen = false;
  bool pico_stale_seen = false;
  bool pico_skeleton_valid_seen = false;
  bool pico_skeleton_hidden_after_stale = false;
  bool previous_stale = false;
  double stale_transition_ms = 0.0;
  const auto finish = std::chrono::steady_clock::now() +
                      std::chrono::duration<double>(options.duration_seconds);
  while (running.load(std::memory_order_acquire) && !stop_requested &&
         (options.continuous || std::chrono::steady_clock::now() < finish)) {
    drainJointPlotQueue(joint_plot_queue, plot_observation);
    if (snapshots.tryReadLatest(latest)) {
      pico_live_seen = pico_live_seen || latest.pico_live;
      pico_skeleton_valid_seen = pico_skeleton_valid_seen ||
          (latest.pico_live && latest.pico_upper_limb_skeleton.valid);
      pico_skeleton_hidden_after_stale = pico_skeleton_hidden_after_stale ||
          (latest.pico_stale && !latest.pico_upper_limb_skeleton.valid);
      if (pico_live_seen && latest.pico_stale && !previous_stale) {
        pico_stale_seen = true;
        stale_transition_ms = latest.pico_frame_age_ms;
      }
      previous_stale = latest.pico_stale;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  running.store(false, std::memory_order_release);
  control_thread.join();
  drainJointPlotQueue(joint_plot_queue, plot_observation);
  if (snapshots.tryReadLatest(latest)) {
    pico_live_seen = pico_live_seen || latest.pico_live;
    pico_skeleton_valid_seen = pico_skeleton_valid_seen ||
        (latest.pico_live && latest.pico_upper_limb_skeleton.valid);
    pico_skeleton_hidden_after_stale = pico_skeleton_hidden_after_stale ||
        (latest.pico_stale && !latest.pico_upper_limb_skeleton.valid);
    if (pico_live_seen && latest.pico_stale && !previous_stale) {
      pico_stale_seen = true;
      stale_transition_ms = latest.pico_frame_age_ms;
    }
  }

  std::cout << "pico_headless_complete sequence=" << latest.sequence
            << " algorithm=" << toString(latest.algorithm)
            << " control_level=" << toString(latest.control_level)
            << " accepted=" << latest.accepted
            << " pico_configured=" << latest.pico_configured
            << " pico_enabled=" << latest.pico_enabled
            << " arm_angle_mode="
            << toString(latest.arm_angle_reference_mode)
            << " pico_live=" << latest.pico_live
            << " pico_stale=" << latest.pico_stale
            << " pico_live_seen=" << pico_live_seen
            << " pico_stale_seen=" << pico_stale_seen
            << " pico_skeleton_valid_seen=" << pico_skeleton_valid_seen
            << " pico_skeleton_hidden_after_stale="
            << pico_skeleton_hidden_after_stale
            << " pico_stale_transition_ms=" << stale_transition_ms
            << " pico_tracking_epoch=" << latest.pico_tracking_epoch
            << " pico_sequence=" << latest.pico_sequence
            << " pico_datagrams=" << latest.pico_datagrams
            << " pico_accepted=" << latest.pico_accepted
            << " pico_malformed=" << latest.pico_malformed
            << " pico_crc_failures=" << latest.pico_crc_failures
            << " pico_reordered=" << latest.pico_reordered
            << " pico_jump_rejections=" << latest.pico_jump_rejections
            << " pico_superseded=" << latest.pico_superseded
            << " pico_epoch_resets=" << latest.pico_epoch_resets
            << " pico_resynchronizations="
            << latest.pico_resynchronizations
            << " pico_reset_applies=" << latest.pico_reset_applies
            << " pico_left_source_timestamp_ns="
            << latest.pico_left_source_timestamp_ns
            << " pico_right_source_timestamp_ns="
            << latest.pico_right_source_timestamp_ns
            << " pico_input_frequency_hz=" << latest.pico_input_frequency_hz
            << " pico_frame_age_ms=" << latest.pico_frame_age_ms
            << " pico_receive_to_control_us="
            << latest.pico_receive_to_control_us
            << " pico_bridge_to_control_us="
            << latest.pico_bridge_to_control_us
            << " hand_configured=" << latest.hand_configured
            << " hand_live=" << latest.hand_live
            << " hand_stale=" << latest.hand_stale
            << " hand_sequence=" << latest.hand_sequence
            << " hand_datagrams=" << latest.hand_datagrams
            << " hand_accepted=" << latest.hand_accepted
            << " hand_malformed=" << latest.hand_malformed
            << " hand_crc_failures=" << latest.hand_crc_failures
            << " hand_reordered=" << latest.hand_reordered
            << " hand_left_q0=" << latest.left_hand_q[0]
            << " hand_right_q0=" << latest.right_hand_q[0]
            << " left_arm_angle_source="
            << toString(latest.left_ik.arm_angle_reference_source)
            << " left_arm_angle_error_rad="
            << latest.left_ik.arm_angle_error_rad
            << " right_arm_angle_source="
            << toString(latest.right_ik.arm_angle_reference_source)
            << " right_arm_angle_error_rad="
            << latest.right_ik.arm_angle_error_rad
            << " cycle_p99_us=" << latest.cycle_p99_us
            << " deadline_misses=" << latest.deadline_misses
            << " control_failures=" << latest.control_failures
            << " snapshot_drops=" << latest.snapshot_drops
            << " telemetry_drops=" << latest.telemetry_drops
            << " joint_plot_drained=" << plot_observation.drained
            << " joint_plot_both_arms_finite="
            << plot_observation.both_arms_finite
            << " joint_plot_reference_jerk_valid_seen="
            << plot_observation.reference_jerk_valid_seen
            << " joint_plot_actual_jerk_valid_seen="
            << plot_observation.actual_jerk_valid_seen << '\n';
  return latest.sequence > 0U && latest.control_failures == 0U ? 0 : 2;
}

int runViewer(const Options& options, BoundedSpscQueue<ViewerCommand>& commands,
              LatestSnapshotExchange<ViewerSnapshot>& snapshots,
              BoundedSpscQueue<JointKinematicsSample>& joint_plot_queue,
              std::atomic<bool>& running, std::thread& control_thread) {
  if (glfwInit() == GLFW_FALSE) {
    throw std::runtime_error("GLFW initialization failed; use --headless without a display");
  }
  GLFWwindow* window = glfwCreateWindow(1280, 800, "Tianji dual-arm QP-IK V1", nullptr, nullptr);
  if (window == nullptr) {
    glfwTerminate();
    throw std::runtime_error("GLFW window creation failed");
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);
  MujocoRobot render_robot(options.model_path);
  ViewerApplication application(render_robot, commands);
  application.show_pico_skeleton = options.pico_skeleton_overlay;
  application.target_left_body = mj_name2id(render_robot.model(), mjOBJ_BODY, "target_L");
  application.target_right_body = mj_name2id(render_robot.model(), mjOBJ_BODY, "target_R");
  if (application.target_left_body < 0 || application.target_right_body < 0) {
    glfwDestroyWindow(window);
    glfwTerminate();
    throw std::runtime_error("viewer target bodies are missing");
  }
  mjv_defaultFreeCamera(render_robot.model(), &application.camera);
  mjv_defaultOption(&application.visual_options);
  application.visual_options.frame = mjFRAME_SITE;
  mjv_defaultPerturb(&application.perturb);
  mjv_defaultScene(&application.scene);
  mjr_defaultContext(&application.context);
  mjv_makeScene(render_robot.model(), &application.scene, 2000);
  mjr_makeContext(render_robot.model(), &application.context, mjFONTSCALE_150);
  glfwSetWindowUserPointer(window, &application);
  glfwSetKeyCallback(window, keyboardCallback);
  glfwSetMouseButtonCallback(window, mouseButtonCallback);
  glfwSetCursorPosCallback(window, cursorPositionCallback);
  glfwSetScrollCallback(window, scrollCallback);
  glfwSetWindowFocusCallback(window, windowFocusCallback);

  const auto start = std::chrono::steady_clock::now();
  JointPlotObservation plot_observation;
  while (glfwWindowShouldClose(window) == GLFW_FALSE) {
    if (options.simulation_recovery && stop_requested) {
      glfwSetWindowShouldClose(window, GLFW_TRUE);
      continue;
    }
    if (!running.load(std::memory_order_acquire)) {
      glfwSetWindowShouldClose(window, GLFW_TRUE);
      continue;
    }
    if (options.duration_seconds > 0.0 &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() >=
            options.duration_seconds) {
      glfwSetWindowShouldClose(window, GLFW_TRUE);
      continue;
    }
    if (snapshots.tryReadLatest(application.snapshot)) {
      renderSnapshot(application);
      if (options.simulation_recovery) {
        static const char* phases[] = {"WAITING", "TELEOP", "BRAKING", "HOMING", "HOME_REACHED", "HOLD", "FAULT"};
        const int phase = application.snapshot.simulation_phase;
        if (phase >= 0 && phase < 7) {
          const std::string title = std::string(application.snapshot.algorithm == IkAlgorithm::kPicoEeFrankaDls
              ? "Franka DLS + Ruckig SIM | " : "Ceres LM + Ruckig SIM | ") + phases[phase] +
              " | S: start | H: Home | P/Space: hold";
          glfwSetWindowTitle(window, title.c_str());
        }
      }
    }
    drainJointPlotQueue(joint_plot_queue, plot_observation,
                        &application.joint_plot_history);
    int width = 1;
    int height = 1;
    glfwGetFramebufferSize(window, &width, &height);
    application.joint_plot_layout = computeJointPlotLayout(
        width, height, application.show_joint_plots);
    const mjrRect viewport = application.joint_plot_layout.scene;
    mjv_updateScene(render_robot.model(), render_robot.data(), &application.visual_options,
                    &application.perturb, &application.camera, mjCAT_ALL, &application.scene);
    const Pose selected_target = mocapPose(application, application.selected_arm);
    appendInteractiveMarker(application.marker.geometry(selected_target),
                            application.hovered_handle,
                            application.marker.activeHandle(), &application.scene);
    if (application.show_pico_skeleton) {
      appendPicoUpperLimbSkeleton(
          application.snapshot.pico_upper_limb_skeleton, &application.scene,
          SkeletonOverlayStyle::kPico);
      appendPicoUpperLimbSkeleton(
          application.snapshot.spark_upper_limb_skeleton, &application.scene,
          SkeletonOverlayStyle::kSpark);
    }
    mjr_render(viewport, &application.scene, &application.context);
    drawOverlay(application, viewport);
    if (application.joint_plot_layout.visible) {
      application.joint_plot.update(
          application.joint_plot_history, application.joint_plot_arm,
          application.joint_plot_metric, 5.0);
      application.joint_plot.render(application.joint_plot_layout,
                                    application.context);
      char plot_status[640]{};
      std::snprintf(
          plot_status, sizeof(plot_status),
          "%s arm | %s [%s] | last 5 s\n"
          "%s\n"
          "F3 metric | F4 left/right + lock | F5 follow L/R\n"
          "control %.0f Hz | history %zu/%zu\n"
          "samples %llu | drops %llu | drained %llu\n"
          "reference derivatives: %s\n"
          "actual derivatives: %s",
          application.joint_plot_arm == ArmSide::kLeft ? "Left" : "Right",
          plotMetricName(application.joint_plot_metric),
          plotMetricUnit(application.joint_plot_metric),
          usesSharedRootDirectIk(application.snapshot.algorithm)
              ? "cyan Ruckig | blue model (diff) | red limits\nRuckig jerk = delta acceleration / dt"
              : "cyan reference | blue actual | red limits",
          200.0,
          application.joint_plot_history.size(),
          application.joint_plot_history.capacity(),
          static_cast<unsigned long long>(
              application.snapshot.joint_plot_samples),
          static_cast<unsigned long long>(
              application.snapshot.joint_plot_drops),
          static_cast<unsigned long long>(plot_observation.drained),
          application.snapshot.joint_plot_reference_derivatives_valid
              ? "valid"
              : "warming/reset",
          application.snapshot.joint_plot_actual_derivatives_valid
              ? "valid"
              : "warming/reset");
      mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT,
                  application.joint_plot_layout.status, plot_status, nullptr,
                  &application.context);
    }
    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  running.store(false, std::memory_order_release);
  control_thread.join();
  mjr_freeContext(&application.context);
  mjv_freeScene(&application.scene);
  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}

int run(int argc, char** argv) {
  Options options = parseOptions(argc, argv);
  if (options.continuous || options.simulation_recovery) {
    stop_requested = 0;
    if (std::signal(SIGINT, requestStop) == SIG_ERR ||
        std::signal(SIGTERM, requestStop) == SIG_ERR) {
      throw std::runtime_error("could not install continuous headless stop handlers");
    }
  }
  std::cout << "viewer_config=" << options.config_path << '\n';
  QpIkConfig config = loadConfig(options.config_path, ConfigConsumer::kSharedRootAware);
  if (options.control_level_override.has_value()) {
    config.control_level = *options.control_level_override;
  }
  if (options.algorithm_override.has_value()) {
    config.ik_algorithm = *options.algorithm_override;
  }
  if (!config.shared_root_profile_path.empty()) {
    if ((!usesSparkHeadroomFeedforwardVelocityQp(config.ik_algorithm) &&
         !usesSharedRootDirectIk(config.ik_algorithm)) ||
        config.control_level != ControlLevel::kVelocity || !options.pico_teleop)
      throw std::invalid_argument("shared-root requires PICO headroom/feedforward velocity mode");
    if (options.joint_command_port != 0U)
      throw std::invalid_argument("shared-root experimental mode forbids joint command export");
    const auto shared = loadSharedRootOptions(config.shared_root_profile_path);
    if (!options.model_path_explicit) options.model_path = shared.mujoco_path;
    if (sharedRootSha256File(options.model_path) != sharedRootSha256File(shared.mujoco_path))
      throw std::invalid_argument("shared-root requires its frozen palm TCP model");
    std::cout << "shared_root=experimental; device_acceptance=false; joint_export=disabled\n";
  }
  if (usesSparkGuidance(config.ik_algorithm)) {
    if (!options.pico_teleop) {
      throw std::invalid_argument(
          "Spark guidance modes require --pico-teleop");
    }
    if (config.control_level != ControlLevel::kVelocity) {
      throw std::invalid_argument(
          "Spark guidance modes require --control-level velocity");
    }
    if (config.spark_upper_qpoases.enforce_hard_jerk_bounds) {
      config.joint_limits.hard_jerk_enabled = true;
    }
  }
  if (options.model_state_only_override.has_value()) {
    config.controller.model_state_only = *options.model_state_only_override;
  }
  if (usesSharedRootDirectIk(config.ik_algorithm)) {
    if (config.shared_root_profile_path.empty() || !config.controller.model_state_only ||
        options.joint_command_port != 0U)
      throw std::invalid_argument("Ceres viewer requires enabled shared-root model-only mode; joint export forbidden");
    if (config.ik_algorithm == IkAlgorithm::kPicoEeFrankaCeresLm && !PicoEeFrankaCeresLmIk7::available())
      throw std::invalid_argument("Ceres not built: configure TIANJI_ENABLE_CERES=ON");
  }
  if (options.sim_allow_pico_jumps && !options.simulation_recovery)
    throw std::invalid_argument("--sim-allow-pico-jumps requires --simulation-recovery (DLS/Ceres model-only, no export)");
  if (options.simulation_recovery &&
      (!usesSharedRootDirectIk(config.ik_algorithm) ||
       config.shared_root_profile_path.empty() || !config.controller.model_state_only ||
       !options.pico_teleop || options.joint_command_port != 0U))
    throw std::invalid_argument("simulation recovery requires shared-root DLS/Ceres, PICO, model-only, no export");
  if (options.hand_teleop && options.pico_teleop && options.hand_port == options.pico_port)
    throw std::invalid_argument("hand and PICO ports must differ");
  if (options.simulation_recovery && options.hand_teleop && options.hand_bind != "127.0.0.1")
    throw std::invalid_argument("simulation recovery hand input must bind to 127.0.0.1");
  std::cout << (config.controller.model_state_only
                    ? "control_state_source=model_reference"
                    : "control_state_source=actual_feedback_guarded")
            << '\n';
  BoundedSpscQueue<ViewerCommand> commands(128U);
  LatestSnapshotExchange<ViewerSnapshot> snapshots(16U);
  BoundedSpscQueue<JointKinematicsSample> joint_plot_queue(4096U);
  BoundedSpscQueue<JointKinematicsSample> joint_telemetry(16384U);
  TelemetryBuffer telemetry(8192U);
  LatestSpscExchange<PicoTeleopFrame> pico_frames;
  LatestSpscExchange<WujiHandTeleopFrame> hand_frames;
  std::unique_ptr<PicoUdpReceiver> pico_receiver;
  if (options.pico_teleop) {
    PicoUdpReceiverOptions receiver_options;
    receiver_options.bind_address = options.pico_bind;
    receiver_options.port = options.pico_port;
    receiver_options.max_position_jump_m =
        config.pico_teleop.max_position_jump_m;
    receiver_options.max_orientation_jump_rad =
        config.pico_teleop.max_orientation_jump_rad;
    receiver_options.record_path = options.pico_record_path;
    receiver_options.reject_pose_jumps = !options.sim_allow_pico_jumps;
    if (options.sim_allow_pico_jumps)
      std::cerr << "WARNING: simulation-only PICO pose jump rejection DISABLED; freshness and motion limits remain enabled" << std::endl;
    pico_receiver = std::make_unique<PicoUdpReceiver>(
        std::move(receiver_options), pico_frames);
  }
  MujocoRobot control_robot(options.model_path);
  std::unique_ptr<WujiHandUdpReceiver> hand_receiver;
  if (options.hand_teleop) {
    if (!control_robot.hasHandMappings()) {
      throw std::invalid_argument(
          "--hand-teleop requires a combined Wuji Hand 2 model");
    }
    WujiHandUdpReceiverOptions receiver_options;
    receiver_options.bind_address = options.hand_bind;
    receiver_options.port = options.hand_port;
    receiver_options.stale_timeout_seconds =
        options.hand_stale_timeout_seconds;
    hand_receiver = std::make_unique<WujiHandUdpReceiver>(
        std::move(receiver_options), hand_frames);
  }
  std::unique_ptr<JointCommandExporter> joint_command_exporter;
  if (options.joint_command_port != 0U) {
    joint_command_exporter = std::make_unique<JointCommandExporter>(
        options.joint_command_host, options.joint_command_port);
  }
  std::atomic<bool> running{true};
  std::atomic<bool> control_finished{false};
  std::exception_ptr control_error;
  std::exception_ptr telemetry_error;
  std::exception_ptr joint_telemetry_error;
  std::thread telemetry_thread;
  std::thread joint_telemetry_thread;
  std::thread control_thread;
  const ArmAngleReferenceMode initial_arm_angle_reference_mode =
      options.pico_teleop ? ArmAngleReferenceMode::kOutwardOnly
                          : options.arm_angle_reference_mode;
  try {
    if (pico_receiver != nullptr) {
      pico_receiver->start();
      std::cout << "pico_udp_bind=" << options.pico_bind << ':'
                << pico_receiver->boundPort() << '\n';
      if (!options.pico_record_path.empty()) {
        std::cout << "pico_record_path=" << options.pico_record_path << '\n';
      }
    }
    if (hand_receiver != nullptr) {
      hand_receiver->start();
      std::cout << "hand_udp_bind=" << options.hand_bind << ':'
                << hand_receiver->boundPort() << '\n';
    }
    if (joint_command_exporter != nullptr) {
      std::cout << "joint_command_udp=" << options.joint_command_host << ':'
                << options.joint_command_port
                << " joint_command_packet_size=" << kJointCommandPacketSize
                << " joint_command_state=ready" << std::endl;
    }
    if (!options.telemetry_path.empty()) {
      telemetry_thread = std::thread([&] {
        try {
          writeTelemetryCsv(options.telemetry_path, telemetry, control_finished);
        } catch (...) {
          telemetry_error = std::current_exception();
          running.store(false, std::memory_order_release);
        }
      });
    }
    if (!options.joint_telemetry_path.empty()) {
      joint_telemetry_thread = std::thread([&] {
        try {
          writeJointTelemetryCsv(options.joint_telemetry_path,
                                 joint_telemetry, control_finished);
        } catch (...) {
          joint_telemetry_error = std::current_exception();
          running.store(false, std::memory_order_release);
        }
      });
    }
    control_thread = std::thread([&] {
      try {
        controlLoop(control_robot, config, commands, snapshots,
                    joint_plot_queue,
                    options.joint_telemetry_path.empty()
                        ? nullptr
                        : &joint_telemetry,
                    options.telemetry_path.empty() ? nullptr : &telemetry,
                    options.pico_teleop ? &pico_frames : nullptr,
                    pico_receiver.get(),
                    options.hand_teleop ? &hand_frames : nullptr,
                    hand_receiver.get(),
                    joint_command_exporter.get(),
                    options.pico_teleop,
                    options.simulation_recovery,
                    initial_arm_angle_reference_mode, running);
      } catch (...) {
        control_error = std::current_exception();
        running.store(false, std::memory_order_release);
      }
      control_finished.store(true, std::memory_order_release);
    });
  } catch (...) {
    running.store(false, std::memory_order_release);
    control_finished.store(true, std::memory_order_release);
    if (pico_receiver != nullptr) {
      pico_receiver->stop();
    }
    if (hand_receiver != nullptr) {
      hand_receiver->stop();
    }
    if (control_thread.joinable()) {
      control_thread.join();
    }
    if (telemetry_thread.joinable()) {
      telemetry_thread.join();
    }
    if (joint_telemetry_thread.joinable()) {
      joint_telemetry_thread.join();
    }
    throw;
  }

  int result = 0;
  try {
    if (options.headless && (options.pico_teleop || options.continuous)) {
      result = runPicoHeadless(options, snapshots, joint_plot_queue, running,
                               control_thread);
    } else if (options.headless) {
      result = runHeadless(options, commands, snapshots, joint_plot_queue,
                           running, control_thread);
    } else {
      result = runViewer(options, commands, snapshots, joint_plot_queue,
                         running, control_thread);
    }
  } catch (...) {
    running.store(false, std::memory_order_release);
    if (pico_receiver != nullptr) {
      pico_receiver->stop();
    }
    if (hand_receiver != nullptr) {
      hand_receiver->stop();
    }
    if (control_thread.joinable()) {
      control_thread.join();
    }
    control_finished.store(true, std::memory_order_release);
    if (telemetry_thread.joinable()) {
      telemetry_thread.join();
    }
    if (joint_telemetry_thread.joinable()) {
      joint_telemetry_thread.join();
    }
    throw;
  }
  if (pico_receiver != nullptr) {
    pico_receiver->stop();
    if (!options.pico_record_path.empty()) {
      const PicoReceiverStats recording = pico_receiver->stats();
      std::cout << "pico_record_complete pico_record_state="
                << picoTraceRecorderStateName(recording.recording_state)
                << " pico_recorded_packets=" << recording.recorded_packets
                << " pico_record_packet_size="
                << recording.recording_packet_size
                << '\n';
    }
  }
  if (hand_receiver != nullptr) {
    hand_receiver->stop();
    const WujiHandReceiverStats hand_stats = hand_receiver->stats();
    std::cout << "hand_record_complete hand_udp_state=stopped"
              << " hand_datagrams=" << hand_stats.datagrams
              << " hand_accepted=" << hand_stats.accepted
              << " hand_malformed=" << hand_stats.malformed
              << " hand_crc_failures=" << hand_stats.crc_failures
              << " hand_reordered=" << hand_stats.reordered << '\n';
  }
  if (telemetry_thread.joinable()) {
    telemetry_thread.join();
  }
  if (joint_telemetry_thread.joinable()) {
    joint_telemetry_thread.join();
  }
  if (control_error != nullptr) {
    std::rethrow_exception(control_error);
  }
  if (telemetry_error != nullptr) {
    std::rethrow_exception(telemetry_error);
  }
  if (joint_telemetry_error != nullptr) {
    std::rethrow_exception(joint_telemetry_error);
  }
  return result;
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
