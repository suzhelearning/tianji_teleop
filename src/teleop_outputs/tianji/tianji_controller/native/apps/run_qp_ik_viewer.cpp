#include "tianji_qp_ik/controller.hpp"
#include "tianji_qp_ik/episode_relative.hpp"
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
#include "tianji_qp_ik/shared_root_guidance.hpp"
#include "tianji_qp_ik/target_manager.hpp"
#include "tianji_qp_ik/telemetry.hpp"
#include "tianji_qp_ik/wuji_hand_udp_receiver.hpp"
#include "tianji_qp_ik/resource_paths.hpp"
#ifdef TIANJI_ROS_TRANSPORT
#include "tianji_qp_ik/arm_ros_transport.hpp"
#include <rclcpp/rclcpp.hpp>
#endif

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <charconv>
#include <filesystem>
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
#include <vector>

namespace tianji_qp_ik {
namespace {

volatile std::sig_atomic_t stop_requested = 0;

void requestStop(int) { stop_requested = 1; }


static std::string defaultModelPath() {
  return runtimePackageResource("tianji_description", "models/marvin_m6_wuji2.xml").string();
}


struct Options {
  std::string config_path;
  std::string model_path;
  bool model_path_explicit{false};
  std::string telemetry_path;
  std::string joint_telemetry_path;
  std::string joint_command_host{"127.0.0.1"};
  std::uint16_t joint_command_port{0U};
#ifdef TIANJI_ROS_TRANSPORT
  std::string pico_topic{"/pico/arm_input"};
  std::string joint_target_topic;
  std::string hand_source{"manus"};
  std::string left_hand_topic{"/wuji/left_hand/joint_commands"};
  std::string right_hand_topic{"/wuji/right_hand/joint_commands"};
#endif
  bool help_requested{false};
  bool continuous{false};
  bool headless{false};
  bool external_display{false};
  bool continuous_follow{false};
  bool franka_dls_executor{false};
  bool simulation_recovery{false};
  bool episode_relative{false};
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
  std::optional<bool> model_state_only_override;
  bool handRosEnabled() const noexcept {
#ifdef TIANJI_ROS_TRANSPORT
    return hand_teleop && hand_source == "manus";
#else
    return false;
#endif
  }
  bool handUdpEnabled() const noexcept { return hand_teleop && !handRosEnabled(); }
  bool outputEnabled() const noexcept {
#ifdef TIANJI_ROS_TRANSPORT
    return !joint_target_topic.empty();
#else
    return joint_command_port != 0U;
#endif
  }
};

struct ViewerApplication {
  ViewerApplication(MujocoRobot& robot_in, BoundedSpscQueue<ViewerCommand>& commands_in)
      : robot(robot_in), commands(commands_in) {}

  MujocoRobot& robot;
  BoundedSpscQueue<ViewerCommand>& commands;
  bool external_display{false};
  bool continuous_follow{false};
  bool external_quit_sent{false};
  std::string external_status{"Waiting for simulation joint frames"};
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
#ifdef TIANJI_ROS_TRANSPORT
      std::cout << "Usage: tianji_arm_ros [--pico-topic NAME] [--joint-target-topic NAME (empty=disabled)] "
                   "[--episode-relative (data capture reference service)] "
#else
      std::cout << "Usage: tianji_qp_ik_viewer "
#endif
                   "[--config FILE] [--model FILE] "
                   "[--ik-backend franka-dls] [--headless] [--duration SECONDS|--continuous] "
                   "[--external-display (private stdin simulation joint display)] "
                   "[--continuous-follow (requires --external-display)] "
                   "[--telemetry FILE] [--joint-telemetry FILE] "
#ifndef TIANJI_ROS_TRANSPORT
                   "[--joint-command-host LOOPBACK_IPV4] [--joint-command-port PORT (0=disabled)] "
#endif
                   "[--franka-dls-executor (restricted DLS/Ruckig export)] "
                   "[--pico-teleop|--no-pico-teleop] "
                   "[--simulation-recovery (DLS model-only S/H/P gate)] "
                   "[--sim-allow-pico-jumps (requires simulation recovery)] "
                   "[--pico-skeleton-overlay|--no-pico-skeleton-overlay] "
#ifndef TIANJI_ROS_TRANSPORT
                   "[--pico-bind IPV4] [--pico-port PORT] [--pico-record FILE.tjvr] "
#endif
                   "[--hand-teleop|--no-hand-teleop] "
#ifdef TIANJI_ROS_TRANSPORT
                   "[--hand-source manus|exoskeleton (default manus)] "
                   "[--left-hand-topic NAME] [--right-hand-topic NAME] "
#endif
                   "[--hand-bind IPV4] [--hand-port PORT] "
                   "[--hand-stale-timeout SECONDS] "
                   "[--algorithm pico_ee_franka_dls] "
                   "[--model-state-only|--actual-feedback-control]\n";
      options.help_requested = true;
      return options;
    }
#ifdef TIANJI_ROS_TRANSPORT
    if (argument == "--episode-relative") {
      options.episode_relative = true;
      continue;
    }
#endif
    if (argument == "--external-display") {
      options.external_display = true;
      continue;
    }
    if (argument == "--continuous-follow") {
      options.continuous_follow = true;
      continue;
    }
    if (argument == "--franka-dls-executor") {
      options.franka_dls_executor = true;
      continue;
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
#ifdef TIANJI_ROS_TRANSPORT
    } else if (argument == "--pico-topic") {
      options.pico_topic = value;
    } else if (argument == "--joint-target-topic") {
      options.joint_target_topic = value;
#else
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
#endif
#ifdef TIANJI_ROS_TRANSPORT
    } else if (argument == "--hand-source") {
      if (value != "manus" && value != "exoskeleton")
        throw std::invalid_argument("--hand-source must be manus or exoskeleton");
      options.hand_source = value;
    } else if (argument == "--left-hand-topic") {
      options.left_hand_topic = value;
    } else if (argument == "--right-hand-topic") {
      options.right_hand_topic = value;
#endif
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
    } else if (argument == "--algorithm") {
      if (value != "pico_ee_franka_dls")
        throw std::invalid_argument("--algorithm must be pico_ee_franka_dls");
    } else if (argument == "--ik-backend") {
      if (value != "franka-dls")
        throw std::invalid_argument("--ik-backend must be franka-dls");
    } else {
      throw std::invalid_argument("unknown option: " + argument);
    }
  }
  if (options.continuous_follow && !options.external_display) {
    throw std::invalid_argument("--continuous-follow requires --external-display");
  }
  if (options.external_display) {
    if (options.headless || options.continuous || options.simulation_recovery ||
        options.franka_dls_executor || options.episode_relative || options.outputEnabled() ||
        !options.telemetry_path.empty() || !options.joint_telemetry_path.empty() ||
        !options.pico_record_path.empty() || options.duration_seconds != 0.0) {
      throw std::invalid_argument(
          "--external-display is a window-only private display; no control, "
          "recording, export, headless, or duration options");
    }
    options.pico_teleop = false;
    options.hand_teleop = false;
    options.pico_skeleton_overlay = false;
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
#ifndef TIANJI_ROS_TRANSPORT
  if (inet_pton(AF_INET, options.pico_bind.c_str(), &parsed_address) != 1) {
    throw std::invalid_argument("--pico-bind must be a valid IPv4 address");
  }
#else
  if (options.pico_teleop && options.pico_topic.empty())
    throw std::invalid_argument("--pico-topic must not be empty");
  if (options.outputEnabled() && !options.franka_dls_executor)
    throw std::invalid_argument("--joint-target-topic requires --franka-dls-executor");
  if (options.handRosEnabled() &&
      (options.left_hand_topic.empty() || options.right_hand_topic.empty() ||
       options.left_hand_topic == options.right_hand_topic))
    throw std::invalid_argument("Manus requires distinct nonempty hand topics");
#endif
  if (options.handUdpEnabled() && inet_pton(AF_INET, options.hand_bind.c_str(), &parsed_address) != 1) {
    throw std::invalid_argument("--hand-bind must be a valid IPv4 address");
  }
#ifndef TIANJI_ROS_TRANSPORT
  if (inet_pton(AF_INET, options.joint_command_host.c_str(), &parsed_address) != 1 ||
      (ntohl(parsed_address.s_addr) >> 24U) != 127U) {
    throw std::invalid_argument("--joint-command-host must be a loopback IPv4 address");
  }
#endif
  if (options.franka_dls_executor) {
    if (!options.headless || !options.continuous ||
        !options.pico_teleop || !options.outputEnabled() ||
#ifndef TIANJI_ROS_TRANSPORT
        !options.hand_teleop ||
#endif
        options.simulation_recovery || options.sim_allow_pico_jumps) {
      throw std::invalid_argument(
          "--franka-dls-executor requires --headless --continuous, PICO, "
          "joint command output, and no simulation switches");
    }
#ifdef TIANJI_ROS_TRANSPORT
    if (options.handUdpEnabled() &&
        (inet_pton(AF_INET, options.hand_bind.c_str(), &parsed_address) != 1 ||
         (ntohl(parsed_address.s_addr) >> 24U) != 127U))
      throw std::invalid_argument("--franka-dls-executor requires loopback hand input");
#else
    for (const auto* address : {&options.pico_bind, &options.hand_bind}) {
      if (inet_pton(AF_INET, address->c_str(), &parsed_address) != 1 ||
          (ntohl(parsed_address.s_addr) >> 24U) != 127U) {
        throw std::invalid_argument(
            "--franka-dls-executor requires loopback IPv4 PICO and hand inputs");
      }
    }
#endif
  }
#ifdef TIANJI_ROS_TRANSPORT
  if (options.handUdpEnabled() && (options.hand_port == 15000U || options.hand_port == 17000U))
    throw std::invalid_argument("retired arm UDP ports are not available to the ROS arm core");
#endif
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
    const ArmLimits limits = effectiveArmLimits(config.joint_limits, robot.mapping(side).limits, side);
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

bool resumeFromPause(
    bool& paused, DualArmController& controller) {
  if (!paused) {
    return true;
  }
  const bool synchronized = controller.synchronizeReferencesToActual();
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
                    bool& paused,
                    bool& pico_paused,
                    std::unique_ptr<DualArmController>& controller) {
  switch (command.type) {
    case ViewerCommandType::kSimulationStart:
    case ViewerCommandType::kSimulationHome:
    case ViewerCommandType::kSimulationHold:
      break; // Owned exclusively by the opt-in simulation state machine.
    case ViewerCommandType::kSetMode:
      if (!resumeFromPause(paused, *controller)) {
        break;
      }
      targets.setMode(command.mode, target_time);
      break;
    case ViewerCommandType::kResetNominal:
      setInitialConfiguration(robot, config);
      targets = TargetManager(config, currentTargets(robot));
      targets.setMode(TargetMode::kHold, target_time);
      controller = makeController(robot, config);
      paused = false;
      break;
    case ViewerCommandType::kSetManualTarget:
      if (pico_session.enabled()) {
        break;
      }
      if (!resumeFromPause(paused, *controller)) {
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
        if (!resumeFromPause(paused, *controller)) {
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
  }
}

void setConfiguredPlotPositionBounds(const ArmLimits& limits,
                             const QpIkConfig& config,
                             JointKinematicsBounds& bounds) {
  const double margin = config.joint_limits.margin_rad;
  bounds.position_lower = limits.lower_position.array() + margin;
  bounds.position_upper = limits.upper_position.array() - margin;
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
                 PicoInputReceiver* pico_receiver,
                 LatestSpscExchange<WujiHandTeleopFrame>* hand_frames,
                 WujiHandUdpReceiver* hand_receiver,
#ifdef TIANJI_ROS_TRANSPORT
                 ArmRosTransport* hand_ros_transport,
#endif
                 JointCommandSink* joint_command_exporter,
                 bool pico_initially_enabled,
                 bool simulation_recovery,
                 bool episode_relative,
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
    recovery.emplace(config,std::array<ArmLimits,2>{controller->motionLimits(ArmSide::kLeft),
        controller->motionLimits(ArmSide::kRight)},motionPair(),dt);
  std::optional<EpisodeRelativeSession> episode;
  if (episode_relative) episode.emplace(config.cartesian_servo.target_timeout_seconds);
#ifdef TIANJI_ROS_TRANSPORT
  auto* reference_transport = episode ? dynamic_cast<ArmRosTransport*>(pico_receiver) : nullptr;
  if (episode && !reference_transport) throw std::invalid_argument("relative mode requires ROS transport");
#endif
  int reported_phase=-1;
  std::unique_ptr<SharedRootGuidance> shared_root_guidance;
  const bool shared_root_mode = !config.shared_root_profile_path.empty();
  std::optional<SharedRootOptions> shared_root_options;
  if (shared_root_mode) {
    shared_root_options = loadSharedRootOptions(config.shared_root_profile_path);
    if (!shared_root_options->enabled)
      throw std::runtime_error("shared-root profile changed after startup validation");
  }
  shared_root_guidance = std::make_unique<SharedRootGuidance>(
      robot, config, shared_root_options->urdf_path, &*shared_root_options);
  DualArmTargets last_shared_root_targets = initial_targets;
  SharedRootGuidanceDiagnostics shared_root_diagnostics;
  bool pico_paused = false;
  const bool pico_configured = pico_frames != nullptr && pico_receiver != nullptr;
  const bool hand_configured = hand_frames != nullptr;
  PicoTeleopSession pico_session(
      config.cartesian_servo.target_timeout_seconds);
  pico_session.setEnabled(pico_configured && pico_initially_enabled);
  PicoUpperLimbSkeleton latest_pico_upper_limb_skeleton;
  std::uint64_t pico_applied_epoch = 0U;
  std::uint64_t pico_applied_sequence = 0U;
  std::uint64_t pico_reset_applies = 0U;
  std::uint64_t joint_command_epoch = 0U;
  std::uint64_t joint_command_sequence =
      joint_command_exporter != nullptr ? static_cast<std::uint64_t>(monotonicNowNs()) : 0U;
  JointCommandFrame output;
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
  std::int64_t pico_applied_receive_monotonic_ns = 0;
  const auto shared_root_input_fresh = [&](std::int64_t now) {
    return !shared_root_options ||
        (pico_applied_receive_monotonic_ns > 0 &&
         now >= pico_applied_receive_monotonic_ns &&
         static_cast<double>(now - pico_applied_receive_monotonic_ns) * 1.0e-9 <
             shared_root_options->continuity.freshness_s);
  };
  const auto reset_guidance = [&](const ArmMotionState& left,
                                 const ArmMotionState& right) {
    return shared_root_guidance->resetMappingSession(left, right);
  };
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
      if (episode) {
        // GUI/controller buttons cannot grant or rebaseline data motion authority.
        episode->fault();
        last_processed_command_id = std::max(last_processed_command_id, command.id);
        continue;
      }
      if(recovery) {
        bool changed=false;
        if(command.type==ViewerCommandType::kSimulationStart) {
          changed=recovery->start(pico_session.freshness(monotonic_now_ns).live,motionPair());
          std::cout << "DLS_SIM: S "
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
          if(!shared_root_guidance->resetMappingSession(controller->referenceState(ArmSide::kLeft),
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
      const bool previous_paused = paused;
      const bool previous_pico_paused = pico_paused;
      processCommand(command, robot, config, target_time, targets,
                     pico_session, pico_configured, paused, pico_paused, controller);
      if (shared_root_mode && command.type == ViewerCommandType::kResetNominal) {
        pico_session.setEnabled(false);
        pico_paused = true; // Reset/Home never automatically grants takeover.
      }
      if (shared_root_mode && command.type != ViewerCommandType::kResetNominal &&
          (paused != previous_paused || pico_paused != previous_pico_paused)) {
        // Process revocation even when pause/resume commands share one tick.
        if (!reset_guidance(controller->referenceState(ArmSide::kLeft),
                            controller->referenceState(ArmSide::kRight)))
          throw std::runtime_error("shared-root pause/rearm reset failed");
      }
      hand_reset_requested = hand_reset_requested ||
                             pico_paused != previous_pico_paused;
      if (paused != previous_paused ||
          command.type == ViewerCommandType::kResetNominal) {
        plot_reset_requested = true;
      }
      if (command.type == ViewerCommandType::kResetNominal) {
        robot.forward();
        last_shared_root_targets = currentTargets(robot);
        shared_root_diagnostics = {};
        if (!reset_guidance(controller->referenceState(ArmSide::kLeft),
                            controller->referenceState(ArmSide::kRight))) {
          throw std::runtime_error("failed to reset shared-root guidance to initial posture");
        }
      }
      last_processed_command_id = std::max(last_processed_command_id, command.id);
    }

    PicoTeleopFrame pico_frame;
    if (pico_frames != nullptr && pico_frames->tryReadLatest(pico_frame)) {
      joint_command_stream_reset =
          (joint_command_epoch != 0U &&
           joint_command_epoch != pico_frame.tracking_epoch) ||
          pico_frame.stream_discontinuity ||
          pico_frame.resynchronization_generation !=
              joint_command_resynchronization_generation;
      joint_command_epoch = pico_frame.tracking_epoch;
      joint_command_resynchronization_generation =
          pico_frame.resynchronization_generation;
      if (episode) {
        const bool accepted = episode->observe(pico_frame, monotonic_now_ns);
        if (accepted) {
          pico_session.commitApplied(pico_frame);
          pico_applied_bridge_send_monotonic_ns = pico_frame.bridge_send_monotonic_ns;
          pico_applied_receive_monotonic_ns = pico_frame.receive_monotonic_ns;
          pico_applied_epoch = pico_frame.tracking_epoch;
          pico_applied_sequence = pico_frame.sequence;
          pico_left_source_timestamp_ns = pico_right_source_timestamp_ns = pico_frame.source_timestamp_ns;
          latest_pico_upper_limb_skeleton = pico_frame.upper_limb_skeleton;
        } else pico_session.invalidate();
      } else {
      if (!pico_frame.valid) {
        pico_session.invalidate();
        pico_applied_bridge_send_monotonic_ns = 0;
        pico_applied_receive_monotonic_ns = 0;
        joint_command_stream_reset = true;
        if (recovery && recovery->teleop()) recovery->stop(motionPair());
      } else {
      const PicoTeleopButtonAction button_action =
          pico_session.observeButton(pico_frame);
      joint_command_stream_reset = joint_command_stream_reset ||
          button_action != PicoTeleopButtonAction::kNone;
      if(recovery&&button_action==PicoTeleopButtonAction::kPause)
        recovery->stop(motionPair());
      if (button_action == PicoTeleopButtonAction::kPause) {
        pico_paused = true;
        targets.setMode(TargetMode::kHold, target_time);
      } else {
        if (button_action == PicoTeleopButtonAction::kResume) {
          pico_paused = false;
        }
        const PicoTeleopClassification classification =
            pico_session.classify(pico_frame, monotonic_now_ns);
        if (classification.action == PicoTeleopAction::kApply ||
            classification.action == PicoTeleopAction::kResetEpochAndApply) {
          const bool reset_epoch =
              classification.action == PicoTeleopAction::kResetEpochAndApply;
          if(recovery&&recovery->teleop()&&reset_epoch&&pico_applied_epoch!=0)
            recovery->stop(motionPair()); // Reset never grants automatic re-entry.
          if (reset_epoch &&
              !reset_guidance(controller->referenceState(ArmSide::kLeft),
                              controller->referenceState(ArmSide::kRight))) {
            throw std::runtime_error("PICO epoch guidance reset failed");
          }
          const bool target_accepted =
              shared_root_guidance->updateSharedRootFrame(pico_frame, monotonic_now_ns);
          if (target_accepted) {
            if (reset_epoch && !config.controller.model_state_only) {
              const bool synchronized = controller->synchronizeReferencesToActual();
              if (!synchronized) {
                continue;
              }
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
            pico_applied_receive_monotonic_ns = pico_frame.receive_monotonic_ns;
            latest_pico_upper_limb_skeleton = pico_frame.upper_limb_skeleton;
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
      }
    }
    if (episode) {
      episode->tick(monotonic_now_ns);
#ifdef TIANJI_ROS_TRANSPORT
      if (reference_transport->referenceFaulted()) episode->fault();
      TeleopReferenceCommand request;
      while (reference_transport->takeReferenceCommand(request))
        reference_transport->completeReferenceCommand(
            episode->command(request, *controller, monotonicNowNs()));
#endif
    }

    const PicoTeleopFreshness pico_freshness =
        pico_session.freshness(monotonic_now_ns);
    if(recovery&&recovery->teleop()&&!pico_freshness.live)
      recovery->stop(motionPair());
    const PicoReceiverStats pico_stats =
        pico_receiver != nullptr ? pico_receiver->stats() : PicoReceiverStats{};

    const ArmMotionState left_mapping_model = controller->referenceState(ArmSide::kLeft);
    const ArmMotionState right_mapping_model = controller->referenceState(ArmSide::kRight);
    if (!episode) {
      shared_root_diagnostics = shared_root_guidance->stepSharedRoot(
          left_mapping_model, right_mapping_model, dt, monotonic_now_ns,
          !paused && !pico_paused && pico_session.enabled() &&
              (!recovery || recovery->teleop()),
          left_mapping_model.q.allFinite() && left_mapping_model.qdot.allFinite() &&
          left_mapping_model.qddot.allFinite() && right_mapping_model.q.allFinite() &&
          right_mapping_model.qdot.allFinite() && right_mapping_model.qddot.allFinite());
    }
    if (shared_root_diagnostics.accepted) {
      last_shared_root_targets = shared_root_diagnostics.cartesian_targets;
    }
    DualArmTargets desired = last_shared_root_targets;
    desired.left_stale = !pico_freshness.live || !shared_root_diagnostics.accepted ||
                         !shared_root_diagnostics.target_valid;
    desired.right_stale = desired.left_stale;
    if (episode) {
      desired = episode->targets();
      desired.left_stale = desired.right_stale = !episode->active();
    }
    DualArmReferences references = directReferences(desired);
    if (shared_root_mode && desired.left_stale) {
      // Cached references may have been fresh when accepted. They must not
      // mask the current mapping's stale/invalid/disabled state.
      references.left.stale = references.right.stale = true;
      references.left.twist.setZero();references.right.twist.setZero();
    }
    ControllerDiagnostics diagnostics;
    bool recovery_plot_valid = false;
    const bool recovery_home_sample = recovery &&
        recovery->phase() == SimulationRecovery::Phase::kHoming;
    if (episode && !episode->active()) {
      // Preparing, cancelled and preflight states hold q without running IK/OTG.
      // In particular waiting for recorder ACK must never advance a trajectory.
      robot.forward();
      diagnostics.accepted = episode->ready(monotonic_now_ns);
      diagnostics.hold_reason = diagnostics.accepted ? HoldReason::kNone : HoldReason::kSolverFailure;
      for (auto side : {ArmSide::kLeft, ArmSide::kRight}) {
        auto& arm = side == ArmSide::kLeft ? diagnostics.left : diagnostics.right;
        arm.accepted = diagnostics.accepted; arm.hold_reason = diagnostics.hold_reason;
        arm.q_ref = controller->reference(side); arm.q_actual = robot.armPosition(side);
        arm.current = robot.armKinematicsAt(side, arm.q_ref).tcp_pose;
        arm.tcp_actual = robot.tcpPose(side); arm.target = arm.current;
        arm.reference.pose = arm.current; arm.ik.status = SolverStatus::kSolved;
      }
    } else if(recovery&&!recovery->teleop()) {
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
      if (shared_root_diagnostics.reference_reset_required) controller->resetSolvers();
      diagnostics = controller->step(references, dt);
      const bool accepted = diagnostics.accepted;
      const bool bounded_input_hold =
          diagnostics.hold_reason == HoldReason::kNone &&
          diagnostics.left.dls_posture_ruckig_accepted &&
          diagnostics.right.dls_posture_ruckig_accepted;
      if (!accepted && !bounded_input_hold) {
        ++control_failures;
        if (episode) episode->fault();
        if(recovery&&pico_freshness.live&&!desired.left_stale&&!desired.right_stale)
          recovery->stop(motionPair());
      }
      if (shared_root_mode && !episode) {
        const auto commit_time_ns = monotonicNowNs();
        (void)shared_root_guidance->confirmSharedRootReference(
            shared_root_diagnostics.shared_root_cycle,
            accepted && diagnostics.left.accepted && diagnostics.right.accepted &&
            pico_session.freshness(commit_time_ns).live &&
            shared_root_input_fresh(commit_time_ns) &&
            (joint_command_exporter == nullptr ||
             jointCommandPicoBridgeFresh(pico_applied_bridge_send_monotonic_ns,
                                         commit_time_ns)) &&
            shared_root_diagnostics.target_valid);
      }
    } else {
      robot.forward();
      diagnostics.left.q_ref = controller->reference(ArmSide::kLeft);
      diagnostics.right.q_ref = controller->reference(ArmSide::kRight);
      diagnostics.left.q_actual = robot.armPosition(ArmSide::kLeft);
      diagnostics.right.q_actual = robot.armPosition(ArmSide::kRight);
      diagnostics.left.tcp_actual = robot.tcpPose(ArmSide::kLeft);
      diagnostics.right.tcp_actual = robot.tcpPose(ArmSide::kRight);
      diagnostics.left.current = robot.armKinematicsAt(
          ArmSide::kLeft, diagnostics.left.q_ref).tcp_pose;
      diagnostics.right.current = robot.armKinematicsAt(
          ArmSide::kRight, diagnostics.right.q_ref).tcp_pose;
      diagnostics.left.reference = references.left;
      diagnostics.right.reference = references.right;
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
        std::cout << "DLS_SIM: "
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
    snapshot.targets = desired;
    snapshot.algorithm = IkAlgorithm::kPicoEeFrankaDls;
    snapshot.mode = targets.mode();
    snapshot.paused = paused;
    snapshot.accepted = diagnostics.accepted;
    snapshot.hold_reason = diagnostics.hold_reason;
    snapshot.left_target_stale = desired.left_stale;
    snapshot.right_target_stale = desired.right_stale;
    snapshot.hand_configured = hand_configured;
    snapshot.hand_stale = hand_stats.stale;
    snapshot.hand_live = hand_configured && !hand_stats.stale;
    snapshot.hand_left_stale = !left_hand_freshness.live(hand_sample_time_ns);
    snapshot.hand_right_stale = !right_hand_freshness.live(hand_sample_time_ns);
    snapshot.hand_sequence = hand_stats.sequence;
    snapshot.hand_datagrams = hand_stats.datagrams;
    snapshot.hand_accepted = hand_stats.accepted;
    snapshot.hand_malformed = hand_stats.malformed;
    snapshot.hand_crc_failures = hand_stats.crc_failures;
    snapshot.hand_reordered = hand_stats.reordered;
#ifdef TIANJI_ROS_TRANSPORT
    if (hand_ros_transport) {
      const auto ros_stats = hand_ros_transport->handStats();
      snapshot.hand_ros = true;
      snapshot.hand_left_stale = ros_stats.left_stale;
      snapshot.hand_right_stale = ros_stats.right_stale;
      snapshot.hand_stale = ros_stats.left_stale && ros_stats.right_stale;
      snapshot.hand_live = !snapshot.hand_stale;
      snapshot.hand_sequence = ros_stats.sequence;
      snapshot.hand_messages = ros_stats.received;
      snapshot.hand_accepted = ros_stats.accepted;
      snapshot.hand_rejected = ros_stats.rejected;
    }
#endif
    snapshot.left_position_error = diagnostics.left.pose_error.head<3>().norm();
    snapshot.left_orientation_error = diagnostics.left.pose_error.tail<3>().norm();
    snapshot.right_position_error = diagnostics.right.pose_error.head<3>().norm();
    snapshot.right_orientation_error = diagnostics.right.pose_error.tail<3>().norm();
    snapshot.left_solver_status = diagnostics.left.ik.status;
    snapshot.right_solver_status = diagnostics.right.ik.status;
    snapshot.left_ik.accepted = diagnostics.left.accepted;
    snapshot.left_ik.hold_reason = diagnostics.left.hold_reason;
    snapshot.left_ik.position_error = snapshot.left_position_error;
    snapshot.left_ik.orientation_error = snapshot.left_orientation_error;
    snapshot.left_ik.actual_position_error =
        diagnostics.left.actual_pose_error.head<3>().norm();
    snapshot.left_ik.actual_orientation_error =
        diagnostics.left.actual_pose_error.tail<3>().norm();
    snapshot.left_ik.solve_time_us = diagnostics.left.ik.solve_time_us;
    snapshot.left_ik.iterations = diagnostics.left.ik.iterations;
    snapshot.left_ik.status = diagnostics.left.ik.status;
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
    snapshot.right_ik.accepted = diagnostics.right.accepted;
    snapshot.right_ik.hold_reason = diagnostics.right.hold_reason;
    snapshot.right_ik.position_error = snapshot.right_position_error;
    snapshot.right_ik.orientation_error = snapshot.right_orientation_error;
    snapshot.right_ik.actual_position_error =
        diagnostics.right.actual_pose_error.head<3>().norm();
    snapshot.right_ik.actual_orientation_error =
        diagnostics.right.actual_pose_error.tail<3>().norm();
    snapshot.right_ik.solve_time_us = diagnostics.right.ik.solve_time_us;
    snapshot.right_ik.iterations = diagnostics.right.ik.iterations;
    snapshot.right_ik.status = diagnostics.right.ik.status;
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

    JointKinematicsSample plot_sample;
    plot_sample.sequence = sequence;
    plot_sample.time_seconds = control_time;
    plot_sample.reset = plot_reset_requested;
    setConfiguredPlotPositionBounds(controller->motionLimits(ArmSide::kLeft), config,
                            plot_sample.left.bounds);
    setConfiguredPlotPositionBounds(controller->motionLimits(ArmSide::kRight), config,
                            plot_sample.right.bounds);

    const ArmMotionState left_reference_state = controller->referenceState(ArmSide::kLeft);
    const ArmMotionState right_reference_state = controller->referenceState(ArmSide::kRight);
    plot_sample.left.reference.position = left_reference_state.q;
    plot_sample.left.reference.velocity = left_reference_state.qdot;
    plot_sample.right.reference.position = right_reference_state.q;
    plot_sample.right.reference.velocity = right_reference_state.qdot;
    plot_sample.left.actual.position = robot.armPosition(ArmSide::kLeft);
    plot_sample.left.actual.velocity = robot.armVelocity(ArmSide::kLeft);
    plot_sample.right.actual.position = robot.armPosition(ArmSide::kRight);
    plot_sample.right.actual.velocity = robot.armVelocity(ArmSide::kRight);

    const bool ruckig_step_valid =
        diagnostics.left.dls_posture_ruckig_accepted &&
        diagnostics.right.dls_posture_ruckig_accepted;
    plot_sample.left.ruckig_output = plot_sample.right.ruckig_output = true;
    const bool left_output_valid = !paused && (recovery_plot_valid || ruckig_step_valid);
    const bool right_output_valid = !paused && (recovery_plot_valid || ruckig_step_valid);
    const ReferenceAccelerationSource acceleration_source =
        ReferenceAccelerationSource::kRuckigOutput;
    {
      const auto& normal_smoothing = config.pico_ee_franka_dls.post_smoothing;
      for (auto* arm : {&plot_sample.left, &plot_sample.right}) {
        const ArmSide side = arm == &plot_sample.left ? ArmSide::kLeft : ArmSide::kRight;
        const auto smoothing = recovery && !recovery->teleop()
            ? normal_smoothing : controller->trajectorySampleLimits(side);
        Vec7 velocity = (smoothing.velocity_scale * controller->motionLimits(side).velocity)
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
    snapshot.shared_root_upper_limb_skeleton =
        sharedRootUpperLimbSkeleton(shared_root_guidance->latestTargets());
    snapshot.shared_root_upper_limb_skeleton.valid =
        snapshot.shared_root_upper_limb_skeleton.valid &&
        pico_session.enabled() && pico_freshness.live;

    if (joint_command_exporter != nullptr) {
      output.sequence = ++joint_command_sequence;
      output.source_timestamp_ns = monotonicNowNs();
      output.pico_tracking_epoch = joint_command_epoch;
      output.input_monotonic_ns = pico_applied_bridge_send_monotonic_ns;
      output.reference_id = episode ? episode->referenceId() : 0U;
      // Recheck freshness after the solve, not at tick start: a slow solve must
      // never make an already expired PICO/hand input look live to hardware.
      const auto export_pico_stats =
          pico_receiver != nullptr ? pico_receiver->stats() : PicoReceiverStats{};
      const bool source_context_current =
          export_pico_stats.source_valid &&
          export_pico_stats.tracking_epoch == joint_command_epoch &&
          export_pico_stats.resynchronizations ==
              joint_command_resynchronization_generation;
      bool arms_ready =
          !stop_requested && running.load(std::memory_order_acquire) &&
          !paused && !pico_paused && !plot_reset_requested &&
          pico_configured && pico_session.enabled() &&
          pico_session.freshness(output.source_timestamp_ns).live &&
          shared_root_input_fresh(output.source_timestamp_ns) &&
          jointCommandPicoBridgeFresh(pico_applied_bridge_send_monotonic_ns,
                                      output.source_timestamp_ns) &&
          joint_command_epoch == pico_applied_epoch &&
          source_context_current &&
          snapshot.accepted && snapshot.hold_reason == HoldReason::kNone &&
          left_output_valid && right_output_valid &&
          snapshot.left_ik.status == SolverStatus::kSolved &&
          snapshot.right_ik.status == SolverStatus::kSolved &&
          !desired.left_stale && !desired.right_stale &&
          shared_root_diagnostics.accepted && shared_root_diagnostics.target_valid;
      if (episode) {
        arms_ready = !stop_requested && running.load(std::memory_order_acquire) &&
            episode->ready(output.source_timestamp_ns) && source_context_current &&
            jointCommandPicoBridgeFresh(pico_applied_bridge_send_monotonic_ns, output.source_timestamp_ns) &&
            (!episode->active() || (diagnostics.accepted &&
                diagnostics.left.dls_posture_ruckig_accepted && diagnostics.right.dls_posture_ruckig_accepted));
      }
      const bool hands_allowed =
          !stop_requested && running.load(std::memory_order_acquire) &&
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
               arms_ready, plot_reset_requested || joint_command_stream_reset ||
                               !source_context_current)
               ? kJointCommandArmsReadyFlag : 0U) |
          (left_hand_ready ? kJointCommandLeftHandReadyFlag : 0U) |
          (right_hand_ready ? kJointCommandRightHandReadyFlag : 0U));
      // These are the bilaterally committed references (Ruckig q for DLS),
      // never raw IK goals or simulated/actual-feedback arm positions.
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
      sample.mode = snapshot.mode;
      sample.paused = snapshot.paused;
      sample.accepted = snapshot.accepted;
      sample.hold_reason = snapshot.hold_reason;
      sample.left_target_stale = snapshot.left_target_stale;
      sample.right_target_stale = snapshot.right_target_stale;
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
      sample.left_reference_pose = references.left.pose;
      sample.right_reference_pose = references.right.pose;
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
  if (joint_command_exporter != nullptr && output.sequence != 0U) {
    // Explicitly revoke on orderly shutdown; receiver expiry still covers a
    // crash or a lost final UDP datagram. No native packet grants motion.
    output.sequence = ++joint_command_sequence;
    output.source_timestamp_ns = monotonicNowNs();
    output.flags = 0U;
    joint_command_exporter->send(output);
  }
}

std::string solverStatusName(SolverStatus status);

void writeTelemetryCsv(const std::string& path, TelemetryBuffer& telemetry,
                       const std::atomic<bool>& control_finished) {
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("cannot open telemetry output: " + path);
  }
  output << "sequence,control_time_seconds,algorithm,mode,paused,accepted,hold_reason,"
            "left_target_stale,right_target_stale,cycle_time_us,cycle_p99_us,"
            "deadline_misses,control_failures,pico_configured,pico_enabled,pico_live,pico_stale,"
            "pico_tracking_epoch,pico_sequence,pico_datagrams,pico_accepted,pico_malformed,"
            "pico_crc_failures,pico_reordered,pico_jump_rejections,pico_superseded,"
            "pico_epoch_resets,pico_resynchronizations,pico_reset_applies,"
            "pico_left_source_timestamp_ns,pico_right_source_timestamp_ns,"
            "pico_input_frequency_hz,pico_frame_age_ms,pico_receive_to_control_us,"
            "pico_bridge_to_control_us";
  for (const char* side : {"left", "right"}) {
    for (const char* field : {
             "position_error_m", "orientation_error_rad",
             "actual_position_error_m", "actual_orientation_error_rad",
             "accepted", "hold_reason", "solve_time_us", "iterations", "status",
             "dls_posture_reference_active", "dls_posture_status", "dls_posture_iterations",
             "dls_posture_joint_projection_count", "dls_posture_ruckig_accepted",
             "dls_posture_solve_time_us", "dls_posture_initial_position_error_m",
             "dls_posture_initial_orientation_error_rad", "dls_posture_final_position_error_m",
             "dls_posture_final_orientation_error_rad", "dls_posture_goal_error_max_abs",
             "dls_posture_reference_error_max_abs", "dls_posture_velocity_target_max_abs",
             "dls_posture_qdot_error_max_abs", "dls_posture_goal_limit_margin_rad",
             "ee_ik_wall_time_us", "ee_ruckig_wall_time_us", "ee_ik_to_ruckig_wall_time_us",
             "ee_ruckig_invoked", "ee_pinocchio_kinematics"}) {
      output << ',' << side << '_' << field;
    }
    for (const char* pose : {"target", "reference", "actual"}) {
      for (const char* axis : {"px", "py", "pz", "qx", "qy", "qz", "qw"}) {
        output << ',' << side << '_' << pose << '_' << axis;
      }
    }
  }
  output << ",simulation_phase\n" << std::setprecision(12);

  const auto write_arm = [&output](const ArmIkSnapshot& arm) {
    output << ',' << arm.position_error << ',' << arm.orientation_error
           << ',' << arm.actual_position_error << ',' << arm.actual_orientation_error
           << ',' << arm.accepted << ',' << toString(arm.hold_reason)
           << ',' << arm.solve_time_us << ',' << arm.iterations
           << ',' << solverStatusName(arm.status)
           << ',' << arm.dls_posture_reference_active << ',' << arm.dls_posture_status
           << ',' << arm.dls_posture_iterations << ',' << arm.dls_posture_joint_projection_count
           << ',' << arm.dls_posture_ruckig_accepted << ',' << arm.dls_posture_solve_time_us
           << ',' << arm.dls_posture_initial_position_error_m
           << ',' << arm.dls_posture_initial_orientation_error_rad
           << ',' << arm.dls_posture_final_position_error_m
           << ',' << arm.dls_posture_final_orientation_error_rad
           << ',' << arm.dls_posture_goal_error_max_abs
           << ',' << arm.dls_posture_reference_error_max_abs
           << ',' << arm.dls_posture_velocity_target_max_abs
           << ',' << arm.dls_posture_qdot_error_max_abs
           << ',' << arm.dls_posture_goal_limit_margin_rad
           << ',' << arm.ee_ik_wall_time_us << ',' << arm.ee_ruckig_wall_time_us
           << ',' << arm.ee_ik_to_ruckig_wall_time_us << ',' << arm.ee_ruckig_invoked
           << ',' << arm.ee_pinocchio_kinematics;
  };
  const auto write_pose = [&output](const Pose& pose) {
    const Eigen::Quaterniond quaternion(pose.rotation);
    output << ',' << pose.position.x() << ',' << pose.position.y() << ',' << pose.position.z()
           << ',' << quaternion.x() << ',' << quaternion.y() << ',' << quaternion.z()
           << ',' << quaternion.w();
  };
  TelemetrySample sample;
  for (;;) {
    if (telemetry.tryPop(sample)) {
      output << sample.sequence << ',' << sample.control_time_seconds
             << ',' << toString(sample.algorithm) << ',' << toString(sample.mode)
             << ',' << sample.paused << ',' << sample.accepted << ',' << toString(sample.hold_reason)
             << ',' << sample.left_target_stale << ',' << sample.right_target_stale
             << ',' << sample.cycle_time_us << ',' << sample.cycle_p99_us
             << ',' << sample.deadline_misses << ',' << sample.control_failures
             << ',' << sample.pico_configured << ',' << sample.pico_enabled
             << ',' << sample.pico_live << ',' << sample.pico_stale
             << ',' << sample.pico_tracking_epoch << ',' << sample.pico_sequence
             << ',' << sample.pico_datagrams << ',' << sample.pico_accepted
             << ',' << sample.pico_malformed << ',' << sample.pico_crc_failures
             << ',' << sample.pico_reordered << ',' << sample.pico_jump_rejections
             << ',' << sample.pico_superseded << ',' << sample.pico_epoch_resets
             << ',' << sample.pico_resynchronizations << ',' << sample.pico_reset_applies
             << ',' << sample.pico_left_source_timestamp_ns << ',' << sample.pico_right_source_timestamp_ns
             << ',' << sample.pico_input_frequency_hz << ',' << sample.pico_frame_age_ms
             << ',' << sample.pico_receive_to_control_us << ',' << sample.pico_bridge_to_control_us;
      write_arm(sample.left_ik);
      write_pose(sample.left_target_pose);
      write_pose(sample.left_reference_pose);
      write_pose(sample.left_actual_pose);
      write_arm(sample.right_ik);
      write_pose(sample.right_target_pose);
      write_pose(sample.right_reference_pose);
      write_pose(sample.right_actual_pose);
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
  if (application.external_display) {
    if (key == GLFW_KEY_ESCAPE) {
      if (!application.external_quit_sent) {
        std::cout << "DISPLAY_KEY " << GLFW_KEY_Q << std::endl;
        application.external_quit_sent = true;
      }
      glfwSetWindowShouldClose(window, GLFW_TRUE);
      return;
    }
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
    if (key != GLFW_KEY_F1 && key != GLFW_KEY_F2 && key != GLFW_KEY_F3 &&
        key != GLFW_KEY_F4 && key != GLFW_KEY_F5 &&
        key != GLFW_KEY_L && key != GLFW_KEY_R) {
      return;
    }
  }
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
    case GLFW_KEY_P:
      application.marker.cancelDrag();
      application.preview.cancelAll();
      command.type = ViewerCommandType::kTogglePicoTeleop;
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
  if (application.external_display) {
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
    application.hovered_handle = application.external_display
        ? MarkerHandle::kNone
        : currentMarkerHandle(application, viewportCursor(window, x, y));
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
    if (application.external_display) {
      // Display the supplied positions exactly; the controller's hand setter
      // intentionally clamps, but a display must not modify its input state.
      for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
        const HandMapping& mapping = application.robot.handMapping(side);
        const Vec20& position = side == ArmSide::kLeft
            ? application.snapshot.left_hand_q : application.snapshot.right_hand_q;
        for (int joint = 0; joint < kHandDof; ++joint) {
          application.robot.data()->qpos[
              mapping.qpos_addresses[static_cast<std::size_t>(joint)]] = position[joint];
        }
      }
    } else {
      application.robot.setHandPosition(ArmSide::kLeft,
                                        application.snapshot.left_hand_q);
      application.robot.setHandPosition(ArmSide::kRight,
                                        application.snapshot.right_hand_q);
    }
  }
  if (application.external_display) {
    application.robot.forward();
    return;
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
  if (application.external_display) {
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
    return;
  }
  char status[2048]{};
  char help[1024]{};
  const char* selected = application.selected_arm == ArmSide::kLeft ? "left" : "right";
  const char* frame = application.marker.frame() == MarkerFrame::kWorld ? "world" : "local";
  std::snprintf(
      status, sizeof(status),
      "IK: %s | mode: %s | selected: %s | marker: %s%s\n"
      "left  err %.4f m / %.3f rad | %.1f us | %s\n"
      "right err %.4f m / %.3f rad | %.1f us | %s\n"
      "cycle: %.1f us | rolling p99: %.1f us | deadline misses: %llu\n"
      "control failures: %llu | snapshot/telemetry drops: %llu / %llu | command drops: %llu\n"
      "PICO cfg/en/live/stale %d/%d/%d/%d | epoch/seq %llu/%llu | %.1f Hz | age %.1f ms\n"
      "PICO rx accepted/datagrams %llu/%llu | bad/crc/order/jump/super/epoch/resync/reset %llu/%llu/%llu/%llu/%llu/%llu/%llu/%llu | latency recv/bridge %.1f/%.1f us\n"
      "skeleton overlay: %s | PICO %s | shared-root %s\n"
      "Hands %s | %s rx/accepted/rejected %llu/%llu/%llu | stale L/R %d/%d\n"
      "safety: %s",
      toString(application.snapshot.algorithm).c_str(),
      toString(application.snapshot.mode).c_str(),
      selected, frame, application.snapshot.paused ? " | PAUSED" : "",
      application.snapshot.left_position_error, application.snapshot.left_orientation_error,
      application.snapshot.left_ik.solve_time_us,
      solverStatusName(application.snapshot.left_solver_status).c_str(),
      application.snapshot.right_position_error, application.snapshot.right_orientation_error,
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
      application.show_pico_skeleton ? "on" : "off",
      application.snapshot.pico_upper_limb_skeleton.valid ? "valid" : "hidden",
      application.snapshot.shared_root_upper_limb_skeleton.valid ? "valid" : "hidden",
      !application.snapshot.hand_configured ? "disabled" :
          application.snapshot.hand_ros ? "manus" : "exoskeleton",
      application.snapshot.hand_ros ? "ROS messages" : "UDP datagrams",
      static_cast<unsigned long long>(application.snapshot.hand_ros
          ? application.snapshot.hand_messages : application.snapshot.hand_datagrams),
      static_cast<unsigned long long>(application.snapshot.hand_accepted),
      static_cast<unsigned long long>(application.snapshot.hand_ros
          ? application.snapshot.hand_rejected
          : application.snapshot.hand_malformed + application.snapshot.hand_reordered),
      application.snapshot.hand_left_stale, application.snapshot.hand_right_stale,
      toString(application.snapshot.hold_reason).c_str());
  if (application.show_help) {
    std::snprintf(help, sizeof(help),
                  "L/R select | drag XYZ arrow: translate | drag XYZ ring: rotate | "
                  "drag center: view plane | W world/local\n"
                  "0/M manual | 1 circle | 2 figure-8 | 3 orientation | 4 combined | H hold\n"
                  "P PICO teleop | K PICO skeleton | Space pause | N nominal reset\n"
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
            << " accepted=" << latest.accepted
            << " pico_configured=" << latest.pico_configured
            << " pico_enabled=" << latest.pico_enabled
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
            << " hand_source=" << (!latest.hand_configured ? "disabled" :
                                    latest.hand_ros ? "manus" : "exoskeleton")
            << " hand_ros_messages=" << latest.hand_messages
            << " hand_ros_rejected=" << latest.hand_rejected
            << " hand_left_stale=" << latest.hand_left_stale
            << " hand_right_stale=" << latest.hand_right_stale
            << " hand_left_q0=" << latest.left_hand_q[0]
            << " hand_right_q0=" << latest.right_hand_q[0]
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

// Private parent pipe, consumed on the render thread: no control worker or
// network receiver exists in external-display mode. Bounded reads keep the
// camera and window responsive even if the producer outruns rendering.
class ExternalDisplayInput {
 public:
  ExternalDisplayInput(const MujocoRobot& robot, const QpIkConfig& config) {
    {
      const auto& smoothing = config.pico_ee_franka_dls.post_smoothing;
      for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
        JointKinematicsBounds& bounds =
            side == ArmSide::kLeft ? left_bounds_ : right_bounds_;
        const auto limits = effectiveArmLimits(config.joint_limits, robot.mapping(side).limits, side);
        setConfiguredPlotPositionBounds(limits, config, bounds);
        const Vec7 velocity =
            (smoothing.velocity_scale * limits.velocity)
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
    application.snapshot.left_q = Eigen::Map<const Vec7>(joints.data());
    application.snapshot.right_q = Eigen::Map<const Vec7>(joints.data() + 7);
    application.snapshot.left_hand_q = Eigen::Map<const Vec20>(joints.data() + 14);
    application.snapshot.right_hand_q = Eigen::Map<const Vec20>(joints.data() + 34);
    if (sequence_ == 0U) {
      first_timestamp_ = timestamp;
    }
    const double dt = sequence_ == 0U
        ? 0.0 : static_cast<double>(timestamp - previous_timestamp_) * 1e-9;
    JointKinematicsSample sample;
    sample.sequence = ++sequence_;
    sample.time_seconds = static_cast<double>(timestamp - first_timestamp_) * 1e-9;
    sample.left.bounds = left_bounds_;
    sample.right.bounds = right_bounds_;
    setState(sample.left, application.snapshot.left_q, previous_left_,
             left_differentiator_, dt);
    setState(sample.right, application.snapshot.right_q, previous_right_,
             right_differentiator_, dt);
    application.joint_plot_history.push(sample);
    previous_timestamp_ = timestamp;
    application.snapshot.sequence = sequence_;
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

int runViewer(const Options& options, BoundedSpscQueue<ViewerCommand>& commands,
              LatestSnapshotExchange<ViewerSnapshot>& snapshots,
              BoundedSpscQueue<JointKinematicsSample>& joint_plot_queue,
              std::atomic<bool>& running, std::thread& control_thread,
              const QpIkConfig* display_config = nullptr) {
  if (glfwInit() == GLFW_FALSE) {
    throw std::runtime_error("GLFW initialization failed; use --headless without a display");
  }
  GLFWwindow* window = glfwCreateWindow(1280, 800, "Tianji dual-arm DLS", nullptr, nullptr);
  if (window == nullptr) {
    glfwTerminate();
    throw std::runtime_error("GLFW window creation failed");
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);
  MujocoRobot render_robot(options.model_path);
  ViewerApplication application(render_robot, commands);
  application.external_display = options.external_display;
  application.continuous_follow = options.continuous_follow;
  std::optional<ExternalDisplayInput> display_input;
  if (options.external_display) {
    if (!render_robot.hasHandMappings() || display_config == nullptr) {
      glfwDestroyWindow(window);
      glfwTerminate();
      throw std::runtime_error("external display requires a mapped 54-joint model and plot config");
    }
    display_input.emplace(render_robot, *display_config);
    glfwSetWindowTitle(window, "Tianji simulation joint display | no hardware output");
  }
  application.show_pico_skeleton = options.pico_skeleton_overlay;
  application.target_left_body = mj_name2id(render_robot.model(), mjOBJ_BODY, "target_L");
  application.target_right_body = mj_name2id(render_robot.model(), mjOBJ_BODY, "target_R");
  if (!options.external_display &&
      (application.target_left_body < 0 || application.target_right_body < 0)) {
    glfwDestroyWindow(window);
    glfwTerminate();
    throw std::runtime_error("viewer target bodies are missing");
  }
  mjv_defaultFreeCamera(render_robot.model(), &application.camera);
    if (options.external_display) {
      application.camera.distance *= 1.5;
    }
  mjv_defaultOption(&application.visual_options);
  application.visual_options.frame =
      options.external_display ? mjFRAME_NONE : mjFRAME_SITE;
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
  glfwSetWindowCloseCallback(window, [](GLFWwindow* closing_window) {
    auto& app = *static_cast<ViewerApplication*>(glfwGetWindowUserPointer(closing_window));
    if (app.external_display && !app.external_quit_sent) {
      std::cout << "DISPLAY_KEY " << GLFW_KEY_Q << std::endl;
      app.external_quit_sent = true;
    }
  });
  if (options.external_display) {
    std::cout << "DISPLAY_READY" << std::endl;
  }

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
    if (display_input) {
      const auto previous_sequence = application.snapshot.sequence;
      if (!display_input->drain(application)) {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
      }
      if (application.snapshot.sequence != previous_sequence) {
        renderSnapshot(application);
      }
    }
    if (snapshots.tryReadLatest(application.snapshot)) {
      renderSnapshot(application);
      if (options.simulation_recovery) {
        static const char* phases[] = {"WAITING", "TELEOP", "BRAKING", "HOMING", "HOME_REACHED", "HOLD", "FAULT"};
        const int phase = application.snapshot.simulation_phase;
        if (phase >= 0 && phase < 7) {
          const std::string title = std::string("Franka DLS + Ruckig SIM | ") + phases[phase] +
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
    if (options.external_display) {
      // Static target markers are not part of the supplied simulation state.
      const mjModel* model = render_robot.model();
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
    } else {
      const Pose selected_target = mocapPose(application, application.selected_arm);
      appendInteractiveMarker(application.marker.geometry(selected_target),
                              application.hovered_handle,
                              application.marker.activeHandle(), &application.scene);
    }
    if (application.show_pico_skeleton) {
      appendPicoUpperLimbSkeleton(
          application.snapshot.pico_upper_limb_skeleton, &application.scene,
          SkeletonOverlayStyle::kPico);
      appendPicoUpperLimbSkeleton(
          application.snapshot.shared_root_upper_limb_skeleton, &application.scene,
          SkeletonOverlayStyle::kSharedRoot);
    }
    mjr_render(viewport, &application.scene, &application.context);
    drawOverlay(application, viewport);
    if (application.joint_plot_layout.visible) {
      if (options.external_display) {
        application.joint_plot.updateSimulation(
            application.joint_plot_history, application.joint_plot_arm,
            application.joint_plot_metric, 5.0);
      } else {
        application.joint_plot.update(
            application.joint_plot_history, application.joint_plot_arm,
            application.joint_plot_metric, 5.0);
      }
      application.joint_plot.render(application.joint_plot_layout,
                                    application.context);
      char plot_status[640]{};
      if (options.external_display) {
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
      } else {
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
          "cyan Ruckig | blue model (diff) | red limits\nRuckig jerk = delta acceleration / dt",
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
      }
      mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT,
                  application.joint_plot_layout.status, plot_status, nullptr,
                  &application.context);
    }
    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  running.store(false, std::memory_order_release);
  if (control_thread.joinable()) {
    control_thread.join();
  }
  mjr_freeContext(&application.context);
  mjv_freeScene(&application.scene);
  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}

int run(int argc, char** argv) {
  Options options = parseOptions(argc, argv);
  if (options.help_requested) return 0;
  if (options.config_path.empty()) {
    options.config_path = (runtimeWorkspace() / "install/control/share/tianji_controller/config/qp_ik_pico_shared_root_dls.yaml").string();
  }
  if (options.continuous || options.simulation_recovery) {
    stop_requested = 0;
    if (std::signal(SIGINT, requestStop) == SIG_ERR ||
        std::signal(SIGTERM, requestStop) == SIG_ERR) {
      throw std::runtime_error("could not install continuous headless stop handlers");
    }
  }
  if (!options.external_display) {
    std::cout << "viewer_config=" << options.config_path << '\n';
  }
  QpIkConfig config = loadConfig(options.config_path, ConfigConsumer::kSharedRootAware);
  if (options.external_display) {
    if (options.model_path.empty()) {
      options.model_path = config.shared_root_profile_path.empty()
          ? defaultModelPath()
          : loadSharedRootOptions(config.shared_root_profile_path).mujoco_path;
    }
    BoundedSpscQueue<ViewerCommand> commands(1U);
    LatestSnapshotExchange<ViewerSnapshot> snapshots(2U);
    BoundedSpscQueue<JointKinematicsSample> joint_plot_queue(1U);
    std::atomic<bool> running{true};
    std::thread no_control_thread;
    return runViewer(options, commands, snapshots, joint_plot_queue, running,
                     no_control_thread, &config);
  }
  if (options.model_state_only_override.has_value()) {
    config.controller.model_state_only = *options.model_state_only_override;
  }
#ifdef TIANJI_ROS_TRANSPORT
  if (config.shared_root_profile_path.empty() ||
      config.ik_algorithm != IkAlgorithm::kPicoEeFrankaDls ||
      !config.pico_ee_franka_dls.enabled ||
      !config.pico_ee_franka_dls.post_smoothing.enabled ||
      !config.controller.model_state_only)
    throw std::invalid_argument(
        "ROS arm core requires shared-root Franka DLS/Ruckig model-reference velocity control");
#endif
  if (options.episode_relative &&
      (!options.franka_dls_executor || !options.outputEnabled() ||
       options.simulation_recovery || options.external_display || !options.pico_teleop))
    throw std::invalid_argument("--episode-relative requires ROS DLS export and excludes simulation/display mode");
  if (options.franka_dls_executor &&
      (config.shared_root_profile_path.empty() ||
       config.ik_algorithm != IkAlgorithm::kPicoEeFrankaDls ||
       !config.pico_ee_franka_dls.enabled ||
       !config.pico_ee_franka_dls.post_smoothing.enabled ||
       !config.controller.model_state_only)) {
    throw std::invalid_argument(
        "--franka-dls-executor requires enabled shared-root, pico_ee_franka_dls, "
        "Ruckig, model-only state and velocity control");
  }
  if (!config.shared_root_profile_path.empty()) {
    if (!options.pico_teleop)
      throw std::invalid_argument("shared-root requires PICO input");
    if (options.outputEnabled() && !options.franka_dls_executor)
      throw std::invalid_argument(
          "shared-root joint command export requires --franka-dls-executor");
    const auto shared = loadSharedRootOptions(config.shared_root_profile_path);
    if (!shared.enabled)
      throw std::invalid_argument("shared-root execution requires enabled shared-root");
    if (!options.model_path_explicit) options.model_path = shared.mujoco_path;
    if (sharedRootSha256File(options.model_path) != sharedRootSha256File(shared.mujoco_path))
      throw std::invalid_argument("shared-root requires its frozen palm TCP model");
    std::cout << (options.franka_dls_executor
        ? "shared_root=enabled; joint_export=franka_dls_executor; motion_authority=external_python_gate\n"
        : "shared_root=experimental; device_acceptance=false; joint_export=disabled\n");
  }
  if (options.model_path.empty()) options.model_path = defaultModelPath();
    if (config.shared_root_profile_path.empty() || !config.controller.model_state_only ||
        (options.outputEnabled() && !options.franka_dls_executor))
      throw std::invalid_argument(
          "direct IK viewer requires enabled shared-root model-only mode; "
          "joint export requires --franka-dls-executor");
  if (options.sim_allow_pico_jumps && !options.simulation_recovery)
    throw std::invalid_argument("--sim-allow-pico-jumps requires --simulation-recovery (DLS model-only, no export)");
  if (options.simulation_recovery &&
      (config.shared_root_profile_path.empty() || !config.controller.model_state_only ||
       !options.pico_teleop || options.outputEnabled()))
    throw std::invalid_argument("simulation recovery requires shared-root DLS, PICO, model-only, no export");
#ifndef TIANJI_ROS_TRANSPORT
  if (options.hand_teleop && options.pico_teleop && options.hand_port == options.pico_port)
    throw std::invalid_argument("hand and PICO ports must differ");
#endif
  if (options.simulation_recovery && options.handUdpEnabled() && options.hand_bind != "127.0.0.1")
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
#ifdef TIANJI_ROS_TRANSPORT
  std::unique_ptr<ArmRosTransport> ros_transport;
  if (options.pico_teleop) {
    ArmRosTransportOptions transport_options;
    transport_options.pico_topic = options.pico_topic;
    transport_options.left_hand_topic = options.left_hand_topic;
    transport_options.right_hand_topic = options.right_hand_topic;
    transport_options.hand_freshness_seconds = options.hand_stale_timeout_seconds;
    transport_options.joint_target_topic = options.joint_target_topic;
    transport_options.max_position_jump_m = config.pico_teleop.max_position_jump_m;
    transport_options.max_orientation_jump_rad = config.pico_teleop.max_orientation_jump_rad;
    transport_options.freshness_seconds = config.cartesian_servo.target_timeout_seconds;
    transport_options.reject_pose_jumps = !options.sim_allow_pico_jumps;
    transport_options.episode_relative = options.episode_relative;
    ros_transport = std::make_unique<ArmRosTransport>(
        std::move(transport_options), pico_frames, options.handRosEnabled() ? &hand_frames : nullptr);
  }
  PicoInputReceiver* pico_receiver = ros_transport.get();
  JointCommandSink* joint_command_exporter =
      options.outputEnabled() ? ros_transport.get() : nullptr;
#else
  std::unique_ptr<PicoUdpReceiver> udp_receiver;
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
    udp_receiver = std::make_unique<PicoUdpReceiver>(
        std::move(receiver_options), pico_frames);
  }
  PicoInputReceiver* pico_receiver = udp_receiver.get();
#endif
  MujocoRobot control_robot(options.model_path);
  std::unique_ptr<WujiHandUdpReceiver> hand_receiver;
  if (options.hand_teleop) {
    if (!control_robot.hasHandMappings()) {
      throw std::invalid_argument(
          "--hand-teleop requires a combined Wuji Hand 2 model");
    }
  }
  if (options.handUdpEnabled()) {
    WujiHandUdpReceiverOptions receiver_options;
    receiver_options.bind_address = options.hand_bind;
    receiver_options.port = options.hand_port;
    receiver_options.stale_timeout_seconds =
        options.hand_stale_timeout_seconds;
    hand_receiver = std::make_unique<WujiHandUdpReceiver>(
        std::move(receiver_options), hand_frames);
  }
#ifndef TIANJI_ROS_TRANSPORT
  std::unique_ptr<JointCommandExporter> udp_exporter;
  if (options.outputEnabled()) {
    udp_exporter = std::make_unique<JointCommandExporter>(
        options.joint_command_host, options.joint_command_port);
  }
  JointCommandSink* joint_command_exporter = udp_exporter.get();
#endif
  std::atomic<bool> running{true};
  std::atomic<bool> control_finished{false};
  std::exception_ptr control_error;
  std::exception_ptr telemetry_error;
  std::exception_ptr joint_telemetry_error;
  std::thread telemetry_thread;
  std::thread joint_telemetry_thread;
  std::thread control_thread;
  try {
    if (pico_receiver != nullptr) {
      pico_receiver->start();
#ifndef TIANJI_ROS_TRANSPORT
      std::cout << "pico_udp_bind=" << options.pico_bind << ':'
                << udp_receiver->boundPort() << '\n';
      if (!options.pico_record_path.empty()) {
        std::cout << "pico_record_path=" << options.pico_record_path << '\n';
      }
#endif
    }
    if (hand_receiver != nullptr) {
      hand_receiver->start();
      std::cout << "hand_udp_bind=" << options.hand_bind << ':'
                << hand_receiver->boundPort() << '\n';
    }
    if (!options.hand_teleop)
      std::cout << "hand_source=disabled hand_input_state=disabled\n";
    else if (options.handUdpEnabled())
      std::cout << "hand_source=exoskeleton hand_transport=udp\n";
#ifndef TIANJI_ROS_TRANSPORT
    if (joint_command_exporter != nullptr) {
      std::cout << "joint_command_udp=" << options.joint_command_host << ':'
                << options.joint_command_port
                << " joint_command_packet_size=" << kJointCommandPacketSize
                << " joint_command_state=ready" << std::endl;
    }
#endif
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
                    pico_receiver,
                    options.hand_teleop ? &hand_frames : nullptr,
                    hand_receiver.get(),
#ifdef TIANJI_ROS_TRANSPORT
                    options.handRosEnabled() ? ros_transport.get() : nullptr,
#endif
                    joint_command_exporter,
                    options.pico_teleop,
                    options.simulation_recovery,
                    options.episode_relative,
                    running);
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
    if (options.headless) {
      result = runPicoHeadless(options, snapshots, joint_plot_queue, running,
                               control_thread);
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
#ifdef TIANJI_ROS_TRANSPORT
  if (ros_transport) {
    if (options.handRosEnabled()) {
      const auto stats = ros_transport->handStats();
      std::cout << "hand_record_complete hand_ros_state=stopped hand_source=manus"
                << " hand_ros_messages=" << stats.received
                << " hand_accepted=" << stats.accepted
                << " hand_ros_rejected=" << stats.rejected
                << " hand_left_stale=" << stats.left_stale
                << " hand_right_stale=" << stats.right_stale << '\n';
    }
  }
#endif
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
#ifdef TIANJI_ROS_TRANSPORT
    const auto workspace = tianji_qp_ik::runtimeWorkspace();
    const auto prefixes = (workspace / "install/arm-ros/tianji_interfaces").string() +
        ":" + (workspace / ".pixi/envs/arm-ros").string();
    if (setenv("AMENT_PREFIX_PATH", prefixes.c_str(), 1) != 0 ||
        setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp", 1) != 0 ||
        setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST", 1) != 0 ||
        unsetenv("ROS_LOCALHOST_ONLY") != 0)
      throw std::runtime_error("cannot configure isolated arm ROS runtime");
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    struct RosShutdown {
      ~RosShutdown() { rclcpp::shutdown(); }
    } shutdown;
    auto arguments = rclcpp::remove_ros_arguments(argc, argv);
    std::vector<char*> native_arguments;
    native_arguments.reserve(arguments.size());
    for (auto& argument : arguments) native_arguments.push_back(argument.data());
    return tianji_qp_ik::run(static_cast<int>(native_arguments.size()), native_arguments.data());
#else
    return tianji_qp_ik::run(argc, argv);
#endif
  } catch (const std::exception& error) {
    std::cerr << "viewer error: " << error.what() << '\n';
    return 1;
  }
}
