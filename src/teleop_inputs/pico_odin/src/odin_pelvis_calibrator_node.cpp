#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include "pico_odin/calibration_session.hpp"
#include "pico_odin/extrinsics_file.hpp"
#include "pico_odin/ground_ready_gate.hpp"
#include "pico_odin/pose_stationarity.hpp"
#include "pico_odin/pose_stream_continuity.hpp"
#include "pico_odin/skeleton_frame_contract.hpp"

namespace pico_odin {
namespace {

double stamp_seconds(const builtin_interfaces::msg::Time & stamp) {
  return static_cast<double>(stamp.sec) + 1e-9 * static_cast<double>(stamp.nanosec);
}

std::string utc_now() {
  const std::time_t now = std::time(nullptr);
  std::tm value{};
  gmtime_r(&now, &value);
  std::ostringstream output;
  output << std::put_time(&value, "%Y-%m-%dT%H:%M:%SZ");
  return output.str();
}

const char * phase_name(CalibrationPhase phase) {
  switch (phase) {
    case CalibrationPhase::kWaitingForInputs: return "waiting_inputs";
    case CalibrationPhase::kReady: return "ready";
    case CalibrationPhase::kPendingStart: return "pending_start";
    case CalibrationPhase::kNeutral: return "neutral";
    case CalibrationPhase::kExcitation: return "excitation";
    case CalibrationPhase::kFinalNeutral: return "final_neutral";
    case CalibrationPhase::kSolving: return "solving";
    case CalibrationPhase::kSaved: return "saved";
    case CalibrationPhase::kRejected: return "rejected";
    case CalibrationPhase::kCancelled: return "cancelled";
  }
  return "unknown";
}

}  // namespace

class OdinPelvisCalibratorNode final : public rclcpp::Node {
 public:
  OdinPelvisCalibratorNode()
  : Node("odin_pelvis_calibrator"), session_(session_options())
  {
    pico_topic_ = declare_parameter<std::string>("pico_topic", "/pico/smpl");
    odin_topic_ = declare_parameter<std::string>(
      "odin_topic", "/raw/odom/odin_highfreq");
    expected_skeleton_frame_ = declare_parameter<std::string>(
      "expected_skeleton_frame", "pico_ground");
    if (pico_topic_ != "/pico/smpl" || odin_topic_ != "/raw/odom/odin_highfreq") {
      throw std::invalid_argument(
        "installation calibration requires canonical /pico/smpl and raw /raw/odom/odin_highfreq topics");
    }
    if (expected_skeleton_frame_.empty()) {
      throw std::invalid_argument("expected_skeleton_frame must not be empty");
    }
    const auto configured_path = declare_parameter<std::string>("output_file", "");
    output_path_ = configured_path.empty() ? default_extrinsics_path() :
      std::filesystem::path(configured_path);
    max_input_age_sec_ = declare_parameter<double>("max_input_age_sec", 0.5);
    start_on_world_reset_ = declare_parameter<bool>("start_on_world_reset", true);
    world_reset_topic_ = declare_parameter<std::string>(
      "world_reset_topic", "/pico/world_reset");
    ground_ready_topic_ = declare_parameter<std::string>(
      "ground_ready_topic", "/pico/smpl_ground_ready");
    stationary_linear_speed_ = declare_parameter<double>("stationary_linear_speed_mps", 0.04);
    stationary_angular_speed_ = declare_parameter<double>("stationary_angular_speed_radps", 0.08);
    if (world_reset_topic_.empty() || ground_ready_topic_.empty() ||
        !std::isfinite(max_input_age_sec_) || !(max_input_age_sec_ > 0.0) ||
        !std::isfinite(stationary_linear_speed_) ||
        !std::isfinite(stationary_angular_speed_) || stationary_linear_speed_ < 0.0 ||
        stationary_angular_speed_ < 0.0) {
      throw std::invalid_argument("input age and stationarity thresholds must be non-negative");
    }
    PoseStationarityOptions stationarity_options;
    stationarity_options.maximum_linear_speed_mps = stationary_linear_speed_;
    stationarity_options.maximum_angular_speed_radps = stationary_angular_speed_;
    pico_stationarity_ = PoseStationarity(stationarity_options);
    odin_stationarity_ = PoseStationarity(stationarity_options);

    solver_options_.min_pitch_range_rad = get_parameter("min_pitch_range_rad").as_double();
    solver_options_.min_yaw_range_rad = get_parameter("min_yaw_range_rad").as_double();
    const int min_samples = declare_parameter<int>("min_samples", 80);
    if (min_samples < 12) {
      throw std::invalid_argument("min_samples must be at least 12");
    }
    solver_options_.min_pairs = static_cast<std::size_t>(min_samples);
    solver_options_.max_receive_lag_sec =
      declare_parameter<double>("max_receive_lag_sec", 0.30);
    solver_options_.receive_lag_step_sec =
      declare_parameter<double>("receive_lag_step_sec", 0.002);
    solver_options_.min_receive_lag_correlation =
      declare_parameter<double>("min_receive_lag_correlation", 0.25);
    const int min_receive_lag_pairs =
      declare_parameter<int>("min_receive_lag_pairs", 10);
    if (min_receive_lag_pairs < 3) {
      throw std::invalid_argument("min_receive_lag_pairs must be at least 3");
    }
    solver_options_.min_receive_lag_pairs =
      static_cast<std::size_t>(min_receive_lag_pairs);
    solver_options_.max_interpolation_gap_sec =
      declare_parameter<double>("max_interpolation_gap_sec", 0.10);
    solver_options_.mount_x_min_m =
      declare_parameter<double>("mount_x_min_m", -0.65);
    solver_options_.mount_x_max_m =
      declare_parameter<double>("mount_x_max_m", 0.10);
    solver_options_.mount_abs_y_max_m =
      declare_parameter<double>("mount_abs_y_max_m", 0.20);
    solver_options_.mount_abs_z_max_m =
      declare_parameter<double>("mount_abs_z_max_m", 0.50);
    solver_options_.mount_rear_angle_max_rad =
      declare_parameter<double>("mount_rear_angle_max_rad", 1.60);
    solver_options_.y_prior_sigma_m =
      declare_parameter<double>("mount_y_prior_sigma_m", 0.05);
    solver_options_.rotation_prior_sigma_rad =
      declare_parameter<double>("mount_rotation_prior_sigma_rad", 10.0);
    solver_options_.max_rotation_rms_rad =
      declare_parameter<double>("max_rotation_rms_rad", 0.10);
    solver_options_.max_translation_rms_m =
      declare_parameter<double>("max_translation_rms_m", 0.08);
    solver_options_.max_condition_number =
      declare_parameter<double>("max_condition_number", 1e6);
    solver_options_.max_final_neutral_rotation_error_rad =
      declare_parameter<double>("max_final_neutral_rotation_error_rad", 0.10);
    solver_options_.max_final_neutral_tilt_rad =
      declare_parameter<double>("max_final_neutral_tilt_rad", 25.0 * M_PI / 180.0);
    const double lag_steps = 2.0 * solver_options_.max_receive_lag_sec /
      solver_options_.receive_lag_step_sec;
    if (!std::isfinite(solver_options_.max_receive_lag_sec) ||
        !(solver_options_.max_receive_lag_sec > 0.0) ||
        !std::isfinite(solver_options_.receive_lag_step_sec) ||
        !(solver_options_.receive_lag_step_sec > 0.0) ||
        !std::isfinite(lag_steps) || lag_steps > 1e6 ||
        !std::isfinite(solver_options_.min_receive_lag_correlation) ||
        solver_options_.min_receive_lag_correlation < -1.0 ||
        solver_options_.min_receive_lag_correlation > 1.0 ||
        !std::isfinite(solver_options_.max_interpolation_gap_sec) ||
        !(solver_options_.max_interpolation_gap_sec > 0.0) ||
        !std::isfinite(solver_options_.mount_x_min_m) ||
        !std::isfinite(solver_options_.mount_x_max_m) ||
        !(solver_options_.mount_x_min_m < solver_options_.mount_x_max_m) ||
        !std::isfinite(solver_options_.mount_abs_y_max_m) ||
        !(solver_options_.mount_abs_y_max_m > 0.0) ||
        !std::isfinite(solver_options_.mount_abs_z_max_m) ||
        !(solver_options_.mount_abs_z_max_m > 0.0) ||
        !std::isfinite(solver_options_.mount_rear_angle_max_rad) ||
        !(solver_options_.mount_rear_angle_max_rad > 0.0) ||
        solver_options_.mount_rear_angle_max_rad > M_PI ||
        !std::isfinite(solver_options_.y_prior_sigma_m) ||
        !(solver_options_.y_prior_sigma_m > 0.0) ||
        !std::isfinite(solver_options_.rotation_prior_sigma_rad) ||
        !(solver_options_.rotation_prior_sigma_rad > 0.0) ||
        !std::isfinite(solver_options_.max_rotation_rms_rad) ||
        !(solver_options_.max_rotation_rms_rad > 0.0) ||
        !std::isfinite(solver_options_.max_translation_rms_m) ||
        !(solver_options_.max_translation_rms_m > 0.0) ||
        !std::isfinite(solver_options_.max_condition_number) ||
        !(solver_options_.max_condition_number > 0.0) ||
        !std::isfinite(solver_options_.max_final_neutral_rotation_error_rad) ||
        !(solver_options_.max_final_neutral_rotation_error_rad > 0.0) ||
        solver_options_.max_final_neutral_rotation_error_rad > M_PI ||
        !std::isfinite(solver_options_.max_final_neutral_tilt_rad) ||
        !(solver_options_.max_final_neutral_tilt_rad > 0.0) ||
        solver_options_.max_final_neutral_tilt_rad > M_PI) {
      throw std::invalid_argument("invalid receive-lag or mounting-bound parameters");
    }
    PoseStreamContinuityOptions continuity_options;
    continuity_options.restart_receipt_gap_sec =
      declare_parameter<double>("odin_restart_gap_sec", 1.0);
    continuity_options.minimum_translation_jump_m =
      declare_parameter<double>("odin_translation_jump_m", 0.35);
    continuity_options.minimum_rotation_jump_rad =
      declare_parameter<double>("odin_rotation_jump_rad", 0.60);
    if (!std::isfinite(continuity_options.restart_receipt_gap_sec) ||
        !(continuity_options.restart_receipt_gap_sec > 0.0) ||
        !std::isfinite(continuity_options.minimum_translation_jump_m) ||
        !(continuity_options.minimum_translation_jump_m > 0.0) ||
        !std::isfinite(continuity_options.minimum_rotation_jump_rad) ||
        !(continuity_options.minimum_rotation_jump_rad > 0.0)) {
      throw std::invalid_argument("invalid Odin restart detection parameters");
    }
    continuity_ = PoseStreamContinuity(continuity_options);

    pico_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      pico_topic_, rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseArray::ConstSharedPtr message) { on_pico(*message); });
    odin_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odin_topic_, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) { on_odin(*message); });
    world_reset_sub_ = create_subscription<std_msgs::msg::Float32>(
      world_reset_topic_, 10,
      [this](std_msgs::msg::Float32::ConstSharedPtr) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (start_on_world_reset_ &&
            (session_.phase() == CalibrationPhase::kReady ||
             session_.phase() == CalibrationPhase::kPendingStart)) {
          const bool was_pending = session_.phase() == CalibrationPhase::kPendingStart;
          session_.request_start();
          reset_input_freshness_locked();
          ground_ready_gate_.arm();
          if (latest_ground_ready_.has_value()) {
            ground_ready_gate_.observe(*latest_ground_ready_);
          }
          if (!was_pending) {
            RCLCPP_INFO(
              get_logger(),
              "PICO A/world reset received; waiting for the ground-aligned skeleton to resume");
          }
        } else {
          // A second A press during an active attempt cancels it.
          session_.on_world_reset();
        }
        report_phase_locked();
      });
    auto ready_qos = rclcpp::QoS(1);
    ready_qos.reliable().transient_local();
    ground_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
      ground_ready_topic_, ready_qos,
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_ground_ready_ = message->data;
        const bool was_ready = ground_ready_gate_.ready();
        ground_ready_gate_.observe(message->data);
        if (!was_ready && ground_ready_gate_.ready()) {
          reset_input_freshness_locked();
        }
      });
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50), [this]() { on_timer(); });

    configure_terminal();
    keyboard_thread_ = std::thread([this]() { keyboard_loop(); });
    RCLCPP_INFO(
      get_logger(),
      "Odin pelvis calibrator waiting for %s and %s; press PICO controller A "
      "or keyboard 'a' to start, 'q' to quit. Output: %s",
      pico_topic_.c_str(), odin_topic_.c_str(), output_path_.c_str());
  }

  ~OdinPelvisCalibratorNode() override {
    keyboard_running_.store(false);
    if (keyboard_thread_.joinable()) keyboard_thread_.join();
    restore_terminal();
  }

 private:
  void reset_input_freshness_locked() {
    pico_stationarity_.reset();
    odin_stationarity_.reset();
    last_pico_received_ = -std::numeric_limits<double>::infinity();
    last_odin_received_ = -std::numeric_limits<double>::infinity();
  }

  CalibrationSessionOptions session_options() {
    CalibrationSessionOptions options;
    options.neutral_duration_sec = declare_parameter<double>("neutral_duration_sec", 2.0);
    options.final_neutral_duration_sec =
      declare_parameter<double>("final_neutral_duration_sec", 2.0);
    options.attempt_timeout_sec = declare_parameter<double>("attempt_timeout_sec", 30.0);
    options.min_pitch_range_rad =
      declare_parameter<double>("min_pitch_range_rad", 12.0 * M_PI / 180.0);
    options.min_yaw_range_rad =
      declare_parameter<double>("min_yaw_range_rad", 12.0 * M_PI / 180.0);
    options.min_directional_excursion_ratio =
      declare_parameter<double>("min_directional_excursion_ratio", 0.20);
    options.return_upright_tolerance_rad =
      declare_parameter<double>("return_upright_tolerance_rad", 8.0 * M_PI / 180.0);
    options.max_initial_tilt_rad =
      declare_parameter<double>("max_initial_tilt_rad", 25.0 * M_PI / 180.0);
    if (!(options.neutral_duration_sec > 0.0) ||
        !(options.final_neutral_duration_sec > 0.0) ||
        !(options.attempt_timeout_sec > 0.0) ||
        !(options.min_pitch_range_rad > 0.0) ||
        !(options.min_yaw_range_rad > 0.0) ||
        !(options.min_directional_excursion_ratio > 0.0) ||
        options.min_directional_excursion_ratio > 0.5 ||
        !(options.return_upright_tolerance_rad > 0.0) ||
        !(options.max_initial_tilt_rad > 0.0)) {
      throw std::invalid_argument("invalid calibration session parameters");
    }
    return options;
  }

  double steady_seconds() const {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  void on_pico(const geometry_msgs::msg::PoseArray & message) {
    if (!is_expected_skeleton_frame(
        message.header.frame_id, expected_skeleton_frame_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring PICO skeleton in frame '%s'; expected '%s'",
        message.header.frame_id.c_str(), expected_skeleton_frame_.c_str());
      return;
    }
    if (message.poses.empty()) return;
    const double receipt = steady_seconds();
    const double source_stamp = stamp_seconds(message.header.stamp);
    HostTimedPose sample;
    try {
      sample = HostTimedPose{receipt, from_pose_msg_checked(message.poses.front())};
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Invalid PICO pelvis: %s", error.what());
      return;
    }
    if (!finite(sample.pose) || !std::isfinite(source_stamp)) return;

    std::lock_guard<std::mutex> lock(mutex_);
    const auto pico_motion = pico_stationarity_.observe(source_stamp, sample.pose);
    last_pico_received_ = receipt;
    const auto previous_phase = session_.phase();
    const auto & odin_motion = odin_stationarity_.last_observation();
    session_.add_pico(
      sample,
      pico_motion.valid_interval && pico_motion.stationary &&
      odin_motion.valid_interval && odin_motion.stationary,
      receipt);
    if (session_.phase() != previous_phase) report_phase_locked();
    solve_if_ready_locked();
  }

  void on_odin(const nav_msgs::msg::Odometry & message) {
    const double receipt = steady_seconds();
    const double source_stamp = stamp_seconds(message.header.stamp);
    HostTimedPose sample;
    try {
      sample = HostTimedPose{receipt, from_pose_msg_checked(message.pose.pose)};
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Invalid Odin pose: %s", error.what());
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const Discontinuity discontinuity = continuity_.observe(
      source_stamp, receipt, message.header.frame_id, sample.pose);
    if (discontinuity != Discontinuity::kNone) {
      odin_stationarity_.reset();
      session_.on_odin_restart();
      report_phase_locked();
      RCLCPP_WARN(
        get_logger(), "Odin stream discontinuity detected: %s",
        discontinuity_name(discontinuity));
    }
    odin_stationarity_.observe(source_stamp, sample.pose);
    last_odin_received_ = receipt;
    session_.add_odin(sample);
  }

  void on_timer() {
    if (shutdown_requested_.exchange(false)) {
      rclcpp::shutdown();
      return;
    }
    const double now = steady_seconds();
    std::lock_guard<std::mutex> lock(mutex_);
    const bool inputs_fresh =
      now - last_pico_received_ <= max_input_age_sec_ &&
        now - last_odin_received_ <= max_input_age_sec_ &&
        pico_stationarity_.last_observation().valid_interval &&
        odin_stationarity_.last_observation().valid_interval;
    const bool pending_ground_ready =
      session_.phase() != CalibrationPhase::kPendingStart ||
      ground_ready_gate_.ready();
    const bool inputs_ready = inputs_fresh && pending_ground_ready;
    if (inputs_ready) {
      const auto previous_phase = session_.phase();
      session_.observe_valid_inputs(now);
      if (previous_phase == CalibrationPhase::kPendingStart &&
          session_.phase() == CalibrationPhase::kNeutral) {
        ground_ready_gate_.consume();
      }
      if (session_.phase() != previous_phase) {
        report_phase_locked();
        RCLCPP_INFO(
          get_logger(), "Input rates: PICO %.1f Hz, Odin %.1f Hz",
          pico_stationarity_.last_observation().rate_hz,
          odin_stationarity_.last_observation().rate_hz);
      }
    } else {
      const auto previous_phase = session_.phase();
      session_.observe_invalid_inputs();
      if (session_.phase() != previous_phase) report_phase_locked();
    }
    if (keyboard_start_requested_.exchange(false)) {
      if (!session_.start(now)) {
        RCLCPP_WARN(get_logger(), "Cannot start: %s", session_.message().c_str());
      } else {
        report_phase_locked();
      }
    }
  }

  void solve_if_ready_locked() {
    if (session_.phase() != CalibrationPhase::kSolving || solving_) return;
    solving_ = true;
    const auto result = solve_extrinsics(
      session_.pico_samples(), session_.odin_samples(), solver_options_);
    if (!result.success) {
      session_.reject(result.message);
      solving_ = false;
      report_phase_locked();
      return;
    }

    ExtrinsicsDocument document;
    document.valid = true;
    document.calibrated_at = utc_now();
    document.pico_topic = pico_topic_;
    document.odin_topic = odin_topic_;
    document.pelvis_T_odin = result.pelvis_T_odin;
    document.metrics = result.metrics;
    try {
      save_extrinsics_atomic(output_path_, document);
      std::ostringstream message;
      message << "saved " << output_path_ << "; samples=" << result.metrics.sample_count
              << " receive_lag=" << result.receive_lag_sec
              << " correlation=" << result.metrics.time_correlation
              << " rot_rms=" << result.metrics.rotation_rms_rad
              << " trans_rms=" << result.metrics.translation_rms_m;
      session_.mark_saved(message.str());
    } catch (const std::exception & error) {
      session_.reject(std::string("save failed: ") + error.what());
    }
    solving_ = false;
    report_phase_locked();
  }

  void report_phase_locked() {
    RCLCPP_INFO(
      get_logger(), "Calibration phase=%s: %s", phase_name(session_.phase()),
      session_.message().c_str());
  }

  void configure_terminal() {
    if (!isatty(STDIN_FILENO)) return;
    if (tcgetattr(STDIN_FILENO, &original_terminal_) != 0) return;
    termios raw = original_terminal_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) terminal_configured_ = true;
  }

  void restore_terminal() {
    if (terminal_configured_) tcsetattr(STDIN_FILENO, TCSANOW, &original_terminal_);
  }

  void keyboard_loop() {
    while (keyboard_running_.load() && rclcpp::ok()) {
      pollfd descriptor{STDIN_FILENO, POLLIN, 0};
      if (poll(&descriptor, 1, 100) <= 0 || !(descriptor.revents & POLLIN)) continue;
      char key = '\0';
      if (read(STDIN_FILENO, &key, 1) != 1) continue;
      if (key == 'a') keyboard_start_requested_.store(true);
      if (key == 'q') shutdown_requested_.store(true);
    }
  }

  std::string pico_topic_;
  std::string odin_topic_;
  std::string expected_skeleton_frame_;
  std::string world_reset_topic_;
  std::string ground_ready_topic_;
  std::filesystem::path output_path_;
  double max_input_age_sec_{0.5};
  bool start_on_world_reset_{true};
  double stationary_linear_speed_{0.04};
  double stationary_angular_speed_{0.08};
  SolverOptions solver_options_;
  CalibrationSession session_;
  GroundReadyGate ground_ready_gate_;
  std::optional<bool> latest_ground_ready_;
  std::mutex mutex_;
  double last_pico_received_{-std::numeric_limits<double>::infinity()};
  double last_odin_received_{-std::numeric_limits<double>::infinity()};
  PoseStationarity pico_stationarity_;
  PoseStationarity odin_stationarity_;
  PoseStreamContinuity continuity_;
  bool solving_{false};
  std::atomic<bool> keyboard_start_requested_{false};
  std::atomic<bool> shutdown_requested_{false};
  std::atomic<bool> keyboard_running_{true};
  std::thread keyboard_thread_;
  termios original_terminal_{};
  bool terminal_configured_{false};
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pico_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odin_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr world_reset_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ground_ready_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace pico_odin

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pico_odin::OdinPelvisCalibratorNode>());
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
