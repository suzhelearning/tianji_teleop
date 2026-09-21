#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <filesystem>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include "pico_odin/extrinsics_file.hpp"
#include "pico_odin/host_time_synchronizer.hpp"
#include "pico_odin/pose_stationarity.hpp"
#include "pico_odin/pose_stream_continuity.hpp"
#include "pico_odin/runtime_alignment.hpp"
#include "pico_odin/skeleton_frame_contract.hpp"

namespace pico_odin {
namespace {

enum class SkeletonKind {
  kCanonical,
  kFootFused,
};

enum class OdinRate {
  kLowFrequency,
  kHighFrequency,
};

struct PendingSkeleton {
  geometry_msgs::msg::PoseArray message;
  std::vector<Pose3> joints;
  double receipt_sec{0.0};
  SkeletonKind kind{SkeletonKind::kCanonical};
};

double stamp_seconds(const builtin_interfaces::msg::Time & stamp) {
  return static_cast<double>(stamp.sec) + 1e-9 * static_cast<double>(stamp.nanosec);
}

double steady_seconds() {
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

class OdinPelvisRuntimeNode final : public rclcpp::Node {
 public:
  OdinPelvisRuntimeNode() : Node("odin_pelvis_runtime") {
    const auto configured_path = declare_parameter<std::string>("extrinsics_file", "");
    extrinsics_path_ = configured_path.empty() ? default_extrinsics_path() :
      std::filesystem::path(configured_path);
    alignment_samples_ = declare_parameter<int>("alignment_samples", 30);
    max_sync_gap_sec_ = declare_parameter<double>("max_sync_gap_sec", 0.05);
    max_pending_age_sec_ = declare_parameter<double>("max_pending_age_sec", 0.25);
    receive_lag_update_period_sec_ =
      declare_parameter<double>("receive_lag_update_period_sec", 0.5);
    stationary_linear_speed_ = declare_parameter<double>("stationary_linear_speed_mps", 0.04);
    stationary_angular_speed_ = declare_parameter<double>("stationary_angular_speed_radps", 0.08);
    if (alignment_samples_ < 2 || !std::isfinite(max_sync_gap_sec_) ||
        !(max_sync_gap_sec_ > 0.0) || !std::isfinite(max_pending_age_sec_) ||
        !(max_pending_age_sec_ > 0.0) ||
        !std::isfinite(receive_lag_update_period_sec_) ||
        !(receive_lag_update_period_sec_ > 0.0) ||
        !std::isfinite(stationary_linear_speed_) ||
        !std::isfinite(stationary_angular_speed_) ||
        stationary_linear_speed_ < 0.0 || stationary_angular_speed_ < 0.0) {
      throw std::invalid_argument("invalid runtime alignment parameters");
    }
    PoseStationarityOptions stationarity_options;
    stationarity_options.maximum_linear_speed_mps = stationary_linear_speed_;
    stationarity_options.maximum_angular_speed_radps = stationary_angular_speed_;
    pico_alignment_stationarity_ = PoseStationarity(stationarity_options);
    odin_alignment_stationarity_ = PoseStationarity(stationarity_options);
    HostTimeSynchronizerOptions synchronizer_options;
    synchronizer_options.history_sec =
      declare_parameter<double>("host_sync_history_sec", 6.0);
    synchronizer_options.max_sync_gap_sec = max_sync_gap_sec_;
    synchronizer_options.lag_smoothing_alpha =
      declare_parameter<double>("receive_lag_smoothing_alpha", 0.20);
    synchronizer_options.solver_options.max_receive_lag_sec =
      declare_parameter<double>("max_receive_lag_sec", 0.30);
    synchronizer_options.solver_options.receive_lag_step_sec =
      declare_parameter<double>("receive_lag_step_sec", 0.002);
    synchronizer_options.solver_options.min_receive_lag_correlation =
      declare_parameter<double>("min_receive_lag_correlation", 0.25);
    const int min_receive_lag_pairs =
      declare_parameter<int>("min_receive_lag_pairs", 10);
    if (min_receive_lag_pairs < 3) {
      throw std::invalid_argument("min_receive_lag_pairs must be at least 3");
    }
    synchronizer_options.solver_options.min_receive_lag_pairs =
      static_cast<std::size_t>(min_receive_lag_pairs);
    synchronizer_ = HostTimeSynchronizer(synchronizer_options);
    output_frame_ = declare_parameter<std::string>("output_frame", "pico_ground");
    expected_skeleton_frame_ = declare_parameter<std::string>(
      "expected_skeleton_frame", "pico_ground");
    if (expected_skeleton_frame_.empty()) {
      throw std::invalid_argument("expected_skeleton_frame must not be empty");
    }
    raw_odin_topic_ = declare_parameter<std::string>(
      "raw_odin_topic", "/raw/odom/odin");
    raw_odin_highfreq_topic_ = declare_parameter<std::string>(
      "raw_odin_highfreq_topic", "/raw/odom/odin_highfreq");
    pico_smpl_topic_ = declare_parameter<std::string>("pico_smpl_topic", "/pico/smpl");
    pico_smpl_fused_topic_ = declare_parameter<std::string>(
      "pico_smpl_fused_topic", "/pico/smpl_fused");
    world_reset_topic_ = declare_parameter<std::string>(
      "world_reset_topic", "/pico/world_reset");
    pelvis_output_topic_ = declare_parameter<std::string>(
      "pelvis_output_topic", "/calibrated/odom/pelvis");
    pelvis_highfreq_output_topic_ = declare_parameter<std::string>(
      "pelvis_highfreq_output_topic", "/calibrated/odom/pelvis_highfreq");
    smpl_odin_output_topic_ = declare_parameter<std::string>(
      "smpl_odin_output_topic", "/pico/smpl_odin");
    smpl_fused_odin_output_topic_ = declare_parameter<std::string>(
      "smpl_fused_odin_output_topic", "/pico/smpl_fused_odin");
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
    high_frequency_continuity_ = PoseStreamContinuity(continuity_options);
    low_frequency_continuity_ = PoseStreamContinuity(continuity_options);

    try {
      const auto document = load_extrinsics(extrinsics_path_);
      alignment_.emplace(document.pelvis_T_odin);
      RCLCPP_INFO(get_logger(), "Loaded Odin pelvis extrinsics: %s", extrinsics_path_.c_str());
      if (document.pico_topic != pico_smpl_topic_ ||
          document.odin_topic != raw_odin_highfreq_topic_) {
        RCLCPP_WARN(
          get_logger(),
          "Extrinsics were calibrated from PICO=%s Odin=%s, runtime uses PICO=%s Odin=%s",
          document.pico_topic.c_str(), document.odin_topic.c_str(),
          pico_smpl_topic_.c_str(), raw_odin_highfreq_topic_.c_str());
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Pelvis correction disabled; cannot load %s: %s",
        extrinsics_path_.c_str(), error.what());
    }

    const auto sensor_qos = rclcpp::SensorDataQoS();
    pelvis_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      pelvis_output_topic_, sensor_qos);
    pelvis_hf_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      pelvis_highfreq_output_topic_, sensor_qos);
    smpl_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
      smpl_odin_output_topic_, sensor_qos);
    smpl_fused_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
      smpl_fused_odin_output_topic_, sensor_qos);

    odin_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      raw_odin_topic_, sensor_qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        on_odin(*message, OdinRate::kLowFrequency);
      });
    odin_hf_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      raw_odin_highfreq_topic_, sensor_qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        on_odin(*message, OdinRate::kHighFrequency);
      });
    smpl_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      pico_smpl_topic_, sensor_qos,
      [this](geometry_msgs::msg::PoseArray::ConstSharedPtr message) {
        on_skeleton(*message, SkeletonKind::kCanonical);
      });
    smpl_fused_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      pico_smpl_fused_topic_, sensor_qos,
      [this](geometry_msgs::msg::PoseArray::ConstSharedPtr message) {
        on_skeleton(*message, SkeletonKind::kFootFused);
      });
    world_reset_sub_ = create_subscription<std_msgs::msg::Float32>(
      world_reset_topic_, 10,
      [this](std_msgs::msg::Float32::ConstSharedPtr) {
        std::lock_guard<std::mutex> lock(mutex_);
        clear_session_locked("PICO world reset");
      });
  }

 private:
  void clear_session_locked(const char * reason) {
    if (alignment_) alignment_->clear();
    synchronizer_.reset();
    canonical_pending_.clear();
    fused_pending_.clear();
    pico_alignment_samples_.clear();
    odin_alignment_samples_.clear();
    pico_alignment_stationarity_.reset();
    odin_alignment_stationarity_.reset();
    canonical_skeleton_received_sec_ = -std::numeric_limits<double>::infinity();
    last_receive_lag_update_sec_ = -std::numeric_limits<double>::infinity();
    RCLCPP_INFO(get_logger(), "Pelvis session alignment cleared: %s", reason);
  }

  void on_odin(const nav_msgs::msg::Odometry & message, OdinRate rate) {
    if (!alignment_) return;
    Pose3 pose;
    try {
      pose = from_pose_msg_checked(message.pose.pose);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Invalid Odin odometry: %s", error.what());
      return;
    }
    const double source_stamp = stamp_seconds(message.header.stamp);
    const double receipt = steady_seconds();
    std::lock_guard<std::mutex> lock(mutex_);
    const bool high_frequency = rate == OdinRate::kHighFrequency;
    auto & continuity = high_frequency ?
      high_frequency_continuity_ : low_frequency_continuity_;
    const Discontinuity discontinuity = continuity.observe(
      source_stamp, receipt, message.header.frame_id, pose);
    if (discontinuity != Discontinuity::kNone) {
      clear_session_locked(discontinuity_name(discontinuity));
      RCLCPP_WARN(
        get_logger(), "Odin %s stream discontinuity detected: %s; reacquiring alignment",
        high_frequency ? "high-frequency" : "low-frequency",
        discontinuity_name(discontinuity));
    }
    if (high_frequency) {
      synchronizer_.add_odin({receipt, pose});
      if (receipt - last_receive_lag_update_sec_ >= receive_lag_update_period_sec_) {
        last_receive_lag_update_sec_ = receipt;
        if (synchronizer_.update_receive_lag()) {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "PICO/Odin host receive lag %.4f s (correlation %.3f)",
            synchronizer_.receive_lag_sec(),
            synchronizer_.receive_lag_correlation());
        }
      }
      drain_pending_locked(receipt);
    }
    if (!alignment_->initialized()) return;
    publish_corrected_odom_locked(
      message, pose, high_frequency ? pelvis_hf_pub_ : pelvis_pub_);
  }

  void publish_corrected_odom_locked(
    const nav_msgs::msg::Odometry & source,
    const Pose3 & odin_pose,
    const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & publisher)
  {
    nav_msgs::msg::Odometry output = source;
    output.header.frame_id = output_frame_;
    output.child_frame_id = "pelvis";
    output.pose.pose = to_pose_msg(alignment_->corrected_pelvis(odin_pose));

    const Pose3 & pelvis_T_odin = alignment_->pelvis_T_odin();
    const Pose3 odin_T_pelvis = inverse(pelvis_T_odin);
    const Eigen::Matrix3d session_rotation =
      alignment_->session_transform().rotation.toRotationMatrix();
    Eigen::Matrix<double, 6, 6> pose_jacobian =
      Eigen::Matrix<double, 6, 6>::Zero();
    pose_jacobian.block<3, 3>(0, 0) = session_rotation;
    pose_jacobian.block<3, 3>(0, 3) =
      -session_rotation * skew_symmetric(
        odin_pose.rotation * odin_T_pelvis.translation);
    pose_jacobian.block<3, 3>(3, 3) = session_rotation;
    output.pose.covariance = transform_covariance(
      source.pose.covariance, pose_jacobian);

    const Eigen::Vector3d omega_odin(
      source.twist.twist.angular.x, source.twist.twist.angular.y,
      source.twist.twist.angular.z);
    const Eigen::Vector3d linear_odin(
      source.twist.twist.linear.x, source.twist.twist.linear.y,
      source.twist.twist.linear.z);
    const Eigen::Vector3d omega_pelvis = pelvis_T_odin.rotation * omega_odin;
    const Eigen::Vector3d linear_pelvis =
      pelvis_T_odin.rotation * linear_odin - omega_pelvis.cross(pelvis_T_odin.translation);
    output.twist.twist.linear.x = linear_pelvis.x();
    output.twist.twist.linear.y = linear_pelvis.y();
    output.twist.twist.linear.z = linear_pelvis.z();
    output.twist.twist.angular.x = omega_pelvis.x();
    output.twist.twist.angular.y = omega_pelvis.y();
    output.twist.twist.angular.z = omega_pelvis.z();
    output.twist.covariance = transform_covariance(
      source.twist.covariance, twist_adjoint(pelvis_T_odin));
    publisher->publish(output);
  }

  bool alignment_pair_stationary(
    const HostTimedPose & current,
    const Pose3 & current_odin)
  {
    const auto pico_motion =
      pico_alignment_stationarity_.observe(current.receipt_sec, current.pose);
    const auto odin_motion =
      odin_alignment_stationarity_.observe(current.receipt_sec, current_odin);
    return pico_motion.valid_interval && pico_motion.stationary &&
           odin_motion.valid_interval && odin_motion.stationary;
  }

  void on_skeleton(
    const geometry_msgs::msg::PoseArray & message,
    SkeletonKind kind)
  {
    if (!is_expected_skeleton_frame(
        message.header.frame_id, expected_skeleton_frame_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring PICO skeleton in frame '%s'; expected '%s'",
        message.header.frame_id.c_str(), expected_skeleton_frame_.c_str());
      return;
    }
    if (!alignment_ || message.poses.size() != 24) return;
    std::vector<Pose3> joints;
    joints.reserve(message.poses.size());
    try {
      for (const auto & pose : message.poses) joints.push_back(from_pose_msg_checked(pose));
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Invalid PICO skeleton: %s", error.what());
      return;
    }
    const double receipt = steady_seconds();
    std::lock_guard<std::mutex> lock(mutex_);
    const bool canonical = kind == SkeletonKind::kCanonical;
    if (canonical) canonical_skeleton_received_sec_ = receipt;
    if (canonical || receipt - canonical_skeleton_received_sec_ >= 0.5) {
      synchronizer_.add_pico({receipt, joints.front()});
    }
    auto & pending = canonical ? canonical_pending_ : fused_pending_;
    pending.push_back(PendingSkeleton{message, std::move(joints), receipt, kind});
    drain_pending_locked(receipt);
  }

  void process_skeleton_locked(
    const PendingSkeleton & pending,
    const Pose3 & odin_pose)
  {
    const HostTimedPose pico_sample{pending.receipt_sec, pending.joints.front()};

    if (!alignment_->initialized()) {
      if (pending.kind == SkeletonKind::kFootFused &&
          pending.receipt_sec - canonical_skeleton_received_sec_ < 0.5) return;
      if (!alignment_pair_stationary(pico_sample, odin_pose)) {
        pico_alignment_samples_.clear();
        odin_alignment_samples_.clear();
      } else {
        pico_alignment_samples_.push_back(pico_sample.pose);
        odin_alignment_samples_.push_back(odin_pose);
      }
      if (static_cast<int>(pico_alignment_samples_.size()) >= alignment_samples_) {
        alignment_->initialize(
          mean_pose(pico_alignment_samples_), mean_pose(odin_alignment_samples_));
        pico_alignment_samples_.clear();
        odin_alignment_samples_.clear();
        RCLCPP_INFO(
          get_logger(), "Pelvis session alignment initialized; initial height %.3f m",
          pico_sample.pose.translation.z());
      } else {
        return;
      }
    }

    const Pose3 corrected_root = alignment_->corrected_pelvis(odin_pose);
    const auto anchored = alignment_->anchor_skeleton(pending.joints, corrected_root);
    geometry_msgs::msg::PoseArray output = pending.message;
    output.header.frame_id = output_frame_;
    for (std::size_t index = 0; index < anchored.size(); ++index) {
      output.poses[index] = to_pose_msg(anchored[index]);
    }
    (pending.kind == SkeletonKind::kFootFused ? smpl_fused_pub_ : smpl_pub_)
      ->publish(output);
  }

  void drain_queue_locked(
    std::deque<PendingSkeleton> & queue,
    double current_receipt)
  {
    while (!queue.empty() &&
           current_receipt - queue.front().receipt_sec > max_pending_age_sec_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping stale pending PICO skeleton");
      queue.pop_front();
    }
    const auto latest_odin = synchronizer_.latest_odin_receipt();
    if (!latest_odin) return;
    while (!queue.empty()) {
      const auto & pending = queue.front();
      if (synchronizer_.target_odin_receipt(pending.receipt_sec) > *latest_odin) {
        break;
      }
      const auto odin_pose = synchronizer_.interpolate_for_pico(pending.receipt_sec);
      if (!odin_pose) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Dropping PICO skeleton without a valid host-time Odin bracket");
        queue.pop_front();
        continue;
      }
      process_skeleton_locked(pending, *odin_pose);
      queue.pop_front();
    }
  }

  void drain_pending_locked(double current_receipt) {
    drain_queue_locked(canonical_pending_, current_receipt);
    drain_queue_locked(fused_pending_, current_receipt);
  }

  std::filesystem::path extrinsics_path_;
  int alignment_samples_{30};
  double max_sync_gap_sec_{0.05};
  double max_pending_age_sec_{0.25};
  double receive_lag_update_period_sec_{0.5};
  double stationary_linear_speed_{0.04};
  double stationary_angular_speed_{0.08};
  std::string output_frame_{"pico_ground"};
  std::string expected_skeleton_frame_{"pico_ground"};
  std::string raw_odin_topic_, raw_odin_highfreq_topic_;
  std::string pico_smpl_topic_, pico_smpl_fused_topic_, world_reset_topic_;
  std::string pelvis_output_topic_, pelvis_highfreq_output_topic_;
  std::string smpl_odin_output_topic_, smpl_fused_odin_output_topic_;
  std::optional<RuntimeAlignment> alignment_;
  std::mutex mutex_;
  HostTimeSynchronizer synchronizer_;
  std::deque<PendingSkeleton> canonical_pending_;
  std::deque<PendingSkeleton> fused_pending_;
  std::vector<Pose3> pico_alignment_samples_;
  std::vector<Pose3> odin_alignment_samples_;
  PoseStationarity pico_alignment_stationarity_;
  PoseStationarity odin_alignment_stationarity_;
  double canonical_skeleton_received_sec_{-
    std::numeric_limits<double>::infinity()};
  double last_receive_lag_update_sec_{-
    std::numeric_limits<double>::infinity()};
  PoseStreamContinuity high_frequency_continuity_;
  PoseStreamContinuity low_frequency_continuity_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pelvis_pub_, pelvis_hf_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr smpl_pub_, smpl_fused_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odin_sub_, odin_hf_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr smpl_sub_, smpl_fused_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr world_reset_sub_;
};

}  // namespace pico_odin

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pico_odin::OdinPelvisRuntimeNode>());
  rclcpp::shutdown();
  return 0;
}
