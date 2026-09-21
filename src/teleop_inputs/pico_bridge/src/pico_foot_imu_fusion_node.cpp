#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <optional>
#include <string>
#include <stdexcept>
#include <functional>
#include <vector>
#include "pico_bridge/foot_imu_fusion_math.hpp"
#include "pico_bridge/foot_imu_input_state.hpp"
#include "pico_bridge/auto_calibration_state.hpp"

namespace {
namespace fusion = pico_bridge::fusion;
using Q = fusion::Quaternion;
using V = fusion::Vector3;
constexpr size_t kLeftKnee = 4, kRightKnee = 5;
constexpr size_t kLeftAnkle = 7, kRightAnkle = 8, kLeftFoot = 10, kRightFoot = 11;

Q average(const std::vector<Q> &values) {
  if (values.empty()) return {0, 0, 0, 1};
  Q ref = values.front(), sum{0, 0, 0, 0};
  for (Q q : values) {
    if (q[0]*ref[0] + q[1]*ref[1] + q[2]*ref[2] + q[3]*ref[3] < 0)
      for (double &v : q) v = -v;
    for (size_t i = 0; i < 4; ++i) sum[i] += q[i];
  }
  return fusion::normalize(sum);
}

bool from_msg(const geometry_msgs::msg::Quaternion &q, Q &out) {
  return fusion::try_normalize({q.x, q.y, q.z, q.w}, out);
}
void to_msg(const Q &q, geometry_msgs::msg::Quaternion &out) { out.x=q[0]; out.y=q[1]; out.z=q[2]; out.w=q[3]; }
V position(const geometry_msgs::msg::Pose &p) { return {p.position.x, p.position.y, p.position.z}; }
}

class PicoFootImuFusionNode final : public rclcpp::Node {
public:
  PicoFootImuFusionNode() : Node("pico_foot_imu_fusion") {
    pico_topic_ = declare_parameter("pico_topic", std::string("/pico/smpl"));
    left_topic_ = declare_parameter("left_imu_topic", std::string("/imu/left_feet"));
    right_topic_ = declare_parameter("right_imu_topic", std::string("/imu/right_feet"));
    left_ready_topic_ = declare_parameter("left_imu_ready_topic", std::string("/imu/left_feet/ready"));
    right_ready_topic_ = declare_parameter("right_imu_ready_topic", std::string("/imu/right_feet/ready"));
    output_topic_ = declare_parameter("output_topic", std::string("/pico/smpl_fused"));
    max_age_ = declare_parameter("max_imu_age_sec", 0.08);
    calibration_samples_ = declare_parameter("calibration_samples", 60);
    calibration_max_step_rad_ = declare_parameter("calibration_max_angular_step_rad", 0.035);
    imu_reset_settle_sec_ = declare_parameter("imu_reset_settle_sec", 1.0);
    if (imu_reset_settle_sec_ < 0.0) throw std::runtime_error("imu_reset_settle_sec must be non-negative");
    auto_calibrate_on_world_reset_ = declare_parameter("auto_calibrate_on_world_reset", true);
    left_zero_service_ = declare_parameter("left_zero_z_service", std::string("/im900/left_foot/zero_z_axis"));
    right_zero_service_ = declare_parameter("right_zero_z_service", std::string("/im900/right_foot/zero_z_axis"));
    left_mount_ = mount_param("left_mount_quaternion");
    right_mount_ = mount_param("right_mount_quaternion");

    auto qos = rclcpp::SensorDataQoS();
    pub_ = create_publisher<geometry_msgs::msg::PoseArray>(output_topic_, qos);
    ankle_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/pico/ankle_relative", qos);
    pico_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(pico_topic_, qos,
      std::bind(&PicoFootImuFusionNode::pico_callback, this, std::placeholders::_1));
    left_sub_ = create_subscription<sensor_msgs::msg::Imu>(left_topic_, qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr m) { imu_callback("left", *m); });
    right_sub_ = create_subscription<sensor_msgs::msg::Imu>(right_topic_, qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr m) { imu_callback("right", *m); });
    left_ready_sub_ = create_subscription<std_msgs::msg::Bool>(left_ready_topic_, qos,
      [this](std_msgs::msg::Bool::ConstSharedPtr m) { ready_callback("left", m->data); });
    right_ready_sub_ = create_subscription<std_msgs::msg::Bool>(right_ready_topic_, qos,
      [this](std_msgs::msg::Bool::ConstSharedPtr m) { ready_callback("right", m->data); });
    world_reset_sub_ = create_subscription<std_msgs::msg::Float32>("/pico/world_reset", rclcpp::QoS(10),
      [this](std_msgs::msg::Float32::ConstSharedPtr m) { world_reset_callback(m->data); });
    left_zero_client_ = create_client<std_srvs::srv::Trigger>(left_zero_service_);
    right_zero_client_ = create_client<std_srvs::srv::Trigger>(right_zero_service_);
    calibrate_srv_ = create_service<std_srvs::srv::Trigger>("/pico_foot_imu_fusion/calibrate",
      std::bind(&PicoFootImuFusionNode::calibrate, this, std::placeholders::_1, std::placeholders::_2));
    reset_srv_ = create_service<std_srvs::srv::Trigger>("/pico_foot_imu_fusion/reset",
      std::bind(&PicoFootImuFusionNode::reset, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  Q mount_param(const std::string &name) {
    auto p = declare_parameter<std::vector<double>>(name, {0, 0, 0, 1});
    if (p.size() != 4) throw std::runtime_error(name + " must contain 4 values");
    return fusion::normalize({p[0], p[1], p[2], p[3]});
  }

  void imu_callback(const std::string &side, const sensor_msgs::msg::Imu &msg) {
    if (msg.orientation_covariance[0] < 0.0) {
      reject_imu(side);
      warn("ignoring IMU message whose orientation is unavailable");
      return;
    }
    Q q{};
    if (!from_msg(msg.orientation, q)) {
      reject_imu(side);
      warn("ignoring invalid IMU quaternion");
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const Q corrected = fusion::multiply(fusion::multiply(side == "left" ? left_mount_ : right_mount_, q),
                                         fusion::inverse(side == "left" ? left_mount_ : right_mount_));
    (side == "left" ? left_imu_ : right_imu_).accept(corrected, now());
  }

  void reject_imu(const std::string &side) {
    std::lock_guard<std::mutex> lock(mutex_);
    (side == "left" ? left_imu_ : right_imu_).reject();
  }

  void ready_callback(const std::string &side, bool ready) {
    std::lock_guard<std::mutex> lock(mutex_);
    bool &state = side == "left" ? left_ready_ : right_ready_;
    state = ready;
    if (!ready) {
      baseline_.reset();
      calibrating_ = false;
      samples_.clear();
      auto_calibration_.fail(auto_calibration_.generation());
      cancel_settle_timer_locked();
    }
  }

  void cancel_settle_timer_locked() {
    if (settle_timer_) settle_timer_->cancel();
    settle_timer_.reset();
  }

  void world_reset_callback(float) {
    if (!auto_calibrate_on_world_reset_) return;
    std::lock_guard<std::mutex> lock(mutex_);
    cancel_settle_timer_locked();
    const auto generation = auto_calibration_.begin();
    baseline_.reset(); calibrating_ = false; samples_.clear();
    left_imu_.reject(); right_imu_.reject();
    if (!left_zero_client_->service_is_ready() || !right_zero_client_->service_is_ready()) {
      auto_calibration_.fail(generation);
      RCLCPP_WARN(get_logger(), "PICO world reset received, but IMU900 zero-Z services are unavailable");
      return;
    }
    send_zero_request(generation, true);
    RCLCPP_INFO(get_logger(), "PICO world reset received; requesting IMU900 Z-axis reset");
  }

  void send_zero_request(std::uint64_t generation, bool left_side) {
    auto client = left_side ? left_zero_client_ : right_zero_client_;
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    client->async_send_request(request, [this, generation, left_side](
      rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      zero_ack_callback(generation, left_side, future.get()->success);
    });
  }

  void zero_ack_callback(std::uint64_t generation, bool left_side, bool success) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (generation != auto_calibration_.generation() ||
        auto_calibration_.phase() != pico_bridge::AutoCalibrationPhase::kAwaitingAcks) return;
    const bool both_acknowledged = auto_calibration_.accept_zero_ack(generation, left_side, success);
    if (!success) {
      RCLCPP_WARN(get_logger(), "IMU900 %s Z-axis reset failed", left_side ? "left" : "right");
      return;
    }
    if (left_side) {
      if (!right_zero_client_->service_is_ready()) {
        auto_calibration_.fail(generation);
        RCLCPP_WARN(get_logger(), "IMU900 right Z-axis reset service became unavailable");
        return;
      }
      send_zero_request(generation, false);
      RCLCPP_INFO(get_logger(), "IMU900 left Z-axis reset acknowledged; requesting right reset");
      return;
    }
    if (!both_acknowledged) return;
    const auto settle_duration = std::chrono::duration<double>(imu_reset_settle_sec_);
    settle_timer_ = create_wall_timer(settle_duration, [this, generation]() {
      std::lock_guard<std::mutex> lock(mutex_);
      if (generation != auto_calibration_.generation() || !auto_calibration_.settling()) return;
      cancel_settle_timer_locked();
      left_imu_.reject(); right_imu_.reject();
      if (!auto_calibration_.finish_settling(generation)) return;
      RCLCPP_INFO(get_logger(), "IMU900 settling complete; waiting for fresh post-settle samples");
    });
    RCLCPP_INFO(get_logger(), "Both IMU900 Z-axis resets acknowledged; settling for %.3f seconds",
      imu_reset_settle_sec_);
  }

  std::chrono::steady_clock::time_point now() const { return std::chrono::steady_clock::now(); }

  void pico_callback(geometry_msgs::msg::PoseArray::ConstSharedPtr msg) {
    if (msg->poses.size() != 24) { warn("ignoring PICO frame without 24 poses"); return; }
    std::vector<Q> orientations(24);
    for (size_t i = 0; i < 24; ++i) {
      if (!from_msg(msg->poses[i].orientation, orientations[i])) {
        warn("ignoring PICO frame with invalid quaternion"); return;
      }
      if (!fusion::is_finite_position(position(msg->poses[i]))) {
        warn("ignoring PICO frame with non-finite position"); return;
      }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const auto t = now();
    const bool imus_fresh = left_ready_ && right_ready_ &&
      left_imu_.is_fresh(t, max_age_) && right_imu_.is_fresh(t, max_age_);
    if (auto_calibration_.awaiting_fresh_imus() && imus_fresh) {
      auto_calibration_.start_sampling(auto_calibration_.generation());
      samples_.clear(); baseline_.reset(); calibrating_ = true;
      RCLCPP_INFO(get_logger(), "Fresh post-reset IMU data received; automatic calibration started");
    }
    if (calibrating_ && imus_fresh) {
      const Q &left_imu = *left_imu_.orientation();
      const Q &right_imu = *right_imu_.orientation();
      if (!samples_.empty() && !fusion::calibration_step_is_stable(
          samples_.back().pelvis, orientations[0],
          samples_.back().left_foot, orientations[kLeftFoot],
          samples_.back().right_foot, orientations[kRightFoot],
          samples_.back().left_imu, left_imu,
          samples_.back().right_imu, right_imu,
          calibration_max_step_rad_)) {
        samples_.clear(); warn("calibration motion detected; restarting neutral sample collection");
      }
      samples_.push_back({msg->poses, orientations[0], orientations[kLeftFoot], orientations[kRightFoot], left_imu, right_imu});
      if (static_cast<int>(samples_.size()) >= calibration_samples_) finish_calibration();
    }
    latest_pico_time_ = t;
    if (!baseline_ || !imus_fresh) { warn("waiting for fresh IMU data and foot fusion calibration"); return; }
    auto out = *msg;
    const Q pelvis = orientations[0];
    const Q left = fusion::aligned_foot_orientation(pelvis, *left_imu_.orientation(), baseline_->pelvis, baseline_->left_imu, baseline_->left_foot);
    const Q right = fusion::aligned_foot_orientation(pelvis, *right_imu_.orientation(), baseline_->pelvis, baseline_->right_imu, baseline_->right_foot);
    to_msg(left, out.poses[kLeftFoot].orientation);
    to_msg(right, out.poses[kRightFoot].orientation);
    const auto lp = fusion::reconstruct_foot_position(position(out.poses[kLeftAnkle]), left, baseline_->left_offset);
    const auto rp = fusion::reconstruct_foot_position(position(out.poses[kRightAnkle]), right, baseline_->right_offset);
    out.poses[kLeftFoot].position.x=lp[0]; out.poses[kLeftFoot].position.y=lp[1]; out.poses[kLeftFoot].position.z=lp[2];
    out.poses[kRightFoot].position.x=rp[0]; out.poses[kRightFoot].position.y=rp[1]; out.poses[kRightFoot].position.z=rp[2];
    pub_->publish(out);
    geometry_msgs::msg::PoseArray ankles; ankles.header = out.header; ankles.poses.resize(2);
    to_msg(fusion::relative_ankle_orientation(orientations[kLeftKnee], left), ankles.poses[0].orientation);
    to_msg(fusion::relative_ankle_orientation(orientations[kRightKnee], right), ankles.poses[1].orientation);
    ankle_pub_->publish(ankles);
  }

  struct Sample { std::vector<geometry_msgs::msg::Pose> poses; Q pelvis, left_foot, right_foot, left_imu, right_imu; };
  struct Baseline { Q pelvis, left_foot, right_foot, left_imu, right_imu; V left_offset, right_offset; };

  void finish_calibration() {
    std::vector<Q> pelvis, left_foot, right_foot, left_imu, right_imu; V lp{0,0,0}, rp{0,0,0};
    for (const auto &s : samples_) { pelvis.push_back(s.pelvis); left_foot.push_back(s.left_foot); right_foot.push_back(s.right_foot); left_imu.push_back(s.left_imu); right_imu.push_back(s.right_imu); }
    for (const auto &s : samples_) { auto a=position(s.poses[kLeftFoot]), b=position(s.poses[kLeftAnkle]); for(int i=0;i<3;++i) lp[i] += a[i]-b[i]; a=position(s.poses[kRightFoot]); b=position(s.poses[kRightAnkle]); for(int i=0;i<3;++i) rp[i] += a[i]-b[i]; }
    Baseline b{average(pelvis), average(left_foot), average(right_foot), average(left_imu), average(right_imu), {}, {}};
    for(int i=0;i<3;++i) { lp[i]/=samples_.size(); rp[i]/=samples_.size(); }
    b.left_offset=fusion::rotate(fusion::inverse(b.left_foot), lp); b.right_offset=fusion::rotate(fusion::inverse(b.right_foot), rp);
    baseline_=b; calibrating_=false; samples_.clear();
    RCLCPP_INFO(get_logger(), "PICO foot IMU calibration complete");
  }

  void calibrate(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> r) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto t = now();
    if (!left_ready_ || !right_ready_ || !left_imu_.is_fresh(t, max_age_) || !right_imu_.is_fresh(t, max_age_) ||
        latest_pico_time_.time_since_epoch().count() == 0 ||
        std::chrono::duration<double>(t-latest_pico_time_).count() > max_age_) {
      r->success=false; r->message="等待有效且新鲜的 PICO/左右 IMU 数据"; return;
    }
    auto_calibration_.cancel(); cancel_settle_timer_locked();
    samples_.clear(); baseline_.reset(); calibrating_=true; r->success=true;
    r->message="开始标定，请保持站立双脚平放 " + std::to_string(calibration_samples_) + " 帧";
  }
  void reset(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> r) {
    std::lock_guard<std::mutex> lock(mutex_); calibrating_=false; samples_.clear(); baseline_.reset(); auto_calibration_.cancel(); cancel_settle_timer_locked(); r->success=true; r->message="融合标定已清除";
  }

  void warn(const std::string &message) {
    const auto t = now();
    if (last_warning_.time_since_epoch().count() == 0 || std::chrono::duration<double>(t-last_warning_).count() > 2.0) {
      RCLCPP_WARN(get_logger(), "%s", message.c_str()); last_warning_=t;
    }
  }

  std::string pico_topic_, left_topic_, right_topic_, left_ready_topic_, right_ready_topic_, output_topic_, left_zero_service_, right_zero_service_; double max_age_, calibration_max_step_rad_, imu_reset_settle_sec_; int calibration_samples_; bool auto_calibrate_on_world_reset_{true};
  Q left_mount_, right_mount_; std::mutex mutex_; fusion::FootImuInputState left_imu_, right_imu_; std::chrono::steady_clock::time_point latest_pico_time_, last_warning_;
  bool calibrating_{false}; bool left_ready_{false}; bool right_ready_{false}; std::vector<Sample> samples_; std::optional<Baseline> baseline_;
  pico_bridge::AutoCalibrationState auto_calibration_;
  rclcpp::TimerBase::SharedPtr settle_timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_, ankle_pub_; rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pico_sub_; rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr left_sub_, right_sub_; rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr left_ready_sub_, right_ready_sub_; rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr world_reset_sub_; rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr left_zero_client_, right_zero_client_; rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_srv_, reset_srv_;
};

int main(int argc, char **argv) { rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<PicoFootImuFusionNode>()); rclcpp::shutdown(); return 0; }
