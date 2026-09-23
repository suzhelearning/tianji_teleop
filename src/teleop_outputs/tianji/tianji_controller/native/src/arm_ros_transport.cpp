#include "tianji_qp_ik/arm_ros_transport.hpp"

#include "tianji_qp_ik/so3.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tianji_interfaces/msg/controller_joint_targets.hpp>
#include <tianji_interfaces/msg/pico_arm_input.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>
#include <time.h>
#include <unordered_set>
#include <utility>

namespace tianji_qp_ik {
namespace {
using PicoMessage = tianji_interfaces::msg::PicoArmInput;
using JointMessage = tianji_interfaces::msg::ControllerJointTargets;
using PublisherId = std::array<std::uint8_t, RMW_GID_STORAGE_SIZE>;

std::int64_t monotonicNowNs() noexcept {
  timespec now{};
  if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) return 0;
  return static_cast<std::int64_t>(now.tv_sec) * 1000000000LL + now.tv_nsec;
}

bool canonicalUuid(const std::string& value) noexcept {
  if (value.size() != 36U) return false;
  for (std::size_t i = 0; i < value.size(); ++i) {
    if (i == 8U || i == 13U || i == 18U || i == 23U) {
      if (value[i] != '-') return false;
    } else if (!((value[i] >= '0' && value[i] <= '9') ||
                 (value[i] >= 'a' && value[i] <= 'f'))) {
      return false;
    }
  }
  return value != "00000000-0000-0000-0000-000000000000";
}

std::string readUuid(const char* path) {
  std::ifstream stream(path);
  std::string value;
  if (!(stream >> value) || !canonicalUuid(value))
    throw std::runtime_error(std::string("cannot read local UUID: ") + path);
  return value;
}

bool fresh(std::int64_t stamp, std::int64_t now, std::int64_t limit) noexcept {
  return stamp > 0 && now >= stamp && now - stamp <= limit;
}

bool quaternion(const geometry_msgs::msg::Quaternion& value,
                Eigen::Quaterniond& result) noexcept {
  result = Eigen::Quaterniond(value.w, value.x, value.y, value.z);
  if (!result.coeffs().allFinite() ||
      std::abs(result.norm() - 1.0) > 1e-3) return false;
  result.normalize();
  return true;
}

bool pose(const geometry_msgs::msg::Pose& value, Pose& result) noexcept {
  result.position = Eigen::Vector3d(value.position.x, value.position.y, value.position.z);
  Eigen::Quaterniond rotation;
  if (!result.position.allFinite() || !quaternion(value.orientation, rotation)) return false;
  result.rotation = rotation.toRotationMatrix();
  return isProperRotation(result.rotation);
}

ArmDirectionReference direction(const geometry_msgs::msg::Vector3& value,
                                bool valid) noexcept {
  ArmDirectionReference result;
  if (!valid) return result;
  result.direction = Eigen::Vector3d(value.x, value.y, value.z);
  const double norm = result.direction.norm();
  if (!result.direction.allFinite() || !std::isfinite(norm) || norm <= 1e-9) {
    result.direction.setZero();
    return result;
  }
  result.direction /= norm;
  result.valid = true;
  result.source = ArmDirectionReferenceSource::kPico;
  return result;
}

bool decode(const PicoMessage& message, PicoTeleopFrame& frame) noexcept {
  if (message.sequence == 0U || message.tracking_epoch == 0U ||
      message.source_timestamp_ns <= 0 || message.published_monotonic_ns <= 0 ||
      (message.upper_limb_rotations_valid && !message.upper_limb_valid)) return false;
  frame.sequence = message.sequence;
  frame.tracking_epoch = message.tracking_epoch;
  frame.source_timestamp_ns = message.source_timestamp_ns;
  frame.bridge_send_monotonic_ns = message.published_monotonic_ns;
  frame.user_button_pressed = message.user_button_pressed;
  if (!pose(message.left_target, frame.left) || !pose(message.right_target, frame.right))
    return false;
  frame.left_arm_direction = direction(message.left_arm_direction, message.left_arm_direction_valid);
  frame.right_arm_direction = direction(message.right_arm_direction, message.right_arm_direction_valid);
  frame.upper_limb_skeleton.valid = message.upper_limb_valid;
  frame.upper_limb_skeleton.rotations_valid = message.upper_limb_rotations_valid;
  for (std::size_t i = 0; i < kPicoUpperLimbPointCount; ++i) {
    if (message.upper_limb_valid) {
      const auto& point = message.upper_limb_points[i];
      frame.upper_limb_skeleton.points[i] = Eigen::Vector3d(point.x, point.y, point.z);
      if (!frame.upper_limb_skeleton.points[i].allFinite()) return false;
    }
    if (message.upper_limb_rotations_valid &&
        !quaternion(message.upper_limb_rotations[i], frame.upper_limb_skeleton.rotations[i]))
      return false;
  }
  return true;
}
}  // namespace

struct ArmRosTransport::Impl {
  Impl(ArmRosTransportOptions options_in, LatestSpscExchange<PicoTeleopFrame>& input_in)
      : options(std::move(options_in)), input(input_in),
        gate(options.max_position_jump_m, options.max_orientation_jump_rad,
             options.reject_pose_jumps),
        boot_id(readUuid("/proc/sys/kernel/random/boot_id")),
        session_id(readUuid("/proc/sys/kernel/random/uuid")) {
    if (options.pico_topic.empty() || !std::isfinite(options.freshness_seconds) ||
        options.freshness_seconds <= 0.0)
      throw std::invalid_argument("ROS arm input topic and positive freshness are required");
    freshness_ns = static_cast<std::int64_t>(std::min(0.300, options.freshness_seconds) * 1e9);
    node = std::make_shared<rclcpp::Node>("tianji_arm_core");
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    subscription = node->create_subscription<PicoMessage>(options.pico_topic, qos,
        [this](PicoMessage::ConstSharedPtr message, const rclcpp::MessageInfo& info) {
          receive(*message, info);
        });
    if (!options.joint_target_topic.empty()) {
      publisher = node->create_publisher<JointMessage>(options.joint_target_topic, qos);
      outgoing.boot_id = boot_id;
      outgoing.session_id = session_id;
    }
    counters.source_valid = false;
    read_stats.source_valid = false;
    timer = node->create_wall_timer(std::chrono::milliseconds(1), [this] { tick(); });
    executor.add_node(node);
  }

  ~Impl() { stop(); }

  void start() {
    if (running.exchange(true, std::memory_order_acq_rel)) return;
    try {
      worker = std::thread([this] {
        try {
          executor.spin();
          if (!stopping.load(std::memory_order_acquire))
            throw std::runtime_error("ROS arm executor stopped unexpectedly");
        } catch (const std::exception& error) {
          failed.store(true, std::memory_order_release);
          invalidate("executor_failure");
          std::cerr << "arm_ros_transport_error=" << error.what() << std::endl;
        } catch (...) {
          failed.store(true, std::memory_order_release);
          invalidate("executor_failure");
        }
        running.store(false, std::memory_order_release);
      });
    } catch (...) {
      running.store(false, std::memory_order_release);
      throw;
    }
    std::cout << "pico_ros_topic=" << subscription->get_topic_name()
              << " pico_ros_state=ready node=" << node->get_fully_qualified_name() << std::endl;
    if (publisher)
      std::cout << "joint_target_topic=" << publisher->get_topic_name()
                << " joint_command_state=ready transport=ros2 boot_id=" << boot_id
                << " session_id=" << session_id << std::endl;
  }

  void stop() noexcept {
    stopping.store(true, std::memory_order_release);
    // The timer drains the final control-thread revocation before cancelling.
    // No middleware publish is performed by the caller/control thread.
    if (!running.load(std::memory_order_acquire) || failed.load(std::memory_order_acquire))
      executor.cancel();
    if (worker.joinable()) worker.join();
  }

  bool singleton(const PublisherId* received = nullptr) {
    const auto endpoints = node->get_publishers_info_by_topic(subscription->get_topic_name());
    if (endpoints.size() != 1U || endpoints.front().node_name() != "pico_arm_input" ||
        endpoints.front().node_namespace() != "/") return false;
    return received == nullptr || endpoints.front().endpoint_gid() == *received;
  }

  void publishStats() noexcept { (void)stats_exchange.publish(counters); }

  void advanceGeneration(std::int64_t boundary_ns) noexcept {
    ++counters.resynchronizations;
    input_after_ns = std::max(input_after_ns, boundary_ns);
    revoked_flags.fetch_or(static_cast<std::uint8_t>(published_ready_flags & kJointCommandArmsReadyFlag),
                           std::memory_order_release);
  }

  void invalidate(const char* reason) noexcept {
    if (!counters.source_valid) return;
    counters.source_valid = false;
    advanceGeneration(monotonicNowNs());
    PicoTeleopFrame revoked;
    revoked.valid = false;
    revoked.tracking_epoch = counters.tracking_epoch;
    revoked.sequence = counters.sequence;
    revoked.resynchronization_generation = counters.resynchronizations;
    revoked.receive_monotonic_ns = monotonicNowNs();
    publishStats();
    (void)input.publish(revoked);
    // Logging and middleware work are confined to this executor thread.
    RCLCPP_WARN(node->get_logger(), "PICO input revoked: %s", reason);
  }

  void receive(const PicoMessage& message, const rclcpp::MessageInfo& info) {
    ++counters.datagrams;
    const auto now = monotonicNowNs();
    PublisherId received{};
    std::copy_n(info.get_rmw_message_info().publisher_gid.data, received.size(), received.begin());
    if (!singleton(&received) || message.boot_id != boot_id ||
        !canonicalUuid(message.session_id) ||
        !fresh(message.published_monotonic_ns, now, freshness_ns)) {
      ++counters.malformed;
      invalidate("publisher_identity_or_publication_age");
      publishStats();
      return;
    }
    if (bound && received != publisher_id && message.session_id == input_session) {
      ++counters.malformed;
      invalidate("session_reused_by_another_publisher");
      publishStats();
      return;
    }
    const bool identity_changed = !bound || received != publisher_id || message.session_id != input_session;
    if (identity_changed) {
      if (retired_sessions.count(message.session_id) != 0U ||
          message.published_monotonic_ns <= last_publication_ns) {
        ++counters.reordered;
        invalidate("retired_publisher");
        publishStats();
        return;
      }
      if (bound) {
        retired_sessions.insert(input_session);
        // An identity transition is persistent even if no invalid sample was received.
        advanceGeneration(message.published_monotonic_ns);
      }
      bound = true;
      publisher_id = received;
      input_session = message.session_id;
      gate.reset();
      previous_source_ns = 0;
      previous_sequence = 0U;
      previous_epoch = 0U;
      previous_revocation_generation = message.revocation_generation;
      source_offset_ns = std::numeric_limits<std::int64_t>::max();
    }
    if (message.published_monotonic_ns <= last_publication_ns ||
        message.sequence <= previous_sequence) {
      ++counters.reordered;
      invalidate("publication_or_sequence_rollback");
      publishStats();
      return;
    }
    last_publication_ns = message.published_monotonic_ns;
    previous_sequence = message.sequence;
    if (message.revocation_generation < previous_revocation_generation) {
      ++counters.reordered;
      invalidate("revocation_generation_rollback");
      publishStats();
      return;
    }
    if (message.revocation_generation != previous_revocation_generation) {
      previous_revocation_generation = message.revocation_generation;
      advanceGeneration(message.published_monotonic_ns);
      gate.reset();
    }
    if (!message.valid) {
      invalidate("source_invalid");
      publishStats();
      return;
    }
    PicoTeleopFrame frame;
    if (!decode(message, frame)) {
      ++counters.malformed;
      invalidate("invalid_pose_or_metadata");
      publishStats();
      return;
    }
    const bool epoch_changed = previous_epoch != frame.tracking_epoch;
    if (previous_epoch != 0U && frame.tracking_epoch < previous_epoch) {
      ++counters.reordered;
      invalidate("tracking_epoch_rollback");
      publishStats();
      return;
    }
    if (epoch_changed) {
      if (previous_epoch != 0U) advanceGeneration(message.published_monotonic_ns);
      previous_source_ns = 0;
      source_offset_ns = std::numeric_limits<std::int64_t>::max();
    }
    if (previous_source_ns > 0 && frame.source_timestamp_ns <= previous_source_ns) {
      ++counters.reordered;
      invalidate("source_clock_rollback_or_repeat");
      publishStats();
      return;
    }
    if (previous_source_ns > 0 &&
        frame.source_timestamp_ns - previous_source_ns >
            now - previous_source_receive_ns + freshness_ns) {
      ++counters.malformed;
      invalidate("source_clock_forward_jump");
      publishStats();
      return;
    }
    // PICO uses another clock. Estimate only elapsed-clock lag from the best
    // observed offset; never replace its timestamp with the host clock.
    const auto offset = now - frame.source_timestamp_ns;
    source_offset_ns = std::min(source_offset_ns, offset);
    if (static_cast<long double>(offset) - static_cast<long double>(source_offset_ns) >
        static_cast<long double>(freshness_ns)) {
      ++counters.malformed;
      invalidate("source_clock_stalled");
      publishStats();
      return;
    }
    if (previous_source_ns > 0)
      counters.input_frequency_hz = 1e9 / static_cast<double>(frame.source_timestamp_ns - previous_source_ns);
    previous_source_ns = frame.source_timestamp_ns;
    previous_source_receive_ns = now;
    previous_epoch = frame.tracking_epoch;
    frame.receive_monotonic_ns = now;
    const auto decision = gate.evaluate(frame);
    if (!decision.accepted) {
      if (decision.reason == PicoStreamRejectReason::kPositionJump ||
          decision.reason == PicoStreamRejectReason::kOrientationJump)
        ++counters.jump_rejections;
      else
        ++counters.reordered;
      publishStats();
      return;
    }
    if (decision.epoch_changed) ++counters.epoch_resets;
    if (decision.stream_discontinuity) advanceGeneration(message.published_monotonic_ns);
    frame.stream_discontinuity = decision.stream_discontinuity;
    frame.resynchronization_generation = counters.resynchronizations;
    counters.tracking_epoch = frame.tracking_epoch;
    counters.sequence = frame.sequence;
    counters.latest_receive_monotonic_ns = now;
    counters.source_valid = true;
    ++counters.accepted;
    publishStats();
    if (input.publish(frame) == LatestPublishResult::kSuperseded) ++counters.superseded;
  }

  void tick() {
    const auto now = monotonicNowNs();
    if (now - last_graph_check_ns >= 20000000) {
      last_graph_check_ns = now;
      if (!singleton(bound ? &publisher_id : nullptr)) invalidate("publisher_missing_or_ambiguous");
    }
    if (counters.source_valid && !fresh(last_publication_ns, now, freshness_ns))
      invalidate("input_expired");
    publishOutput(now);
    if (stopping.load(std::memory_order_acquire)) executor.cancel();
  }

  void publishOutput(std::int64_t now) {
    if (!publisher) return;
    JointCommandFrame frame;
    if (!output.tryReadLatest(frame)) return;
    if (frame.sequence <= published_sequence || frame.source_timestamp_ns <= published_production_ns)
      throw std::runtime_error("joint output sequence/production clock rollback");
    published_sequence = frame.sequence;
    published_production_ns = frame.source_timestamp_ns;
    frame.flags = static_cast<std::uint8_t>(frame.flags & ~revoked_flags.load(std::memory_order_acquire));
    if (!fresh(frame.source_timestamp_ns, now, 100000000)) frame.flags = 0U;
    if (!counters.source_valid || counters.tracking_epoch != frame.pico_tracking_epoch ||
        frame.input_monotonic_ns < input_after_ns ||
        !fresh(frame.input_monotonic_ns, now, freshness_ns))
      frame.flags = static_cast<std::uint8_t>(frame.flags & ~kJointCommandArmsReadyFlag);
    revoked_flags.fetch_or(static_cast<std::uint8_t>(published_ready_flags & ~frame.flags),
                           std::memory_order_release);
    published_ready_flags = static_cast<std::uint8_t>(published_ready_flags | frame.flags);
    outgoing.sequence = frame.sequence;
    outgoing.produced_monotonic_ns = frame.source_timestamp_ns;
    outgoing.input_monotonic_ns = frame.input_monotonic_ns;
    outgoing.tracking_epoch = frame.pico_tracking_epoch;
    outgoing.flags = frame.flags;
    std::copy_n(frame.position_rad.begin(), 7U, outgoing.left_arm.begin());
    std::copy_n(frame.position_rad.begin() + 7, 7U, outgoing.right_arm.begin());
    std::copy_n(frame.position_rad.begin() + 14, 20U, outgoing.left_hand.begin());
    std::copy_n(frame.position_rad.begin() + 34, 20U, outgoing.right_hand.begin());
    publisher->publish(outgoing);
  }

  void send(const JointCommandFrame& frame) {
    if (failed.load(std::memory_order_acquire) || !running.load(std::memory_order_acquire))
      throw std::runtime_error("ROS joint output transport is not running");
    if (!publisher) throw std::logic_error("ROS joint output is disabled");
    if (frame.sequence == 0U || frame.source_timestamp_ns <= 0 ||
        (frame.flags & ~kJointCommandKnownFlags) != 0U ||
        !std::all_of(frame.position_rad.begin(), frame.position_rad.end(),
                     [](double value) { return std::isfinite(value); }))
      throw std::runtime_error("invalid committed ROS joint output");
    // A falling readiness edge must survive arbitrary latest-only coalescing.
    // Native restart (and the external executor gate) is required to re-arm.
    revoked_flags.fetch_or(static_cast<std::uint8_t>(seen_ready_flags & ~frame.flags),
                           std::memory_order_release);
    seen_ready_flags = static_cast<std::uint8_t>(seen_ready_flags | frame.flags);
    (void)output.publish(frame);
  }

  PicoReceiverStats stats() const noexcept {
    (void)stats_exchange.tryReadLatest(read_stats);
    auto result = read_stats;
    if (failed.load(std::memory_order_acquire)) result.source_valid = false;
    return result;
  }

  ArmRosTransportOptions options;
  LatestSpscExchange<PicoTeleopFrame>& input;
  LatestSpscExchange<JointCommandFrame> output;
  mutable LatestSpscExchange<PicoReceiverStats> stats_exchange;
  mutable PicoReceiverStats read_stats;
  PicoReceiverStats counters;
  PicoTeleopStreamGate gate;
  const std::string boot_id;
  const std::string session_id;
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Subscription<PicoMessage>::SharedPtr subscription;
  rclcpp::Publisher<JointMessage>::SharedPtr publisher;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::executors::SingleThreadedExecutor executor;
  JointMessage outgoing;
  std::thread worker;
  std::atomic<bool> running{false};
  std::atomic<bool> stopping{false};
  std::atomic<bool> failed{false};
  std::atomic<std::uint8_t> revoked_flags{0U};
  std::uint8_t seen_ready_flags{0U};  // Control thread only.
  std::uint8_t published_ready_flags{0U};  // ROS executor thread only.
  bool bound{false};
  PublisherId publisher_id{};
  std::string input_session;
  std::unordered_set<std::string> retired_sessions;
  std::int64_t freshness_ns{0};
  std::int64_t last_graph_check_ns{0};
  std::int64_t last_publication_ns{0};
  std::int64_t input_after_ns{0};
  std::int64_t previous_source_ns{0};
  std::int64_t previous_source_receive_ns{0};
  std::int64_t source_offset_ns{std::numeric_limits<std::int64_t>::max()};
  std::uint64_t previous_sequence{0U};
  std::uint64_t previous_epoch{0U};
  std::uint64_t previous_revocation_generation{0U};
  std::uint64_t published_sequence{0U};
  std::int64_t published_production_ns{0};
};

ArmRosTransport::ArmRosTransport(ArmRosTransportOptions options,
                                 LatestSpscExchange<PicoTeleopFrame>& input)
    : impl_(std::make_unique<Impl>(std::move(options), input)) {}
ArmRosTransport::~ArmRosTransport() = default;
void ArmRosTransport::start() { impl_->start(); }
void ArmRosTransport::stop() noexcept { impl_->stop(); }
PicoReceiverStats ArmRosTransport::stats() const noexcept { return impl_->stats(); }
void ArmRosTransport::send(const JointCommandFrame& frame) { impl_->send(frame); }

}  // namespace tianji_qp_ik
