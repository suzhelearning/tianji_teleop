#include "tianji_qp_ik/arm_ros_transport.hpp"

#include "tianji_qp_ik/so3.hpp"
#include "tianji_qp_ik/shared_root_input.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tianji_interfaces/msg/controller_joint_targets.hpp>
#include <tianji_interfaces/msg/pico_arm_input.hpp>
#include <tianji_interfaces/msg/hand_joint_command.hpp>
#include <tianji_interfaces/srv/teleop_reference.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <thread>
#include <time.h>
#include <unordered_set>
#include <utility>

namespace tianji_qp_ik {
namespace {
using PicoMessage = tianji_interfaces::msg::PicoArmInput;
using JointMessage = tianji_interfaces::msg::ControllerJointTargets;
using ReferenceService = tianji_interfaces::srv::TeleopReference;
using HandMessage = tianji_interfaces::msg::HandJointCommand;
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
  Impl(ArmRosTransportOptions options_in, LatestSpscExchange<PicoTeleopFrame>& input_in,
       LatestSpscExchange<WujiHandTeleopFrame>* hand_input_in)
      : options(std::move(options_in)), input(input_in), hand_input(hand_input_in),
        gate(options.max_position_jump_m, options.max_orientation_jump_rad,
             options.reject_pose_jumps),
        boot_id(readUuid("/proc/sys/kernel/random/boot_id")),
        session_id(readUuid("/proc/sys/kernel/random/uuid")) {
    if (options.pico_topic.empty() || !std::isfinite(options.freshness_seconds) ||
        options.freshness_seconds <= 0.0)
      throw std::invalid_argument("ROS arm input topic and positive freshness are required");
    freshness_ns = static_cast<std::int64_t>(std::min(0.500, options.freshness_seconds) * 1e9);
    node = std::make_shared<rclcpp::Node>("tianji_arm_core");
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    subscription = node->create_subscription<PicoMessage>(options.pico_topic, qos,
        [this](PicoMessage::ConstSharedPtr message, const rclcpp::MessageInfo& info) {
          receive(*message, info);
        });
    if (hand_input) {
      if (options.left_hand_topic.empty() || options.right_hand_topic.empty() ||
          options.left_hand_topic == options.right_hand_topic ||
          !std::isfinite(options.hand_freshness_seconds) || options.hand_freshness_seconds <= 0)
        throw std::invalid_argument("Manus requires distinct nonempty hand topics and positive freshness");
      hand_freshness_ns = static_cast<std::int64_t>(
          std::min(0.150, options.hand_freshness_seconds) * 1e9);
      const std::array<std::string, 2> topics{options.left_hand_topic, options.right_hand_topic};
      for (std::size_t side = 0; side < hands.size(); ++side) {
        hands[side].subscription = node->create_subscription<HandMessage>(topics[side], qos,
            [this, side](HandMessage::ConstSharedPtr message, const rclcpp::MessageInfo& info) {
              receiveHand(side, *message, info);
            });
      }
      if (std::string(hands[0].subscription->get_topic_name()) ==
          hands[1].subscription->get_topic_name())
        throw std::invalid_argument("Manus hand topics must resolve to distinct topics");
    }
    if (!options.joint_target_topic.empty()) {
      publisher = node->create_publisher<JointMessage>(options.joint_target_topic, qos);
      outgoing.boot_id = boot_id;
      outgoing.session_id = session_id;
    }
    if (options.episode_relative) {
      if (!publisher) throw std::invalid_argument("episode-relative requires joint output");
      reference_service = node->create_service<ReferenceService>(
          "/tianji/controller/teleop_reference",
          [this](std::shared_ptr<rmw_request_id_t> header,
                 std::shared_ptr<ReferenceService::Request> request) {
            receiveReference(std::move(header), std::move(request));
          });
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
          invalidateHands();
          std::cerr << "arm_ros_transport_error=" << error.what() << std::endl;
        } catch (...) {
          failed.store(true, std::memory_order_release);
          invalidate("executor_failure");
          invalidateHands();
        }
        running.store(false, std::memory_order_release);
      });
    } catch (...) {
      running.store(false, std::memory_order_release);
      throw;
    }
    std::cout << "pico_ros_topic=" << subscription->get_topic_name()
              << " pico_ros_state=ready node=" << node->get_fully_qualified_name() << std::endl;
    if (hand_input)
      std::cout << "hand_source=manus hand_ros_state=ready left_hand_topic="
                << hands[0].subscription->get_topic_name() << " right_hand_topic="
                << hands[1].subscription->get_topic_name() << std::endl;
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
    // After join there is no middleware/control callback racing destruction.
    if (pending_header) {
      try { finishReference(nullptr, "controller_stopped"); }
      catch (const std::exception& error) {
        std::cerr << "reference_shutdown_reply_error=" << error.what() << std::endl;
        pending_header.reset(); pending_request.reset();
      }
    }
  }

  struct HandSource {
    rclcpp::Subscription<HandMessage>::SharedPtr subscription;
    PublisherId publisher_id{};
    std::string session;
    std::uint32_t glove_id{0U};
    std::uint64_t sequence{0U}, source_generation{0U}, generation{0U};
    std::int64_t source_ns{0}, input_after_ns{0};
    std::array<double, kWujiHandJointDof> positions{};
    bool bound{false}, valid{false};
  };

  bool handSingleton(std::size_t side, const PublisherId* received = nullptr) {
    const auto endpoints = node->get_publishers_info_by_topic(
        hands[side].subscription->get_topic_name());
    if (endpoints.size() != 1U ||
        endpoints.front().topic_type() != "tianji_interfaces/msg/HandJointCommand" ||
        endpoints.front().node_name() != "manus_hand2_retarget" ||
        endpoints.front().node_namespace() != "/") return false;
    const auto& gid = endpoints.front().endpoint_gid();
    return std::any_of(gid.begin(), gid.end(), [](auto byte) { return byte != 0; }) &&
        (received == nullptr || gid == *received);
  }

  void publishHands(std::int64_t now) noexcept {
    if (!hand_input) return;
    WujiHandTeleopFrame frame;
    frame.sequence = ++hand_counters.sequence;
    frame.source_timestamp_ns = std::max(now, last_hand_publication_ns + 1);
    last_hand_publication_ns = frame.source_timestamp_ns;
    frame.receive_monotonic_ns = now;
    frame.left = hands[0].positions;
    frame.right = hands[1].positions;
    frame.left_valid = hands[0].valid;
    frame.right_valid = hands[1].valid;
    frame.left_source_timestamp_ns = frame.left_valid ? hands[0].source_ns : 0;
    frame.right_source_timestamp_ns = frame.right_valid ? hands[1].source_ns : 0;
    frame.left_revocation_generation = hands[0].generation;
    frame.right_revocation_generation = hands[1].generation;
    hand_counters.left_stale = !hands[0].valid;
    hand_counters.right_stale = !hands[1].valid;
    (void)hand_stats_exchange.publish(hand_counters);
    (void)hand_input->publish(frame);
  }

  void revokeHand(std::size_t side, std::int64_t now) noexcept {
    auto& hand = hands[side];
    hand.valid = false;
    hand.input_after_ns = std::max(hand.input_after_ns, now);
    ++hand.generation;
    const auto flag = side == 0 ? kJointCommandLeftHandReadyFlag : kJointCommandRightHandReadyFlag;
    revoked_flags.fetch_or(static_cast<std::uint8_t>(published_ready_flags & flag),
                           std::memory_order_release);
  }

  void invalidateHands() noexcept {
    if (!hand_input) return;
    const auto now = monotonicNowNs();
    for (std::size_t side = 0; side < hands.size(); ++side) revokeHand(side, now);
    publishHands(now);
  }

  void receiveHand(std::size_t side, const HandMessage& message,
                   const rclcpp::MessageInfo& info) {
    ++hand_counters.received;
    const auto now = monotonicNowNs();
    auto& hand = hands[side];
    const auto reject = [&] {
      ++hand_counters.rejected;
      revokeHand(side, now);
      publishHands(now);
    };
    PublisherId received{};
    std::copy_n(info.get_rmw_message_info().publisher_gid.data, received.size(), received.begin());
    if (!handSingleton(side, &received) || message.boot_id != boot_id ||
        !canonicalUuid(message.session_id) || message.glove_id == 0U ||
        message.side != (side == 0 ? "left" : "right")) {
      reject(); return;
    }
    static constexpr std::array<const char*, kWujiHandJointDof> names{
        "thumb_S1", "thumb_S2", "thumb_S3", "thumb_S4",
        "index_S1", "index_S2", "index_S3", "index_S4",
        "middle_S1", "middle_S2", "middle_S3", "middle_S4",
        "ring_S1", "ring_S2", "ring_S3", "ring_S4",
        "pinky_S1", "pinky_S2", "pinky_S3", "pinky_S4"};
    if (!std::equal(names.begin(), names.end(), message.joint_names.begin(),
                    [](const char* expected, const auto& actual) { return actual == expected; }) ||
        (hand.bound && (received != hand.publisher_id || message.session_id != hand.session ||
                       message.glove_id != hand.glove_id)) ||
        message.sequence == 0U || message.source_monotonic_ns <= 0 ||
        message.source_monotonic_ns > now ||
        message.sequence < hand.sequence || message.source_monotonic_ns < hand.source_ns ||
        message.revocation_generation < hand.source_generation) {
      reject(); return;
    }
    if (message.valid &&
        (!fresh(message.source_monotonic_ns, now, hand_freshness_ns) ||
         message.source_monotonic_ns <= hand.input_after_ns ||
         message.sequence <= hand.sequence || message.source_monotonic_ns <= hand.source_ns ||
         !std::all_of(message.position_rad.begin(), message.position_rad.end(),
                      [](double position) { return std::isfinite(position); }))) {
      reject(); return;
    }
    if (!hand.bound) {
      hand.publisher_id = received;
      hand.session = message.session_id;
      hand.glove_id = message.glove_id;
      hand.bound = true;
    }
    // Keep revocations sticky even when a subsequent valid message supersedes
    // this frame before the control thread drains its latest-value exchange.
    if (message.revocation_generation != hand.source_generation ||
        (hand.valid && !fresh(hand.source_ns, now, hand_freshness_ns)))
      revokeHand(side, message.source_monotonic_ns);
    hand.sequence = message.sequence;
    hand.source_ns = message.source_monotonic_ns;
    hand.source_generation = message.revocation_generation;
    if (!message.valid) {
      revokeHand(side, now);
    } else {
      hand.positions = message.position_rad;
      hand.valid = true;
    }
    ++hand_counters.accepted;
    publishHands(now);
  }

  HandRosStats handStats() const noexcept {
    (void)hand_stats_exchange.tryReadLatest(read_hand_stats);
    auto result = read_hand_stats;
    if (failed.load(std::memory_order_acquire) || !running.load(std::memory_order_acquire))
      result.left_stale = result.right_stale = true;
    return result;
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
    if (options.episode_relative) {
      const auto raw = TjvrSharedRootInputAdapter{}.adapt(frame);
      if (!raw.valid) {
        ++counters.malformed;
        invalidate("invalid_relative_palm_skeleton");
        publishStats();
        return;
      }
      // The absolute target branch may contain morphology scaling/offsets.
      // Relative-mode continuity watchdogs must monitor the same raw palms as IK.
      frame.left = {raw.left.p_control_root_Ct, raw.left.R_palm_Ct};
      frame.right = {raw.right.p_control_root_Ct, raw.right.R_palm_Ct};
    }
    const auto decision = gate.evaluate(frame);
    if (!decision.accepted) {
      if (decision.reason == PicoStreamRejectReason::kPositionJump ||
          decision.reason == PicoStreamRejectReason::kOrientationJump)
        ++counters.jump_rejections;
      else
        ++counters.reordered;
      if (options.episode_relative) invalidate("relative_palm_stream_rejected");
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
      if (!singleton(bound ? &publisher_id : nullptr))
        invalidate("publisher_missing_or_ambiguous");
      if (hand_input) {
        for (std::size_t side = 0; side < hands.size(); ++side) {
          if (hands[side].valid && !handSingleton(side, &hands[side].publisher_id)) {
            revokeHand(side, now);
            publishHands(now);
          }
        }
      }
    }
    if (counters.source_valid && !fresh(last_publication_ns, now, freshness_ns))
      invalidate("input_expired");
    if (hand_input) {
      for (std::size_t side = 0; side < hands.size(); ++side) {
        if (hands[side].valid && !fresh(hands[side].source_ns, now, hand_freshness_ns)) {
          revokeHand(side, now);
          publishHands(now);
        }
      }
    }
    publishOutput();
    finishReferenceReplies();
    if (stopping.load(std::memory_order_acquire)) {
      if (pending_header) finishReference(nullptr, "controller_stopped");
      executor.cancel();
    }
  }

  void publishOutput() {
    if (!publisher) return;
    JointCommandFrame frame;
    if (!output.tryReadLatest(frame)) return;
    // The control thread can publish while tick() checks the DDS graph.
    // Sample time after acquiring the frame, never before that concurrent write.
    const auto now = monotonicNowNs();
    if (frame.sequence <= published_sequence || frame.source_timestamp_ns <= published_production_ns)
      throw std::runtime_error("joint output sequence/production clock rollback");
    published_sequence = frame.sequence;
    published_production_ns = frame.source_timestamp_ns;
    frame.flags = static_cast<std::uint8_t>(frame.flags & ~revoked_flags.load(std::memory_order_acquire));
    if (!fresh(frame.source_timestamp_ns, now, 100000000)) frame.flags = 0U;
    if (reference_fault.load(std::memory_order_acquire) ||
        !counters.source_valid || counters.tracking_epoch != frame.pico_tracking_epoch ||
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
    outgoing.reference_id = options.episode_relative ? frame.reference_id : 0U;
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
  void receiveReference(std::shared_ptr<rmw_request_id_t> header,
                        std::shared_ptr<ReferenceService::Request> request) {
    const auto reject = [&](const char* message) {
      ReferenceService::Response response;
      response.request_id = request->request_id;
      response.session_id = request->session_id;
      response.reference_id = request->reference_id;
      response.state = reference_fault.load(std::memory_order_acquire) ? "fault" : "rejected";
      response.message = message;
      reference_service->send_response(*header, response);
    };
    const auto now = monotonicNowNs();
    if (stopping.load(std::memory_order_acquire) || reference_fault.load(std::memory_order_acquire)) {
      reject("controller_stopped_or_faulted"); return;
    }
    if (request->controller_session_id != session_id || !canonicalUuid(request->session_id) ||
        request->request_id.empty() || request->request_id.size() > 128U ||
        !fresh(request->measured_monotonic_ns, now, 100000000)) {
      reject("identity_or_command_age_invalid"); return;
    }
    ReferenceOperation operation;
    if (request->operation == "prepare") operation = ReferenceOperation::kPrepare;
    else if (request->operation == "activate") operation = ReferenceOperation::kActivate;
    else if (request->operation == "cancel") operation = ReferenceOperation::kCancel;
    else { reject("unknown_operation"); return; }
    if (owner_session.empty()) {
      if (operation != ReferenceOperation::kPrepare) { reject("owner_not_bound"); return; }
      owner_session = request->session_id;
      std::copy_n(header->writer_guid, owner_guid.size(), owner_guid.begin());
    }
    if (request->session_id != owner_session ||
        !std::equal(owner_guid.begin(), owner_guid.end(), header->writer_guid)) {
      reject("executor_owner_mismatch"); return;
    }
    if (header->sequence_number <= last_reference_sequence ||
        seen_reference_requests.count(request->request_id) != 0U) {
      reject("duplicate_or_reordered_request"); return;
    }
    last_reference_sequence = header->sequence_number;
    if (seen_reference_requests.size() >= 65536U) {
      reject("request_history_exhausted_restart_required"); return;
    }
    seen_reference_requests.insert(request->request_id);
    if (pending_header) { reject("reference_command_busy"); return; }
    TeleopReferenceCommand command;
    command.token = ++reference_token;
    command.operation = operation; command.reference_id = request->reference_id;
    command.received_ns = now; command.measured_ns = request->measured_monotonic_ns;
    command.stable_time_s = request->stable_time_s;
    command.position_tolerance_m = request->position_tolerance_m;
    command.orientation_tolerance_rad = request->orientation_tolerance_rad;
    for (std::size_t arm = 0; arm < 2; ++arm)
      for (int j = 0; j < kArmDof; ++j)
        command.measured_q[arm][j] = request->measured_arm_rad[arm * 7U + static_cast<std::size_t>(j)];
    for (int j = 0; j < 3; ++j) {
      command.waist_lower[j] = request->waist_lower_m[static_cast<std::size_t>(j)];
      command.waist_upper[j] = request->waist_upper_m[static_cast<std::size_t>(j)];
    }
    if (!reference_commands.tryPush(command)) { reject("reference_queue_full"); return; }
    pending_header = std::move(header); pending_request = std::move(request);
    pending_deadline_ns = now + 250000000;
  }

  void finishReference(const TeleopReferenceResult* result, const char* failure) {
    if (!pending_header) return;
    ReferenceService::Response response;
    response.request_id = pending_request->request_id;
    response.session_id = pending_request->session_id;
    response.reference_id = pending_request->reference_id;
    response.state = result ? referencePhaseName(result->phase) : "fault";
    response.success = result && result->success;
    response.message = result ? result->message : failure;
    if (result) {
      response.input_monotonic_ns = result->input_ns;
      response.tracking_epoch = result->tracking_epoch;
      for (std::size_t arm = 0; arm < 2; ++arm) {
        for (int j = 0; j < kArmDof; ++j)
          response.measured_arm_rad[arm * 7U + static_cast<std::size_t>(j)] = result->measured_q[arm][j];
        const auto encode_pose = [arm](const Pose& pose, auto& values) {
          const Eigen::Quaterniond q(pose.rotation);
          for (int j = 0; j < 3; ++j) values[arm * 7U + static_cast<std::size_t>(j)] = pose.position[j];
          values[arm * 7U + 3U] = q.x(); values[arm * 7U + 4U] = q.y();
          values[arm * 7U + 5U] = q.z(); values[arm * 7U + 6U] = q.w();
        };
        encode_pose(result->human_reference[arm], response.human_reference);
        encode_pose(result->robot_reference[arm], response.robot_reference);
      }
    }
    reference_service->send_response(*pending_header, response);
    pending_header.reset(); pending_request.reset(); pending_result.reset();
  }
  void finishReferenceReplies() {
    TeleopReferenceResult result;
    while (reference_results.tryPop(result))
      if (pending_header && result.token == reference_token) pending_result = result;
    if (!pending_header) return;
    if (monotonicNowNs() > pending_deadline_ns) {
      reference_fault.store(true, std::memory_order_release);
      finishReference(nullptr, "control_reply_timeout");
      return;
    }
    if (pending_result) {
      // Prepare ACK cannot overtake its held q/reference-id publication.
      const bool published = outgoing.reference_id == pending_result->reference_id &&
          outgoing.produced_monotonic_ns >= pending_request->measured_monotonic_ns;
      if (!pending_result->success || published)
        finishReference(&*pending_result, nullptr);
    }
  }

  ArmRosTransportOptions options;
  LatestSpscExchange<PicoTeleopFrame>& input;
  LatestSpscExchange<WujiHandTeleopFrame>* hand_input;
  std::array<HandSource, 2> hands;
  HandRosStats hand_counters;
  mutable LatestSpscExchange<HandRosStats> hand_stats_exchange;
  mutable HandRosStats read_hand_stats;
  std::int64_t hand_freshness_ns{0}, last_hand_publication_ns{0};
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
  rclcpp::Service<ReferenceService>::SharedPtr reference_service;
  BoundedSpscQueue<TeleopReferenceCommand> reference_commands{2U};
  BoundedSpscQueue<TeleopReferenceResult> reference_results{2U};
  std::shared_ptr<rmw_request_id_t> pending_header;
  std::shared_ptr<ReferenceService::Request> pending_request;
  std::optional<TeleopReferenceResult> pending_result;
  std::string owner_session;
  PublisherId owner_guid{};
  std::unordered_set<std::string> seen_reference_requests;
  std::uint64_t reference_token{0};
  std::int64_t last_reference_sequence{0}, pending_deadline_ns{0};
  std::atomic<bool> reference_fault{false};
};

ArmRosTransport::ArmRosTransport(ArmRosTransportOptions options,
                                 LatestSpscExchange<PicoTeleopFrame>& input,
                                 LatestSpscExchange<WujiHandTeleopFrame>* hand_input)
    : impl_(std::make_unique<Impl>(std::move(options), input, hand_input)) {}
ArmRosTransport::~ArmRosTransport() = default;
void ArmRosTransport::start() { impl_->start(); }
void ArmRosTransport::stop() noexcept { impl_->stop(); }
PicoReceiverStats ArmRosTransport::stats() const noexcept { return impl_->stats(); }
HandRosStats ArmRosTransport::handStats() const noexcept { return impl_->handStats(); }
void ArmRosTransport::send(const JointCommandFrame& frame) { impl_->send(frame); }
bool ArmRosTransport::takeReferenceCommand(TeleopReferenceCommand& command) noexcept {
  return impl_->reference_commands.tryPop(command);
}
void ArmRosTransport::completeReferenceCommand(const TeleopReferenceResult& result) noexcept {
  if (!impl_->reference_results.tryPush(result))
    impl_->reference_fault.store(true, std::memory_order_release);
}
bool ArmRosTransport::referenceFaulted() const noexcept {
  return impl_->reference_fault.load(std::memory_order_acquire);
}

}  // namespace tianji_qp_ik
