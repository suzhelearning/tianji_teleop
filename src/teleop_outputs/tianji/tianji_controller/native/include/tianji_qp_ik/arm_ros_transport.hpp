#pragma once

#include "tianji_qp_ik/joint_command.hpp"
#include "tianji_qp_ik/pico_input_receiver.hpp"
#include "tianji_qp_ik/telemetry.hpp"
#include "tianji_qp_ik/episode_relative.hpp"
#include "tianji_qp_ik/wuji_hand_teleop_protocol.hpp"

#include <memory>
#include <string>

namespace tianji_qp_ik {

struct ArmRosTransportOptions {
  std::string pico_topic{"/pico/arm_input"};
  std::string joint_target_topic;
  std::string left_hand_topic{"/wuji/left_hand/joint_commands"};
  std::string right_hand_topic{"/wuji/right_hand/joint_commands"};
  double hand_freshness_seconds{0.100};
  double max_position_jump_m{0.15};
  double max_orientation_jump_rad{0.60};
  double freshness_seconds{0.100};
  bool reject_pose_jumps{true};
  bool episode_relative{false};
};

struct HandRosStats {
  std::uint64_t received{0U}, accepted{0U}, rejected{0U}, sequence{0U};
  bool left_stale{true}, right_stale{true};
};

// One ROS executor owns all middleware calls. The control loop only reads the
// existing fixed-size input exchange and writes a fixed-size output exchange.
class ArmRosTransport final : public PicoInputReceiver, public JointCommandSink {
 public:
  ArmRosTransport(ArmRosTransportOptions options,
                  LatestSpscExchange<PicoTeleopFrame>& input,
                  LatestSpscExchange<WujiHandTeleopFrame>* hand_input = nullptr);
  ~ArmRosTransport() override;
  ArmRosTransport(const ArmRosTransport&) = delete;
  ArmRosTransport& operator=(const ArmRosTransport&) = delete;
  void start() override;
  void stop() noexcept override;
  PicoReceiverStats stats() const noexcept override;
  HandRosStats handStats() const noexcept;
  void send(const JointCommandFrame& frame) override;
  bool takeReferenceCommand(TeleopReferenceCommand& command) noexcept;
  void completeReferenceCommand(const TeleopReferenceResult& result) noexcept;
  bool referenceFaulted() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace tianji_qp_ik
