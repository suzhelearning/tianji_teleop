#pragma once

#include "tianji_qp_ik/joint_command.hpp"
#include "tianji_qp_ik/pico_input_receiver.hpp"
#include "tianji_qp_ik/telemetry.hpp"

#include <memory>
#include <string>

namespace tianji_qp_ik {

struct ArmRosTransportOptions {
  std::string pico_topic{"/pico/arm_input"};
  std::string joint_target_topic;
  double max_position_jump_m{0.15};
  double max_orientation_jump_rad{0.60};
  double freshness_seconds{0.100};
  bool reject_pose_jumps{true};
};

// One ROS executor owns all middleware calls. The control loop only reads the
// existing fixed-size input exchange and writes a fixed-size output exchange.
class ArmRosTransport final : public PicoInputReceiver, public JointCommandSink {
 public:
  ArmRosTransport(ArmRosTransportOptions options,
                  LatestSpscExchange<PicoTeleopFrame>& input);
  ~ArmRosTransport() override;
  ArmRosTransport(const ArmRosTransport&) = delete;
  ArmRosTransport& operator=(const ArmRosTransport&) = delete;
  void start() override;
  void stop() noexcept override;
  PicoReceiverStats stats() const noexcept override;
  void send(const JointCommandFrame& frame) override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace tianji_qp_ik
