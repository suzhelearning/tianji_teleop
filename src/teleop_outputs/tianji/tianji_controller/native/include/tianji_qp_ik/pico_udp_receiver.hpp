#pragma once

#include "tianji_qp_ik/pico_teleop_protocol.hpp"
#include "tianji_qp_ik/pico_input_receiver.hpp"
#include "tianji_qp_ik/telemetry.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace tianji_qp_ik {

struct PicoUdpReceiverOptions {
  std::string bind_address{"127.0.0.1"};
  std::uint16_t port{15000U};
  double max_position_jump_m{0.15};
  double max_orientation_jump_rad{0.60};
  std::string record_path;
  bool reject_pose_jumps{true};
};

class PicoUdpReceiver final : public PicoInputReceiver {
 public:
  PicoUdpReceiver(PicoUdpReceiverOptions options,
                  LatestSpscExchange<PicoTeleopFrame>& exchange);
  ~PicoUdpReceiver() override;

  PicoUdpReceiver(const PicoUdpReceiver&) = delete;
  PicoUdpReceiver& operator=(const PicoUdpReceiver&) = delete;

  void start() override;
  void stop() noexcept override;
  std::uint16_t boundPort() const noexcept;
  PicoReceiverStats stats() const noexcept override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace tianji_qp_ik
