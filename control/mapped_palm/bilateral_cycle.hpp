#pragma once
#include "tianji_mapped_palm/controller.hpp"
#include "tianji_mapped_palm/pico_teleop_session.hpp"
#include "tianji_mapped_palm/pico_ee_headroom.hpp"
#include <memory>
#include <optional>

namespace tianji_mapped_palm {
struct BilateralCycleResult {
  std::uint64_t tick_id{}, applied_epoch{}, applied_sequence{};
  bool epoch_reset{}, control_executed{};
  PicoTeleopButtonAction button_action{PicoTeleopButtonAction::kNone};
  PicoTeleopFreshness freshness;
  ControllerDiagnostics control;
  ArmMotionState left, right;
  PicoEeHeadroomResult left_headroom, right_headroom;
};
class NativeMappedPalmCycle {
 public:
  NativeMappedPalmCycle(QpIkConfig, const std::string& model, const std::string& urdf, bool enabled);
  ~NativeMappedPalmCycle();
  BilateralCycleResult step(std::uint64_t tick, std::int64_t now,
                           const std::optional<PicoTeleopFrame>& frame);
  void reset_at_rest(const Vec7& left, const Vec7& right);
  ArmMotionState reference_state(ArmSide side) const;
  void configure_height(double left, double right);
  void configure_xz(double lx,double rx,double lz,double rz);
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
