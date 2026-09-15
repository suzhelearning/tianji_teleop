// Deployment adapter for the pinned Viewer's mapped-palm-only control path.
// The imported controller/reference/QP implementation is byte-manifested.
#include "bilateral_cycle.hpp"
#include "tianji_mapped_palm/pico_ee_reference.hpp"
#include "tianji_mapped_palm/pico_mapped_corrected_palm.hpp"
#include "tianji_mapped_palm/pico_skeleton_arm_angle.hpp"
#include "tianji_mapped_palm/target_manager.hpp"
#include "tianji_mapped_palm/so3.hpp"
#include <array>
#include <stdexcept>

namespace tianji_mapped_palm {
namespace {
QpIkConfig checked(QpIkConfig c) {
  if (c.ik_algorithm != IkAlgorithm::kPicoEeMappedCorrectedPalmVelocityQp ||
      c.control_level != ControlLevel::kVelocity || !c.controller.model_state_only ||
      c.controller.rate_hz != 200.0 || c.cartesian_otg.enabled ||
      c.iterative_dls.posture_reference_enabled || c.dls_posture_ruckig.enabled)
    throw std::invalid_argument("mapped palm requires 200Hz model-reference velocity baseline without generic OTG/DLS");
  // Viewer --pico-teleop uses a 50 ms input timeout, independently of YAML.
  c.cartesian_servo.target_timeout_seconds = 0.050;
  return c;
}
DualArmTargets currentTargets(MujocoRobot& robot) {
  // setArmState writes qpos/qvel but does not refresh site_xpos/site_xmat.
  // Match the original Viewer: epoch/resync anchors use FK of the latest
  // model command, never the cached TCP from the preceding control tick.
  robot.forward();
  return {robot.tcpPose(ArmSide::kLeft), robot.tcpPose(ArmSide::kRight)};
}
}

struct NativeMappedPalmCycle::Impl {
  QpIkConfig config;
  std::string model_path, urdf_path;
  MujocoRobot robot;
  std::unique_ptr<DualArmController> controller;
  std::unique_ptr<TargetManager> targets;
  PicoTeleopSession session;
  ArmDirectionReferenceManager directions;
  DualArmDirectionReferences latest_directions;
  std::array<PicoEeReferenceGenerator, 2> reference;
  std::array<PicoEeReferenceSample, 2> samples;
  std::array<PicoEeHeadroomGovernor, 2> headroom;
  bool paused;
  double left_height_offset{}, right_height_offset{};
  double left_x_offset{},right_x_offset{};
  std::uint64_t last_tick{}, applied_epoch{}, applied_sequence{};
  std::int64_t last_now{};
  const double dt;

  Impl(QpIkConfig c, const std::string& model, const std::string& urdf, bool enabled,
       const std::optional<std::array<Vec7, 2>>& initial = std::nullopt)
      : config(checked(c)), model_path(model), urdf_path(urdf), robot(model),
        session(config.cartesian_servo.target_timeout_seconds),
        directions(config.arm_angle.reference_rate_limit_rad_s),
        reference{PicoEeReferenceGenerator(config.spark_feedforward_velocity_qp, config.cartesian_servo,
                                           config.pico_ee_twist_estimator),
                  PicoEeReferenceGenerator(config.spark_feedforward_velocity_qp, config.cartesian_servo,
                                           config.pico_ee_twist_estimator)},
        headroom{PicoEeHeadroomGovernor(config.spark_headroom_feedforward_velocity_qp,
                     robot.mapping(ArmSide::kLeft).limits, config.joint_acceleration_limits),
                 PicoEeHeadroomGovernor(config.spark_headroom_feedforward_velocity_qp,
                     robot.mapping(ArmSide::kRight).limits, config.joint_acceleration_limits)},
        paused(!enabled), dt(1.0 / config.controller.rate_hz) {
    if (!robot.hasHandMappings() || !robot.usesHandTcpFrame(ArmSide::kLeft) ||
        !robot.usesHandTcpFrame(ArmSide::kRight))
      throw std::invalid_argument("mapped palm requires bilateral Hand2 and hand_tcp_frame_L/R");
    for (auto side : {ArmSide::kLeft, ArmSide::kRight}) {
      const auto& limits = robot.mapping(side).limits;
      auto q = initial ? (*initial)[side == ArmSide::kLeft ? 0 : 1]
                       : configuredInitialPosture(config.controller, limits, side);
      if (!q.allFinite() || (q.array() < limits.lower_position.array()).any() ||
          (q.array() > limits.upper_position.array()).any())
        throw std::invalid_argument("invalid model reset state");
      robot.setArmState(side, q, Vec7::Zero());
    }
    robot.forward();
    targets = std::make_unique<TargetManager>(config, currentTargets(robot));
    controller = std::make_unique<DualArmController>(robot, config);
    session.setEnabled(enabled);
  }

  BilateralCycleResult step(std::uint64_t tick, std::int64_t now,
                            const std::optional<PicoTeleopFrame>& frame) {
    if (tick != last_tick + 1 || now <= last_now || now <= 0)
      throw std::invalid_argument("consecutive tick and increasing monotonic timestamp required");
    const double time = static_cast<double>(now) * 1e-9;
    BilateralCycleResult out;
    out.tick_id = tick;
    if (frame) {
      out.button_action = session.observeButton(*frame);
      if (out.button_action == PicoTeleopButtonAction::kPause) {
        paused = true;
        headroom[0].reset(); headroom[1].reset();
        targets->setMode(TargetMode::kHold, time);
      } else {
        if (out.button_action == PicoTeleopButtonAction::kResume) paused = false;
        auto classification = session.classify(*frame, now);
        if (classification.action == PicoTeleopAction::kApply ||
            classification.action == PicoTeleopAction::kResetEpochAndApply) {
          const bool reset = classification.action == PicoTeleopAction::kResetEpochAndApply;
          const auto current = currentTargets(robot);
          TargetManager candidate = reset ? TargetManager(config, current) : *targets;
          candidate.setMode(TargetMode::kManual, time);
          auto mapped = selectMappedCorrectedPalm(*frame);
          if (mapped.valid) {
            mapped.left.position.x() += left_x_offset;
            mapped.right.position.x() += right_x_offset;
            mapped.left.position.z() += left_height_offset;
            mapped.right.position.z() += right_height_offset;
            const double age = 1e-9 * static_cast<double>(std::max<std::int64_t>(0, now - frame->bridge_send_monotonic_ns));
            auto left = reference[0].update(mapped.left, frame->sequence, frame->tracking_epoch,
                frame->source_timestamp_ns, frame->stream_discontinuity, dt, age);
            auto right = reference[1].update(mapped.right, frame->sequence, frame->tracking_epoch,
                frame->source_timestamp_ns, frame->stream_discontinuity, dt, age);
            if (left.valid && right.valid && left.pose_valid && right.pose_valid) {
              latest_directions = selectMappedSkeletonArmDirections(frame->upper_limb_skeleton);
              if (candidate.setManualTargets(left.pose, right.pose, 1e-9 * frame->source_timestamp_ns,
                                              1e-9 * frame->receive_monotonic_ns)) {
                samples = {left, right};
                *targets = std::move(candidate);
                session.commitApplied(*frame);
                applied_epoch = frame->tracking_epoch;
                applied_sequence = frame->sequence;
                out.epoch_reset = reset;
              }
            }
          }
        }
      }
    }
    out.freshness = session.freshness(now);
    const auto arm_directions = directions.update(selectArmDirectionReferences(
        ArmAngleReferenceMode::kPicoOutward, out.freshness.live, latest_directions), dt);
    auto desired = targets->sample(time);
    DualArmCartesianFeedforwardDemand demand;
    for (int i = 0; i < 2; ++i) {
      auto& d = i == 0 ? demand.left : demand.right;
      PicoEeFeedforwardAllocation allocation;
      const auto& s = samples[i];
      if (out.freshness.live && session.enabled() && !paused && s.valid && !s.stale) {
        allocation = allocatePicoEeFeedforward(s.low_frequency_feedforward_twist,
            s.high_frequency_feedforward_twist, headroom[i].state().scale,
            config.pico_ee_headroom.low_frequency_min_scale, config.pico_ee_headroom.high_frequency_min_scale);
        d.valid = true;
        d.low_frequency = s.low_frequency_feedforward_twist;
        d.high_frequency = s.high_frequency_feedforward_twist;
        d.legacy_low_scale = allocation.low_frequency_scale;
        d.legacy_high_scale = allocation.high_frequency_scale;
        d.estimator_confidence = s.causal_selected ? s.causal.confidence : 1.0;
        d.freshness = 1.0;
        d.reset = s.causal.failure == CausalSe3TwistFailure::kEpochReset ||
                  s.causal.failure == CausalSe3TwistFailure::kStreamDiscontinuity;
        d.redundancy_authority = picoEeRedundancyAuthority(config.pico_ee_headroom, headroom[i].state());
      }
      (i == 0 ? desired.left_twist : desired.right_twist) = allocation.twist;
    }
    controller->setArmAngleReferenceMode(ArmAngleReferenceMode::kPicoOutward);
    if (!paused) {
      out.control_executed = true;
      out.control = controller->step(directReferences(desired), arm_directions, demand, dt);
      for (int i = 0; i < 2; ++i) {
        const auto side = i == 0 ? ArmSide::kLeft : ArmSide::kRight;
        const auto& arm = i == 0 ? out.control.left : out.control.right;
        PicoEeHeadroomFeedback f;
        f.accepted = out.freshness.live && arm.accepted && arm.ik.status == SolverStatus::kSolved;
        f.qdot = arm.ik.qdot;
        f.qddot = controller->previousAcceleration(side);
        f.task_scale_position = arm.ik.task_scale_position;
        f.task_scale_orientation = arm.ik.task_scale_orientation;
        (void)headroom[i].update(f, dt);
      }
    } else {
      robot.forward();
      out.control.left.target = desired.left;
      out.control.right.target = desired.right;
    }
    out.left = controller->referenceState(ArmSide::kLeft);
    out.right = controller->referenceState(ArmSide::kRight);
    out.left_headroom = headroom[0].state(); out.right_headroom = headroom[1].state();
    out.applied_epoch = applied_epoch; out.applied_sequence = applied_sequence;
    last_tick = tick; last_now = now;
    return out;
  }
};
NativeMappedPalmCycle::NativeMappedPalmCycle(QpIkConfig c, const std::string& m,
    const std::string& u, bool enabled) : impl_(std::make_unique<Impl>(c, m, u, enabled)) {}
NativeMappedPalmCycle::~NativeMappedPalmCycle() = default;
BilateralCycleResult NativeMappedPalmCycle::step(std::uint64_t t, std::int64_t n,
    const std::optional<PicoTeleopFrame>& f) { return impl_->step(t, n, f); }
ArmMotionState NativeMappedPalmCycle::reference_state(ArmSide s) const { return impl_->controller->referenceState(s); }
void NativeMappedPalmCycle::configure_height(double left, double right) {
  if (impl_->last_tick != 0 || !std::isfinite(left) || !std::isfinite(right) ||
      std::abs(left) > 1.0 || std::abs(right) > 1.0)
    throw std::invalid_argument("height configuration requires tick zero and bounded finite offsets");
  impl_->left_height_offset = left;
  impl_->right_height_offset = right;
}
void NativeMappedPalmCycle::configure_xz(double lx,double rx,double lz,double rz) {
  for(double v:{lx,rx,lz,rz}) if(!std::isfinite(v) || std::abs(v)>1.)
    throw std::invalid_argument("XZ offsets must be finite within 1 m");
  configure_height(lz,rz);
  impl_->left_x_offset=lx;impl_->right_x_offset=rx;
}
void NativeMappedPalmCycle::reset_at_rest(const Vec7& l, const Vec7& r) {
  auto next = std::make_unique<Impl>(impl_->config, impl_->model_path, impl_->urdf_path,
                                   true, std::array<Vec7, 2>{l, r});
  impl_ = std::move(next);
}
}
