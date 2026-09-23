#pragma once
#include "tianji_qp_ik/ruckig_trajectory_limiter.hpp"
#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>

namespace tianji_qp_ik {
// In-process model-reference simulation only. No command export or hardware authority.
class SimulationRecovery {
 public:
  static constexpr double kHomeVelocity = .5;
  static constexpr double kHomeAcceleration = .8;
  static constexpr double kHomeJerk = 3.;
  enum class Phase { kWaiting, kTeleop, kBraking, kHoming, kHomeReached, kHold, kFault };
  using Pair = std::array<ArmMotionState, 2>;
  SimulationRecovery(const QpIkConfig& config, std::array<ArmLimits,2> limits,
                     Pair home, double dt, double settled_seconds=.3)
      : limits_(limits), home_(home), dt_(dt), settled_seconds_(settled_seconds), stop_config_(
            config.pico_ee_franka_dls.post_smoothing) {
    if(!std::isfinite(settled_seconds_)||settled_seconds_<=0.)
      throw std::invalid_argument("Invalid simulation settled duration");
    for(auto& l:limits_) {
      l.lower_position.array()+=config.joint_limits.margin_rad;
      l.upper_position.array()-=config.joint_limits.margin_rad;
    }
    home_config_=stop_config_;
    home_config_.max_velocity_rad_s=stop_config_.max_velocity_rad_s.cwiseMin(Vec7::Constant(kHomeVelocity));
    home_config_.max_acceleration_rad_s2=stop_config_.max_acceleration_rad_s2.cwiseMin(Vec7::Constant(kHomeAcceleration));
    home_config_.max_jerk_rad_s3=stop_config_.max_jerk_rad_s3.cwiseMin(Vec7::Constant(kHomeJerk));
    for(int i=0;i<2;++i) {
      RuckigTrajectoryLimiter7 check(home_config_,limits_[i],dt_);
      if(!check.canReset(home_[i]))throw std::invalid_argument("Invalid simulation Home");
    }
  }
  Phase phase() const {return phase_;}
  bool teleop() const {return phase_==Phase::kTeleop;}
  bool handsPaused(bool paused, bool pico_paused) const {
    return paused || pico_paused || !teleop();
  }
  const char* name() const {
    switch(phase_) {
      case Phase::kWaiting:return "WAITING";
      case Phase::kTeleop:return "TELEOP";
      case Phase::kBraking:return "BRAKING";
      case Phase::kHoming:return "HOMING";
      case Phase::kHomeReached:return "HOME_REACHED";
      case Phase::kHold:return "HOLD";
      case Phase::kFault:return "FAULT";
    }
    return "FAULT";
  }
  static bool atRest(const Pair& state) {
    for(const auto& s:state)
      if(!s.q.allFinite()||!s.qdot.allFinite()||!s.qddot.allFinite()||
         s.qdot.cwiseAbs().maxCoeff()>1e-6||s.qddot.cwiseAbs().maxCoeff()>1e-5)return false;
    return true;
  }
  bool start(bool fresh, const Pair& current) {
    if(!fresh||!atRest(current)||!(phase_==Phase::kWaiting||phase_==Phase::kHold||phase_==Phase::kHomeReached))return false;
    phase_=Phase::kTeleop;return true;
  }
  bool stop(const Pair& current, bool home=false) {
    if(phase_==Phase::kFault)return false;
    if(home&&(phase_==Phase::kBraking||phase_==Phase::kHoming))return false;
    if(!resetLimiters(stop_config_,current)) {phase_=Phase::kFault;return false;}
    pending_home_=home;settled_=0;elapsed_=0;phase_=Phase::kBraking;return true;
  }
  void fault(){phase_=Phase::kFault;}
  Pair update(const Pair& current) {
    if(phase_!=Phase::kBraking&&phase_!=Phase::kHoming)return current;
    elapsed_+=dt_;
    if(elapsed_>60.) {phase_=Phase::kFault;return current;}
    auto candidates=limiters_;
    Pair next=current;
    for(int i=0;i<2;++i) {
      const auto result=candidates[i]->update(phase_==Phase::kHoming?home_[i].q:current[i].q,dt_);
      if(!result.accepted){phase_=Phase::kFault;return current;}
      next[i]=result.state;
    }
    for(int i=0;i<2;++i)limiters_[i].emplace(*candidates[i]); // Commit both validated candidates.
    bool settled=atRest(next);
    if(phase_==Phase::kHoming)
      for(int i=0;i<2;++i)settled=settled&&(next[i].q-home_[i].q).cwiseAbs().maxCoeff()<1e-6;
    settled_=settled?settled_+dt_:0;
    if(settled_>=settled_seconds_) {
      if(phase_==Phase::kBraking&&pending_home_) {
        if(!resetLimiters(home_config_,next))phase_=Phase::kFault;
        else {phase_=Phase::kHoming;settled_=0;elapsed_=0;}
      } else phase_=phase_==Phase::kHoming?Phase::kHomeReached:Phase::kHold;
    }
    return next;
  }
 private:
  bool resetLimiters(const DlsPostureRuckigConfig& cfg,const Pair& current) {
    std::array<std::optional<RuckigTrajectoryLimiter7>,2> next;
    for(int i=0;i<2;++i) {
      next[i].emplace(cfg,limits_[i],dt_);
      if(!next[i]->reset(current[i]))return false;
    }
    for(int i=0;i<2;++i)limiters_[i].emplace(*next[i]);
    return true;
  }
  std::array<ArmLimits,2> limits_;
  Pair home_;
  double dt_,settled_seconds_,settled_{0},elapsed_{0};
  DlsPostureRuckigConfig stop_config_,home_config_;
  std::array<std::optional<RuckigTrajectoryLimiter7>,2> limiters_;
  Phase phase_{Phase::kWaiting};
  bool pending_home_{false};
};
} // namespace tianji_qp_ik
