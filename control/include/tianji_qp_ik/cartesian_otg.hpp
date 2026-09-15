#pragma once

#include "tianji_qp_ik/config.hpp"
#include "tianji_qp_ik/target_manager.hpp"

#include <ruckig/ruckig.hpp>

namespace tianji_qp_ik {

class CartesianReferenceGenerator {
 public:
  explicit CartesianReferenceGenerator(CartesianOtgConfig config,
                                       double dt_seconds);

  void reset(const Pose& measured);
  void reconfigure(CartesianOtgConfig config);
  CartesianReference update(const Pose& target, const Vec6& target_twist,
                            bool stale, double dt_seconds);
  const CartesianReference& state() const noexcept { return state_; }

 private:
  CartesianOtgConfig config_;
  ruckig::Ruckig<3> translation_otg_;
  ruckig::InputParameter<3> translation_input_;
  ruckig::OutputParameter<3> translation_output_;
  ruckig::Ruckig<3> orientation_otg_;
  ruckig::InputParameter<3> orientation_input_;
  ruckig::OutputParameter<3> orientation_output_;
  Eigen::Vector3d orientation_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d orientation_target_position_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d orientation_limit_target_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d orientation_limit_direction_{Eigen::Vector3d::UnitX()};
  bool orientation_limit_initialized_{false};
  bool orientation_limits_isotropic_{false};
  bool translation_hold_active_{false};
  double translation_quiet_seconds_{0.0};
  Eigen::Vector3d translation_hold_target_{Eigen::Vector3d::Zero()};
  bool orientation_hold_active_{false};
  double orientation_quiet_seconds_{0.0};
  Eigen::Matrix3d orientation_hold_target_{Eigen::Matrix3d::Identity()};
  CartesianReference state_;
  bool initialized_{false};
};

}  // namespace tianji_qp_ik
