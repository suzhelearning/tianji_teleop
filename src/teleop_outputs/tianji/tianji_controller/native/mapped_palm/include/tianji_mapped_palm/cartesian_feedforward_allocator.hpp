#pragma once

#include "tianji_mapped_palm/config.hpp"
#include "tianji_mapped_palm/velocity_ik.hpp"

#include <chrono>
#include <memory>
#include <string_view>

namespace tianji_mapped_palm {

enum class CartesianFeedforwardLimit {
  kNone,
  kInvalidInput,
  kPreviewSolverFailure,
  kPreviewNonFinite,
  kTaskScale,
  kExecutionResidual,
  kDirection,
  kVelocityBudget,
  kAccelerationBudget,
  kJerkBudget,
  kArmAngle,
  kNullspaceLeakage,
  kBudgetTimeout,
};

struct CartesianFeedforwardDemand {
  bool valid{false};
  Vec6 low_frequency{Vec6::Zero()};
  Vec6 high_frequency{Vec6::Zero()};
  double legacy_low_scale{0.0};
  double legacy_high_scale{0.0};
  double freshness{1.0};
  double estimator_confidence{1.0};
  bool reset{false};
  double redundancy_authority{1.0};
};

struct DualArmCartesianFeedforwardDemand {
  CartesianFeedforwardDemand left;
  CartesianFeedforwardDemand right;
};

struct CartesianFeedforwardAllocationResult {
  bool valid{false};
  bool candidate_selected{false};
  Vec6 allocated_feedforward{Vec6::Zero()};
  Vec7 legacy_preview_qdot{Vec7::Zero()};
  Vec7 preview_qdot{Vec7::Zero()};
  double low_scale{0.0};
  double high_scale{0.0};
  int preview_solve_count{0};
  double compute_time_us{0.0};
  double normalized_execution_residual{0.0};
  double velocity_usage{0.0};
  double acceleration_usage{0.0};
  double jerk_usage{0.0};
  double arm_angle_residual{0.0};
  double nullspace_leakage{0.0};
  CartesianFeedforwardLimit limiting_factor{
      CartesianFeedforwardLimit::kNone};
};

std::string_view toString(CartesianFeedforwardLimit limit) noexcept;

class CartesianFeedforwardAllocator7 {
 public:
  CartesianFeedforwardAllocator7(
      CartesianFeedforwardAllocatorConfig config,
      JointLimitConfig dynamic_limits, CartesianServoConfig servo,
      std::unique_ptr<IArmVelocityIk> preview_solver);
  CartesianFeedforwardAllocationResult allocate(
      const ArmIkInput& base_input, const Vec6& feedback_twist,
      const CartesianFeedforwardDemand& demand);
  void reset() noexcept;

 private:
  struct Evaluation {
    bool accepted{false};
    ArmIkResult preview;
    double normalized_residual{0.0};
    double velocity_usage{0.0};
    double acceleration_usage{0.0};
    double jerk_usage{0.0};
    double arm_angle_residual{0.0};
    double nullspace_leakage{0.0};
    CartesianFeedforwardLimit limit{CartesianFeedforwardLimit::kNone};
  };

  Evaluation evaluate(const ArmIkInput& base_input,
                      const Vec6& feedback_twist,
                      const CartesianFeedforwardDemand& demand,
                      double low_scale, double high_scale,
                      const Evaluation* legacy, int& solve_count);
  bool timedOut() const noexcept;

  CartesianFeedforwardAllocatorConfig config_;
  JointLimitConfig dynamic_limits_;
  CartesianServoConfig servo_;
  std::unique_ptr<IArmVelocityIk> preview_solver_;
  bool initialized_{false};
  double previous_low_scale_{0.0};
  double previous_high_scale_{0.0};
  std::chrono::steady_clock::time_point start_time_{};
};

}  // namespace tianji_mapped_palm
