#pragma once

#include "tianji_mapped_palm/cartesian_servo.hpp"
#include "tianji_mapped_palm/config.hpp"
#include "tianji_mapped_palm/velocity_ik.hpp"

#include <chrono>
#include <limits>
#include <memory>
#include <string_view>

namespace tianji_mapped_palm {

enum class CartesianTaskAllocationLimit {
  kNone,
  kInvalidInput,
  kPreviewSolverFailure,
  kPreviewNonFinite,
  kTaskScale,
  kExecutionResidual,
  kVelocityBudget,
  kAccelerationBudget,
  kJerkBudget,
  kBudgetTimeout,
};

std::string_view toString(CartesianTaskAllocationLimit limit) noexcept;

struct CartesianTaskAllocationResult {
  // valid describes the preview, not authority over the live command.
  bool valid{false};
  bool authorized{false};
  double predicted_task_scale{1.0};
  double predicted_nullspace_alpha_command{0.0};
  Vec6 predicted_desired_twist{Vec6::Zero()};
  Vec7 preview_qdot{Vec7::Zero()};
  double normalized_residual{0.0};
  double velocity_usage{0.0};
  double acceleration_usage{0.0};
  double jerk_usage{0.0};
  double preview_task_scale_position{1.0};
  double preview_task_scale_orientation{1.0};
  int preview_solve_count{0};
  double compute_time_us{0.0};
  CartesianTaskAllocationLimit limiting_factor{
      CartesianTaskAllocationLimit::kNone};
};

class CartesianTaskAllocator7 {
 public:
  CartesianTaskAllocator7(
      CartesianTaskAllocationConfig config, CartesianServoConfig servo,
      std::unique_ptr<IArmVelocityIk> preview_solver);
  CartesianTaskAllocator7(
      CartesianTaskAllocationConfig config, JointLimitConfig dynamic_limits,
      CartesianServoConfig servo, std::unique_ptr<IArmVelocityIk> preview_solver);

  CartesianTaskAllocationResult allocate(const ArmIkInput& base_input);
  CartesianTaskAllocationResult allocate(const ArmIkInput& base_input,
                                         const Vec6& feedback_twist);
  void reset() noexcept;

 private:
  struct Evaluation {
    bool accepted{false};
    ArmIkResult preview;
    Vec6 desired_twist{Vec6::Zero()};
    double normalized_residual{0.0};
    double velocity_usage{0.0};
    double acceleration_usage{0.0};
    double jerk_usage{0.0};
    double nullspace_alpha_command{0.0};
    double selection_cost{0.0};
    CartesianTaskAllocationLimit limit{
        CartesianTaskAllocationLimit::kNone};
  };

  Evaluation evaluate(const ArmIkInput& base_input, double scale,
                      const Vec6* feedback_twist, int& solve_count,
                      double nullspace_alpha_override =
                          std::numeric_limits<double>::quiet_NaN());
  CartesianTaskAllocationResult allocateImpl(const ArmIkInput& base_input,
                                              const Vec6* feedback_twist);
  bool timedOut() const noexcept;

  CartesianTaskAllocationConfig config_;
  JointLimitConfig dynamic_limits_;
  CartesianServoConfig servo_;
  std::unique_ptr<IArmVelocityIk> preview_solver_;
  std::chrono::steady_clock::time_point start_time_{};
};

}  // namespace tianji_mapped_palm
