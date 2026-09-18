#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <limits>
#include <string>

namespace tianji_teleop
{

enum class ArmSide
{
  kLeft,
  kRight,
};

using ArmJointVector = Eigen::Matrix<double, 7, 1>;

// 后端公共配置。所有角度均为弧度、长度均为米；后端不支持的优化项可以
// 忽略，但 maximum_joint_step_rad 和两个末端容差属于公共安全契约。
struct IkSettings
{
  int max_iterations{24};
  double position_tolerance_m{1.0e-3};
  double orientation_tolerance_rad{1.0e-2};
  double minimum_damping{1.0e-3};
  double maximum_damping{1.5e-1};
  double singular_value_threshold{5.0e-2};
  double maximum_iteration_step_rad{8.0e-2};
  double maximum_joint_step_rad{3.0 * 3.14159265358979323846 / 180.0};
  double joint_limit_margin_rad{5.0 * 3.14159265358979323846 / 180.0};
  double arm_angle_gain{0.0};
  double arm_angle_tolerance_rad{
    2.0 * 3.14159265358979323846 / 180.0};
  double arm_angle_finite_difference_rad{1.0e-4};
  double arm_angle_merit_weight{1.0e-3};
  double nullspace_damping{1.0e-3};
  double joint_center_gain{0.0};
  double joint_center_activation_margin_rad{
    15.0 * 3.14159265358979323846 / 180.0};
  double joint_center_merit_weight{1.0e-3};
  double singularity_avoidance_gain{0.0};
  double singularity_finite_difference_rad{1.0e-4};
  double singularity_merit_weight{1.0e-2};

  // pinocchio_qp 后端参数。任务残差按最大笛卡尔速度归一化，关节相关
  // 代价按各关节速度上限归一化，因此下面权重均为无量纲相对权重。
  double control_period_s{1.0 / 200.0};
  double qp_position_time_constant_s{0.30};
  double qp_orientation_time_constant_s{0.40};
  double qp_max_linear_speed_m_s{0.25};
  double qp_max_angular_speed_rad_s{1.00};
  ArmJointVector qp_joint_velocity_limits_rad_s{
    ArmJointVector::Constant(55.0 * 3.14159265358979323846 / 180.0)};
  double qp_position_weight{1.0};
  double qp_orientation_weight{0.45};
  double qp_velocity_regularization_weight{2.0e-2};
  double qp_continuity_weight{6.0e-2};
  double qp_posture_weight{8.0e-3};
  double qp_posture_time_constant_s{2.5};
  ArmJointVector qp_left_nominal_rad{ArmJointVector::Zero()};
  ArmJointVector qp_right_nominal_rad{ArmJointVector::Zero()};
  double qp_joint_limit_activation_margin_rad{
    15.0 * 3.14159265358979323846 / 180.0};
  double qp_joint_limit_velocity_damper_gain{4.0};
  double qp_singularity_critical_threshold{1.5e-2};
  double qp_singularity_orientation_scale{0.15};
  double qp_singularity_posture_multiplier{8.0};
  double qp_singularity_velocity_multiplier{4.0};
  double qp_singularity_escape_weight{3.0e-2};
  double qp_singularity_escape_speed_rad_s{0.15};
  int qp_max_active_set_iterations{48};
  double qp_active_set_tolerance{1.0e-9};

  // 天机官方 libKine 后端。ZSP 使用显式开关，避免把 arm_angle_gain
  // 这个连续增益误当成厂商接口的二值模式。dgr 保留厂商原始单位。
  bool official_use_zsp{false};
  double official_dgr1{0.05};
  double official_dgr2{0.05};
  double official_dgr3{0.0};
  double official_joint_limit_soft_margin_rad{
    5.0 * 3.14159265358979323846 / 180.0};
  double official_candidate_continuity_weight{1.0};
  double official_candidate_limit_weight{0.20};
  double official_candidate_posture_weight{0.02};
  ArmJointVector official_left_nominal_rad{ArmJointVector::Zero()};
  ArmJointVector official_right_nominal_rad{ArmJointVector::Zero()};
  int official_orientation_relaxation_steps{3};
  int official_workspace_backoff_iterations{8};
  int official_worker_timeout_ms{25};
  int official_worker_restart_attempts{1};
};

struct IkResult
{
  // Stateful reference-model backends report their own one-tick displacement;
  // it must not be reconstructed from asynchronous coordinator feedback.
  bool model_state_only{false};
  ArmJointVector reference_velocity_rad_s{ArmJointVector::Zero()};
  ArmJointVector joints_rad{ArmJointVector::Zero()};
  Eigen::Isometry3d achieved_pose{Eigen::Isometry3d::Identity()};
  bool accepted{false};
  bool converged{false};
  bool saturated{false};
  bool joint_step_limited{false};
  bool singularity_active{false};
  double position_error_m{0.0};
  double orientation_error_rad{0.0};
  // 后端无法提供的诊断量应保持 NaN；状态 JSON 会把它们发布为 null。
  double minimum_singular_value{
    std::numeric_limits<double>::quiet_NaN()};
  double damping{std::numeric_limits<double>::quiet_NaN()};
  double arm_angle_error_rad{
    std::numeric_limits<double>::quiet_NaN()};
  double minimum_limit_margin_rad{
    std::numeric_limits<double>::quiet_NaN()};
  double maximum_joint_step_rad{0.0};
  double requested_maximum_joint_step_rad{0.0};
  double solve_time_ms{std::numeric_limits<double>::quiet_NaN()};
  double transport_time_ms{std::numeric_limits<double>::quiet_NaN()};
  double workspace_backoff_fraction{1.0};
  double position_velocity_residual_m_s{
    std::numeric_limits<double>::quiet_NaN()};
  double orientation_velocity_residual_rad_s{
    std::numeric_limits<double>::quiet_NaN()};
  int solver_iterations{0};
  int active_joint_constraints{0};
  int candidate_count{0};
  int selected_candidate_index{-1};
  int transport_restart_count{0};
  bool soft_limit_active{false};
  bool workspace_backoff_active{false};
  bool orientation_relaxed{false};
  bool transport_recovered{false};
  std::string status;
};

// 双臂 IK 后端稳定边界（arm_ik_solver_v1）。
//
// 坐标与单位契约：
// - 关节顺序固定为 Joint1_{L,R} ... Joint7_{L,R}，单位为弧度；
// - 位姿为对应 Base_{L,R} 到 TCP_Link_{L,R} 的变换，平移单位为米；
// - `elbow_reference_direction` 位于同一 Base 坐标系，是 libKine zsp_para 约定的
//   参考平面方向，而不是物理肩肘偏移方向。
//
// solve() 必须总是返回有限的 joints_rad 和 achieved_pose。accepted=true
// 表示上层可以安全采用 joints_rad；converged=true 表示末端主任务已满足
// 配置容差。每次被接受的输出不得超过 maximum_joint_step_rad。
class ArmIkSolver
{
public:
  virtual ~ArmIkSolver() = default;

  // Optional lifecycle/timing extension. Stateless and legacy solvers retain
  // exactly their existing solve path through these default implementations.
  virtual void reset(ArmSide) const {}
  virtual void command_feedback(ArmSide, const ArmJointVector&) const {}
  virtual bool owns_reference_state() const { return false; }
  virtual IkResult solve_timed(ArmSide side, const Eigen::Isometry3d& target,
    const ArmJointVector& q, const Eigen::Vector3d& elbow,
    double, double, double) const { return solve(side,target,q,elbow); }

  virtual Eigen::Isometry3d forward(
    ArmSide side,
    const ArmJointVector & joints_rad) const = 0;

  virtual IkResult solve(
    ArmSide side,
    const Eigen::Isometry3d & target_pose,
    const ArmJointVector & current_joints_rad,
    const Eigen::Vector3d & elbow_reference_direction) const = 0;
};

}  // namespace tianji_teleop
