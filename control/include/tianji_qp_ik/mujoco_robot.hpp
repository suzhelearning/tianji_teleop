#pragma once

#include "tianji_qp_ik/types.hpp"

#include <mujoco/mujoco.h>

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace tianji_qp_ik {

struct ArmMapping {
  std::array<std::string, kArmDof> joint_names;
  std::array<std::string, kArmDof> body_names;
  std::array<int, kArmDof> joint_ids{};
  std::array<int, kArmDof> body_ids{};
  std::array<int, kArmDof> qpos_addresses{};
  std::array<int, kArmDof> dof_addresses{};
  int tcp_site_id{-1};
  int tcp_body_id{-1};
  ArmLimits limits;
};

struct HandMapping {
  std::array<std::string, kHandDof> joint_names;
  std::array<int, kHandDof> joint_ids{};
  std::array<int, kHandDof> qpos_addresses{};
  Vec20 lower_position{Vec20::Zero()};
  Vec20 upper_position{Vec20::Zero()};
};

struct ArmKinematicSample {
  Pose tcp_pose;
  Mat67 tcp_jacobian{Mat67::Zero()};
  Eigen::Vector3d shoulder_position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d elbow_position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d wrist_position{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d shoulder_rotation{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d elbow_rotation{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d wrist_rotation{Eigen::Matrix3d::Identity()};
  Mat37 shoulder_position_jacobian{Mat37::Zero()};
  Mat37 elbow_position_jacobian{Mat37::Zero()};
  Mat37 wrist_position_jacobian{Mat37::Zero()};
};

class MujocoRobot {
 public:
  explicit MujocoRobot(const std::string& model_path);
  ~MujocoRobot();

  MujocoRobot(const MujocoRobot&) = delete;
  MujocoRobot& operator=(const MujocoRobot&) = delete;
  MujocoRobot(MujocoRobot&&) = delete;
  MujocoRobot& operator=(MujocoRobot&&) = delete;

  const mjModel* model() const noexcept { return model_; }
  mjData* data() noexcept { return data_; }
  const mjData* data() const noexcept { return data_; }

  const ArmMapping& mapping(ArmSide side) const noexcept;
  bool hasHandMappings() const noexcept {
    return left_hand_.has_value() && right_hand_.has_value();
  }
  const HandMapping& handMapping(ArmSide side) const;
  void setArmPosition(ArmSide side, const Vec7& position);
  void setArmState(ArmSide side, const Vec7& position, const Vec7& velocity);
  void setHandPosition(ArmSide side, const Vec20& position);
  // This MuJoCo backend owns data_ synchronously, so each read observes its
  // current state. A hardware adapter must reject stale timestamped feedback
  // before exposing an equivalent position to the controller.
  Vec7 armPosition(ArmSide side) const;
  Vec7 armVelocity(ArmSide side) const;
  Vec20 handPosition(ArmSide side) const;
  void forward();
  Pose tcpPose(ArmSide side) const;
  // Fixed transform of the selected TCP site in the Link7 body frame.
  // This is shared with Pinocchio/Spark so all IK layers use the same TCP.
  Pose tcpRelativeToLink7(ArmSide side) const;
  Mat67 tcpJacobianWorld(ArmSide side);
  ArmKinematicSample armKinematicsAt(ArmSide side, const Vec7& position);
  Vec6 tcpJacobianDotTimesVelocityWorld(ArmSide side, const Vec7& q,
                                        const Vec7& qdot);
  int targetBodyId(ArmSide side) const noexcept;
  int targetMocapId(ArmSide side) const noexcept;

 private:
  ArmMapping buildMapping(ArmSide side) const;
  HandMapping buildHandMapping(ArmSide side) const;

  mjModel* model_{nullptr};
  mjData* data_{nullptr};
  mjData* kinematics_data_{nullptr};
  ArmMapping left_;
  ArmMapping right_;
  std::optional<HandMapping> left_hand_;
  std::optional<HandMapping> right_hand_;
  std::array<int, 2> target_body_ids_{{-1, -1}};
  std::array<int, 2> target_mocap_ids_{{-1, -1}};
  std::vector<mjtNum> jacobian_position_;
  std::vector<mjtNum> jacobian_rotation_;
  std::vector<mjtNum> kinematics_jacobian_position_;
  std::vector<mjtNum> kinematics_jacobian_rotation_;
};

std::string toString(ArmSide side);

}  // namespace tianji_qp_ik
