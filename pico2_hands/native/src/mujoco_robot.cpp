#include "tianji_v131/mujoco_robot.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <unordered_set>

namespace tianji_v131 {
namespace {

std::string jointName(int one_based_index, ArmSide side) {
  return "Joint" + std::to_string(one_based_index) + "_" +
         (side == ArmSide::kLeft ? "L" : "R");
}

std::string tcpSiteName(ArmSide side) {
  return side == ArmSide::kLeft ? "tcp_L" : "tcp_R";
}

Pose tcpPoseFromData(const ArmMapping& arm, const mjData* data) {
  Pose result;
  result.position =
      Eigen::Map<const Eigen::Vector3d>(&data->site_xpos[3 * arm.tcp_site_id]);
  using RowMajorMatrix3d = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;
  result.rotation = Eigen::Map<const RowMajorMatrix3d>(
      &data->site_xmat[9 * arm.tcp_site_id]);
  return result;
}

Mat67 tcpJacobianFromData(const mjModel* model, const mjData* data,
                          const ArmMapping& arm,
                          std::vector<mjtNum>& jacobian_position,
                          std::vector<mjtNum>& jacobian_rotation) {
  mj_jacSite(model, data, jacobian_position.data(), jacobian_rotation.data(),
             arm.tcp_site_id);
  Mat67 result;
  for (int column = 0; column < kArmDof; ++column) {
    const int source_column =
        arm.dof_addresses[static_cast<std::size_t>(column)];
    for (int row = 0; row < 3; ++row) {
      result(row, column) =
          jacobian_position[static_cast<std::size_t>(row * model->nv +
                                                     source_column)];
      result(row + 3, column) =
          jacobian_rotation[static_cast<std::size_t>(row * model->nv +
                                                     source_column)];
    }
  }
  return result;
}

Eigen::Vector3d bodyPosition(const mjData* data, int body_id) {
  return Eigen::Map<const Eigen::Vector3d>(&data->xpos[3 * body_id]);
}

Eigen::Matrix3d bodyRotation(const mjData* data, int body_id) {
  using RowMajorMatrix3d = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;
  return Eigen::Map<const RowMajorMatrix3d>(&data->xmat[9 * body_id]);
}

Mat37 bodyPositionJacobianFromData(
    const mjModel* model, const mjData* data, const ArmMapping& arm,
    int body_id, std::vector<mjtNum>& jacobian_position,
    std::vector<mjtNum>& jacobian_rotation) {
  mj_jacBody(model, data, jacobian_position.data(), jacobian_rotation.data(),
             body_id);
  Mat37 result;
  for (int column = 0; column < kArmDof; ++column) {
    const int source_column =
        arm.dof_addresses[static_cast<std::size_t>(column)];
    for (int row = 0; row < 3; ++row) {
      result(row, column) =
          jacobian_position[static_cast<std::size_t>(row * model->nv +
                                                     source_column)];
    }
  }
  return result;
}

}  // namespace

MujocoRobot::MujocoRobot(const std::string& model_path) {
  char error[1024]{};
  model_ = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
  if (model_ == nullptr) {
    throw std::runtime_error("failed to load MuJoCo model '" + model_path + "': " + error);
  }
  data_ = mj_makeData(model_);
  if (data_ == nullptr) {
    mj_deleteModel(model_);
    model_ = nullptr;
    throw std::runtime_error("failed to allocate MuJoCo data");
  }
  kinematics_data_ = mj_makeData(model_);
  if (kinematics_data_ == nullptr) {
    mj_deleteData(data_);
    mj_deleteModel(model_);
    data_ = nullptr;
    model_ = nullptr;
    throw std::runtime_error("failed to allocate MuJoCo kinematics data");
  }

  try {
    left_ = buildMapping(ArmSide::kLeft);
    right_ = buildMapping(ArmSide::kRight);
    if (model_->nq != 14 || model_->nv != 14) {
      throw std::runtime_error("expected exactly nq=nv=14 for dual-arm V1");
    }
    for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
      const std::size_t index = side == ArmSide::kLeft ? 0U : 1U;
      const std::string name = side == ArmSide::kLeft ? "target_L" : "target_R";
      const int body_id = mj_name2id(model_, mjOBJ_BODY, name.c_str());
      if (body_id < 0 || model_->body_mocapid[body_id] < 0) {
        throw std::runtime_error("missing mocap target body: " + name);
      }
      target_body_ids_[index] = body_id;
      target_mocap_ids_[index] = model_->body_mocapid[body_id];
    }
    if (target_mocap_ids_[0] == target_mocap_ids_[1]) {
      throw std::runtime_error("left and right targets share a mocap id");
    }
    jacobian_position_.resize(static_cast<std::size_t>(3 * model_->nv));
    jacobian_rotation_.resize(static_cast<std::size_t>(3 * model_->nv));
    kinematics_jacobian_position_.resize(
        static_cast<std::size_t>(3 * model_->nv));
    kinematics_jacobian_rotation_.resize(
        static_cast<std::size_t>(3 * model_->nv));
    forward();
  } catch (...) {
    mj_deleteData(kinematics_data_);
    mj_deleteData(data_);
    mj_deleteModel(model_);
    kinematics_data_ = nullptr;
    data_ = nullptr;
    model_ = nullptr;
    throw;
  }
}

MujocoRobot::~MujocoRobot() {
  if (kinematics_data_ != nullptr) {
    mj_deleteData(kinematics_data_);
  }
  if (data_ != nullptr) {
    mj_deleteData(data_);
  }
  if (model_ != nullptr) {
    mj_deleteModel(model_);
  }
}

ArmMapping MujocoRobot::buildMapping(ArmSide side) const {
  ArmMapping result;
  std::unordered_set<int> joint_ids;
  std::unordered_set<int> body_ids;
  std::unordered_set<int> qpos_addresses;
  std::unordered_set<int> dof_addresses;

  if (model_->nuser_jnt < 1) {
    throw std::runtime_error("model must store the URDF velocity limit in joint user[0]");
  }

  for (int index = 0; index < kArmDof; ++index) {
    const std::size_t array_index = static_cast<std::size_t>(index);
    result.joint_names[array_index] = jointName(index + 1, side);
    result.body_names[array_index] =
        "Link" + std::to_string(index + 1) + "_" +
        (side == ArmSide::kLeft ? "L" : "R");
    const int body_id =
        mj_name2id(model_, mjOBJ_BODY, result.body_names[array_index].c_str());
    if (body_id < 0 || !body_ids.insert(body_id).second) {
      throw std::runtime_error("missing or duplicate body: " + result.body_names[array_index]);
    }
    const int id = mj_name2id(model_, mjOBJ_JOINT, result.joint_names[array_index].c_str());
    if (id < 0) {
      throw std::runtime_error("missing joint: " + result.joint_names[array_index]);
    }
    if (model_->jnt_type[id] != mjJNT_HINGE) {
      throw std::runtime_error("joint is not scalar revolute: " + result.joint_names[array_index]);
    }
    const int qpos_address = model_->jnt_qposadr[id];
    const int dof_address = model_->jnt_dofadr[id];
    if (!joint_ids.insert(id).second || !qpos_addresses.insert(qpos_address).second ||
        !dof_addresses.insert(dof_address).second) {
      throw std::runtime_error("duplicate joint mapping for " + result.joint_names[array_index]);
    }
    result.joint_ids[array_index] = id;
    result.body_ids[array_index] = body_id;
    result.qpos_addresses[array_index] = qpos_address;
    result.dof_addresses[array_index] = dof_address;
    result.limits.lower_position[index] = model_->jnt_range[2 * id];
    result.limits.upper_position[index] = model_->jnt_range[2 * id + 1];
    result.limits.velocity[index] = model_->jnt_user[id * model_->nuser_jnt];
    if (!std::isfinite(result.limits.lower_position[index]) ||
        !std::isfinite(result.limits.upper_position[index]) ||
        !std::isfinite(result.limits.velocity[index]) ||
        !(result.limits.upper_position[index] > result.limits.lower_position[index]) ||
        !(result.limits.velocity[index] > 0.0)) {
      throw std::runtime_error("invalid limits for " + result.joint_names[array_index]);
    }
  }

  result.tcp_site_id = mj_name2id(model_, mjOBJ_SITE, tcpSiteName(side).c_str());
  if (result.tcp_site_id < 0) {
    throw std::runtime_error("missing TCP site: " + tcpSiteName(side));
  }
  result.tcp_body_id = model_->site_bodyid[result.tcp_site_id];
  if (result.tcp_body_id != result.body_ids.back()) {
    throw std::runtime_error("TCP site is not attached to the expected Link7 body");
  }
  return result;
}

const ArmMapping& MujocoRobot::mapping(ArmSide side) const noexcept {
  return side == ArmSide::kLeft ? left_ : right_;
}

void MujocoRobot::setArmPosition(ArmSide side, const Vec7& position) {
  if (!position.allFinite()) {
    throw std::invalid_argument("joint position contains NaN or infinity");
  }
  const ArmMapping& arm = mapping(side);
  for (int index = 0; index < kArmDof; ++index) {
    data_->qpos[arm.qpos_addresses[static_cast<std::size_t>(index)]] = position[index];
  }
}

void MujocoRobot::setArmState(ArmSide side, const Vec7& position,
                              const Vec7& velocity) {
  if (!position.allFinite() || !velocity.allFinite()) {
    throw std::invalid_argument("joint state contains NaN or infinity");
  }
  const ArmMapping& arm = mapping(side);
  for (int index = 0; index < kArmDof; ++index) {
    const std::size_t mapped = static_cast<std::size_t>(index);
    data_->qpos[arm.qpos_addresses[mapped]] = position[index];
    data_->qvel[arm.dof_addresses[mapped]] = velocity[index];
  }
}

Vec7 MujocoRobot::armPosition(ArmSide side) const {
  Vec7 result;
  const ArmMapping& arm = mapping(side);
  for (int index = 0; index < kArmDof; ++index) {
    result[index] = data_->qpos[arm.qpos_addresses[static_cast<std::size_t>(index)]];
  }
  return result;
}

Vec7 MujocoRobot::armVelocity(ArmSide side) const {
  Vec7 result;
  const ArmMapping& arm = mapping(side);
  for (int index = 0; index < kArmDof; ++index) {
    result[index] =
        data_->qvel[arm.dof_addresses[static_cast<std::size_t>(index)]];
  }
  return result;
}

void MujocoRobot::forward() { mj_forward(model_, data_); }

Pose MujocoRobot::tcpPose(ArmSide side) const {
  return tcpPoseFromData(mapping(side), data_);
}

Mat67 MujocoRobot::tcpJacobianWorld(ArmSide side) {
  const ArmMapping& arm = mapping(side);
  return tcpJacobianFromData(model_, data_, arm, jacobian_position_,
                             jacobian_rotation_);
}

ArmKinematicSample MujocoRobot::armKinematicsAt(ArmSide side,
                                                const Vec7& position) {
  if (!position.allFinite()) {
    throw std::invalid_argument("joint position contains NaN or infinity");
  }
  mj_copyData(kinematics_data_, model_, data_);
  const ArmMapping& arm = mapping(side);
  for (int index = 0; index < kArmDof; ++index) {
    kinematics_data_->qpos[
        arm.qpos_addresses[static_cast<std::size_t>(index)]] = position[index];
  }
  mj_forward(model_, kinematics_data_);

  ArmKinematicSample result;
  result.tcp_pose = tcpPoseFromData(arm, kinematics_data_);
  result.tcp_jacobian = tcpJacobianFromData(
      model_, kinematics_data_, arm, kinematics_jacobian_position_,
      kinematics_jacobian_rotation_);
  result.shoulder_position = bodyPosition(kinematics_data_, arm.body_ids[0]);
  result.elbow_position = bodyPosition(kinematics_data_, arm.body_ids[3]);
  result.wrist_position = bodyPosition(kinematics_data_, arm.body_ids[4]);
  result.shoulder_rotation = bodyRotation(kinematics_data_, arm.body_ids[0]);
  result.elbow_rotation = bodyRotation(kinematics_data_, arm.body_ids[3]);
  result.wrist_rotation = bodyRotation(kinematics_data_, arm.body_ids[4]);
  result.shoulder_position_jacobian = bodyPositionJacobianFromData(
      model_, kinematics_data_, arm, arm.body_ids[0],
      kinematics_jacobian_position_, kinematics_jacobian_rotation_);
  result.elbow_position_jacobian = bodyPositionJacobianFromData(
      model_, kinematics_data_, arm, arm.body_ids[3],
      kinematics_jacobian_position_, kinematics_jacobian_rotation_);
  result.wrist_position_jacobian = bodyPositionJacobianFromData(
      model_, kinematics_data_, arm, arm.body_ids[4],
      kinematics_jacobian_position_, kinematics_jacobian_rotation_);
  return result;
}

Vec6 MujocoRobot::tcpJacobianDotTimesVelocityWorld(ArmSide side,
                                                   const Vec7& q,
                                                   const Vec7& qdot) {
  if (!q.allFinite() || !qdot.allFinite()) {
    throw std::invalid_argument("Jdot*qdot input contains NaN or infinity");
  }
  constexpr double kEpsilon = 1e-6;
  const Vec6 plus =
      armKinematicsAt(side, q + kEpsilon * qdot).tcp_jacobian * qdot;
  const Vec6 minus =
      armKinematicsAt(side, q - kEpsilon * qdot).tcp_jacobian * qdot;
  return (plus - minus) / (2.0 * kEpsilon);
}

int MujocoRobot::targetBodyId(ArmSide side) const noexcept {
  return target_body_ids_[side == ArmSide::kLeft ? 0U : 1U];
}

int MujocoRobot::targetMocapId(ArmSide side) const noexcept {
  return target_mocap_ids_[side == ArmSide::kLeft ? 0U : 1U];
}

std::string toString(ArmSide side) { return side == ArmSide::kLeft ? "left" : "right"; }

}  // namespace tianji_v131
