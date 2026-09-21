#include "tianji_qp_ik/pinocchio_arm_kinematics.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <Eigen/Geometry>

#include <array>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace tianji_qp_ik {
namespace {

constexpr int kExpectedDualArmDof = 2 * kArmDof;

std::string suffix(ArmSide side) {
  return side == ArmSide::kLeft ? "L" : "R";
}

pinocchio::FrameIndex requireFrame(const pinocchio::Model& model,
                                   const std::string& name) {
  if (!model.existFrame(name)) {
    throw std::runtime_error("Pinocchio model is missing frame: " + name);
  }
  return model.getFrameId(name);
}

Pose legacyTcpRelativeToLink7() {
  Pose result;
  Eigen::Quaterniond tcp_quaternion(0.499998, 0.5, -0.5, 0.500002);
  tcp_quaternion.normalize();
  result.position = Eigen::Vector3d(0.0, -0.095, 0.0);
  result.rotation = tcp_quaternion.toRotationMatrix();
  return result;
}

}  // namespace

struct PinocchioArmKinematics::Impl {
  struct ArmMapping {
    std::array<pinocchio::JointIndex, kArmDof> joint_ids{};
    std::array<int, kArmDof> q_indices{};
    std::array<int, kArmDof> velocity_indices{};
    pinocchio::FrameIndex shoulder_frame{0};
    pinocchio::FrameIndex elbow_frame{0};
    pinocchio::FrameIndex wrist_frame{0};
    pinocchio::FrameIndex tcp_frame{0};
  };

  explicit Impl(const std::string& urdf_path)
      : Impl(urdf_path,
             std::array<Pose, 2>{legacyTcpRelativeToLink7(),
                                 legacyTcpRelativeToLink7()}) {}

  Impl(const std::string& urdf_path,
       const std::array<Pose, 2>& tcp_relative_to_link7) {
    try {
      pinocchio::urdf::buildModel(urdf_path, model);
    } catch (const std::exception& error) {
      throw std::runtime_error("failed to load Pinocchio URDF '" + urdf_path +
                               "': " + error.what());
    }
    if (model.nq != kExpectedDualArmDof || model.nv != kExpectedDualArmDof) {
      throw std::runtime_error("Pinocchio Tianji model must have nq=nv=14");
    }
    left = buildMapping(ArmSide::kLeft, tcp_relative_to_link7[0]);
    right = buildMapping(ArmSide::kRight, tcp_relative_to_link7[1]);
    data = std::make_unique<pinocchio::Data>(model);
    configuration = Eigen::VectorXd::Zero(model.nq);
    frame_jacobian = Eigen::MatrixXd::Zero(6, model.nv);
  }

  ArmMapping buildMapping(ArmSide side, const Pose& tcp_relative_to_link7) {
    ArmMapping mapping;
    const std::string side_suffix = suffix(side);
    for (int index = 0; index < kArmDof; ++index) {
      const std::string name =
          "Joint" + std::to_string(index + 1) + "_" + side_suffix;
      if (!model.existJointName(name)) {
        throw std::runtime_error("Pinocchio model is missing joint: " + name);
      }
      const pinocchio::JointIndex joint_id = model.getJointId(name);
      const auto& joint = model.joints[joint_id];
      if (joint.nq() != 1 || joint.nv() != 1) {
        throw std::runtime_error("Pinocchio arm joint is not scalar: " + name);
      }
      mapping.joint_ids[static_cast<std::size_t>(index)] = joint_id;
      mapping.q_indices[static_cast<std::size_t>(index)] = joint.idx_q();
      mapping.velocity_indices[static_cast<std::size_t>(index)] =
          joint.idx_v();
    }

    mapping.shoulder_frame = requireFrame(model, "Link1_" + side_suffix);
    mapping.elbow_frame = requireFrame(model, "Link4_" + side_suffix);
    mapping.wrist_frame = requireFrame(model, "Link5_" + side_suffix);
    const pinocchio::FrameIndex link7_frame =
        requireFrame(model, "Link7_" + side_suffix);
    const pinocchio::Frame& link7 = model.frames[link7_frame];
    if (!tcp_relative_to_link7.position.allFinite() ||
        !tcp_relative_to_link7.rotation.allFinite()) {
      throw std::invalid_argument(
          "Pinocchio TCP transform contains NaN or infinity");
    }
    const pinocchio::SE3 tcp_relative(tcp_relative_to_link7.rotation,
                                      tcp_relative_to_link7.position);
    const std::string tcp_name = "tcp_" + side_suffix;
    mapping.tcp_frame = model.addFrame(
        pinocchio::Frame(tcp_name, link7.parentJoint, link7_frame,
                         link7.placement * tcp_relative,
                         pinocchio::OP_FRAME),
        false);
    return mapping;
  }

  const ArmMapping& mapping(ArmSide side) const noexcept {
    return side == ArmSide::kLeft ? left : right;
  }

  Eigen::Vector3d framePosition(pinocchio::FrameIndex frame) const {
    return data->oMf[frame].translation();
  }

  Eigen::Matrix3d frameRotation(pinocchio::FrameIndex frame) const {
    return data->oMf[frame].rotation();
  }

  Mat67 frameJacobian(pinocchio::FrameIndex frame,
                      const ArmMapping& arm) {
    frame_jacobian.setZero();
    pinocchio::getFrameJacobian(model, *data, frame,
                                pinocchio::LOCAL_WORLD_ALIGNED,
                                frame_jacobian);
    Mat67 result;
    for (int column = 0; column < kArmDof; ++column) {
      result.col(column) = frame_jacobian.col(
          arm.velocity_indices[static_cast<std::size_t>(column)]);
    }
    return result;
  }

  ArmKinematicSample sample(ArmSide side, const Vec7& q) {
    if (!q.allFinite()) {
      throw std::invalid_argument(
          "Pinocchio joint position contains NaN or infinity");
    }
    const ArmMapping& arm = mapping(side);
    for (int index = 0; index < kArmDof; ++index) {
      configuration[arm.q_indices[static_cast<std::size_t>(index)]] = q[index];
    }
    // This overload computes joint placements as well as Jacobians. A preceding
    // forwardKinematics call would repeat the full model traversal for every
    // IK sample (including each iterative solve and line-search candidate).
    pinocchio::computeJointJacobians(model, *data, configuration);
    pinocchio::updateFramePlacements(model, *data);

    ArmKinematicSample result;
    const pinocchio::SE3& tcp = data->oMf[arm.tcp_frame];
    result.tcp_pose.position = tcp.translation();
    result.tcp_pose.rotation = tcp.rotation();
    result.tcp_jacobian = frameJacobian(arm.tcp_frame, arm);
    result.shoulder_position = framePosition(arm.shoulder_frame);
    result.elbow_position = framePosition(arm.elbow_frame);
    result.wrist_position = framePosition(arm.wrist_frame);
    result.shoulder_rotation = frameRotation(arm.shoulder_frame);
    result.elbow_rotation = frameRotation(arm.elbow_frame);
    result.wrist_rotation = frameRotation(arm.wrist_frame);
    result.shoulder_position_jacobian =
        frameJacobian(arm.shoulder_frame, arm).topRows<3>();
    result.elbow_position_jacobian =
        frameJacobian(arm.elbow_frame, arm).topRows<3>();
    result.wrist_position_jacobian =
        frameJacobian(arm.wrist_frame, arm).topRows<3>();
    return result;
  }

  pinocchio::Model model;
  ArmMapping left;
  ArmMapping right;
  std::unique_ptr<pinocchio::Data> data;
  Eigen::VectorXd configuration;
  Eigen::MatrixXd frame_jacobian;
};

PinocchioArmKinematics::PinocchioArmKinematics(
    const std::string& urdf_path)
    : impl_(std::make_unique<Impl>(urdf_path)) {}

PinocchioArmKinematics::PinocchioArmKinematics(
    const std::string& urdf_path,
    const std::array<Pose, 2>& tcp_relative_to_link7)
    : impl_(std::make_unique<Impl>(urdf_path, tcp_relative_to_link7)) {}

PinocchioArmKinematics::~PinocchioArmKinematics() = default;
PinocchioArmKinematics::PinocchioArmKinematics(
    PinocchioArmKinematics&&) noexcept = default;
PinocchioArmKinematics& PinocchioArmKinematics::operator=(
    PinocchioArmKinematics&&) noexcept = default;

int PinocchioArmKinematics::configurationSize() const noexcept {
  return impl_->model.nq;
}

int PinocchioArmKinematics::velocitySize() const noexcept {
  return impl_->model.nv;
}

ArmKinematicSample PinocchioArmKinematics::sample(ArmSide side,
                                                  const Vec7& q) {
  return impl_->sample(side, q);
}

}  // namespace tianji_qp_ik
