#include "tianji_qp_ik/dls_pinocchio_arm_kinematics.hpp"
#include "tianji_qp_ik/so3.hpp"

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

Pose sourceTcp() {
  Pose tcp;
  Eigen::Quaterniond q(0.499998, 0.5, -0.5, 0.500002);
  tcp.rotation=q.normalized().toRotationMatrix();
  tcp.position=Eigen::Vector3d(0,-.095,0);
  return tcp;
}
}  // namespace

struct DlsPinocchioArmKinematics::Impl {
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
      : Impl(urdf_path, {sourceTcp(), sourceTcp()}) {}
  Impl(const std::string& urdf_path, const std::array<Pose,2>& tcp) {
    try {
      pinocchio::urdf::buildModel(urdf_path, model);
    } catch (const std::exception& error) {
      throw std::runtime_error("failed to load Pinocchio URDF '" + urdf_path +
                               "': " + error.what());
    }
    if (model.nq != kExpectedDualArmDof || model.nv != kExpectedDualArmDof) {
      throw std::runtime_error("Pinocchio Tianji model must have nq=nv=14");
    }
    left = buildMapping(ArmSide::kLeft, tcp[0]);
    right = buildMapping(ArmSide::kRight, tcp[1]);
    data = std::make_unique<pinocchio::Data>(model);
    // The Tianji model is validated as a fixed 14-DoF system above.  Keep the
    // hot-path workspaces fixed-size so the direct DLS evaluator does not
    // perform Eigen heap management on every candidate sample.
    configuration.setZero();
    frame_jacobian.setZero();
  }

  ArmMapping buildMapping(ArmSide side, const Pose& tcp) {
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
    if (!tcp.position.allFinite() || !isProperRotation(tcp.rotation))
      throw std::invalid_argument("invalid DLS Pinocchio TCP transform");
    const pinocchio::SE3 tcp_relative(tcp.rotation, tcp.position);
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

  ArmLimits limits(ArmSide side) const {
    const ArmMapping& arm = mapping(side);
    ArmLimits result;
    for (int index = 0; index < kArmDof; ++index) {
      const int q_index =
          arm.q_indices[static_cast<std::size_t>(index)];
      const int velocity_index =
          arm.velocity_indices[static_cast<std::size_t>(index)];
      result.lower_position[index] = model.lowerPositionLimit[q_index];
      result.upper_position[index] = model.upperPositionLimit[q_index];
      result.velocity[index] = model.velocityLimit[velocity_index];
    }
    if (!result.lower_position.allFinite() ||
        !result.upper_position.allFinite() ||
        !result.velocity.allFinite() ||
        (result.lower_position.array() >=
         result.upper_position.array()).any() ||
        (result.velocity.array() <= 0.0).any()) {
      throw std::runtime_error(
          "Pinocchio Tianji model contains invalid arm limits");
    }
    return result;
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
    pinocchio::forwardKinematics(model, *data, configuration);
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

  ArmKinematicSample sampleTcp(ArmSide side, const Vec7& q) {
    if (!q.allFinite()) {
      throw std::invalid_argument(
          "Pinocchio joint position contains NaN or infinity");
    }
    const ArmMapping& arm = mapping(side);
    for (int index = 0; index < kArmDof; ++index) {
      configuration[arm.q_indices[static_cast<std::size_t>(index)]] = q[index];
    }
    // The direct EE DLS path does not need the placements or Jacobians of the
    // shoulder, elbow, or wrist. Updating only the TCP frame avoids the
    // extra frame work performed by sample().
    pinocchio::forwardKinematics(model, *data, configuration);
    pinocchio::computeJointJacobians(model, *data, configuration);
    pinocchio::updateFramePlacement(model, *data, arm.tcp_frame);

    ArmKinematicSample result;
    const pinocchio::SE3& tcp = data->oMf[arm.tcp_frame];
    result.tcp_pose.position = tcp.translation();
    result.tcp_pose.rotation = tcp.rotation();
    result.tcp_jacobian = frameJacobian(arm.tcp_frame, arm);
    return result;
  }

  pinocchio::Model model;
  ArmMapping left;
  ArmMapping right;
  std::unique_ptr<pinocchio::Data> data;
  Eigen::Matrix<double, kExpectedDualArmDof, 1> configuration;
  Eigen::Matrix<double, 6, kExpectedDualArmDof> frame_jacobian;
};

DlsPinocchioArmKinematics::DlsPinocchioArmKinematics(
    const std::string& urdf_path)
    : impl_(std::make_unique<Impl>(urdf_path)) {}

DlsPinocchioArmKinematics::DlsPinocchioArmKinematics(
    const std::string& urdf_path, const std::array<Pose,2>& tcp)
    : impl_(std::make_unique<Impl>(urdf_path, tcp)) {}

DlsPinocchioArmKinematics::~DlsPinocchioArmKinematics() = default;
DlsPinocchioArmKinematics::DlsPinocchioArmKinematics(
    DlsPinocchioArmKinematics&&) noexcept = default;
DlsPinocchioArmKinematics& DlsPinocchioArmKinematics::operator=(
    DlsPinocchioArmKinematics&&) noexcept = default;

int DlsPinocchioArmKinematics::configurationSize() const noexcept {
  return impl_->model.nq;
}

int DlsPinocchioArmKinematics::velocitySize() const noexcept {
  return impl_->model.nv;
}

ArmLimits DlsPinocchioArmKinematics::limits(ArmSide side) const {
  return impl_->limits(side);
}

ArmKinematicSample DlsPinocchioArmKinematics::sample(ArmSide side,
                                                  const Vec7& q) {
  return impl_->sample(side, q);
}

ArmKinematicSample DlsPinocchioArmKinematics::sampleTcp(ArmSide side,
                                                     const Vec7& q) {
  return impl_->sampleTcp(side, q);
}

}  // namespace tianji_qp_ik
