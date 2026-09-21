#pragma once
#include "tianji_qp_ik/pico_teleop_protocol.hpp"
#include <string_view>

namespace tianji_qp_ik {
enum class ShapeProxySemantic { kReconstructedPalmProxy };
enum class PalmPositionFieldSource { kSkeletonReconstructedPalm };
enum class PalmRotationFieldSource { kSkeletonHandRotation };
enum class PalmBasisState { kRawHandNeedsFixedSideBasis };

struct SharedRootSideInput {
  Eigen::Vector3d p_shoulder_root_Ct{Eigen::Vector3d::Zero()};
  Eigen::Vector3d p_elbow_root_Ct{Eigen::Vector3d::Zero()};
  Eigen::Vector3d p_wrist_root_Ct{Eigen::Vector3d::Zero()};
  Eigen::Vector3d p_control_root_Ct{Eigen::Vector3d::Zero()};
  Eigen::Vector3d p_shape_proxy_root_Ct{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d R_palm_Ct{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d R_shoulder_Ct{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d R_elbow_Ct{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d R_wrist_Ct{Eigen::Matrix3d::Identity()};
  ShapeProxySemantic shape_proxy_semantic{ShapeProxySemantic::kReconstructedPalmProxy};
  PalmPositionFieldSource position_source{PalmPositionFieldSource::kSkeletonReconstructedPalm};
  PalmRotationFieldSource rotation_source{PalmRotationFieldSource::kSkeletonHandRotation};
  PalmBasisState basis_source{PalmBasisState::kRawHandNeedsFixedSideBasis};
};

// Only valid=true publishes the entire bilateral value. Invalid frames never
// expose partially converted sides. No raw packet Pose is a fallback source.
struct SharedRootInput {
  bool valid{false};
  std::string_view detail{"not_adapted"};
  SharedRootSideInput left, right;
  std::uint64_t sequence{0}, tracking_epoch{0}, resynchronization_generation{0};
  std::int64_t source_timestamp_ns{0}, receive_monotonic_ns{0};
  bool stream_discontinuity{false};
};

class TjvrSharedRootInputAdapter {
 public:
  // Version 1 is pinned to shared_root_tjvr_input_contract.yaml. Artifact hash
  // validation belongs to configuration/startup, not this allocation-free path.
  SharedRootInput adapt(const PicoTeleopFrame& frame) const noexcept;
};
}  // namespace tianji_qp_ik
