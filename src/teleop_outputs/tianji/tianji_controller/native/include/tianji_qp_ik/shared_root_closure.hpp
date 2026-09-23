#pragma once
#include "tianji_qp_ik/shared_root_retarget.hpp"
#include <optional>
namespace tianji_qp_ik {
// Virtual wrist-center frame: origin at the physical wrist, axes parallel to TCP.
struct SharedRootClosureSideGeometry {
  std::string shoulder_frame, elbow_frame, wrist_frame, tcp_frame;
  Eigen::Vector3d shoulder_B{Eigen::Vector3d::Zero()};
  double upper_length_m{0}, forearm_length_m{0};
  Pose tcp_to_wrist_center, link7_to_tcp;
};
using SharedRootClosureGeometry=std::array<SharedRootClosureSideGeometry,2>;
using SharedRootElbowHistory=std::array<std::optional<Eigen::Vector3d>,2>;
enum class ClosureStatus { kAccepted, kInvalidInput, kOutsideWorkspace, kBranchUndetermined, kResidualFailure };
struct SharedRootClosedArm {
  ClosureStatus status{ClosureStatus::kInvalidInput};
  SharedRootArmTarget target;
  bool valid() const noexcept { return status==ClosureStatus::kAccepted; }
};
// Pure geometry; never mutates the caller-owned accepted branch history.
SharedRootClosedArm closeSharedRootArm(const SharedRootClosureSideGeometry&,
    const Pose& palm,const Eigen::Vector3d& preferred_elbow,
    const std::optional<Eigen::Vector3d>& accepted_elbow=std::nullopt) noexcept;
bool closeSharedRootTargets(SharedRootTargets&,const SharedRootClosureGeometry&,
                            const SharedRootElbowHistory&) noexcept;
} // namespace tianji_qp_ik
