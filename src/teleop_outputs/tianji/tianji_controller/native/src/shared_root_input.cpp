#include "tianji_qp_ik/shared_root_input.hpp"
#include <cmath>

namespace tianji_qp_ik {
namespace {
SharedRootSideInput convert(const PicoUpperLimbSkeleton& s, bool left) noexcept {
  const std::size_t shoulder=left ? kPicoLeftShoulderPoint : kPicoRightShoulderPoint;
  const std::size_t elbow=left ? kPicoLeftElbowPoint : kPicoRightElbowPoint;
  const std::size_t wrist=left ? kPicoLeftWristPoint : kPicoRightWristPoint;
  const std::size_t hand=left ? kPicoLeftHandPoint : kPicoRightHandPoint;
  // Frozen bridge contract, not the robot root translation.
  const Eigen::Vector3d origin(0,0,1.121);
  SharedRootSideInput out;
  out.p_shoulder_root_Ct=s.points[shoulder]-origin;
  out.p_elbow_root_Ct=s.points[elbow]-origin;
  out.p_wrist_root_Ct=s.points[wrist]-origin;
  out.p_control_root_Ct=s.points[hand]-origin;
  out.p_shape_proxy_root_Ct=out.p_control_root_Ct;
  Eigen::Matrix3d basis;
  basis.col(0)=(left ? -1.0 : 1.0)*Eigen::Vector3d::UnitY();
  basis.col(1)=(left ? -1.0 : 1.0)*Eigen::Vector3d::UnitZ();
  basis.col(2)=Eigen::Vector3d::UnitX();
  out.R_palm_Ct=s.rotations[hand].normalized().toRotationMatrix()*basis;
  out.R_shoulder_Ct=s.rotations[shoulder].normalized().toRotationMatrix();
  out.R_elbow_Ct=s.rotations[elbow].normalized().toRotationMatrix();
  out.R_wrist_Ct=s.rotations[wrist].normalized().toRotationMatrix();
  return out;
}
}  // namespace

SharedRootInput TjvrSharedRootInputAdapter::adapt(const PicoTeleopFrame& frame) const noexcept {
  SharedRootInput out;
  out.sequence=frame.sequence;
  out.tracking_epoch=frame.tracking_epoch;
  out.resynchronization_generation=frame.resynchronization_generation;
  out.source_timestamp_ns=frame.source_timestamp_ns;
  out.receive_monotonic_ns=frame.receive_monotonic_ns;
  out.stream_discontinuity=frame.stream_discontinuity;
  out.detail="invalid_metadata";
  if (frame.tracking_epoch==0 || frame.source_timestamp_ns<=0 ||
      frame.receive_monotonic_ns<=0) return out;
  const auto& s=frame.upper_limb_skeleton;
  out.detail="invalid_skeleton";
  if (!s.valid || !s.rotations_valid) return out;
  for (std::size_t i=0;i<kPicoUpperLimbPointCount;++i) {
    if (!s.points[i].allFinite() || !s.rotations[i].coeffs().allFinite() ||
        std::abs(s.rotations[i].norm()-1.0)>1e-3) return out;
  }
  out.left=convert(s,true);
  out.right=convert(s,false);
  out.valid=true;
  out.detail="accepted";
  return out;
}
}  // namespace tianji_qp_ik
