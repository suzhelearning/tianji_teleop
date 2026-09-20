#pragma once
#include "tianji_qp_ik/shared_root_continuity.hpp"
#include <string>
namespace tianji_qp_ik {
struct SharedRootOptions {
  bool enabled{false};
  SharedRootMorphologyConfig morphology;
  SharedRootBuilderConfig builder;
  SharedRootContinuityConfig continuity;
  std::array<SharedRootClosureSideGeometry,2> closure_geometry;
  std::string profile_path,input_contract_path,geometry_path,urdf_path,mujoco_path;
  std::string profile_sha256,input_sha256,geometry_sha256;
};
// Startup-only I/O; never called from the control tick. No subprocess or SDK.
std::string sharedRootSha256File(const std::string& path);
SharedRootOptions loadSharedRootOptions(const std::string& profile_path);
} // namespace tianji_qp_ik
