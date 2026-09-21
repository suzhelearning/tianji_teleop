#include "pico_odin/runtime_alignment.hpp"

#include <stdexcept>

namespace pico_odin {

Eigen::Matrix<double, 6, 6> twist_adjoint(const Pose3 & pelvis_T_odin) {
  Eigen::Matrix<double, 6, 6> adjoint = Eigen::Matrix<double, 6, 6>::Zero();
  const Eigen::Matrix3d rotation = pelvis_T_odin.rotation.toRotationMatrix();
  adjoint.block<3, 3>(0, 0) = rotation;
  adjoint.block<3, 3>(0, 3) =
    skew_symmetric(pelvis_T_odin.translation) * rotation;
  adjoint.block<3, 3>(3, 3) = rotation;
  return adjoint;
}

Covariance6 transform_covariance(
  const Covariance6 & covariance,
  const Eigen::Matrix<double, 6, 6> & jacobian)
{
  if (covariance[0] < 0.0) return covariance;
  const Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> input(
    covariance.data());
  const Eigen::Matrix<double, 6, 6> output = jacobian * input * jacobian.transpose();
  Covariance6 result{};
  Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(result.data()) = output;
  return result;
}

RuntimeAlignment::RuntimeAlignment(Pose3 pelvis_T_odin)
: pelvis_T_odin_(std::move(pelvis_T_odin))
{
  if (!finite(pelvis_T_odin_)) {
    throw std::invalid_argument("pelvis_T_odin is invalid");
  }
}

void RuntimeAlignment::clear() {
  pico_world_T_odin_world_.reset();
}

void RuntimeAlignment::initialize(
  const Pose3 & pico_world_T_pelvis0,
  const Pose3 & odin_world_T_odin0)
{
  const Pose3 odin_world_T_pelvis0 = compose(
    odin_world_T_odin0, inverse(pelvis_T_odin_));
  pico_world_T_odin_world_ = compose(
    pico_world_T_pelvis0, inverse(odin_world_T_pelvis0));
}

bool RuntimeAlignment::initialized() const {
  return pico_world_T_odin_world_.has_value();
}

const Pose3 & RuntimeAlignment::session_transform() const {
  if (!pico_world_T_odin_world_) {
    throw std::logic_error("runtime alignment is not initialized");
  }
  return *pico_world_T_odin_world_;
}

Pose3 RuntimeAlignment::corrected_pelvis(const Pose3 & odin_world_T_odin) const {
  if (!pico_world_T_odin_world_) {
    throw std::logic_error("runtime alignment is not initialized");
  }
  return compose(
    *pico_world_T_odin_world_,
    compose(odin_world_T_odin, inverse(pelvis_T_odin_)));
}

std::vector<Pose3> RuntimeAlignment::anchor_skeleton(
  const std::vector<Pose3> & pico_joints,
  const Pose3 & corrected_root) const
{
  if (pico_joints.empty()) return {};
  const Pose3 root_change = compose(corrected_root, inverse(pico_joints.front()));
  std::vector<Pose3> anchored;
  anchored.reserve(pico_joints.size());
  for (const auto & joint : pico_joints) anchored.push_back(compose(root_change, joint));
  return anchored;
}

}  // namespace pico_odin
