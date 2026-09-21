#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>

namespace pico_odin {

struct Pose3 {
  Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
};

inline Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d & vector) {
  Eigen::Matrix3d matrix;
  matrix <<
    0.0, -vector.z(), vector.y(),
    vector.z(), 0.0, -vector.x(),
    -vector.y(), vector.x(), 0.0;
  return matrix;
}

inline Eigen::Quaterniond normalized(Eigen::Quaterniond quaternion) {
  const double norm = quaternion.norm();
  if (!std::isfinite(norm) || norm < 1e-12) {
    throw std::invalid_argument("quaternion is not normalizable");
  }
  quaternion.normalize();
  return quaternion;
}

inline Pose3 compose(const Pose3 & a_T_b, const Pose3 & b_T_c) {
  const Eigen::Quaterniond rotation = normalized(a_T_b.rotation * b_T_c.rotation);
  return Pose3{
    rotation,
    a_T_b.translation + a_T_b.rotation * b_T_c.translation};
}

inline Pose3 inverse(const Pose3 & a_T_b) {
  const Eigen::Quaterniond rotation = normalized(a_T_b.rotation).conjugate();
  return Pose3{rotation, -(rotation * a_T_b.translation)};
}

inline Pose3 interpolate(const Pose3 & start, const Pose3 & finish, double alpha) {
  alpha = std::clamp(alpha, 0.0, 1.0);
  return Pose3{
    normalized(start.rotation).slerp(alpha, normalized(finish.rotation)).normalized(),
    (1.0 - alpha) * start.translation + alpha * finish.translation};
}

inline double angular_distance(
  const Eigen::Quaterniond & first, const Eigen::Quaterniond & second)
{
  const double dot = std::clamp(
    std::abs(normalized(first).dot(normalized(second))), 0.0, 1.0);
  return 2.0 * std::acos(dot);
}

inline bool finite(const Pose3 & pose) {
  return pose.translation.allFinite() && pose.rotation.coeffs().allFinite() &&
         pose.rotation.norm() > 1e-12;
}

inline Pose3 mean_pose(const std::vector<Pose3> & poses) {
  if (poses.empty()) {
    throw std::invalid_argument("cannot average an empty pose collection");
  }

  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  Eigen::Vector4d coefficients = Eigen::Vector4d::Zero();
  const Eigen::Quaterniond reference = normalized(poses.front().rotation);
  for (const auto & pose : poses) {
    Eigen::Quaterniond quaternion = normalized(pose.rotation);
    if (reference.dot(quaternion) < 0.0) {
      quaternion.coeffs() *= -1.0;
    }
    coefficients += quaternion.coeffs();
    translation += pose.translation;
  }

  Eigen::Quaterniond rotation(
    coefficients.w(), coefficients.x(), coefficients.y(), coefficients.z());
  return Pose3{
    normalized(rotation),
    translation / static_cast<double>(poses.size())};
}

inline geometry_msgs::msg::Pose to_pose_msg(const Pose3 & pose) {
  const Eigen::Quaterniond rotation = normalized(pose.rotation);
  geometry_msgs::msg::Pose message;
  message.position.x = pose.translation.x();
  message.position.y = pose.translation.y();
  message.position.z = pose.translation.z();
  message.orientation.x = rotation.x();
  message.orientation.y = rotation.y();
  message.orientation.z = rotation.z();
  message.orientation.w = rotation.w();
  return message;
}

inline Pose3 from_pose_msg(const geometry_msgs::msg::Pose & message) {
  return Pose3{
    normalized(Eigen::Quaterniond(
      message.orientation.w, message.orientation.x,
      message.orientation.y, message.orientation.z)),
    Eigen::Vector3d(message.position.x, message.position.y, message.position.z)};
}

inline Pose3 from_pose_msg_checked(
  const geometry_msgs::msg::Pose & message,
  double quaternion_norm_tolerance = 1e-3)
{
  const Eigen::Vector3d translation(
    message.position.x, message.position.y, message.position.z);
  const Eigen::Quaterniond rotation(
    message.orientation.w, message.orientation.x,
    message.orientation.y, message.orientation.z);
  if (!std::isfinite(quaternion_norm_tolerance) || quaternion_norm_tolerance < 0.0 ||
      !translation.allFinite() || !rotation.coeffs().allFinite() ||
      std::abs(rotation.norm() - 1.0) > quaternion_norm_tolerance) {
    throw std::invalid_argument(
      "pose contains non-finite values or a non-normalized quaternion");
  }
  return Pose3{normalized(rotation), translation};
}

}  // namespace pico_odin
