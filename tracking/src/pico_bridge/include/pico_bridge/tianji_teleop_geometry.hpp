#pragma once

#include <array>
#include <cstddef>
#include <string>

#include <Eigen/Geometry>

namespace pico_bridge {

inline constexpr std::size_t kPicoSmplJointCount = 24;
inline constexpr std::size_t kPicoSpine2 = 6;
inline constexpr std::size_t kPicoLeftShoulder = 16;
inline constexpr std::size_t kPicoRightShoulder = 17;
inline constexpr std::size_t kPicoLeftElbow = 18;
inline constexpr std::size_t kPicoRightElbow = 19;
inline constexpr std::size_t kPicoLeftWrist = 20;
inline constexpr std::size_t kPicoRightWrist = 21;
inline constexpr std::size_t kPicoLeftHand = 22;
inline constexpr std::size_t kPicoRightHand = 23;
inline constexpr std::size_t kUpperLimbLeftShoulder = 0;
inline constexpr std::size_t kUpperLimbLeftElbow = 1;
inline constexpr std::size_t kUpperLimbLeftWrist = 2;
inline constexpr std::size_t kUpperLimbLeftHand = 3;
inline constexpr std::size_t kUpperLimbRightShoulder = 4;
inline constexpr std::size_t kUpperLimbRightElbow = 5;
inline constexpr std::size_t kUpperLimbRightWrist = 6;
inline constexpr std::size_t kUpperLimbRightHand = 7;
inline constexpr std::size_t kUpperLimbPointCount = 8;
inline constexpr double kDefaultPicoWorldXOffsetM = 0.10;
inline constexpr double kDefaultRobotArmReachScale = 0.95;

enum class PicoPositionRetargetingMode
{
  kRobotArmSegments,
  kPicoPalm,
};

struct PicoPositionRetargetingConfig
{
  PicoPositionRetargetingMode mode{
    PicoPositionRetargetingMode::kRobotArmSegments};
  double robot_arm_reach_scale{kDefaultRobotArmReachScale};
  double pico_world_x_offset_m{kDefaultPicoWorldXOffsetM};
};

struct PicoSkeletonPose
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};

using PicoSkeletonFrame = std::array<PicoSkeletonPose, kPicoSmplJointCount>;

struct PicoArmDirection
{
  bool valid{false};
  Eigen::Vector3d direction{Eigen::Vector3d::Zero()};
};

struct PicoUpperLimbSkeleton
{
  PicoUpperLimbSkeleton()
  {
    for (Eigen::Vector3d & point : points) {
      point.setZero();
    }
    for (Eigen::Quaterniond & rotation : rotations) {
      rotation.setIdentity();
    }
  }
  bool valid{false};
  bool rotations_valid{false};
  std::array<Eigen::Vector3d, kUpperLimbPointCount> points{};
  std::array<Eigen::Quaterniond, kUpperLimbPointCount> rotations{};
};

struct PicoShoulderMapResult
{
  bool valid{false};
  std::string rejection_reason;
  Eigen::Isometry3d pico_shoulder_frame{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d left_target{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d right_target{Eigen::Isometry3d::Identity()};
  PicoArmDirection left_arm_direction;
  PicoArmDirection right_arm_direction;
  PicoUpperLimbSkeleton upper_limb_skeleton;
};

PicoShoulderMapResult map_pico_palms_to_tianji(
  const PicoSkeletonFrame & skeleton,
  const PicoPositionRetargetingConfig & config = {});

}  // namespace pico_bridge
