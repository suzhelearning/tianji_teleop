#pragma once

#include "tianji_qp_ik/config.hpp"
#include "tianji_qp_ik/pico_teleop_protocol.hpp"
#include "tianji_qp_ik/types.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <string_view>

namespace tianji_qp_ik {

enum class SharedRootSegmentSource {
  kInvalid,
  kRotation,
  kPositionFallback,
};

std::string_view toString(SharedRootSegmentSource source) noexcept;

struct SharedRootRobotGeometry {
  Eigen::Vector3d left_shoulder{Eigen::Vector3d::Zero()};
  Eigen::Vector3d right_shoulder{Eigen::Vector3d::Zero()};
  Eigen::Vector3d left_upper_arm_local{Eigen::Vector3d::Zero()};
  Eigen::Vector3d left_forearm_local{Eigen::Vector3d::Zero()};
  Eigen::Vector3d left_wrist_to_palm_local{Eigen::Vector3d::Zero()};
  Eigen::Vector3d right_upper_arm_local{Eigen::Vector3d::Zero()};
  Eigen::Vector3d right_forearm_local{Eigen::Vector3d::Zero()};
  Eigen::Vector3d right_wrist_to_palm_local{Eigen::Vector3d::Zero()};
};

struct SharedRootArmTarget {
  Pose palm;
  Eigen::Vector3d shoulder{Eigen::Vector3d::Zero()};
  Eigen::Vector3d elbow{Eigen::Vector3d::Zero()};
  Eigen::Vector3d wrist{Eigen::Vector3d::Zero()};
  Eigen::Vector3d hand{Eigen::Vector3d::Zero()};
  double upper_scale{1.0};
  double forearm_scale{1.0};
  SharedRootSegmentSource upper_source{SharedRootSegmentSource::kInvalid};
  SharedRootSegmentSource forearm_source{SharedRootSegmentSource::kInvalid};
  SharedRootSegmentSource hand_source{SharedRootSegmentSource::kInvalid};
};

struct SharedRootTargets {
  bool valid{false};
  bool calibrated{false};
  int calibration_samples{0};
  std::string_view detail{"not_initialized"};
  SharedRootArmTarget left;
  SharedRootArmTarget right;
};

class SharedRootSkeletonScaler {
 public:
  SharedRootSkeletonScaler(SharedRootRobotGeometry geometry,
                           SharedRootShapeConfig config);

  void reset() noexcept;
  SharedRootTargets update(const PicoUpperLimbSkeleton& skeleton,
                           const Pose& left_palm, const Pose& right_palm);

 private:
  struct ArmBuildResult {
    bool valid{false};
    std::string_view detail{"not_built"};
    SharedRootArmTarget target;
    std::array<bool, 3> accepted_rotations{{false, false, false}};
    std::array<bool, 3> accepted_frame_alignments{{false, false, false}};
    std::array<Eigen::Vector3d, 3> source_local_directions{
        {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
         Eigen::Vector3d::Zero()}};
  };

  bool validInput(const PicoUpperLimbSkeleton& skeleton,
                  const Pose& left_palm, const Pose& right_palm) const noexcept;
  ArmBuildResult makeTarget(ArmSide side,
                            const PicoUpperLimbSkeleton& skeleton,
                            const Pose& palm) const;
  void commitRotations(
      ArmSide side, const PicoUpperLimbSkeleton& skeleton,
      const std::array<bool, 3>& accepted_rotations,
      const std::array<bool, 3>& accepted_frame_alignments,
      const std::array<Eigen::Vector3d, 3>& source_local_directions) noexcept;

  SharedRootRobotGeometry geometry_;
  SharedRootShapeConfig config_;
  std::array<Eigen::Quaterniond, kPicoUpperLimbPointCount>
      previous_rotations_{};
  std::array<bool, kPicoUpperLimbPointCount> previous_rotations_valid_{};
  std::array<Eigen::Vector3d, kPicoUpperLimbPointCount>
      source_local_directions_{};
  std::array<bool, kPicoUpperLimbPointCount>
      source_local_directions_valid_{};
};

}  // namespace tianji_qp_ik
