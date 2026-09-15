#pragma once

#include "tianji_qp_ik/config.hpp"
#include "tianji_qp_ik/pico_teleop_protocol.hpp"
#include "tianji_qp_ik/types.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <string_view>

namespace tianji_qp_ik {

enum class SparkSegmentSource {
  kInvalid,
  kRotation,
  kPositionFallback,
};

std::string_view toString(SparkSegmentSource source) noexcept;

struct SparkUpperRobotGeometry {
  Eigen::Vector3d left_shoulder{Eigen::Vector3d::Zero()};
  Eigen::Vector3d right_shoulder{Eigen::Vector3d::Zero()};
  Eigen::Vector3d left_upper_arm_local{Eigen::Vector3d::Zero()};
  Eigen::Vector3d left_forearm_local{Eigen::Vector3d::Zero()};
  Eigen::Vector3d left_wrist_to_palm_local{Eigen::Vector3d::Zero()};
  Eigen::Vector3d right_upper_arm_local{Eigen::Vector3d::Zero()};
  Eigen::Vector3d right_forearm_local{Eigen::Vector3d::Zero()};
  Eigen::Vector3d right_wrist_to_palm_local{Eigen::Vector3d::Zero()};
};

struct SparkUpperArmTarget {
  Pose palm;
  Eigen::Vector3d shoulder{Eigen::Vector3d::Zero()};
  Eigen::Vector3d elbow{Eigen::Vector3d::Zero()};
  Eigen::Vector3d wrist{Eigen::Vector3d::Zero()};
  Eigen::Vector3d hand{Eigen::Vector3d::Zero()};
  double upper_scale{1.0};
  double forearm_scale{1.0};
  SparkSegmentSource upper_source{SparkSegmentSource::kInvalid};
  SparkSegmentSource forearm_source{SparkSegmentSource::kInvalid};
  SparkSegmentSource hand_source{SparkSegmentSource::kInvalid};
};

struct SparkUpperTargets {
  bool valid{false};
  bool calibrated{false};
  int calibration_samples{0};
  std::string_view detail{"not_initialized"};
  SparkUpperArmTarget left;
  SparkUpperArmTarget right;
};

class UpperSparkSkeletonScaler {
 public:
  UpperSparkSkeletonScaler(SparkUpperRobotGeometry geometry,
                           SparkUpperQpoasesConfig config);

  void reset() noexcept;
  SparkUpperTargets update(const PicoUpperLimbSkeleton& skeleton,
                           const Pose& left_palm, const Pose& right_palm);

 private:
  struct ArmBuildResult {
    bool valid{false};
    std::string_view detail{"not_built"};
    SparkUpperArmTarget target;
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

  SparkUpperRobotGeometry geometry_;
  SparkUpperQpoasesConfig config_;
  std::array<Eigen::Quaterniond, kPicoUpperLimbPointCount>
      previous_rotations_{};
  std::array<bool, kPicoUpperLimbPointCount> previous_rotations_valid_{};
  std::array<Eigen::Vector3d, kPicoUpperLimbPointCount>
      source_local_directions_{};
  std::array<bool, kPicoUpperLimbPointCount>
      source_local_directions_valid_{};
};

}  // namespace tianji_qp_ik
