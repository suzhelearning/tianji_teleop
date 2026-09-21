#pragma once
#include "tianji_qp_ik/shared_root_morphology.hpp"
#include "tianji_qp_ik/spark_upper_retarget.hpp"
#include "tianji_qp_ik/shared_root_closure.hpp"

namespace tianji_qp_ik {
struct SharedRootBuilderConfig {
  SparkUpperRobotGeometry geometry;
  SparkUpperQpoasesConfig shape_config;
  // Frozen loader always supplies R3 geometry. Empty only for legacy algebra unit fixtures.
  std::optional<SharedRootClosureGeometry> closure_geometry;
  Eigen::Matrix3d R_BCt{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d o_B{0,0,1.121};
  double center_tau_s{.03}, relation_tau_s{.03}, direction_tau_s{.03};
  double orientation_tau_s{.03}, scale_tau_s{.03}, maximum_source_dt_s{.10};
  double maximum_center_m{1}, maximum_correction_m{.75};
  bool reachable_projection_enabled{false};
  double projection_maximum_translation_m{.03}, projection_maximum_speed_m_s{.5};
};
struct SharedRootBuiltTargets {
  bool valid{false}, intent_evidence_valid{false};
  bool stream_discontinuity{false};
  std::string_view detail{"not_built"};
  SparkUpperTargets raw, filtered;
  SparkUpperTargets raw_preference, filtered_preference;
  Eigen::Vector3d reachable_translation{Eigen::Vector3d::Zero()};
  Pose left_intent, right_intent;
  Vec6 left_intent_twist{Vec6::Zero()}, right_intent_twist{Vec6::Zero()};
  std::uint64_t sequence{0}, epoch{0}, generation{0};
  std::int64_t source_timestamp_ns{0}, receive_monotonic_ns{0};
};
class SharedRootTargetBuilder {
 public:
  explicit SharedRootTargetBuilder(SharedRootBuilderConfig config);
  SharedRootBuiltTargets update(const SharedRootInput&,const MorphologyEstimate&);
  void reset() noexcept;
  // Called on a short input interruption; scales stay locked, derivatives don't.
  void resetDerivatives() noexcept { intent_history_valid_=false; }
  // Rebuild filter/selector history across a gap while retaining the intent
  // scale for the same identity. Full epoch/session change must use reset().
  void beginRecovery() noexcept;
  void setElbowHistory(const SharedRootElbowHistory& h) noexcept { elbow_history_=h; }
 private:
  SharedRootBuiltTargets updateCandidate(const SharedRootInput&,const MorphologyEstimate&);
  SharedRootBuilderConfig config_;
  UpperSparkSkeletonScaler scaler_;
  SharedRootBuiltTargets previous_;
  SharedRootElbowHistory elbow_history_;
  Eigen::Vector3d center_{Eigen::Vector3d::Zero()}, relation_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d filtered_scale_{Eigen::Vector3d::Ones()}, intent_scale_{Eigen::Vector3d::Ones()};
  bool initialized_{false}, intent_upgraded_{false}, intent_history_valid_{false};
  bool intent_scale_locked_{false};
};
SparkUpperTargets blendSharedRootTargets(const SparkUpperTargets& start,
                                         const SparkUpperTargets& goal,double alpha);
} // namespace tianji_qp_ik
