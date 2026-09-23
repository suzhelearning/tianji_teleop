#include "tianji_qp_ik/shared_root_target_builder.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

namespace tianji_qp_ik {
namespace {
SharedRootBuilderConfig config() {
  SharedRootBuilderConfig c;
  c.geometry.left_shoulder={0,.21,1.121}; c.geometry.right_shoulder={0,-.21,1.121};
  c.geometry.left_upper_arm_local=c.geometry.right_upper_arm_local={.28,0,0};
  c.geometry.left_forearm_local=c.geometry.right_forearm_local={.31,0,0};
  c.geometry.left_wrist_to_palm_local=c.geometry.right_wrist_to_palm_local={.13,0,0};
  return c;
}
SharedRootInput input(std::uint64_t n) {
  SharedRootInput f; f.valid=true; f.sequence=n; f.tracking_epoch=1;
  f.source_timestamp_ns=static_cast<std::int64_t>(n)*10000000;
  f.receive_monotonic_ns=f.source_timestamp_ns+1000000000;
  for (auto* s:{&f.left,&f.right}) {
    const double y=s==&f.left ? .2 : -.2;
    s->p_shoulder_root_Ct={0,y,0}; s->p_elbow_root_Ct={.25,y,0};
    s->p_wrist_root_Ct={.5,y,0}; s->p_control_root_Ct={.57,y,0};
    s->p_shape_proxy_root_Ct=s->p_control_root_Ct;
  }
  return f;
}
MorphologyEstimate scale() {
  MorphologyEstimate m; m.valid=m.new_sample=true;
  m.reach_scale=1.1; m.lateral_scale=1.05; m.state=MorphologyState::kProvisional;
  return m;
}
TEST(SharedRootTargetBuilder, PreservesRawRelationAndSeparatesPalmFromShape) {
  SharedRootTargetBuilder b(config());
  const auto out=b.update(input(1),scale());
  ASSERT_TRUE(out.valid);
  EXPECT_NEAR(out.raw.right.palm.position.y()-out.raw.left.palm.position.y(),-.42,1e-12);
  EXPECT_NEAR(out.raw.left.palm.position.x(),.627,1e-12);
  EXPECT_NEAR(out.raw.left.palm.position.z(),1.121,1e-12);
  EXPECT_NEAR(out.raw.left.hand.x(),.72,1e-12);
  EXPECT_FALSE(out.intent_evidence_valid);
}
TEST(SharedRootTargetBuilder, R3ClosesBothLayersWithoutCommittingHistory) {
  auto c=config();SharedRootClosureGeometry geometry;
  for(int side=0;side<2;++side) {
    auto& g=geometry[side];g.shoulder_B=side?c.geometry.right_shoulder:c.geometry.left_shoulder;
    g.upper_length_m=.28;g.forearm_length_m=.31;
    g.tcp_to_wrist_center.position={-.13,0,0};
  }
  c.closure_geometry=geometry;SharedRootTargetBuilder builder(c);
  for(std::uint64_t n=1;n<=30;++n) {
    auto f=input(n);f.left.p_elbow_root_Ct.y()+=.08;f.right.p_elbow_root_Ct.y()-=.08;
    f.left.p_control_root_Ct.z()=f.left.p_shape_proxy_root_Ct.z()=.001*double(n);
    auto out=builder.update(f,scale());ASSERT_TRUE(out.valid)<<out.detail;
    for(const auto* layer:{&out.raw,&out.filtered})for(int side=0;side<2;++side) {
      const auto& t=side?layer->right:layer->left;
      EXPECT_EQ(t.hand,t.palm.position);
      EXPECT_NEAR((t.elbow-t.shoulder).norm(),.28,1e-9);
      EXPECT_NEAR((t.wrist-t.elbow).norm(),.31,1e-9);
      EXPECT_LT((t.wrist-t.palm.position-t.palm.rotation*geometry[side].tcp_to_wrist_center.position).norm(),1e-12);
    }
  }
  auto invalid=input(31);invalid.left.p_control_root_Ct={.9,.2,0};
  EXPECT_FALSE(builder.update(invalid,scale()).valid);
}
TEST(SharedRootTargetBuilder, ReachableProjectionPreservesRelationAndHonorsBounds) {
  auto c=config();SharedRootClosureGeometry geometry;
  for(int side=0;side<2;++side) {
    auto& g=geometry[side];g.shoulder_B=side?c.geometry.right_shoulder:c.geometry.left_shoulder;
    g.upper_length_m=.28;g.forearm_length_m=.31;g.tcp_to_wrist_center.position={-.13,0,0};
  }
  c.closure_geometry=geometry;
  auto f=input(1);auto m=scale();m.reach_scale=m.lateral_scale=1;
  for(auto* s:{&f.left,&f.right}) {
    s->p_control_root_Ct.x()=.735;s->p_shape_proxy_root_Ct=s->p_control_root_Ct;
    s->p_elbow_root_Ct.z()=.1;
  }
  EXPECT_FALSE(SharedRootTargetBuilder(c).update(f,m).valid);
  c.reachable_projection_enabled=true;
  SharedRootTargetBuilder b(c);
  const auto out=b.update(f,m);ASSERT_TRUE(out.valid)<<out.detail;
  EXPECT_GT(out.reachable_translation.norm(),.015);
  EXPECT_LE(out.reachable_translation.norm(),.03+1e-10);
  EXPECT_LT(((out.raw.right.palm.position-out.raw.left.palm.position)-
    (out.raw_preference.right.palm.position-out.raw_preference.left.palm.position)).norm(),1e-12);
  EXPECT_EQ(out.raw.left.palm.rotation,out.raw_preference.left.palm.rotation);
  EXPECT_FALSE(out.intent_evidence_valid);
  for(int side=0;side<2;++side) {
    const auto& arm=side?out.filtered.right:out.filtered.left;
    EXPECT_NEAR((arm.elbow-arm.shoulder).norm(),.28,1e-9);
    EXPECT_NEAR((arm.wrist-arm.elbow).norm(),.31,1e-9);
    EXPECT_EQ(arm.hand,arm.palm.position);
  }
  auto next=f;next.sequence=2;next.source_timestamp_ns+=10000000;next.receive_monotonic_ns+=10000000;
  const auto accepted=b.update(next,m);ASSERT_TRUE(accepted.valid)<<accepted.detail;
  EXPECT_LE((accepted.reachable_translation-out.reachable_translation).norm(),.005+1e-10);
  auto excessive=next;excessive.sequence=3;excessive.source_timestamp_ns+=10000000;excessive.receive_monotonic_ns+=10000000;
  excessive.left.p_control_root_Ct.x()=excessive.right.p_control_root_Ct.x()=.85;
  SharedRootTargetBuilder reference=b;
  const auto rejected=b.update(excessive,m);EXPECT_FALSE(rejected.valid);
  next.sequence=4;next.source_timestamp_ns+=20000000;next.receive_monotonic_ns+=20000000;
  EXPECT_EQ(b.update(next,m).filtered.left.palm.position,reference.update(next,m).filtered.left.palm.position);
}
TEST(SharedRootTargetBuilder, ReachableProjectionDisabledAndAlreadyReachableAreIdentical) {
  auto c=config();SharedRootClosureGeometry geometry;
  for(int side=0;side<2;++side) {
    auto& g=geometry[side];g.shoulder_B=side?c.geometry.right_shoulder:c.geometry.left_shoulder;
    g.upper_length_m=.28;g.forearm_length_m=.31;g.tcp_to_wrist_center.position={-.13,0,0};
  }
  c.closure_geometry=geometry;SharedRootTargetBuilder old(c);
  c.reachable_projection_enabled=true;SharedRootTargetBuilder projected(c);
  for(int n=1;n<20;++n) {
    auto f=input(n);f.left.p_elbow_root_Ct.z()=.08;f.right.p_elbow_root_Ct.z()=.08;
    const auto a=old.update(f,scale()),b=projected.update(f,scale());
    ASSERT_TRUE(a.valid);ASSERT_TRUE(b.valid);
    EXPECT_EQ(a.filtered.left.palm.position,b.filtered.left.palm.position);
    EXPECT_EQ(a.filtered.right.palm.position,b.filtered.right.palm.position);
    EXPECT_DOUBLE_EQ(b.reachable_translation.norm(),0);
  }
}
TEST(SharedRootTargetBuilder, ReachableProjectionRejectsConflictingArmsAndSpeedJump) {
  auto c=config();SharedRootClosureGeometry geometry;
  c.maximum_correction_m=2; // Isolate incompatible reach balls from the shape gate.
  for(int side=0;side<2;++side) {
    auto& g=geometry[side];g.shoulder_B=side?c.geometry.right_shoulder:c.geometry.left_shoulder;
    g.upper_length_m=.28;g.forearm_length_m=.31;g.tcp_to_wrist_center.position={-.13,0,0};
  }
  c.closure_geometry=geometry;c.reachable_projection_enabled=true;
  auto m=scale();m.reach_scale=m.lateral_scale=1;
  auto f=input(1);f.left.p_elbow_root_Ct.z()=f.right.p_elbow_root_Ct.z()=.08;
  f.left.p_control_root_Ct.x()=.735;f.right.p_control_root_Ct.x()=-.475;
  const auto conflict=SharedRootTargetBuilder(c).update(f,m);
  EXPECT_FALSE(conflict.valid);
  SharedRootTargetBuilder b(c);f=input(1);
  f.left.p_elbow_root_Ct.z()=f.right.p_elbow_root_Ct.z()=.08;
  ASSERT_TRUE(b.update(f,m).valid);
  f.sequence=2;f.source_timestamp_ns+=10000000;f.receive_monotonic_ns+=10000000;
  f.left.p_control_root_Ct.x()=f.right.p_control_root_Ct.x()=.74;
  const auto jump=b.update(f,m);EXPECT_FALSE(jump.valid);
}
TEST(SharedRootTargetBuilder, NamedBilateralActionsPreserveRawSharedTransform) {
  struct Action {const char* name;Eigen::Vector3d left,right;};
  const std::array<Action,7> actions{{
    {"approach",{.6,.03,0},{.6,-.03,0}},
    {"separate",{.5,.5,0},{.5,-.5,0}},
    {"cross",{.6,-.1,0},{.6,.1,0}},
    {"depth_offset",{.7,.2,0},{.4,-.2,0}},
    {"height_offset",{.6,.2,.15},{.6,-.2,-.15}},
    {"left_stationary",{.57,.2,0},{.7,-.3,.1}},
    {"common_translation",{.67,.2,.1},{.67,-.2,.1}}
  }};
  for(const auto& action:actions) {
    SCOPED_TRACE(action.name);
    auto f=input(1);auto m=scale();
    f.left.p_control_root_Ct=f.left.p_shape_proxy_root_Ct=action.left;
    f.right.p_control_root_Ct=f.right.p_shape_proxy_root_Ct=action.right;
    SharedRootTargetBuilder builder(config());const auto out=builder.update(f,m);
    ASSERT_TRUE(out.valid)<<out.detail;
    const Eigen::Vector3d s(m.reach_scale,m.lateral_scale,m.reach_scale);
    EXPECT_LT(((out.raw.right.palm.position-out.raw.left.palm.position)-
        s.cwiseProduct(action.right-action.left)).norm(),1e-12);
    EXPECT_LT((.5*(out.raw.right.palm.position+out.raw.left.palm.position)-
        config().o_B-s.cwiseProduct(.5*(action.right+action.left))).norm(),1e-12);
    EXPECT_EQ(out.raw.left.palm.position,out.filtered.left.palm.position);
    EXPECT_EQ(out.raw.right.palm.position,out.filtered.right.palm.position);
  }
}
TEST(SharedRootTargetBuilder, WideBilateralReachHasNoInterPalmDistanceGate) {
  SharedRootTargetBuilder b(config());
  auto f=input(1);auto m=scale();
  m.reach_scale=m.lateral_scale=1;
  for(auto* s:{&f.left,&f.right}) {
    const double sign=s==&f.left?1:-1;
    s->p_elbow_root_Ct={0,sign*.45,0};
    s->p_wrist_root_Ct={0,sign*.75,0};
    s->p_control_root_Ct={0,sign*.9,0};
    s->p_shape_proxy_root_Ct=s->p_control_root_Ct;
  }
  for(int n=1;n<=3;++n) {
    f.sequence=n;f.source_timestamp_ns=n*10000000;
    f.receive_monotonic_ns=f.source_timestamp_ns+1000000000;
    const auto out=b.update(f,m);
    ASSERT_TRUE(out.valid)<<out.detail;
    EXPECT_NEAR((out.raw.right.palm.position-out.raw.left.palm.position).norm(),1.8,1e-12);
    EXPECT_NEAR((out.filtered.right.palm.position-out.filtered.left.palm.position).norm(),1.8,1e-12);
  }
}
TEST(SharedRootTargetBuilder, CenterAndNonfiniteGuardsRemainEnabled) {
  auto f=input(1);
  f.left.p_control_root_Ct.x()=2;f.right.p_control_root_Ct.x()=2;
  SharedRootTargetBuilder b(config());
  const auto out=b.update(f,scale());
  EXPECT_FALSE(out.valid);
  f=input(1);f.left.p_control_root_Ct.y()=std::numeric_limits<double>::infinity();
  EXPECT_FALSE(b.update(f,scale()).valid);
}
TEST(SharedRootTargetBuilder, ScaleUpgradeDoesNotCreateIntentVelocity) {
  SharedRootTargetBuilder b(config()); auto m=scale();
  ASSERT_TRUE(b.update(input(1),m).valid);
  m.state=MorphologyState::kConfident; m.reach_scale=1.2;
  auto out=b.update(input(2),m);
  ASSERT_TRUE(out.valid); EXPECT_FALSE(out.intent_evidence_valid);
  EXPECT_DOUBLE_EQ(out.left_intent_twist.norm(),0);
  m.reach_scale=1.3;
  out=b.update(input(3),m);
  ASSERT_TRUE(out.intent_evidence_valid);
  EXPECT_NEAR(out.left_intent_twist.norm(),0,1e-12);
  EXPECT_NEAR(out.left_intent.position.x(),.57*1.2,1e-12);
}
TEST(SharedRootTargetBuilder, DuplicateAndInvalidDoNotAdvanceFilter) {
  SharedRootTargetBuilder b(config()), reference(config()); auto m=scale();
  b.update(input(1),m); reference.update(input(1),m);
  auto duplicate=input(1); duplicate.left.p_control_root_Ct.x()+=.1;
  EXPECT_FALSE(b.update(duplicate,m).valid);
  auto bad=input(2); bad.right.R_palm_Ct.setZero();
  EXPECT_FALSE(b.update(bad,m).valid);
  const auto a=b.update(input(3),m), r=reference.update(input(3),m);
  ASSERT_TRUE(a.valid);
  EXPECT_EQ(a.filtered.left.palm.position,r.filtered.left.palm.position);
}
TEST(SharedRootTargetBuilder, ShortHoldRebasesIntentAndKeepsLockedScale) {
  SharedRootTargetBuilder b(config()); auto m=scale();
  b.update(input(1),m); b.resetDerivatives(); m.reach_scale=1.4;
  auto out=b.update(input(2),m);
  ASSERT_TRUE(out.valid); EXPECT_FALSE(out.intent_evidence_valid);
  EXPECT_NEAR(out.left_intent.position.x(),.57*1.1,1e-12);
}
TEST(SharedRootTargetBuilder, LongGapRecoveryHasNoDerivativeAcrossGap) {
  SharedRootTargetBuilder b(config()); auto m=scale();
  b.update(input(1),m);
  EXPECT_FALSE(b.update(input(30),m).valid);
  b.beginRecovery();m.reach_scale=1.3;
  auto out=b.update(input(30),m);
  ASSERT_TRUE(out.valid);EXPECT_FALSE(out.intent_evidence_valid);
  EXPECT_NEAR(out.left_intent.position.x(),.57*1.1,1e-12);
}
TEST(SharedRootTargetBuilder, EffectiveHumanSizesHaveSameUnclippedRawTargets) {
  SharedRootTargetBuilder baseline(config());
  const auto reference=baseline.update(input(1),scale());ASSERT_TRUE(reference.valid);
  for(double factor:{.8,1.,1.2}) {
    SharedRootTargetBuilder b(config());auto f=input(1);auto m=scale();
    for(auto* s:{&f.left,&f.right}) {
      s->p_shoulder_root_Ct*=factor;s->p_elbow_root_Ct*=factor;
      s->p_wrist_root_Ct*=factor;s->p_control_root_Ct*=factor;s->p_shape_proxy_root_Ct*=factor;
    }
    m.reach_scale/=factor;m.lateral_scale/=factor;
    const auto out=b.update(f,m);ASSERT_TRUE(out.valid);
    EXPECT_LT((out.raw.left.palm.position-reference.raw.left.palm.position).norm(),1e-12);
    EXPECT_LT((out.raw.right.palm.position-reference.raw.right.palm.position).norm(),1e-12);
  }
}
TEST(SharedRootTargetBuilder, RotationJumpFallsBackInSameRobotFrame) {
  auto c=config();c.R_BCt=Eigen::AngleAxisd(.4,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  SharedRootTargetBuilder b(c);auto m=scale();b.update(input(1),m);
  auto f=input(2);
  f.left.R_shoulder_Ct=Eigen::AngleAxisd(1.5,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto out=b.update(f,m);ASSERT_TRUE(out.valid);
  EXPECT_EQ(out.raw.left.upper_source,SharedRootSegmentSource::kPositionFallback);
  const Eigen::Vector3d expected=.28*c.R_BCt*Eigen::Vector3d::UnitX();
  EXPECT_LT((out.raw.left.elbow-out.raw.left.shoulder-expected).norm(),1e-12);
}
TEST(SharedRootTargetBuilder, EpochDropsIntentLockAndCannotReuseOldDerivatives) {
  SharedRootTargetBuilder b(config());auto m=scale();b.update(input(1),m);
  auto f=input(2);f.tracking_epoch=2;m.reach_scale=1.3;
  const auto out=b.update(f,m);ASSERT_TRUE(out.valid);
  EXPECT_FALSE(out.intent_evidence_valid);
  EXPECT_NEAR(out.left_intent.position.x(),.57*1.3,1e-12);
}
TEST(SharedRootTargetBuilder, FilterMovesWithoutOverwritingProxyAndBlendUsesOneAlpha) {
  SharedRootTargetBuilder b(config()); auto m=scale();
  const auto a=b.update(input(1),m);
  auto f=input(2); f.left.p_control_root_Ct.x()+=.03; f.right.p_control_root_Ct.x()+=.03;
  const auto z=b.update(f,m); ASSERT_TRUE(z.valid);
  EXPECT_GT(z.filtered.left.palm.position.x(),a.filtered.left.palm.position.x());
  EXPECT_LT(z.filtered.left.palm.position.x(),z.raw.left.palm.position.x());
  const auto blend=blendSharedRootTargets(a.filtered,z.filtered,.5);
  EXPECT_EQ(blend.left.palm.position,.5*(a.filtered.left.palm.position+z.filtered.left.palm.position));
  EXPECT_NE(blend.left.palm.position,blend.left.hand);
}
TEST(SharedRootTargetBuilder, OfflineScaleAblationMeasuresRawDistortionNotTracking) {
  // Test-only intervention on the morphology estimate: no runtime mode or
  // production profile changes. The same frames feed both builders.
  SharedRootTargetBuilder anisotropic(config()), isotropic(config());
  auto a_scale=scale(); a_scale.state=MorphologyState::kConfident;
  a_scale.reach_scale=1.2; a_scale.lateral_scale=.8;
  auto i_scale=a_scale; i_scale.lateral_scale=i_scale.reach_scale;
  Eigen::Vector2d amin=Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector2d amax=-amin, imin=amin, imax=-amin;
  double max_angle_a=0, max_angle_i=0;
  constexpr int samples=360;
  for(int n=0;n<samples;++n) {
    const double phase=2*std::acos(-1.)*n/samples;
    auto f=input(n+1);
    const Eigen::Vector3d circle(.06*std::cos(phase),.06*std::sin(phase),0);
    f.left.p_control_root_Ct=Eigen::Vector3d(.45,.2,0)+circle;
    f.right.p_control_root_Ct=Eigen::Vector3d(.45,-.2,0)-circle;
    f.left.p_shape_proxy_root_Ct=f.left.p_control_root_Ct;
    f.right.p_shape_proxy_root_Ct=f.right.p_control_root_Ct;
    const auto a=anisotropic.update(f,a_scale), i=isotropic.update(f,i_scale);
    ASSERT_TRUE(a.valid) << n << ": " << a.detail;
    ASSERT_TRUE(i.valid) << n << ": " << i.detail;
    const Eigen::Vector3d raw_relation=f.right.p_control_root_Ct-f.left.p_control_root_Ct;
    const Eigen::Vector3d da=a.raw.right.palm.position-a.raw.left.palm.position;
    const Eigen::Vector3d di=i.raw.right.palm.position-i.raw.left.palm.position;
    EXPECT_LT((da-Eigen::Vector3d(1.2,.8,1.2).cwiseProduct(raw_relation)).norm(),1e-12);
    EXPECT_LT((di-1.2*raw_relation).norm(),1e-12);
    auto angle=[&](const Eigen::Vector3d& d) {
      return std::atan2(raw_relation.cross(d).norm(),raw_relation.dot(d));
    };
    max_angle_a=std::max(max_angle_a,angle(da));
    max_angle_i=std::max(max_angle_i,angle(di));
    amin=amin.cwiseMin(a.raw.left.palm.position.head<2>());
    amax=amax.cwiseMax(a.raw.left.palm.position.head<2>());
    imin=imin.cwiseMin(i.raw.left.palm.position.head<2>());
    imax=imax.cwiseMax(i.raw.left.palm.position.head<2>());
    // Mapping scale must not rescale anatomical shape or apply palm basis again.
    EXPECT_EQ(a.raw.left.hand,i.raw.left.hand);
    EXPECT_EQ(a.raw.right.wrist,i.raw.right.wrist);
    EXPECT_EQ(a.raw.left.palm.rotation,i.raw.left.palm.rotation);
  }
  const double ratio_a=(amax.x()-amin.x())/(amax.y()-amin.y());
  const double ratio_i=(imax.x()-imin.x())/(imax.y()-imin.y());
  EXPECT_NEAR(ratio_a,1.5,1e-12);
  EXPECT_NEAR(ratio_i,1.,1e-12);
  EXPECT_GT(max_angle_a,.01); EXPECT_LT(max_angle_i,1e-12);
  std::cout << "scale_ablation_raw_only samples=" << samples
            << " excluded=0 anisotropic_xy_axis_ratio=" << ratio_a
            << " isotropic_xy_axis_ratio=" << ratio_i
            << " anisotropic_relation_max_angle_rad=" << max_angle_a
            << " isotropic_relation_max_angle_rad=" << max_angle_i
            << " tracking_and_reachability=not_evaluated\n";
}
} // namespace
} // namespace tianji_qp_ik
