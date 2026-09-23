#include "tianji_qp_ik/shared_root_pipeline.hpp"
#include <gtest/gtest.h>

namespace tianji_qp_ik {
namespace {
SharedRootOptions options() {
  SharedRootOptions o;
  auto& g=o.builder.geometry;
  g.left_shoulder={0,.21,1.121};g.right_shoulder={0,-.21,1.121};
  g.left_upper_arm_local=g.right_upper_arm_local={.28,0,0};
  g.left_forearm_local=g.right_forearm_local={.31,0,0};
  g.left_wrist_to_palm_local=g.right_wrist_to_palm_local={.13,0,0};
  return o;
}
PicoTeleopFrame frame(std::uint64_t n,double z=0,double size=1) {
  PicoTeleopFrame f;f.sequence=n;f.tracking_epoch=1;
  f.source_timestamp_ns=static_cast<std::int64_t>(n)*10000000;
  f.receive_monotonic_ns=f.source_timestamp_ns+1000000000;
  auto& s=f.upper_limb_skeleton;s.valid=s.rotations_valid=true;
  for(std::size_t side=0;side<2;++side) {
    const double y=side==0?.2:-.2;
    s.points[side*4]={0,y,1.121};
    s.points[side*4+1]={.25*size,y,1.121};
    s.points[side*4+2]={.50*size,y,1.121+z};
    s.points[side*4+3]={.57*size,y,1.121+z};
  }
  return f;
}
SharedRootTargets model() {
  SharedRootTargets t;t.valid=true;
  t.left.shoulder={0,.21,1.121};t.right.shoulder={0,-.21,1.121};
  t.left.elbow={.1,.3,1};t.right.elbow={.1,-.3,1};
  t.left.wrist={.2,.3,.9};t.right.wrist={.2,-.3,.9};
  t.left.hand=t.left.palm.position={.3,.3,.9};
  t.right.hand=t.right.palm.position={.3,-.3,.9};return t;
}
SharedRootContinuityOutput advance(SharedRootPipeline& p,std::uint64_t first,std::uint64_t last,
                                   double z=0,double size=1) {
  SharedRootContinuityOutput out;
  for(auto n=first;n<=last;++n) {
    auto f=frame(n,z,size);p.observe(f,f.receive_monotonic_ns);
    out=p.step(f.receive_monotonic_ns,true,model());
  }
  return out;
}
TEST(SharedRootPipeline, FullChainRecoversAfterGapWithoutNewHandMotion) {
  SharedRootPipeline p(options());
  auto out=advance(p,1,20);ASSERT_TRUE(out.valid);ASSERT_EQ(out.alpha,1);
  ASSERT_TRUE(p.accept(20,1,0));
  EXPECT_FALSE(p.step(1400000000,true,model()).valid);
  auto f=frame(41,.05);ASSERT_TRUE(p.observe(f,f.receive_monotonic_ns));
  EXPECT_FALSE(p.candidate().intent_evidence_valid);
  EXPECT_FALSE(p.step(f.receive_monotonic_ns,true,model()).valid);
  out=advance(p,42,60,.05);ASSERT_TRUE(out.valid);EXPECT_EQ(out.alpha,1);
  EXPECT_EQ(out.state,SharedRootState::kRecovering);
  EXPECT_TRUE(out.suppress_stationary_hold);
  EXPECT_NEAR(out.target.left.palm.position.z(),p.candidate().filtered.left.palm.position.z(),1e-12);
  ASSERT_TRUE(p.accept(60,1,0));
  EXPECT_EQ(p.step(1600000000,true,model()).state,SharedRootState::kTracking);
}
TEST(SharedRootPipeline, RevocationCannotReuseCandidateOrRepeatedSource) {
  SharedRootPipeline p(options());advance(p,1,20);
  ASSERT_FALSE(p.step(1200000000,false,model()).valid);
  auto f=frame(20);EXPECT_FALSE(p.observe(f,f.receive_monotonic_ns));
  EXPECT_FALSE(p.step(1200000000,true,model()).valid);
  auto out=advance(p,21,25);EXPECT_EQ(out.state,SharedRootState::kRecovering);
  EXPECT_EQ(out.alpha,0);
}
TEST(SharedRootPipeline, InvalidScaleRebuildsFromEmptyWindowNotShadow) {
  SharedRootPipeline p(options());advance(p,1,20);ASSERT_TRUE(p.accept(20,1,0));
  auto out=advance(p,21,47,0,1.25);
  EXPECT_FALSE(out.valid);
  EXPECT_LT(p.morphology().samples,15U);
  out=advance(p,48,90,0,1.25);
  ASSERT_TRUE(out.valid);EXPECT_EQ(p.morphology().state,MorphologyState::kConfident);
  EXPECT_EQ(out.alpha,1);EXPECT_TRUE(p.accept(90,1,0));
}
TEST(SharedRootPipeline, DuplicatesAndOldPacketsCannotTeachScale) {
  SharedRootPipeline p(options());auto f=frame(1);
  ASSERT_TRUE(p.observe(f,f.receive_monotonic_ns));
  for(int i=0;i<20;++i) EXPECT_FALSE(p.observe(f,f.receive_monotonic_ns));
  EXPECT_EQ(p.morphology().samples,1U);
  auto stale=frame(2);EXPECT_FALSE(p.observe(stale,2000000000));
  EXPECT_EQ(p.morphology().samples,1U);
  EXPECT_FALSE(p.step(2000000000,true,model()).valid);
}
TEST(SharedRootPipeline, EpochChangeClearsScaleIntentAndRecoveryEvidence) {
  SharedRootPipeline p(options());advance(p,1,20);ASSERT_TRUE(p.accept(20,1,0));
  auto f=frame(21);f.tracking_epoch=2;
  ASSERT_TRUE(p.observe(f,f.receive_monotonic_ns));
  EXPECT_FALSE(p.candidate().intent_evidence_valid);
  EXPECT_EQ(p.morphology().samples,1U);
  EXPECT_FALSE(p.step(f.receive_monotonic_ns,true,model()).valid);
  EXPECT_FALSE(p.accept(20,1,0));
}
TEST(SharedRootPipeline, GapRebasesBuilderEvenWithoutAnInterveningControlTick) {
  SharedRootPipeline p(options());advance(p,1,20);ASSERT_TRUE(p.accept(20,1,0));
  auto f=frame(60,.05);ASSERT_TRUE(p.observe(f,f.receive_monotonic_ns));
  EXPECT_FALSE(p.candidate().intent_evidence_valid);
  EXPECT_FALSE(p.step(f.receive_monotonic_ns,true,model()).valid);
  auto out=advance(p,61,80,.05);ASSERT_TRUE(out.valid);
  EXPECT_TRUE(p.accept(80,1,0));
}
TEST(SharedRootPipeline, RejectedPalmGateDoesNotTeachMorphologyOrKeepFresh) {
  SharedRootPipeline p(options());advance(p,1,20);ASSERT_TRUE(p.accept(20,1,0));
  auto bad=frame(21,0,1.05);
  // Rigid offset preserves sane lengths but violates the mapped workspace gate.
  for(auto& point:bad.upper_limb_skeleton.points)point.x()+=10;
  EXPECT_FALSE(p.observe(bad,bad.receive_monotonic_ns));
  EXPECT_EQ(p.morphology().samples,20U);
  auto held=p.step(bad.receive_monotonic_ns,true,model());
  EXPECT_EQ(held.state,SharedRootState::kHoldLastMapped);
  EXPECT_EQ(held.receive_monotonic_ns,1200000000);
  auto f=frame(22);ASSERT_TRUE(p.observe(f,f.receive_monotonic_ns));
  EXPECT_FALSE(p.candidate().intent_evidence_valid);
}

// Synthetic bridge frames using the measured/derived zhoujie geometry snapshot:
// pico-simple/cal-91c71441c51b47db9366390c96c96199, height 1.62 m.
// Manifest SHA256: 5fd97dee1c3342c6e7c2d488f2cccba8ea75135208d198ea104d4a5627e0df71.
// Shoulder width and motion below are synthetic, NOT personal measurements.
PicoTeleopFrame zhoujieFrame(std::uint64_t n, double bend=0) {
  constexpr double upper=.25252884000000003, forearm=.24776442;
  constexpr double wrist_palm=.05999994;
  auto f=frame(n);
  for(std::size_t side=0;side<2;++side) {
    const auto base=side*4;
    const Eigen::Vector3d direction(std::cos(bend),0,std::sin(bend));
    f.upper_limb_skeleton.points[base+1]=f.upper_limb_skeleton.points[base]+Eigen::Vector3d(upper,0,0);
    f.upper_limb_skeleton.points[base+2]=f.upper_limb_skeleton.points[base+1]+forearm*direction;
    f.upper_limb_skeleton.points[base+3]=f.upper_limb_skeleton.points[base+2]+wrist_palm*direction;
  }
  return f;
}
TEST(SharedRootPipeline, ZhoujieSymmetricGeometryUsesCommonScaleWithoutCalibration) {
  auto config=loadSharedRootOptions(TIANJI_PROJECT_SOURCE_DIR "/config/qp_ik_pico_shared_root_dls.yaml");
  EXPECT_FALSE(config.enabled); // Offline pipeline test does not enable any entry point.
  SharedRootPipeline p(config);
  for(std::uint64_t n=1;n<=40;++n) {
    auto f=zhoujieFrame(n);
    ASSERT_TRUE(p.observe(f,f.receive_monotonic_ns));
    p.step(f.receive_monotonic_ns,true,model());
  }
  ASSERT_EQ(p.morphology().state,MorphologyState::kConfident);
  const double scale=config.morphology.robot_reach_m/(.25252884000000003+.24776442+.05999994);
  EXPECT_NEAR(p.morphology().reach_scale,scale,1e-12);
  EXPECT_NEAR(p.morphology().lateral_scale,config.morphology.robot_width_m/.4,1e-12);
  const auto& target=p.candidate();
  ASSERT_TRUE(target.valid);
  // Morphology scales the preference; the active DLS profile then applies a
  // separately bounded reachable projection to the final palm targets.
  EXPECT_NEAR(target.raw_preference.left.palm.position.x()-config.builder.o_B.x(),
              config.morphology.robot_reach_m,1e-12);
  EXPECT_NEAR(target.raw.right.palm.position.x(),target.raw.left.palm.position.x(),1e-12);
  EXPECT_NEAR(target.raw.left.palm.position.z(),1.121,1e-12);
  EXPECT_NEAR((target.raw.left.palm.position-target.raw.right.palm.position).norm(),.423,1e-12);
  EXPECT_NEAR(target.left_intent_twist.norm(),0,1e-12);
  EXPECT_NEAR(target.right_intent_twist.norm(),0,1e-12);
}
TEST(SharedRootPipeline, ZhoujieRecoveryAcceptsStationaryNewPoseAndRejectsOldAck) {
  auto config=loadSharedRootOptions(TIANJI_PROJECT_SOURCE_DIR "/config/qp_ik_pico_shared_root_dls.yaml");
  SharedRootPipeline p(config);
  SharedRootTargets recovery_model;
  for(std::uint64_t n=1;n<=40;++n) {
    auto f=zhoujieFrame(n);ASSERT_TRUE(p.observe(f,f.receive_monotonic_ns));
    if(n==1)recovery_model=p.candidate().filtered;
    p.step(f.receive_monotonic_ns,true,recovery_model);
  }
  ASSERT_TRUE(p.accept(40,1,0));
  EXPECT_FALSE(p.step(1600000000,true,recovery_model).valid);
  SharedRootContinuityOutput out;
  // Move during the gap, then remain perfectly still throughout recovery.
  for(std::uint64_t n=70;n<=110;++n) {
    auto f=zhoujieFrame(n,.2);ASSERT_TRUE(p.observe(f,f.receive_monotonic_ns));
    out=p.step(f.receive_monotonic_ns,true,recovery_model);
    if(n<110&&out.valid&&out.alpha<1)(void)p.accept(n,1,0);
    if(n==70) { EXPECT_FALSE(p.candidate().intent_evidence_valid); }
  }
  ASSERT_TRUE(out.valid);EXPECT_EQ(out.alpha,1);
  EXPECT_TRUE(out.suppress_stationary_hold);
  EXPECT_FALSE(p.accept(40,1,0));
  EXPECT_TRUE(p.accept(110,1,0));
  EXPECT_EQ(p.step(2100000000,true,recovery_model).state,SharedRootState::kTracking);
  EXPECT_NEAR(out.target.left.palm.position.x(),out.target.right.palm.position.x(),1e-12);
  EXPECT_GT(out.target.left.palm.position.z(),1.121);
}
} // namespace
} // namespace tianji_qp_ik
