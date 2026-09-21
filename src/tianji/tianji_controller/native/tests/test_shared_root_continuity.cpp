#include "tianji_qp_ik/shared_root_continuity.hpp"
#include <gtest/gtest.h>
namespace tianji_qp_ik {
namespace {
SharedRootBuiltTargets target(std::uint64_t n,std::int64_t ms,double x=0) {
  SharedRootBuiltTargets t; t.valid=t.filtered.valid=true;
  t.epoch=1; t.sequence=n; t.source_timestamp_ns=ms*1000000;
  t.receive_monotonic_ns=(ms+1000)*1000000;
  t.filtered.left.palm.position.x()=x; t.filtered.right.palm.position.x()=x;
  return t;
}
TEST(SharedRootContinuity, FreshFramesAndFinalAcceptanceRequired) {
  SharedRootContinuity c;
  auto model=target(1,1).filtered;
  for(std::uint64_t n=1;n<=5;++n) {auto f=target(n,static_cast<std::int64_t>(n)*10,1); c.observe(f,f.receive_monotonic_ns);}
  auto r=c.step(1050000000,true,model);
  ASSERT_EQ(r.state,SharedRootState::kRecovering);
  EXPECT_DOUBLE_EQ(r.alpha,0);
  for(std::uint64_t n=6;n<=20;++n) {auto f=target(n,static_cast<std::int64_t>(n)*10,1); c.observe(f,f.receive_monotonic_ns); r=c.step(f.receive_monotonic_ns,true,model);}
  EXPECT_DOUBLE_EQ(r.alpha,1);
  EXPECT_TRUE(r.suppress_stationary_hold);
  EXPECT_FALSE(c.accept(19,1,0));
  EXPECT_TRUE(c.accept(20,1,0));
  EXPECT_EQ(c.step(1200000000,true,model).state,SharedRootState::kTracking);
}
TEST(SharedRootContinuity, DuplicatesCannotBecomeRecoveryEvidenceOrKeepFresh) {
  SharedRootContinuity c; auto f=target(1,10);
  for(int i=0;i<20;++i)c.observe(f,f.receive_monotonic_ns);
  EXPECT_EQ(c.step(f.receive_monotonic_ns,true,f.filtered).state,SharedRootState::kUninitialized);
  EXPECT_FALSE(c.step(2000000000,true,f.filtered).valid);
}
TEST(SharedRootContinuity, AuthorizationAndEpochDiscardOldContext) {
  SharedRootContinuity c; auto f=target(1,10);
  for(std::uint64_t n=1;n<=5;++n){f=target(n,static_cast<std::int64_t>(n)*10);c.observe(f,f.receive_monotonic_ns);}
  ASSERT_TRUE(c.step(f.receive_monotonic_ns,true,f.filtered).valid);
  EXPECT_FALSE(c.step(f.receive_monotonic_ns,false,f.filtered).valid);
  EXPECT_FALSE(c.step(f.receive_monotonic_ns,true,f.filtered).valid);
  f=target(1,60); f.epoch=2; c.observe(f,f.receive_monotonic_ns);
  EXPECT_FALSE(c.step(f.receive_monotonic_ns,true,f.filtered).valid);
}
TEST(SharedRootContinuity, LostInputThenStationaryNewPoseStillRecovers) {
  SharedRootContinuity c; auto model=target(1,1).filtered;
  SharedRootContinuityOutput r;
  for(std::uint64_t n=1;n<=20;++n) {
    auto f=target(n,static_cast<std::int64_t>(n)*10,.2);
    c.observe(f,f.receive_monotonic_ns);r=c.step(f.receive_monotonic_ns,true,model);
  }
  ASSERT_TRUE(c.accept(20,1,0));
  EXPECT_FALSE(c.step(1400000000,true,model).valid);
  // Hand moved to .4 while packets were absent, and is stationary on return.
  for(std::uint64_t n=41;n<=60;++n) {
    auto f=target(n,static_cast<std::int64_t>(n)*10,.4);
    c.observe(f,f.receive_monotonic_ns);r=c.step(f.receive_monotonic_ns,true,model);
  }
  ASSERT_TRUE(r.valid);EXPECT_DOUBLE_EQ(r.alpha,1);
  EXPECT_NEAR(r.target.left.palm.position.x(),.4,1e-12);
  EXPECT_TRUE(c.accept(60,1,0));
}
TEST(SharedRootContinuity, InvalidFrameHoldsWithoutRefreshingTimestamp) {
  SharedRootContinuity c;auto model=target(1,1).filtered;
  for(std::uint64_t n=1;n<=5;++n) {
    auto f=target(n,static_cast<std::int64_t>(n)*10);
    c.observe(f,f.receive_monotonic_ns);c.step(f.receive_monotonic_ns,true,model);
  }
  auto bad=target(6,60);bad.valid=false;c.observe(bad,bad.receive_monotonic_ns);
  auto out=c.step(bad.receive_monotonic_ns,true,model);
  EXPECT_TRUE(out.valid);EXPECT_EQ(out.state,SharedRootState::kHoldLastMapped);
  EXPECT_EQ(out.receive_monotonic_ns,1050000000);
  EXPECT_FALSE(c.step(1160000000,true,model).valid);
}
TEST(SharedRootContinuity, ExplicitDiscontinuityCannotReuseRecoveryFrames) {
  SharedRootContinuity c;auto model=target(1,1).filtered;
  for(std::uint64_t n=1;n<=5;++n) {
    auto f=target(n,static_cast<std::int64_t>(n)*10);
    c.observe(f,f.receive_monotonic_ns);
  }
  ASSERT_TRUE(c.step(1050000000,true,model).valid);
  auto f=target(6,60);f.stream_discontinuity=true;c.observe(f,f.receive_monotonic_ns);
  EXPECT_FALSE(c.step(f.receive_monotonic_ns,true,model).valid);
}
SharedRootClosureGeometry closedGeometry() {
  SharedRootClosureGeometry g;
  for(auto& s:g){s.upper_length_m=s.forearm_length_m=.5;s.tcp_to_wrist_center.position={0,0,-.1};}
  return g;
}
SharedRootBuiltTargets closedTarget(std::uint64_t n,double x=.6) {
  auto t=target(n,static_cast<std::int64_t>(n)*10,x);
  for(auto* s:{&t.filtered.left,&t.filtered.right}) {
    s->palm.position.z()=.1;s->elbow={.3,.4,0};
  }
  t.filtered_preference=t.filtered;
  EXPECT_TRUE(closeSharedRootTargets(t.filtered,closedGeometry(),{}));
  return t;
}
TEST(SharedRootContinuity, R3BlendClosureAtomicHistoryAndStaleAck) {
  SharedRootContinuity c({},closedGeometry());auto model=closedTarget(1,.5).filtered;
  SharedRootContinuityOutput r;
  for(std::uint64_t n=1;n<=20;++n) {
    auto f=closedTarget(n);c.observe(f,f.receive_monotonic_ns);
    r=c.step(f.receive_monotonic_ns,true,model);
    if(!r.valid)continue;
    EXPECT_FALSE(c.accept(n,2,0));
    if(n==5){EXPECT_FALSE(c.acceptedElbows()[0]);}
    for(const auto* s:{&r.target.left,&r.target.right}) {
      EXPECT_EQ(s->hand,s->palm.position);
      EXPECT_NEAR((s->elbow-s->shoulder).norm(),.5,1e-9);
      EXPECT_NEAR((s->wrist-s->elbow).norm(),.5,1e-9);
    }
    (void)c.accept(n,1,0);
    ASSERT_TRUE(c.acceptedElbows()[0]);ASSERT_TRUE(c.acceptedElbows()[1]);
    EXPECT_EQ(*c.acceptedElbows()[0],r.target.left.elbow);
    EXPECT_FALSE(c.accept(n,1,0));
  }
  auto f=closedTarget(21,.65);auto old=*c.acceptedElbows()[0];
  c.observe(f,f.receive_monotonic_ns);r=c.step(f.receive_monotonic_ns,true,model);
  ASSERT_EQ(r.state,SharedRootState::kTracking);ASSERT_TRUE(r.valid);
  EXPECT_EQ(*c.acceptedElbows()[0],old); // candidate is not acceptance
  (void)c.accept(21,1,0);EXPECT_EQ(*c.acceptedElbows()[0],r.target.left.elbow);
  auto bad=closedTarget(22);bad.valid=false;c.observe(bad,bad.receive_monotonic_ns);
  auto held=c.step(bad.receive_monotonic_ns,true,model);
  EXPECT_EQ(held.state,SharedRootState::kHoldLastMapped);
  EXPECT_EQ(held.target.left.elbow,r.target.left.elbow);
  EXPECT_FALSE(c.accept(22,1,0));
  bad=closedTarget(23);bad.epoch=2;c.observe(bad,bad.receive_monotonic_ns);
  EXPECT_FALSE(c.acceptedElbows()[0]);EXPECT_FALSE(c.accept(21,1,0));
  c.step(bad.receive_monotonic_ns,false,model);EXPECT_FALSE(c.elbowHistory()[0]);
}
TEST(SharedRootContinuity, R3InfeasibleIntermediateBlendNeverEmitsStretchedChain) {
  auto geometry=closedGeometry();for(auto& g:geometry){g.upper_length_m=.5;g.forearm_length_m=.2;}
  // Isolate workspace rejection from the separate branch-jump safety test.
  SharedRootContinuityConfig config;config.maximum_elbow_step_m=10;
  SharedRootContinuity c(config,geometry);
  auto model=closedTarget(1,-.5).filtered;
  ASSERT_TRUE(closeSharedRootTargets(model,geometry,{}));
  bool rejected=false;
  for(std::uint64_t n=1;n<=20;++n) {
    auto f=closedTarget(n,.5);ASSERT_TRUE(closeSharedRootTargets(f.filtered,geometry,{}));
    c.observe(f,f.receive_monotonic_ns);const auto r=c.step(f.receive_monotonic_ns,true,model);
    if(r.valid)(void)c.accept(n,1,0);
    if(r.target.detail=="MappedPalmOutsideGeometricWorkspace")rejected=true;
  }
  EXPECT_TRUE(rejected);
}
TEST(SharedRootContinuity, R3BranchJumpCannotCommitEitherSide) {
  SharedRootContinuity c({},closedGeometry());auto model=closedTarget(1).filtered;
  for(std::uint64_t n=1;n<=20;++n) {
    auto f=closedTarget(n);c.observe(f,f.receive_monotonic_ns);
    if(c.step(f.receive_monotonic_ns,true,model).valid)(void)c.accept(n,1,0);
  }
  ASSERT_TRUE(c.acceptedElbows()[0]);const auto old=c.acceptedElbows();
  auto f=closedTarget(21);f.filtered_preference.left.elbow.y()=-.4;
  c.observe(f,f.receive_monotonic_ns);const auto r=c.step(f.receive_monotonic_ns,true,model);
  EXPECT_FALSE(r.valid);EXPECT_EQ(r.target.detail,"ElbowBranchDiscontinuity");
  EXPECT_FALSE(c.accept(21,1,0));EXPECT_EQ(c.acceptedElbows(),old);
}
} // namespace
} // namespace tianji_qp_ik
