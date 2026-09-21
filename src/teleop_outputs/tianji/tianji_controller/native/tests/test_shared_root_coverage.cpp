#include "../apps/shared_root_coverage.hpp"
#include "../apps/shared_root_mapping_probe.hpp"
#include <gtest/gtest.h>

TEST(SharedRootCoverage, FreshSamplesAndExplicitEnd) {
  SharedRootCoverageTimeline t(100);
  t.observe(1000,true);t.observe(1050,true);t.observe(1100,true);
  EXPECT_EQ(t.through(1100).invalidDuration(),0);
  EXPECT_EQ(t.through(1100).duration(),100);
  EXPECT_EQ(t.through(1300).maximumInvalidDuration(),100);
  EXPECT_EQ(t.through(1150).through(1300).maximumInvalidDuration(),100);
  EXPECT_EQ(t.through(1150).through(1300).duration(),300);
  EXPECT_EQ(t.through(1100).maximumInvalidDuration(),0); // const query
}
TEST(SharedRootCoverage, SilenceJoinsSubsequentInvalidFrames) {
  SharedRootCoverageTimeline t(100);
  t.observe(1000,true);t.observe(1300,false);t.observe(1350,false);
  t.observe(1400,true);t.observe(1450,false);t.observe(1500,true);
  EXPECT_EQ(t.duration(),500);
  EXPECT_EQ(t.invalidDuration(),350);
  EXPECT_EQ(t.maximumInvalidDuration(),300);
}
TEST(SharedRootCoverage, InvalidAtStartAndEqualReceiveTimes) {
  SharedRootCoverageTimeline t(100);
  t.observe(1000,false);t.observe(1000,false);t.observe(1200,true);
  EXPECT_EQ(t.maximumInvalidDuration(),200);
  EXPECT_EQ(t.through(1200).duration(),200);
}
TEST(SharedRootCoverage, RejectRegressionsAndUndefinedWindow) {
  EXPECT_THROW(SharedRootCoverageTimeline(0),std::invalid_argument);
  SharedRootCoverageTimeline t(100);
  EXPECT_THROW(t.through(1000),std::invalid_argument);
  t.observe(1000,true);
  EXPECT_THROW(t.observe(999,true),std::invalid_argument);
  EXPECT_THROW(t.through(999),std::invalid_argument);
}

TEST(SharedRootMappingProbe, FeasibleAndDisjointBallsRemainDistinct) {
  tianji_qp_ik::MappingProbeBalls balls;balls.count=2;
  balls.center[0]={.02,0,0};balls.radius[0]=.01;
  balls.center[1].setZero();balls.radius[1]=.02;
  EXPECT_LE(balls.maximumResidual(balls.solve(64)),1e-10);
  balls.radius[1]=.005;
  EXPECT_NEAR(balls.maximumResidual(balls.solve(64)),.005,1e-12);
  EXPECT_NEAR(balls.maximumResidual(balls.solve(4096)),.005,1e-12);
}

namespace {
std::string probeDegenerateSpan(bool reset) {
  using namespace tianji_qp_ik;
  MappingTransitionProbe probe;SharedRootBuilderConfig config;
  SharedRootClosureGeometry geometry;
  for(auto& g:geometry){g.upper_length_m=g.forearm_length_m=.5;}
  config.closure_geometry=geometry;
  testing::internal::CaptureStdout();
  for(int n=1;n<=3;++n) {
    SharedRootBuiltTargets target;target.valid=target.raw.valid=target.filtered.valid=true;
    target.sequence=static_cast<std::uint64_t>(n);target.source_timestamp_ns=n*10000000;
    target.receive_monotonic_ns=target.source_timestamp_ns+1000000000;
    for(auto* layer:{&target.raw,&target.filtered})for(auto* arm:{&layer->left,&layer->right}) {
      arm->shoulder.setZero();arm->wrist={n==2?1.:.6,0,0};
      arm->elbow={n==2?.5:.3,n==1?.4:n==2?0.:-.4,0};
      arm->palm.position=arm->hand=arm->wrist;
    }
    probe.observe(target,config,n>1&&!(reset&&n==3),1000000000);
  }
  probe.report();return testing::internal::GetCapturedStdout();
}
}
TEST(SharedRootMappingProbe, DetectsOpposedEndpointsAcrossStraightArm) {
  const auto text=probeDegenerateSpan(false);
  EXPECT_NE(text.find("compared=4 opposed=4 unresolved=0"),std::string::npos);
  EXPECT_NE(text.find("degenerate_frames=1 transported_cosine=-1"),std::string::npos);
  // Undefined adjacent directions must not be manufactured from moving circles.
  EXPECT_EQ(text.find("elbow_direction_event "),std::string::npos);
}
TEST(SharedRootMappingProbe, ResetDoesNotInventBranchContinuity) {
  const auto text=probeDegenerateSpan(true);
  EXPECT_NE(text.find("compared=0 opposed=0 unresolved=0"),std::string::npos);
  EXPECT_NE(text.find("history_starts=2"),std::string::npos);
}
