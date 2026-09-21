#include "tianji_qp_ik/shared_root_morphology.hpp"
#include <gtest/gtest.h>
namespace tianji_qp_ik {
namespace {
SharedRootInput sample(std::uint64_t n, double scale=1.0) {
  SharedRootInput f; f.valid=true; f.sequence=n; f.tracking_epoch=1;
  f.source_timestamp_ns=static_cast<std::int64_t>(n)*10000000;
  f.receive_monotonic_ns=f.source_timestamp_ns+1000000000;
  for (auto* s : {&f.left,&f.right}) {
    double sign=s==&f.left ? 1.0 : -1.0;
    s->p_shoulder_root_Ct=Eigen::Vector3d(0,sign*.2,0)*scale;
    s->p_elbow_root_Ct=s->p_shoulder_root_Ct+Eigen::Vector3d(.27,0,0)*scale;
    s->p_wrist_root_Ct=s->p_elbow_root_Ct+Eigen::Vector3d(.25,0,0)*scale;
    s->p_control_root_Ct=s->p_wrist_root_Ct+Eigen::Vector3d(.07,0,0)*scale;
  }
  return f;
}
TEST(SharedRootMorphology, CorrectsBadFirstSampleAndIgnoresDuplicates) {
  SharedRootMorphologyEstimator e;
  auto result=e.update(sample(1,1.3));
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.state,MorphologyState::kProvisional);
  for (int i=0;i<30;++i) result=e.update(sample(1,1.3));
  EXPECT_EQ(result.samples,1U);
  for (std::uint64_t i=2;i<=20;++i) result=e.update(sample(i));
  ASSERT_EQ(result.state,MorphologyState::kConfident);
  EXPECT_NEAR(result.lateral_scale,.423/.4,1e-12);
  EXPECT_NEAR(result.reach_scale,(.2875639059409229+.3145155004129367+.1315)/.59,1e-12);
}
TEST(SharedRootMorphology, EpochResetsAndSustainedOutliersInvalidate) {
  SharedRootMorphologyEstimator e;
  for (std::uint64_t i=1;i<=20;++i) e.update(sample(i));
  MorphologyEstimate out;
  for (std::uint64_t i=21;i<=60;++i) out=e.update(sample(i,1.3));
  EXPECT_FALSE(out.valid);
  auto fresh=sample(61); fresh.tracking_epoch=2;
  out=e.update(fresh);
  EXPECT_TRUE(out.valid);
  EXPECT_EQ(out.state,MorphologyState::kProvisional);
  EXPECT_EQ(out.samples,1U);
}
TEST(SharedRootMorphology, ScalingUsesEffectiveMetricGeometry) {
  SharedRootMorphologyEstimator a,b;
  auto x=a.update(sample(1)); auto y=b.update(sample(1,1.2));
  EXPECT_NEAR(x.reach_scale,y.reach_scale*1.2,1e-12);
  EXPECT_NEAR(x.lateral_scale,y.lateral_scale*1.2,1e-12);
}
} // namespace
} // namespace tianji_qp_ik
