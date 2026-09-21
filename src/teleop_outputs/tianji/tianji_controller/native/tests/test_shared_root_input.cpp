#include "tianji_qp_ik/shared_root_input.hpp"
#include <gtest/gtest.h>
#include <limits>

namespace tianji_qp_ik {
namespace {
PicoTeleopFrame frame() {
  PicoTeleopFrame f;
  f.sequence=1; f.tracking_epoch=1;
  f.source_timestamp_ns=100; f.receive_monotonic_ns=200;
  f.upper_limb_skeleton.valid=true;
  f.upper_limb_skeleton.rotations_valid=true;
  for (int i=0;i<8;++i) {
    f.upper_limb_skeleton.points[static_cast<std::size_t>(i)] =
        Eigen::Vector3d(.1*i, i<4 ? .2 : -.2, 1.121+.01*i);
  }
  // Packet side poses deliberately disagree. They must never be selected.
  f.left.position = f.right.position = Eigen::Vector3d::Constant(99);
  return f;
}
TEST(SharedRootInput, SubtractsOnlyFixedOriginAndAppliesBasisOnce) {
  TjvrSharedRootInputAdapter adapter;
  auto f=frame();
  const auto out=adapter.adapt(f);
  ASSERT_TRUE(out.valid);
  EXPECT_LT((out.left.p_control_root_Ct-Eigen::Vector3d(.3,.2,.03)).norm(),1e-12);
  EXPECT_EQ(out.left.p_control_root_Ct,out.left.p_shape_proxy_root_Ct);
  EXPECT_LT((out.right.p_control_root_Ct-out.left.p_control_root_Ct-
      (f.upper_limb_skeleton.points[7]-f.upper_limb_skeleton.points[3])).norm(),1e-12);
  EXPECT_EQ(out.left.R_palm_Ct.col(0), -Eigen::Vector3d::UnitY());
  EXPECT_EQ(out.right.R_palm_Ct.col(0), Eigen::Vector3d::UnitY());
  EXPECT_EQ(out.left.R_palm_Ct.col(2), Eigen::Vector3d::UnitX());
  EXPECT_EQ(out.sequence,f.sequence);
  EXPECT_EQ(out.receive_monotonic_ns,f.receive_monotonic_ns);
}
TEST(SharedRootInput, NoRuntimeRecenteringOrHiddenScale) {
  TjvrSharedRootInputAdapter adapter;
  auto f=frame();
  const auto a=adapter.adapt(f);
  for (auto& p:f.upper_limb_skeleton.points) p += Eigen::Vector3d(.02,.03,.04);
  const auto b=adapter.adapt(f);
  ASSERT_TRUE(b.valid);
  EXPECT_LT((b.left.p_control_root_Ct-a.left.p_control_root_Ct-
      Eigen::Vector3d(.02,.03,.04)).norm(),1e-12);
}
TEST(SharedRootInput, RejectsInvalidBilateralInputWithoutPoseFallback) {
  TjvrSharedRootInputAdapter adapter;
  auto f=frame();
  f.upper_limb_skeleton.rotations[7].coeffs().setZero();
  EXPECT_FALSE(adapter.adapt(f).valid);
  f=frame(); f.upper_limb_skeleton.points[6].x()=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(adapter.adapt(f).valid);
  f=frame(); f.upper_limb_skeleton.rotations_valid=false;
  EXPECT_FALSE(adapter.adapt(f).valid); // required palm rotation has no alternate source
  f=frame(); f.tracking_epoch=0;
  EXPECT_FALSE(adapter.adapt(f).valid);
}
}  // namespace
}  // namespace tianji_qp_ik
