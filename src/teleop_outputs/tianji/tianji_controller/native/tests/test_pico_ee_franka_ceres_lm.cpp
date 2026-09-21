#include "tianji_qp_ik/pico_ee_franka_ceres_lm.hpp"
#include "tianji_qp_ik/so3.hpp"
#include "tianji_qp_ik/pinocchio_arm_kinematics.hpp"
#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <limits>
namespace tianji_qp_ik { namespace {
PicoEeFrankaDlsInput input() {
  PicoEeFrankaDlsInput i;
  i.target_valid = true; i.target_stale = false; i.dt = .005;
  i.seed.setZero(); i.seed_velocity.setZero(); i.seed_acceleration.setZero();
  i.home_reference.setConstant(.5);
  i.limits.lower_position.setConstant(-1); i.limits.upper_position.setConstant(1);
  i.limits.velocity.setConstant(4);
  i.evaluate = [](const Vec7& q) {
    ArmKinematicSample s; s.tcp_pose.position = q.head<3>();
    s.tcp_pose.rotation.setIdentity(); s.tcp_jacobian.setZero();
    s.tcp_jacobian.topLeftCorner<3,3>().setIdentity(); return s;
  };
  i.target.rotation.setIdentity(); i.target.position << .1, -.05, .08;
  return i;
}
PicoEeFrankaCeresLmConfig config() {
  PicoEeFrankaCeresLmConfig c; c.max_solver_time_seconds=1.; return c;
}
TEST(CeresLm, AnalyticRotationJacobianMatchesFiniteDifferences) {
  Pose current,target; current.position << .2,-.1,.3; target.position << -.2,.4,.1;
  current.rotation=Eigen::AngleAxisd(.7,Eigen::Vector3d(1,2,3).normalized()).toRotationMatrix();
  target.rotation=Eigen::AngleAxisd(2.5,Eigen::Vector3d(2,-1,1).normalized()).toRotationMatrix();
  Mat67 g; g.setRandom();
  const Mat67 analytic=PicoEeFrankaCeresLmIk7::residualJacobian(poseErrorWorld(target,current),g);
  for(int j=0;j<7;++j) {
    Pose plus=current,minus=current; const double h=1e-7;
    plus.position+=h*g.col(j).head<3>(); minus.position-=h*g.col(j).head<3>();
    const Eigen::Vector3d w=g.col(j).tail<3>();
    plus.rotation=Eigen::AngleAxisd(h*w.norm(),w.normalized()).toRotationMatrix()*current.rotation;
    minus.rotation=Eigen::AngleAxisd(-h*w.norm(),w.normalized()).toRotationMatrix()*current.rotation;
    EXPECT_LT(((poseErrorWorld(target,plus)-poseErrorWorld(target,minus))/(2*h)-analytic.col(j)).norm(),1e-7);
  }
}
TEST(CeresLm, ReachableAndStationaryNoDrift) {
  PicoEeFrankaCeresLmIk7 solver(config(),.05); auto i=input(); auto r=solver.solve(i);
  ASSERT_TRUE(r.accepted); EXPECT_EQ(r.dls.status,PoseDlsStatus::kConverged);
  EXPECT_LT((r.goal.head<3>()-i.target.position).norm(),.0005);
  i.seed=r.goal; auto stationary=solver.solve(i);
  ASSERT_TRUE(stationary.accepted); EXPECT_EQ((stationary.goal-i.seed).norm(),0.);
}
TEST(CeresLm, IterationLimitReturnsBestEvaluatedCandidateAndNotPreviousGoal) {
  auto c=config(); c.max_iterations=1; c.nullspace_enabled=false;
  PicoEeFrankaCeresLmIk7 solver(c,.05); auto i=input(); ASSERT_TRUE(solver.solve(i).accepted);
  i.seed.setConstant(-.4); i.target.position.setConstant(5.);
  double best=std::numeric_limits<double>::infinity(); Vec7 best_q;
  auto evaluate=i.evaluate;
  i.evaluate=[&](const Vec7& q) { auto s=evaluate(q); double e=poseErrorWorld(i.target,s.tcp_pose).squaredNorm();
    if(e<best) {best=e;best_q=q;} return s; };
  auto r=solver.solve(i); ASSERT_TRUE(r.accepted); EXPECT_FALSE(r.target_held);
  EXPECT_NE(r.dls.status,PoseDlsStatus::kRejected); EXPECT_NE(r.dls.status,PoseDlsStatus::kConverged);
  EXPECT_LT((r.goal-best_q).norm(),1e-12);
  EXPECT_LE((r.goal-i.seed).cwiseAbs().maxCoeff(),.2+1e-12);
  EXPECT_LE(r.goal.maxCoeff(),.95); EXPECT_GE(r.goal.minCoeff(),-.95);
}
TEST(CeresLm, FlatUnreachableTargetReturnsCurrentSeedSuccessfully) {
  auto i=input(); auto evaluate=i.evaluate;
  i.evaluate=[evaluate](const Vec7&) {auto s=evaluate(Vec7::Zero());s.tcp_jacobian.setZero();return s;};
  i.seed.setConstant(.3); auto c=config(); c.nullspace_enabled=false;
  PicoEeFrankaCeresLmIk7 solver(c,.05);
  auto r=solver.solve(i); EXPECT_TRUE(r.accepted); EXPECT_FALSE(r.target_held);
  EXPECT_EQ(r.dls.status,PoseDlsStatus::kNotConverged); EXPECT_EQ((r.goal-i.seed).norm(),0.);
}
TEST(CeresLm, ExpiredBudgetStillReturnsCurrentCandidate) {
  auto c=config(); c.max_solver_time_seconds=1e-12; auto i=input();
  auto r=PicoEeFrankaCeresLmIk7(c,.05).solve(i);
  EXPECT_TRUE(r.accepted); EXPECT_FALSE(r.target_held); EXPECT_EQ((r.goal-i.seed).norm(),0.);
}
TEST(CeresLm, NullspaceMovesRedundantJointsTowardHome) {
  auto c=config(); auto i=input(); auto r=PicoEeFrankaCeresLmIk7(c,.05).solve(i);
  EXPECT_TRUE(r.accepted); EXPECT_GT(r.goal.tail<4>().norm(),0.);
  EXPECT_LT((r.goal-i.home_reference).norm(),(i.seed-i.home_reference).norm());
  EXPECT_LT((r.goal.head<3>()-i.target.position).norm(),.0005);
}
TEST(CeresLm, InvalidInputIsNotMisreportedAsAValidCandidate) {
  auto i=input(); i.target.position[0]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(PicoEeFrankaCeresLmIk7(config(),.05).solve(i).accepted);
}


} }
