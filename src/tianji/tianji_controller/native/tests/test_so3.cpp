#include "tianji_qp_ik/so3.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>

namespace tianji_qp_ik {
namespace {

TEST(So3Log, IdentityIsZero) {
  EXPECT_TRUE(so3Log(Eigen::Matrix3d::Identity()).isZero(1e-14));
}

TEST(So3Log, SmallAngleUsesStableLimit) {
  const Eigen::Matrix3d rotation =
      Eigen::AngleAxisd(1e-9, Eigen::Vector3d::UnitX()).toRotationMatrix();
  const Eigen::Vector3d result = so3Log(rotation);
  EXPECT_NEAR(result.x(), 1e-9, 1e-12);
  EXPECT_NEAR(result.y(), 0.0, 1e-14);
  EXPECT_NEAR(result.z(), 0.0, 1e-14);
}

TEST(So3Log, GenericRotationReturnsAxisAngle) {
  const Eigen::Vector3d axis = Eigen::Vector3d(1.0, -2.0, 0.5).normalized();
  const double angle = 1.2;
  const Eigen::Vector3d result =
      so3Log(Eigen::AngleAxisd(angle, axis).toRotationMatrix());
  EXPECT_LT((result - angle * axis).norm(), 1e-12);
}

TEST(So3Log, NearPiIsStable) {
  const Eigen::Vector3d axis = Eigen::Vector3d(0.3, -0.4, 0.5).normalized();
  const double angle = 3.14159265358979323846 - 1e-7;
  const Eigen::Vector3d result =
      so3Log(Eigen::AngleAxisd(angle, axis).toRotationMatrix());
  EXPECT_NEAR(result.norm(), angle, 1e-7);
  EXPECT_GT(result.normalized().dot(axis), 1.0 - 1e-7);
}

TEST(So3Log, QuaternionSignEquivalentInputsMatchAndInvalidMatrixIsRejected) {
  const Eigen::Quaterniond first(
      Eigen::AngleAxisd(0.7, Eigen::Vector3d(1.0, 2.0, -1.0).normalized()));
  const Eigen::Quaterniond second(-first.w(), -first.x(), -first.y(), -first.z());
  EXPECT_TRUE(so3Log(first.toRotationMatrix()).isApprox(so3Log(second.toRotationMatrix()),
                                                       1e-14));
  Eigen::Matrix3d invalid = Eigen::Matrix3d::Identity();
  invalid(0, 0) = 2.0;
  EXPECT_FALSE(isProperRotation(invalid));
  EXPECT_THROW(so3Log(invalid), std::invalid_argument);
}

TEST(PoseError, UsesWorldFrameRotationDifference) {
  Pose current{Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Matrix3d::Identity()};
  Pose desired{Eigen::Vector3d(1.1, 1.8, 3.3),
               Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitZ()).toRotationMatrix()};
  const Vec6 error = poseErrorWorld(desired, current);
  EXPECT_TRUE(error.head<3>().isApprox(Eigen::Vector3d(0.1, -0.2, 0.3), 1e-14));
  EXPECT_TRUE(error.tail<3>().isApprox(Eigen::Vector3d(0.0, 0.0, 0.25), 1e-12));
}

}  // namespace
}  // namespace tianji_qp_ik
