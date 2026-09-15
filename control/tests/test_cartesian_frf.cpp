#include "tianji_qp_ik/cartesian_frf.hpp"
#include "tianji_qp_ik/so3.hpp"

#include <gtest/gtest.h>
#include <Eigen/Geometry>

#include <cmath>

namespace tianji_qp_ik {
namespace {

TEST(CartesianFrf, ChirpIsDeterministicAndZeroOutsideSweep) {
  FrfExcitationConfig config;
  config.dt = 0.005;
  config.warmup_seconds = 2.0;
  config.chirp_seconds = 20.0;
  config.settle_seconds = 2.0;
  config.start_hz = 0.2;
  config.end_hz = 15.0;
  config.amplitude = 0.005;

  EXPECT_DOUBLE_EQ(logChirpSample(config, 0), 0.0);
  EXPECT_DOUBLE_EQ(logChirpSample(config, 399), 0.0);
  EXPECT_DOUBLE_EQ(logChirpSample(config, 4800), 0.0);
  for (std::int64_t sample = 400; sample < 4400; sample += 137) {
    EXPECT_DOUBLE_EQ(logChirpSample(config, sample),
                     logChirpSample(config, sample));
    EXPECT_LE(std::abs(logChirpSample(config, sample)), config.amplitude);
  }
}

TEST(CartesianFrf, InstantaneousFrequencyReachesConfiguredEndpoints) {
  FrfExcitationConfig config;
  EXPECT_NEAR(logChirpInstantaneousFrequency(config, 0.0), config.start_hz,
              1.0e-12);
  EXPECT_NEAR(logChirpInstantaneousFrequency(config, config.chirp_seconds),
              config.end_hz, 1.0e-10);
}

TEST(CartesianFrf, RotationPerturbationUsesRequestedSo3Axis) {
  Pose center;
  center.rotation =
      Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const double angle = 0.017;
  const Pose target = perturbPose(center, FrfChannel::kRy, angle);
  EXPECT_TRUE(isProperRotation(target.rotation));
  const Eigen::Vector3d delta = so3Log(target.rotation * center.rotation.transpose());
  EXPECT_NEAR(delta.x(), 0.0, 1.0e-10);
  EXPECT_NEAR(delta.y(), angle, 1.0e-10);
  EXPECT_NEAR(delta.z(), 0.0, 1.0e-10);
}

}  // namespace
}  // namespace tianji_qp_ik
