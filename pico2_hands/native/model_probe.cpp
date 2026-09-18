// Offline mesh-free model/Jacobian and model-state integration smoke test.
// This does not replace the pending full host/Pinocchio reference comparison.
#include "tianji_v131/mujoco_robot.hpp"
#include "tianji_v131/velocity_qp.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

int main(int argc, char** argv) {
  using namespace tianji_v131;
  if (argc != 2) return 2;
  MujocoRobot robot(argv[1]);
  double max_error = 0.0;
  for (const auto side : {ArmSide::kLeft, ArmSide::kRight}) {
    Vec7 q;
    q << 55, -65, -70, -60, 60, 0, 0;
    if (side == ArmSide::kRight) { q[0] *= -1; q[2] *= -1; q[4] *= -1; }
    q *= std::acos(-1.0) / 180.0;
    const auto home = robot.armKinematicsAt(side, q);
    for (int j = 0; j < 7; ++j) {
      auto plus = q, minus = q;
      plus[j] += 1e-6; minus[j] -= 1e-6;
      const Eigen::Vector3d numeric =
        (robot.armKinematicsAt(side, plus).tcp_pose.position -
         robot.armKinematicsAt(side, minus).tcp_pose.position) / 2e-6;
      max_error = std::max(max_error,
        (numeric - home.tcp_jacobian.block<3, 1>(0, j)).norm());
    }
    VelocityQp solver(robot.mapping(side).limits);
    auto target = home.tcp_pose;
    target.position.x() += .01;
    int accepted = 0;
    Vec7 last = q;
    for (int i = 0; i < 200; ++i) {
      const double timestamp = 1.0 + i * .005;
      const auto result = solver.solve(side, target, q,
        [&](const Vec7& x) { return robot.armKinematicsAt(side, x); },
        .005, timestamp, timestamp, timestamp);
      if (!result.q.allFinite()) throw std::runtime_error("nonfinite model state");
      if (result.accepted) { ++accepted; last = result.q; }
    }
    if (accepted < 100 || (last - q).norm() < 1e-4)
      throw std::runtime_error("model state did not advance independently of fixed seed");
  }
  if (max_error > 1e-7) throw std::runtime_error("world TCP Jacobian mismatch");
  std::cout << "model_smoke_passed max_translation_jacobian_error=" << max_error << '\n';
}
