// Offline/local-pipe V131 worker. No sockets, SDK, publication or actuator.
// Fixed little-endian protocol; the caller owns all motion authorization.
#include "tianji_teleop/ik/dexhand_qp_arm_ik.hpp"
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
constexpr std::size_t request_size = 272, response_size = 432;
template<class T> T load(const unsigned char* p) { T x; std::memcpy(&x, p, sizeof x); return x; }
template<class T> void put(unsigned char* p, T x) { std::memcpy(p, &x, sizeof x); }
void require(bool ok, const char* why) { if (!ok) throw std::runtime_error(why); }
using namespace tianji_teleop;
int run(const char* urdf, const char* model) {
  const std::uint32_t endian = 1;
  require(*reinterpret_cast<const unsigned char*>(&endian) == 1 && sizeof(double) == 8,
          "worker requires little-endian IEEE doubles");
  IkSettings settings;
  settings.control_period_s = .005;
  settings.maximum_joint_step_rad = .02; // simulation profile only
  DexhandQpArmIk solver(urdf, settings, model);
  std::cout << "{\"schema_version\":1,\"kind\":\"pico2_ik_ready\",\"simulation_only\":true}\n" << std::flush;
  std::uint64_t sequence = 0, epoch = 0;
  double last_now = -1, last_source = -1;
  for (;;) {
    std::array<unsigned char, request_size> request{};
    std::cin.read(reinterpret_cast<char*>(request.data()), request.size());
    if (std::cin.gcount() == 0 && std::cin.eof()) return 0;
    require(std::cin.gcount() == static_cast<std::streamsize>(request.size()), "truncated IK request");
    require(std::memcmp(request.data(), "P2IQ", 4) == 0 && request[4] == 1 &&
            load<std::uint16_t>(&request[6]) == request_size, "IK request header mismatch");
    const auto operation = request[5]; // 1 reset, 2 solve, 3 read-only FK
    const auto next_sequence = load<std::uint64_t>(&request[8]);
    const auto next_epoch = load<std::uint64_t>(&request[16]);
    require(operation >= 1 && operation <= 3 && next_sequence > sequence, "IK operation/sequence invalid");
    require(next_sequence <= static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()), "IK sequence out of range");
    require(next_epoch <= static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()), "IK epoch out of range");
    const double source = load<double>(&request[24]), received = load<double>(&request[32]), now = load<double>(&request[40]);
    require(std::isfinite(source) && std::isfinite(received) && std::isfinite(now) &&
            source >= 0 && received >= 0 && now >= received, "IK timestamps invalid");
    if (operation == 1) require(next_epoch == epoch + 1, "IK reset requires next epoch");
    else require(next_epoch == epoch && (operation == 3 || epoch > 0), "IK epoch mismatch/unprepared");
    if (operation == 2) require(now > last_now && source >= last_source, "IK clock rollback");
    std::array<ArmJointVector, 2> seeds;
    std::array<Eigen::Isometry3d, 2> targets;
    // Validate both sides before either solver can advance.
    for (int side = 0; side < 2; ++side) {
      for (int j = 0; j < 7; ++j) seeds[side][j] = load<double>(&request[48 + (side * 7 + j) * 8]);
      require(seeds[side].allFinite(), "nonfinite IK seed");
      targets[side] = Eigen::Isometry3d::Identity();
      std::array<double, 7> pose{};
      for (int j = 0; j < 7; ++j) {
        pose[j] = load<double>(&request[160 + (side * 7 + j) * 8]);
        require(std::isfinite(pose[j]), "nonfinite IK pose");
      }
      if (operation == 2) {
        Eigen::Quaterniond q(pose[6], pose[3], pose[4], pose[5]);
        require(std::isfinite(q.norm()) && q.norm() > 1e-12, "invalid IK quaternion norm");
        targets[side].translation() = Eigen::Vector3d(pose[0], pose[1], pose[2]);
        targets[side].linear() = q.normalized().toRotationMatrix();
      }
    }
    if (operation == 1) {
      solver.reset(ArmSide::kLeft); solver.reset(ArmSide::kRight);
      epoch = next_epoch; last_now = last_source = -1;
    }
    std::array<unsigned char, response_size> response{};
    std::memcpy(response.data(), "P2IR", 4); response[4] = 1; response[5] = operation;
    put<std::uint16_t>(&response[6], response_size);
    put(&response[8], next_sequence); put(&response[16], epoch);
    for (int side = 0; side < 2; ++side) {
      const auto arm = side == 0 ? ArmSide::kLeft : ArmSide::kRight;
      IkResult result;
      if (operation == 2) result = solver.solve_timed(arm, targets[side], seeds[side],
                                                    Eigen::Vector3d::Zero(), source, received, now);
      else { result.joints_rad = seeds[side]; result.achieved_pose = solver.forward(arm, seeds[side]); }
      const auto offset = 24 + side * 204;
      const std::uint32_t flags = (result.accepted ? 1 : 0) | (result.converged ? 2 : 0) | (result.model_state_only ? 4 : 0);
      put(&response[offset], flags);
      require(result.joints_rad.allFinite() && result.achieved_pose.matrix().allFinite(), "nonfinite IK output");
      for (int j = 0; j < 7; ++j) put(&response[offset + 4 + j * 8], result.joints_rad[j]);
      const Eigen::Quaterniond q(result.achieved_pose.rotation());
      for (int j = 0; j < 3; ++j) put(&response[offset + 60 + j * 8], result.achieved_pose.translation()[j]);
      for (int j = 0; j < 4; ++j) put(&response[offset + 84 + j * 8], q.coeffs()[j]);
      put(&response[offset + 116], result.position_error_m);
      put(&response[offset + 124], result.orientation_error_rad);
      put(&response[offset + 132], std::isfinite(result.solve_time_ms) ? result.solve_time_ms : 0.0);
      require(result.status.size() < 64, "IK diagnostic too long");
      std::memcpy(&response[offset + 140], result.status.data(), result.status.size());
    }
    if (operation == 2) { last_now = now; last_source = source; }
    sequence = next_sequence;
    std::cout.write(reinterpret_cast<const char*>(response.data()), response.size());
    std::cout.flush();
    require(bool(std::cout), "IK response pipe failed");
  }
}
}
int main(int argc, char** argv) {
  try { if (argc != 3) return 2; return run(argv[1], argv[2]); }
  catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
