// Computation only: no sockets, device SDKs, actuator enable, or background clock.
#include "tianji_qp_ik/cartesian_otg.hpp"
#include "tianji_qp_ik/controller.hpp"

#include <Eigen/Geometry>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {
using namespace tianji_qp_ik;
constexpr double kDt = 0.005;
constexpr std::size_t kMaxLine = 2048;

bool line(std::string& text) {
  text.clear();
  char ch;
  while (std::cin.get(ch)) {
    if (ch == '\n') return true;
    if (text.size() == kMaxLine) throw std::runtime_error("oversized request");
    text.push_back(ch);
  }
  if (!text.empty()) throw std::runtime_error("truncated request");
  return false;
}

double number(std::istringstream& in) {
  double value;
  if (!(in >> value) || !std::isfinite(value)) throw std::runtime_error("invalid finite number");
  return value;
}

void end(std::istringstream& in) {
  in >> std::ws;
  if (!in.eof()) throw std::runtime_error("unexpected request data");
}

Pose pose(std::istringstream& in) {
  Pose p;
  for (int i = 0; i < 3; ++i) p.position[i] = number(in);
  const double x = number(in), y = number(in), z = number(in), w = number(in);
  Eigen::Quaterniond q(w, x, y, z);
  if (std::abs(q.norm() - 1.0) > 1e-3) throw std::runtime_error("non-unit quaternion");
  p.rotation = q.normalized().toRotationMatrix();
  return p;
}

void checkQ(const Vec7& q, const ArmLimits& limits) {
  if (!q.allFinite() || (q.array() < limits.lower_position.array()).any() ||
      (q.array() > limits.upper_position.array()).any()) {
    throw std::runtime_error("joint position outside model limits");
  }
}

void readState(std::istringstream& in, MujocoRobot& robot) {
  Vec7 left, right;
  for (int i = 0; i < 7; ++i) left[i] = number(in);
  for (int i = 0; i < 7; ++i) right[i] = number(in);
  end(in);
  checkQ(left, robot.mapping(ArmSide::kLeft).limits);
  checkQ(right, robot.mapping(ArmSide::kRight).limits);
  robot.setArmState(ArmSide::kLeft, left, Vec7::Zero());
  robot.setArmState(ArmSide::kRight, right, Vec7::Zero());
  robot.forward();
}

template <typename Derived>
void array(std::ostream& out, const Eigen::MatrixBase<Derived>& values) {
  out << '[';
  for (Eigen::Index i = 0; i < values.size(); ++i) {
    if (i) out << ',';
    if (!std::isfinite(values[i])) throw std::runtime_error("non-finite response");
    out << values[i];
  }
  out << ']';
}

void writeString(std::ostream& out, const std::string& value) {
  constexpr char hex[] = "0123456789abcdef";
  out << '"';
  for (const unsigned char ch : value) {
    if (ch == '"' || ch == '\\') {
      out << '\\' << ch;
    } else if (ch < 0x20) {
      out << "\\u00" << hex[ch >> 4] << hex[ch & 0xf];
    } else {
      out << ch;
    }
  }
  out << '"';
}

void writePose(std::ostream& out, const Pose& p) {
  Eigen::Matrix<double, 7, 1> v;
  v.head<3>() = p.position;
  v.tail<4>() = Eigen::Quaterniond(p.rotation).normalized().coeffs();
  array(out, v);
}

Pose sitePose(const MujocoRobot& robot, const char* name) {
  const int site = mj_name2id(robot.model(), mjOBJ_SITE, name);
  if (site < 0) throw std::runtime_error("missing flange TCP site");
  using RowMatrix = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;
  Pose result;
  result.position = Eigen::Map<const Eigen::Vector3d>(robot.data()->site_xpos + 3 * site);
  result.rotation = Eigen::Map<const RowMatrix>(robot.data()->site_xmat + 9 * site);
  return result;
}

Pose meshPose(const MujocoRobot& robot, const char* name) {
  const auto* m = robot.model();
  const auto* d = robot.data();
  const int mesh = mj_name2id(m, mjOBJ_MESH, name);
  if (mesh < 0) throw std::runtime_error("missing frame mesh");
  int geom = -1;
  for (int i = 0; i < m->ngeom; ++i) {
    if (m->geom_type[i] == mjGEOM_MESH && m->geom_dataid[i] == mesh) {
      if (geom >= 0) throw std::runtime_error("ambiguous frame mesh");
      geom = i;
    }
  }
  if (geom < 0) throw std::runtime_error("missing frame mesh geom");
  using RowMatrix = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;
  const Eigen::Matrix3d geom_rotation = Eigen::Map<const RowMatrix>(d->geom_xmat + 9 * geom);
  const auto* q = m->mesh_quat + 4 * mesh;
  Pose result;
  // The compiler recenters/reorients mesh vertices and folds that transform
  // into the geom. Undo it to recover the original URDF wrist link frame.
  result.rotation = geom_rotation * Eigen::Quaterniond(q[0], q[1], q[2], q[3]).normalized().toRotationMatrix().transpose();
  result.position = Eigen::Map<const Eigen::Vector3d>(d->geom_xpos + 3 * geom) -
      result.rotation * Eigen::Map<const Eigen::Vector3d>(m->mesh_pos + 3 * mesh);
  return result;
}

void state(std::ostream& out, const MujocoRobot& robot) {
  Eigen::Matrix<double, 14, 1> q;
  q.head<7>() = robot.armPosition(ArmSide::kLeft);
  q.tail<7>() = robot.armPosition(ArmSide::kRight);
  out << ",\"q\":";
  array(out, q);
  out << ",\"tcp\":{\"left\":";
  writePose(out, robot.tcpPose(ArmSide::kLeft));
  out << ",\"right\":";
  writePose(out, robot.tcpPose(ArmSide::kRight));
  out << "},\"wrist\":{\"left\":";
  writePose(out, meshPose(robot, "wuji2_l_wrist"));
  out << ",\"right\":";
  writePose(out, meshPose(robot, "wuji2_r_wrist"));
  out << '}';
}

void bounds(std::ostream& out, const MujocoRobot& robot) {
  Eigen::Matrix<double, 54, 1> lower, upper;
  lower.head<7>() = robot.mapping(ArmSide::kLeft).limits.lower_position;
  lower.segment<7>(7) = robot.mapping(ArmSide::kRight).limits.lower_position;
  upper.head<7>() = robot.mapping(ArmSide::kLeft).limits.upper_position;
  upper.segment<7>(7) = robot.mapping(ArmSide::kRight).limits.upper_position;
  lower.segment<20>(14) = robot.handMapping(ArmSide::kLeft).lower_position;
  lower.tail<20>() = robot.handMapping(ArmSide::kRight).lower_position;
  upper.segment<20>(14) = robot.handMapping(ArmSide::kLeft).upper_position;
  upper.tail<20>() = robot.handMapping(ArmSide::kRight).upper_position;
  out << ",\"lower\":";
  array(out, lower);
  out << ",\"upper\":";
  array(out, upper);
}
}  // namespace

int main(int argc, char** argv) {
  // Keep even third-party solver stdout off the framed response channel.
  const int output_fd = dup(STDOUT_FILENO);
  if (output_fd < 0 || dup2(STDERR_FILENO, STDOUT_FILENO) < 0) return 2;
  FILE* output = fdopen(output_fd, "w");
  if (output == nullptr) { close(output_fd); return 2; }
  unsigned long long sequence = 0;
  std::string command = "startup";
  try {
    if (argc != 3) throw std::runtime_error("usage: mocap_tcp_worker CONFIG MODEL");
    auto config = loadConfig(argv[1]);
    if (config.control_level != ControlLevel::kVelocity ||
        config.ik_algorithm != IkAlgorithm::kHierarchicalQp ||
        std::abs(config.controller.rate_hz - 200.0) > 1e-9) {
      throw std::runtime_error("requires standalone hierarchical_qp velocity config at 200 Hz");
    }
    config.controller.model_state_only = true;
    MujocoRobot robot(argv[2]);
    if (!robot.hasHandMappings()) throw std::runtime_error("requires bilateral Wuji Hand2 model");
    std::string text;
    if (!line(text)) throw std::runtime_error("missing INIT request");
    std::istringstream initial(text);
    initial >> command;
    if (command != "INIT") throw std::runtime_error("expected INIT q14");
    readState(initial, robot);
    DualArmController controller(robot, config);
    controller.setArmAngleReferenceMode(ArmAngleReferenceMode::kDefaultDown);
    if (!controller.synchronizeReferencesToActual()) throw std::runtime_error("initial state rejected");
    CartesianReferenceGenerator left_otg(config.cartesian_otg, kDt);
    CartesianReferenceGenerator right_otg(config.cartesian_otg, kDt);
    auto resetOtg = [&]() {
      left_otg.reset(robot.tcpPose(ArmSide::kLeft));
      right_otg.reset(robot.tcpPose(ArmSide::kRight));
    };
    resetOtg();
    auto holdArm = [&](ArmSide side, const Vec7& q) {
      if (!controller.setReferenceState(side, {q, Vec7::Zero(), Vec7::Zero()})) {
        throw std::runtime_error("held reference state rejected");
      }
    };
    std::ostringstream ready;
    ready << std::setprecision(17) << "{\"kind\":\"ready\",\"sequence\":0";
    state(ready, robot);
    bounds(ready, robot);
    ready << ",\"base\":{\"left\":";
    writePose(ready, meshPose(robot, "Base_L"));
    ready << ",\"right\":";
    writePose(ready, meshPose(robot, "Base_R"));
    ready << '}';
    ready << ",\"flange\":{\"left\":";
    writePose(ready, sitePose(robot, "flange_tcp_L"));
    ready << ",\"right\":";
    writePose(ready, sitePose(robot, "flange_tcp_R"));
    ready << '}';
    ready << "}\n";
    if (fputs(ready.str().c_str(), output) < 0 || fflush(output) != 0) throw std::runtime_error("response pipe closed");
    while (line(text)) {
      std::istringstream in(text);
      std::string token;
      in >> command >> token;
      if (token.empty() || token.find_first_not_of("0123456789") != std::string::npos ||
          sequence == std::numeric_limits<unsigned long long>::max() ||
          std::stoull(token) != sequence + 1) throw std::runtime_error("invalid request sequence");
      ++sequence;
      std::ostringstream response;
      response << std::setprecision(17) << "{\"sequence\":" << sequence;
      if (command == "SYNC") {
        readState(in, robot);
        // model_state_only synchronizeReferencesToActual resets history but
        // deliberately does not copy q; explicitly seed the public reference API.
        for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
          if (!controller.setReferenceState(side, {robot.armPosition(side), Vec7::Zero(), Vec7::Zero()})) {
            throw std::runtime_error("reference state rejected");
          }
        }
        controller.resetSolvers();
        if (!controller.synchronizeReferencesToActual()) throw std::runtime_error("state synchronization rejected");
        resetOtg();
        response << ",\"kind\":\"sync\"";
      } else if (command == "STEP") {
        int mask = 0;
        if (!(in >> mask) || mask < 0 || mask > 3) throw std::runtime_error("invalid target mask");
        DualArmTargets targets;
        targets.left = (mask & 1) ? pose(in) : robot.tcpPose(ArmSide::kLeft);
        targets.right = (mask & 2) ? pose(in) : robot.tcpPose(ArmSide::kRight);
        targets.left_stale = !(mask & 1);
        targets.right_stale = !(mask & 2);
        end(in);
        // Staleness alone does not freeze the dual-arm controller or Cartesian
        // OTG. Seed only omitted sides at their held q/FK with zero derivatives.
        Vec7 left_held_q, right_held_q;
        if (targets.left_stale) {
          left_held_q = controller.reference(ArmSide::kLeft);
          holdArm(ArmSide::kLeft, left_held_q);
          left_otg.reset(targets.left);
        }
        if (targets.right_stale) {
          right_held_q = controller.reference(ArmSide::kRight);
          holdArm(ArmSide::kRight, right_held_q);
          right_otg.reset(targets.right);
        }
        DualArmReferences references;
        references.left = targets.left_stale ? left_otg.state()
            : left_otg.update(targets.left, Vec6::Zero(), false, kDt);
        references.right = targets.right_stale ? right_otg.state()
            : right_otg.update(targets.right, Vec6::Zero(), false, kDt);
        references.left.stale = targets.left_stale;
        references.right.stale = targets.right_stale;
        ControllerDiagnostics result;
        if (mask != 0) {
          result = config.cartesian_otg.enabled ? controller.step(references, kDt) : controller.step(targets, kDt);
        }
        // step() advances both sides, including fallback/history on stale ones.
        // Restore omitted sides through the per-side API, which also resets the
        // solver history, without disturbing the present side's ongoing motion.
        if (targets.left_stale) {
          holdArm(ArmSide::kLeft, left_held_q);
          result.left = {};
          result.left.q_ref = left_held_q;
          result.left.hold_reason = HoldReason::kNone;
        }
        if (targets.right_stale) {
          holdArm(ArmSide::kRight, right_held_q);
          result.right = {};
          result.right.q_ref = right_held_q;
          result.right.hold_reason = HoldReason::kNone;
        }
        checkQ(result.left.q_ref, robot.mapping(ArmSide::kLeft).limits);
        checkQ(result.right.q_ref, robot.mapping(ArmSide::kRight).limits);
        // A requested arm must be accepted; the Python owner latches any rejection.
        response << ",\"kind\":\"step\",\"accepted\":[" << (result.left.accepted ? "true" : "false")
                 << ',' << (result.right.accepted ? "true" : "false") << "],\"hold_reason\":["
                 << static_cast<int>(result.left.hold_reason) << ',' << static_cast<int>(result.right.hold_reason)
                 << "],\"solver_status\":[" << static_cast<int>(result.left.ik.status) << ','
                 << static_cast<int>(result.right.ik.status) << ']';
        robot.forward();
      } else {
        throw std::runtime_error("unknown request");
      }
      state(response, robot);
      response << "}\n";
      if (fputs(response.str().c_str(), output) < 0 || fflush(output) != 0) throw std::runtime_error("response pipe closed");
    }
    fclose(output);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "mocap_tcp_worker: " << error.what() << '\n';
    // A caught native failure must reach the owner, not only its terminal.
    // This is a terminal response: never continue with a partial command state.
    std::ostringstream response;
    response << "{\"kind\":\"error\",\"sequence\":" << sequence << ",\"command\":";
    writeString(response, command);
    response << ",\"error\":";
    writeString(response, error.what());
    response << "}\n";
    fputs(response.str().c_str(), output);
    fflush(output);
    fclose(output);
    return 2;
  }
}
