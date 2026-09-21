#include "tianji_qp_ik/cartesian_frf.hpp"
#include "tianji_qp_ik/config.hpp"
#include "tianji_qp_ik/controller.hpp"
#include "tianji_qp_ik/iterative_pose_dls.hpp"
#include "tianji_qp_ik/mujoco_robot.hpp"
#include "tianji_qp_ik/pico_teleop_protocol.hpp"
#include "tianji_qp_ik/so3.hpp"
#include "tianji_qp_ik/spark_guidance.hpp"

#include <Eigen/Core>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace tianji_qp_ik {
namespace {

struct Options {
  std::string config_path;
  std::string model_path;
  std::string urdf_path;
  std::string algorithm;
  ArmSide arm{ArmSide::kLeft};
  std::string working_point{"center"};
  FrfChannel channel{FrfChannel::kX};
  std::string output_path;
  FrfExcitationConfig excitation;
  bool amplitude_explicit{false};
};

std::string requireValue(int argc, char** argv, int& index) {
  if (++index >= argc) throw std::invalid_argument("missing option value");
  return argv[index];
}

Options parseOptions(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (key == "--config") options.config_path = requireValue(argc, argv, i);
    else if (key == "--model") options.model_path = requireValue(argc, argv, i);
    else if (key == "--urdf") options.urdf_path = requireValue(argc, argv, i);
    else if (key == "--algorithm") options.algorithm = requireValue(argc, argv, i);
    else if (key == "--arm") {
      const std::string value = requireValue(argc, argv, i);
      if (value == "left") options.arm = ArmSide::kLeft;
      else if (value == "right") options.arm = ArmSide::kRight;
      else throw std::invalid_argument("arm must be left or right");
    } else if (key == "--working-point") {
      options.working_point = requireValue(argc, argv, i);
    } else if (key == "--channel") {
      options.channel = parseFrfChannel(requireValue(argc, argv, i));
    } else if (key == "--output") options.output_path = requireValue(argc, argv, i);
    else if (key == "--warmup") options.excitation.warmup_seconds = std::stod(requireValue(argc, argv, i));
    else if (key == "--chirp") options.excitation.chirp_seconds = std::stod(requireValue(argc, argv, i));
    else if (key == "--settle") options.excitation.settle_seconds = std::stod(requireValue(argc, argv, i));
    else if (key == "--start-hz") options.excitation.start_hz = std::stod(requireValue(argc, argv, i));
    else if (key == "--end-hz") options.excitation.end_hz = std::stod(requireValue(argc, argv, i));
    else if (key == "--amplitude") {
      options.excitation.amplitude = std::stod(requireValue(argc, argv, i));
      options.amplitude_explicit = true;
    }
    else throw std::invalid_argument("unknown option: " + key);
  }
  if (options.config_path.empty() || options.model_path.empty() ||
      options.urdf_path.empty() || options.algorithm.empty() ||
      options.output_path.empty()) {
    throw std::invalid_argument("config, model, urdf, algorithm and output are required");
  }
  const bool rotational = options.channel == FrfChannel::kRx ||
                          options.channel == FrfChannel::kRy ||
                          options.channel == FrfChannel::kRz;
  if (rotational && !options.amplitude_explicit) {
    options.excitation.amplitude = M_PI / 180.0;
  }
  return options;
}

IkAlgorithm algorithmFromString(const std::string& value) {
  if (value == "hierarchical_qp") return IkAlgorithm::kHierarchicalQp;
  if (value == "spark_upper_qpoases_velocity_qp")
    return IkAlgorithm::kSparkUpperQpoasesVelocityQp;
  if (value == "spark_upper_qpoases_cartesian_otg_velocity_qp")
    return IkAlgorithm::kSparkUpperQpoasesCartesianOtgVelocityQp;
  if (value == "spark_upper_qpoases_feedforward_velocity_qp")
    return IkAlgorithm::kSparkUpperQpoasesFeedforwardVelocityQp;
  if (value == "spark_upper_qpoases_headroom_feedforward_velocity_qp")
    return IkAlgorithm::kSparkUpperQpoasesHeadroomFeedforwardVelocityQp;
  throw std::invalid_argument("unsupported FRF algorithm: " + value);
}

Eigen::Vector3d workingPointOffset(const std::string& name) {
  if (name == "center") return Eigen::Vector3d::Zero();
  if (name == "x_negative") return {-0.05, 0.0, 0.0};
  if (name == "x_positive") return {0.05, 0.0, 0.0};
  if (name == "y_negative") return {0.0, -0.05, 0.0};
  if (name == "y_positive") return {0.0, 0.05, 0.0};
  if (name == "z_negative") return {0.0, 0.0, -0.05};
  if (name == "z_positive") return {0.0, 0.0, 0.05};
  throw std::invalid_argument("unknown working point: " + name);
}

ArmMotionState motionState(MujocoRobot& robot, ArmSide side) {
  ArmMotionState state;
  state.q = robot.armPosition(side);
  state.qdot = robot.armVelocity(side);
  return state;
}

PicoTeleopFrame baseFrame(MujocoRobot& robot) {
  const auto left = robot.armKinematicsAt(ArmSide::kLeft, robot.armPosition(ArmSide::kLeft));
  const auto right = robot.armKinematicsAt(ArmSide::kRight, robot.armPosition(ArmSide::kRight));
  PicoTeleopFrame frame;
  frame.tracking_epoch = 1;
  frame.upper_limb_skeleton.valid = true;
  frame.upper_limb_skeleton.points = {
      left.shoulder_position, left.elbow_position, left.wrist_position, left.tcp_pose.position,
      right.shoulder_position, right.elbow_position, right.wrist_position, right.tcp_pose.position};
  frame.left = left.tcp_pose;
  frame.right = right.tcp_pose;
  return frame;
}

SparkPostureGuideMode sparkMode(IkAlgorithm algorithm) {
  if (usesSparkOtgConsistentVelocityQp(algorithm))
    return SparkPostureGuideMode::kOtgConsistentJointReferenceVelocity;
  if (usesSparkHeadroomFeedforwardVelocityQp(algorithm))
    return SparkPostureGuideMode::kHeadroomFeedforwardJointReferenceVelocity;
  if (usesSparkFeedforwardVelocityQp(algorithm))
    return SparkPostureGuideMode::kFeedforwardJointReferenceVelocity;
  return SparkPostureGuideMode::kJointReferenceVelocity;
}

double poseComponent(const Pose& pose, const Pose& center, FrfChannel channel) {
  switch (channel) {
    case FrfChannel::kX: return pose.position.x() - center.position.x();
    case FrfChannel::kY: return pose.position.y() - center.position.y();
    case FrfChannel::kZ: return pose.position.z() - center.position.z();
    case FrfChannel::kRx: return so3Log(pose.rotation * center.rotation.transpose()).x();
    case FrfChannel::kRy: return so3Log(pose.rotation * center.rotation.transpose()).y();
    case FrfChannel::kRz: return so3Log(pose.rotation * center.rotation.transpose()).z();
  }
  return 0.0;
}

void writeJointColumns(std::ostream& stream, const Vec7& value) {
  for (int joint = 0; joint < kArmDof; ++joint) stream << ',' << value[joint];
}

int run(const Options& options) {
  QpIkConfig config = loadConfig(options.config_path);
  config.controller.rate_hz = 200.0;
  config.controller.model_state_only = true;
  config.control_level = ControlLevel::kVelocity;
  config.ik_algorithm = algorithmFromString(options.algorithm);
  MujocoRobot robot(options.model_path);
  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    robot.setArmState(side, configuredInitialPosture(config.controller,
                      robot.mapping(side).limits, side), Vec7::Zero());
  }
  robot.forward();
  const Pose initial_left = robot.tcpPose(ArmSide::kLeft);
  const Pose initial_right = robot.tcpPose(ArmSide::kRight);
  const Eigen::Vector3d offset = workingPointOffset(options.working_point);
  Pose left_center = initial_left;
  Pose right_center = initial_right;
  if (options.arm == ArmSide::kLeft) left_center.position += offset;
  else right_center.position += offset;

  DualArmController controller(robot, config);
  controller.setArmAngleReferenceMode(ArmAngleReferenceMode::kOutwardOnly);
  std::unique_ptr<DualArmSparkGuidance> guidance;
  if (usesSparkGuidance(config.ik_algorithm)) {
    guidance = std::make_unique<DualArmSparkGuidance>(
        robot, config, options.urdf_path, sparkMode(config.ik_algorithm));
    if (!guidance->reset(motionState(robot, ArmSide::kLeft),
                         motionState(robot, ArmSide::kRight))) {
      throw std::runtime_error("failed to reset SPARK guidance");
    }
  }
  PicoTeleopFrame frame = baseFrame(robot);
  Vec7 source_q = robot.armPosition(options.arm);
  IterativePoseDlsIk7 source_ik(config.iterative_dls,
                                config.joint_limits.margin_rad);

  std::ofstream output(options.output_path);
  if (!output) throw std::runtime_error("cannot open output CSV");
  output << std::setprecision(17);
  output << "# algorithm=" << options.algorithm << '\n';
  output << "# arm=" << (options.arm == ArmSide::kLeft ? "left" : "right") << '\n';
  output << "# working_point=" << options.working_point << '\n';
  output << "# channel=" << toString(options.channel) << '\n';
  output << "# dt=0.005\n# source_dt=0.010\n";
  output << "# warmup=" << options.excitation.warmup_seconds << '\n';
  output << "# chirp=" << options.excitation.chirp_seconds << '\n';
  output << "# settle=" << options.excitation.settle_seconds << '\n';
  output << "sample,time_s,input,skeleton_input,output,target_input,accepted,solver_status,solve_time_us,position_violation,velocity_violation,acceleration_violation,jerk_violation";
  for (const char* prefix : {"q", "dq", "ddq", "jerk"})
    for (int joint = 1; joint <= kArmDof; ++joint) output << ',' << prefix << joint;
  output << '\n';

  const double dt = 0.005;
  FrfExcitationConfig excitation = options.excitation;
  excitation.dt = dt;
  const int samples = static_cast<int>(std::llround(
      (excitation.warmup_seconds + excitation.chirp_seconds + excitation.settle_seconds) / dt));
  Vec7 previous_qdot = Vec7::Zero();
  Vec7 previous_qddot = Vec7::Zero();
  Pose accepted_center = options.arm == ArmSide::kLeft ? left_center : right_center;
  bool accepted_center_set = false;
  double skeleton_input = 0.0;
  for (int sample = 0; sample < samples; ++sample) {
    const double command = logChirpSample(excitation, sample);
    DualArmTargets targets{left_center, right_center};
    SparkGuidanceDiagnostics spark;
    if (guidance) {
      if ((sample % 2) == 0) {
        const Pose perturbed = perturbPose(
            options.arm == ArmSide::kLeft ? left_center : right_center,
            options.channel, command);
        PoseDlsInput source_input;
        source_input.target = perturbed;
        source_input.seed = source_q;
        source_input.limits = robot.mapping(options.arm).limits;
        source_input.evaluate = [&robot, &options](const Vec7& q) {
          return robot.armKinematicsAt(options.arm, q);
        };
        const PoseDlsResult source_result = source_ik.solve(source_input);
        if (source_result.status == PoseDlsStatus::kRejected)
          throw std::runtime_error("synthetic corrected-skeleton IK failed");
        source_q = source_result.q;
        const ArmKinematicSample source_geometry =
            robot.armKinematicsAt(options.arm, source_q);
        skeleton_input = poseComponent(
            source_geometry.tcp_pose,
            options.arm == ArmSide::kLeft ? left_center : right_center,
            options.channel);
        if (options.arm == ArmSide::kLeft) {
          frame.left = perturbed;
          frame.upper_limb_skeleton.points[0] = source_geometry.shoulder_position;
          frame.upper_limb_skeleton.points[1] = source_geometry.elbow_position;
          frame.upper_limb_skeleton.points[2] = source_geometry.wrist_position;
          frame.upper_limb_skeleton.points[3] = source_geometry.tcp_pose.position;
        } else {
          frame.right = perturbed;
          frame.upper_limb_skeleton.points[4] = source_geometry.shoulder_position;
          frame.upper_limb_skeleton.points[5] = source_geometry.elbow_position;
          frame.upper_limb_skeleton.points[6] = source_geometry.wrist_position;
          frame.upper_limb_skeleton.points[7] = source_geometry.tcp_pose.position;
        }
        frame.sequence = static_cast<std::uint64_t>(sample / 2 + 1);
        frame.source_timestamp_ns = static_cast<std::int64_t>(sample) * 5000000;
        if (!guidance->updatePicoFrame(frame).valid)
          throw std::runtime_error("synthetic SPARK frame rejected");
      }
      spark = guidance->step(controller.referenceState(ArmSide::kLeft),
                             controller.referenceState(ArmSide::kRight), dt);
      if (!spark.accepted) throw std::runtime_error("SPARK guidance failed");
      targets = spark.cartesian_targets;
    } else {
      if (options.arm == ArmSide::kLeft) targets.left = perturbPose(left_center, options.channel, command);
      else targets.right = perturbPose(right_center, options.channel, command);
    }
    const Pose accepted_pose = options.arm == ArmSide::kLeft ? targets.left : targets.right;
    if (!accepted_center_set && sample == 0) {
      accepted_center = accepted_pose;
      accepted_center_set = true;
    }
    ControllerDiagnostics result;
    if (guidance) {
      const DualArmReferences references = spark.cartesian_references_valid
          ? spark.cartesian_references : DualArmReferences{};
      if (spark.cartesian_references_valid) {
        result = controller.step(references, {}, spark.posture_tasks, dt);
      } else {
        result = controller.step(targets, {}, dt);
      }
    } else {
      result = controller.step(targets, {}, dt);
    }
    if (guidance && usesSparkHeadroomFeedforwardVelocityQp(
                        config.ik_algorithm)) {
      const auto feedback = [&controller](
                                ArmSide side,
                                const ArmControllerDiagnostics& arm) {
        SparkConstraintHeadroomFeedback value;
        value.accepted = arm.accepted &&
                         arm.ik.status == SolverStatus::kSolved;
        value.qdot = arm.ik.qdot;
        value.qddot = controller.previousAcceleration(side);
        value.task_scale_position = arm.ik.task_scale_position;
        value.task_scale_orientation = arm.ik.task_scale_orientation;
        return value;
      };
      guidance->updateHeadroomFeedback(
          feedback(ArmSide::kLeft, result.left),
          feedback(ArmSide::kRight, result.right), dt);
    }
    robot.forward();
    const ArmControllerDiagnostics& arm = options.arm == ArmSide::kLeft ? result.left : result.right;
    const Vec7 q = arm.q_ref;
    const Vec7 qdot = arm.ik.qdot;
    const Vec7 qddot = (qdot - previous_qdot) / dt;
    const Vec7 jerk = (qddot - previous_qddot) / dt;
    previous_qdot = qdot;
    previous_qddot = qddot;
    const Pose measured = robot.tcpPose(options.arm);
    const double actual_input = poseComponent(accepted_pose, accepted_center, options.channel);
    const double measured_output = poseComponent(measured, accepted_center, options.channel);
    const ArmLimits& limits = robot.mapping(options.arm).limits;
    const bool position_violation =
        (q.array() < (limits.lower_position.array() - config.safety.bound_tolerance)).any() ||
        (q.array() > (limits.upper_position.array() + config.safety.bound_tolerance)).any();
    const Vec7 velocity_limit = config.joint_limits.velocity_scale * limits.velocity;
    const bool velocity_violation =
        (qdot.cwiseAbs().array() > velocity_limit.array() + 1.0e-8).any();
    const bool acceleration_violation =
        (qddot.cwiseAbs().array() > config.joint_limits.max_acceleration_rad_s2.array() + 1.0e-6).any();
    const bool jerk_violation = config.joint_limits.hard_jerk_enabled &&
        (jerk.cwiseAbs().array() > config.joint_limits.max_jerk_rad_s3.array() + 1.0e-4).any();
    output << sample << ',' << sample * dt << ',' << command << ',' << skeleton_input
           << ',' << measured_output
           << ',' << actual_input << ',' << (arm.accepted ? 1 : 0) << ','
           << static_cast<int>(arm.ik.status) << ',' << arm.ik.solve_time_us
           << ',' << position_violation << ',' << velocity_violation
           << ',' << acceleration_violation << ',' << jerk_violation;
    writeJointColumns(output, q);
    writeJointColumns(output, qdot);
    writeJointColumns(output, qddot);
    writeJointColumns(output, jerk);
    output << '\n';
  }
  return 0;
}

}  // namespace
}  // namespace tianji_qp_ik

int main(int argc, char** argv) {
  try {
    return tianji_qp_ik::run(tianji_qp_ik::parseOptions(argc, argv));
  } catch (const std::exception& error) {
    std::cerr << "cartesian_frf_error: " << error.what() << '\n';
    return 2;
  }
}
