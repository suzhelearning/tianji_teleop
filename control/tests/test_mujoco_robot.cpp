#include "tianji_qp_ik/arm_angle.hpp"
#include "tianji_qp_ik/mujoco_robot.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

namespace tianji_qp_ik {
namespace {

std::string modelPath() {
  return (std::filesystem::path(TIANJI_PROJECT_SOURCE_DIR) / "models" /
          "marvin_m6_qp_test.xml")
      .string();
}

std::string picoFastModelPath() {
  return (std::filesystem::path(TIANJI_PROJECT_SOURCE_DIR) / "models" /
          "marvin_m6_qp_pico_fast.xml")
      .string();
}

std::string fullWujiModelPath() {
  return (std::filesystem::path(TIANJI_PROJECT_SOURCE_DIR) / "models" /
          "marvin_m6_wuji2.xml")
      .string();
}

void expectUrdfJointLimits(const std::filesystem::path& path,
                           const std::string& joint_name,
                           const std::string& lower,
                           const std::string& upper) {
  std::ifstream stream(path);
  ASSERT_TRUE(stream.good()) << path;
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  const std::string document = buffer.str();
  const std::string name = "name=\"" + joint_name + "\"";
  const std::size_t joint_begin = document.find(name);
  ASSERT_NE(joint_begin, std::string::npos) << path << " " << joint_name;
  const std::size_t joint_end = document.find("</joint>", joint_begin);
  ASSERT_NE(joint_end, std::string::npos) << path << " " << joint_name;
  const std::string joint =
      document.substr(joint_begin, joint_end - joint_begin);
  EXPECT_NE(joint.find("lower=\"" + lower + "\""), std::string::npos)
      << path << " " << joint_name;
  EXPECT_NE(joint.find("upper=\"" + upper + "\""), std::string::npos)
      << path << " " << joint_name;
}

void expectUrdfJointVelocity(const std::filesystem::path& path,
                             const std::string& joint_name,
                             const std::string& velocity) {
  std::ifstream stream(path);
  ASSERT_TRUE(stream.good()) << path;
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  const std::string document = buffer.str();
  const std::string name = "name=\"" + joint_name + "\"";
  const std::size_t joint_begin = document.find(name);
  ASSERT_NE(joint_begin, std::string::npos) << path << " " << joint_name;
  const std::size_t joint_end = document.find("</joint>", joint_begin);
  ASSERT_NE(joint_end, std::string::npos) << path << " " << joint_name;
  const std::string joint =
      document.substr(joint_begin, joint_end - joint_begin);
  EXPECT_NE(joint.find("velocity=\"" + velocity + "\""), std::string::npos)
      << path << " " << joint_name;
}

void expectUrdfJointExists(const std::filesystem::path& path,
                           const std::string& joint_name) {
  std::ifstream stream(path);
  ASSERT_TRUE(stream.good()) << path;
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  const std::string document = buffer.str();
  EXPECT_NE(document.find("<joint name=\"" + joint_name + "\""),
            std::string::npos)
      << path << " " << joint_name;
}

double signedArmAngleError(const ArmKinematicSample& sample,
                           const Eigen::Vector3d& world_reference) {
  const Eigen::Vector3d axis =
      (sample.wrist_position - sample.shoulder_position).normalized();
  Eigen::Vector3d current =
      sample.elbow_position - sample.shoulder_position;
  current -= axis * axis.dot(current);
  current.normalize();
  Eigen::Vector3d projected =
      world_reference - axis * axis.dot(world_reference);
  projected.normalize();
  return std::atan2(axis.dot(current.cross(projected)),
                    current.dot(projected));
}

TEST(MujocoRobot, MapsExactlySevenJointsPerArmByName) {
  MujocoRobot robot(modelPath());
  EXPECT_EQ(robot.model()->nq, 14);
  EXPECT_EQ(robot.model()->nv, 14);
  EXPECT_EQ(robot.model()->nmocap, 2);

  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    const ArmMapping& mapping = robot.mapping(side);
    const std::string suffix = side == ArmSide::kLeft ? "L" : "R";
    for (int index = 0; index < kArmDof; ++index) {
      EXPECT_EQ(mapping.joint_names[static_cast<std::size_t>(index)],
                "Joint" + std::to_string(index + 1) + "_" + suffix);
      EXPECT_EQ(mapping.body_names[static_cast<std::size_t>(index)],
                "Link" + std::to_string(index + 1) + "_" + suffix);
      EXPECT_GE(mapping.joint_ids[static_cast<std::size_t>(index)], 0);
      EXPECT_GE(mapping.body_ids[static_cast<std::size_t>(index)], 0);
      EXPECT_GE(mapping.qpos_addresses[static_cast<std::size_t>(index)], 0);
      EXPECT_GE(mapping.dof_addresses[static_cast<std::size_t>(index)], 0);
      EXPECT_GT(mapping.limits.upper_position[index], mapping.limits.lower_position[index]);
      EXPECT_NEAR(mapping.limits.velocity[index], 3.1416, 1e-12);
    }
    EXPECT_GE(mapping.tcp_site_id, 0);
    EXPECT_EQ(mapping.tcp_body_id, mapping.body_ids.back());
    EXPECT_GE(robot.targetBodyId(side), 0);
    EXPECT_GE(robot.targetMocapId(side), 0);
  }
  EXPECT_NE(robot.targetMocapId(ArmSide::kLeft), robot.targetMocapId(ArmSide::kRight));
}

TEST(MujocoRobot, Joint4UsesHumanLikeElbowRange) {
  MujocoRobot robot(modelPath());

  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    const ArmLimits& limits = robot.mapping(side).limits;
    EXPECT_NEAR(limits.lower_position[3], -2.5307, 1e-12);
    EXPECT_NEAR(limits.upper_position[3], 0.0, 1e-12);
  }
}

TEST(MujocoRobot, BothModelsUseSideSpecificJoint1AndJoint3TeleoperationEnvelope) {
  for (const std::string& path : {modelPath(), picoFastModelPath()}) {
    MujocoRobot robot(path);
    const ArmLimits& left = robot.mapping(ArmSide::kLeft).limits;
    const ArmLimits& right = robot.mapping(ArmSide::kRight).limits;

    EXPECT_NEAR(left.lower_position[0], -1.5708, 1e-12) << path;
    EXPECT_NEAR(left.upper_position[0], 3.1067, 1e-12) << path;
    EXPECT_NEAR(right.lower_position[0], -3.1067, 1e-12) << path;
    EXPECT_NEAR(right.upper_position[0], 1.5708, 1e-12) << path;
    EXPECT_NEAR(left.lower_position[2], -3.1067, 1e-12) << path;
    EXPECT_NEAR(left.upper_position[2], 0.0, 1e-12) << path;
    EXPECT_NEAR(right.lower_position[2], 0.0, 1e-12) << path;
    EXPECT_NEAR(right.upper_position[2], 3.1067, 1e-12) << path;
  }
}

TEST(MujocoRobot, AllUrdfModelsUseSideSpecificJoint1AndJoint3Envelope) {
  const std::filesystem::path root(TIANJI_PROJECT_SOURCE_DIR);
  for (const std::filesystem::path& relative : {
           std::filesystem::path("marvin_m6_ccs/urdf/marvin_m6_s_ccs_696_v4.urdf"),
           std::filesystem::path(
               "marvin_m6_ccs/urdf/marvin_m6_s_ccs_696_v4_mujoco.urdf"),
           std::filesystem::path("models/marvin_m6_s_ccs_696_v4_local.urdf")}) {
    const std::filesystem::path path = root / relative;
    expectUrdfJointLimits(path, "Joint1_L", "-1.5708", "3.1067");
    expectUrdfJointLimits(path, "Joint1_R", "-3.1067", "1.5708");
    expectUrdfJointLimits(path, "Joint3_L", "-3.1067", "0");
    expectUrdfJointLimits(path, "Joint3_R", "0", "3.1067");
  }
}

TEST(MujocoRobot, PicoFastModelUsesFourRadiansPerSecondJointLimits) {
  MujocoRobot robot(picoFastModelPath());
  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    const ArmLimits& limits = robot.mapping(side).limits;
    for (int joint = 0; joint < kArmDof; ++joint) {
      EXPECT_NEAR(limits.velocity[joint], 4.0, 1e-12);
    }
    EXPECT_NEAR(limits.lower_position[3], -2.5307, 1e-12);
    EXPECT_NEAR(limits.upper_position[3], 0.0, 1e-12);
  }
}

TEST(MujocoRobot, FullWujiModelContainsArmHandsAndViewerTargets) {
  char error[1024]{};
  mjModel* model = mj_loadXML(fullWujiModelPath().c_str(), nullptr, error,
                              sizeof(error));
  ASSERT_NE(model, nullptr) << error;
  ASSERT_EQ(model->nuser_jnt, 1);
  EXPECT_EQ(model->nq, 54);
  EXPECT_EQ(model->nv, 54);

  struct ArmLimit {
    const char* name;
    double lower;
    double upper;
  };
  const std::array<ArmLimit, 14> expected_arm_limits{{
      {"Joint1_L", -1.5708, 3.1067},
      {"Joint2_L", -2.0944, 2.0944},
      {"Joint3_L", -3.1067, 0.0},
      {"Joint4_L", -2.5307, 0.0},
      {"Joint5_L", -3.1067, 3.1067},
      {"Joint6_L", -1.0472, 1.0472},
      {"Joint7_L", -1.5708, 1.5708},
      {"Joint1_R", -3.1067, 1.5708},
      {"Joint2_R", -2.0944, 2.0944},
      {"Joint3_R", 0.0, 3.1067},
      {"Joint4_R", -2.5307, 0.0},
      {"Joint5_R", -3.1067, 3.1067},
      {"Joint6_R", -1.0472, 1.0472},
      {"Joint7_R", -1.5708, 1.5708},
  }};
  for (const ArmLimit& expected : expected_arm_limits) {
    const int id = mj_name2id(model, mjOBJ_JOINT, expected.name);
    ASSERT_GE(id, 0) << expected.name;
    EXPECT_NEAR(model->jnt_range[2 * id], expected.lower, 1e-12)
        << expected.name;
    EXPECT_NEAR(model->jnt_range[2 * id + 1], expected.upper, 1e-12)
        << expected.name;
    EXPECT_NEAR(model->jnt_user[id], 4.0, 1e-12) << expected.name;
  }

  for (const char* joint : {"l_thumb_cmc_flex", "l_index_finger_mcp_flex",
                            "r_thumb_cmc_flex", "r_index_finger_mcp_flex"}) {
    EXPECT_GE(mj_name2id(model, mjOBJ_JOINT, joint), 0) << joint;
  }
  for (const char* site : {"tcp_L", "tcp_R"}) {
    EXPECT_GE(mj_name2id(model, mjOBJ_SITE, site), 0) << site;
  }
  for (const char* target : {"target_L", "target_R"}) {
    const int body_id = mj_name2id(model, mjOBJ_BODY, target);
    ASSERT_GE(body_id, 0) << target;
    EXPECT_GE(model->body_mocapid[body_id], 0) << target;
  }
  mj_deleteModel(model);
}

TEST(MujocoRobot, MapsArmJointsInFullWujiModelWithHandDofs) {
  MujocoRobot robot(fullWujiModelPath());
  ASSERT_EQ(robot.model()->nq, 54);
  ASSERT_EQ(robot.model()->nv, 54);
  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    const ArmMapping& mapping = robot.mapping(side);
    for (int index = 0; index < kArmDof; ++index) {
      const std::size_t offset = static_cast<std::size_t>(index);
      EXPECT_GE(mapping.joint_ids[offset], 0);
      EXPECT_GE(mapping.body_ids[offset], 0);
      EXPECT_GE(mapping.qpos_addresses[offset], 0);
      EXPECT_GE(mapping.dof_addresses[offset], 0);
      EXPECT_NEAR(mapping.limits.velocity[index], 4.0, 1e-12);
    }
  }
}

TEST(MujocoRobot, MapsOfficialWujiHand2JointsByNameAndIsolatesArms) {
  MujocoRobot robot(fullWujiModelPath());
  const Vec7 left_arm_before = robot.armPosition(ArmSide::kLeft);
  const Vec7 right_arm_before = robot.armPosition(ArmSide::kRight);

  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    const HandMapping& mapping = robot.handMapping(side);
    const std::string prefix = side == ArmSide::kLeft ? "l_" : "r_";
    const std::array<const char*, kHandDof> stems{
        "thumb_cmc_flex", "thumb_cmc_abd", "thumb_mcp", "thumb_ip",
        "index_finger_mcp_flex", "index_finger_mcp_abd",
        "index_finger_pip", "index_finger_dip",
        "middle_finger_mcp_flex", "middle_finger_mcp_abd",
        "middle_finger_pip", "middle_finger_dip",
        "ring_finger_mcp_flex", "ring_finger_mcp_abd",
        "ring_finger_pip", "ring_finger_dip",
        "pinky_mcp_flex", "pinky_mcp_abd", "pinky_pip", "pinky_dip"};
    for (int joint = 0; joint < kHandDof; ++joint) {
      const std::size_t index = static_cast<std::size_t>(joint);
      EXPECT_EQ(mapping.joint_names[index], prefix + stems[index]);
      EXPECT_GE(mapping.joint_ids[index], 0);
      EXPECT_GE(mapping.qpos_addresses[index], 0);
      EXPECT_GT(mapping.upper_position[joint], mapping.lower_position[joint]);
      EXPECT_NE(mapping.qpos_addresses[index],
                robot.mapping(side).qpos_addresses[0]);
    }
  }

  Vec20 left_command = Vec20::LinSpaced(kHandDof, -2.0, 2.0);
  const Vec20 right_before = robot.handPosition(ArmSide::kRight);
  robot.setHandPosition(ArmSide::kLeft, left_command);

  const Vec20 left_after = robot.handPosition(ArmSide::kLeft);
  for (int joint = 0; joint < kHandDof; ++joint) {
    EXPECT_GE(left_after[joint],
              robot.handMapping(ArmSide::kLeft).lower_position[joint]);
    EXPECT_LE(left_after[joint],
              robot.handMapping(ArmSide::kLeft).upper_position[joint]);
  }
  EXPECT_TRUE(robot.handPosition(ArmSide::kRight).isApprox(right_before));
  EXPECT_TRUE(robot.armPosition(ArmSide::kLeft).isApprox(left_arm_before));
  EXPECT_TRUE(robot.armPosition(ArmSide::kRight).isApprox(right_arm_before));
}

TEST(MujocoRobot, WujiHand2ModelUsesHandWristTcpAndKeepsFlangeTcp) {
  MujocoRobot main_model(picoFastModelPath());
  MujocoRobot hand2_model(fullWujiModelPath());
  main_model.forward();
  hand2_model.forward();

  const auto site_pose = [](const MujocoRobot& robot, int site_id) {
    Pose pose;
    pose.position = Eigen::Map<const Eigen::Vector3d>(
        &robot.data()->site_xpos[3 * site_id]);
    using RowMajorMatrix3d = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;
    pose.rotation = Eigen::Map<const RowMajorMatrix3d>(
        &robot.data()->site_xmat[9 * site_id]);
    return pose;
  };

  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    const Pose main_tcp = main_model.tcpPose(side);
    const Pose hand2_tcp = hand2_model.tcpPose(side);
    const std::string suffix = side == ArmSide::kLeft ? "L" : "R";
    const int flange_site = mj_name2id(
        hand2_model.model(), mjOBJ_SITE, ("flange_tcp_" + suffix).c_str());
    const int hand_site = mj_name2id(
        hand2_model.model(), mjOBJ_SITE, ("hand_tcp_" + suffix).c_str());
    const int tcp_site = mj_name2id(
        hand2_model.model(), mjOBJ_SITE, ("tcp_" + suffix).c_str());

    ASSERT_GE(flange_site, 0);
    ASSERT_GE(hand_site, 0);
    ASSERT_GE(tcp_site, 0);
    EXPECT_EQ(
        mj_id2name(hand2_model.model(), mjOBJ_SITE,
                   hand2_model.mapping(side).tcp_site_id),
        std::string("hand_tcp_frame_") + suffix);

    const Pose flange_tcp = site_pose(hand2_model, flange_site);
    const Pose hand_tcp = site_pose(hand2_model, hand_site);
    const Pose selected_tcp = site_pose(hand2_model, tcp_site);
    const Eigen::Vector3d hand_offset_in_flange =
        flange_tcp.rotation.transpose() *
        (hand_tcp.position - flange_tcp.position);

    EXPECT_TRUE(main_tcp.position.isApprox(flange_tcp.position, 1.0e-12))
        << "flange TCP position differs from main for "
        << (side == ArmSide::kLeft ? "left" : "right") << " arm";
    EXPECT_TRUE(main_tcp.rotation.isApprox(flange_tcp.rotation, 1.0e-12))
        << "flange TCP orientation differs from main for "
        << (side == ArmSide::kLeft ? "left" : "right") << " arm";
    EXPECT_TRUE(hand2_tcp.position.isApprox(hand_tcp.position, 1.0e-12))
        << "selected TCP position differs from hand wrist for "
        << (side == ArmSide::kLeft ? "left" : "right") << " arm";
    EXPECT_TRUE(hand2_tcp.rotation.isApprox(hand_tcp.rotation, 1.0e-12))
        << "selected TCP orientation differs from hand wrist for "
        << (side == ArmSide::kLeft ? "left" : "right") << " arm";
    EXPECT_TRUE(hand_tcp.position.isApprox(selected_tcp.position, 1.0e-12))
        << "hand TCP position differs from selected TCP for "
        << (side == ArmSide::kLeft ? "left" : "right") << " arm";
    EXPECT_TRUE(hand_tcp.rotation.isApprox(selected_tcp.rotation, 1.0e-12))
        << "hand TCP orientation differs from selected TCP for "
        << (side == ArmSide::kLeft ? "left" : "right") << " arm";
    EXPECT_TRUE(hand_tcp.rotation.isApprox(flange_tcp.rotation, 1.0e-12))
        << "hand TCP orientation differs from the PICO-compatible flange TCP for "
        << (side == ArmSide::kLeft ? "left" : "right") << " arm";
    EXPECT_NEAR(hand_offset_in_flange.x(), 0.0, 1.0e-6)
        << "hand wrist is not on the flange Z axis for "
        << (side == ArmSide::kLeft ? "left" : "right") << " arm";
    EXPECT_NEAR(hand_offset_in_flange.y(), 0.0, 1.0e-6)
        << "hand wrist is not on the flange Z axis for "
        << (side == ArmSide::kLeft ? "left" : "right") << " arm";
    EXPECT_NEAR(hand_offset_in_flange.z(), 0.0365, 1.0e-6)
        << "hand wrist Z offset changed for "
        << (side == ArmSide::kLeft ? "left" : "right") << " arm";
  }
}

TEST(MujocoRobot, WujiHand2ModelExposesHandTcpCoordinateAxes) {
  MujocoRobot robot(fullWujiModelPath());

  for (const std::string suffix : {"L", "R"}) {
    const int origin_id = mj_name2id(
        robot.model(), mjOBJ_GEOM,
        ("hand_tcp_" + suffix + "_origin").c_str());
    ASSERT_GE(origin_id, 0) << "missing hand TCP origin for " << suffix;
    EXPECT_EQ(robot.model()->geom_type[origin_id], mjGEOM_SPHERE);
    EXPECT_EQ(robot.model()->geom_priority[origin_id], 0);
    EXPECT_EQ(robot.model()->geom_contype[origin_id], 0);
    EXPECT_EQ(robot.model()->geom_conaffinity[origin_id], 0);

    for (const std::string axis : {"x", "y", "z"}) {
      const int geom_id = mj_name2id(
          robot.model(), mjOBJ_GEOM,
          ("hand_tcp_" + suffix + "_axis_" + axis).c_str());

      ASSERT_GE(geom_id, 0)
          << "missing hand TCP " << axis << " axis for " << suffix;
      EXPECT_EQ(robot.model()->geom_type[geom_id], mjGEOM_CAPSULE);
      EXPECT_EQ(robot.model()->geom_contype[geom_id], 0);
      EXPECT_EQ(robot.model()->geom_conaffinity[geom_id], 0);
      EXPECT_GE(robot.model()->geom_size[3 * geom_id], 0.003);
      EXPECT_EQ(
          mj_id2name(robot.model(), mjOBJ_BODY,
                     robot.model()->geom_bodyid[geom_id]),
          std::string("hand_tcp_mount_") + suffix);
    }
  }
}

TEST(MujocoRobot, SelectedTcpRelativeToLink7UsesHandTcpFrame) {
  MujocoRobot robot(fullWujiModelPath());
  Eigen::Quaterniond expected_rotation(0.499998, 0.5, -0.5, 0.500002);
  expected_rotation.normalize();

  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    const Pose relative = robot.tcpRelativeToLink7(side);
    EXPECT_TRUE(relative.position.isApprox(
        Eigen::Vector3d(0.0, -0.1315, 0.0), 1.0e-6));
    EXPECT_TRUE(relative.rotation.isApprox(expected_rotation.toRotationMatrix(),
                                           1.0e-6));
  }
}

TEST(MujocoRobot, FullWujiModelJ1ChangesArmPose) {
  MujocoRobot robot(fullWujiModelPath());
  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    const ArmLimits& limits = robot.mapping(side).limits;
    Vec7 base = 0.5 * (limits.lower_position + limits.upper_position);
    robot.setArmPosition(side, base);
    robot.forward();
    const Pose before = robot.tcpPose(side);

    Vec7 moved = base;
    moved[0] += 0.25;
    robot.setArmPosition(side, moved);
    robot.forward();
    const Pose after = robot.tcpPose(side);

    EXPECT_NEAR(robot.armPosition(side)[0], moved[0], 1e-12);
    EXPECT_GT((after.position - before.position).norm(), 1e-5)
        << toString(side);
    EXPECT_GT((after.rotation - before.rotation).norm(), 1e-5)
        << toString(side);
  }
}

TEST(MujocoRobot, ImportedWujiUrdfUsesPicoFastArmEnvelope) {
  const std::filesystem::path path =
      std::filesystem::path(TIANJI_PROJECT_SOURCE_DIR) / "models" /
      "tianji_wuji2" /
      "tianji_wuji2.urdf";
  const std::array<const char*, 4> expected_hand_joints{
      "r_thumb_cmc_flex", "r_index_finger_mcp_flex",
      "l_thumb_cmc_flex", "l_index_finger_mcp_flex"};
  for (const char* joint : expected_hand_joints) {
    expectUrdfJointExists(path, joint);
  }

  struct ArmLimit {
    const char* name;
    const char* lower;
    const char* upper;
  };
  const std::array<ArmLimit, 14> expected_arm_limits{{
      {"Joint1_L", "-1.5708", "3.1067"},
      {"Joint2_L", "-2.0944", "2.0944"},
      {"Joint3_L", "-3.1067", "0"},
      {"Joint4_L", "-2.5307", "0"},
      {"Joint5_L", "-3.1067", "3.1067"},
      {"Joint6_L", "-1.0472", "1.0472"},
      {"Joint7_L", "-1.5708", "1.5708"},
      {"Joint1_R", "-3.1067", "1.5708"},
      {"Joint2_R", "-2.0944", "2.0944"},
      {"Joint3_R", "0", "3.1067"},
      {"Joint4_R", "-2.5307", "0"},
      {"Joint5_R", "-3.1067", "3.1067"},
      {"Joint6_R", "-1.0472", "1.0472"},
      {"Joint7_R", "-1.5708", "1.5708"},
  }};
  for (const ArmLimit& expected : expected_arm_limits) {
    expectUrdfJointLimits(path, expected.name, expected.lower, expected.upper);
    expectUrdfJointVelocity(path, expected.name, "4.0");
  }
}

TEST(MujocoRobot, SettingOneArmDoesNotChangeTheOther) {
  MujocoRobot robot(modelPath());
  const Vec7 right_before = robot.armPosition(ArmSide::kRight);
  Vec7 left;
  left << 0.1, -0.2, 0.3, -0.4, 0.2, -0.1, 0.05;
  robot.setArmPosition(ArmSide::kLeft, left);
  robot.forward();
  EXPECT_TRUE(robot.armPosition(ArmSide::kLeft).isApprox(left));
  EXPECT_TRUE(robot.armPosition(ArmSide::kRight).isApprox(right_before));
}

TEST(MujocoRobot, MapsVelocityStateAndKeepsArmsIsolated) {
  MujocoRobot robot(modelPath());
  Vec7 left_position;
  left_position << 0.1, -0.2, 0.3, -0.4, 0.2, -0.1, 0.05;
  Vec7 left_velocity;
  left_velocity << 0.7, -0.6, 0.5, -0.4, 0.3, -0.2, 0.1;
  const Vec7 right_position = robot.armPosition(ArmSide::kRight);
  const Vec7 right_velocity = Vec7::Constant(-0.25);
  robot.setArmState(ArmSide::kRight, right_position, right_velocity);
  robot.setArmState(ArmSide::kLeft, left_position, left_velocity);

  EXPECT_TRUE(robot.armPosition(ArmSide::kLeft).isApprox(left_position));
  EXPECT_TRUE(robot.armVelocity(ArmSide::kLeft).isApprox(left_velocity));
  EXPECT_TRUE(robot.armPosition(ArmSide::kRight).isApprox(right_position));
  EXPECT_TRUE(robot.armVelocity(ArmSide::kRight).isApprox(right_velocity));
}

TEST(MujocoRobot, TcpPosesAreFiniteProperRotations) {
  MujocoRobot robot(modelPath());
  robot.forward();
  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    const Pose pose = robot.tcpPose(side);
    EXPECT_TRUE(pose.position.allFinite());
    EXPECT_TRUE(pose.rotation.allFinite());
    EXPECT_TRUE((pose.rotation.transpose() * pose.rotation).isApprox(Eigen::Matrix3d::Identity(),
                                                                    1e-10));
    EXPECT_NEAR(pose.rotation.determinant(), 1.0, 1e-10);
  }
}

TEST(MujocoRobot, ArbitraryArmKinematicsDoesNotMutateActualState) {
  MujocoRobot robot(modelPath());
  Vec7 left_actual;
  left_actual << 0.10, -0.20, 0.30, -0.60, 0.20, -0.10, 0.05;
  Vec7 right_actual;
  right_actual << -0.15, 0.10, -0.20, -0.50, -0.10, 0.20, -0.05;
  robot.setArmPosition(ArmSide::kLeft, left_actual);
  robot.setArmPosition(ArmSide::kRight, right_actual);
  robot.forward();
  const Pose left_pose_before = robot.tcpPose(ArmSide::kLeft);
  const Pose right_pose_before = robot.tcpPose(ArmSide::kRight);

  Vec7 left_model = left_actual;
  left_model[0] += 0.20;
  left_model[3] -= 0.10;
  const ArmKinematicSample sample =
      robot.armKinematicsAt(ArmSide::kLeft, left_model);

  EXPECT_GT((sample.tcp_pose.position - left_pose_before.position).norm(),
            1e-4);
  EXPECT_TRUE(sample.tcp_pose.position.allFinite());
  EXPECT_TRUE(sample.tcp_pose.rotation.allFinite());
  EXPECT_TRUE(sample.tcp_jacobian.allFinite());
  EXPECT_TRUE(sample.shoulder_position.allFinite());
  EXPECT_TRUE(sample.elbow_position.allFinite());
  EXPECT_TRUE(sample.wrist_position.allFinite());
  EXPECT_TRUE(sample.shoulder_position_jacobian.allFinite());
  EXPECT_TRUE(sample.elbow_position_jacobian.allFinite());
  EXPECT_TRUE(sample.wrist_position_jacobian.allFinite());
  EXPECT_GT((sample.elbow_position - sample.shoulder_position).norm(), 0.05);
  EXPECT_GT((sample.wrist_position - sample.elbow_position).norm(), 0.05);
  constexpr double kFiniteDifferenceStep = 1e-6;
  for (int column = 0; column < kArmDof; ++column) {
    Vec7 plus = left_model;
    Vec7 minus = left_model;
    plus[column] += kFiniteDifferenceStep;
    minus[column] -= kFiniteDifferenceStep;
    const Eigen::Vector3d finite_difference =
        (robot.armKinematicsAt(ArmSide::kLeft, plus).tcp_pose.position -
         robot.armKinematicsAt(ArmSide::kLeft, minus).tcp_pose.position) /
        (2.0 * kFiniteDifferenceStep);
    EXPECT_TRUE(sample.tcp_jacobian.col(column).head<3>().isApprox(
        finite_difference, 1e-7));
    const Eigen::Vector3d elbow_finite_difference =
        (robot.armKinematicsAt(ArmSide::kLeft, plus).elbow_position -
         robot.armKinematicsAt(ArmSide::kLeft, minus).elbow_position) /
        (2.0 * kFiniteDifferenceStep);
    EXPECT_TRUE(sample.elbow_position_jacobian.col(column).isApprox(
        elbow_finite_difference, 1e-7));
    const Eigen::Vector3d shoulder_finite_difference =
        (robot.armKinematicsAt(ArmSide::kLeft, plus).shoulder_position -
         robot.armKinematicsAt(ArmSide::kLeft, minus).shoulder_position) /
        (2.0 * kFiniteDifferenceStep);
    EXPECT_TRUE(sample.shoulder_position_jacobian.col(column).isApprox(
        shoulder_finite_difference, 1e-7));
    const Eigen::Vector3d wrist_finite_difference =
        (robot.armKinematicsAt(ArmSide::kLeft, plus).wrist_position -
         robot.armKinematicsAt(ArmSide::kLeft, minus).wrist_position) /
        (2.0 * kFiniteDifferenceStep);
    EXPECT_TRUE(sample.wrist_position_jacobian.col(column).isApprox(
        wrist_finite_difference, 1e-7));
  }
  EXPECT_TRUE(robot.armPosition(ArmSide::kLeft).isApprox(left_actual, 1e-12));
  EXPECT_TRUE(robot.armPosition(ArmSide::kRight).isApprox(right_actual, 1e-12));
  const Pose left_pose_after = robot.tcpPose(ArmSide::kLeft);
  const Pose right_pose_after = robot.tcpPose(ArmSide::kRight);
  EXPECT_TRUE(left_pose_after.position.isApprox(left_pose_before.position,
                                                1e-12));
  EXPECT_TRUE(left_pose_after.rotation.isApprox(left_pose_before.rotation,
                                                1e-12));
  EXPECT_TRUE(right_pose_after.position.isApprox(right_pose_before.position,
                                                 1e-12));
  EXPECT_TRUE(right_pose_after.rotation.isApprox(right_pose_before.rotation,
                                                 1e-12));
}

TEST(MujocoRobot, RejectsNonFiniteArbitraryKinematicsPosition) {
  MujocoRobot robot(modelPath());
  Vec7 invalid = robot.armPosition(ArmSide::kLeft);
  invalid[2] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(robot.armKinematicsAt(ArmSide::kLeft, invalid),
               std::invalid_argument);
}

TEST(MujocoRobot, NominalArmAngleJacobianMatchesModelFiniteDifference) {
  MujocoRobot robot(modelPath());
  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    const ArmLimits& limits = robot.mapping(side).limits;
    const Vec7 nominal =
        0.5 * (limits.lower_position + limits.upper_position);
    const ArmKinematicSample model =
        robot.armKinematicsAt(side, nominal);
    ArmAngleTaskBuilder builder(side, 0.015, 0.050, 8.0);
    const ArmDirectionReference reference =
        side == ArmSide::kLeft ? defaultArmDirectionReferences().left
                               : defaultArmDirectionReferences().right;
    const ArmAngleTask task = builder.compute(
        {model.shoulder_position, model.elbow_position, model.wrist_position,
         model.shoulder_position_jacobian, model.elbow_position_jacobian,
         model.wrist_position_jacobian},
        reference, 0.005);
    ASSERT_TRUE(task.active);
    ASSERT_GT(task.jacobian.norm(), 1e-4) << toString(side);

    constexpr double kStep = 1e-6;
    for (int joint = 0; joint < kArmDof; ++joint) {
      Vec7 plus = nominal;
      Vec7 minus = nominal;
      plus[joint] += kStep;
      minus[joint] -= kStep;
      const double plus_error = signedArmAngleError(
          robot.armKinematicsAt(side, plus), reference.direction);
      const double minus_error = signedArmAngleError(
          robot.armKinematicsAt(side, minus), reference.direction);
      const double wrapped_error_delta = std::atan2(
          std::sin(plus_error - minus_error),
          std::cos(plus_error - minus_error));
      const double expected_current_rate =
          -wrapped_error_delta / (2.0 * kStep);
      EXPECT_NEAR(task.jacobian[joint], expected_current_rate, 1e-7)
          << toString(side) << " joint=" << joint;
    }
  }
}

TEST(MujocoRobot, MissingModelIsRejected) {
  EXPECT_THROW(MujocoRobot("/definitely/missing/tianji.xml"), std::runtime_error);
}

}  // namespace
}  // namespace tianji_qp_ik
