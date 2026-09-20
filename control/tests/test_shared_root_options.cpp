#include "tianji_qp_ik/shared_root_options.hpp"
#include <gtest/gtest.h>
#include <fstream>
#include <filesystem>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

namespace tianji_qp_ik {
TEST(SharedRootOptions, NativeFingerprintsMatchFrozenEvidence) {
  EXPECT_EQ(sharedRootSha256File(TIANJI_PROJECT_SOURCE_DIR "/config/shared_root_tjvr_input_contract.yaml"),
            "68304d5b76b063996bdf386b28d103af1d998f7f9a801acb5a55960aaa2fccda");
  auto options=loadSharedRootOptions(TIANJI_PROJECT_SOURCE_DIR "/config/qp_ik_pico_shared_root.yaml");
  EXPECT_FALSE(options.enabled);
  EXPECT_NEAR(options.morphology.robot_width_m,.423,1e-12);
  EXPECT_NEAR(options.builder.geometry.left_wrist_to_palm_local.norm(),.1615,1e-12);
  EXPECT_EQ(options.morphology.minimum_unique_samples,15U);
  EXPECT_EQ(options.continuity.recovery_frames,5U);
}
TEST(SharedRootOptions, Sha256KnownVectors) {
  auto p=std::filesystem::temp_directory_path()/("shared_root_sha_"+std::to_string(getpid()));
  {std::ofstream f(p);}
  EXPECT_EQ(sharedRootSha256File(p.string()),"e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  {std::ofstream f(p);f<<"abc";}
  EXPECT_EQ(sharedRootSha256File(p.string()),"ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  std::filesystem::remove(p);
  EXPECT_THROW(sharedRootSha256File(p.string()),std::runtime_error);
}
TEST(SharedRootOptions, ActiveSparkAndCeresShareSourceModelAndMotionEnvelope) {
  const std::string base=TIANJI_PROJECT_SOURCE_DIR "/config/";
  const auto spark=loadSharedRootOptions(base+"qp_ik_pico_shared_root_reachable.yaml");
  const auto ceres=loadSharedRootOptions(base+"qp_ik_pico_shared_root_ceres.yaml");
  EXPECT_FALSE(spark.enabled); EXPECT_FALSE(ceres.enabled);
  EXPECT_EQ(spark.geometry_sha256,ceres.geometry_sha256);
  EXPECT_EQ(spark.urdf_path,ceres.urdf_path);
  EXPECT_EQ(spark.mujoco_path,ceres.mujoco_path);
  const auto s=YAML::LoadFile(base+"qp_ik_pico_shared_root_reachable.yaml");
  const auto c=YAML::LoadFile(base+"qp_ik_pico_shared_root_ceres.yaml");
  EXPECT_EQ(s["ik"]["algorithm"].as<std::string>(),"spark_upper_qpoases_headroom_feedforward_velocity_qp");
  for(const auto* key:{"rate_hz","initial_left_q_rad","initial_right_q_rad"})
    EXPECT_EQ(YAML::Dump(s["controller"][key]),YAML::Dump(c["controller"][key]))<<key;
  for(const auto* block:{"joint_limits","joint_acceleration_limits"})
    for(const auto* key:{"margin_rad","velocity_scale","max_acceleration_rad_s2","max_jerk_rad_s3","hard_jerk_enabled"})
      EXPECT_EQ(YAML::Dump(s[block][key]),YAML::Dump(c[block][key]))<<block<<":"<<key;
}
class SharedRootOptionsMutation : public ::testing::Test {
 protected:
  void SetUp() override {
    auto pattern=(std::filesystem::temp_directory_path()/"shared_root_options_XXXXXX").string();
    auto* dir=mkdtemp(pattern.data());ASSERT_NE(dir,nullptr);
    directory=dir;profile=directory/"profile.yaml";
    node=YAML::LoadFile(TIANJI_PROJECT_SOURCE_DIR "/config/qp_ik_pico_shared_root.yaml");
    auto c=node["spark_shared_root"];
    c["input_contract_artifact"]=TIANJI_PROJECT_SOURCE_DIR "/config/shared_root_tjvr_input_contract.yaml";
    c["robot_geometry_artifact"]=TIANJI_PROJECT_SOURCE_DIR "/config/shared_root_robot_geometry.yaml";
  }
  void TearDown() override { if(!directory.empty())std::filesystem::remove_all(directory); }
  void write() { std::ofstream stream(profile); stream<<node; }
  std::filesystem::path directory,profile;
  YAML::Node node;
};
TEST_F(SharedRootOptionsMutation, IndependentProfileLocationAndExplicitPaths) {
  write();EXPECT_NO_THROW(loadSharedRootOptions(profile.string()));
}
TEST_F(SharedRootOptionsMutation, SourceLimitRevisionSupportsBothReviewedBackends) {
  node["spark_shared_root"]["robot_geometry_artifact"]=TIANJI_PROJECT_SOURCE_DIR "/config/shared_root_robot_geometry_ceres.yaml";
  write();EXPECT_NO_THROW(loadSharedRootOptions(profile.string()));
  node["ik"]["algorithm"]="pico_ee_franka_ceres_lm";
  write();EXPECT_NO_THROW(loadSharedRootOptions(profile.string()));
}
TEST_F(SharedRootOptionsMutation, ReachableProjectionIsExplicitAndValidated) {
  write();EXPECT_FALSE(loadSharedRootOptions(profile.string()).builder.reachable_projection_enabled);
  auto p=node["spark_shared_root"]["reachable_projection"];
  p["enabled"]=true;p["maximum_translation_m"]=.03;p["maximum_speed_m_s"]=.5;write();
  EXPECT_TRUE(loadSharedRootOptions(profile.string()).builder.reachable_projection_enabled);
  p["maximum_translation_m"]=.11;write();
  EXPECT_THROW(loadSharedRootOptions(profile.string()),std::runtime_error);
  p["maximum_translation_m"]=.03;p["maximum_speed_m_s"]=".nan";write();
  EXPECT_THROW(loadSharedRootOptions(profile.string()),std::runtime_error);
}
TEST_F(SharedRootOptionsMutation, EnabledProfileRequiresAwareConsumerAndValidFullContract) {
  node["spark_shared_root"]["enabled"]=true;write();
  EXPECT_THROW(loadConfig(profile.string()),std::runtime_error);
  const auto config=loadConfig(profile.string(),ConfigConsumer::kSharedRootAware);
  EXPECT_EQ(config.shared_root_profile_path,std::filesystem::canonical(profile).string());
  node["spark_shared_root"]["morphology"]["window_frames"]=129;write();
  EXPECT_THROW(loadConfig(profile.string(),ConfigConsumer::kSharedRootAware),std::runtime_error);
}
TEST_F(SharedRootOptionsMutation, DisabledAwareConsumerDoesNotAccessArtifacts) {
  node["spark_shared_root"]["input_contract_artifact"]="nonexistent";write();
  const auto config=loadConfig(profile.string(),ConfigConsumer::kSharedRootAware);
  EXPECT_TRUE(config.shared_root_profile_path.empty());
}
TEST_F(SharedRootOptionsMutation, RejectsMixUnknownNonfiniteAndInvalidWindows) {
  auto c=node["spark_shared_root"];c["mix"]=.5;write();
  EXPECT_THROW(loadSharedRootOptions(profile.string()),std::runtime_error);
  c.remove("mix");c["morphology"]["window_frames"]=129;write();
  EXPECT_THROW(loadSharedRootOptions(profile.string()),std::runtime_error);
  c["morphology"]["window_frames"]=31;c["target_gate"]["maximum_center_offset_from_robot_shoulders_m"]=".nan";write();
  EXPECT_THROW(loadSharedRootOptions(profile.string()),std::runtime_error);
}
TEST_F(SharedRootOptionsMutation, TamperedGeometryCannotRelaxEvidenceChecks) {
  auto geometry=YAML::LoadFile(TIANJI_PROJECT_SOURCE_DIR "/config/shared_root_robot_geometry.yaml");
  geometry["robot_geometry"]["cross_model_position_tolerance_m"]=10;
  const auto artifact=directory/"geometry.yaml";
  {std::ofstream stream(artifact);stream<<geometry;}
  node["spark_shared_root"]["robot_geometry_artifact"]=artifact.string();write();
  EXPECT_THROW(loadSharedRootOptions(profile.string()),std::runtime_error);
}
TEST_F(SharedRootOptionsMutation, RejectsDuplicateConfigurationKeys) {
  write();
  {std::ofstream stream(profile,std::ios::app);stream<<"\n  enabled: false\n";}
  EXPECT_THROW(loadSharedRootOptions(profile.string()),std::runtime_error);
}
} // namespace tianji_qp_ik
