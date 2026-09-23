#include "tianji_qp_ik/shared_root_options.hpp"
#include <gtest/gtest.h>
#include <fstream>
#include <filesystem>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

namespace tianji_qp_ik {
TEST(SharedRootOptions, Sha256KnownVectors) {
  auto p=std::filesystem::temp_directory_path()/("shared_root_sha_"+std::to_string(getpid()));
  {std::ofstream f(p);}
  EXPECT_EQ(sharedRootSha256File(p.string()),"e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  {std::ofstream f(p);f<<"abc";}
  EXPECT_EQ(sharedRootSha256File(p.string()),"ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  std::filesystem::remove(p);
  EXPECT_THROW(sharedRootSha256File(p.string()),std::runtime_error);
}
class SharedRootOptionsMutation : public ::testing::Test {
 protected:
  void SetUp() override {
    auto pattern=(std::filesystem::temp_directory_path()/"shared_root_options_XXXXXX").string();
    auto* dir=mkdtemp(pattern.data());ASSERT_NE(dir,nullptr);
    directory=dir;profile=directory/"profile.yaml";
    node=YAML::LoadFile(TIANJI_PROJECT_SOURCE_DIR "/config/qp_ik_pico_shared_root_dls.yaml");
    auto c=node["shared_root"];
    c["input_contract_artifact"]=TIANJI_PROJECT_SOURCE_DIR "/config/shared_root_tjvr_input_contract.yaml";
    c["robot_geometry_artifact"]=TIANJI_PROJECT_SOURCE_DIR "/config/shared_root_robot_geometry_dls.yaml";
  }
  void TearDown() override { if(!directory.empty())std::filesystem::remove_all(directory); }
  void write() { std::ofstream stream(profile); stream<<node; }
  std::filesystem::path directory,profile;
  YAML::Node node;
};
TEST_F(SharedRootOptionsMutation, ReachableProjectionIsExplicitAndValidated) {
  node["shared_root"]["reachable_projection"]["enabled"]=false;
  write();EXPECT_FALSE(loadSharedRootOptions(profile.string()).builder.reachable_projection_enabled);
  auto p=node["shared_root"]["reachable_projection"];
  p["enabled"]=true;p["maximum_translation_m"]=.03;p["maximum_speed_m_s"]=.5;write();
  EXPECT_TRUE(loadSharedRootOptions(profile.string()).builder.reachable_projection_enabled);
  p["maximum_translation_m"]=.11;write();
  EXPECT_THROW(loadSharedRootOptions(profile.string()),std::runtime_error);
  p["maximum_translation_m"]=.03;p["maximum_speed_m_s"]=".nan";write();
  EXPECT_THROW(loadSharedRootOptions(profile.string()),std::runtime_error);
}
TEST_F(SharedRootOptionsMutation, EnabledProfileRequiresAwareConsumerAndValidFullContract) {
  node["shared_root"]["enabled"]=true;write();
  EXPECT_THROW(loadConfig(profile.string()),std::runtime_error);
  const auto config=loadConfig(profile.string(),ConfigConsumer::kSharedRootAware);
  EXPECT_EQ(config.shared_root_profile_path,std::filesystem::canonical(profile).string());
  node["shared_root"]["morphology"]["window_frames"]=129;write();
  EXPECT_THROW(loadConfig(profile.string(),ConfigConsumer::kSharedRootAware),std::runtime_error);
}
TEST_F(SharedRootOptionsMutation, DisabledAwareConsumerDoesNotAccessArtifacts) {
  node["shared_root"]["input_contract_artifact"]="nonexistent";write();
  const auto config=loadConfig(profile.string(),ConfigConsumer::kSharedRootAware);
  EXPECT_TRUE(config.shared_root_profile_path.empty());
}
TEST_F(SharedRootOptionsMutation, RejectsMixUnknownNonfiniteAndInvalidWindows) {
  auto c=node["shared_root"];c["mix"]=.5;write();
  EXPECT_THROW(loadSharedRootOptions(profile.string()),std::runtime_error);
  c.remove("mix");c["morphology"]["window_frames"]=129;write();
  EXPECT_THROW(loadSharedRootOptions(profile.string()),std::runtime_error);
  c["morphology"]["window_frames"]=31;c["target_gate"]["maximum_center_offset_from_robot_shoulders_m"]=".nan";write();
  EXPECT_THROW(loadSharedRootOptions(profile.string()),std::runtime_error);
}
TEST_F(SharedRootOptionsMutation, TamperedGeometryCannotRelaxEvidenceChecks) {
  auto geometry=YAML::LoadFile(TIANJI_PROJECT_SOURCE_DIR "/config/shared_root_robot_geometry_dls.yaml");
  geometry["robot_geometry"]["cross_model_position_tolerance_m"]=10;
  const auto artifact=directory/"geometry.yaml";
  {std::ofstream stream(artifact);stream<<geometry;}
  node["shared_root"]["robot_geometry_artifact"]=artifact.string();write();
  EXPECT_THROW(loadSharedRootOptions(profile.string()),std::runtime_error);
}
TEST_F(SharedRootOptionsMutation, RejectsDuplicateConfigurationKeys) {
  write();
  {std::ofstream stream(profile,std::ios::app);stream<<"\nshared_root: {}\n";}
  EXPECT_THROW(loadSharedRootOptions(profile.string()),std::runtime_error);
}
} // namespace tianji_qp_ik
