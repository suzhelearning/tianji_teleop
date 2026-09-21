#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "pico_odin/extrinsics_file.hpp"

namespace po = pico_odin;

namespace {

std::string read_all(const std::filesystem::path & path) {
  std::ifstream input(path);
  std::ostringstream contents;
  contents << input.rdbuf();
  return contents.str();
}

po::ExtrinsicsDocument valid_document() {
  po::ExtrinsicsDocument document;
  document.valid = true;
  document.calibrated_at = "2026-07-21T12:00:00Z";
  document.pico_topic = "/pico/smpl";
  document.odin_topic = "/raw/odom/odin_highfreq";
  document.pelvis_T_odin = po::Pose3{
    Eigen::Quaterniond(
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(0.12, Eigen::Vector3d::UnitY())),
    Eigen::Vector3d(-0.27, 0.01, 0.09)};
  document.metrics.sample_count = 180;
  document.metrics.time_correlation = 0.93;
  document.metrics.pitch_range_rad = 0.42;
  document.metrics.yaw_range_rad = 0.31;
  document.metrics.rotation_rms_rad = 0.012;
  document.metrics.translation_rms_m = 0.009;
  document.metrics.condition_number = 17.0;
  document.metrics.refinement_iterations = 4;
  return document;
}

}  // namespace

TEST(ExtrinsicsFile, RoundTripPreservesTransformAndQualityMetrics) {
  const auto root = std::filesystem::temp_directory_path() /
    "pico_odin_extrinsics_round_trip";
  std::filesystem::remove_all(root);
  const auto path = root / "odin_pelvis.yaml";
  const auto expected = valid_document();

  ASSERT_NO_THROW(po::save_extrinsics_atomic(path, expected));
  const auto actual = po::load_extrinsics(path);

  EXPECT_TRUE(actual.valid);
  EXPECT_EQ(actual.schema_version, 3);
  EXPECT_EQ(actual.pico_topic, "/pico/smpl");
  EXPECT_EQ(actual.odin_topic, "/raw/odom/odin_highfreq");
  EXPECT_EQ(actual.metrics.sample_count, 180);
  EXPECT_NEAR(actual.metrics.time_correlation, 0.93, 1e-12);
  EXPECT_EQ(actual.metrics.refinement_iterations, 4);
  EXPECT_LT(
    po::angular_distance(actual.pelvis_T_odin.rotation, expected.pelvis_T_odin.rotation),
    1e-10);
  EXPECT_LT(
    (actual.pelvis_T_odin.translation - expected.pelvis_T_odin.translation).norm(),
    1e-10);
  EXPECT_EQ(read_all(path).find("time_offset_sec"), std::string::npos);
  std::filesystem::remove_all(root);
}

TEST(ExtrinsicsFile, InvalidReplacementCannotOverwriteLastValidCalibration) {
  const auto root = std::filesystem::temp_directory_path() /
    "pico_odin_extrinsics_preserve";
  std::filesystem::remove_all(root);
  const auto path = root / "odin_pelvis.yaml";
  auto document = valid_document();
  po::save_extrinsics_atomic(path, document);
  const std::string previous = read_all(path);

  document.pelvis_T_odin.translation.x() =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(po::save_extrinsics_atomic(path, document), std::runtime_error);
  EXPECT_EQ(read_all(path), previous);
  EXPECT_TRUE(po::load_extrinsics(path).valid);
  std::filesystem::remove_all(root);
}

TEST(ExtrinsicsFile, LoaderRejectsWrongTransformConvention) {
  const auto root = std::filesystem::temp_directory_path() /
    "pico_odin_extrinsics_convention";
  std::filesystem::remove_all(root);
  std::filesystem::create_directories(root);
  const auto path = root / "odin_pelvis.yaml";
  std::ofstream output(path);
  output << "schema_version: 1\nvalid: true\ntransform_convention: wrong\n";
  output.close();

  EXPECT_THROW(po::load_extrinsics(path), std::runtime_error);
  std::filesystem::remove_all(root);
}

TEST(ExtrinsicsFile, LoadsAndUpgradesLegacySchemaOneCalibration) {
  const auto root = std::filesystem::temp_directory_path() /
    "pico_odin_extrinsics_schema_one";
  std::filesystem::remove_all(root);
  std::filesystem::create_directories(root);
  const auto path = root / "odin_pelvis.yaml";
  std::ofstream output(path);
  output << R"(schema_version: 1
valid: true
transform_convention: T_pelvis_odin
quaternion_order: xyzw
calibrated_at: 2026-07-21T12:00:00Z
translation_m: {x: -0.27, y: 0.01, z: 0.09}
quaternion_xyzw: {x: 0.0, y: 0.0, z: 1.0, w: 0.0}
matrix_row_major: [-1.0, 0.0, 0.0, -0.27, 0.0, -1.0, 0.0, 0.01, 0.0, 0.0, 1.0, 0.09, 0.0, 0.0, 0.0, 1.0]
quality:
  sample_count: 180
  time_offset_sec: 0.024
  pitch_range_rad: 0.42
  yaw_range_rad: 0.31
  rotation_rms_rad: 0.012
  translation_rms_m: 0.009
  condition_number: 17.0
)";
  output.close();

  const auto document = po::load_extrinsics(path);

  EXPECT_EQ(document.schema_version, 3);
  EXPECT_EQ(document.pico_topic, "/pico/smpl");
  EXPECT_EQ(document.odin_topic, "/raw/odom/odin_highfreq");
  EXPECT_DOUBLE_EQ(document.metrics.time_correlation, 0.0);
  EXPECT_EQ(document.metrics.refinement_iterations, 0);
  std::filesystem::remove_all(root);
}

TEST(ExtrinsicsFile, LoadsSchemaTwoAndDiscardsPersistedTimingOffset) {
  const auto root = std::filesystem::temp_directory_path() /
    "pico_odin_extrinsics_schema_two";
  std::filesystem::remove_all(root);
  std::filesystem::create_directories(root);
  const auto path = root / "odin_pelvis.yaml";
  std::ofstream output(path);
  output << R"(schema_version: 2
valid: true
transform_convention: T_pelvis_odin
quaternion_order: xyzw
calibrated_at: 2026-07-21T12:00:00Z
source_topics: {pico: /pico/smpl, odin: /raw/odom/odin_highfreq}
translation_m: {x: -0.27, y: 0.01, z: 0.09}
quaternion_xyzw: {x: 0.0, y: 0.0, z: 1.0, w: 0.0}
matrix_row_major: [-1.0, 0.0, 0.0, -0.27, 0.0, -1.0, 0.0, 0.01, 0.0, 0.0, 1.0, 0.09, 0.0, 0.0, 0.0, 1.0]
quality:
  sample_count: 180
  time_offset_sec: 1234.5
  time_correlation: 0.93
  pitch_range_rad: 0.42
  yaw_range_rad: 0.31
  rotation_rms_rad: 0.012
  translation_rms_m: 0.009
  condition_number: 17.0
  refinement_iterations: 4
)";
  output.close();

  const auto document = po::load_extrinsics(path);

  EXPECT_EQ(document.schema_version, 3);
  EXPECT_EQ(document.pico_topic, "/pico/smpl");
  EXPECT_EQ(document.odin_topic, "/raw/odom/odin_highfreq");
  EXPECT_DOUBLE_EQ(document.metrics.time_correlation, 0.93);
  EXPECT_EQ(document.metrics.refinement_iterations, 4);
  std::filesystem::remove_all(root);
}
