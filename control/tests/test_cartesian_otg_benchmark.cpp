#include <gtest/gtest.h>

#include <cstdlib>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <vector>

namespace {

std::vector<std::string> split(const std::string& row) {
  std::vector<std::string> fields;
  std::stringstream stream(row);
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}

TEST(CartesianOtgBenchmark, BothControllersStayFiniteAndRespectHardBounds) {
  const std::filesystem::path output =
      std::filesystem::temp_directory_path() /
      ("tianji_otg_benchmark_test_" + std::to_string(::getpid()) + ".csv");
  const std::string command =
      std::string(TIANJI_PROJECT_BINARY_DIR) +
      "/tianji_cartesian_otg_benchmark --config " +
      std::string(TIANJI_PROJECT_SOURCE_DIR) +
      "/config/qp_ik_cartesian_otg_acceleration.yaml --model " +
      std::string(TIANJI_PROJECT_SOURCE_DIR) +
      "/models/marvin_m6_qp_test.xml --steps 80 --output " + output.string();
  ASSERT_EQ(std::system(command.c_str()), 0);

  std::ifstream input(output);
  ASSERT_TRUE(input.good());
  std::string row;
  ASSERT_TRUE(static_cast<bool>(std::getline(input, row)));
  int rows = 0;
  bool saw_direct = false;
  bool saw_otg = false;
  bool saw_acceleration = false;
  while (std::getline(input, row)) {
    const std::vector<std::string> fields = split(row);
    ASSERT_EQ(fields.size(), 26U) << row;
    saw_direct = saw_direct || fields[0] == "direct_velocity_qp";
    saw_otg = saw_otg || fields[0] == "otg_velocity_qp";
    saw_acceleration = saw_acceleration ||
                       fields[0] == "otg_acceleration_qp";
    if (fields[0] == "otg_acceleration_qp") {
      EXPECT_LE(std::stod(fields[16]), 6000.0 + 1e-6) << row;
    } else {
      EXPECT_LE(std::stod(fields[14]), 60.0 + 1e-6) << row;
    }
    EXPECT_EQ(fields[23], "0") << row;
    EXPECT_EQ(fields[24], "1") << row;
    EXPECT_EQ(fields[25], "1") << row;
    ++rows;
  }
  EXPECT_EQ(rows, 36);
  EXPECT_TRUE(saw_direct);
  EXPECT_TRUE(saw_otg);
  EXPECT_TRUE(saw_acceleration);
  std::filesystem::remove(output);
}

TEST(CartesianOtgBenchmark,
     CircleSupportsFixedAndDynamicConsistentPicoOutwardProfiles) {
  const std::filesystem::path output =
      std::filesystem::temp_directory_path() /
      ("tianji_current_circle_benchmark_test_" +
       std::to_string(::getpid()) + ".csv");
  const std::string command =
      std::string(TIANJI_PROJECT_BINARY_DIR) +
      "/tianji_cartesian_otg_benchmark --config " +
      std::string(TIANJI_PROJECT_SOURCE_DIR) +
      "/config/qp_ik_pico_teleop.yaml --model " +
      std::string(TIANJI_PROJECT_SOURCE_DIR) +
      "/models/marvin_m6_qp_pico_fast.xml --steps 80 --scenario circle "
      "--arm-angle-profile both --output " + output.string();
  ASSERT_EQ(std::system(command.c_str()), 0);

  std::ifstream input(output);
  ASSERT_TRUE(input.good());
  std::string row;
  ASSERT_TRUE(static_cast<bool>(std::getline(input, row)));
  const std::vector<std::string> header = split(row);
  const auto column = [&header](const std::string& name) {
    return std::find(header.begin(), header.end(), name);
  };
  EXPECT_NE(column("arm_angle_profile"), header.end());
  EXPECT_NE(column("left_arm_angle_rms_rad"), header.end());
  EXPECT_NE(column("right_arm_angle_p95_rad"), header.end());
  EXPECT_NE(column("left_outward_min_m"), header.end());
  EXPECT_NE(column("right_outward_active_count"), header.end());

  int rows = 0;
  int fixed_rows = 0;
  int dynamic_rows = 0;
  while (std::getline(input, row)) {
    const std::vector<std::string> fields = split(row);
    ASSERT_EQ(fields.size(), header.size()) << row;
    const auto profile_index = static_cast<std::size_t>(
        std::distance(header.begin(), column("arm_angle_profile")));
    const auto scenario_index = static_cast<std::size_t>(
        std::distance(header.begin(), column("scenario")));
    EXPECT_EQ(fields[scenario_index], "circle");
    fixed_rows += fields[profile_index] == "fixed" ? 1 : 0;
    dynamic_rows += fields[profile_index] == "dynamic_consistent" ? 1 : 0;
    ++rows;
  }
  EXPECT_EQ(rows, 6);
  EXPECT_EQ(fixed_rows, 3);
  EXPECT_EQ(dynamic_rows, 3);
  std::filesystem::remove(output);
}

}  // namespace
