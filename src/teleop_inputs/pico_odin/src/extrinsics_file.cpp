#include "pico_odin/extrinsics_file.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <system_error>

#include <yaml-cpp/yaml.h>

namespace pico_odin {
namespace {

constexpr const char * kConvention = "T_pelvis_odin";

bool finite_metrics(const CalibrationMetrics & metrics) {
  return metrics.sample_count >= 0 &&
         metrics.refinement_iterations >= 0 &&
         std::isfinite(metrics.time_correlation) &&
         std::isfinite(metrics.pitch_range_rad) &&
         std::isfinite(metrics.yaw_range_rad) &&
         std::isfinite(metrics.rotation_rms_rad) &&
         std::isfinite(metrics.translation_rms_m) &&
         std::isfinite(metrics.condition_number);
}

void validate(const ExtrinsicsDocument & document) {
  if (document.schema_version != 3) {
    throw std::runtime_error("unsupported extrinsics schema version");
  }
  if (!document.valid) {
    throw std::runtime_error("extrinsics document is not marked valid");
  }
  if (!finite(document.pelvis_T_odin)) {
    throw std::runtime_error("extrinsics transform contains non-finite values");
  }
  if (!finite_metrics(document.metrics)) {
    throw std::runtime_error("extrinsics quality metrics contain invalid values");
  }
  if (document.pico_topic.empty() || document.odin_topic.empty()) {
    throw std::runtime_error("extrinsics source topics are missing");
  }
}

Eigen::Matrix4d matrix_of(const Pose3 & pose) {
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  matrix.block<3, 3>(0, 0) = normalized(pose.rotation).toRotationMatrix();
  matrix.block<3, 1>(0, 3) = pose.translation;
  return matrix;
}

YAML::Node encode(const ExtrinsicsDocument & document) {
  validate(document);
  const Eigen::Quaterniond quaternion = normalized(document.pelvis_T_odin.rotation);
  const Eigen::Matrix4d matrix = matrix_of(document.pelvis_T_odin);

  YAML::Node root;
  root["schema_version"] = document.schema_version;
  root["valid"] = document.valid;
  root["transform_convention"] = kConvention;
  root["quaternion_order"] = "xyzw";
  root["calibrated_at"] = document.calibrated_at;
  auto source_topics = root["source_topics"];
  source_topics["pico"] = document.pico_topic;
  source_topics["odin"] = document.odin_topic;

  auto translation = root["translation_m"];
  translation["x"] = document.pelvis_T_odin.translation.x();
  translation["y"] = document.pelvis_T_odin.translation.y();
  translation["z"] = document.pelvis_T_odin.translation.z();

  auto orientation = root["quaternion_xyzw"];
  orientation["x"] = quaternion.x();
  orientation["y"] = quaternion.y();
  orientation["z"] = quaternion.z();
  orientation["w"] = quaternion.w();

  YAML::Node matrix_node(YAML::NodeType::Sequence);
  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < 4; ++column) {
      matrix_node.push_back(matrix(row, column));
    }
  }
  root["matrix_row_major"] = matrix_node;

  auto quality = root["quality"];
  quality["sample_count"] = document.metrics.sample_count;
  quality["time_correlation"] = document.metrics.time_correlation;
  quality["pitch_range_rad"] = document.metrics.pitch_range_rad;
  quality["yaw_range_rad"] = document.metrics.yaw_range_rad;
  quality["rotation_rms_rad"] = document.metrics.rotation_rms_rad;
  quality["translation_rms_m"] = document.metrics.translation_rms_m;
  quality["condition_number"] = document.metrics.condition_number;
  quality["refinement_iterations"] = document.metrics.refinement_iterations;
  return root;
}

double required_double(const YAML::Node & node, const char * key) {
  if (!node || !node[key]) {
    throw std::runtime_error(std::string("missing YAML field: ") + key);
  }
  return node[key].as<double>();
}

}  // namespace

std::filesystem::path default_extrinsics_path() {
  if (const char * config = std::getenv("XDG_CONFIG_HOME"); config && config[0] != '\0') {
    return std::filesystem::path(config) / "pico_tracker" / "odin_pelvis_extrinsics.yaml";
  }
  if (const char * user_home = std::getenv("HOME"); user_home && user_home[0] != '\0') {
    return std::filesystem::path(user_home) / ".config" / "pico_tracker" /
           "odin_pelvis_extrinsics.yaml";
  }
  throw std::runtime_error("cannot resolve config directory: HOME and XDG_CONFIG_HOME are unset");
}

ExtrinsicsDocument load_extrinsics(const std::filesystem::path & path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path.string());
  } catch (const YAML::Exception & error) {
    throw std::runtime_error("failed to load extrinsics YAML: " + std::string(error.what()));
  }

  try {
    if (!root["transform_convention"] ||
        root["transform_convention"].as<std::string>() != kConvention) {
      throw std::runtime_error("unsupported transform convention");
    }
    if (!root["quaternion_order"] || root["quaternion_order"].as<std::string>() != "xyzw") {
      throw std::runtime_error("unsupported quaternion order");
    }

    const int file_schema_version = root["schema_version"].as<int>();
    if (file_schema_version != 1 && file_schema_version != 2 &&
        file_schema_version != 3) {
      throw std::runtime_error("unsupported extrinsics schema version");
    }

    ExtrinsicsDocument document;
    // Upgrade legacy files in memory. Schema 1 lacked provenance and schema 2
    // persisted a session-only device/transport timing offset. The spatial
    // installation transform remains valid, but that timing value is ignored.
    document.schema_version = 3;
    document.valid = root["valid"].as<bool>();
    document.calibrated_at = root["calibrated_at"] ? root["calibrated_at"].as<std::string>() : "";
    if (file_schema_version >= 2) {
      const auto source_topics = root["source_topics"];
      document.pico_topic = source_topics["pico"].as<std::string>();
      document.odin_topic = source_topics["odin"].as<std::string>();
    } else {
      document.pico_topic = "/pico/smpl";
      document.odin_topic = "/raw/odom/odin_highfreq";
    }

    const auto translation = root["translation_m"];
    const auto orientation = root["quaternion_xyzw"];
    document.pelvis_T_odin.translation = Eigen::Vector3d(
      required_double(translation, "x"), required_double(translation, "y"),
      required_double(translation, "z"));
    document.pelvis_T_odin.rotation = normalized(Eigen::Quaterniond(
      required_double(orientation, "w"), required_double(orientation, "x"),
      required_double(orientation, "y"), required_double(orientation, "z")));

    const auto quality = root["quality"];
    document.metrics.sample_count = quality["sample_count"].as<int>();
    if (file_schema_version <= 2) {
      static_cast<void>(required_double(quality, "time_offset_sec"));
    }
    document.metrics.time_correlation = file_schema_version >= 2 ?
      required_double(quality, "time_correlation") : 0.0;
    document.metrics.pitch_range_rad = required_double(quality, "pitch_range_rad");
    document.metrics.yaw_range_rad = required_double(quality, "yaw_range_rad");
    document.metrics.rotation_rms_rad = required_double(quality, "rotation_rms_rad");
    document.metrics.translation_rms_m = required_double(quality, "translation_rms_m");
    document.metrics.condition_number = required_double(quality, "condition_number");
    document.metrics.refinement_iterations = file_schema_version >= 2 ?
      quality["refinement_iterations"].as<int>() : 0;

    const auto matrix_node = root["matrix_row_major"];
    if (!matrix_node || !matrix_node.IsSequence() || matrix_node.size() != 16) {
      throw std::runtime_error("matrix_row_major must contain 16 values");
    }
    const Eigen::Matrix4d expected = matrix_of(document.pelvis_T_odin);
    for (int row = 0; row < 4; ++row) {
      for (int column = 0; column < 4; ++column) {
        const double saved = matrix_node[row * 4 + column].as<double>();
        if (!std::isfinite(saved) || std::abs(saved - expected(row, column)) > 1e-8) {
          throw std::runtime_error("matrix_row_major disagrees with transform fields");
        }
      }
    }
    validate(document);
    return document;
  } catch (const YAML::Exception & error) {
    throw std::runtime_error("invalid extrinsics YAML: " + std::string(error.what()));
  }
}

void save_extrinsics_atomic(
  const std::filesystem::path & path,
  const ExtrinsicsDocument & document)
{
  const YAML::Node root = encode(document);
  std::error_code error;
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path(), error);
    if (error) {
      throw std::runtime_error("failed to create extrinsics directory: " + error.message());
    }
  }

  std::filesystem::path temporary = path;
  temporary += ".tmp";
  {
    std::ofstream output(temporary, std::ios::trunc);
    if (!output) {
      throw std::runtime_error("failed to open temporary extrinsics file");
    }
    output << root << '\n';
    output.flush();
    if (!output) {
      std::filesystem::remove(temporary, error);
      throw std::runtime_error("failed to write temporary extrinsics file");
    }
  }

  try {
    static_cast<void>(load_extrinsics(temporary));
  } catch (...) {
    std::filesystem::remove(temporary, error);
    throw;
  }

  std::filesystem::rename(temporary, path, error);
  if (error) {
    std::filesystem::remove(temporary);
    throw std::runtime_error("failed to replace extrinsics file atomically: " + error.message());
  }
}

}  // namespace pico_odin
