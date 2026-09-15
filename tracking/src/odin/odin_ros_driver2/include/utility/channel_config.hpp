#pragma once

#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

namespace odin_ros_driver {

struct ChannelEnableConfig {
  bool raw_point{false};
  bool slam_point{false};
  bool image0{false};
  bool image1{false};
  bool imu{false};
  bool odom{false};
};

inline bool required_channel_switch(
  const YAML::Node & keys, const std::string & name)
{
  const YAML::Node value = keys[name];
  if (!value || !value.IsScalar()) {
    throw std::runtime_error("missing or non-scalar channel key: " + name);
  }
  int parsed = 0;
  try {
    parsed = value.as<int>();
  } catch (const YAML::Exception & error) {
    throw std::runtime_error("invalid channel key " + name + ": " + error.what());
  }
  if (parsed != 0 && parsed != 1) {
    throw std::runtime_error("channel key must be 0 or 1: " + name);
  }
  return parsed == 1;
}

inline ChannelEnableConfig parse_channel_enable_config(const YAML::Node & root) {
  const YAML::Node keys = root["register_keys"];
  if (!keys || !keys.IsMap()) {
    throw std::runtime_error("channel config requires a register_keys map");
  }
  return ChannelEnableConfig{
    required_channel_switch(keys, "enable_raw_point"),
    required_channel_switch(keys, "enable_slam_point"),
    required_channel_switch(keys, "enable_image0"),
    required_channel_switch(keys, "enable_image1"),
    required_channel_switch(keys, "enable_imu"),
    required_channel_switch(keys, "enable_odom")};
}

inline ChannelEnableConfig load_channel_enable_config(const std::string & path) {
  if (path.empty()) {
    throw std::runtime_error("channel config path must not be empty");
  }
  try {
    return parse_channel_enable_config(YAML::LoadFile(path));
  } catch (const std::runtime_error &) {
    throw;
  } catch (const std::exception & error) {
    throw std::runtime_error(
      "failed to load channel config " + path + ": " + error.what());
  }
}

inline std::string resolve_channel_config_path(
  const std::string & selected_path, const std::string & package_share)
{
  if (selected_path.empty()) {
    throw std::runtime_error("channel_config_file must not be empty");
  }
  if (selected_path.front() == '/') return selected_path;
  if (package_share.empty()) {
    throw std::runtime_error("Odin driver package share path is unavailable");
  }
  return package_share + "/config/" + selected_path;
}

}  // namespace odin_ros_driver
