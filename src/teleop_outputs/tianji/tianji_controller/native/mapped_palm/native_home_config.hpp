#pragma once
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <stdexcept>
#include <string>

// Deployment Home override only; reference algorithm configuration stays intact.
template<class Config>
void apply_deployment_home(Config& config, const std::string& path) {
  if (path.empty()) return;
  const auto node = YAML::LoadFile(path);
  const auto left = node["left_home_rad"], right = node["right_home_rad"];
  for (const auto& side : {left, right}) {
    if (!side.IsSequence() || side.size() != 7)
      throw std::invalid_argument("Home requires seven joints per side");
    for (const auto& value : side)
      if (!std::isfinite(value.template as<double>()))
        throw std::invalid_argument("Home must be finite");
  }
  for (int i=0;i<7;++i) {
    config.controller.initial_left_q_rad[i]=left[i].as<double>();
    config.controller.initial_right_q_rad[i]=right[i].as<double>();
  }
  config.controller.initial_posture_enabled=true;
  // Native cycle construction separately validates against the selected URDF.
}
