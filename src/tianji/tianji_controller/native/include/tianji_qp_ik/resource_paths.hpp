#pragma once

#include <cstdlib>
#include <filesystem>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace tianji_qp_ik {

inline std::filesystem::path runtimeWorkspace() {
  namespace fs = std::filesystem;
  if (const char* declared = std::getenv("TIANJI_WORKSPACE")) {
    const fs::path root(declared);
    if (root.is_absolute() && fs::is_regular_file(root / "pixi.toml")) return fs::canonical(root);
    throw std::runtime_error("invalid TIANJI_WORKSPACE: " + root.string());
  }
  throw std::runtime_error("set TIANJI_WORKSPACE to the absolute Tianji workspace path");
}

inline std::filesystem::path runtimePackageResource(
    const std::string& package, const std::filesystem::path& relative) {
  namespace fs = std::filesystem;
  const char* environment = std::getenv("TIANJI_ENVIRONMENT");
  const auto install = runtimeWorkspace() / "install" / (environment ? environment : "default");
  // colcon supports both merged and per-package (isolated) install prefixes.
  std::vector<fs::path> prefixes{install, install / package};
  if (const char* declared = std::getenv("AMENT_PREFIX_PATH")) {
    std::istringstream stream(declared);
    std::string prefix;
    while (std::getline(stream, prefix, ':'))
      if (!prefix.empty()) prefixes.emplace_back(prefix);
  }
  for (const auto& prefix : prefixes) {
    const auto share = prefix / "share" / package;
    if (!fs::is_directory(share)) continue;
    const auto target = share / relative;
    if (fs::exists(target)) return fs::canonical(target);
    throw std::runtime_error("installed resource missing: " + target.string());
  }
  throw std::runtime_error("package " + package + " is not built; run pixi run build");
}

// Preserve explicit and config-local paths. Only the migrated description
// package reference has a second, installed meaning; arbitrary missing custom
// files must never be replaced by a similarly named bundled model.
inline std::filesystem::path controllerResource(
    const std::filesystem::path& profile, const std::filesystem::path& value) {
  namespace fs = std::filesystem;
  if (value.is_absolute()) return value;
  const auto local = fs::absolute(profile).parent_path() / value;
  if (fs::exists(local)) return fs::canonical(local);
  const std::string reference = value.generic_string();
  const std::string description = "../../../tianji_description/";
  if (reference.compare(0, description.size(), description) == 0)
    return runtimePackageResource("tianji_description", reference.substr(description.size()));
  return local.lexically_normal();
}

}  // namespace tianji_qp_ik
