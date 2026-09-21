// 1. Parameters loaded in program but not set in YAML: warn and use default values
// 2. Parameter type errors: warn, use default values, and indicate error location
// 3. Empty parameters: warn, use default values, and indicate error location

#pragma once

#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <memory>
#include <map>
#include <cctype>

class YAMLConfigLoader {
 public:
  // Constructor with optional config file loading
  YAMLConfigLoader(const std::string& config_file = "") {
    if (!config_file.empty()) {
      loadConfig(config_file);
    }
  }

  // Load config from file
  bool loadConfig(const std::string& config_file) {
    try {
      root_node = YAML::LoadFile(config_file);
      return true;
    } catch (const std::exception& e) {
      std::cerr << "[YAMLConfigLoader] Failed to load config file: " << e.what() << std::endl;
      return false;
    }
  }

  // Load config from yaml string content
  bool loadFromString(const std::string& yaml_content) {
    try {
      root_node = YAML::Load(yaml_content);
      return true;
    } catch (const std::exception& e) {
      std::cerr << "[YAMLConfigLoader] Failed to parse yaml string: " << e.what() << std::endl;
      return false;
    }
  }

  // Get info about all loaded parameters
  std::string getInfo() const {
    std::stringstream ss;
    ss << "=== Loaded Configuration Parameters ===" << std::endl;

    for (const auto& entry : loaded_params) {
      ss << entry.first << ": " << entry.second << std::endl;
    }

    return ss.str();
  }

  template <typename T>
  T getValue(const std::string& key, const T& default_value) const {
    if (!root_node.IsMap()) {
      std::cout << "[YAMLConfigLoader] Warning: Root node is not a map" << std::endl;
      recordParameter(key, default_value, true);
      return default_value;
    }

    T result = key.find('.') != std::string::npos
                   ? getNestedValue<T>(key, default_value)
                   : getValueWithDefault(root_node, key, default_value, key);

    recordParameter(key, result, false);
    return result;
  }

  template <typename T>
  std::vector<T> getVector(const std::string& key) const {
    if (!root_node.IsMap()) {
      std::cout << "[YAMLConfigLoader] Warning: Root node is not a map" << std::endl;
      recordParameter(key, std::vector<T>(), true);
      return std::vector<T>();
    }

    std::vector<T> result = key.find('.') != std::string::npos
                                ? getNestedVector<T>(key)
                                : getVectorFromNode<T>(root_node, key, key);

    recordParameter(key, result, false);
    return result;
  }

 private:
  template <typename T>
  std::vector<T> parseVectorFromScalar(const std::string& s) const {
    std::string cleaned;
    cleaned.reserve(s.size());

    auto is_num_char = [](unsigned char c) {
      return (std::isdigit(c) != 0) || c == '.' || c == '-' || c == '+' || c == 'e' || c == 'E';
    };

    for (char ch : s) {
      cleaned.push_back(is_num_char(static_cast<unsigned char>(ch)) ? ch : ' ');
    }

    std::istringstream iss(cleaned);
    std::vector<T> out;
    T v;
    while (iss >> v) {
      out.push_back(v);
    }
    return out;
  }

  template <typename T>
  std::vector<T> parseVectorFromNodeDump(const YAML::Node& node) const {
    return parseVectorFromScalar<T>(YAML::Dump(node));
  }

  // Record parameter with its value
  template <typename T>
  void recordParameter(const std::string& key, const T& value, bool is_default) const {
    std::stringstream ss;
    ss << value;

    if (is_default) {
      ss << " (default)";
    }

    loaded_params[key] = ss.str();
  }

  // Specialization for string
  void recordParameter(const std::string& key, const std::string& value, bool is_default) const {
    std::stringstream ss;
    ss << "\"" << value << "\"";

    if (is_default) {
      ss << " (default)";
    }

    loaded_params[key] = ss.str();
  }

  // Specialization for bool
  void recordParameter(const std::string& key, const bool& value, bool is_default) const {
    std::stringstream ss;
    ss << (value ? "true" : "false");

    if (is_default) {
      ss << " (default)";
    }

    loaded_params[key] = ss.str();
  }

  // Specialization for vector<double>
  void recordParameter(const std::string& key, const std::vector<double>& value,
                       bool is_default) const {
    std::stringstream ss;
    ss << "[";
    for (size_t i = 0; i < value.size(); ++i) {
      ss << value[i];
      if (i < value.size() - 1) ss << ", ";
    }
    ss << "]";

    if (is_default) {
      ss << " (default)";
    }

    loaded_params[key] = ss.str();
  }

  // Specialization for vector<int>
  void recordParameter(const std::string& key, const std::vector<int>& value,
                       bool is_default) const {
    std::stringstream ss;
    ss << "[";
    for (size_t i = 0; i < value.size(); ++i) {
      ss << value[i];
      if (i < value.size() - 1) ss << ", ";
    }
    ss << "]";

    if (is_default) {
      ss << " (default)";
    }

    loaded_params[key] = ss.str();
  }

  template <typename T>
  T getValueWithDefault(const YAML::Node& node, const std::string& key, const T& default_value,
                        const std::string& full_path) const {
    if (!node[key]) {
      std::cout << "[YAMLConfigLoader] Warning: Parameter " << full_path
                << " not found, using default value" << std::endl;
      return default_value;
    }

    try {
      return node[key].as<T>();
    } catch (const YAML::Exception& e) {
      std::cout << "[YAMLConfigLoader] Warning: Parameter " << full_path
                << " type conversion failed: " << e.what() << ", using default value" << std::endl;
      return default_value;
    }
  }

  template <typename T>
  std::vector<T> getVectorFromNode(const YAML::Node& node, const std::string& key,
                                   const std::string& full_path) const {
    if (!node[key]) {
      std::cout << "[YAMLConfigLoader] Warning: Vector parameter " << full_path
                << " not found, returning empty vector" << std::endl;
      return std::vector<T>();
    }
    if (node[key].IsSequence()) {
      return node[key].as<std::vector<T>>();
    }
    if (node[key].IsScalar()) {
      return parseVectorFromScalar<T>(node[key].as<std::string>());
    }
    return parseVectorFromNodeDump<T>(node[key]);
  }

  template <typename T>
  T getNestedValue(const std::string& key_path, const T& default_value) const {
    std::vector<std::string> keys = splitKey(key_path);
    return getNestedValueRecursive(root_node, keys, 0, default_value, key_path);
  }

  template <typename T>
  T getNestedValueRecursive(const YAML::Node& node, const std::vector<std::string>& keys,
                            size_t index, const T& default_value,
                            const std::string& full_path) const {
    if (index >= keys.size()) return default_value;

    const std::string& current_key = keys[index];

    if (!node[current_key]) {
      std::cout << "[YAMLConfigLoader] Warning: Node " << full_path
                << " not found, using default value" << std::endl;
      return default_value;
    }

    // If this is the last key, return the value
    if (index == keys.size() - 1) {
      return getValueWithDefault(node, current_key, default_value, full_path);
    }

    // If not the last key, check if it's a Map
    if (!node[current_key].IsMap()) {
      std::cout << "[YAMLConfigLoader] Warning: Node " << full_path
                << " is not a map, using default value" << std::endl;
      return default_value;
    }

    // Recursively navigate to next level
    return getNestedValueRecursive<T>(node[current_key], keys, index + 1, default_value, full_path);
  }

  template <typename T>
  std::vector<T> getNestedVector(const std::string& key_path) const {
    std::vector<std::string> keys = splitKey(key_path);
    return getNestedVectorRecursive<T>(root_node, keys, 0, key_path);
  }

  template <typename T>
  std::vector<T> getNestedVectorRecursive(const YAML::Node& node,
                                          const std::vector<std::string>& keys, size_t index,
                                          const std::string& full_path) const {
    if (index >= keys.size()) return std::vector<T>();

    const std::string& current_key = keys[index];

    if (!node[current_key]) {
      std::cout << "[YAMLConfigLoader] Warning: Node " << full_path
                << " not found, returning empty vector" << std::endl;
      return std::vector<T>();
    }

    // If this is the last key, return the vector
    if (index == keys.size() - 1) {
      if (node[current_key].IsSequence()) {
        return node[current_key].as<std::vector<T>>();
      }
      if (node[current_key].IsScalar()) {
        return parseVectorFromScalar<T>(node[current_key].as<std::string>());
      }
      return parseVectorFromNodeDump<T>(node[current_key]);
    }

    // If not the last key, check if it's a Map
    if (!node[current_key].IsMap()) {
      std::cout << "[YAMLConfigLoader] Warning: Node " << full_path
                << " is not a map, returning empty vector" << std::endl;
      return std::vector<T>();
    }

    // Recursively navigate to next level
    return getNestedVectorRecursive<T>(node[current_key], keys, index + 1, full_path);
  }

  std::vector<std::string> splitKey(const std::string& key_path) const {
    std::vector<std::string> keys;
    std::stringstream ss(key_path);
    std::string key;

    while (std::getline(ss, key, '.')) {
      keys.push_back(key);
    }

    return keys;
  }

  std::string buildKeyPath(const std::vector<std::string>& keys, size_t end_index) const {
    std::string path;
    for (size_t i = 0; i <= end_index && i < keys.size(); ++i) {
      if (i > 0) path += ".";
      path += keys[i];
    }
    return path;
  }

  YAML::Node root_node;
  mutable std::map<std::string, std::string>
      loaded_params;  // Store loaded parameters and their values
};

typedef std::shared_ptr<YAMLConfigLoader> ConfigLoaderPtr;

namespace yaml_utils {

// Helper to safely get YAML value with default (only warns if key missing, does not catch type
// errors)
template <typename T>
inline T safeGet(const YAML::Node& node, const std::string& key, const T& default_val,
                 const std::string& prefix = "") {
  if (!node[key]) {
    std::cerr << prefix << "Warning: key '" << key << "' not found, using default" << std::endl;
    return default_val;
  }
  return node[key].as<T>();  // Let type conversion errors propagate
}

// Helper to get vector from YAML with size validation
template <typename T>
inline bool safeGetVector(const YAML::Node& node, const std::string& key, std::vector<T>& out,
                          size_t expected_size, const std::string& prefix = "") {
  if (!node[key]) {
    std::cerr << prefix << "Warning: key '" << key << "' not found" << std::endl;
    return false;
  }
  out = node[key].as<std::vector<T>>();
  if (out.size() < expected_size) {
    std::cerr << prefix << "Warning: '" << key << "' has " << out.size() << " elements, expected "
              << expected_size << std::endl;
    return false;
  }
  return true;
}

// Resolve camera YAML node key by resolution.
// Priority: 1) cam_{id}_{width}_{height}  (exact match)
//           2) cam_{id}                    (old format fallback)
//           3) first cam_{id}_*            (any resolution, last resort)
// Returns empty string only if no matching key exists at all.
inline std::string ResolveCamKey(const YAML::Node& root, int cam_id, int width, int height) {
  std::string id_str = std::to_string(cam_id);

  // 1) Exact resolution match
  if (width > 0 && height > 0) {
    std::string exact = "cam_" + id_str + "_" + std::to_string(width) + "_" + std::to_string(height);
    if (root[exact]) return exact;
  }

  // 2) Generic key (old format)
  std::string generic = "cam_" + id_str;
  if (root[generic]) return generic;

  // 3) Any cam_{id}_* key as last resort
  std::string prefix = "cam_" + id_str + "_";
  for (auto it = root.begin(); it != root.end(); ++it) {
    std::string key = it->first.as<std::string>("");
    if (key.rfind(prefix, 0) == 0) {
      std::cerr << "[ResolveCamKey] Warning: no calibration for cam_" << cam_id;
      if (width > 0 && height > 0)
        std::cerr << " at " << width << "x" << height;
      std::cerr << ", using default key '" << key << "'" << std::endl;
      return key;
    }
  }

  return "";
}

}  // namespace yaml_utils