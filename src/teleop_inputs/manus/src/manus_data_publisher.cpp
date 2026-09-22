// Acquisition lifecycle and raw semantic names adapted from wuji_teleop's
// manus_ros2/src/ManusDataPublisher.{cpp,hpp}.
// MIT License
// Copyright (c) 2025 Wuji Technology Co., Ltd.
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <ManusSDK.h>
#include <rclcpp/rclcpp.hpp>
#include <tianji_interfaces/msg/manus_glove.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <sys/socket.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

namespace {
using Glove = tianji_interfaces::msg::ManusGlove;
constexpr size_t kMaxNodes = 64;
constexpr const char * kSides[] = {"left", "right"};

int64_t monotonic_ns()
{
  timespec value{};
  if (clock_gettime(CLOCK_MONOTONIC, &value) != 0) {
    throw std::runtime_error("CLOCK_MONOTONIC unavailable");
  }
  return static_cast<int64_t>(value.tv_sec) * 1000000000LL + value.tv_nsec;
}

std::string identity(const char * path)
{
  std::ifstream file(path);
  std::string value;
  if (!(file >> value) || value.size() != 36) {
    throw std::runtime_error(std::string("Cannot read UUID identity: ") + path);
  }
  for (size_t i = 0; i < value.size(); ++i) {
    const bool dash = i == 8 || i == 13 || i == 18 || i == 23;
    if ((dash && value[i] != '-') ||
      (!dash && !((value[i] >= '0' && value[i] <= '9') ||
      (value[i] >= 'a' && value[i] <= 'f'))))
    {
      throw std::runtime_error("Invalid UUID identity");
    }
  }
  return value;
}

void checked(SDKReturnCode result, const char * operation)
{
  if (result != SDKReturnCode_Success) {
    throw std::runtime_error(std::string(operation) + " failed: " +
            std::to_string(static_cast<int>(result)));
  }
}

const char * chain_name(ChainType chain)
{
  switch (chain) {
    case ChainType_Hand: return "Hand";
    case ChainType_FingerThumb: return "Thumb";
    case ChainType_FingerIndex: return "Index";
    case ChainType_FingerMiddle: return "Middle";
    case ChainType_FingerRing: return "Ring";
    case ChainType_FingerPinky: return "Pinky";
    default: return nullptr;
  }
}

const char * joint_name(FingerJointType joint)
{
  switch (joint) {
    case FingerJointType_Metacarpal: return "MCP";
    case FingerJointType_Proximal: return "PIP";
    case FingerJointType_Intermediate: return "IP";
    case FingerJointType_Distal: return "DIP";
    case FingerJointType_Tip: return "TIP";
    default: return "Invalid";
  }
}

// IDs are opaque identifiers, NEVER array offsets. One rooted, acyclic hand
// tree and unique semantic labels are required before publishing any pose.
bool topology_valid(const NodeInfo * info, const SkeletonNode * nodes, size_t count, Side side)
{
  size_t root = count;
  for (size_t i = 0; i < count; ++i) {
    if (!chain_name(info[i].chainType) || info[i].side != side) {return false;}
    if (info[i].chainType == ChainType_Hand) {
      if (root != count) {return false;}
      root = i;
    } else if (std::strcmp(joint_name(info[i].fingerJointType), "Invalid") == 0) {
      return false;
    }
    for (size_t j = 0; j < i; ++j) {
      if (info[i].nodeId == info[j].nodeId || nodes[i].id == nodes[j].id ||
        (info[i].chainType == info[j].chainType &&
        info[i].fingerJointType == info[j].fingerJointType)) {return false;}
    }
    bool found = false;
    for (size_t j = 0; j < count; ++j) {found |= nodes[i].id == info[j].nodeId;}
    if (!found) {return false;}
    const auto & p = nodes[i].transform.position;
    const auto & q = nodes[i].transform.rotation;
    const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      std::abs(p.x) > 10.0 || std::abs(p.y) > 10.0 || std::abs(p.z) > 10.0 ||
      !std::isfinite(norm) || norm < 0.5 || norm > 1.5) {return false;}
  }
  if (root == count) {return false;}
  for (size_t i = 0; i < count; ++i) {
    if (i != root && info[root].parentId == info[i].nodeId) {return false;}
    size_t current = i;
    size_t depth = 0;
    while (current != root) {
      if (++depth >= count) {return false;}
      size_t parent = count;
      for (size_t j = 0; j < count; ++j) {
        if (info[j].nodeId == info[current].parentId) {parent = j; break;}
      }
      if (parent == count) {return false;}
      current = parent;
    }
  }
  return true;
}

class ManusPublisher final : public rclcpp::Node {
public:
  ManusPublisher() : Node("manus_data_publisher")
  {
    const auto directory = declare_parameter<std::string>("calibration.directory", "");
    const double max_age = declare_parameter<double>("max_source_age_s", 0.25);
    if (!std::isfinite(max_age) || max_age <= 0.0 || max_age > 60.0) {
      throw std::runtime_error("max_source_age_s must be finite and in (0, 60]");
    }
    max_age_ns_ = static_cast<int64_t>(max_age * 1e9);
    boot_id_ = identity("/proc/sys/kernel/random/boot_id");
    for (size_t i = 0; i < 2; ++i) {
      const auto filename = declare_parameter<std::string>(
        std::string("calibration.") + kSides[i] + "_file", "");
      if (filename.empty()) {throw std::runtime_error("Both calibration files must be supplied");}
      auto path = std::filesystem::path(filename);
      if (path.is_relative()) {path = std::filesystem::path(directory) / path;}
      if (!path.is_absolute()) {throw std::runtime_error("Calibration paths must resolve absolute");}
      std::ifstream input(path, std::ios::binary | std::ios::ate);
      const auto size = input.tellg();
      if (!input || size <= 0 || size > 16 * 1024 * 1024) {
        throw std::runtime_error("Missing, empty or oversized calibration: " + path.string());
      }
      calibration_[i].resize(static_cast<size_t>(size));
      input.seekg(0);
      if (!input.read(reinterpret_cast<char *>(calibration_[i].data()), size)) {
        throw std::runtime_error("Incomplete calibration read: " + path.string());
      }
      publishers_[i] = create_publisher<Glove>(std::string("/manus/raw/") + kSides[i],
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());
    }
  }

  ~ManusPublisher() override
  {
    stopping_.store(true);
    timer_.reset();
    if (sdk_attempted_) {
      const auto result = CoreSdk_ShutDown();
      if (result != SDKReturnCode_Success) {
        RCLCPP_ERROR(get_logger(), "MANUS SDK shutdown failed: %d", static_cast<int>(result));
      }
    }
    // Shutdown joins SDK callback threads before object storage is released.
    instance_.store(nullptr);
    if (owner_socket_ >= 0) {close(owner_socket_);}
  }

  void start()
  {
    // Linux abstract socket ownership is machine/network-namespace scoped, not
    // per ROS domain, source checkout, user, or replaceable lock-file inode.
    owner_socket_ = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (owner_socket_ < 0) {throw std::runtime_error("Cannot create MANUS SDK owner lock");}
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    constexpr char owner[] = "tianji.manus.integrated.sdk.owner";
    std::memcpy(address.sun_path + 1, owner, sizeof(owner) - 1);
    if (bind(owner_socket_, reinterpret_cast<sockaddr *>(&address),
      offsetof(sockaddr_un, sun_path) + sizeof(owner)) != 0)
    {
      throw std::runtime_error("MANUS Integrated SDK already owned (or owner lock unavailable)");
    }
    instance_.store(this);
    sdk_attempted_ = true;
    checked(CoreSdk_InitializeIntegrated(), "InitializeIntegrated");
    checked(CoreSdk_RegisterCallbackForLandscapeStream(on_landscape), "Register landscape callback");
    checked(CoreSdk_RegisterCallbackForRawSkeletonStream(on_raw), "Register raw skeleton callback");
    // Match the reference publisher's VUH contract explicitly. Its initializer
    // can overwrite the member's aggregate defaults; do not depend on that.
    CoordinateSystemVUH coordinates{};
    CoordinateSystemVUH_Init(&coordinates);
    coordinates.view = AxisView_XFromViewer;
    coordinates.up = AxisPolarity_PositiveZ;
    coordinates.handedness = Side_Right;
    coordinates.unitScale = 1.0f;
    checked(CoreSdk_InitializeCoordinateSystemWithVUH(coordinates, true), "Set VUH/world coordinates");
    checked(CoreSdk_LookForHosts(5, false), "LookForHosts");
    uint32_t count = 0;
    checked(CoreSdk_GetNumberOfAvailableHostsFound(&count), "Get host count");
    if (count != 1) {throw std::runtime_error("Expected exactly one Integrated SDK host");}
    ManusHost host{};
    checked(CoreSdk_GetAvailableHostsFound(&host, 1), "Get host");
    checked(CoreSdk_ConnectToHost(host), "ConnectToHost");
    checked(CoreSdk_SetRawSkeletonHandMotion(HandMotion_None), "Set raw hand motion");
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() {poll();});
    RCLCPP_INFO(get_logger(), "MANUS SDK connected; waiting for licensed landscape and calibration");
  }

private:
  struct SideState {
    uint32_t candidate = 0;
    uint64_t epoch = 0;
    bool configured = false;
    bool attempted = false;
    bool valid = false;
    uint64_t sequence = 0;
    uint64_t last_sdk_time = 0;
    int64_t configured_ns = 0;
    Glove last;
  };

  static void on_landscape(const Landscape * landscape) noexcept
  {
    auto * self = instance_.load();
    if (!self || self->stopping_.load()) {return;}
    try {self->landscape(landscape);} catch (const std::exception & error) {
      self->fail_callbacks(error.what());
    } catch (...) {self->fail_callbacks("Unknown landscape callback error");}
  }

  void landscape(const Landscape * landscape)
  {
    std::array<uint32_t, 2> candidates{};
    std::array<size_t, 2> counts{};
    bool acceptable = landscape && landscape->settings.license.integrated &&
      landscape->gloveDevices.gloveCount <= MAX_NUMBER_OF_GLOVES;
    if (acceptable) {
      for (uint32_t i = 0; i < landscape->gloveDevices.gloveCount; ++i) {
        const auto & glove = landscape->gloveDevices.gloves[i];
        if (glove.excluded) {continue;}
        if (!glove.id || (glove.side != Side_Left && glove.side != Side_Right)) {
          acceptable = false; break;
        }
        const size_t side = glove.side == Side_Left ? 0 : 1;
        candidates[side] = glove.id;
        ++counts[side];
      }
    }
    if (candidates[0] && candidates[0] == candidates[1]) {acceptable = false;}
    std::lock_guard<std::mutex> lock(mutex_);
    for (size_t i = 0; i < 2; ++i) {
      const uint32_t candidate = acceptable && counts[i] == 1 ? candidates[i] : 0;
      auto & state = sides_[i];
      if (state.candidate != candidate) {
        invalidate(i);
        ++state.epoch;
        state.candidate = candidate;
        state.configured = false;
        state.attempted = false;
      }
    }
  }

  static void on_raw(const SkeletonStreamInfo * stream) noexcept
  {
    auto * self = instance_.load();
    if (!self || self->stopping_.load()) {return;}
    try {
      // Timestamp arrival of THIS callback, before SDK reads, waits, or publish.
      const auto received_ns = monotonic_ns();
      self->raw(stream, received_ns);
    } catch (const std::exception & error) {self->fail_callbacks(error.what());}
    catch (...) {self->fail_callbacks("Unknown raw callback error");}
  }

  void raw(const SkeletonStreamInfo * stream, int64_t received_ns)
  {
    if (!stream || stream->skeletonsCount > MAX_NUMBER_OF_GLOVES) {
      fail_callbacks("Invalid raw stream bounds"); return;
    }
    std::array<RawSkeletonInfo, MAX_NUMBER_OF_GLOVES> skeletons{};
    for (uint32_t i = 0; i < stream->skeletonsCount; ++i) {
      if (CoreSdk_GetRawSkeletonInfo(i, &skeletons[i]) != SDKReturnCode_Success ||
        skeletons[i].gloveId == 0)
      {
        fail_callbacks("GetRawSkeletonInfo failed or invalid glove ID"); return;
      }
      for (uint32_t j = 0; j < i; ++j) {
        if (skeletons[i].gloveId == skeletons[j].gloveId) {
          fail_callbacks("Duplicate glove ID in raw stream"); return;
        }
      }
    }
    for (uint32_t index = 0; index < stream->skeletonsCount; ++index) {
      const auto & info = skeletons[index];
      size_t side = 2;
      uint64_t epoch = 0;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        for (size_t i = 0; i < 2; ++i) {
          if (sides_[i].configured && sides_[i].candidate == info.gloveId &&
            received_ns > sides_[i].configured_ns)
          {side = i; epoch = sides_[i].epoch; break;}
        }
      }
      if (side == 2) {continue;}
      std::array<SkeletonNode, kMaxNodes> nodes{};
      std::array<NodeInfo, kMaxNodes> hierarchy{};
      bool valid = info.nodesCount >= 21 && info.nodesCount <= kMaxNodes;
      if (valid) {
        valid = CoreSdk_GetRawSkeletonData(index, nodes.data(), info.nodesCount) == SDKReturnCode_Success &&
          CoreSdk_GetRawSkeletonNodeInfoArray(info.gloveId, hierarchy.data(), info.nodesCount) == SDKReturnCode_Success;
      }
      if (valid) {
        valid = topology_valid(hierarchy.data(), nodes.data(), info.nodesCount,
          side == 0 ? Side_Left : Side_Right);
      }
      const auto now = monotonic_ns();
      valid = valid && received_ns > 0 && received_ns <= now && now - received_ns <= max_age_ns_;
      std::lock_guard<std::mutex> lock(mutex_);
      auto & state = sides_[side];
      if (!state.configured || state.epoch != epoch) {continue;}
      // SDK publish time is opaque UTC/timecode, NOT our monotonic source time.
      // Use it only to reject repeated/backwards SDK frames within a session.
      if (stream->publishTime.time == 0 || stream->publishTime.time <= state.last_sdk_time ||
        state.sequence == std::numeric_limits<uint64_t>::max()) {valid = false;}
      if (stream->publishTime.time > state.last_sdk_time) {state.last_sdk_time = stream->publishTime.time;}
      if (state.sequence == std::numeric_limits<uint64_t>::max()) {
        invalidate(side); state.configured = false; continue;
      }
      ++state.sequence;
      if (!valid) {
        invalidate(side);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "MANUS %s raw frame rejected (SDK data, topology, bounds, or source time)",
          kSides[side]);
        continue;
      }
      Glove message;
      message.glove_id = info.gloveId;
      message.side = kSides[side];
      message.session_id = state.last.session_id;
      message.boot_id = boot_id_;
      message.sequence = state.sequence;
      message.source_monotonic_ns = received_ns;
      message.valid = true;
      message.raw_nodes.reserve(info.nodesCount);
      for (size_t n = 0; n < info.nodesCount; ++n) {
        const NodeInfo * metadata = nullptr;
        for (size_t j = 0; j < info.nodesCount; ++j) {
          if (hierarchy[j].nodeId == nodes[n].id) {metadata = &hierarchy[j]; break;}
        }
        tianji_interfaces::msg::ManusRawNode node;
        node.node_id = nodes[n].id;
        node.parent_node_id = metadata->parentId;
        node.chain_type = chain_name(metadata->chainType);
        node.joint_type = joint_name(metadata->fingerJointType);
        const auto & p = nodes[n].transform.position;
        const auto & q = nodes[n].transform.rotation;
        node.pose.position.x = p.x; node.pose.position.y = p.y; node.pose.position.z = p.z;
        node.pose.orientation.x = q.x; node.pose.orientation.y = q.y;
        node.pose.orientation.z = q.z; node.pose.orientation.w = q.w;
        message.raw_nodes.push_back(std::move(node));
      }
      publishers_[side]->publish(message);
      // Retain identity/time only, never a cached skeleton to republish as fresh.
      message.raw_nodes.clear();
      state.last = std::move(message);
      state.valid = true;
    }
  }

  void invalidate(size_t side)
  {
    auto & state = sides_[side];
    if (state.valid) {
      state.last.valid = false;
      publishers_[side]->publish(state.last);
      state.valid = false;
    }
  }

  void fail_callbacks(const char * error) noexcept
  {
    try {
      std::lock_guard<std::mutex> lock(mutex_);
      for (size_t i = 0; i < 2; ++i) {invalidate(i);}
      RCLCPP_ERROR(get_logger(), "MANUS callback rejected: %s", error);
    } catch (...) {
      // No exception may unwind into the proprietary SDK callback dispatcher.
      stopping_.store(true);
    }
  }

  void poll()
  {
    for (size_t side = 0; side < 2; ++side) {
      uint32_t glove = 0;
      uint64_t epoch = 0;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        auto & state = sides_[side];
        const auto now = monotonic_ns();
        if (state.valid && (state.last.source_monotonic_ns > now ||
          now - state.last.source_monotonic_ns > max_age_ns_)) {invalidate(side);}
        if (!state.candidate || state.attempted || stopping_.load()) {continue;}
        state.attempted = true;
        glove = state.candidate;
        epoch = state.epoch;
      }
      // No raw frame is needed to discover or configure a glove. Never hold
      // mutex_ across a control API: calibration may synchronously callback.
      SetGloveCalibrationReturnCode calibration_result = SetGloveCalibrationReturnCode_Error;
      const auto result = CoreSdk_SetGloveCalibration(glove, calibration_[side].data(),
        static_cast<uint32_t>(calibration_[side].size()), &calibration_result);
      const auto session = identity("/proc/sys/kernel/random/uuid");
      std::lock_guard<std::mutex> lock(mutex_);
      auto & state = sides_[side];
      if (state.epoch != epoch) {continue;}
      if (result != SDKReturnCode_Success || calibration_result != SetGloveCalibrationReturnCode_Success) {
        RCLCPP_ERROR(get_logger(), "MANUS %s calibration rejected for %u: SDK=%d calibration=%d; restart after correcting profile",
          kSides[side], glove, static_cast<int>(result), static_cast<int>(calibration_result));
        continue;
      }
      state.sequence = 0;
      state.last_sdk_time = 0;
      state.last = Glove{};
      state.last.session_id = session;
      state.configured_ns = monotonic_ns();
      state.configured = true;
      RCLCPP_INFO(get_logger(), "MANUS %s calibrated glove=%u session=%s; ready for fresh raw frames",
        kSides[side], glove, session.c_str());
    }
  }

  inline static std::atomic<ManusPublisher *> instance_{nullptr};
  std::atomic<bool> stopping_{false};
  int owner_socket_ = -1;
  bool sdk_attempted_ = false;
  int64_t max_age_ns_ = 0;
  std::string boot_id_;
  std::mutex mutex_;
  std::array<SideState, 2> sides_;
  std::array<std::vector<unsigned char>, 2> calibration_;
  std::array<rclcpp::Publisher<Glove>::SharedPtr, 2> publishers_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<ManusPublisher>();
    node->start();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("manus_data_publisher"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
