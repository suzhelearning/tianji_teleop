// Safety guards incorporate code from the former tianji Manus publisher.
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

#include "ManusDataPublisher.hpp"
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <utility>

namespace {
constexpr const char * kSides[] = {"left", "right"};
constexpr uint64_t kMaxCounter = std::numeric_limits<uint64_t>::max();

const char * raw_nodes_rejection(const SkeletonNode * nodes, size_t count, uint32_t & node)
{
  if (count < 21 || count > ClientRawSkeleton::MaxNodes) {return "node-count-bounds";}
  bool nonzero_position = false;
  for (size_t i = 0; i < count; ++i) {
    node = nodes[i].id;
    for (size_t j = 0; j < i; ++j) {
      if (nodes[i].id == nodes[j].id) {return "duplicate-raw-node";}
    }
    const auto & p = nodes[i].transform.position;
    const auto & q = nodes[i].transform.rotation;
    const auto & scale = nodes[i].transform.scale;
    const double norm = static_cast<double>(q.x) * q.x + static_cast<double>(q.y) * q.y +
      static_cast<double>(q.z) * q.z + static_cast<double>(q.w) * q.w;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {return "position-nonfinite";}
    if (std::abs(p.x) > 10.0 || std::abs(p.y) > 10.0 || std::abs(p.z) > 10.0) {return "position-bounds";}
    if (!std::isfinite(norm) || norm < 0.5 || norm > 1.5) {return "rotation-norm";}
    if (!std::isfinite(scale.x) || !std::isfinite(scale.y) || !std::isfinite(scale.z)) {return "scale-nonfinite";}
    nonzero_position |= p.x != 0.0f || p.y != 0.0f || p.z != 0.0f;
  }
  node = 0;
  return nonzero_position ? nullptr : "all-positions-zero";
}
}  // namespace

ManusDataPublisher::ManusDataPublisher() : Node("manus_data_publisher")
{
  // SDK startup is deliberately separate: a failed Initialize still destroys
  // this fully constructed object and shuts down registered callback threads.
  const auto directory = declare_parameter<std::string>("calibration.directory", "");
  const double age = declare_parameter<double>("max_source_age_s", 0.25);
  if (!std::isfinite(age) || age <= 0.0 || age > 60.0) {
    throw std::runtime_error("max_source_age_s must be finite and in (0, 60]");
  }
  m_MaxSourceAgeNs = static_cast<int64_t>(age * 1e9);
  m_BootId = ReadUuid("/proc/sys/kernel/random/boot_id");
  for (size_t side = 0; side < 2; ++side) {
    const auto filename = declare_parameter<std::string>(
      std::string("calibration.") + kSides[side] + "_file", "");
    if (filename.empty()) {throw std::runtime_error("Both calibration files must be supplied");}
    auto path = std::filesystem::path(filename);
    if (path.is_relative()) {path = std::filesystem::path(directory) / path;}
    if (!path.is_absolute()) {throw std::runtime_error("Calibration paths must resolve absolute");}
    std::ifstream input(path, std::ios::binary | std::ios::ate);
    const auto size = input.tellg();
    if (!input || size <= 0 || size > 16 * 1024 * 1024) {
      throw std::runtime_error("Missing, empty or oversized calibration: " + path.string());
    }
    m_Calibration[side].resize(static_cast<size_t>(size));
    input.seekg(0);
    if (!input.read(reinterpret_cast<char *>(m_Calibration[side].data()), size)) {
      throw std::runtime_error("Incomplete calibration read: " + path.string());
    }
    m_GloveData[side].manusGlovesPub = create_publisher<Glove>(
      std::string("/manus/raw/") + kSides[side],
      rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());
  }
}

ManusDataPublisher::~ManusDataPublisher()
{
  if (ShutDown() != ClientReturnCode::ClientReturnCode_Success) {
    RCLCPP_ERROR(get_logger(), "MANUS SDK shutdown failed; ownership retained until process exit");
  }
}

ClientReturnCode ManusDataPublisher::Initialize()
{
  if (m_Initialized || m_SdkAttempted || m_Stopping.load()) {
    return ClientReturnCode::ClientReturnCode_FailedToInitialize;
  }
  AcquireSdkOwnership();
  {
    std::lock_guard<std::mutex> callbacks(s_CallbackMutex);
    if (s_Instance.load()) {throw std::runtime_error("MANUS publisher already initialized");}
    s_Instance.store(this);
  }
  auto result = InitializeSDK();
  if (result != ClientReturnCode::ClientReturnCode_Success) {return result;}
  result = Connect();
  if (result != ClientReturnCode::ClientReturnCode_Success) {return result;}
  // User commands require a connected Core; Integrated SDK can crash before Connect().
  // The loaded CoreLite settings file can disable auto user assignment. Without
  // an assigned user Core never generates raw skeletons, so a publisher that
  // never manages users itself would stay silent while devices stream. Restore
  // the documented SDK default and report the result; setups that assign users
  // explicitly are left untouched because a failure here is not fatal.
  const auto assignment = CoreSdk_SetAutoUserAssignment(true);
  if (assignment == SDKReturnCode_Success) {
    RCLCPP_INFO(get_logger(), "MANUS auto user assignment enabled (SDK default)");
  } else {
    RCLCPP_WARN(get_logger(),
      "MANUS auto user assignment request failed (%d); expecting pre-assigned users",
      static_cast<int>(assignment));
  }
  if (CoreSdk_SetRawSkeletonHandMotion(HandMotion_None) != SDKReturnCode_Success) {
    return ClientReturnCode::ClientReturnCode_FailedToInitialize;
  }
  m_PublishTimer = create_wall_timer(std::chrono::microseconds(8333),
    [this] {PublishCallback();});
  m_Initialized = true;
  RCLCPP_INFO(get_logger(), "MANUS official ROS acquisition connected; waiting for licensed gloves and calibration");
  return ClientReturnCode::ClientReturnCode_Success;
}

ClientReturnCode ManusDataPublisher::InitializeSDK()
{
  m_SdkAttempted = true;
  if (CoreSdk_InitializeIntegrated() != SDKReturnCode_Success) {
    return ClientReturnCode::ClientReturnCode_FailedToInitialize;
  }
  const auto result = RegisterAllCallbacks();
  if (result != ClientReturnCode::ClientReturnCode_Success) {return result;}
  CoordinateSystemVUH coordinates{};
  CoordinateSystemVUH_Init(&coordinates);
  // Init overwrites aggregate defaults: set the downstream VUH contract after it.
  coordinates.view = AxisView_XFromViewer;
  coordinates.up = AxisPolarity_PositiveZ;
  coordinates.handedness = Side_Right;
  coordinates.unitScale = 1.0f;
  if (CoreSdk_InitializeCoordinateSystemWithVUH(coordinates, true) != SDKReturnCode_Success) {
    return ClientReturnCode::ClientReturnCode_FailedToInitialize;
  }
  return ClientReturnCode::ClientReturnCode_Success;
}

ClientReturnCode ManusDataPublisher::RegisterAllCallbacks()
{
  // Raw publication does not depend on ergonomics or raw sensor streams.
  if (CoreSdk_RegisterCallbackForRawSkeletonStream(*OnRawSkeletonStreamCallback) != SDKReturnCode_Success ||
    CoreSdk_RegisterCallbackForLandscapeStream(*OnLandscapeCallback) != SDKReturnCode_Success ||
    CoreSdk_RegisterCallbackForOnDisconnect(*OnDisconnectCallback) != SDKReturnCode_Success)
  {
    return ClientReturnCode::ClientReturnCode_FailedToInitialize;
  }
  return ClientReturnCode::ClientReturnCode_Success;
}

ClientReturnCode ManusDataPublisher::Connect()
{
  if (CoreSdk_LookForHosts(5, false) != SDKReturnCode_Success) {
    return ClientReturnCode::ClientReturnCode_FailedToFindHosts;
  }
  uint32_t count = 0;
  if (CoreSdk_GetNumberOfAvailableHostsFound(&count) != SDKReturnCode_Success || count != 1) {
    return ClientReturnCode::ClientReturnCode_FailedToFindHosts;
  }
  ManusHost host{};
  if (CoreSdk_GetAvailableHostsFound(&host, 1) != SDKReturnCode_Success ||
    CoreSdk_ConnectToHost(host) != SDKReturnCode_Success)
  {
    return ClientReturnCode::ClientReturnCode_FailedToConnect;
  }
  return ClientReturnCode::ClientReturnCode_Success;
}

ClientReturnCode ManusDataPublisher::ShutDown() noexcept
{
  m_Stopping.store(true);
  m_PublishTimer.reset();
  {
    // Drain callbacks that already acquired this object; later callbacks see
    // null. Do not hold this gate across SDK shutdown, which joins its threads.
    std::lock_guard<std::mutex> callbacks(s_CallbackMutex);
    if (s_Instance.load() == this) {s_Instance.store(nullptr);}
  }
  if (m_ShutdownFailed) {return ClientReturnCode::ClientReturnCode_FailedToShutDownSDK;}
  if (m_SdkAttempted) {
    m_SdkAttempted = false;
    if (CoreSdk_ShutDown() != SDKReturnCode_Success) {
      m_ShutdownFailed = true;
      return ClientReturnCode::ClientReturnCode_FailedToShutDownSDK;
    }
  }
  ReleaseSdkOwnership();
  return ClientReturnCode::ClientReturnCode_Success;
}

void ManusDataPublisher::Revoke(size_t side)
{
  auto & state = m_GloveData[side];
  state.pending = false;
  state.valid = false;
  state.invalidation_pending = true;
  if (state.revocation_generation == kMaxCounter) {
    state.halted = true;
    state.configured = false;
  } else {
    ++state.revocation_generation;
  }
}

void ManusDataPublisher::Retire(size_t side, uint32_t candidate)
{
  Revoke(side);
  auto & state = m_GloveData[side];
  if (state.epoch == kMaxCounter) {state.halted = true;} else {++state.epoch;}
  state.candidate = candidate;
  state.configured = false;
  state.attempted = false;
  state.hierarchy_count = 0;
  state.diagnostic.reason = candidate ? "awaiting-configuration" : "no-landscape-candidate";
}

void ManusDataPublisher::FailCallbacks() noexcept
{
  try {
    std::lock_guard<std::mutex> lock(m_RawSkeletonMutex);
    for (size_t side = 0; side < 2; ++side) {Revoke(side);}
  } catch (...) {
    // No C++ exception can cross the proprietary callback dispatcher.
    m_Stopping.store(true);
  }
}

void ManusDataPublisher::OnLandscapeCallback(const Landscape * landscape) noexcept
{
  std::lock_guard<std::mutex> callbacks(s_CallbackMutex);
  auto * self = s_Instance.load();
  if (!self || self->m_Stopping.load()) {return;}
  try {self->UpdateLandscape(landscape);} catch (...) {self->FailCallbacks();}
}

void ManusDataPublisher::UpdateLandscape(const Landscape * landscape)
{
  std::array<uint32_t, 2> candidates{};
  std::array<size_t, 2> counts{};
  if (landscape) {
    // Bounded change-only report: raw skeletons require an assigned user, so a
    // zero user/glove landscape is the first place a silent stream shows up.
    const uint32_t gloves = landscape->gloveDevices.gloveCount;
    const uint32_t users = landscape->users.userCount;
    if (users != m_ReportedUsers || gloves != m_ReportedGloves) {
      m_ReportedUsers = users;
      m_ReportedGloves = gloves;
      if (gloves == 0 || users == 0) {
        RCLCPP_WARN(get_logger(),
          "MANUS landscape gloves=%u users=%u licensed=%d; raw skeletons need an assigned user",
          gloves, users, landscape->settings.license.integrated ? 1 : 0);
      } else {
        RCLCPP_INFO(get_logger(), "MANUS landscape gloves=%u users=%u", gloves, users);
      }
    }
  }
  bool acceptable = landscape && landscape->settings.license.integrated &&
    landscape->gloveDevices.gloveCount <= MAX_NUMBER_OF_GLOVES;
  if (acceptable) {
    for (uint32_t i = 0; i < landscape->gloveDevices.gloveCount; ++i) {
      const auto & glove = landscape->gloveDevices.gloves[i];
      if (glove.excluded) {continue;}
      if (!glove.id || (glove.side != Side_Left && glove.side != Side_Right)) {
        acceptable = false; break;
      }
      for (uint32_t previous = 0; previous < i; ++previous) {
        const auto & other = landscape->gloveDevices.gloves[previous];
        if (!other.excluded && other.id == glove.id) {acceptable = false; break;}
      }
      if (!acceptable) {break;}
      const size_t side = glove.side == Side_Left ? 0 : 1;
      candidates[side] = glove.id;
      ++counts[side];
    }
  }
  if (candidates[0] && candidates[0] == candidates[1]) {acceptable = false;}
  std::lock_guard<std::mutex> lock(m_RawSkeletonMutex);
  for (size_t side = 0; side < 2; ++side) {
    const uint32_t candidate = acceptable && counts[side] == 1 ? candidates[side] : 0;
    if (m_GloveData[side].candidate != candidate) {Retire(side, candidate);}
  }
}

void ManusDataPublisher::OnDisconnectCallback(const ManusHost *) noexcept
{
  std::lock_guard<std::mutex> callbacks(s_CallbackMutex);
  auto * self = s_Instance.load();
  if (!self || self->m_Stopping.load()) {return;}
  try {
    std::lock_guard<std::mutex> lock(self->m_RawSkeletonMutex);
    for (size_t side = 0; side < 2; ++side) {self->Retire(side, 0);}
  } catch (...) {self->FailCallbacks();}
}

void ManusDataPublisher::OnRawSkeletonStreamCallback(const SkeletonStreamInfo * stream) noexcept
{
  // Timestamp THIS SDK callback before waiting for either callback/state gate.
  int64_t received_ns = 0;
  try {received_ns = MonotonicNs();} catch (...) {}
  std::lock_guard<std::mutex> callbacks(s_CallbackMutex);
  auto * self = s_Instance.load();
  if (!self || self->m_Stopping.load()) {return;}
  try {self->CopyRawSkeletons(stream, received_ns);} catch (...) {self->FailCallbacks();}
}

void ManusDataPublisher::CopyRawSkeletons(const SkeletonStreamInfo * stream, int64_t received_ns)
{
  {
    std::lock_guard<std::mutex> lock(m_RawSkeletonMutex);
    if (m_RawCallbacks != kMaxCounter) {++m_RawCallbacks;}
    m_StreamSkeletons = stream ? stream->skeletonsCount : 0;
    m_StreamGloves.fill(0);
    m_StreamSdkResult = 0;
    m_StreamReason = !stream ? "null-stream" : received_ns <= 0 ? "callback-clock" :
      stream->skeletonsCount > MAX_NUMBER_OF_GLOVES ? "skeleton-count-bounds" :
      stream->skeletonsCount == 0 ? "zero-skeletons" : "reading-info";
  }
  if (!stream || received_ns <= 0 || stream->skeletonsCount > MAX_NUMBER_OF_GLOVES) {
    FailCallbacks(); return;
  }
  std::array<RawSkeletonInfo, MAX_NUMBER_OF_GLOVES> skeletons{};
  for (uint32_t index = 0; index < stream->skeletonsCount; ++index) {
    const auto result = CoreSdk_GetRawSkeletonInfo(index, &skeletons[index]);
    {
      std::lock_guard<std::mutex> lock(m_RawSkeletonMutex);
      if (index < m_StreamGloves.size()) {m_StreamGloves[index] = skeletons[index].gloveId;}
      m_StreamSdkResult = static_cast<int>(result);
      if (result != SDKReturnCode_Success) {m_StreamReason = "sdk-raw-info";}
      else if (!skeletons[index].gloveId) {m_StreamReason = "zero-glove-id";}
    }
    if (result != SDKReturnCode_Success || skeletons[index].gloveId == 0) {
      FailCallbacks(); return;
    }
    for (uint32_t previous = 0; previous < index; ++previous) {
      if (skeletons[index].gloveId == skeletons[previous].gloveId) {
        {
          std::lock_guard<std::mutex> lock(m_RawSkeletonMutex);
          m_StreamReason = "duplicate-glove-id";
        }
        FailCallbacks(); return;
      }
    }
  }
  {
    std::lock_guard<std::mutex> lock(m_RawSkeletonMutex);
    if (stream->skeletonsCount) {m_StreamReason = "no-eligible-glove";}
  }
  for (uint32_t index = 0; index < stream->skeletonsCount; ++index) {
    const auto & info = skeletons[index];
    size_t side = 2;
    uint64_t epoch = 0;
    {
      std::lock_guard<std::mutex> lock(m_RawSkeletonMutex);
      for (size_t i = 0; i < 2; ++i) {
        auto & state = m_GloveData[i];
        if (state.candidate == info.gloveId) {
          if (state.diagnostic.matched != kMaxCounter) {++state.diagnostic.matched;}
          state.diagnostic.nodes = info.nodesCount;
          state.diagnostic.reason = !state.configured ? "not-configured" :
            state.halted ? "halted" : received_ns <= state.configured_ns ?
            "callback-before-configuration" : "matched";
        }
        if (state.configured && !state.halted && state.candidate == info.gloveId &&
          received_ns > state.configured_ns)
        {
          m_StreamReason = "matched-glove";
          side = i; epoch = state.epoch; break;
        }
      }
    }
    if (side == 2) {continue;}
    ClientRawSkeleton raw;
    raw.info = info;
    raw.info.publishTime = stream->publishTime;
    raw.source_monotonic_ns = received_ns;
    raw.epoch = epoch;
    bool valid = info.nodesCount >= 21 && info.nodesCount <= ClientRawSkeleton::MaxNodes;
    const char * rejection = valid ? nullptr : "node-count-bounds";
    int sdk_result = 0;
    if (valid) {
      const auto result = CoreSdk_GetRawSkeletonData(index, raw.nodes.data(), info.nodesCount);
      sdk_result = static_cast<int>(result);
      valid = result == SDKReturnCode_Success;
      if (!valid) {rejection = "sdk-raw-data";}
    }
    const auto now = MonotonicNs();
    if (valid && (received_ns > now || now - received_ns > m_MaxSourceAgeNs)) {
      valid = false; rejection = received_ns > now ? "callback-future" : "callback-stale";
    }
    std::lock_guard<std::mutex> lock(m_RawSkeletonMutex);
    auto & state = m_GloveData[side];
    if (!state.configured || state.epoch != epoch || state.halted) {continue;}
    auto & diagnostic = state.diagnostic;
    diagnostic.sdk_result = sdk_result;
    diagnostic.node = 0;
    diagnostic.parent = 0;
    diagnostic.metadata_side = 0;
    diagnostic.chain = ChainType{};
    diagnostic.joint = FingerJointType{};
    // Opaque SDK UTC/timecode is only a repeat/backwards guard, never source time.
    if (stream->publishTime.time == 0 || stream->publishTime.time <= state.last_sdk_time) {
      valid = false;
      if (!rejection) {rejection = stream->publishTime.time == 0 ? "sdk-time-zero" : "sdk-time-repeat-backward";}
    }
    if (stream->publishTime.time > state.last_sdk_time) {state.last_sdk_time = stream->publishTime.time;}
    if (state.sequence == kMaxCounter) {
      diagnostic.reason = "sequence-exhausted";
      Revoke(side); state.halted = true; continue;
    }
    raw.sequence = ++state.sequence;
    // Metadata was fetched off-callback. Checking this immutable epoch snapshot
    // here preserves invalid edges even if a later valid frame replaces the slot.
    if (valid) {
      if (state.hierarchy_count) {
        valid = info.nodesCount == state.hierarchy_count && TopologyValid(
          state.hierarchy.data(), raw.nodes.data(), info.nodesCount,
          side == 0 ? Side_Left : Side_Right, diagnostic);
        if (info.nodesCount != state.hierarchy_count) {rejection = "hierarchy-count-mismatch";}
        else if (!valid) {rejection = diagnostic.reason;}
      } else {
        rejection = raw_nodes_rejection(raw.nodes.data(), info.nodesCount, diagnostic.node);
        valid = !rejection;
      }
    }
    if (!valid) {diagnostic.reason = rejection; Revoke(side); continue;}
    diagnostic.reason = "pending";
    raw.revocation_generation = state.revocation_generation;
    state.raw = std::move(raw);
    state.pending = true;
  }
}

void ManusDataPublisher::PublishInvalidation(size_t side)
{
  // Caller holds the short state lock; SDK control calls are never made here.
  auto & state = m_GloveData[side];
  if (!state.invalidation_pending) {return;}
  if (!state.last.session_id.empty()) {
    state.last.valid = false;
    state.last.revocation_generation = state.revocation_generation;
    state.last.raw_nodes.clear();
    state.manusGlovesPub->publish(state.last);
  }
  state.invalidation_pending = false;
}

void ManusDataPublisher::ConfigureGlove(size_t side)
{
  uint32_t glove = 0;
  uint64_t epoch = 0;
  {
    std::lock_guard<std::mutex> lock(m_RawSkeletonMutex);
    auto & state = m_GloveData[side];
    PublishInvalidation(side);
    if (!state.candidate || state.attempted || state.halted || m_Stopping.load()) {return;}
    state.attempted = true;
    glove = state.candidate;
    epoch = state.epoch;
  }
  // Calibration can synchronously invoke callbacks: no state/callback lock held.
  SetGloveCalibrationReturnCode calibration_result = SetGloveCalibrationReturnCode_Error;
  const auto result = CoreSdk_SetGloveCalibration(glove, m_Calibration[side].data(),
    static_cast<uint32_t>(m_Calibration[side].size()), &calibration_result);
  const auto session = ReadUuid("/proc/sys/kernel/random/uuid");
  const auto configured_ns = MonotonicNs();
  std::lock_guard<std::mutex> lock(m_RawSkeletonMutex);
  auto & state = m_GloveData[side];
  if (state.epoch != epoch || m_Stopping.load()) {return;}
  if (result != SDKReturnCode_Success || calibration_result != SetGloveCalibrationReturnCode_Success) {
    Revoke(side);
    state.diagnostic.reason = "sdk-calibration";
    state.diagnostic.sdk_result = static_cast<int>(result);
    RCLCPP_ERROR(get_logger(), "MANUS %s calibration rejected for %u: SDK=%d calibration=%d; restart after correcting profile",
      kSides[side], glove, static_cast<int>(result), static_cast<int>(calibration_result));
    return;
  }
  state.sequence = 0;
  state.last_sdk_time = 0;
  state.last = Glove{};
  state.last.glove_id = glove;
  state.last.side = kSides[side];
  state.last.session_id = session;
  state.last.boot_id = m_BootId;
  state.last.revocation_generation = state.revocation_generation;
  state.configured_ns = configured_ns;
  state.hierarchy_count = 0;
  state.pending = false;
  state.configured = true;
  state.diagnostic.reason = "waiting-for-raw";
  RCLCPP_INFO(get_logger(), "MANUS %s calibrated glove=%u session=%s; waiting for fresh raw skeleton",
    kSides[side], glove, session.c_str());
}

void ManusDataPublisher::PublishSide(size_t side)
{
  uint32_t glove = 0;
  uint32_t count = 0;
  uint64_t epoch = 0;
  {
    std::lock_guard<std::mutex> lock(m_RawSkeletonMutex);
    auto & state = m_GloveData[side];
    const auto now = MonotonicNs();
    if (state.valid && (state.last.source_monotonic_ns > now ||
      now - state.last.source_monotonic_ns > m_MaxSourceAgeNs)) {
      state.diagnostic.reason = state.last.source_monotonic_ns > now ? "output-future" : "output-stale";
      Revoke(side);
    }
    PublishInvalidation(side);
    if (!state.configured || !state.pending || state.halted || m_Stopping.load()) {return;}
    if (!state.hierarchy_count) {
      glove = state.candidate;
      count = state.raw.info.nodesCount;
      epoch = state.epoch;
    }
  }
  if (count) {
    std::array<NodeInfo, ClientRawSkeleton::MaxNodes> hierarchy{};
    const auto result = CoreSdk_GetRawSkeletonNodeInfoArray(glove, hierarchy.data(), count);
    std::lock_guard<std::mutex> lock(m_RawSkeletonMutex);
    auto & state = m_GloveData[side];
    if (!state.configured || state.epoch != epoch || m_Stopping.load()) {return;}
    if (result != SDKReturnCode_Success) {
      state.diagnostic.reason = "sdk-node-info";
      state.diagnostic.sdk_result = static_cast<int>(result);
      Revoke(side); PublishInvalidation(side); return;
    }
    state.hierarchy = hierarchy;
    state.hierarchy_count = count;
  }
  std::lock_guard<std::mutex> lock(m_RawSkeletonMutex);
  auto & state = m_GloveData[side];
  PublishInvalidation(side);
  if (!state.configured || !state.pending || state.halted || m_Stopping.load()) {return;}
  // Consume once. A timer tick without a new callback can only revoke an aged
  // source; it cannot refresh source time or republish a cached valid pose.
  state.pending = false;
  const auto & raw = state.raw;
  const auto now = MonotonicNs();
  const char * rejection = raw.epoch != state.epoch ? "epoch-mismatch" :
    raw.revocation_generation != state.revocation_generation ? "revocation-mismatch" :
    raw.source_monotonic_ns <= state.configured_ns ? "source-before-configuration" :
    raw.source_monotonic_ns > now ? "source-future" :
    now - raw.source_monotonic_ns > m_MaxSourceAgeNs ? "source-stale" :
    raw.info.nodesCount != state.hierarchy_count ? "hierarchy-count-mismatch" : nullptr;
  if (rejection || !TopologyValid(state.hierarchy.data(), raw.nodes.data(), raw.info.nodesCount,
    side == 0 ? Side_Left : Side_Right, state.diagnostic))
  {
    if (rejection) {state.diagnostic.reason = rejection;}
    Revoke(side); PublishInvalidation(side); return;
  }
  Glove message;
  message.glove_id = state.candidate;
  message.side = SideToString(side == 0 ? Side_Left : Side_Right);
  message.session_id = state.last.session_id;
  message.boot_id = m_BootId;
  message.sequence = raw.sequence;
  message.source_monotonic_ns = raw.source_monotonic_ns;
  message.revocation_generation = state.revocation_generation;
  message.valid = true;
  message.raw_nodes.reserve(raw.info.nodesCount);
  for (size_t index = 0; index < raw.info.nodesCount; ++index) {
    const auto & source = raw.nodes[index];
    const NodeInfo * metadata = nullptr;
    for (size_t n = 0; n < state.hierarchy_count; ++n) {
      if (state.hierarchy[n].nodeId == source.id) {metadata = &state.hierarchy[n]; break;}
    }
    tianji_interfaces::msg::ManusRawNode node;
    node.node_id = source.id;
    node.parent_node_id = metadata->parentId;
    node.chain_type = ChainTypeToString(metadata->chainType);
    node.joint_type = JointTypeToString(metadata->fingerJointType);
    const auto & p = source.transform.position;
    const auto & q = source.transform.rotation;
    node.pose.position.x = p.x; node.pose.position.y = p.y; node.pose.position.z = p.z;
    node.pose.orientation.x = q.x; node.pose.orientation.y = q.y;
    node.pose.orientation.z = q.z; node.pose.orientation.w = q.w;
    message.raw_nodes.push_back(std::move(node));
  }
  state.manusGlovesPub->publish(message);
  message.raw_nodes.clear();
  state.last = std::move(message);
  state.valid = true;
  state.diagnostic.reason = "published";
  if (state.diagnostic.published != kMaxCounter) {++state.diagnostic.published;}
}

void ManusDataPublisher::PublishDiagnostics() noexcept
{
  // Diagnostics must never revoke a sample or hold the handoff gate while logging.
  try {
    struct Snapshot {
      RawDiagnostic diagnostic;
      uint32_t glove = 0;
      uint32_t hierarchy = 0;
      bool configured = false;
      bool pending = false;
      bool valid = false;
      bool halted = false;
    };
    std::array<Snapshot, 2> sides;
    uint64_t callbacks = 0;
    uint32_t skeletons = 0;
    std::array<uint32_t, 4> gloves{};
    const char * reason = nullptr;
    int sdk_result = 0;
    {
      std::lock_guard<std::mutex> lock(m_RawSkeletonMutex);
      if (m_GloveData[0].valid && m_GloveData[1].valid) {return;}
      const auto now = MonotonicNs();
      if (m_LastDiagnosticNs && now - m_LastDiagnosticNs < 5000000000LL) {return;}
      m_LastDiagnosticNs = now;
      callbacks = m_RawCallbacks;
      skeletons = m_StreamSkeletons;
      gloves = m_StreamGloves;
      reason = m_StreamReason;
      sdk_result = m_StreamSdkResult;
      for (size_t i = 0; i < sides.size(); ++i) {
        const auto & state = m_GloveData[i];
        sides[i] = {state.diagnostic, state.candidate, state.hierarchy_count,
          state.configured, state.pending, state.valid, state.halted};
      }
    }
    std::array<std::array<char, 512>, 2> descriptions{};
    for (size_t i = 0; i < sides.size(); ++i) {
      const auto & state = sides[i];
      const auto & d = state.diagnostic;
      std::snprintf(descriptions[i].data(), descriptions[i].size(),
        "%s{id=%u matched=%llu published=%llu configured=%d pending=%d valid=%d halted=%d "
        "reason=%s sdk=%d nodes=%u/%u node=%u parent=%u metadata_side=%d chain=%d:%s joint=%d:%s}",
        kSides[i], state.glove, static_cast<unsigned long long>(d.matched),
        static_cast<unsigned long long>(d.published), state.configured, state.pending,
        state.valid, state.halted, d.reason, d.sdk_result, d.nodes, state.hierarchy,
        d.node, d.parent, d.metadata_side, static_cast<int>(d.chain), ChainTypeToString(d.chain),
        static_cast<int>(d.joint), JointTypeToString(d.joint));
    }
    RCLCPP_WARN(get_logger(),
      "MANUS raw diagnostic callbacks=%llu skeletons=%u sdk_gloves_first4=[%u,%u,%u,%u] "
      "stream=%s sdk=%d %s %s",
      static_cast<unsigned long long>(callbacks), skeletons, gloves[0], gloves[1], gloves[2],
      gloves[3], reason, sdk_result, descriptions[0].data(), descriptions[1].data());
  } catch (...) {
    // Observability is best effort; existing validity and lifetime rules are unchanged.
  }
}

void ManusDataPublisher::PublishCallback() noexcept
{
  try {
    for (size_t side = 0; side < 2; ++side) {
      ConfigureGlove(side);
      PublishSide(side);
    }
    PublishDiagnostics();
  } catch (const std::exception & error) {
    FailCallbacks();
    RCLCPP_ERROR(get_logger(), "MANUS publish/control failed: %s", error.what());
  } catch (...) {
    FailCallbacks();
    RCLCPP_ERROR(get_logger(), "MANUS publish/control failed");
  }
}

const char * ManusDataPublisher::SideToString(Side side)
{
  switch (side) {
    case Side_Left: return "left";
    case Side_Right: return "right";
    default: return "Invalid";
  }
}

const char * ManusDataPublisher::JointTypeToString(FingerJointType joint)
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

const char * ManusDataPublisher::ChainTypeToString(ChainType chain)
{
  switch (chain) {
    case ChainType_Hand: return "Hand";
    case ChainType_FingerThumb: return "Thumb";
    case ChainType_FingerIndex: return "Index";
    case ChainType_FingerMiddle: return "Middle";
    case ChainType_FingerRing: return "Ring";
    case ChainType_FingerPinky: return "Pinky";
    default: return "Invalid";
  }
}

bool ManusDataPublisher::TopologyValid(const NodeInfo * info, const SkeletonNode * nodes,
  size_t count, Side side, RawDiagnostic & diagnostic)
{
  const auto numeric = raw_nodes_rejection(nodes, count, diagnostic.node);
  if (numeric) {diagnostic.reason = numeric; return false;}
  const auto reject = [&](const char * reason, size_t index) {
      diagnostic.reason = reason;
      if (index < count) {
        diagnostic.node = info[index].nodeId;
        diagnostic.parent = info[index].parentId;
        diagnostic.metadata_side = static_cast<int>(info[index].side);
        diagnostic.chain = info[index].chainType;
        diagnostic.joint = info[index].fingerJointType;
      }
      return false;
    };
  size_t root = count;
  for (size_t i = 0; i < count; ++i) {
    if (std::strcmp(ChainTypeToString(info[i].chainType), "Invalid") == 0) {return reject("metadata-chain-type", i);}
    if (info[i].side != side) {return reject("metadata-side", i);}
    if (info[i].chainType == ChainType_Hand) {
      if (root != count) {return reject("multiple-hand-roots", i);}
      root = i;
    } else if (std::strcmp(JointTypeToString(info[i].fingerJointType), "Invalid") == 0) {
      return reject("metadata-joint-type", i);
    }
    for (size_t j = 0; j < i; ++j) {
      if (info[i].nodeId == info[j].nodeId) {return reject("duplicate-metadata-node", i);}
      if (info[i].chainType == info[j].chainType && info[i].fingerJointType == info[j].fingerJointType) {
        return reject("duplicate-semantic-joint", i);
      }
    }
    bool found = false;
    for (size_t j = 0; j < count; ++j) {found |= nodes[i].id == info[j].nodeId;}
    if (!found) {
      reject("raw-node-without-metadata", count);
      diagnostic.node = nodes[i].id;
      return false;
    }
  }
  if (root == count) {return reject("missing-hand-root", count);}
  // Require the exact semantic subset used by hand2, not merely 21 nodes.
  constexpr ChainType fingers[] = {ChainType_FingerThumb, ChainType_FingerIndex,
    ChainType_FingerMiddle, ChainType_FingerRing, ChainType_FingerPinky};
  for (const auto finger : fingers) {
    const FingerJointType required[] = {
      finger == ChainType_FingerThumb ? FingerJointType_Metacarpal : FingerJointType_Intermediate,
      FingerJointType_Proximal, FingerJointType_Distal, FingerJointType_Tip};
    for (const auto joint : required) {
      bool found = false;
      for (size_t i = 0; i < count; ++i) {found |= info[i].chainType == finger && info[i].fingerJointType == joint;}
      if (!found) {
        reject("missing-semantic-joint", count);
        diagnostic.chain = finger;
        diagnostic.joint = joint;
        return false;
      }
    }
  }
  for (size_t i = 0; i < count; ++i) {
    if (i != root && info[root].parentId == info[i].nodeId) {return reject("root-parent-is-child", root);}
    size_t current = i;
    size_t depth = 0;
    while (current != root) {
      if (++depth >= count) {return reject("parent-cycle", current);}
      size_t parent = count;
      for (size_t j = 0; j < count; ++j) {
        if (info[j].nodeId == info[current].parentId) {parent = j; break;}
      }
      if (parent == count) {return reject("missing-parent", current);}
      current = parent;
    }
  }
  return true;
}
