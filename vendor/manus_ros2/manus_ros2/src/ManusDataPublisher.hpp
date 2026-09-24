#ifndef MANUS_ROS2_MANUS_DATA_PUBLISHER_HPP
#define MANUS_ROS2_MANUS_DATA_PUBLISHER_HPP

#include "ClientPlatformSpecific.hpp"
#include "ManusSDK.h"
#include <rclcpp/rclcpp.hpp>
#include <tianji_interfaces/msg/manus_glove.hpp>
#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

// Adapted from the MANUS Core 3.0.1 SDK ROS2 sample. The sample's SDK
// callback -> raw skeleton handoff -> PublishCallback architecture is retained.
enum class ClientReturnCode : int {
  ClientReturnCode_Success = 0,
  ClientReturnCode_FailedToInitialize,
  ClientReturnCode_FailedToFindHosts,
  ClientReturnCode_FailedToConnect,
  ClientReturnCode_FailedToShutDownSDK
};

struct ClientRawSkeleton {
  static constexpr size_t MaxNodes = 64;
  RawSkeletonInfo info{};
  std::array<SkeletonNode, MaxNodes> nodes{};
  int64_t source_monotonic_ns = 0;
  uint64_t epoch = 0;
  uint64_t sequence = 0;
  uint64_t revocation_generation = 0;
};

class ManusDataPublisher final : public SDKClientPlatformSpecific, public rclcpp::Node {
public:
  ManusDataPublisher();
  ~ManusDataPublisher() override;
  ClientReturnCode Initialize();
  ClientReturnCode InitializeSDK();
  ClientReturnCode ShutDown() noexcept;
  ClientReturnCode RegisterAllCallbacks();
  static void OnRawSkeletonStreamCallback(const SkeletonStreamInfo * stream) noexcept;
  static void OnLandscapeCallback(const Landscape * landscape) noexcept;
  static void OnDisconnectCallback(const ManusHost * host) noexcept;
  void PublishCallback() noexcept;

private:
  using Glove = tianji_interfaces::msg::ManusGlove;
  struct RawDiagnostic {
    const char * reason = "waiting-for-raw";
    int sdk_result = 0;
    uint32_t nodes = 0;
    uint32_t node = 0;
    uint32_t parent = 0;
    int metadata_side = 0;
    ChainType chain{};
    FingerJointType joint{};
    uint64_t matched = 0;
    uint64_t published = 0;
  };
  struct GloveRawSkeletonData {
    uint32_t candidate = 0;
    uint64_t epoch = 0;
    uint64_t sequence = 0;
    uint64_t last_sdk_time = 0;
    uint64_t revocation_generation = 0;
    int64_t configured_ns = 0;
    bool configured = false;
    bool attempted = false;
    bool halted = false;
    bool pending = false;
    bool valid = false;
    bool invalidation_pending = false;
    uint32_t hierarchy_count = 0;
    RawDiagnostic diagnostic;
    std::array<NodeInfo, ClientRawSkeleton::MaxNodes> hierarchy{};
    ClientRawSkeleton raw;
    Glove last;
    rclcpp::Publisher<Glove>::SharedPtr manusGlovesPub;
  };

  ClientReturnCode Connect();
  static const char * SideToString(Side side);
  static const char * JointTypeToString(FingerJointType joint);
  static const char * ChainTypeToString(ChainType chain);
  static bool TopologyValid(const NodeInfo * info, const SkeletonNode * nodes,
    size_t count, Side side, RawDiagnostic & diagnostic);
  void CopyRawSkeletons(const SkeletonStreamInfo * stream, int64_t received_ns);
  void UpdateLandscape(const Landscape * landscape);
  void Revoke(size_t side);  // Requires m_RawSkeletonMutex; never calls the SDK or ROS.
  void Retire(size_t side, uint32_t candidate);
  void FailCallbacks() noexcept;
  void ConfigureGlove(size_t side);
  void PublishSide(size_t side);
  void PublishInvalidation(size_t side);
  void PublishDiagnostics() noexcept;

  inline static std::atomic<ManusDataPublisher *> s_Instance{nullptr};
  std::atomic<bool> m_Stopping{false};
  bool m_SdkAttempted = false;
  bool m_ShutdownFailed = false;
  bool m_Initialized = false;
  int64_t m_MaxSourceAgeNs = 0;
  std::string m_BootId;
  std::mutex m_RawSkeletonMutex;
  std::array<GloveRawSkeletonData, 2> m_GloveData;
  std::array<std::vector<unsigned char>, 2> m_Calibration;
  uint64_t m_RawCallbacks = 0;
  uint32_t m_StreamSkeletons = 0;
  std::array<uint32_t, 4> m_StreamGloves{};
  const char * m_StreamReason = "no-callback";
  int m_StreamSdkResult = 0;
  int64_t m_LastDiagnosticNs = 0;
  uint32_t m_ReportedUsers = 0xFFFFFFFFu;
  uint32_t m_ReportedGloves = 0xFFFFFFFFu;
  rclcpp::TimerBase::SharedPtr m_PublishTimer;
  inline static std::mutex s_CallbackMutex;
};

#endif
