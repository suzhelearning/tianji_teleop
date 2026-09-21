#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <future>
#include <memory>

#include "odin_lidar_api.h"
#include "internal/odin_lidar_inner.h"
#include "IProtocol.hpp"
#include "ITransport.hpp"

namespace odin {
namespace sdk {

/**
 * @brief CommandChannel configuration (transport-agnostic)
 *
 * Network/USB address configuration is handled by ITransport externally.
 * CommandChannel only deals with communication logic.
 */
struct CommandChannelConfig {
  uint32_t default_timeout_ms = 1000;
  OdinDeviceHandle device_handle = kInvalidDeviceHandle;
};

class CommandChannel {
 public:
  // Protocol and Transport injection for polymorphism
  CommandChannel(std::unique_ptr<IProtocol> protocol, std::unique_ptr<ITransport> transport);
  ~CommandChannel();

  bool Start(const CommandChannelConfig &config);
  void Stop();

  bool SendAsync(uint16_t cmd_id, const std::vector<uint8_t> &payload, OdinCommandCallback callback,
                 void *client_data, uint32_t timeout_ms);

  bool SendSync(uint16_t cmd_id, const std::vector<uint8_t> &payload, OdinCommandSyncResponse &sr,
                uint32_t timeout_ms);

 private:
  struct PendingCommand {
    OdinCommandCallback callback = nullptr;
    void *client_data = nullptr;
    uint16_t cmd_id = 0;
    std::chrono::steady_clock::time_point deadline;
  };

  void HandlePacket(const uint8_t *data, size_t length, const ITransportAddress &from);
  void CleanupTimeouts();
  void TimeoutLoop();

  bool started_ = false;
  std::thread timeout_thread_;
  std::atomic<bool> running_{false};

  std::atomic<int> active_callbacks_{0};
  std::mutex active_mutex_;
  std::condition_variable active_cv_;
  bool waiting_for_callbacks_ = false;

  CommandChannelConfig config_;
  std::atomic<uint16_t> next_seq_{0};

  std::unique_ptr<IProtocol> protocol_;
  std::unique_ptr<ITransport> transport_;

  std::mutex pending_mutex_;
  std::map<uint16_t, PendingCommand> pending_;
};

}  // namespace sdk
}  // namespace odin
