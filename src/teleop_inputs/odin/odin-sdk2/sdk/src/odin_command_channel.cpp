#include "odin_command_channel.h"
#include "logger.h"

#include <chrono>
#include <cstring>
#include <memory>
#include <vector>

namespace odin {
namespace sdk {

CommandChannel::CommandChannel(std::unique_ptr<IProtocol> protocol,
                               std::unique_ptr<ITransport> transport)
    : protocol_(std::move(protocol)), transport_(std::move(transport)) {}

CommandChannel::~CommandChannel() { Stop(); }

bool CommandChannel::Start(const CommandChannelConfig &config) {
  if (started_) {
    return true;
  }

  config_ = config;

  // Transport should already be configured (Open + SetRemoteTarget called externally)
  if (!transport_->IsOpen()) {
    LOG_ERROR("CommandChannel::Start: transport not open\n");
    return false;
  }

  // Set receive callback
  transport_->SetReceiveCallback(
      [this](const uint8_t *data, size_t length, const ITransportAddress &from) {
        this->HandlePacket(data, length, from);
      });

  // Start receiving
  if (!transport_->StartReceiving()) {
    LOG_ERROR("CommandChannel::Start: transport StartReceiving failed\n");
    return false;
  }

  // Start timeout cleanup thread
  running_.store(true);
  timeout_thread_ = std::thread(&CommandChannel::TimeoutLoop, this);

  started_ = true;
  return true;
}

void CommandChannel::Stop() {
  if (!started_) {
    return;
  }

  // Stop timeout thread
  running_.store(false);
  if (timeout_thread_.joinable()) {
    timeout_thread_.join();
  }

  // Stop transport
  transport_->StopReceiving();
  transport_->Close();

  // Wait for active callbacks
  {
    std::unique_lock<std::mutex> lock(active_mutex_);
    waiting_for_callbacks_ = true;
    active_cv_.wait(lock, [this]() { return active_callbacks_.load() == 0; });
    waiting_for_callbacks_ = false;
  }

  started_ = false;

  // Notify pending commands
  std::vector<std::pair<uint16_t, PendingCommand>> leftovers;
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    for (auto &kv : pending_) {
      leftovers.push_back(kv);
    }
    pending_.clear();
  }
  for (const auto &item : leftovers) {
    if (item.second.callback) {
      OdinCommandResponse response;
      response.device = config_.device_handle;
      response.seq = item.first;
      response.cmd_id = item.second.cmd_id;
      item.second.callback(OdinResult::kStopped, &response, item.second.client_data);
    }
  }
}

bool CommandChannel::SendAsync(uint16_t cmd_id, const std::vector<uint8_t> &payload,
                               OdinCommandCallback callback, void *client_data,
                               uint32_t timeout_ms) {
  if (!started_ || !transport_->IsOpen()) {
    return false;
  }

  uint16_t seq = next_seq_.fetch_add(1);
  if (timeout_ms == 0) {
    timeout_ms = config_.default_timeout_ms;
  }

  const uint8_t *payload_ptr = payload.empty() ? nullptr : payload.data();
  std::vector<uint8_t> frame;

  // Create packet using IPacket interface
  auto pkt = protocol_->CreatePacket();
  pkt->SetCommandId(cmd_id);
  pkt->SetSequence(seq);
  pkt->SetAttribute(PacketAttr::kCmdType, PacketAttr::kCmdTypeReq);
  pkt->SetAttribute(PacketAttr::kSendType, PacketAttr::kSendTypeHost);
  if (payload_ptr && payload.size() > 0) {
    pkt->SetPayload(payload_ptr, payload.size());
  }

  if (!protocol_->Pack(*pkt, frame)) {
    return false;
  }

  // Send via transport (uses pre-configured remote target)
  int sent = transport_->Send(frame.data(), frame.size());
  if (sent < 0 || static_cast<size_t>(sent) != frame.size()) {
    return false;
  }

  PendingCommand pending;
  pending.callback = callback;
  pending.client_data = client_data;
  pending.cmd_id = cmd_id;
  pending.deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    pending_[seq] = pending;
  }

  return true;
}

bool CommandChannel::SendSync(uint16_t cmd_id, const std::vector<uint8_t> &payload,
                              OdinCommandSyncResponse &sr, uint32_t timeout_ms) {
  std::promise<OdinCommandSyncResponse> pending_promises;

  auto cb = [](OdinResult result, const OdinCommandResponse *response, void *client_data) {
    std::promise<OdinCommandSyncResponse> *prom =
        static_cast<std::promise<OdinCommandSyncResponse> *>(client_data);
    OdinCommandSyncResponse sr;
    sr.result = result;
    sr.response = std::move(*response);
    prom->set_value(std::move(sr));
  };

  if (false == SendAsync(cmd_id, payload, cb, &pending_promises, timeout_ms)) {
    return false;
  }

  std::future<OdinCommandSyncResponse> fut = pending_promises.get_future();
  if (fut.wait_for(std::chrono::milliseconds(timeout_ms * 2)) == std::future_status::ready) {
    sr = fut.get();
  } else {
    return false;
  }

  return true;
}

void CommandChannel::HandlePacket(const uint8_t *data, size_t length,
                                  const ITransportAddress &from) {
  // Note: Source filtering is now transport's responsibility if needed
  // CommandChannel is transport-agnostic
  (void)from;  // Unused, filtering done at transport level if required

  active_callbacks_.fetch_add(1);

  std::unique_ptr<IPacket> packet;
  int consumed = protocol_->Parse(data, length, packet);
  if (consumed <= 0 || !packet) {
    active_callbacks_.fetch_sub(1);
    return;
  }

  // Use IPacket interface for polymorphism
  if (packet->GetAttribute(PacketAttr::kCmdType) != PacketAttr::kCmdTypeAck) {
    active_callbacks_.fetch_sub(1);
    return;
  }

  PendingCommand pending;
  bool found = false;
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    auto it = pending_.find(static_cast<uint16_t>(packet->GetSequence()));
    if (it != pending_.end()) {
      pending = it->second;
      pending_.erase(it);
      found = true;
    }
  }

  if (!found || pending.callback == nullptr) {
    if (active_callbacks_.fetch_sub(1) == 1) {
      std::lock_guard<std::mutex> lock(active_mutex_);
      if (waiting_for_callbacks_) {
        active_cv_.notify_all();
      }
    }
    return;
  }

  OdinCommandResponse response;
  response.seq = static_cast<uint16_t>(packet->GetSequence());
  response.cmd_id = static_cast<uint16_t>(packet->GetCommandId());
  response.device = config_.device_handle;
  if (packet->GetPayload() && packet->GetPayloadSize() > 0) {
    response.payload.assign(packet->GetPayload(), packet->GetPayload() + packet->GetPayloadSize());
  }

  if (pending.callback) {
    pending.callback(OdinResult::kOk, &response, pending.client_data);
  }

  if (active_callbacks_.fetch_sub(1) == 1) {
    std::lock_guard<std::mutex> lock(active_mutex_);
    if (waiting_for_callbacks_) {
      active_cv_.notify_all();
    }
  }
}

void CommandChannel::CleanupTimeouts() {
  std::vector<std::pair<PendingCommand, uint16_t>> expired;
  auto now = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    for (auto it = pending_.begin(); it != pending_.end();) {
      if (it->second.deadline <= now) {
        expired.push_back(std::make_pair(it->second, it->first));
        it = pending_.erase(it);
      } else {
        ++it;
      }
    }
  }

  for (const auto &item : expired) {
    const PendingCommand &pending = item.first;
    if (pending.callback) {
      OdinCommandResponse response;
      response.device = config_.device_handle;
      response.seq = item.second;
      response.cmd_id = pending.cmd_id;
      pending.callback(OdinResult::kTimeout, &response, pending.client_data);
    }
  }
}

void CommandChannel::TimeoutLoop() {
  while (running_.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    CleanupTimeouts();
  }
}

}  // namespace sdk
}  // namespace odin
