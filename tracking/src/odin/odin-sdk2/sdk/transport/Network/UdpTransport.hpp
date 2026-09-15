#pragma once

#include "../ITransport/ITransport.hpp"
#include <atomic>
#include <mutex>
#include <thread>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
using SocketHandle = SOCKET;
constexpr SocketHandle kInvalidSocketHandle = INVALID_SOCKET;
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
using SocketHandle = int;
constexpr SocketHandle kInvalidSocketHandle = -1;
#endif

namespace odin {
namespace sdk {

/**
 * @brief UDP transport implementation
 */
class UdpTransport : public ITransport {
 public:
  UdpTransport() = default;
  ~UdpTransport() override;

  // ITransport interface
  bool Open(const ITransportAddress& local_address) override;
  void SetRemoteTarget(const ITransportAddress& remote_address) override;
  void Close() override;
  bool IsOpen() const override;
  int Send(const uint8_t* data, size_t length) override;
  void SetReceiveCallback(TransportReceiveCallback callback) override;
  bool StartReceiving() override;
  void StopReceiving() override;
  const char* GetTransportName() const override { return "UDP"; }

  // UDP-specific methods
  SocketHandle GetSocket() const { return socket_; }

 private:
  void ReceiveLoop();

  SocketHandle socket_ = kInvalidSocketHandle;
  NetworkAddress local_address_;
  NetworkAddress remote_address_;
  TransportReceiveCallback receive_callback_;

  std::thread receive_thread_;
  std::atomic<bool> running_{false};
  std::mutex send_mutex_;
  std::mutex callback_mutex_;
};

}  // namespace sdk
}  // namespace odin
