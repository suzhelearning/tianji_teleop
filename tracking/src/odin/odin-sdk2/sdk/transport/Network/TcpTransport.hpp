#pragma once

#include <atomic>
#include <mutex>
#include <thread>

#include "ITransport.hpp"

namespace odin {
namespace sdk {

/**
 * @brief TCP Transport (supports both Server and Client modes)
 *
 * Server mode (Open): Listens for device connection on local port.
 *   - Used for data channels where device actively connects to host.
 *   - Per protocol design: when transport_mode=TCP, device TCP connects to host:dst_port
 *
 * Client mode (Connect): Actively connects to remote device.
 *   - Used for command channel where SDK connects to device's TCP port.
 */
class TcpTransport : public ITransport {
 public:
  TcpTransport();
  ~TcpTransport() override;

  // ITransport interface
  bool Open(const ITransportAddress& local_address) override;
  void SetRemoteTarget(const ITransportAddress& remote_address) override;
  void Close() override;
  bool IsOpen() const override;
  int Send(const uint8_t* data, size_t length) override;
  void SetReceiveCallback(TransportReceiveCallback callback) override;
  bool StartReceiving() override;
  void StopReceiving() override;
  const char* GetTransportName() const override { return "TCP"; }

  /**
   * @brief Connect to remote server (Client mode)
   * @param remote_address Remote server address to connect to
   * @return true if connection successful
   */
  bool Connect(const ITransportAddress& remote_address);

  /**
   * @brief Check if connected (Client mode)
   */
  bool IsConnected() const { return connected_.load(); }

 private:
  bool OpenAsClient();   // Client mode: connect to remote server
  bool OpenAsServer();   // Server mode: listen for connections
  void AcceptLoop();
  void ReceiveLoop();

  int listen_socket_ = -1;
  int client_socket_ = -1;
  std::atomic<bool> running_{false};
  std::atomic<bool> connected_{false};
  std::thread accept_thread_;
  std::thread receive_thread_;
  mutable std::mutex mutex_;
  TransportReceiveCallback receive_callback_;
  NetworkAddress local_addr_;
  NetworkAddress remote_addr_;
};

}  // namespace sdk
}  // namespace odin
