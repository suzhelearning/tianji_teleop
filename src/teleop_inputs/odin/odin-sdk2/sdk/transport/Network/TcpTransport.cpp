#include "TcpTransport.hpp"

#include <cstring>
#include <vector>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include "logger.h"

namespace odin {
namespace sdk {

TcpTransport::TcpTransport() = default;

TcpTransport::~TcpTransport() { Close(); }

bool TcpTransport::Open(const ITransportAddress& local_address) {
  const auto* addr = dynamic_cast<const NetworkAddress*>(&local_address);
  if (!addr) {
    LOG_ERROR("TcpTransport::Open: invalid address type\n");
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (listen_socket_ >= 0 || client_socket_ >= 0) {
    LOG_WARN("TcpTransport::Open: already open\n");
    return true;
  }

  local_addr_ = *addr;

  // Check if remote target is set - if so, use Client mode
  if (!remote_addr_.ip.empty() && remote_addr_.port != 0) {
    return OpenAsClient();
  }

  // Server mode: listen for incoming connections
  return OpenAsServer();
}

bool TcpTransport::OpenAsClient() {
  // Create TCP socket
  client_socket_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (client_socket_ < 0) {
    LOG_ERROR("TcpTransport::OpenAsClient: socket() failed\n");
    return false;
  }

  // Note: We use select() for receive timeout in ReceiveLoop, not SO_RCVTIMEO
  // Set send timeout only
  struct timeval tv;
  tv.tv_sec = 3;  // 3 second timeout for send
  tv.tv_usec = 0;
  setsockopt(client_socket_, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char*>(&tv), sizeof(tv));

  // Connect to remote server
  sockaddr_in server_addr{};
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(remote_addr_.port);
  if (inet_pton(AF_INET, remote_addr_.ip.c_str(), &server_addr.sin_addr) <= 0) {
    LOG_ERROR("TcpTransport::OpenAsClient: invalid IP address %s\n", remote_addr_.ip.c_str());
#ifdef _WIN32
    closesocket(client_socket_);
#else
    close(client_socket_);
#endif
    client_socket_ = -1;
    return false;
  }

  if (connect(client_socket_, reinterpret_cast<sockaddr*>(&server_addr), sizeof(server_addr)) < 0) {
    LOG_ERROR("TcpTransport::OpenAsClient: connect() failed to %s:%u\n", 
              remote_addr_.ip.c_str(), remote_addr_.port);
#ifdef _WIN32
    closesocket(client_socket_);
#else
    close(client_socket_);
#endif
    client_socket_ = -1;
    return false;
  }

  connected_.store(true);
  LOG_INFO("TcpTransport: connected to %s:%u (client mode)\n", remote_addr_.ip.c_str(), remote_addr_.port);
  return true;
}

bool TcpTransport::OpenAsServer() {
  // Create TCP socket
  listen_socket_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (listen_socket_ < 0) {
    LOG_ERROR("TcpTransport::OpenAsServer: socket() failed\n");
    return false;
  }

  // Allow address reuse
  int opt = 1;
  setsockopt(listen_socket_, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&opt),
             sizeof(opt));

  // Bind to local address
  sockaddr_in bind_addr{};
  bind_addr.sin_family = AF_INET;
  bind_addr.sin_port = htons(local_addr_.port);
  if (local_addr_.ip.empty() || local_addr_.ip == "0.0.0.0") {
    bind_addr.sin_addr.s_addr = INADDR_ANY;
  } else {
    inet_pton(AF_INET, local_addr_.ip.c_str(), &bind_addr.sin_addr);
  }

  if (bind(listen_socket_, reinterpret_cast<sockaddr*>(&bind_addr), sizeof(bind_addr)) < 0) {
    LOG_ERROR("TcpTransport::OpenAsServer: bind() failed on port %u\n", local_addr_.port);
#ifdef _WIN32
    closesocket(listen_socket_);
#else
    close(listen_socket_);
#endif
    listen_socket_ = -1;
    return false;
  }

  // Start listening
  if (listen(listen_socket_, 1) < 0) {
    LOG_ERROR("TcpTransport::OpenAsServer: listen() failed\n");
#ifdef _WIN32
    closesocket(listen_socket_);
#else
    close(listen_socket_);
#endif
    listen_socket_ = -1;
    return false;
  }

  LOG_INFO("TcpTransport: listening on port %u (server mode)\n", local_addr_.port);
  return true;
}

bool TcpTransport::Connect(const ITransportAddress& remote_address) {
  const auto* addr = dynamic_cast<const NetworkAddress*>(&remote_address);
  if (!addr) {
    LOG_ERROR("TcpTransport::Connect: invalid address type\n");
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  
  // Close existing sockets if any
  if (client_socket_ >= 0) {
#ifdef _WIN32
    closesocket(client_socket_);
#else
    close(client_socket_);
#endif
    client_socket_ = -1;
  }
  if (listen_socket_ >= 0) {
#ifdef _WIN32
    closesocket(listen_socket_);
#else
    close(listen_socket_);
#endif
    listen_socket_ = -1;
  }

  remote_addr_ = *addr;

  // Create TCP socket
  client_socket_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (client_socket_ < 0) {
    LOG_ERROR("TcpTransport::Connect: socket() failed\n");
    return false;
  }

  // Set connection timeout
  struct timeval tv;
  tv.tv_sec = 3;  // 3 second timeout
  tv.tv_usec = 0;
  setsockopt(client_socket_, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&tv), sizeof(tv));
  setsockopt(client_socket_, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char*>(&tv), sizeof(tv));

  // Connect to remote server
  sockaddr_in server_addr{};
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(remote_addr_.port);
  if (inet_pton(AF_INET, remote_addr_.ip.c_str(), &server_addr.sin_addr) <= 0) {
    LOG_ERROR("TcpTransport::Connect: invalid IP address %s\n", remote_addr_.ip.c_str());
#ifdef _WIN32
    closesocket(client_socket_);
#else
    close(client_socket_);
#endif
    client_socket_ = -1;
    return false;
  }

  if (connect(client_socket_, reinterpret_cast<sockaddr*>(&server_addr), sizeof(server_addr)) < 0) {
    LOG_ERROR("TcpTransport::Connect: connect() failed to %s:%u\n", 
              remote_addr_.ip.c_str(), remote_addr_.port);
#ifdef _WIN32
    closesocket(client_socket_);
#else
    close(client_socket_);
#endif
    client_socket_ = -1;
    return false;
  }

  connected_.store(true);
  LOG_INFO("TcpTransport: connected to %s:%u\n", remote_addr_.ip.c_str(), remote_addr_.port);
  return true;
}

void TcpTransport::SetRemoteTarget(const ITransportAddress& remote_address) {
  const auto* addr = dynamic_cast<const NetworkAddress*>(&remote_address);
  if (addr) {
    std::lock_guard<std::mutex> lock(mutex_);
    remote_addr_ = *addr;
  }
}

void TcpTransport::Close() {
  StopReceiving();

  std::lock_guard<std::mutex> lock(mutex_);
  if (client_socket_ >= 0) {
#ifdef _WIN32
    closesocket(client_socket_);
#else
    close(client_socket_);
#endif
    client_socket_ = -1;
  }

  if (listen_socket_ >= 0) {
#ifdef _WIN32
    closesocket(listen_socket_);
#else
    close(listen_socket_);
#endif
    listen_socket_ = -1;
  }

  connected_.store(false);
  LOG_INFO("TcpTransport: closed\n");
}

bool TcpTransport::IsOpen() const {
  std::lock_guard<std::mutex> lock(mutex_);
  // Open in either server mode (listen_socket_) or client mode (client_socket_)
  return listen_socket_ >= 0 || client_socket_ >= 0;
}

int TcpTransport::Send(const uint8_t* data, size_t length) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (client_socket_ < 0) {
    return -1;
  }
  return static_cast<int>(send(client_socket_, reinterpret_cast<const char*>(data), length, 0));
}

void TcpTransport::SetReceiveCallback(TransportReceiveCallback callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  receive_callback_ = std::move(callback);
}

bool TcpTransport::StartReceiving() {
  if (running_.load()) {
    return true;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    // Client mode: already connected, start receive loop directly
    if (client_socket_ >= 0 && connected_.load()) {
      running_.store(true);
      receive_thread_ = std::thread(&TcpTransport::ReceiveLoop, this);
      return true;
    }
    // Server mode: need listen socket to accept connections
    if (listen_socket_ < 0) {
      LOG_ERROR("TcpTransport::StartReceiving: not open\n");
      return false;
    }
  }

  // Server mode: start accept loop
  running_.store(true);
  accept_thread_ = std::thread(&TcpTransport::AcceptLoop, this);
  return true;
}

void TcpTransport::StopReceiving() {
  running_.store(false);

  // Close sockets to unblock accept/recv
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (client_socket_ >= 0) {
#ifdef _WIN32
      shutdown(client_socket_, SD_BOTH);
#else
      shutdown(client_socket_, SHUT_RDWR);
#endif
    }
    if (listen_socket_ >= 0) {
#ifdef _WIN32
      shutdown(listen_socket_, SD_BOTH);
#else
      shutdown(listen_socket_, SHUT_RDWR);
#endif
    }
  }

  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
  if (accept_thread_.joinable()) {
    accept_thread_.join();
  }
}

void TcpTransport::AcceptLoop() {
  while (running_.load()) {
    sockaddr_in client_addr{};
    socklen_t addr_len = sizeof(client_addr);

    int sock;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (listen_socket_ < 0) break;
      sock = listen_socket_;
    }

    int new_client = accept(sock, reinterpret_cast<sockaddr*>(&client_addr), &addr_len);
    if (new_client < 0) {
      if (running_.load()) {
        LOG_WARN("TcpTransport::AcceptLoop: accept() failed\n");
      }
      continue;
    }

    char ip_str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &client_addr.sin_addr, ip_str, sizeof(ip_str));
    LOG_INFO("TcpTransport: client connected from %s:%u\n", ip_str, ntohs(client_addr.sin_port));

    {
      std::lock_guard<std::mutex> lock(mutex_);
      // Close previous client if any
      if (client_socket_ >= 0) {
#ifdef _WIN32
        closesocket(client_socket_);
#else
        close(client_socket_);
#endif
      }
      client_socket_ = new_client;
      connected_.store(true);
    }

    // Start receive thread for this client
    if (receive_thread_.joinable()) {
      receive_thread_.join();
    }
    receive_thread_ = std::thread(&TcpTransport::ReceiveLoop, this);
  }
}

void TcpTransport::ReceiveLoop() {
  constexpr size_t kBufferSize = 65536;
  std::vector<uint8_t> buffer(kBufferSize);

  while (running_.load() && connected_.load()) {
    int sock;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      sock = client_socket_;
    }
    if (sock < 0) break;

    // Use select() to wait for data with timeout (same pattern as UdpTransport)
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(sock, &read_fds);

    timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 200 * 1000;  // 200ms timeout

#ifdef _WIN32
    int ready = select(0, &read_fds, nullptr, nullptr, &tv);
#else
    int ready = select(sock + 1, &read_fds, nullptr, nullptr, &tv);
#endif

    if (ready < 0) {
      // select error
      if (running_.load()) {
        LOG_WARN("TcpTransport: select error, disconnecting\n");
      }
      connected_.store(false);
      break;
    }

    if (ready == 0) {
      // Timeout, no data available - continue waiting
      continue;
    }

    // Data available, read it
    if (FD_ISSET(sock, &read_fds)) {
      ssize_t received = recv(sock, reinterpret_cast<char*>(buffer.data()), buffer.size(), 0);
      
      if (received < 0) {
        // recv error
        if (running_.load()) {
          LOG_WARN("TcpTransport: recv error, disconnecting\n");
        }
        connected_.store(false);
        break;
      }
      
      if (received == 0) {
        // Peer closed connection gracefully
        if (running_.load()) {
          LOG_INFO("TcpTransport: peer closed connection\n");
        }
        connected_.store(false);
        break;
      }

      TransportReceiveCallback cb;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        cb = receive_callback_;
      }

      if (cb) {
        cb(buffer.data(), static_cast<size_t>(received), local_addr_);
      }
    }
  }
}

}  // namespace sdk
}  // namespace odin
