#include "UdpTransport.hpp"
#include <cstring>
#include <vector>

namespace odin {
namespace sdk {

UdpTransport::~UdpTransport() {
  StopReceiving();
  Close();
}

bool UdpTransport::Open(const ITransportAddress& local_address) {
  if (socket_ != kInvalidSocketHandle) {
    return true;  // Already open
  }

  // Cast to NetworkAddress (UDP requires network address)
  const NetworkAddress* net_addr = dynamic_cast<const NetworkAddress*>(&local_address);
  if (!net_addr) {
    return false;  // Invalid address type
  }

  socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_ == kInvalidSocketHandle) {
    return false;
  }

  sockaddr_in local_addr;
  std::memset(&local_addr, 0, sizeof(local_addr));
  local_addr.sin_family = AF_INET;
  local_addr.sin_port = htons(net_addr->port);

  if (net_addr->ip.empty() || net_addr->ip == "0.0.0.0") {
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  } else {
    if (inet_pton(AF_INET, net_addr->ip.c_str(), &local_addr.sin_addr) != 1) {
      Close();
      return false;
    }
  }

  if (::bind(socket_, reinterpret_cast<sockaddr*>(&local_addr), sizeof(local_addr)) != 0) {
    Close();
    return false;
  }

  // Set receive buffer size to 8MB to reduce packet loss on high-speed data transfer
  // This requires net.core.rmem_max >= 8388608 on Linux
  int rcvbuf_size = 8 * 1024 * 1024;  // 8MB
  if (setsockopt(socket_, SOL_SOCKET, SO_RCVBUF, 
                 reinterpret_cast<const char*>(&rcvbuf_size), sizeof(rcvbuf_size)) != 0) {
    // Failed to set buffer size, continue anyway with default
  }

  local_address_ = *net_addr;
  return true;
}

void UdpTransport::SetRemoteTarget(const ITransportAddress& remote_address) {
  const NetworkAddress* net_addr = dynamic_cast<const NetworkAddress*>(&remote_address);
  if (net_addr) {
    remote_address_ = *net_addr;
  }
}

void UdpTransport::Close() {
  if (socket_ != kInvalidSocketHandle) {
#ifdef _WIN32
    closesocket(socket_);
#else
    ::close(socket_);
#endif
    socket_ = kInvalidSocketHandle;
  }
}

bool UdpTransport::IsOpen() const { return socket_ != kInvalidSocketHandle; }

int UdpTransport::Send(const uint8_t* data, size_t length) {
  if (socket_ == kInvalidSocketHandle) {
    return -1;
  }

  if (remote_address_.ip.empty()) {
    return -1;  // Remote target not set
  }

  sockaddr_in dest_addr;
  std::memset(&dest_addr, 0, sizeof(dest_addr));
  dest_addr.sin_family = AF_INET;
  dest_addr.sin_port = htons(remote_address_.port);
  if (inet_pton(AF_INET, remote_address_.ip.c_str(), &dest_addr.sin_addr) != 1) {
    return -1;
  }

  int sent = 0;
  {
    std::lock_guard<std::mutex> lock(send_mutex_);
    sent = sendto(socket_, reinterpret_cast<const char*>(data), static_cast<int>(length), 0,
                  reinterpret_cast<sockaddr*>(&dest_addr), sizeof(dest_addr));
  }
  return sent;
}

void UdpTransport::SetReceiveCallback(TransportReceiveCallback callback) {
  std::lock_guard<std::mutex> lock(callback_mutex_);
  receive_callback_ = std::move(callback);
}

bool UdpTransport::StartReceiving() {
  if (socket_ == kInvalidSocketHandle) {
    return false;
  }
  if (running_.load()) {
    return true;  // Already running
  }

  running_.store(true);
  receive_thread_ = std::thread(&UdpTransport::ReceiveLoop, this);
  return true;
}

void UdpTransport::StopReceiving() {
  if (!running_.load()) {
    return;
  }

  running_.store(false);

  // Shutdown socket to unblock recvfrom
  if (socket_ != kInvalidSocketHandle) {
#ifdef _WIN32
    shutdown(socket_, SD_BOTH);
#else
    shutdown(socket_, SHUT_RDWR);
#endif
  }

  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
}

void UdpTransport::ReceiveLoop() {
  std::vector<uint8_t> buffer(2048);

  while (running_.load()) {
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(socket_, &read_fds);

    timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 200 * 1000;  // 200ms timeout

#ifdef _WIN32
    int ready = select(0, &read_fds, nullptr, nullptr, &tv);
#else
    int ready = select(static_cast<int>(socket_ + 1), &read_fds, nullptr, nullptr, &tv);
#endif

    if (ready > 0 && FD_ISSET(socket_, &read_fds)) {
      sockaddr_in from_addr;
      socklen_t addr_len = sizeof(from_addr);
      int len =
          recvfrom(socket_, reinterpret_cast<char*>(buffer.data()), static_cast<int>(buffer.size()),
                   0, reinterpret_cast<sockaddr*>(&from_addr), &addr_len);

      if (len > 0) {
        NetworkAddress from;
        char ip_str[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &from_addr.sin_addr, ip_str, sizeof(ip_str));
        from.ip = ip_str;
        from.port = ntohs(from_addr.sin_port);

        TransportReceiveCallback cb;
        {
          std::lock_guard<std::mutex> lock(callback_mutex_);
          cb = receive_callback_;
        }
        if (cb) {
          cb(buffer.data(), static_cast<size_t>(len), from);
        }
      }
    }
  }
}

}  // namespace sdk
}  // namespace odin
