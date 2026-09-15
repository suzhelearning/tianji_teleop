#pragma once

#include <string>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace odin {
namespace sdk {
namespace internal {

#ifdef _WIN32
using SocketHandle = SOCKET;
static const SocketHandle kInvalidSocket = INVALID_SOCKET;
inline void CloseSocket(SocketHandle socket_handle) {
  if (socket_handle != kInvalidSocket) {
    closesocket(socket_handle);
  }
}
inline void ShutdownSocket(SocketHandle socket_handle) {
  if (socket_handle != kInvalidSocket) {
    shutdown(socket_handle, SD_BOTH);
  }
}
#else
using SocketHandle = int;
static const SocketHandle kInvalidSocket = -1;
inline void CloseSocket(SocketHandle socket_handle) {
  if (socket_handle != kInvalidSocket) {
    close(socket_handle);
  }
}
inline void ShutdownSocket(SocketHandle socket_handle) {
  if (socket_handle != kInvalidSocket) {
    shutdown(socket_handle, SHUT_RDWR);
  }
}
#endif

inline bool ParseIpv4(const std::string &ip, in_addr *addr) {
  if (ip.empty() || ip == "0.0.0.0") {
    addr->s_addr = htonl(INADDR_ANY);
    return true;
  }
  return inet_pton(AF_INET, ip.c_str(), addr) == 1;
}

}  // namespace internal
}  // namespace sdk
}  // namespace odin
