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

#include "ClientPlatformSpecific.hpp"
#include <cstddef>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <sys/socket.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

void SDKClientPlatformSpecific::AcquireSdkOwnership()
{
  if (m_OwnerSocket >= 0) {throw std::runtime_error("MANUS SDK ownership already acquired");}
  m_OwnerSocket = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
  if (m_OwnerSocket < 0) {throw std::runtime_error("Cannot create MANUS SDK owner lock");}
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  constexpr char owner[] = "tianji.manus.integrated.sdk.owner";
  std::memcpy(address.sun_path + 1, owner, sizeof(owner) - 1);
  // Abstract socket is machine/network-namespace scoped, independent of ROS
  // domain, checkout, user and replaceable filesystem lock inodes.
  if (bind(m_OwnerSocket, reinterpret_cast<sockaddr *>(&address),
    offsetof(sockaddr_un, sun_path) + sizeof(owner)) != 0)
  {
    ReleaseSdkOwnership();
    throw std::runtime_error("MANUS Integrated SDK already owned (or owner lock unavailable)");
  }
}

void SDKClientPlatformSpecific::ReleaseSdkOwnership() noexcept
{
  if (m_OwnerSocket >= 0) {close(m_OwnerSocket); m_OwnerSocket = -1;}
}

int64_t SDKClientPlatformSpecific::MonotonicNs()
{
  timespec value{};
  if (clock_gettime(CLOCK_MONOTONIC, &value) != 0) {
    throw std::runtime_error("CLOCK_MONOTONIC unavailable");
  }
  return static_cast<int64_t>(value.tv_sec) * 1000000000LL + value.tv_nsec;
}

std::string SDKClientPlatformSpecific::ReadUuid(const char * path)
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
