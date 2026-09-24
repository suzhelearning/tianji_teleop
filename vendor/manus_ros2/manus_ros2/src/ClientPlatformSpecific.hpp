#ifndef MANUS_ROS2_CLIENT_PLATFORM_SPECIFIC_HPP
#define MANUS_ROS2_CLIENT_PLATFORM_SPECIFIC_HPP

#include <cstdint>
#include <string>

// ROS owns signals and console handling; only acquisition platform services
// remain from the sample's SDKClientPlatformSpecific boundary.
class SDKClientPlatformSpecific {
protected:
  void AcquireSdkOwnership();
  void ReleaseSdkOwnership() noexcept;
  static int64_t MonotonicNs();
  static std::string ReadUuid(const char * path);
private:
  int m_OwnerSocket = -1;
};

#endif
