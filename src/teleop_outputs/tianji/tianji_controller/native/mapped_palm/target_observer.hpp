#pragma once
#include "bilateral_cycle.hpp"
#include "tianji_mapped_palm/crc32.hpp"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <array>
#include <cstring>
#include <stdexcept>

// Observation only, separate loopback port and non-TJRC magic. Best effort:
// renderer loss never grants/revokes motion permission or blocks computation.
class TargetObserver {
 public:
  explicit TargetObserver(unsigned port) {
    fd_=socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
    if(fd_<0) throw std::runtime_error("target observer socket failed");
    destination_.sin_family=AF_INET; destination_.sin_port=htons(static_cast<std::uint16_t>(port));
    destination_.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
  }
  ~TargetObserver() { if(fd_>=0) close(fd_); }
  TargetObserver(const TargetObserver&)=delete;
  TargetObserver& operator=(const TargetObserver&)=delete;
  void publish(const tianji_mapped_palm::BilateralCycleResult& r,std::uint64_t sequence,
               std::int64_t stamp,bool valid) noexcept {
    std::array<std::uint8_t,140> bytes{};
    std::memcpy(bytes.data(),"MPT1",4); bytes[4]=1; bytes[5]=valid?1:0;
    auto put=[&](std::size_t offset,std::uint64_t value,std::size_t n) {
      for(std::size_t i=0;i<n;++i) bytes[offset+i]=static_cast<std::uint8_t>((value>>(8*i))&255U);
    };
    put(6,bytes.size(),2); put(8,sequence,8); put(16,static_cast<std::uint64_t>(stamp),8);
    for(int side=0;side<2;++side) {
      const auto& target=side==0?r.control.left.target:r.control.right.target;
      Eigen::Vector3d p=Eigen::Vector3d::Zero(); Eigen::Quaterniond q=Eigen::Quaterniond::Identity();
      if(valid) {p=target.position;q=Eigen::Quaterniond(target.rotation);}
      const std::array<double,7> values{p.x(),p.y(),p.z(),q.x(),q.y(),q.z(),q.w()};
      for(std::size_t i=0;i<values.size();++i) {
        std::uint64_t bits{}; std::memcpy(&bits,&values[i],8); put(24+static_cast<std::size_t>(side)*56+i*8,bits,8);
      }
    }
    put(136,tianji_mapped_palm::crc32(bytes.data(),136),4);
    (void)sendto(fd_,bytes.data(),bytes.size(),MSG_DONTWAIT,reinterpret_cast<const sockaddr*>(&destination_),sizeof(destination_));
  }
 private:
  int fd_{-1}; sockaddr_in destination_{};
};
