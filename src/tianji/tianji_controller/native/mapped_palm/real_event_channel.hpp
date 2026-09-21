#pragma once
#include "tianji_mapped_palm/joint_command.hpp"
#include <sys/socket.h>
#include <array>
#include <cstring>
#include <string>
#include <stdexcept>

// Private inherited SOCK_SEQPACKET endpoint: atomic event + unchanged TJRC v2.
// Never replace a missing event with a synthetic ready command.
inline void sendRealEvent(int fd,const std::string& token,unsigned state,
                         std::uint64_t tick,std::uint64_t generation,
                         const tianji_mapped_palm::JointCommandFrame& frame,
                         bool calibration=false,std::uint64_t revision=0,
                         std::uint64_t calibration_epoch=0,unsigned calibration_state=0,
                         const std::array<double,4>& offsets={},
                         bool dropout=false,std::int64_t input_valid_ns=0) {
  const auto command=tianji_mapped_palm::encodeJointCommandPacket(frame);
  std::array<unsigned char,144+tianji_mapped_palm::kJointCommandPacketSize> bytes{};
  const unsigned header=dropout?144:calibration?136:80;
  const auto size=header+command.size();
  std::memcpy(bytes.data(),"MRE1",4); bytes[4]=1; bytes[5]=state;
  auto put=[&](unsigned offset,std::uint64_t v,unsigned n) {
    for(unsigned i=0;i<n;++i) bytes[offset+i]=(v>>(8*i))&255;
  };
  bytes[4]=dropout?3:calibration?2:1;
  put(6,size,2); std::memcpy(bytes.data()+8,token.data(),32);
  put(40,frame.sequence,8); put(48,tick,8); put(56,frame.source_timestamp_ns,8);
  put(64,frame.pico_tracking_epoch,8); put(72,generation,8);
  if(calibration) {
    put(80,revision,8); put(88,calibration_epoch,8); put(96,calibration_state,8);
    for(unsigned i=0;i<4;++i) { std::uint64_t bits; std::memcpy(&bits,&offsets[i],8); put(104+8*i,bits,8); }
  }
  if(dropout) put(136,static_cast<std::uint64_t>(input_valid_ns),8);
  std::memcpy(bytes.data()+header,command.data(),command.size());
  if(send(fd,bytes.data(),size,MSG_DONTWAIT|MSG_NOSIGNAL)!=static_cast<ssize_t>(size))
    throw std::runtime_error("real event channel blocked/closed; stopping controller");
}
