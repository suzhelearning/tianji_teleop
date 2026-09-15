#pragma once

#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>
#include <vector>

#include "OdinProtocol.hpp"
#include "odin_lidar_def.h"

namespace odin {
namespace sdk {

/**
 * @brief TCP stream buffer for frame reassembly
 */
struct TcpStreamBuffer {
  std::vector<uint8_t> buffer;
  
  void Append(const uint8_t* data, size_t len) {
    buffer.insert(buffer.end(), data, data + len);
  }
  
  void Clear() { buffer.clear(); }
  
  size_t Size() const { return buffer.size(); }
  
  const uint8_t* Data() const { return buffer.data(); }
  
  void Consume(size_t len) {
    if (len >= buffer.size()) {
      buffer.clear();
    } else {
      buffer.erase(buffer.begin(), buffer.begin() + len);
    }
  }
};

/**
 * @brief Point cloud frame assembly state
 */
struct PointFrameState {
  OdinPointCloudPacket packet;
  uint16_t last_udp_seq = 0;
  uint16_t udp_segments = 0;
  bool active = false;
  
  void Reset() {
    packet = OdinPointCloudPacket();
    last_udp_seq = 0;
    udp_segments = 0;
    active = false;
  }
};

/**
 * @brief Image frame assembly state
 */
struct ImageFrameState {
  OdinImagePacket packet;
  uint16_t last_udp_seq = 0;
  uint16_t udp_segments = 0;
  uint8_t last_byte = 0;
  bool last_byte_valid = false;
  bool active = false;
  
  void Reset() {
    packet = OdinImagePacket();
    last_udp_seq = 0;
    udp_segments = 0;
    last_byte = 0;
    last_byte_valid = false;
    active = false;
  }
};

/**
 * @brief Odin protocol frame assembler
 * 
 * Handles frame assembly from UDP/TCP packets for different data types.
 * Supports point cloud, image, IMU, and odometry data.
 */
class OdinFrameAssembler {
  friend class OdinFrameAssemblerTestPeer;

 public:
  OdinFrameAssembler(OdinDeviceHandle handle, bool use_tcp);
  ~OdinFrameAssembler() = default;

  // Callback types
  using PointCloudCallback = std::function<void(const OdinPointCloudPacket&)>;
  using SlamCallback = std::function<void(const OdinSlamPacket&)>;
  using ImageCallback = std::function<void(const OdinImagePacket&)>;
  using ImuCallback = std::function<void(const OdinImuPacket&)>;
  using OdomCallback = std::function<void(const OdinOdomPacket&, OdomSourceType)>;

  // Set callbacks
  void SetPointCloudCallback(PointCloudCallback cb) { point_cloud_cb_ = cb; }
  void SetSlamCallback(SlamCallback cb) { slam_cb_ = cb; }
  void SetImageCallback(ImageCallback cb) { image_cb_ = cb; }
  void SetImuCallback(ImuCallback cb) { imu_cb_ = cb; }
  void SetOdomCallback(OdomCallback cb) { odom_cb_ = cb; }

  // Process raw data for different channel types
  void ProcessRawPointData(const uint8_t* data, size_t length);
  void ProcessSlamPointData(const uint8_t* data, size_t length);
  void ProcessImageData(const uint8_t* data, size_t length);
  void ProcessImuData(const uint8_t* data, size_t length);
  void ProcessOdomData(const uint8_t* data, size_t length);

  // Reset state
  void Reset();

 private:
  // Parsing helpers
  bool ParsePointCloudPacket(const uint8_t* data, size_t length, OdinPointCloudPacket* packet);
  bool ParseImagePacket(const uint8_t* data, size_t length, OdinImagePacket* packet);
  bool ParseImuPacket(const uint8_t* data, size_t length, OdinImuPacket* packet);
  bool ParseOdomPacket(const uint8_t* data, size_t length, OdinOdomPacket* packet);

  // Frame assembly
  void ProcessPointFragment(const OdinPointCloudPacket& fragment, bool is_slam);
  void EmitPointFrame(bool is_slam);
  void ProcessImageFragment(const OdinImagePacket& fragment);
  void EmitImageFrame();
  bool DetectJpegTerminator(const std::vector<uint8_t>& chunk);
  bool FillLostPacketsWithZeros(uint16_t lost_packets, size_t points_per_packet, bool is_slam);
  uint16_t ExpectedPointUdpCount(bool is_slam) const;
  size_t MaxPointPayloadBytes(bool is_slam) const;

  // TCP buffer processing
  void ProcessTcpBuffer(std::function<void(const uint8_t*, size_t)> handler);

  OdinDeviceHandle handle_;
  bool use_tcp_;

  // Frame states
  PointFrameState raw_point_state_;
  PointFrameState slam_point_state_;
  ImageFrameState image_state_;
  std::mutex frame_mutex_;

  // TCP buffer
  TcpStreamBuffer tcp_buffer_;
  std::mutex tcp_buffer_mutex_;

  // Callbacks
  PointCloudCallback point_cloud_cb_;
  SlamCallback slam_cb_;
  ImageCallback image_cb_;
  ImuCallback imu_cb_;
  OdomCallback odom_cb_;
};

}  // namespace sdk
}  // namespace odin
