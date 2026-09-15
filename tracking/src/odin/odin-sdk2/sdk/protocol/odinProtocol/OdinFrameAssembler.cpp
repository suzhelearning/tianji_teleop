#include "OdinFrameAssembler.hpp"

#include <algorithm>

#include "logger.h"

namespace odin {
namespace sdk {

// Constants for Odin2 point cloud data
constexpr size_t kDataHeaderSize = OdinConst::kDataHeaderSize;
constexpr size_t kUdpMaxDataSize = 1472 - kDataHeaderSize;
constexpr uint16_t kPclWidth = 256;
constexpr uint16_t kPclHeight = 192;

OdinFrameAssembler::OdinFrameAssembler(OdinDeviceHandle handle, bool use_tcp)
    : handle_(handle), use_tcp_(use_tcp) {}

void OdinFrameAssembler::Reset() {
  std::lock_guard<std::mutex> lock(frame_mutex_);
  raw_point_state_.Reset();
  slam_point_state_.Reset();
  image_state_.Reset();
  
  std::lock_guard<std::mutex> tcp_lock(tcp_buffer_mutex_);
  tcp_buffer_.Clear();
}

// =============================================================================
// Parsing helpers
// =============================================================================

bool OdinFrameAssembler::ParsePointCloudPacket(const uint8_t* data, size_t length,
                                                OdinPointCloudPacket* packet) {
  if (!data || !packet || length < kDataHeaderSize) return false;

  const OdinDataFrameHeader* header = reinterpret_cast<const OdinDataFrameHeader*>(data);
  uint16_t frame_length = header->length;

  if (frame_length > length || frame_length < kDataHeaderSize) return false;

  size_t payload_size = frame_length - kDataHeaderSize;
  const uint8_t* payload_ptr = data + kDataHeaderSize;

  packet->version = header->version;
  packet->length = frame_length;
  packet->point_count = header->dot_or_sample_count;
  packet->udp_count = header->udp_count;
  packet->frame_count = header->frame_count;
  packet->data_type = header->data_type;
  packet->time_type = header->time_type;
  packet->reserved = header->reserved;
  packet->timestamp = header->timestamp;
  packet->payload.assign(payload_ptr, payload_ptr + payload_size);
  return true;
}

bool OdinFrameAssembler::ParseImagePacket(const uint8_t* data, size_t length,
                                           OdinImagePacket* packet) {
  if (!data || !packet || length < kDataHeaderSize) return false;

  const OdinDataFrameHeader* header = reinterpret_cast<const OdinDataFrameHeader*>(data);
  uint16_t frame_length = header->length;
  if (frame_length > length || frame_length < kDataHeaderSize) return false;

  size_t payload_size = frame_length - kDataHeaderSize;
  const uint8_t* payload_ptr = data + kDataHeaderSize;

  packet->version = header->version;
  packet->length = frame_length;
  packet->udp_count = header->udp_count;
  packet->frame_count = header->frame_count;
  packet->timestamp = header->timestamp;
  packet->payload.assign(payload_ptr, payload_ptr + payload_size);
  return true;
}

bool OdinFrameAssembler::ParseImuPacket(const uint8_t* data, size_t length,
                                         OdinImuPacket* packet) {
  if (!data || !packet || length < kDataHeaderSize) return false;

  const OdinDataFrameHeader* header = reinterpret_cast<const OdinDataFrameHeader*>(data);
  uint16_t frame_length = header->length;
  if (frame_length > length || frame_length < kDataHeaderSize) return false;

  size_t payload_size = frame_length - kDataHeaderSize;
  const uint8_t* payload_ptr = data + kDataHeaderSize;

  packet->version = header->version;
  packet->length = frame_length;
  packet->sample_count = header->dot_or_sample_count;
  packet->timestamp = header->timestamp;
  packet->payload.assign(payload_ptr, payload_ptr + payload_size);
  return true;
}

bool OdinFrameAssembler::ParseOdomPacket(const uint8_t* data, size_t length,
                                          OdinOdomPacket* packet) {
  if (!data || !packet || length < kDataHeaderSize) return false;

  const OdinDataFrameHeader* header = reinterpret_cast<const OdinDataFrameHeader*>(data);
  uint16_t frame_length = header->length;
  if (frame_length > length || frame_length < kDataHeaderSize) return false;

  size_t payload_size = frame_length - kDataHeaderSize;
  const uint8_t* payload_ptr = data + kDataHeaderSize;

  packet->version = header->version;
  packet->length = frame_length;
  packet->data_count = header->dot_or_sample_count;
  packet->frame_count = header->frame_count;
  packet->timestamp = header->timestamp;
  packet->payload.assign(payload_ptr, payload_ptr + payload_size);
  return true;
}

// =============================================================================
// TCP buffer processing
// =============================================================================

void OdinFrameAssembler::ProcessTcpBuffer(
    std::function<void(const uint8_t*, size_t)> handler) {
  std::lock_guard<std::mutex> lock(tcp_buffer_mutex_);

  while (tcp_buffer_.Size() >= kDataHeaderSize) {
    const uint8_t* buf = tcp_buffer_.Data();

    // Parse header to get frame length
    const OdinDataFrameHeader* header = reinterpret_cast<const OdinDataFrameHeader*>(buf);
    uint16_t frame_len = header->length;

    if (tcp_buffer_.Size() < frame_len) {
      // Incomplete frame, wait for more data
      break;
    }

    // Complete frame, process it
    handler(buf, frame_len);
    tcp_buffer_.Consume(frame_len);
  }
}

// =============================================================================
// Point cloud processing
// =============================================================================

uint16_t OdinFrameAssembler::ExpectedPointUdpCount(bool is_slam) const {
  constexpr size_t kTotalPoints = kPclWidth * kPclHeight;
  const size_t point_size =
      is_slam ? sizeof(OdinSlamPoint<uint16_t>) : sizeof(OdinRawPoint<uint16_t>);
  const size_t points_per_packet = kUdpMaxDataSize / point_size;
  return static_cast<uint16_t>(
      (kTotalPoints + points_per_packet - 1) / points_per_packet);
}

size_t OdinFrameAssembler::MaxPointPayloadBytes(bool is_slam) const {
  constexpr size_t kTotalPoints = kPclWidth * kPclHeight;
  const size_t point_size =
      is_slam ? sizeof(OdinSlamPoint<uint16_t>) : sizeof(OdinRawPoint<uint16_t>);
  return kTotalPoints * point_size;
}

bool OdinFrameAssembler::FillLostPacketsWithZeros(uint16_t lost_packets,
                                                   size_t points_per_packet,
                                                   bool is_slam) {
  if (lost_packets == 0 || points_per_packet == 0) return true;
  
  PointFrameState& state = is_slam ? slam_point_state_ : raw_point_state_;
  size_t lost_points = static_cast<size_t>(lost_packets) * points_per_packet;
  size_t point_size = is_slam ? sizeof(OdinSlamPoint<uint16_t>) : sizeof(OdinRawPoint<uint16_t>);
  size_t fill_bytes = lost_points * point_size;

  const size_t max_payload_bytes = MaxPointPayloadBytes(is_slam);
  if (state.packet.payload.size() > max_payload_bytes ||
      fill_bytes > max_payload_bytes - state.packet.payload.size()) {
    LOG_WARN("Dropping point cloud frame: packet-loss fill exceeds frame capacity\n");
    return false;
  }

  LOG_WARN("Point cloud packet loss: lost %u packets (%zu points)\n", 
           lost_packets, lost_points);

  std::vector<uint8_t> zero_fill(fill_bytes, 0);
  state.packet.payload.insert(state.packet.payload.end(),
                               zero_fill.begin(), zero_fill.end());

  uint32_t new_total = static_cast<uint32_t>(state.packet.point_count) + lost_points;
  if (new_total > 65535) new_total = 65535;
  state.packet.point_count = static_cast<uint16_t>(new_total);
  state.udp_segments += lost_packets;
  return true;
}

void OdinFrameAssembler::ProcessPointFragment(const OdinPointCloudPacket& fragment,
                                               bool is_slam) {
  const uint16_t expected_packets = ExpectedPointUdpCount(is_slam);
  const size_t max_payload_bytes = MaxPointPayloadBytes(is_slam);
  std::lock_guard<std::mutex> lock(frame_mutex_);

  PointFrameState& state = is_slam ? slam_point_state_ : raw_point_state_;

  bool start_new_frame =
      state.active && (fragment.frame_count != state.packet.frame_count ||
                        fragment.udp_count < state.last_udp_seq);

  if (start_new_frame) {
    EmitPointFrame(is_slam);
  }

  if (!state.active) {
    state.active = true;
    // Copy header metadata from the first fragment, but reset accumulators.
    // NOTE: Do NOT keep fragment.point_count here - it will be re-accumulated
    // below via `state.packet.point_count += fragment.point_count`. Otherwise
    // the first packet's point count is double-counted, causing the assembled
    // frame to claim more points than the payload actually contains (showing
    // up as spurious "packet truncated" warnings downstream).
    state.packet = fragment;
    state.packet.payload.clear();
    state.packet.payload.reserve(max_payload_bytes);
    state.packet.point_count = 0;
    state.last_udp_seq = 0;
    state.udp_segments = 0;
  }

  // Check for packet loss
  if (fragment.udp_count > state.last_udp_seq + 1 && state.last_udp_seq > 0) {
    uint16_t lost = fragment.udp_count - state.last_udp_seq - 1;
    const size_t segments_after_fragment =
        static_cast<size_t>(state.udp_segments) + lost + 1;
    if (segments_after_fragment > expected_packets) {
      LOG_WARN(
          "Dropping point cloud frame: UDP gap would require %zu packets, maximum is %u\n",
          segments_after_fragment, expected_packets);
      state.Reset();
      return;
    }
    size_t point_size = is_slam ? sizeof(OdinSlamPoint<uint16_t>) : sizeof(OdinRawPoint<uint16_t>);
    size_t pts_per_pkt = kUdpMaxDataSize / point_size;
    if (!FillLostPacketsWithZeros(lost, pts_per_pkt, is_slam)) {
      state.Reset();
      return;
    }
  }

  // Append payload
  if (state.packet.payload.size() > max_payload_bytes ||
      fragment.payload.size() > max_payload_bytes - state.packet.payload.size()) {
    LOG_WARN("Dropping point cloud frame: payload exceeds frame capacity\n");
    state.Reset();
    return;
  }
  state.packet.payload.insert(state.packet.payload.end(),
                               fragment.payload.begin(), fragment.payload.end());
  const uint32_t accumulated_points =
      static_cast<uint32_t>(state.packet.point_count) + fragment.point_count;
  state.packet.point_count = static_cast<uint16_t>(
      std::min<uint32_t>(accumulated_points, 65535));
  state.last_udp_seq = fragment.udp_count;
  state.udp_segments++;

  // Check if frame is complete
  if (state.udp_segments >= expected_packets) {
    EmitPointFrame(is_slam);
  }
}

void OdinFrameAssembler::EmitPointFrame(bool is_slam) {
  PointFrameState& state = is_slam ? slam_point_state_ : raw_point_state_;

  if (!state.active || state.packet.payload.empty()) {
    state.Reset();
    return;
  }

  state.packet.udp_count = state.udp_segments;
  size_t total_length = kDataHeaderSize + state.packet.payload.size();
  if (total_length > 65535) total_length = 65535;
  state.packet.length = static_cast<uint16_t>(total_length);
  state.packet.device = handle_;

  if (is_slam && slam_cb_) {
    // Convert uint16 to float for SLAM (with -30000 offset for y,z)
    // Coordinate transformation (rotation/translation) will be done by SlamOdomSynchronizer
    size_t src_point_size = sizeof(OdinSlamPoint<uint16_t>);
    size_t point_count = state.packet.payload.size() / src_point_size;
    const OdinSlamPoint<uint16_t>* src =
        reinterpret_cast<const OdinSlamPoint<uint16_t>*>(state.packet.payload.data());

    std::vector<uint8_t> new_payload(point_count * sizeof(OdinSlamPoint<float>));
    OdinSlamPoint<float>* dst = reinterpret_cast<OdinSlamPoint<float>*>(new_payload.data());

    for (size_t i = 0; i < point_count; ++i) {
      // Convert to float with offset (in millimeters)
      dst[i].x = static_cast<float>(src[i].x);
      dst[i].y = static_cast<float>(src[i].y) - 30000.0f;
      dst[i].z = static_cast<float>(src[i].z) - 30000.0f;
      dst[i].r = src[i].r;
      dst[i].g = src[i].g;
      dst[i].b = src[i].b;
      dst[i].a = src[i].a;
    }
    state.packet.payload = std::move(new_payload);
    slam_cb_(state.packet);
  } else if (!is_slam && point_cloud_cb_) {
    // Convert uint16 to float for raw point cloud
    size_t src_point_size = sizeof(OdinRawPoint<uint16_t>);
    size_t point_count = state.packet.payload.size() / src_point_size;
    const OdinRawPoint<uint16_t>* src =
        reinterpret_cast<const OdinRawPoint<uint16_t>*>(state.packet.payload.data());

    std::vector<uint8_t> new_payload(point_count * sizeof(OdinRawPoint<float>));
    OdinRawPoint<float>* dst = reinterpret_cast<OdinRawPoint<float>*>(new_payload.data());

    for (size_t i = 0; i < point_count; ++i) {
      dst[i].x = static_cast<float>(src[i].x);
      dst[i].y = static_cast<float>(src[i].y) - 30000.0f;
      dst[i].z = static_cast<float>(src[i].z) - 30000.0f;
      dst[i].intensity = src[i].intensity;
      dst[i].confidence = src[i].confidence;
    }
    state.packet.payload = std::move(new_payload);
    point_cloud_cb_(state.packet);
  }

  state.Reset();
}

void OdinFrameAssembler::ProcessRawPointData(const uint8_t* data, size_t length) {
  if (use_tcp_) {
    {
      std::lock_guard<std::mutex> lock(tcp_buffer_mutex_);
      tcp_buffer_.Append(data, length);
    }
    ProcessTcpBuffer([this](const uint8_t* frame_data, size_t frame_len) {
      OdinPointCloudPacket packet;
      if (ParsePointCloudPacket(frame_data, frame_len, &packet)) {
        ProcessPointFragment(packet, false);
      }
    });
  } else {
    OdinPointCloudPacket packet;
    if (ParsePointCloudPacket(data, length, &packet)) {
      ProcessPointFragment(packet, false);
    }
  }
}

void OdinFrameAssembler::ProcessSlamPointData(const uint8_t* data, size_t length) {
  if (use_tcp_) {
    {
      std::lock_guard<std::mutex> lock(tcp_buffer_mutex_);
      tcp_buffer_.Append(data, length);
    }
    ProcessTcpBuffer([this](const uint8_t* frame_data, size_t frame_len) {
      OdinPointCloudPacket packet;
      if (ParsePointCloudPacket(frame_data, frame_len, &packet)) {
        ProcessPointFragment(packet, true);
      }
    });
  } else {
    OdinPointCloudPacket packet;
    if (ParsePointCloudPacket(data, length, &packet)) {
      ProcessPointFragment(packet, true);
    }
  }
}

// =============================================================================
// Image processing
// =============================================================================

bool OdinFrameAssembler::DetectJpegTerminator(const std::vector<uint8_t>& chunk) {
  if (chunk.empty()) return false;

  if (image_state_.last_byte_valid && image_state_.last_byte == 0xFF && chunk[0] == 0xD9) {
    return true;
  }

  for (size_t i = 1; i < chunk.size(); ++i) {
    if (chunk[i - 1] == 0xFF && chunk[i] == 0xD9) return true;
  }

  image_state_.last_byte = chunk.back();
  image_state_.last_byte_valid = true;
  return false;
}

void OdinFrameAssembler::ProcessImageFragment(const OdinImagePacket& fragment) {
  std::lock_guard<std::mutex> lock(frame_mutex_);

  bool start_new_frame =
      image_state_.active && (fragment.frame_count != image_state_.packet.frame_count ||
                               fragment.udp_count < image_state_.last_udp_seq);

  if (start_new_frame) {
    EmitImageFrame();
  }

  if (!image_state_.active) {
    image_state_.active = true;
    image_state_.packet = fragment;
    image_state_.packet.payload.clear();
    image_state_.last_udp_seq = 0;
    image_state_.udp_segments = 0;
    image_state_.last_byte = 0;
    image_state_.last_byte_valid = false;
  }

  // Append payload
  image_state_.packet.payload.insert(image_state_.packet.payload.end(),
                                      fragment.payload.begin(), fragment.payload.end());
  image_state_.last_udp_seq = fragment.udp_count;
  image_state_.udp_segments++;

  // Check for JPEG terminator (0xFF 0xD9)
  if (DetectJpegTerminator(fragment.payload)) {
    EmitImageFrame();
  }
}

void OdinFrameAssembler::EmitImageFrame() {
  if (!image_state_.active || image_state_.packet.payload.empty()) {
    image_state_.Reset();
    return;
  }

  constexpr size_t kLengthFieldSize = 4;
  constexpr size_t kEOISize = 2;

  if (image_state_.packet.payload.size() < kLengthFieldSize + kEOISize + 2) {
    image_state_.Reset();
    return;
  }

  size_t payload_size = image_state_.packet.payload.size();
  const uint8_t* len_ptr = image_state_.packet.payload.data() + payload_size - kLengthFieldSize - kEOISize;
  uint32_t expected_len =
      static_cast<uint32_t>(len_ptr[0]) << 24 | (static_cast<uint32_t>(len_ptr[1]) << 16) |
      (static_cast<uint32_t>(len_ptr[2]) << 8) | (static_cast<uint32_t>(len_ptr[3]));

  uint32_t actual_len = static_cast<uint32_t>(payload_size - kLengthFieldSize);

  if (actual_len != expected_len) {
    LOG_WARN("Image frame dropped: expected %u bytes, got %u\n", expected_len, actual_len);
    image_state_.Reset();
    return;
  }

  // Restore JPEG EOI marker
  uint8_t* data = image_state_.packet.payload.data();
  size_t jpeg_data_len = payload_size - kLengthFieldSize - kEOISize;
  data[jpeg_data_len] = 0xFF;
  data[jpeg_data_len + 1] = 0xD9;
  image_state_.packet.payload.resize(actual_len);

  image_state_.packet.udp_count = image_state_.udp_segments;
  size_t total_length = kDataHeaderSize + image_state_.packet.payload.size();
  if (total_length > 65535) total_length = 65535;
  image_state_.packet.length = static_cast<uint16_t>(total_length);
  image_state_.packet.device = handle_;

  if (image_cb_) {
    image_cb_(image_state_.packet);
  }

  image_state_.Reset();
}

void OdinFrameAssembler::ProcessImageData(const uint8_t* data, size_t length) {
  if (use_tcp_) {
    {
      std::lock_guard<std::mutex> lock(tcp_buffer_mutex_);
      tcp_buffer_.Append(data, length);
    }
    ProcessTcpBuffer([this](const uint8_t* frame_data, size_t frame_len) {
      OdinImagePacket packet;
      if (ParseImagePacket(frame_data, frame_len, &packet)) {
        ProcessImageFragment(packet);
      }
    });
  } else {
    OdinImagePacket packet;
    if (ParseImagePacket(data, length, &packet)) {
      ProcessImageFragment(packet);
    }
  }
}

// =============================================================================
// IMU/Odom processing
// =============================================================================

void OdinFrameAssembler::ProcessImuData(const uint8_t* data, size_t length) {
  auto process_packet = [this](const uint8_t* frame_data, size_t frame_len) {
    OdinImuPacket packet;
    if (ParseImuPacket(frame_data, frame_len, &packet)) {
      if (!imu_cb_) {
        return;
      }
      
      // Detect firmware version by per-sample size and parse accordingly.
      //
      // Forward-compatible parsing strategy:
      //   - Legacy firmware: 24 bytes per sample (6 floats: gyro_xyz + acc_xyz only).
      //     Need to synthesize timestamp/sequence from packet header.
      //   - Current/future firmware: sample size >= sizeof(OdinImuSample).
      //     Fields are assumed to be appended at the end of the struct over time,
      //     so we copy min(firmware_size, current_struct_size). Any extra trailing
      //     fields the firmware sends (newer than this SDK knows about) are ignored;
      //     any fields newer than firmware sends remain zero-initialized.
      constexpr size_t kLegacySampleSize = 24;  // 6 * sizeof(float)
      constexpr size_t kCurrentSampleSize = sizeof(OdinImuSample);
      
      const size_t payload_size = packet.payload.size();
      
      // Determine per-sample size: prefer current/larger size if payload divides cleanly,
      // otherwise fall back to legacy 24-byte format.
      size_t sample_size = 0;
      if (payload_size >= kCurrentSampleSize && payload_size % kCurrentSampleSize == 0) {
        sample_size = kCurrentSampleSize;
      } else if (payload_size >= kLegacySampleSize && payload_size % kLegacySampleSize == 0) {
        sample_size = kLegacySampleSize;
      } else {
        return;  // Unknown format
      }
      
      const bool is_legacy = (sample_size == kLegacySampleSize);
      const size_t sample_count = payload_size / sample_size;
      const size_t copy_size = std::min(sample_size, kCurrentSampleSize);
      const uint8_t* raw_data = packet.payload.data();
      
      for (size_t i = 0; i < sample_count; ++i) {
        // Zero-initialize so unknown/missing fields default to 0.
        OdinImuSample sample{};
        const uint8_t* sample_ptr = raw_data + i * sample_size;
        
        // Copy known fields by their layout (struct grows by appending fields).
        std::memcpy(&sample, sample_ptr, copy_size);
        
        if (is_legacy) {
          // Legacy firmware doesn't carry per-sample timestamp/sequence - synthesize them.
          sample.timestamp = packet.timestamp;
          sample.sequence = i;
        }
        
        // Create single-sample packet and callback
        OdinImuPacket single_packet;
        single_packet.device = handle_;
        single_packet.version = packet.version;
        single_packet.sample_count = 1;
        single_packet.timestamp = sample.timestamp;
        single_packet.payload.assign(
            reinterpret_cast<const uint8_t*>(&sample),
            reinterpret_cast<const uint8_t*>(&sample) + sizeof(OdinImuSample));
        imu_cb_(single_packet);
      }
    }
  };

  if (use_tcp_) {
    {
      std::lock_guard<std::mutex> lock(tcp_buffer_mutex_);
      tcp_buffer_.Append(data, length);
    }
    ProcessTcpBuffer(process_packet);
  } else {
    process_packet(data, length);
  }
}

void OdinFrameAssembler::ProcessOdomData(const uint8_t* data, size_t length) {
  auto process_packet = [this](const uint8_t* frame_data, size_t frame_len) {
    OdinOdomPacket packet;
    if (ParseOdomPacket(frame_data, frame_len, &packet)) {
      packet.device = handle_;
      
      // Extract odom_type from payload
      OdomSourceType odom_type = OdomSourceType::kOdom2ImuLow;
      if (packet.payload.size() >= sizeof(OdinOdomData)) {
        const OdinOdomData* odom_data =
            reinterpret_cast<const OdinOdomData*>(packet.payload.data());
        odom_type = static_cast<OdomSourceType>(odom_data->type);
      }
      
      if (odom_cb_) {
        odom_cb_(packet, odom_type);
      }
    }
  };

  if (use_tcp_) {
    {
      std::lock_guard<std::mutex> lock(tcp_buffer_mutex_);
      tcp_buffer_.Append(data, length);
    }
    ProcessTcpBuffer(process_packet);
  } else {
    process_packet(data, length);
  }
}

}  // namespace sdk
}  // namespace odin
