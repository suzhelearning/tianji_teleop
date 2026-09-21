// include/pico_bridge/pico_frame.hpp
#pragma once
#include <array>
#include <cstdint>
#include <cstring>

namespace pico_bridge {

constexpr int HEADER_SIZE = 1 + 1 + 8 + 4;         // 14 bytes
constexpr uint8_t FRAME_MAGIC = 0xAB;
constexpr uint32_t MAX_PAYLOAD_BYTES = 64 * 1024 * 1024;

// Frame type IDs — mirror receiver.py / PICO_Streaming_Guide.md.
constexpr uint8_t TYPE_CAM_LEFT    = 0x01;
constexpr uint8_t TYPE_CAM_RIGHT   = 0x02;
constexpr uint8_t TYPE_POSE_LEFT   = 0x03;
constexpr uint8_t TYPE_POSE_RIGHT  = 0x04;
constexpr uint8_t TYPE_POSE_HEAD   = 0x05;
constexpr uint8_t TYPE_WORLD_RESET = 0x06;  // 4B float yaw, published as /pico/world_reset
constexpr uint8_t TYPE_CTRL_LEFT   = 0x07;  // controller button state, payload undocumented
constexpr uint8_t TYPE_CTRL_RIGHT  = 0x08;  // controller button state, payload undocumented
constexpr uint8_t TYPE_RECORD_FLAG = 0x09;  // 1B, non-zero = start recording
constexpr uint8_t TYPE_BLE_LEFT    = 0x10;
constexpr uint8_t TYPE_BLE_RIGHT   = 0x11;

// Full PICO BodyTrackerRole stream. Frame type is 0x20 + enum index and each
// payload is pos.xyz + quat.xyzw (7 little-endian float32 values). The order is
// the protocol contract in pico_stream_record_receiver.py.
constexpr uint8_t TYPE_BODY_BASE = 0x20;
constexpr uint8_t TYPE_BODY_LAST = 0x37;
constexpr size_t BODY_JOINT_COUNT = 24;
constexpr std::array<const char*, BODY_JOINT_COUNT> BODY_JOINT_NAMES = {
    "Pelvis",
    "LEFT_HIP", "RIGHT_HIP", "SPINE1",
    "LEFT_KNEE", "RIGHT_KNEE", "SPINE2",
    "LEFT_ANKLE", "RIGHT_ANKLE", "SPINE3",
    "LEFT_FOOT", "RIGHT_FOOT", "NECK",
    "LEFT_COLLAR", "RIGHT_COLLAR", "HEAD",
    "LEFT_SHOULDER", "RIGHT_SHOULDER",
    "LEFT_ELBOW", "RIGHT_ELBOW",
    "LEFT_WRIST", "RIGHT_WRIST", "LEFT_HAND", "RIGHT_HAND",
};

inline bool is_body_pose_type(uint8_t type) {
    return type >= TYPE_BODY_BASE && type <= TYPE_BODY_LAST;
}

inline size_t body_joint_index(uint8_t type) {
    return static_cast<size_t>(type - TYPE_BODY_BASE);
}

struct FrameHeader {
    uint8_t  magic;
    uint8_t  type;
    int64_t  ts_ms;
    uint32_t payload_len;
};

// Parse 14-byte little-endian header. Returns false on bad magic or oversized payload.
inline bool parse_frame_header(const uint8_t* buf, FrameHeader& out) {
    out.magic = buf[0];
    out.type  = buf[1];
    std::memcpy(&out.ts_ms,       buf + 2, 8);
    std::memcpy(&out.payload_len, buf + 10, 4);
    if (out.magic != FRAME_MAGIC) return false;
    if (out.payload_len > MAX_PAYLOAD_BYTES) return false;
    return true;
}

// Decode 7×float32 little-endian pose payload into pos[3] + quat[4] (xyzw).
// Returns false if payload_len < 28.
inline bool parse_pose_payload(const uint8_t* payload, size_t len,
                                float pos[3], float quat_xyzw[4]) {
    if (len < 28) return false;
    std::memcpy(pos,        payload,      12);
    std::memcpy(quat_xyzw,  payload + 12, 16);
    return true;
}

inline bool parse_world_reset_payload(const uint8_t* payload, size_t len, float& yaw) {
    if (len < sizeof(float)) return false;
    std::memcpy(&yaw, payload, sizeof(float));
    return true;
}

// Split a BLE payload [4B esp32_ts | sensor_bytes] → esp32_ts + (data_ptr, data_len).
// Returns false if payload is shorter than 4 bytes.
inline bool split_ble_payload(const uint8_t* payload, size_t len,
                               uint32_t& esp32_ts,
                               const uint8_t*& data_ptr, size_t& data_len) {
    if (len < 4) return false;
    std::memcpy(&esp32_ts, payload, 4);
    data_ptr = payload + 4;
    data_len = len - 4;
    return true;
}

}  // namespace pico_bridge
