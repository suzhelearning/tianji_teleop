// test/test_pico_frame.cpp
#include <gtest/gtest.h>
#include <cstring>
#include "pico_bridge/pico_frame.hpp"

using pico_bridge::FrameHeader;
using pico_bridge::parse_frame_header;
using pico_bridge::HEADER_SIZE;

TEST(PicoFrame, ParsesValidHeader) {
    uint8_t buf[HEADER_SIZE];
    buf[0] = 0xAB;                        // magic
    buf[1] = 0x03;                        // type = pose_left
    int64_t ts_ms = 123456;
    std::memcpy(buf + 2, &ts_ms, 8);
    uint32_t payload_len = 28;
    std::memcpy(buf + 10, &payload_len, 4);

    FrameHeader hdr;
    ASSERT_TRUE(parse_frame_header(buf, hdr));
    EXPECT_EQ(hdr.magic, 0xAB);
    EXPECT_EQ(hdr.type, 0x03);
    EXPECT_EQ(hdr.ts_ms, 123456);
    EXPECT_EQ(hdr.payload_len, 28u);
}

TEST(PicoFrame, TypeConstantsMatchWireProtocol) {
    EXPECT_EQ(pico_bridge::TYPE_WORLD_RESET, 0x06);
    EXPECT_EQ(pico_bridge::TYPE_CTRL_LEFT,   0x07);
    EXPECT_EQ(pico_bridge::TYPE_CTRL_RIGHT,  0x08);
    EXPECT_EQ(pico_bridge::TYPE_RECORD_FLAG, 0x09);
    EXPECT_EQ(pico_bridge::TYPE_BODY_BASE, 0x20);
    EXPECT_EQ(pico_bridge::TYPE_BODY_LAST, 0x37);
    EXPECT_EQ(pico_bridge::BODY_JOINT_COUNT, 24u);
    EXPECT_STREQ(pico_bridge::BODY_JOINT_NAMES[0], "Pelvis");
    EXPECT_STREQ(pico_bridge::BODY_JOINT_NAMES[7], "LEFT_ANKLE");
    EXPECT_STREQ(pico_bridge::BODY_JOINT_NAMES[23], "RIGHT_HAND");
    for (uint8_t type = 0x20; type <= 0x37; ++type) {
        EXPECT_TRUE(pico_bridge::is_body_pose_type(type));
        EXPECT_EQ(pico_bridge::body_joint_index(type), type - 0x20);
    }
    EXPECT_FALSE(pico_bridge::is_body_pose_type(0x1F));
    EXPECT_FALSE(pico_bridge::is_body_pose_type(0x38));
}

TEST(PicoFrame, ParsesWorldResetYaw) {
    const float expected = 1.25F;
    uint8_t payload[sizeof(float)];
    std::memcpy(payload, &expected, sizeof(expected));
    float yaw{};
    ASSERT_TRUE(pico_bridge::parse_world_reset_payload(payload, sizeof(payload), yaw));
    EXPECT_FLOAT_EQ(yaw, expected);
    EXPECT_FALSE(pico_bridge::parse_world_reset_payload(payload, sizeof(payload) - 1, yaw));
}

TEST(PicoFrame, BodyPosePayloadReusesPoseParser) {
    // Body tracker frames carry the same 7×float32 payload as head/hand poses.
    float in[7] = {1.5f, -2.0f, 0.25f, 0.0f, 0.0f, 0.7071f, 0.7071f};
    uint8_t payload[28];
    std::memcpy(payload, in, sizeof(in));

    float pos[3], quat[4];
    ASSERT_TRUE(pico_bridge::parse_pose_payload(payload, sizeof(payload), pos, quat));
    EXPECT_FLOAT_EQ(pos[0], 1.5f);
    EXPECT_FLOAT_EQ(pos[2], 0.25f);
    EXPECT_FLOAT_EQ(quat[3], 0.7071f);
}

TEST(PicoFrame, RejectsBadMagic) {
    uint8_t buf[HEADER_SIZE] = {0x00};
    FrameHeader hdr;
    EXPECT_FALSE(parse_frame_header(buf, hdr));
}

TEST(PicoFrame, RejectsOversizedPayload) {
    uint8_t buf[HEADER_SIZE];
    buf[0] = 0xAB;
    buf[1] = 0x01;
    int64_t ts = 0;
    std::memcpy(buf + 2, &ts, 8);
    uint32_t big = 65 * 1024 * 1024;      // 65 MB — exceeds protocol's 64 MB cap
    std::memcpy(buf + 10, &big, 4);

    FrameHeader hdr;
    EXPECT_FALSE(parse_frame_header(buf, hdr));
}
