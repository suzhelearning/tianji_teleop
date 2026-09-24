// test/test_pico_frame.cpp
#include <gtest/gtest.h>
#include <cstring>
#include <vector>
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

TEST(PicoFrame, SetGroundUsesRightAInEveryControllerWireLayout) {
    struct Layout {
        uint8_t type;
        size_t length;
        size_t right_offset;
    };
    const Layout layouts[] = {
        {pico_bridge::TYPE_CTRL_RIGHT, 25, 0},
        {pico_bridge::TYPE_CTRL_ALL, 50, 25},
        {pico_bridge::TYPE_TRACKING_ALL, 807, 26},
        {pico_bridge::TYPE_TRACKING_ALL_HALF, 429, 26},
    };
    for (const auto& layout : layouts) {
        SCOPED_TRACE(static_cast<int>(layout.type));
        pico_bridge::ControllerSetGroundEdge edge;
        std::vector<uint8_t> payload(layout.length);
        if (layout.right_offset == 26) payload[0] = 1;
        payload[layout.right_offset + 22] = 1;
        // Feature bits are not documented and must not gate a real A press.
        payload[layout.right_offset + 23] = 0xA5;
        const auto observe = [&](int64_t ts) {
            return edge.observe(layout.type, ts, payload.data(), payload.size());
        };
        // Connecting with A held is not a rising edge.
        payload[layout.right_offset] = 1;
        EXPECT_FALSE(observe(10));
        payload[layout.right_offset] = 0;
        EXPECT_FALSE(observe(11));
        payload[layout.right_offset] = 1;
        EXPECT_TRUE(observe(12));
        EXPECT_FALSE(observe(13));
        payload[layout.right_offset] = 0;
        EXPECT_FALSE(observe(14));
        payload[layout.right_offset] = 1;
        EXPECT_TRUE(observe(15));
    }
}

TEST(PicoFrame, InvalidControllerRequiresFreshReleaseBeforeNextPress) {
    pico_bridge::ControllerSetGroundEdge edge;
    std::array<uint8_t, 25> payload{};
    payload[22] = 1;
    const auto observe = [&](int64_t ts) {
        return edge.observe(pico_bridge::TYPE_CTRL_RIGHT, ts, payload.data(), payload.size());
    };
    EXPECT_FALSE(observe(1));  // Released and valid.
    payload[22] = 0;
    EXPECT_FALSE(observe(2));  // Lost controller validity.
    payload[0] = 1;
    payload[22] = 1;
    EXPECT_FALSE(observe(3));  // Held through validity recovery.
    payload[0] = 0;
    EXPECT_FALSE(observe(3));  // Duplicate release cannot arm.
    payload[0] = 1;
    EXPECT_FALSE(observe(4));
    payload[0] = 0;
    EXPECT_FALSE(observe(5));
    payload[0] = 1;
    EXPECT_TRUE(observe(6));
}

TEST(PicoFrame, MalformedControllerCannotTriggerOrKeepEdgeArmed) {
    pico_bridge::ControllerSetGroundEdge edge;
    std::array<uint8_t, 26> payload{};
    payload[22] = 1;
    int64_t ts = 0;
    for (const size_t bad_length : {size_t{0}, size_t{24}, size_t{26}}) {
        payload[0] = 0;
        EXPECT_FALSE(edge.observe(pico_bridge::TYPE_CTRL_RIGHT, ++ts, payload.data(), 25));
        payload[0] = 1;
        EXPECT_FALSE(edge.observe(pico_bridge::TYPE_CTRL_RIGHT, ++ts, payload.data(), bad_length));
        EXPECT_FALSE(edge.observe(pico_bridge::TYPE_CTRL_RIGHT, ++ts, payload.data(), 25));
    }
    payload[0] = 0;
    EXPECT_FALSE(edge.observe(pico_bridge::TYPE_CTRL_RIGHT, ++ts, payload.data(), 25));
    EXPECT_FALSE(edge.observe(pico_bridge::TYPE_CTRL_RIGHT, ++ts, nullptr, 25));
    payload[0] = 1;
    EXPECT_FALSE(edge.observe(pico_bridge::TYPE_CTRL_RIGHT, ++ts, payload.data(), 25));
    payload[0] = 0;
    EXPECT_FALSE(edge.observe(pico_bridge::TYPE_CTRL_RIGHT, ++ts, payload.data(), 25));
    payload[0] = 1;
    EXPECT_TRUE(edge.observe(pico_bridge::TYPE_CTRL_RIGHT, ++ts, payload.data(), 25));
}

TEST(PicoFrame, ControllerFreshnessIsSharedAcrossWireLayouts) {
    pico_bridge::ControllerSetGroundEdge edge;
    std::array<uint8_t, 25> single{};
    std::array<uint8_t, 50> packed{};
    single[22] = packed[47] = 1;
    EXPECT_FALSE(edge.observe(pico_bridge::TYPE_CTRL_RIGHT, 100, single.data(), single.size()));
    single[0] = 1;
    EXPECT_FALSE(edge.observe(pico_bridge::TYPE_CTRL_RIGHT, -1, single.data(), single.size()));
    EXPECT_FALSE(edge.observe(pico_bridge::TYPE_CTRL_RIGHT, 99, single.data(), single.size()));
    EXPECT_FALSE(edge.observe(pico_bridge::TYPE_CTRL_RIGHT, 100, single.data(), single.size()));
    EXPECT_TRUE(edge.observe(pico_bridge::TYPE_CTRL_RIGHT, 101, single.data(), single.size()));
    // A delayed aggregate release must not manufacture another press.
    EXPECT_FALSE(edge.observe(pico_bridge::TYPE_CTRL_ALL, 100, packed.data(), packed.size()));
    EXPECT_FALSE(edge.observe(pico_bridge::TYPE_CTRL_ALL, 101, packed.data(), packed.size()));
    EXPECT_FALSE(edge.observe(pico_bridge::TYPE_CTRL_RIGHT, 102, single.data(), single.size()));
    EXPECT_FALSE(edge.observe(pico_bridge::TYPE_CTRL_ALL, 103, packed.data(), packed.size()));
    // Stale invalid state must not disarm a fresh release either.
    single[22] = 0;
    EXPECT_FALSE(edge.observe(pico_bridge::TYPE_CTRL_RIGHT, 102, single.data(), single.size()));
    packed[25] = 1;
    EXPECT_TRUE(edge.observe(pico_bridge::TYPE_CTRL_ALL, 104, packed.data(), packed.size()));
}

TEST(PicoFrame, ReconnectionRequiresReleaseAndRestartsControllerClock) {
    pico_bridge::ControllerSetGroundEdge edge;
    std::array<uint8_t, 25> payload{};
    payload[22] = 1;
    const auto observe = [&](int64_t ts) {
        return edge.observe(pico_bridge::TYPE_CTRL_RIGHT, ts, payload.data(), payload.size());
    };
    EXPECT_FALSE(observe(100));
    edge.reset_connection();  // Do not retain an armed release across clients.
    payload[0] = 1;
    EXPECT_FALSE(observe(1));
    EXPECT_FALSE(observe(2));
    payload[0] = 0;
    EXPECT_FALSE(observe(3));
    payload[0] = 1;
    EXPECT_TRUE(observe(4));
    edge.reset_connection();  // Nor turn an already-observed hold into a press.
    EXPECT_FALSE(observe(5));
    payload[0] = 0;
    EXPECT_FALSE(observe(6));
    payload[0] = 1;
    EXPECT_TRUE(observe(7));
}

TEST(PicoFrame, DisabledPackedControllersAndLeftXAndRecordAreNotSetGround) {
    pico_bridge::ControllerSetGroundEdge edge;
    std::array<uint8_t, 807> payload{};
    payload[48] = 1;
    EXPECT_FALSE(edge.observe(pico_bridge::TYPE_TRACKING_ALL, 100, payload.data(), payload.size()));
    payload[0] = 1;
    payload[26] = 1;
    EXPECT_FALSE(edge.observe(pico_bridge::TYPE_TRACKING_ALL, 1, payload.data(), payload.size()));
    payload[26] = 0;
    EXPECT_FALSE(edge.observe(pico_bridge::TYPE_TRACKING_ALL, 2, payload.data(), payload.size()));
    payload[1] = 1;  // Left primary X.
    EXPECT_FALSE(edge.observe(pico_bridge::TYPE_TRACKING_ALL, 3, payload.data(), payload.size()));
    std::array<uint8_t, 25> left{};
    left[0] = left[22] = 1;
    EXPECT_FALSE(edge.observe(pico_bridge::TYPE_CTRL_LEFT, 100, left.data(), left.size()));
    EXPECT_FALSE(edge.observe(pico_bridge::TYPE_RECORD_FLAG, 100, left.data(), 1));
    payload[26] = 1;
    EXPECT_TRUE(edge.observe(pico_bridge::TYPE_TRACKING_ALL, 4, payload.data(), payload.size()));
}

TEST(PicoFrame, MalformedPackedFrameDisarmsControllerEdge) {
    pico_bridge::ControllerSetGroundEdge edge;
    std::array<uint8_t, 429> payload{};
    payload[0] = payload[48] = 1;
    const auto observe = [&](int64_t ts, size_t len) {
        return edge.observe(pico_bridge::TYPE_TRACKING_ALL_HALF, ts, payload.data(), len);
    };
    EXPECT_FALSE(observe(1, payload.size()));
    payload[0] = 0x81;  // Unknown packed-header flag.
    payload[26] = 1;
    EXPECT_FALSE(observe(2, payload.size()));
    payload[0] = 1;
    EXPECT_FALSE(observe(3, payload.size()));
    payload[26] = 0;
    EXPECT_FALSE(observe(4, payload.size()));
    payload[26] = 1;
    EXPECT_FALSE(observe(5, payload.size() - 1));
    EXPECT_FALSE(observe(6, payload.size()));
    payload[26] = 0;
    EXPECT_FALSE(observe(7, payload.size()));
    // The bridge also disarms after enabled nonfinite packed poses.
    edge.invalidate(8);
    payload[26] = 1;
    EXPECT_FALSE(observe(9, payload.size()));
    payload[26] = 0;
    EXPECT_FALSE(observe(10, payload.size()));
    payload[26] = 1;
    EXPECT_TRUE(observe(11, payload.size()));
}
