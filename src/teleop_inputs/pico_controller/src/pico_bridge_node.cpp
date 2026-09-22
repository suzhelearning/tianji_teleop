// src/pico_bridge_node.cpp
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <pico_bridge/msg/ble_frame.hpp>

#include <array>
#include <atomic>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include "pico_bridge/pico_frame.hpp"
#include "pico_bridge/tracking_epoch_store.hpp"

namespace pf = pico_bridge;

class PicoBridgeNode : public rclcpp::Node {
public:
    PicoBridgeNode() : Node("pico_bridge") {
        host_ = declare_parameter<std::string>("host", "127.0.0.1");
        port_ = declare_parameter<int>("port", 9999);
        smpl_raw_topic_ = declare_parameter<std::string>(
            "smpl_raw_topic", "/pico/smpl_raw");
        world_reset_topic_ = declare_parameter<std::string>(
            "world_reset_topic", "/pico/world_reset");
        tracking_epoch_topic_ = declare_parameter<std::string>(
            "tracking_epoch_topic", "/pico/tracking_epoch");
        tracking_epoch_status_topic_ = declare_parameter<std::string>(
            "tracking_epoch_status_topic", "/pico/tracking_epoch/status");
        tracking_epoch_state_file_ = declare_parameter<std::string>(
            "tracking_epoch_state_file", "~/.config/pico_tracker/tracking_epoch");
        if (smpl_raw_topic_.empty() || world_reset_topic_.empty()) {
            throw std::invalid_argument("SMPL raw and world reset topics must not be empty");
        }
        if (tracking_epoch_state_file_.empty()) {
            throw std::invalid_argument("tracking_epoch_state_file must not be empty");
        }
        if (tracking_epoch_state_file_.rfind("~/", 0) == 0) {
            const char * home = std::getenv("HOME");
            if (home == nullptr || *home == '\0') {
                throw std::invalid_argument("HOME is required for a ~ tracking epoch path");
            }
            tracking_epoch_state_file_ = std::string(home) + tracking_epoch_state_file_.substr(1);
        }

        auto sensor_qos = rclcpp::SensorDataQoS();
        cam_l_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
            "/pico/cam_left/compressed", sensor_qos);
        cam_r_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
            "/pico/cam_right/compressed", sensor_qos);

        pose_h_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pico/pose/head", 10);
        pose_l_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pico/pose/left_hand", 10);
        pose_r_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pico/pose/right_hand", 10);

        ble_l_pub_ = create_publisher<pico_bridge::msg::BleFrame>("/pico/ble/left", 10);
        ble_r_pub_ = create_publisher<pico_bridge::msg::BleFrame>("/pico/ble/right", 10);

        // The APK sends one frame per BodyTrackerRole (0x20-0x37). Publish a
        // single PoseArray only after all 24 joints for a timestamp arrive, so
        // the recorder cannot mix joints from adjacent body frames.
        smpl_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
            smpl_raw_topic_, sensor_qos);
        smpl_poses_.resize(pf::BODY_JOINT_COUNT);

        record_flag_pub_ = create_publisher<std_msgs::msg::Bool>("/pico/record_flag", 10);
        world_reset_pub_ = create_publisher<std_msgs::msg::Float32>(world_reset_topic_, 10);
        auto epoch_qos = rclcpp::QoS(1).reliable().transient_local();
        tracking_epoch_pub_ = create_publisher<std_msgs::msg::UInt64>(
            tracking_epoch_topic_, epoch_qos);
        tracking_epoch_status_pub_ = create_publisher<std_msgs::msg::String>(
            tracking_epoch_status_topic_, epoch_qos);
        publish_tracking_epoch(0, "unknown");

        // Bind before returning from construction so a conflicting listener
        // fails startup rather than silently retrying in the receiver thread.
        listener_ = open_listener();
        try {
            if (::pipe2(wake_pipe_.data(), O_NONBLOCK | O_CLOEXEC) < 0) {
                throw std::system_error(errno, std::generic_category(),
                                        "Cannot create PicoBridge receiver wake pipe");
            }
            running_ = true;
            RCLCPP_INFO(get_logger(),
                        "PicoBridge listening on %s:%d; waiting for wired APK TCP client",
                        host_.c_str(), port_);
            thread_ = std::thread(&PicoBridgeNode::recv_loop, this);
        } catch (...) {
            running_ = false;
            ::close(listener_);
            for (const int fd : wake_pipe_) {
                if (fd >= 0) ::close(fd);
            }
            throw;
        }
    }

    ~PicoBridgeNode() override {
        running_ = false;
        // Wake poll without closing a descriptor the receiver may still use.
        // The receiver alone owns client closes; listener/pipe close after join.
        const uint8_t wake = 1;
        while (::write(wake_pipe_[1], &wake, sizeof(wake)) < 0 && errno == EINTR) {}
        if (thread_.joinable()) thread_.join();
        ::close(listener_);
        for (const int fd : wake_pipe_) ::close(fd);
    }

private:
    static builtin_interfaces::msg::Time ts_from_ms(int64_t ts_ms) {
        builtin_interfaces::msg::Time t;
        t.sec = static_cast<int32_t>(ts_ms / 1000);
        t.nanosec = static_cast<uint32_t>((ts_ms % 1000) * 1'000'000);
        return t;
    }

    bool wait_readable(int fd) {
        pollfd fds[2]{{fd, POLLIN, 0}, {wake_pipe_[0], POLLIN, 0}};
        while (running_ && rclcpp::ok()) {
            const int ready = ::poll(fds, 2, 250);
            if (ready < 0) {
                if (errno == EINTR) continue;
                RCLCPP_ERROR(get_logger(), "Polling tracking socket failed: %s",
                             std::strerror(errno));
                return false;
            }
            if (!running_ || fds[1].revents != 0) return false;
            if (ready > 0 && fds[0].revents != 0) return true;
        }
        return false;
    }

    bool recv_exact(int fd, uint8_t* buf, size_t n) {
        size_t got = 0;
        while (got < n && running_ && rclcpp::ok()) {
            ssize_t r = ::recv(fd, buf + got, n - got, 0);
            if (r < 0) {
                if (errno == EINTR) continue;
                if ((errno == EAGAIN || errno == EWOULDBLOCK) && wait_readable(fd)) continue;
                return false;
            }
            if (r == 0) return false;  // peer closed
            got += static_cast<size_t>(r);
        }
        return got == n;
    }

    int open_listener() {
        if (port_ < 1 || port_ > 65535) {
            throw std::invalid_argument("PicoBridge port must be in 1..65535");
        }
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(static_cast<uint16_t>(port_));
        if (::inet_pton(AF_INET, host_.c_str(), &addr.sin_addr) != 1 ||
            (ntohl(addr.sin_addr.s_addr) >> 24) != 127) {
            throw std::invalid_argument(
                "PicoBridge host must be a numeric IPv4 loopback address (127.0.0.0/8)");
        }
        const std::string endpoint = host_ + ":" + std::to_string(port_);
        const int fd = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (fd < 0) {
            throw std::system_error(errno, std::generic_category(),
                                    "Cannot create PicoBridge tracking listener at " + endpoint);
        }
        const auto fail = [&](const char* operation) {
            const int error = errno;
            ::close(fd);
            throw std::system_error(error, std::generic_category(),
                                    std::string("Cannot ") + operation +
                                    " PicoBridge tracking listener at " + endpoint);
        };
        const int one = 1;
        if (::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one)) < 0) {
            fail("configure");
        }
        if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            fail("bind");
        }
        if (::listen(fd, 1) < 0) fail("listen on");
        return fd;
    }

    void clear_body_frame() {
        smpl_ts_ms_ = -1;
        smpl_received_.fill(false);
        smpl_received_count_ = 0;
        smpl_published_ = false;
    }

    void publish_tracking_epoch(uint64_t epoch, const std::string& source) {
        std_msgs::msg::UInt64 epoch_message;
        epoch_message.data = epoch;
        tracking_epoch_pub_->publish(epoch_message);
        std_msgs::msg::String status_message;
        status_message.data = "{\"tracking_epoch\":" + std::to_string(epoch) +
            ",\"tracking_epoch_source\":\"" + source + "\"}";
        tracking_epoch_status_pub_->publish(status_message);
    }

    void advance_tracking_epoch(const std::string& source) {
        clear_body_frame();
        try {
            const uint64_t epoch = pico_bridge::reserve_tracking_epoch(tracking_epoch_state_file_);
            tracking_epoch_.store(epoch);
            publish_tracking_epoch(epoch, source);
        } catch (const std::exception & error) {
            RCLCPP_FATAL(get_logger(), "Cannot reserve tracking epoch: %s", error.what());
            running_ = false;
        }
    }

    void publish_pose(rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub,
                     int64_t ts_ms, const uint8_t* payload, size_t len) {
        float pos[3], q[4];
        if (!pf::parse_pose_payload(payload, len, pos, q)) return;
        geometry_msgs::msg::PoseStamped m;
        m.header.stamp = ts_from_ms(ts_ms);
        m.header.frame_id = "pico";
        m.pose.position.x = pos[0];
        m.pose.position.y = pos[1];
        m.pose.position.z = pos[2];
        m.pose.orientation.x = q[0];
        m.pose.orientation.y = q[1];
        m.pose.orientation.z = q[2];
        m.pose.orientation.w = q[3];
        pub->publish(m);
    }

    void publish_cam(rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub,
                     int64_t ts_ms, std::vector<uint8_t>&& jpeg) {
        sensor_msgs::msg::CompressedImage m;
        m.header.stamp = ts_from_ms(ts_ms);
        m.header.frame_id = "pico";
        m.format = "jpeg";
        m.data = std::move(jpeg);
        pub->publish(m);
    }

    void accept_body_pose(uint8_t type, int64_t ts_ms,
                          const uint8_t* payload, size_t len) {
        float pos[3], q[4];
        if (!pf::parse_pose_payload(payload, len, pos, q)) {
            RCLCPP_WARN(get_logger(), "Invalid body pose payload for type 0x%02X", type);
            return;
        }

        if (ts_ms != smpl_ts_ms_) {
            smpl_ts_ms_ = ts_ms;
            smpl_received_.fill(false);
            smpl_received_count_ = 0;
            smpl_published_ = false;
        }

        const size_t index = pf::body_joint_index(type);
        auto& pose = smpl_poses_[index];
        pose.position.x = pos[0];
        pose.position.y = pos[1];
        pose.position.z = pos[2];
        pose.orientation.x = q[0];
        pose.orientation.y = q[1];
        pose.orientation.z = q[2];
        pose.orientation.w = q[3];
        if (!smpl_received_[index]) {
            smpl_received_[index] = true;
            ++smpl_received_count_;
        }

        if (smpl_received_count_ == pf::BODY_JOINT_COUNT && !smpl_published_) {
            geometry_msgs::msg::PoseArray msg;
            msg.header.stamp = ts_from_ms(ts_ms);
            msg.header.frame_id = "pico";
            msg.poses.assign(smpl_poses_.begin(), smpl_poses_.end());
            smpl_pub_->publish(msg);
            smpl_published_ = true;
        }
    }

    void publish_ble(rclcpp::Publisher<pico_bridge::msg::BleFrame>::SharedPtr pub,
                     int64_t ts_ms, const uint8_t* payload, size_t len) {
        uint32_t esp32_ts;
        const uint8_t* data_ptr;
        size_t data_len;
        if (!pf::split_ble_payload(payload, len, esp32_ts, data_ptr, data_len)) return;

        pico_bridge::msg::BleFrame m;
        m.header.stamp = ts_from_ms(ts_ms);
        m.header.frame_id = "pico";
        m.esp32_ts = esp32_ts;
        m.data.assign(data_ptr, data_ptr + data_len);
        pub->publish(m);
    }

    void publish_packed_tracking(uint8_t type, int64_t ts_ms,
                                 const uint8_t* payload, size_t len) {
        const bool body_only = type == pf::TYPE_BODY_ALL;
        const bool half = type == pf::TYPE_TRACKING_ALL_HALF;
        const size_t stride = half ? 14 : 28;
        const size_t offset = body_only ? 0 : 51;  // flags + two 25-byte controllers
        const size_t count = pf::BODY_JOINT_COUNT + (body_only ? 0 : 3);
        if (len != offset + count * stride ||
            (!body_only && (payload[0] & ~uint8_t{7}) != 0)) {
            RCLCPP_WARN(get_logger(), "Invalid packed tracking payload for type 0x%02X", type);
            return;
        }
        const uint8_t flags = body_only ? 4 : payload[0];
        float poses[pf::BODY_JOINT_COUNT + 3][7]{};
        // Validate the entire enabled frame before publishing any part.
        for (size_t i = 0; i < count; ++i) {
            const bool enabled = (!body_only && i < 3) ? (flags & 2) : (flags & 4);
            if (enabled && !pf::decode_packed_pose(payload + offset + i * stride, half, poses[i])) {
                RCLCPP_WARN(get_logger(), "Nonfinite packed tracking pose");
                return;
            }
        }
        if (!body_only && (flags & 2)) {
            publish_pose(pose_l_pub_, ts_ms, reinterpret_cast<const uint8_t*>(poses[0]), 28);
            publish_pose(pose_r_pub_, ts_ms, reinterpret_cast<const uint8_t*>(poses[1]), 28);
            publish_pose(pose_h_pub_, ts_ms, reinterpret_cast<const uint8_t*>(poses[2]), 28);
        }
        if (flags & 4) {
            clear_body_frame();
            for (size_t i = 0; i < pf::BODY_JOINT_COUNT; ++i) {
                accept_body_pose(pf::TYPE_BODY_BASE + i, ts_ms,
                    reinterpret_cast<const uint8_t*>(poses[i + (body_only ? 0 : 3)]), 28);
            }
        }
    }

    void recv_loop() {
        while (running_ && rclcpp::ok()) {
            if (!wait_readable(listener_)) break;
            const int client = ::accept4(listener_, nullptr, nullptr,
                                         SOCK_NONBLOCK | SOCK_CLOEXEC);
            if (client < 0) {
                if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK ||
                    errno == ECONNABORTED) continue;
                RCLCPP_ERROR(get_logger(), "Accepting tracking client failed: %s",
                             std::strerror(errno));
                running_ = false;
                break;
            }
            if (!running_ || !rclcpp::ok()) {
                ::close(client);
                break;
            }
            const int one = 1;
            ::setsockopt(client, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
            RCLCPP_INFO(get_logger(), "Accepted wired APK tracking client on %s:%d",
                        host_.c_str(), port_);
            advance_tracking_epoch("tcp_connection");

            while (running_ && rclcpp::ok()) {
                uint8_t header[pf::HEADER_SIZE];
                if (!recv_exact(client, header, pf::HEADER_SIZE)) break;
                pf::FrameHeader h;
                if (!pf::parse_frame_header(header, h)) {
                    RCLCPP_WARN(get_logger(),
                        "Bad/invalid header (magic=0x%02X, payload_len=%u); closing tracking client",
                        h.magic, h.payload_len);
                    break;  // Close this client and wait for the APK to reconnect.
                }
                std::vector<uint8_t> payload(h.payload_len);
                if (h.payload_len > 0 && !recv_exact(client, payload.data(), h.payload_len)) break;

                switch (h.type) {
                    case pf::TYPE_BODY_ALL:
                    case pf::TYPE_TRACKING_ALL:
                    case pf::TYPE_TRACKING_ALL_HALF:
                        publish_packed_tracking(h.type, h.ts_ms, payload.data(), payload.size()); break;
                    case pf::TYPE_CAM_LEFT:
                        publish_cam(cam_l_pub_, h.ts_ms, std::move(payload)); break;
                    case pf::TYPE_CAM_RIGHT:
                        publish_cam(cam_r_pub_, h.ts_ms, std::move(payload)); break;
                    case pf::TYPE_POSE_LEFT:
                        publish_pose(pose_l_pub_, h.ts_ms, payload.data(), payload.size()); break;
                    case pf::TYPE_POSE_RIGHT:
                        publish_pose(pose_r_pub_, h.ts_ms, payload.data(), payload.size()); break;
                    case pf::TYPE_POSE_HEAD:
                        publish_pose(pose_h_pub_, h.ts_ms, payload.data(), payload.size()); break;
                    case pf::TYPE_BLE_LEFT:
                        publish_ble(ble_l_pub_, h.ts_ms, payload.data(), payload.size()); break;
                    case pf::TYPE_BLE_RIGHT:
                        publish_ble(ble_r_pub_, h.ts_ms, payload.data(), payload.size()); break;
                    case pf::TYPE_RECORD_FLAG: {
                        std_msgs::msg::Bool m;
                        m.data = !payload.empty() && payload[0] != 0;
                        record_flag_pub_->publish(m);
                        RCLCPP_INFO(get_logger(), "Record flag: %d", m.data);
                        break;
                    }
                    case pf::TYPE_WORLD_RESET: {
                        float yaw{};
                        if (!pf::parse_world_reset_payload(payload.data(), payload.size(), yaw)) {
                            RCLCPP_WARN(get_logger(), "Invalid world reset payload");
                            break;
                        }
                        std_msgs::msg::Float32 m;
                        m.data = yaw;
                        advance_tracking_epoch("wire_world_reset");
                        world_reset_pub_->publish(m);
                        RCLCPP_INFO(get_logger(), "PICO world reset yaw: %.3f", yaw);
                        break;
                    }
                    case pf::TYPE_CTRL_LEFT:
                    case pf::TYPE_CTRL_RIGHT:
                    case pf::TYPE_CTRL_ALL:
                        break;  // recognized, intentionally not published
                    default: {
                        if (pf::is_body_pose_type(h.type))
                            accept_body_pose(h.type, h.ts_ms, payload.data(), payload.size());
                        else
                            RCLCPP_WARN(get_logger(), "Unknown type 0x%02X, skipping", h.type);
                    }
                }
            }

            ::close(client);
            clear_body_frame();
            if (running_ && rclcpp::ok()) {
                RCLCPP_WARN(get_logger(), "Tracking client disconnected; waiting for wired APK");
            }
        }
    }

    std::string host_;
    std::string tracking_epoch_state_file_;
    std::string smpl_raw_topic_;
    std::string world_reset_topic_;
    std::string tracking_epoch_topic_;
    std::string tracking_epoch_status_topic_;
    int port_ = 9999;
    std::atomic<bool> running_{false};
    int listener_ = -1;
    std::array<int, 2> wake_pipe_{{-1, -1}};
    std::atomic<uint64_t> tracking_epoch_{0};
    std::thread thread_;

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr cam_l_pub_, cam_r_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_h_pub_, pose_l_pub_, pose_r_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr smpl_pub_;
    std::vector<geometry_msgs::msg::Pose> smpl_poses_;
    std::array<bool, pf::BODY_JOINT_COUNT> smpl_received_{};
    int64_t smpl_ts_ms_ = -1;
    size_t smpl_received_count_ = 0;
    bool smpl_published_ = false;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr record_flag_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr world_reset_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr tracking_epoch_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tracking_epoch_status_pub_;
    rclcpp::Publisher<pico_bridge::msg::BleFrame>::SharedPtr ble_l_pub_, ble_r_pub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PicoBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
