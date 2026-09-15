// src/pico_bridge_node.cpp
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
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
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "pico_bridge/pico_frame.hpp"
#include "pico_bridge/tracking_epoch_store.hpp"

using namespace std::chrono_literals;
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

        running_ = true;
        thread_ = std::thread(&PicoBridgeNode::recv_loop, this);

        RCLCPP_INFO(get_logger(), "PicoBridge connecting to %s:%d ...", host_.c_str(), port_);
    }

    ~PicoBridgeNode() override {
        running_ = false;
        int s = sock_.exchange(-1);
        if (s >= 0) ::shutdown(s, SHUT_RDWR);
        if (thread_.joinable()) thread_.join();
        if (s >= 0) ::close(s);
    }

private:
    static builtin_interfaces::msg::Time ts_from_ms(int64_t ts_ms) {
        builtin_interfaces::msg::Time t;
        t.sec = static_cast<int32_t>(ts_ms / 1000);
        t.nanosec = static_cast<uint32_t>((ts_ms % 1000) * 1'000'000);
        return t;
    }

    bool recv_exact(int fd, uint8_t* buf, size_t n) {
        size_t got = 0;
        while (got < n) {
            ssize_t r = ::recv(fd, buf + got, n - got, 0);
            if (r < 0) {
                if (errno == EINTR) continue;
                return false;
            }
            if (r == 0) return false;  // peer closed
            got += static_cast<size_t>(r);
        }
        return true;
    }

    int connect_once() {
        int s = ::socket(AF_INET, SOCK_STREAM, 0);
        if (s < 0) return -1;
        int one = 1;
        ::setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(static_cast<uint16_t>(port_));
        ::inet_pton(AF_INET, host_.c_str(), &addr.sin_addr);
        if (::connect(s, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            ::close(s);
            return -1;
        }
        return s;
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
        try {
            const uint64_t epoch = pico_bridge::reserve_tracking_epoch(tracking_epoch_state_file_);
            tracking_epoch_.store(epoch);
            publish_tracking_epoch(epoch, source);
        } catch (const std::exception & error) {
            RCLCPP_FATAL(get_logger(), "Cannot reserve tracking epoch: %s", error.what());
            running_ = false;
            const int socket = sock_.exchange(-1);
            if (socket >= 0) {
                ::shutdown(socket, SHUT_RDWR);
            }
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

    void recv_loop() {
        while (running_ && rclcpp::ok()) {
            sock_ = connect_once();
            if (sock_ < 0) {
                RCLCPP_WARN(get_logger(), "Connect to %s:%d failed, retrying in 2s",
                            host_.c_str(), port_);
                std::this_thread::sleep_for(2s);
                continue;
            }
            RCLCPP_INFO(get_logger(), "Connected to %s:%d", host_.c_str(), port_);
            advance_tracking_epoch("tcp_connection");

            while (running_ && rclcpp::ok()) {
                uint8_t header[pf::HEADER_SIZE];
                if (!recv_exact(sock_, header, pf::HEADER_SIZE)) break;
                pf::FrameHeader h;
                if (!pf::parse_frame_header(header, h)) {
                    RCLCPP_WARN(get_logger(),
                        "Bad/invalid header (magic=0x%02X, payload_len=%u); forcing reconnect",
                        h.magic, h.payload_len);
                    break;  // exit inner loop → outer loop will close sock and reconnect
                }
                std::vector<uint8_t> payload(h.payload_len);
                if (h.payload_len > 0 && !recv_exact(sock_, payload.data(), h.payload_len)) break;

                switch (h.type) {
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
                        break;  // recognized, intentionally not published
                    default: {
                        if (pf::is_body_pose_type(h.type))
                            accept_body_pose(h.type, h.ts_ms, payload.data(), payload.size());
                        else
                            RCLCPP_WARN(get_logger(), "Unknown type 0x%02X, skipping", h.type);
                    }
                }
            }

            RCLCPP_WARN(get_logger(), "Disconnected; reconnecting in 2s");
            int s2 = sock_.exchange(-1);
            if (s2 >= 0) ::close(s2);
            std::this_thread::sleep_for(2s);
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
    std::atomic<int> sock_{-1};
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
