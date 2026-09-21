/**
 * @file ros_compat.hpp
 * @brief ROS1/ROS2 compatibility layer with unified wrapper classes
 *
 * Provides wrapper classes that allow writing ROS-version-agnostic code.
 * The main code uses these wrappers and does NOT need #if/#endif blocks.
 *
 * Usage:
 *   ros_compat::Publisher<Image> pub = node.advertise<Image>("topic", 10);
 *   pub.publish(msg);  // Works for both ROS1 and ROS2!
 */

#pragma once

#include <functional>
#include <memory>
#include <string>

// ============================================================================
// ROS Version Detection (can be overridden by CMake)
// ============================================================================
#ifndef ROS_VERSION_MAJOR
#if defined(RCLCPP__RCLCPP_HPP_)
#define ROS_VERSION_MAJOR 2
#elif defined(ROS_ROS_H)
#define ROS_VERSION_MAJOR 1
#endif
#endif

// ============================================================================
// ROS1 Implementation
// ============================================================================
#if ROS_VERSION_MAJOR == 1

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <stereo_msgs/DisparityImage.h>

namespace ros_compat {

// Message type aliases
using CompressedImage = sensor_msgs::CompressedImage;
using CompressedImageConstPtr = sensor_msgs::CompressedImageConstPtr;
using Image = sensor_msgs::Image;
using ImagePtr = sensor_msgs::ImagePtr;
using PointCloud2 = sensor_msgs::PointCloud2;
using PointCloud2ConstPtr = sensor_msgs::PointCloud2ConstPtr;
using CameraInfo = sensor_msgs::CameraInfo;
using Header = std_msgs::Header;
using DisparityImage = stereo_msgs::DisparityImage;
using DisparityImageConstPtr = stereo_msgs::DisparityImageConstPtr;
using Time = ros::Time;
using Duration = ros::Duration;

// ============================================================================
// Publisher wrapper - provides unified .publish() interface
// ============================================================================
template <typename T>
class Publisher {
 public:
  Publisher() = default;
  explicit Publisher(ros::Publisher pub) : pub_(pub) {}

  void publish(const T& msg) { pub_.publish(msg); }
  void publish(const typename T::Ptr& msg) { pub_.publish(msg); }
  void publish(const typename T::ConstPtr& msg) { pub_.publish(msg); }

  bool valid() const { return pub_.getNumSubscribers() >= 0; }

 private:
  ros::Publisher pub_;
};

// ============================================================================
// Subscriber wrapper
// ============================================================================
template <typename T>
class Subscriber {
 public:
  Subscriber() = default;
  explicit Subscriber(ros::Subscriber sub) : sub_(sub) {}

 private:
  ros::Subscriber sub_;
};

// ============================================================================
// Node wrapper - provides unified API for parameter/pub/sub
// ============================================================================
class Node {
 public:
  Node(const std::string& name) : nh_(), pnh_("~"), name_(name) {}

  // Parameters
  template <typename T>
  T param(const std::string& key, const T& default_val) {
    T val;
    pnh_.param<T>(key, val, default_val);
    return val;
  }

  // Publisher
  template <typename T>
  Publisher<T> advertise(const std::string& topic, int queue_size) {
    return Publisher<T>(nh_.advertise<T>(topic, queue_size));
  }

  // Subscriber
  template <typename T, typename Callback>
  Subscriber<T> subscribe(const std::string& topic, int queue_size, Callback&& cb) {
    return Subscriber<T>(nh_.subscribe<T>(topic, queue_size, std::forward<Callback>(cb)));
  }

  ros::NodeHandle& nh() { return nh_; }
  ros::NodeHandle& pnh() { return pnh_; }
  const std::string& name() const { return name_; }

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::string name_;
};

// Time/Duration helpers
inline Duration durationFromMs(int ms) { return ros::Duration(ms / 1000.0); }
inline Duration absDiff(const Time& a, const Time& b) { return (a > b) ? (a - b) : (b - a); }

// Logging (standalone, no node context needed)
#define RC_LOG_INFO(...) ROS_INFO(__VA_ARGS__)
#define RC_LOG_WARN(...) ROS_WARN(__VA_ARGS__)
#define RC_LOG_ERROR(...) ROS_ERROR(__VA_ARGS__)
#define RC_LOG_FATAL(...) ROS_FATAL(__VA_ARGS__)
#define RC_LOG_INFO_STREAM(x) ROS_INFO_STREAM(x)
#define RC_LOG_ERROR_STREAM(x) ROS_ERROR_STREAM(x)

// Spin
inline void spin() { ros::spin(); }
inline bool ok() { return ros::ok(); }

// String message type
using StringMsg = std_msgs::String;

// Executor wrapper (no-op for ROS1, spin is blocking)
class Executor {
 public:
  Executor() = default;

  // ROS1 doesn't have dynamic node management in executor
  void add_node(void*) {}
  void remove_node(void*) {}

  void spin_some() {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
};

// ROS lifecycle helpers
inline void init(int argc, char** argv, const std::string& name = "odin_ros_driver") {
  ros::init(argc, argv, name);
}
inline void shutdown() { ros::shutdown(); }

}  // namespace ros_compat

// ============================================================================
// ROS2 Implementation
// ============================================================================
#elif ROS_VERSION_MAJOR == 2

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>

namespace ros_compat {

// Message type aliases
using CompressedImage = sensor_msgs::msg::CompressedImage;
using CompressedImageConstPtr = sensor_msgs::msg::CompressedImage::ConstSharedPtr;
using Image = sensor_msgs::msg::Image;
using ImagePtr = sensor_msgs::msg::Image::SharedPtr;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;
using CameraInfo = sensor_msgs::msg::CameraInfo;
using Header = std_msgs::msg::Header;
using DisparityImage = stereo_msgs::msg::DisparityImage;
using DisparityImageConstPtr = stereo_msgs::msg::DisparityImage::ConstSharedPtr;
using Time = rclcpp::Time;
using Duration = rclcpp::Duration;

// ============================================================================
// Publisher wrapper - provides unified .publish() interface
// ============================================================================
template <typename T>
class Publisher {
 public:
  Publisher() = default;
  explicit Publisher(typename rclcpp::Publisher<T>::SharedPtr pub) : pub_(pub) {}

  void publish(const T& msg) { pub_->publish(msg); }
  void publish(const std::shared_ptr<T>& msg) { pub_->publish(*msg); }
  void publish(const std::shared_ptr<const T>& msg) { pub_->publish(*msg); }

  bool valid() const { return pub_ != nullptr; }

 private:
  typename rclcpp::Publisher<T>::SharedPtr pub_;
};

// ============================================================================
// Subscriber wrapper
// ============================================================================
template <typename T>
class Subscriber {
 public:
  Subscriber() = default;
  explicit Subscriber(typename rclcpp::Subscription<T>::SharedPtr sub) : sub_(sub) {}

 private:
  typename rclcpp::Subscription<T>::SharedPtr sub_;
};

// ============================================================================
// Node wrapper - provides unified API for parameter/pub/sub
// ============================================================================
class Node : public rclcpp::Node {
 public:
  Node(const std::string& name) : rclcpp::Node(name) {}

  // Parameters (declare + get)
  template <typename T>
  T param(const std::string& key, const T& default_val) {
    if (!has_parameter(key)) {
      declare_parameter<T>(key, default_val);
    }
    return get_parameter(key).get_value<T>();
  }

  // Publisher with RELIABLE QoS for compatibility with RViz default settings
  // Queue depth should be small (1-5) to avoid latency accumulation
  template <typename T>
  Publisher<T> advertise(const std::string& topic, int queue_size) {
    auto qos = rclcpp::QoS(queue_size)
                   .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                   .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    return Publisher<T>(create_publisher<T>(topic, qos));
  }

  // Subscriber with RELIABLE QoS to match odin_driver publisher
  // Queue depth should be small (1-5) to ensure real-time processing
  template <typename T, typename Callback>
  Subscriber<T> subscribe(const std::string& topic, int queue_size, Callback&& cb) {
    auto qos = rclcpp::QoS(queue_size)
                   .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                   .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    return Subscriber<T>(create_subscription<T>(topic, qos, std::forward<Callback>(cb)));
  }
};

// Time/Duration helpers
inline Duration durationFromMs(int ms) { return rclcpp::Duration(std::chrono::milliseconds(ms)); }
inline Duration absDiff(const Time& a, const Time& b) { return (a > b) ? (a - b) : (b - a); }

// Logging (standalone - uses default logger)
#define RC_LOG_INFO(...) RCLCPP_INFO(rclcpp::get_logger("post_process"), __VA_ARGS__)
#define RC_LOG_WARN(...) RCLCPP_WARN(rclcpp::get_logger("post_process"), __VA_ARGS__)
#define RC_LOG_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("post_process"), __VA_ARGS__)
#define RC_LOG_FATAL(...) RCLCPP_FATAL(rclcpp::get_logger("post_process"), __VA_ARGS__)
#define RC_LOG_INFO_STREAM(x) RCLCPP_INFO_STREAM(rclcpp::get_logger("post_process"), x)
#define RC_LOG_ERROR_STREAM(x) RCLCPP_ERROR_STREAM(rclcpp::get_logger("post_process"), x)

// Spin helper (for shared_ptr node)
template <typename NodeT>
void spin(std::shared_ptr<NodeT> node) {
  rclcpp::spin(node);
}
inline bool ok() { return rclcpp::ok(); }

// String message type
using StringMsg = std_msgs::msg::String;

// Executor wrapper for dynamic node management
class Executor {
 public:
  Executor() : executor_(std::make_shared<rclcpp::executors::MultiThreadedExecutor>()) {}

  void add_node(rclcpp::Node::SharedPtr node) { executor_->add_node(node); }
  void remove_node(rclcpp::Node::SharedPtr node) { executor_->remove_node(node); }

  void spin_some() { executor_->spin_some(std::chrono::milliseconds(100)); }

 private:
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
};

// ROS lifecycle helpers
inline void init(int argc, char** argv) { rclcpp::init(argc, argv); }
inline void shutdown() { rclcpp::shutdown(); }

}  // namespace ros_compat

#else
#error "ROS_VERSION_MAJOR must be 1 or 2"
#endif
