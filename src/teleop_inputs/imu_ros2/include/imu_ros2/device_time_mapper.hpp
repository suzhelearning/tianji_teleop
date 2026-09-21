#ifndef IMU_ROS2__DEVICE_TIME_MAPPER_HPP_
#define IMU_ROS2__DEVICE_TIME_MAPPER_HPP_

#include <cstdint>

#include "rclcpp/rclcpp.hpp"

namespace imu_ros2
{

class DeviceTimeMapper
{
public:
  rclcpp::Time to_stamp(uint32_t device_time_ms, const rclcpp::Time & host_now)
  {
    if (!has_base_) {
      base_device_ms_ = device_time_ms;
      base_ros_time_ = host_now;
      has_base_ = true;
    }

    const uint32_t delta_ms = device_time_ms - base_device_ms_;
    return base_ros_time_ + rclcpp::Duration::from_nanoseconds(
             static_cast<int64_t>(static_cast<uint64_t>(delta_ms) * 1000000ULL));
  }

private:
  bool has_base_{false};
  uint32_t base_device_ms_{0};
  rclcpp::Time base_ros_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace imu_ros2

#endif  // IMU_ROS2__DEVICE_TIME_MAPPER_HPP_
