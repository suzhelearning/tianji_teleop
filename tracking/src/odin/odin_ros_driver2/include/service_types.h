#pragma once

// =============================================================================
// Service Type Definitions
// =============================================================================
// This header consolidates all ROS service includes and type aliases.
// When adding new services:
//   1. Add the srv file to srv/
//   2. Add the include here
//   3. Add a type alias if desired
// No changes needed in odin_ros_driver_node.cpp includes.
// =============================================================================

#if defined(ODIN_ROS2)
// ROS2 service headers
#include "odin_ros_driver_rev1/srv/get_calibration.hpp"
#include "odin_ros_driver_rev1/srv/set_sensor_mode.hpp"

namespace odin_ros_driver {
namespace srv {
using GetCalibration = odin_ros_driver_rev1::srv::GetCalibration;
using SetSensorMode = odin_ros_driver_rev1::srv::SetSensorMode;
}  // namespace srv
}  // namespace odin_ros_driver

#else
// ROS1 service headers
#include "odin_ros_driver_rev1/GetCalibration.h"
#include "odin_ros_driver_rev1/SetSensorMode.h"

namespace odin_ros_driver {
namespace srv {
using GetCalibration = odin_ros_driver_rev1::GetCalibration;
using SetSensorMode = odin_ros_driver_rev1::SetSensorMode;
}  // namespace srv
}  // namespace odin_ros_driver

#endif
