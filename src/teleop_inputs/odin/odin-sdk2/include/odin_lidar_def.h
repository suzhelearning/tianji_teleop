#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace odin {
namespace sdk {

// ============================================================================
// Basic Types
// ============================================================================

using OdinDeviceHandle = uint32_t;
constexpr OdinDeviceHandle kInvalidDeviceHandle = 0;

// ============================================================================
// Enumerations
// ============================================================================

/**
 * @brief SDK API result codes
 */
enum class OdinResult {
  kOk = 0,              ///< Operation successful
  kNotInitialized,      ///< SDK not initialized or device not connected
  kInvalidArgument,     ///< Invalid argument
  kSocketError,         ///< Network socket error
  kTimeout,             ///< Operation timed out
  kStopped,             ///< Operation stopped
  kFirmwareIllegal,     ///< Invalid firmware file
  kCommunicationError,  ///< Communication error
  kUnknownError         ///< Unknown error
};

/**
 * @brief Device connection type
 */
enum class MTConnectionType : uint8_t {
  kEthernet = 0,   ///< Ethernet (UDP/TCP)
  kUsb = 1,        ///< USB connection
  kUnknown = 0xFF  ///< Unknown connection type
};

/**
 * @brief Device operating mode
 */
enum class OdinOperatingMode : uint8_t {
  kStandby = 0,    ///< Standby mode, stops data streaming
  kNormal = 1,     ///< Normal operating mode, starts data streaming
  kUpgrade = 2,    ///< Upgrade mode
  kUnknown = 0xFF  ///< Unknown mode
};

/**
 * @brief Device file type
 */
enum class OdinFileType : uint8_t {
  kRelocationMap = 0,    ///< Relocation map file
  kCalibrationFile = 1,  ///< Camera calibration file
  kDevLogFile = 2,       ///< Device log file
  kFirmware = 3,         ///< Firmware/OTA file
  kUnknown = 0xFF        ///< Unknown file type
};

enum class OdomSourceType : uint8_t {
  kOdom2ImuLow   = 0,  // (odom→imu_low)
  kOdom2ImuHigh  = 1,  // (odom→imu_high)
  kLidar2Camera  = 2,  // (lidar→camera)
  kLidar2Imu     = 3,  // (lidar→imu)
  kOdom2Map      = 4,  // (odom→map)
};

/**
 * @brief Data channel type for ChannelConfig command (0x05)
 * Uses segmented numbering per protocol design
 */
enum class OdinDataChannel : uint8_t {
  kRawPoint = 0x00,   // Raw point cloud
  kSlamPoint = 0x01,  // SLAM point cloud
  kImage0 = 0x10,     // Left camera
  kImage1 = 0x11,     // Right camera
  kImu = 0x20,        // IMU
  kOdom = 0x30,       // Odometry
};

/**
 * @brief Transport mode for data channel
 */
enum class OdinTransportMode : uint8_t {
  kUdp = 0,  // UDP transport (default)
  kTcp = 1,  // TCP transport
};

/**
 * @brief Unified data format for all sensor types
 * 
 * Format values are organized by sensor type:
 * - Image (0x00-0x0F): MJPEG, YUYV, NV12, NV21, RGB24
 * - RawPoint (0x10-0x1F): XYZIC, XYZ
 * - SlamPoint (0x20-0x2F): XYZRGBA, XYZ
 * - IMU (0x30-0x3F): 6-axis, 9-axis
 * - Odom (0x40-0x4F): Standard
 */
enum class OdinDataFormat : uint8_t {
  // Image formats (0x00-0x0F)
  kImageMjpeg = 0x00,   // MJPEG compressed
  kImageYuyv = 0x01,    // YUYV raw
  kImageNv12 = 0x02,    // NV12
  kImageNv21 = 0x03,    // NV21
  kImageRgb24 = 0x04,   // RGB24

  // RawPoint formats (0x10-0x1F)
  kRawPointXyzic = 0x10,  // XYZ + Intensity + Confidence (8B per point)
  kRawPointXyz = 0x11,    // XYZ only (6B per point, simplified)

  // SlamPoint formats (0x20-0x2F)
  kSlamPointXyzrgba = 0x20,  // XYZ + RGBA (10B per point)
  kSlamPointXyz = 0x21,      // XYZ only (6B per point, simplified)

  // IMU formats (0x30-0x3F)
  kImu6Axis = 0x30,     // Gyro + Acc (6-axis)
  kImu9Axis = 0x31,     // Gyro + Acc + Mag (9-axis)

  // Odom formats (0x40-0x4F)
  kOdomStandard = 0x40, // Standard odometry

  kUnknown = 0xFF
};

/**
 * @brief Stream configuration - a valid combination of resolution, fps, and format
 * 
 * Fields set to 0 or kUnknown will not be sent to device (device uses default).
 * Only set the fields you want to configure.
 * Image format uses OdinDataFormat::kImageXxx series
 */
struct OdinStreamCfg {
  uint16_t width = 0;           // Resolution width (0 = not sent, use device default)
  uint16_t height = 0;          // Resolution height (0 = not sent, use device default)
  uint8_t fps = 0;              // Frame rate (0 = not sent, use device default)
  OdinDataFormat format = OdinDataFormat::kUnknown;  // Image format (kUnknown = not sent)
};

/**
 * @brief Sensor capability - supported modes for a data channel
 */
struct OdinSensorCapability {
  OdinDataChannel channel = OdinDataChannel::kRawPoint;
  std::vector<OdinStreamCfg> modes;                    // Supported mode list
};

struct OdinPointCloudPacket {
  OdinDeviceHandle device = kInvalidDeviceHandle;
  uint8_t version = 0;
  uint16_t length = 0;
  uint16_t point_count = 0;
  uint16_t udp_count = 0;
  uint32_t frame_count = 0;
  uint8_t data_type = 0;
  uint8_t time_type = 0;
  uint16_t reserved = 0;
  uint64_t timestamp = 0;
  std::vector<uint8_t> payload;
};

using OdinSlamPacket = OdinPointCloudPacket;

struct OdinImagePacket {
  OdinDeviceHandle device = kInvalidDeviceHandle;
  uint8_t version = 0;
  uint16_t length = 0;
  uint16_t udp_count = 0;
  uint32_t frame_count = 0;
  uint64_t timestamp = 0;
  std::vector<uint8_t> payload;
};

struct OdinImuPacket {
  OdinDeviceHandle device = kInvalidDeviceHandle;
  uint8_t version = 0;
  uint16_t length = 0;
  uint16_t sample_count = 0;
  uint64_t timestamp = 0;
  std::vector<uint8_t> payload;
};

struct OdinOdomPacket {
  OdinDeviceHandle device = kInvalidDeviceHandle;
  uint8_t version = 0;
  uint16_t length = 0;
  uint16_t data_count = 0;
  uint32_t frame_count = 0;
  uint64_t timestamp = 0;
  std::vector<uint8_t> payload;
};

#pragma pack(push, 1)
template <typename T>
struct OdinRawPoint {
  T x = T(0);
  T y = T(0);
  T z = T(0);
  uint8_t intensity = 0;
  uint8_t confidence = 0;
};
// static_assert(sizeof(OdinRawPoint) == 15, "Unexpected OdinRawPoint size");

template <typename T>
struct OdinSlamPoint {
  T x = T(0);
  T y = T(0);
  T z = T(0);
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;
  uint8_t a = 0;
};
// static_assert(sizeof(OdinSlamPoint) == 16, "Unexpected OdinSlamPoint size");

struct OdinImuSample {
  float gyro_x = 0.0f;
  float gyro_y = 0.0f;
  float gyro_z = 0.0f;
  float acc_x = 0.0f;
  float acc_y = 0.0f;
  float acc_z = 0.0f;
  uint64_t timestamp = 0;
  uint64_t sequence = 0;
};
// static_assert(sizeof(OdinImuSample) == 40, "Unexpected OdinImuSample size");

struct OdinOdomData {
  int64_t pos[3] = {0};
  int64_t orient[4] = {0};
  int64_t linear_velocity[3] = {0};
  int64_t angular_velocity[3] = {0};
  int64_t cov[18] = {0};
  uint8_t type;
};
static_assert(sizeof(OdinOdomData) == 249, "Unexpected OdinOdomData size");
#pragma pack(pop)



/**
 * @brief Network attribute configuration
 *
 * Used for getting/setting device network configuration via HTTP API.
 * Can be embedded in DiscoveredDevice or used standalone.
 */
struct NetworkAttribute {
  std::string ip_address;  ///< Device IP address
  std::string gateway;     ///< Gateway address
  std::string netmask;     ///< Subnet mask
  std::string mac;         ///< Device MAC address
  bool dhcp = false;       ///< DHCP enabled
  bool ptp = false;        ///< PTP enabled
};

/**
 * @brief Discovered device information
 *
 * Contains device identity and network configuration.
 * Network settings are grouped in the 'network' member for easy access.
 */
struct DiscoveredDevice {
  std::string sn;                ///< Device serial number
  std::string model;             ///< Device model name (e.g., "Odin-L1")
  std::string firmware_version;  ///< Device firmware version
  std::string host_ip;           ///< Host interface IP that discovered this device

  NetworkAttribute network;      ///< Network configuration (IP, gateway, netmask, MAC, DHCP, PTP)

  MTConnectionType connection = MTConnectionType::kEthernet;  ///< Connection type
};



// ============================================================================
// Callback Type Definitions
// ============================================================================

/** @brief Raw point cloud callback type */
using OdinPointCloudCallback = void (*)(const OdinPointCloudPacket &packet, void *client_data);
/** @brief SLAM point cloud callback type */
using OdinSlamCallback = void (*)(const OdinSlamPacket &packet, void *client_data);
/** @brief Image callback type (left camera) */
using OdinImageCallback = void (*)(const OdinImagePacket &packet, void *client_data);
/** @brief Image callback type (right camera) */
using OdinImageCallback2 = void (*)(const OdinImagePacket &packet, void *client_data);
/** @brief IMU data callback type */
using OdinImuCallback = void (*)(const OdinImuPacket &packet, void *client_data);
/** @brief Odometry data callback type (with odom source type) */
using OdinOdomCallback = void (*)(const OdinOdomPacket &packet, OdomSourceType odom_type,
                                  void *client_data);

/** @brief Upgrade progress callback type */
using OdinUpgradeProgressCallback = void (*)(OdinDeviceHandle device, float progress,
                                             void *client_data);

}  // namespace sdk
}  // namespace odin
