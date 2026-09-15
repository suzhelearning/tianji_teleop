#include "odin2.h"
#include "OdinCommand.hpp"
#include "OdinCaptureBase.hpp"

#include "http_client.h"
#include "logger.h"

#include <Eigen/Dense>

#include <chrono>
#include <random>
#include <thread>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace odin {
namespace sdk {

// Global port tracking for multi-device support
static std::mutex g_port_mutex;
static std::set<uint16_t> g_used_ports;

Odin2Device::Odin2Device(OdinDeviceHandle handle)
    : handle_(handle),
      command_(std::make_unique<OdinCommand>()),
      slam_odom_sync_(std::make_unique<SlamOdomSynchronizer>()) {
  // Set SLAM transform function for coordinate transformation using Odom pose
  // Input: float format (already converted from uint16 by OdinFrameAssembler)
  // Output: float format with rotation and translation applied
  if (slam_odom_sync_) {
    slam_odom_sync_->SetSlamTransformFunction(
        [](OdinPointCloudPacket& packet, const OdinOdomPacket& odom_pkt) {
          // Match found - transform points to world frame: pt_world = R * pt_imu + t
          if (odom_pkt.payload.size() < sizeof(OdinOdomData)) {
            return;
          }
          const OdinOdomData* odom_data =
              reinterpret_cast<const OdinOdomData*>(odom_pkt.payload.data());

          // Extract quaternion and position from odom
          constexpr double kPosScale = 1e-6;
          constexpr double kOrientScale = 1e-6;
          double tx = odom_data->pos[0] * kPosScale;
          double ty = odom_data->pos[1] * kPosScale;
          double tz = odom_data->pos[2] * kPosScale;
          double qx = odom_data->orient[0] * kOrientScale;
          double qy = odom_data->orient[1] * kOrientScale;
          double qz = odom_data->orient[2] * kOrientScale;
          double qw = odom_data->orient[3] * kOrientScale;

          // Build rotation matrix R^{w}_{i} from quaternion using Eigen
          Eigen::Quaterniond q(qw, qx, qy, qz);
          q.normalize();
          Eigen::Matrix3d R = q.toRotationMatrix();
          Eigen::Vector3d t(tx, ty, tz);

          // Transform each point: pt_world = R * pt_imu + t
          // Input is already float format (converted by OdinFrameAssembler)
          size_t point_size = sizeof(OdinSlamPoint<float>);
          size_t point_count = packet.payload.size() / point_size;
          OdinSlamPoint<float>* points =
              reinterpret_cast<OdinSlamPoint<float>*>(packet.payload.data());

          for (size_t i = 0; i < point_count; ++i) {
            // Convert from millimeters to meters for transformation
            Eigen::Vector3d pt_imu(static_cast<double>(points[i].x) * 0.001,
                                   static_cast<double>(points[i].y) * 0.001,
                                   static_cast<double>(points[i].z) * 0.001);

            // Apply rotation and translation: pt_world = R * pt_imu + t
            Eigen::Vector3d pt_world = R * pt_imu + t;

            // Store back as float (in millimeters)
            points[i].x = static_cast<float>(pt_world.x() / 0.001);
            points[i].y = static_cast<float>(pt_world.y() / 0.001);
            points[i].z = static_cast<float>(pt_world.z() / 0.001);
            // r, g, b, a remain unchanged
          }
        });
  }
}

Odin2Device::~Odin2Device() {
  Disconnect();
}

bool Odin2Device::IsPortAvailable(const std::string& ip, uint16_t port) {
  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) return false;

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  if (ip.empty() || ip == "0.0.0.0") {
    addr.sin_addr.s_addr = INADDR_ANY;
  } else {
    inet_pton(AF_INET, ip.c_str(), &addr.sin_addr);
  }

  int result = bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
#ifdef _WIN32
  closesocket(sock);
#else
  close(sock);
#endif
  return result == 0;
}

uint16_t Odin2Device::GenerateAvailablePort() {
  static std::random_device rd;
  static std::mt19937 gen(rd());
  std::uniform_int_distribution<uint16_t> dist(10000, 60000);

  std::lock_guard<std::mutex> lock(g_port_mutex);
  for (int i = 0; i < 1000; ++i) {
    uint16_t port = dist(gen);
    if (g_used_ports.count(port) == 0 && IsPortAvailable(host_ip_, port)) {
      g_used_ports.insert(port);
      return port;
    }
  }
  return 0;
}

bool Odin2Device::Connect(const DiscoveredDevice& discovered_device) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (connected_.load()) return true;

  discovered_device_ = discovered_device;
  device_ip_ = discovered_device.network.ip_address;
  host_ip_ = discovered_device.host_ip.empty() ? "0.0.0.0" : discovered_device.host_ip;
  serial_number_ = discovered_device.sn;
  model_ = discovered_device.model;
  firmware_version_ = discovered_device.firmware_version;

  if (device_ip_.empty()) {
    LOG_ERROR("Odin2Device::Connect: Invalid device IP\n");
    return false;
  }

  // Build transport-agnostic addresses and connect through ICommand
  NetworkAddress device_addr(device_ip_, 0);
  NetworkAddress local_addr(host_ip_, 0);
  if (!command_->Connect(device_addr, &local_addr, handle_)) {
    LOG_ERROR("Odin2Device::Connect: Command connect failed\n");
    return false;
  }

  connected_.store(true);
  LOG_INFO("Odin2Device connected: %s (SN: %s, Model: %s)\n", 
           device_ip_.c_str(), serial_number_.c_str(), model_.c_str());

  StartHeartbeat();
  return true;
}

void Odin2Device::Disconnect() {
  StopHeartbeat();

  std::lock_guard<std::mutex> lock(mutex_);
  if (!connected_.load()) return;

  StopDataChannels();
  command_->Disconnect();

  connected_.store(false);
  LOG_INFO("Odin2Device disconnected: %s\n", device_ip_.c_str());
}

bool Odin2Device::IsConnected() const {
  return connected_.load();
}

OdinDeviceHandle Odin2Device::GetHandle() const {
  return handle_;
}

std::string Odin2Device::GetSerialNumber() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return serial_number_;
}

std::string Odin2Device::GetModel() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return model_;
}

MTConnectionType Odin2Device::GetConnectionType() const {
  return MTConnectionType::kEthernet;
}

const DiscoveredDevice& Odin2Device::GetDiscoveredDevice() const {
  return discovered_device_;
}

OdinResult Odin2Device::GetFirmwareVersion(std::string& version, uint32_t timeout_ms) {
  return command_->GetFirmwareVersion(version, timeout_ms);
}

OdinResult Odin2Device::SetOperatingMode(OdinOperatingMode mode, uint32_t timeout_ms) {
  return command_->SetOperatingMode(mode, timeout_ms);
}

OdinResult Odin2Device::SetSensorMode(uint8_t mode, uint32_t timeout_ms) {
  return command_->SetSensorMode(mode, timeout_ms);
}

OdinResult Odin2Device::GetSensorCapability(std::vector<OdinSensorCapability>& capabilities,
                                             const std::vector<OdinDataChannel>& channels,
                                             uint32_t timeout_ms) {
  return command_->GetSensorCapability(capabilities, channels, timeout_ms);
}

void Odin2Device::EnableSlamOdomSync(bool enabled, uint32_t max_frame_lag) {
  if (slam_odom_sync_) {
    slam_odom_sync_->SetEnabled(enabled);
    slam_odom_sync_->SetMaxFrameLag(max_frame_lag);
    LOG_INFO("Odin2Device[%d]: SLAM-Odom sync %s (max_lag=%u)\n", handle_,
             enabled ? "enabled" : "disabled", max_frame_lag);
  }
}

OdinResult Odin2Device::StartStream(OdinDataChannel channel, OdinTransportMode transport,
                                     const OdinStreamCfg* mode, uint32_t timeout_ms) {
  // Validate: mode parameter only valid for Image channels
  bool is_image_channel = (channel == OdinDataChannel::kImage0 || 
                           channel == OdinDataChannel::kImage1);
  if (mode != nullptr && !is_image_channel) {
    LOG_ERROR("StartStream: mode parameter only valid for Image channels\n");
    return OdinResult::kInvalidArgument;
  }

  // Get port for this channel
  uint16_t* port_ptr = nullptr;
  switch (channel) {
    case OdinDataChannel::kRawPoint:  port_ptr = &ports_.raw_point; break;
    case OdinDataChannel::kSlamPoint: port_ptr = &ports_.slam; break;
    case OdinDataChannel::kImage0:    port_ptr = &ports_.jpeg; break;
    case OdinDataChannel::kImage1:    port_ptr = &ports_.jpeg2; break;
    case OdinDataChannel::kImu:       port_ptr = &ports_.imu; break;
    case OdinDataChannel::kOdom:      port_ptr = &ports_.odom; break;
    default:
      LOG_ERROR("StartStream: unknown channel 0x%02x\n", static_cast<uint8_t>(channel));
      return OdinResult::kInvalidArgument;
  }

  // Allocate port if not already set
  uint16_t dst_port = *port_ptr;
  if (dst_port == 0) {
    dst_port = GenerateAvailablePort();
    if (dst_port == 0) {
      LOG_ERROR("StartStream: failed to allocate port for channel 0x%02x\n",
                static_cast<uint8_t>(channel));
      return OdinResult::kUnknownError;
    }
    *port_ptr = dst_port;
  }

  // Step 1: Send start stream command
  OdinResult result = command_->SendStartStream(channel, transport, dst_port, mode, timeout_ms);
  if (result != OdinResult::kOk) {
    LOG_ERROR("StartStream: SendStartStream failed\n");
    return result;
  }

  // Step 2: Create capture and assembler
  bool use_tcp = (transport == OdinTransportMode::kTcp);
  auto capture = std::make_unique<NetworkCapture>(use_tcp);
  auto assembler = std::make_unique<OdinFrameAssembler>(handle_, use_tcp);

  // Set up assembler callbacks based on channel
  switch (channel) {
    case OdinDataChannel::kRawPoint:
      assembler->SetPointCloudCallback([this](const OdinPointCloudPacket& pkt) {
        if (point_cloud_cb_) point_cloud_cb_(pkt, point_cloud_user_data_);
      });
      break;
    case OdinDataChannel::kSlamPoint:
      assembler->SetSlamCallback([this](const OdinSlamPacket& pkt) {
        // Data is already in float format (converted by OdinFrameAssembler)
        // Route through synchronizer for coordinate transformation if enabled
        bool sync_enabled = slam_odom_sync_ && slam_odom_sync_->IsEnabled();
        if (sync_enabled) {
          // Synchronizer will apply rotation/translation and call user callback
          slam_odom_sync_->ProcessSlam(pkt);
        } else {
          // No sync - call user callback directly (data already in float format)
          if (slam_cb_) {
            slam_cb_(pkt, slam_user_data_);
          }
        }
      });
      break;
    case OdinDataChannel::kImage0:
      assembler->SetImageCallback([this](const OdinImagePacket& pkt) {
        if (image_cb_) image_cb_(pkt, image_user_data_);
      });
      break;
    case OdinDataChannel::kImage1:
      assembler->SetImageCallback([this](const OdinImagePacket& pkt) {
        if (image2_cb_) image2_cb_(pkt, image2_user_data_);
      });
      break;
    case OdinDataChannel::kImu:
      assembler->SetImuCallback([this](const OdinImuPacket& pkt) {
        if (imu_cb_) imu_cb_(pkt, imu_user_data_);
      });
      break;
    case OdinDataChannel::kOdom:
      assembler->SetOdomCallback([this](const OdinOdomPacket& pkt, OdomSourceType type) {
        const bool sync_enabled = slam_odom_sync_ && slam_odom_sync_->IsEnabled();
        const bool is_low_frequency = (type == OdomSourceType::kOdom2ImuLow);

        // Routing rules:
        //   - LF + sync enabled  -> SlamOdomSynchronizer (SLAM/odom alignment).
        //   - LF + sync disabled -> deliver to user callback directly.
        //   - HF (always)        -> deliver to user callback directly, bypassing
        //                            the synchronizer (HF odom must not be gated
        //                            by SLAM frame timing).
        if (is_low_frequency && sync_enabled) {
          slam_odom_sync_->ProcessOdom(pkt, type);
        } else {
          if (odom_cb_) odom_cb_(pkt, type, odom_user_data_);
        }
      });
      break;
    default:
      LOG_ERROR("StartStream: Unknown channel 0x%02x\n", static_cast<uint8_t>(channel));
      command_->SendCloseStream(channel, timeout_ms);
      return OdinResult::kInvalidArgument;
  }

  // Create raw data callback that routes to assembler
  OdinFrameAssembler* asm_ptr = assembler.get();
  RawDataCallback raw_cb;
  switch (channel) {
    case OdinDataChannel::kRawPoint:
      raw_cb = [asm_ptr](const uint8_t* data, size_t len, const ITransportAddress&) {
        asm_ptr->ProcessRawPointData(data, len);
      };
      break;
    case OdinDataChannel::kSlamPoint:
      raw_cb = [asm_ptr](const uint8_t* data, size_t len, const ITransportAddress&) {
        asm_ptr->ProcessSlamPointData(data, len);
      };
      break;
    case OdinDataChannel::kImage0:
    case OdinDataChannel::kImage1:
      raw_cb = [asm_ptr](const uint8_t* data, size_t len, const ITransportAddress&) {
        asm_ptr->ProcessImageData(data, len);
      };
      break;
    case OdinDataChannel::kImu:
      raw_cb = [asm_ptr](const uint8_t* data, size_t len, const ITransportAddress&) {
        asm_ptr->ProcessImuData(data, len);
      };
      break;
    case OdinDataChannel::kOdom:
      raw_cb = [asm_ptr](const uint8_t* data, size_t len, const ITransportAddress&) {
        asm_ptr->ProcessOdomData(data, len);
      };
      break;
    default:
      break;
  }

  // Start capture with address
  NetworkAddress bind_addr(host_ip_, dst_port);
  if (!capture->Start(bind_addr, raw_cb)) {
    LOG_ERROR("StartStream: StartCapture failed\n");
    command_->SendCloseStream(channel, timeout_ms);
    return OdinResult::kSocketError;
  }

  captures_[channel] = std::move(capture);
  assemblers_[channel] = std::move(assembler);

  LOG_INFO("StartStream: channel=0x%02x, port=%u, %s ok\n",
           static_cast<uint8_t>(channel), dst_port,
           transport == OdinTransportMode::kTcp ? "TCP" : "UDP");
  return OdinResult::kOk;
}

OdinResult Odin2Device::CloseStream(OdinDataChannel channel, uint32_t timeout_ms) {
  // Step 1: Stop and remove capture instance
  auto it = captures_.find(channel);
  if (it != captures_.end()) {
    it->second->Stop();
    captures_.erase(it);
  }

  // Remove assembler
  auto asm_it = assemblers_.find(channel);
  if (asm_it != assemblers_.end()) {
    assemblers_.erase(asm_it);
  }

  // Step 2: Send close stream command
  OdinResult result = command_->SendCloseStream(channel, timeout_ms);
  if (result != OdinResult::kOk) {
    LOG_ERROR("CloseStream: SendCloseStream failed\n");
    return result;
  }

  LOG_INFO("CloseStream: channel=0x%02x ok\n", static_cast<uint8_t>(channel));
  return OdinResult::kOk;
}

void Odin2Device::RegisterPointCloudCallback(OdinPointCloudCallback cb, void* user_data) {
  point_cloud_cb_ = cb;
  point_cloud_user_data_ = user_data;
}

void Odin2Device::RegisterSlamCallback(OdinSlamCallback cb, void* user_data) {
  slam_cb_ = cb;
  slam_user_data_ = user_data;
  // Update synchronizer callbacks
  if (slam_odom_sync_) {
    slam_odom_sync_->SetCallbacks(cb, user_data, odom_cb_, odom_user_data_);
  }
}

void Odin2Device::RegisterImageCallback(OdinImageCallback cb, void* user_data) {
  image_cb_ = cb;
  image_user_data_ = user_data;
}

void Odin2Device::RegisterImageCallback2(OdinImageCallback2 cb, void* user_data) {
  image2_cb_ = cb;
  image2_user_data_ = user_data;
}

void Odin2Device::RegisterImuCallback(OdinImuCallback cb, void* user_data) {
  imu_cb_ = cb;
  imu_user_data_ = user_data;
}

void Odin2Device::RegisterOdomCallback(OdinOdomCallback cb, void* user_data) {
  odom_cb_ = cb;
  odom_user_data_ = user_data;
  // Update synchronizer callbacks
  if (slam_odom_sync_) {
    slam_odom_sync_->SetCallbacks(slam_cb_, slam_user_data_, cb, user_data);
  }
}

void Odin2Device::StopDataChannels() {
  for (auto& pair : captures_) {
    if (pair.second) {
      pair.second->Stop();
    }
  }
  captures_.clear();
  assemblers_.clear();
}

// =============================================================================
// File Transfer
// =============================================================================

OdinResult Odin2Device::SendFile(OdinFileType type, const std::string& file_path,
                                  OdinUpgradeProgressCallback cb) {
  if (!IsConnected()) {
    return OdinResult::kNotInitialized;
  }

  if (type == OdinFileType::kFirmware) {
    return SendFileFirmwareOta(file_path, cb);
  }

  // Initialize file transfer
  if (!https_transfer_) {
    https_transfer_ = std::make_unique<HttpsFileTransfer>();
    http::Client probe("http://" + device_ip_ + ":8080");
    if (probe.health_check(2000)) {
      https_transfer_->Connect("http://" + device_ip_ + ":8080");
    } else {
      std::string https_url = device_ip_ + ":" + std::to_string(ports_.file);
      if (!https_transfer_->Connect(https_url)) {
        LOG_ERROR("SendFile: Failed to connect to HTTP server\n");
        https_transfer_.reset();
        return OdinResult::kCommunicationError;
      }
    }
  }

  std::string remote_name;
  switch (type) {
    case OdinFileType::kCalibrationFile: remote_name = "calibration"; break;
    case OdinFileType::kRelocationMap:   remote_name = "relocation_map"; break;
    default:
      LOG_ERROR("SendFile: Unsupported file type: %d\n", static_cast<int>(type));
      return OdinResult::kInvalidArgument;
  }

  FileTransferProgressCallback progress_cb = nullptr;
  if (cb) {
    OdinDeviceHandle h = handle_;
    progress_cb = [cb, h](size_t current, size_t total) {
      if (total > 0) {
        float progress = static_cast<float>(current) * 100.0f / static_cast<float>(total);
        cb(h, progress, nullptr);
      }
    };
  }

  auto result = https_transfer_->Upload(file_path, remote_name, progress_cb);
  if (!result.success) {
    LOG_ERROR("SendFile: Upload failed: %s\n", result.error_msg.c_str());
    return OdinResult::kCommunicationError;
  }

  return OdinResult::kOk;
}

OdinResult Odin2Device::GetFile(OdinFileType type, const std::string& save_path,
                                 OdinUpgradeProgressCallback cb) {
  if (!IsConnected()) {
    return OdinResult::kNotInitialized;
  }

  if (!https_transfer_) {
    https_transfer_ = std::make_unique<HttpsFileTransfer>();
    http::Client probe("http://" + device_ip_ + ":8080");
    if (probe.health_check(2000)) {
      https_transfer_->Connect("http://" + device_ip_ + ":8080");
    } else {
      std::string https_url = device_ip_ + ":" + std::to_string(ports_.file);
      if (!https_transfer_->Connect(https_url)) {
        LOG_ERROR("GetFile: Failed to connect to HTTP server\n");
        https_transfer_.reset();
        return OdinResult::kCommunicationError;
      }
    }
  }

  std::string remote_name;
  switch (type) {
    case OdinFileType::kRelocationMap:   remote_name = "relocation_map"; break;
    case OdinFileType::kCalibrationFile: remote_name = "calibration"; break;
    case OdinFileType::kDevLogFile:      remote_name = "logs"; break;
    default:
      LOG_ERROR("GetFile: Unknown file type: %d\n", static_cast<int>(type));
      return OdinResult::kInvalidArgument;
  }

  FileTransferProgressCallback progress_cb = nullptr;
  if (cb) {
    OdinDeviceHandle h = handle_;
    progress_cb = [cb, h](size_t current, size_t total) {
      if (total > 0) {
        float progress = static_cast<float>(current) * 100.0f / static_cast<float>(total);
        cb(h, progress, nullptr);
      }
    };
  }

  auto result = https_transfer_->Download(remote_name, save_path, progress_cb);
  if (!result.success) {
    LOG_ERROR("GetFile: Download failed: %s\n", result.error_msg.c_str());
    return OdinResult::kCommunicationError;
  }

  if (cb) {
    cb(handle_, 100.0, nullptr);
  }
  return OdinResult::kOk;
}

// =============================================================================
// Firmware OTA
// =============================================================================

namespace {
float MapOtaStateToProgress(const std::string& state, int device_progress) {
  if (state == "UPLOADING") {
    return 50.0f * device_progress / 100.0f;
  } else if (state == "VERIFYING") {
    return 50.0f + 10.0f * device_progress / 100.0f;
  } else if (state == "INSTALLING_MCU") {
    return 60.0f + 10.0f * device_progress / 100.0f;
  } else if (state == "INSTALLING_SOC") {
    return 70.0f + 10.0f * device_progress / 100.0f;
  } else if (state == "REBOOTING") {
    return 80.0f + 15.0f * device_progress / 100.0f;
  } else if (state == "POST_VERIFY") {
    return 95.0f + 5.0f * device_progress / 100.0f;
  } else if (state == "DONE") {
    return 100.0f;
  } else if (state == "FAILED") {
    return -1.0f;
  }
  return 0.0f;
}
}  // namespace

OdinResult Odin2Device::SendFileFirmwareOta(const std::string& firmware_path,
                                             OdinUpgradeProgressCallback cb) {
  constexpr int kDefaultHttpPort = 8080;
  constexpr int kTimeoutS = 300;

  LOG_INFO("SendFileFirmwareOta: Stopping heartbeat for OTA\n");
  StopHeartbeat();

  OdinResult mode_result = command_->SetOperatingMode(OdinOperatingMode::kUpgrade, 3000);
  if (mode_result != OdinResult::kOk) {
    LOG_ERROR("SendFileFirmwareOta: Failed to set upgrade mode\n");
    StartHeartbeat();
    return mode_result;
  }

  int http_port = kDefaultHttpPort;
  {
    std::string probe_url = "http://" + device_ip_ + ":" + std::to_string(kDefaultHttpPort);
    http::Client probe(probe_url);
    if (!probe.health_check(2000) && ports_.file != kDefaultHttpPort) {
      http_port = ports_.file;
    }
  }

  std::string base_url = "http://" + device_ip_ + ":" + std::to_string(http_port);
  http::Client client(base_url);

  if (!client.health_check(3000)) {
    LOG_WARN("SendFileFirmwareOta: HTTP not reachable, falling back to HTTPS\n");
    if (!https_transfer_) {
      https_transfer_ = std::make_unique<HttpsFileTransfer>();
      std::string https_url = device_ip_ + ":" + std::to_string(ports_.file);
      if (!https_transfer_->Connect(https_url)) {
        LOG_ERROR("SendFileFirmwareOta: Failed to connect to HTTPS server\n");
        https_transfer_.reset();
        StartHeartbeat();
        return OdinResult::kCommunicationError;
      }
    }
    FileTransferProgressCallback progress_cb = nullptr;
    if (cb) {
      OdinDeviceHandle h = handle_;
      progress_cb = [cb, h](size_t current, size_t total) {
        if (total > 0) {
          float progress = static_cast<float>(current) * 100.0f / static_cast<float>(total);
          cb(h, progress, nullptr);
        }
      };
    }
    auto result = https_transfer_->Upload(firmware_path, "firmware", progress_cb);
    if (!result.success) {
      LOG_ERROR("SendFileFirmwareOta: HTTPS upload failed: %s\n", result.error_msg.c_str());
      StartHeartbeat();
      return OdinResult::kCommunicationError;
    }
    return OdinResult::kOk;
  }

  http::OtaStatus status;
  if (client.ota_status(status)) {
    if (status.state == "VERIFYING" || status.state == "INSTALLING_MCU" ||
        status.state == "INSTALLING_SOC" || status.state == "REBOOTING" ||
        status.state == "POST_VERIFY") {
      LOG_INFO("SendFileFirmwareOta: OTA already in progress\n");
    } else if (status.state == "UPLOADING" || status.state == "DONE" || 
               status.state == "FAILED") {
      if (!client.ota_reset()) {
        LOG_ERROR("SendFileFirmwareOta: Failed to reset OTA state\n");
        StartHeartbeat();
        return OdinResult::kCommunicationError;
      }
    }
  }

  if (status.state != "VERIFYING" && status.state != "INSTALLING_MCU" &&
      status.state != "INSTALLING_SOC" && status.state != "REBOOTING" &&
      status.state != "POST_VERIFY") {
    
    static OdinUpgradeProgressCallback s_cb = nullptr;
    static OdinDeviceHandle s_handle = kInvalidDeviceHandle;
    s_cb = cb;
    s_handle = handle_;

    auto upload_progress = [](uint64_t current, uint64_t total) -> int {
      if (s_cb && total > 0) {
        float progress = 50.0f * current / total;
        s_cb(s_handle, progress, nullptr);
      }
      return 0;
    };

    if (!client.ota_upload(firmware_path, upload_progress)) {
      LOG_ERROR("SendFileFirmwareOta: Upload failed\n");
      StartHeartbeat();
      return OdinResult::kCommunicationError;
    }

    if (!client.ota_trigger()) {
      LOG_ERROR("SendFileFirmwareOta: Trigger failed\n");
      StartHeartbeat();
      return OdinResult::kCommunicationError;
    }
  }

  auto start_time = std::chrono::steady_clock::now();
  std::string last_state;
  bool in_reboot_phase = false;

  while (true) {
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now() - start_time);
    if (elapsed.count() >= kTimeoutS) {
      if (in_reboot_phase) {
        http::log_set_level(http::kLogInfo);
      }
      LOG_ERROR("SendFileFirmwareOta: Timeout\n");
      return OdinResult::kTimeout;
    }

    if (!client.ota_status(status)) {
      if (!in_reboot_phase) {
        in_reboot_phase = true;
        http::log_set_level(http::kLogNone);
        LOG_INFO("SendFileFirmwareOta: Device rebooting...\n");
      }
      std::this_thread::sleep_for(std::chrono::seconds(3));
      continue;
    }

    if (in_reboot_phase) {
      in_reboot_phase = false;
      http::log_set_level(http::kLogInfo);
      LOG_INFO("SendFileFirmwareOta: Device reconnected\n");
    }

    if (status.state != last_state) {
      LOG_INFO("SendFileFirmwareOta: State: %s\n", status.state.c_str());
      last_state = status.state;
    }

    float progress = MapOtaStateToProgress(status.state, status.progress);
    if (progress < 0) {
      LOG_ERROR("SendFileFirmwareOta: OTA failed: %s\n", status.error.c_str());
      return OdinResult::kUnknownError;
    }

    if (cb) {
      cb(handle_, progress, nullptr);
    }

    if (status.state == "DONE") {
      if (cb) {
        cb(handle_, 100.0f, nullptr);
      }
      LOG_INFO("SendFileFirmwareOta: OTA completed\n");
      if (heartbeat_failed_cb_) {
        heartbeat_failed_cb_(handle_);
      }
      return OdinResult::kOk;
    }

    if (status.state == "FAILED") {
      LOG_ERROR("SendFileFirmwareOta: Device reported failure: %s\n", status.error.c_str());
      return OdinResult::kUnknownError;
    }

    int sleep_ms = (status.state == "REBOOTING") ? 5000 : 2000;
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
  }
}

// =============================================================================
// Heartbeat
// =============================================================================

void Odin2Device::StartHeartbeat() {
  if (heartbeat_running_.load()) return;

  if (!command_->IsHeartbeatSupported()) {
    LOG_INFO("Odin2Device: Heartbeat not supported\n");
    return;
  }

  heartbeat_running_.store(true);
  heartbeat_thread_ = std::thread(&Odin2Device::HeartbeatThread, this);
  LOG_DEBUG("Odin2Device: Heartbeat thread started\n");
}

void Odin2Device::StopHeartbeat() {
  if (!heartbeat_running_.load()) return;

  heartbeat_running_.store(false);

  if (heartbeat_thread_.joinable()) {
    if (std::this_thread::get_id() == heartbeat_thread_.get_id()) {
      heartbeat_thread_.detach();
    } else {
      heartbeat_thread_.join();
    }
  }
  LOG_DEBUG("Odin2Device: Heartbeat thread stopped\n");
}

void Odin2Device::HeartbeatThread() {
  int consecutive_failures = 0;

  while (heartbeat_running_.load()) {
    uint32_t elapsed = 0;
    while (heartbeat_running_.load() && elapsed < heartbeat_interval_ms_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      elapsed += 100;
    }

    if (!heartbeat_running_.load()) break;

    if (!command_->IsHeartbeatSupported()) {
      continue;
    }

    if (command_->SendHeartbeat(heartbeat_interval_ms_, 1000)) {
      consecutive_failures = 0;
    } else {
      consecutive_failures++;
      LOG_WARN("Odin2Device: Heartbeat failed (%d/%d)\n", 
               consecutive_failures, heartbeat_max_failures_);

      if (consecutive_failures >= heartbeat_max_failures_) {
        LOG_ERROR("Odin2Device: Heartbeat failed %d times, device offline\n",
                  consecutive_failures);
        if (heartbeat_failed_cb_) {
          heartbeat_failed_cb_(handle_);
        }
        break;
      }
    }
  }
}

void Odin2Device::SetHeartbeatFailedCallback(HeartbeatFailedCallback cb) {
  heartbeat_failed_cb_ = cb;
}

void Odin2Device::SetHeartbeatInterval(uint32_t interval_ms) {
  heartbeat_interval_ms_ = interval_ms;
  LOG_INFO("Odin2Device: Heartbeat interval set to %u ms\n", interval_ms);
}

void Odin2Device::SetHeartbeatTimeout(uint32_t timeout_ms) {
  heartbeat_max_failures_ = static_cast<int>((timeout_ms + heartbeat_interval_ms_ - 1) / heartbeat_interval_ms_);
  LOG_INFO("Odin2Device: Heartbeat timeout set to %u ms, max failures = %d\n", 
           timeout_ms, heartbeat_max_failures_);
}

OdinResult Odin2Device::GetNetworkAttribute(NetworkAttribute& attr, uint32_t timeout_ms) {
  (void)timeout_ms;  // timeout handled internally by http::Client

  if (!connected_.load()) {
    return OdinResult::kNotInitialized;
  }

  // Use http::Client with base URL http://<device_ip>:8080
  std::string base_url = "http://" + device_ip_ + ":8080";
  http::Client client(base_url);

  std::string response;
  if (!client.get_network(response)) {
    LOG_ERROR("Odin2Device: GetNetworkAttribute failed\n");
    return OdinResult::kUnknownError;
  }

  // Parse JSON response: {"ip_address":"x.x.x.x","gateway":"x.x.x.x","netmask":"x.x.x.x","dhcp":true/false}
  // Simple JSON parsing (no external library dependency)
  auto extract_string = [&response](const std::string& key) -> std::string {
    std::string search = "\"" + key + "\":\"";
    size_t pos = response.find(search);
    if (pos == std::string::npos) return "";
    pos += search.length();
    size_t end = response.find("\"", pos);
    if (end == std::string::npos) return "";
    return response.substr(pos, end - pos);
  };

  auto extract_bool = [&response](const std::string& key) -> bool {
    std::string search = "\"" + key + "\":";
    size_t pos = response.find(search);
    if (pos == std::string::npos) return false;
    pos += search.length();
    // Skip whitespace
    while (pos < response.size() && (response[pos] == ' ' || response[pos] == '\t')) pos++;
    return (response.substr(pos, 4) == "true");
  };

  attr.ip_address = extract_string("ip_address");
  attr.gateway = extract_string("gateway");
  attr.netmask = extract_string("netmask");
  attr.dhcp = extract_bool("dhcp");
  attr.ptp = extract_bool("ptp");

  LOG_INFO("Odin2Device: GetNetworkAttribute success - ip=%s, gateway=%s, netmask=%s, dhcp=%d, ptp=%d\n",
           attr.ip_address.c_str(), attr.gateway.c_str(), attr.netmask.c_str(),
           attr.dhcp ? 1 : 0, attr.ptp ? 1 : 0);

  return OdinResult::kOk;
}

OdinResult Odin2Device::SetNetworkAttribute(const NetworkAttribute& attr, uint32_t timeout_ms) {
  (void)timeout_ms;  // timeout handled internally by http::Client

  if (!connected_.load()) {
    return OdinResult::kNotInitialized;
  }

  // Use http::Client with base URL http://<device_ip>:8080
  std::string base_url = "http://" + device_ip_ + ":8080";
  http::Client client(base_url);

  // Build JSON body
  std::string body = "{";
  body += "\"ip_address\":\"" + attr.ip_address + "\",";
  body += "\"gateway\":\"" + attr.gateway + "\",";
  body += "\"netmask\":\"" + attr.netmask + "\",";
  body += "\"dhcp\":" + std::string(attr.dhcp ? "true" : "false") + ",";
  body += "\"ptp\":" + std::string(attr.ptp ? "true" : "false");
  body += "}";

  if (!client.set_network(body)) {
    LOG_ERROR("Odin2Device: SetNetworkAttribute failed\n");
    return OdinResult::kUnknownError;
  }

  LOG_INFO("Odin2Device: SetNetworkAttribute success - ip=%s, gateway=%s, netmask=%s, dhcp=%d, ptp=%d\n",
           attr.ip_address.c_str(), attr.gateway.c_str(), attr.netmask.c_str(),
           attr.dhcp ? 1 : 0, attr.ptp ? 1 : 0);

  return OdinResult::kOk;
}

OdinResult Odin2Device::Reboot(uint32_t timeout_ms) {
  (void)timeout_ms;  // timeout handled internally by http::Client

  if (!connected_.load()) {
    return OdinResult::kNotInitialized;
  }

  // Use http::Client with base URL http://<device_ip>:8080
  std::string base_url = "http://" + device_ip_ + ":8080";
  http::Client client(base_url);

  LOG_INFO("Odin2Device: Sending reboot command...\n");

  if (!client.reboot()) {
    LOG_ERROR("Odin2Device: Reboot failed\n");
    return OdinResult::kUnknownError;
  }

  LOG_INFO("Odin2Device: Reboot command sent successfully\n");
  return OdinResult::kOk;
}

}  // namespace sdk
}  // namespace odin
