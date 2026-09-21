#pragma once

#include <deque>
#include <functional>
#include <mutex>
#include <cstdint>
#include "odin_lidar_def.h"

namespace odin {
namespace sdk {

/**
 * @brief SLAM-Odom data synchronizer
 *
 * Matches SLAM and Odom data by frame_count before invoking callbacks.
 * Each device instance has its own synchronizer for independent operation.
 */
class SlamOdomSynchronizer {
 public:
  SlamOdomSynchronizer();
  ~SlamOdomSynchronizer() = default;

  /**
   * @brief Enable/disable synchronization
   * @param enabled Whether to enable sync
   */
  void SetEnabled(bool enabled);

  /**
   * @brief Check if synchronization is enabled
   * @return true if enabled
   */
  bool IsEnabled() const;

  /**
   * @brief Set maximum frame lag tolerance
   * @param max_lag Maximum frame count difference before discarding
   */
  void SetMaxFrameLag(uint32_t max_lag);

  /**
   * @brief Set maximum timestamp difference for frame expiration
   * @param timeout_sec Timeout in seconds (default 1.0)
   */
  void SetFrameTimeout(double timeout_sec);

  /**
   * @brief Set user callbacks
   * @param slam_cb SLAM callback
   * @param slam_user SLAM user data
   * @param odom_cb Odom callback
   * @param odom_user Odom user data
   */
  void SetCallbacks(OdinSlamCallback slam_cb, void* slam_user, OdinOdomCallback odom_cb,
                    void* odom_user);

  /**
   * @brief Set SLAM transform function
   * @param transform_fn Function to transform SLAM data using Odom pose
   *
   * This function will be called when matched Odom data is found.
   * It should transform the SLAM point cloud using the Odom pose data.
   */
  using SlamTransformFn =
      std::function<void(OdinPointCloudPacket& slam_packet, const OdinOdomPacket& odom_packet)>;
  void SetSlamTransformFunction(SlamTransformFn transform_fn);

  /**
   * @brief Process incoming SLAM data
   * @param packet SLAM packet
   *
   * If sync is disabled, invokes callback immediately.
   * If sync is enabled, buffers and tries to match with Odom.
   */
  void ProcessSlam(const OdinPointCloudPacket& packet);

  /**
   * @brief Process incoming Odom data
   * @param packet Odom packet
   *
   * If sync is disabled, invokes callback immediately.
   * If sync is enabled, buffers and tries to match with SLAM.
   */
  void ProcessOdom(const OdinOdomPacket& packet,
                   OdomSourceType odom_type = OdomSourceType::kOdom2ImuLow);

  /**
   * @brief Clear all buffered data
   */
  void Clear();

 private:
  struct SlamItem {
    OdinPointCloudPacket packet;
    uint32_t frame_count;
    uint64_t timestamp;
  };

  struct OdomItem {
    OdinOdomPacket packet;
    uint32_t frame_count;
    uint64_t timestamp;
    OdomSourceType odom_type;
  };

  void TryMatch();
  void CleanupOldData();

  mutable std::mutex mutex_;
  bool enabled_ = true;  // Default enabled
  uint32_t max_frame_lag_ = 10;
  double frame_timeout_sec_ = 1.0;  // Frame expiration timeout in seconds

  OdinSlamCallback slam_callback_ = nullptr;
  void* slam_user_data_ = nullptr;
  OdinOdomCallback odom_callback_ = nullptr;
  void* odom_user_data_ = nullptr;

  SlamTransformFn slam_transform_fn_ = nullptr;

  std::deque<SlamItem> slam_queue_;
  std::deque<OdomItem> odom_queue_;

  // Helper to detect timestamp unit and check if diff exceeds timeout
  bool IsTimestampExpired(uint64_t current_ts, uint64_t old_ts) const;
  // Convert timestamp diff to milliseconds for logging
  uint64_t TimestampDiffToMs(uint64_t current_ts, uint64_t old_ts) const;
};

}  // namespace sdk
}  // namespace odin
