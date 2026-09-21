#include "SlamOdomSynchronizer.h"
#include "logger.h"
#include <sstream>
#include <fstream>

namespace odin {
namespace sdk {

namespace {
// Helper to format queue frame IDs as string for logging
template <typename Queue>
std::string FormatQueueIds(const Queue& queue) {
  std::ostringstream oss;
  oss << "[";
  bool first = true;
  for (const auto& item : queue) {
    if (!first) oss << ",";
    oss << static_cast<int>(item.frame_count);
    first = false;
  }
  oss << "]";
  return oss.str();
}
// Global CSV file for SLAM data logging
// static std::ofstream g_slam_csv_file;
// static bool g_slam_csv_initialized = false;

// void InitSlamCsvFile() {
//   if (!g_slam_csv_initialized) {
//     g_slam_csv_file.open("/home/xxx/work/odin2/ros_fold/slam_data.csv");
//     if (g_slam_csv_file.is_open()) {
//       g_slam_csv_file << "timestamp,seq\n";
//       g_slam_csv_file.flush();
//     }
//     g_slam_csv_initialized = true;
//   }
// }

}  // namespace

SlamOdomSynchronizer::SlamOdomSynchronizer() {
  // InitSlamCsvFile();
}

void SlamOdomSynchronizer::SetEnabled(bool enabled) {
  std::lock_guard<std::mutex> lock(mutex_);
  enabled_ = enabled;
  if (!enabled_) {
    // Clear queues when disabling
    slam_queue_.clear();
    odom_queue_.clear();
  }
}

bool SlamOdomSynchronizer::IsEnabled() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return enabled_;
}

void SlamOdomSynchronizer::SetMaxFrameLag(uint32_t max_lag) {
  std::lock_guard<std::mutex> lock(mutex_);
  max_frame_lag_ = max_lag;
}

void SlamOdomSynchronizer::SetFrameTimeout(double timeout_sec) {
  std::lock_guard<std::mutex> lock(mutex_);
  frame_timeout_sec_ = timeout_sec;
}

bool SlamOdomSynchronizer::IsTimestampExpired(uint64_t current_ts, uint64_t old_ts) const {
  if (current_ts <= old_ts) return false;
  uint64_t diff = current_ts - old_ts;

  // Timestamp is in microseconds (device monotonic clock)
  // Convert diff to seconds and compare with timeout
  double diff_sec = static_cast<double>(diff) / 1e6;  // μs -> s
  return diff_sec > frame_timeout_sec_;
}

uint64_t SlamOdomSynchronizer::TimestampDiffToMs(uint64_t current_ts, uint64_t old_ts) const {
  if (current_ts <= old_ts) return 0;
  uint64_t diff = current_ts - old_ts;

  // Timestamp is in microseconds, convert to milliseconds
  return diff / 1000;  // μs -> ms
}

void SlamOdomSynchronizer::SetCallbacks(OdinSlamCallback slam_cb, void* slam_user,
                                        OdinOdomCallback odom_cb, void* odom_user) {
  std::lock_guard<std::mutex> lock(mutex_);
  slam_callback_ = slam_cb;
  slam_user_data_ = slam_user;
  odom_callback_ = odom_cb;
  odom_user_data_ = odom_user;
}

void SlamOdomSynchronizer::SetSlamTransformFunction(SlamTransformFn transform_fn) {
  std::lock_guard<std::mutex> lock(mutex_);
  slam_transform_fn_ = transform_fn;
}

void SlamOdomSynchronizer::ProcessSlam(const OdinPointCloudPacket& packet) {
  std::lock_guard<std::mutex> lock(mutex_);

  // recordCSV
  // if (g_slam_csv_file.is_open()) {
  //   g_slam_csv_file << packet.timestamp << "," << static_cast<int>(packet.frame_count) << "\n";
  //   g_slam_csv_file.flush();
  // }

  // LOG_WARN("--slam timestamp: %llu seq:%d\n",packet.timestamp,packet.frame_count);
  // If sync disabled, invoke callback immediately
  if (!enabled_) {
    if (slam_callback_) {
      slam_callback_(packet, slam_user_data_);
    }
    return;
  }

  // Check timestamp rollback/duplicate: new frame must have timestamp > all existing frames
  if (!slam_queue_.empty()) {
    for (const auto& existing : slam_queue_) {
      if (packet.timestamp <= existing.timestamp) {
        LOG_WARN(
            "SlamOdomSync: SLAM timestamp rollback/duplicate detected! incoming_ts=%llu, "
            "existing_ts=%llu, incoming_seq=%u, existing_seq=%u\n",
            packet.timestamp, existing.timestamp, packet.frame_count, existing.frame_count);
        return;  // Reject this frame
      }
    }
  }

  // Discard all frames older than timeout compared to incoming frame (unordered queue)
  auto slam_it = slam_queue_.begin();
  while (slam_it != slam_queue_.end()) {
    if (IsTimestampExpired(packet.timestamp, slam_it->timestamp)) {
      LOG_WARN(
          "SlamOdomSync: SLAM frame expired, discarding frame_id=%u (age=%lums, incoming=%u), "
          "discarded_ts=%llu, incoming_ts=%llu\n",
          slam_it->frame_count, TimestampDiffToMs(packet.timestamp, slam_it->timestamp),
          packet.frame_count, slam_it->timestamp, packet.timestamp);
      slam_it = slam_queue_.erase(slam_it);
    } else {
      ++slam_it;
    }
  }

  // Discard oldest frame when queue is full
  if (slam_queue_.size() >= max_frame_lag_) {
    LOG_WARN("SlamOdomSync: SLAM queue full (%zu), incoming=%u, slam_ids=%s, odom_ids=%s\n",
             slam_queue_.size(), packet.frame_count, FormatQueueIds(slam_queue_).c_str(),
             FormatQueueIds(odom_queue_).c_str());
    while (slam_queue_.size() >= max_frame_lag_) {
      slam_queue_.pop_front();
    }
  }

  // Add new frame to queue
  SlamItem item;
  item.packet = packet;
  item.frame_count = packet.frame_count;
  item.timestamp = packet.timestamp;
  slam_queue_.push_back(item);

  // Try to match
  TryMatch();
}

void SlamOdomSynchronizer::ProcessOdom(const OdinOdomPacket& packet, OdomSourceType odom_type) {
  std::lock_guard<std::mutex> lock(mutex_);

  // Always publish odom immediately (odom doesn't need coordinate transformation)
  if (odom_callback_) {
    odom_callback_(packet, odom_type, odom_user_data_);
  }

  // If sync disabled, no need to queue for SLAM matching
  if (!enabled_) {
    return;
  }

  // Check timestamp rollback/duplicate: new frame must have timestamp > all existing frames
  if (!odom_queue_.empty()) {
    for (const auto& existing : odom_queue_) {
      if (packet.timestamp <= existing.timestamp) {
        LOG_WARN(
            "SlamOdomSync: Odom timestamp rollback/duplicate detected! incoming_ts=%llu, "
            "existing_ts=%llu, incoming_seq=%u, existing_seq=%u\n",
            packet.timestamp, existing.timestamp, packet.frame_count, existing.frame_count);
        return;  // Reject this frame
      }
    }
  }

  // Discard all frames older than timeout compared to incoming frame (unordered queue)
  auto odom_it = odom_queue_.begin();
  while (odom_it != odom_queue_.end()) {
    if (IsTimestampExpired(packet.timestamp, odom_it->timestamp)) {
      LOG_WARN(
          "SlamOdomSync: Odom frame expired, discarding frame_id=%u (age=%lums, incoming=%u), "
          "discarded_ts=%llu, incoming_ts=%llu\n",
          odom_it->frame_count, TimestampDiffToMs(packet.timestamp, odom_it->timestamp),
          packet.frame_count, odom_it->timestamp, packet.timestamp);
      odom_it = odom_queue_.erase(odom_it);
    } else {
      ++odom_it;
    }
  }

  // Discard oldest frame when queue is full
  if (odom_queue_.size() >= max_frame_lag_) {
    LOG_WARN("SlamOdomSync: Odom queue full (%zu), incoming=%u, slam_ids=%s, odom_ids=%s\n",
             odom_queue_.size(), packet.frame_count, FormatQueueIds(slam_queue_).c_str(),
             FormatQueueIds(odom_queue_).c_str());
    while (odom_queue_.size() >= max_frame_lag_) {
      odom_queue_.pop_front();
    }
  }

  // Add new frame to queue
  OdomItem item;
  item.packet = packet;
  item.frame_count = packet.frame_count;
  item.timestamp = packet.timestamp;
  item.odom_type = odom_type;
  odom_queue_.push_back(item);

  // Try to match
  TryMatch();
}

void SlamOdomSynchronizer::TryMatch() {
  // Must be called with mutex_ locked
  //
  // Sync strategy: match only when frame_id is exactly equal
  // Search both queues to find matching frames

  if (slam_queue_.empty() || odom_queue_.empty()) {
    return;
  }

  // Search SLAM queue for a frame matching any frame in Odom queue
  for (auto slam_it = slam_queue_.begin(); slam_it != slam_queue_.end(); ++slam_it) {
    for (auto odom_it = odom_queue_.begin(); odom_it != odom_queue_.end(); ++odom_it) {
      if (slam_it->frame_count == odom_it->frame_count) {
        // Found a match
        if (slam_it->packet.timestamp != odom_it->packet.timestamp) {
          LOG_WARN(
              "SlamOdomSync: SLAM timestamp and odom timestamp mismatch,but seq[frame cout] is "
              "same, slam=%lu, odom=%lu,slamSeq =%d odomSeq=%d\n",
              slam_it->packet.timestamp, odom_it->packet.timestamp, slam_it->frame_count,
              odom_it->frame_count);
        }
        if (slam_transform_fn_) {
          slam_transform_fn_(slam_it->packet, odom_it->packet);
        }

        // Invoke SLAM callback (odom already published in ProcessOdom)
        if (slam_callback_) {
          slam_callback_(slam_it->packet, slam_user_data_);
        }

        // Remove matched frames
        slam_queue_.erase(slam_it);
        odom_queue_.erase(odom_it);
        return;  // Match one pair at a time
      }
    }
  }
  // No match found, wait for more data
}

void SlamOdomSynchronizer::CleanupOldData() {
  // Must be called with mutex_ locked

  // Limit queue sizes to prevent unbounded growth
  constexpr size_t kMaxQueueSize = 100;

  while (slam_queue_.size() > kMaxQueueSize) {
    LOG_WARN("SlamOdomSync: SLAM queue overflow, discarding frame %u\n",
             slam_queue_.front().frame_count);
    slam_queue_.pop_front();
  }

  while (odom_queue_.size() > kMaxQueueSize) {
    LOG_WARN("SlamOdomSync: Odom queue overflow, discarding frame %u\n",
             odom_queue_.front().frame_count);
    odom_queue_.pop_front();
  }
}

void SlamOdomSynchronizer::Clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  slam_queue_.clear();
  odom_queue_.clear();
}

}  // namespace sdk
}  // namespace odin
