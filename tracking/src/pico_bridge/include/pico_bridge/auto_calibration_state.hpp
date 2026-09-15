#pragma once

#include <cstdint>

namespace pico_bridge {

enum class AutoCalibrationPhase {
  kIdle,
  kAwaitingAcks,
  kSettling,
  kAwaitingFreshImus,
  kSampling,
  kFailed,
};

class AutoCalibrationState {
public:
  std::uint64_t begin() {
    ++generation_;
    phase_ = AutoCalibrationPhase::kAwaitingAcks;
    left_ack_ = false;
    right_ack_ = false;
    return generation_;
  }

  bool accept_zero_ack(std::uint64_t generation, bool left_side, bool success) {
    if (generation != generation_ || phase_ != AutoCalibrationPhase::kAwaitingAcks) {
      return false;
    }
    if (!success) {
      phase_ = AutoCalibrationPhase::kFailed;
      return false;
    }
    if (left_side) {
      left_ack_ = true;
    } else {
      right_ack_ = true;
    }
    if (left_ack_ && right_ack_) {
      phase_ = AutoCalibrationPhase::kSettling;
      return true;
    }
    return false;
  }

  void fail(std::uint64_t generation) {
    if (generation == generation_) {
      phase_ = AutoCalibrationPhase::kFailed;
    }
  }

  void cancel() {
    ++generation_;
    phase_ = AutoCalibrationPhase::kIdle;
    left_ack_ = false;
    right_ack_ = false;
  }

  bool awaiting_fresh_imus() const {
    return phase_ == AutoCalibrationPhase::kAwaitingFreshImus;
  }
  bool settling() const { return phase_ == AutoCalibrationPhase::kSettling; }
  bool finish_settling(std::uint64_t generation) {
    if (generation != generation_ || phase_ != AutoCalibrationPhase::kSettling) return false;
    phase_ = AutoCalibrationPhase::kAwaitingFreshImus;
    return true;
  }
  bool start_sampling(std::uint64_t generation) {
    if (generation != generation_ || phase_ != AutoCalibrationPhase::kAwaitingFreshImus) return false;
    phase_ = AutoCalibrationPhase::kSampling;
    return true;
  }
  bool sampling() const { return phase_ == AutoCalibrationPhase::kSampling; }

  AutoCalibrationPhase phase() const { return phase_; }
  std::uint64_t generation() const { return generation_; }

private:
  std::uint64_t generation_{0};
  AutoCalibrationPhase phase_{AutoCalibrationPhase::kIdle};
  bool left_ack_{false};
  bool right_ack_{false};
};

}  // namespace pico_bridge
