#pragma once

#include <cstdint>

namespace pico_odin {

class GroundReadyGate {
 public:
  void arm() {
    armed_ = true;
    ready_ = false;
    claim_completed_cycle();
    saw_not_ready_ = false;
    defer_completed_cycle_ = false;
  }

  void observe(bool ready) {
    if (!ready) {
      saw_not_ready_ = true;
      if (armed_) {
        // If this gate was already ready, this may be the next reset's
        // false state arriving before its world-reset callback.
        defer_completed_cycle_ = ready_;
        ready_ = false;
      }
      return;
    }
    if (!saw_not_ready_) return;

    saw_not_ready_ = false;
    ++completed_cycle_;
    if (!defer_completed_cycle_) claim_completed_cycle();
    defer_completed_cycle_ = false;
  }

  bool ready() const { return armed_ && ready_; }

  bool consume() {
    if (!ready()) return false;
    armed_ = false;
    saw_not_ready_ = false;
    ready_ = false;
    defer_completed_cycle_ = false;
    return true;
  }

 private:
  void claim_completed_cycle() {
    if (!armed_ || completed_cycle_ == claimed_cycle_) return;
    claimed_cycle_ = completed_cycle_;
    ready_ = true;
  }

  bool armed_{false};
  bool saw_not_ready_{false};
  bool ready_{false};
  bool defer_completed_cycle_{false};
  std::uint64_t completed_cycle_{0};
  std::uint64_t claimed_cycle_{0};
};

}  // namespace pico_odin
