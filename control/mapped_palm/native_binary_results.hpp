// Internal result ABI v1. Little endian, fixed layouts, no native struct padding.
// Input/reset/height and startup retain the existing bounded text protocol.
#pragma once
#include <Eigen/Geometry>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace tianji_native_wire {
using Clock = std::chrono::steady_clock;
inline std::uint64_t elapsed_ns(Clock::time_point start) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now()-start).count();
}
class ResultWriter {
 public:
  explicit ResultWriter(unsigned backend) : backend_(backend) {}
  void byte(unsigned value) {
    if (size_ == data_.size()) throw std::runtime_error("native result overflow");
    data_[size_++] = static_cast<char>(value);
  }
  void integer(std::uint64_t value, unsigned bytes = 8) {
    for (unsigned i=0; i<bytes; ++i) byte((value >> (8*i)) & 255);
  }
  void boolean(bool value) { byte(value ? 1 : 0); }
  void code(int value) { integer(static_cast<std::uint32_t>(value), 4); }
  void real(double value) {
    static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559);
    if (!std::isfinite(value)) throw std::runtime_error("non-finite native result");
    std::uint64_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    integer(bits);
  }
  template<class V> void array(const V& value) {
    for (int i=0; i<value.size(); ++i) real(value[i]);
  }
  template<class R> void common(const R& r, std::int64_t now, bool deterministic) {
    boolean(deterministic); integer(r.tick_id); integer(now);
    integer(r.applied_epoch); integer(r.applied_sequence);
    boolean(r.epoch_reset); boolean(r.freshness.live);
    code(static_cast<int>(r.button_action)); boolean(r.control_executed);
  }
  template<class S, class C> void arm(const S& state, const C& control, double headroom) {
    array(state.q); array(state.qdot); array(state.qddot);
    boolean(control.accepted); code(static_cast<int>(control.ik.status));
    code(static_cast<int>(control.hold_reason)); real(headroom);
    real(control.ik.task_scale_position); real(control.ik.task_scale_orientation);
    array(control.target.position);
    array(Eigen::Quaterniond(control.target.rotation).coeffs());
  }
  template<class G> void guidance(const G& g) {
    array(g.ik.stage1_q); array(g.ik.q); boolean(g.ik.accepted);
    code(g.ik.stage1_iterations); code(g.ik.stage2_iterations); boolean(g.ik.budget_exhausted);
    array(g.feedforward.q); array(g.feedforward.qdot); array(g.feedforward.qddot);
    code(static_cast<int>(g.feedforward.state)); boolean(g.feedforward_target.accepted);
    code(static_cast<int>(g.headroom.state)); boolean(g.settled_hold_active);
    code(static_cast<int>(g.settled_hold_reason)); boolean(g.stationary_joint_reference_held);
  }
  void finish(std::uint64_t solve_ns) {
    const auto encode_ns = elapsed_ns(start_);
    integer(solve_ns); integer(encode_ns);
    const auto payload = size_-8;
    data_[0]='T'; data_[1]='J'; data_[2]='B'; data_[3]='R';
    data_[4]=1; data_[5]=static_cast<char>(backend_);
    data_[6]=static_cast<char>(payload & 255); data_[7]=static_cast<char>((payload >> 8)&255);
    std::cout.write(data_.data(), size_).flush();
    if (!std::cout) throw std::runtime_error("native result write failed");
  }
 private:
  Clock::time_point start_ = Clock::now();
  std::array<char, 2048> data_{};
  std::size_t size_ = 8;
  unsigned backend_;
};
} // namespace tianji_native_wire
