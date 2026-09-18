#pragma once

// Fixed-rate, latest-sample hand scheduler.
//
// This is the application-side owner for the optional native Hand2 route. It
// starts after the existing PICO/Manus driver boundary: the caller supplies a
// validated canonical 21-point frame, session state and source generation.
// The scheduler owns the hand pipeline callback, freshness/epoch gates,
// latest-sample admission, output buffering and its fixed-rate thread. It
// never talks to a device, Zenoh, a robot or MuJoCo directly.

#include <array>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace tianji_hand {

constexpr std::size_t kSchedulerJoints = 20;
constexpr std::size_t kSchedulerPoints = 126;

enum class HandPhase : std::uint8_t {
  idle = 0,
  teleop = 1,
  returning = 2,
  fault = 3,
};

enum class HandOutputStatus : std::uint16_t {
  processed = 1,
  command = 2,
  stale = 3,
};

struct HandSchedulerConfig {
  std::int64_t period_ns = 5'000'000;
  std::int64_t freshness_ns = 200'000'000;
  std::size_t output_capacity = 256;
  // Zero preserves legacy sequence-gap resets. Manus opts into receive-time
  // continuity; transport sequence numbers remain unchanged in every result.
  std::int64_t filter_continuity_ns = 0;
};

struct HandInput {
  std::uint8_t flags = 0;  // bit 0: left, bit 1: right; points are right then left.
  std::uint64_t sequence = 0;
  std::uint64_t timestamp_ns = 0;
  std::uint64_t generation = 0;
  std::array<double, kSchedulerPoints> points{};
};

struct HandSession {
  std::int64_t epoch = 1;
  std::uint64_t sequence = 0;
  std::uint64_t timestamp_ns = 0;
  HandPhase phase = HandPhase::idle;
};

struct HandResetAck {
  std::int64_t epoch=0;
  std::uint64_t sequence=0,requested_ns=0,completed_ns=0;
};

struct HandCommand {
  std::uint64_t output_sequence = 0;
  std::uint64_t input_sequence = 0;
  std::uint64_t input_timestamp_ns = 0;
  std::int64_t epoch = 1;
  std::uint64_t scheduler_timestamp_ns = 0;
  std::uint8_t valid_flags = 0;
  HandPhase phase = HandPhase::idle;
  HandOutputStatus status = HandOutputStatus::processed;
  std::array<double, 40> positions{};
};

struct HandSchedulerStats {
  std::uint64_t accepted_inputs = 0;
  std::uint64_t processed_inputs = 0;
  std::uint64_t superseded_inputs = 0;
  std::uint64_t output_commands = 0;
  std::uint64_t stale_inputs = 0;
  std::uint64_t late_ticks = 0;
  std::uint64_t last_input_sequence = 0;
  std::uint64_t last_processed_sequence = 0;
  std::uint64_t last_output_sequence = 0;
};

class HandScheduler {
 public:
  using SolveFn = std::function<void(const double*, std::uint64_t, double*)>;
  using ResetFn = std::function<void()>;

  HandScheduler(HandSchedulerConfig config, SolveFn left_solver, SolveFn right_solver,
                ResetFn left_reset = {}, ResetFn right_reset = {})
      : config_(config), left_solver_(std::move(left_solver)),
        right_solver_(std::move(right_solver)), left_reset_(std::move(left_reset)),
        right_reset_(std::move(right_reset)), output_ring_(config.output_capacity) {
    if (config_.period_ns <= 0 || config_.period_ns > std::numeric_limits<std::int64_t>::max() / 4 ||
        config_.freshness_ns <= 0 || config_.filter_continuity_ns < 0 ||
        config_.output_capacity == 0 || config_.output_capacity > 8192)
      throw std::invalid_argument("native hand scheduler bounds are invalid");
    if (!left_solver_ || !right_solver_)
      throw std::invalid_argument("native hand scheduler requires both side solvers");
  }

  ~HandScheduler() { stop(); }
  HandScheduler(const HandScheduler&) = delete;
  HandScheduler& operator=(const HandScheduler&) = delete;

  bool submit_session(const HandSession& session) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopped_ || failed_ || !valid_session(session)) return false;
    if (session_seen_ &&
        (session.sequence <= last_session_.sequence ||
         session.timestamp_ns < last_session_.timestamp_ns)) {
      latch_locked("native hand session sequence or timestamp rollback");
      return false;
    }
    if (session.epoch < last_session_epoch_) {
      latch_locked("native hand session epoch rollback");
      return false;
    }
    if (session.phase == HandPhase::fault) {
      latch_locked("native hand session reported fault");
      return false;
    }
    if (session.epoch != last_session_epoch_ || session.phase != last_session_.phase) {
      ++session_revision_;
      output_size_ = 0;
      output_head_ = 0;
    }
    if (session.epoch > last_session_epoch_) {
      // An epoch change is the rearm barrier. The process is not restarted and
      // the source generation is not rewritten; the next input must still be
      // newer than the already processed frame.
      if (latest_input_) last_processed_sequence_ = latest_input_->sequence;
      last_session_epoch_ = session.epoch;
      reset_pending_ = true;
    }
    last_session_ = session;
    session_seen_ = true;
    wake_.notify_all();
    output_ready_.notify_all();
    return true;
  }

  // A submitted epoch is not an ACK. Only the solver owner can publish this
  // after BOTH reset callbacks return, without cancellation or a phase change.
  std::optional<HandResetAck> wait_reset_ack(const HandSession& request,
                                            std::chrono::milliseconds timeout) {
    if(timeout.count()<0 || timeout.count()>60000 || !valid_session(request) ||
       request.sequence==0 || request.phase!=HandPhase::idle)
      throw std::invalid_argument("invalid hand reset ACK wait");
    std::unique_lock<std::mutex> lock(mutex_);
    auto matches=[&] {
      return completed_reset_ && completed_reset_->epoch==request.epoch &&
        completed_reset_->sequence==request.sequence && completed_reset_->requested_ns==request.timestamp_ns;
    };
    auto changed=[&] {
      return last_session_.epoch!=request.epoch || last_session_.sequence!=request.sequence ||
        last_session_.timestamp_ns!=request.timestamp_ns || last_session_.phase!=HandPhase::idle;
    };
    output_ready_.wait_for(lock,timeout,[&]{return stopped_ || failed_ || changed() || matches();});
    if(stopped_ || failed_ || changed() || !matches()) return std::nullopt;
    return completed_reset_;
  }

  bool submit_input(const HandInput& input) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopped_ || failed_ || !valid_input(input)) return false;
    if (input_seen_ &&
        (input.sequence <= last_input_sequence_ || input.timestamp_ns < last_input_timestamp_ns_)) {
      latch_locked("native hand input sequence or timestamp rollback");
      return false;
    }
    if (input_seen_ && input.generation != generation_) {
      latch_locked("native hand input generation changed; recreate the route");
      return false;
    }
    if (!input_seen_) generation_ = input.generation;
    if (latest_input_ && latest_input_->sequence > last_processed_sequence_)
      ++stats_.superseded_inputs;
    latest_input_ = input;
    input_seen_ = true;
    last_input_sequence_ = input.sequence;
    last_input_timestamp_ns_ = input.timestamp_ns;
    ++stats_.accepted_inputs;
    wake_.notify_all();
    return true;
  }

  // Deterministic one-step entry point used by offline tests and by adapters
  // that already own a clock. It performs at most one latest input per call.
  bool tick(std::int64_t now_ns) {
    // Only the tick owner touches stateful solvers. Admission and stop use
    // the short state mutex and never wait for optimizer/reset completion.
    std::lock_guard<std::mutex> owner(tick_mutex_);
    std::unique_lock<std::mutex> lock(mutex_);
    return tick_locked(now_ns, lock);
  }

  bool pop(HandCommand& command) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (output_size_ == 0) return false;
    command = std::move(output_ring_[output_head_]);
    output_head_ = (output_head_ + 1) % output_ring_.size();
    --output_size_;
    return true;
  }

  bool wait_pop(HandCommand& command, std::chrono::milliseconds timeout) {
    if (timeout.count() < 0) throw std::invalid_argument("negative hand output timeout");
    std::unique_lock<std::mutex> lock(mutex_);
    output_ready_.wait_for(lock, timeout, [this] {
      return output_size_ != 0 || stopped_ || failed_;
    });
    if (output_size_ == 0) return false;
    command = std::move(output_ring_[output_head_]);
    output_head_ = (output_head_ + 1) % output_ring_.size();
    --output_size_;
    return true;
  }

  void start() {
    std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);
    std::lock_guard<std::mutex> lock(mutex_);
    if (started_ || stopped_) throw std::logic_error("native hand scheduler is single-use");
    started_ = true;
    thread_ = std::thread([this] { run(); });
  }

  // Interrupt the worker without joining it. The process output writer uses
  // this on a broken stdout; the owner still calls stop() to join safely.
  void request_stop() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopped_ = true;
    }
    wake_.notify_all();
    output_ready_.notify_all();
  }

  void stop() {
    std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);
    request_stop();
    if (thread_.joinable()) thread_.join();
  }

  bool failed() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return failed_;
  }

  std::string failure() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return failure_reason_;
  }

  HandPhase phase() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return session_seen_ ? last_session_.phase : HandPhase::idle;
  }

  std::int64_t epoch() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_session_epoch_;
  }

  bool stopped() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stopped_;
  }

  HandSchedulerStats stats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stats_;
  }

 private:
  static constexpr std::uint8_t kSideFlags = 0x03;
  static constexpr std::uint64_t kMaxInt64 =
      static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max());

  static bool valid_session(const HandSession& session) {
    return session.epoch > 0 && session.sequence <= kMaxInt64 &&
           session.timestamp_ns > 0 && session.timestamp_ns <= kMaxInt64 &&
           static_cast<std::uint8_t>(session.phase) <= static_cast<std::uint8_t>(HandPhase::fault);
  }

  static bool valid_input(const HandInput& input) {
    if (input.flags == 0 || (input.flags & ~kSideFlags) != 0 || input.sequence == 0 ||
        input.sequence > kMaxInt64 || input.timestamp_ns == 0 ||
        input.timestamp_ns > kMaxInt64 || input.generation > kMaxInt64)
      return false;
    for (const double value : input.points)
      if (!std::isfinite(value)) return false;
    return true;
  }

  static std::uint64_t monotonic_now_ns() {
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
  }

  void latch_locked(const std::string& reason) {
    failed_ = true;
    failure_reason_ = reason;
    last_session_.phase = HandPhase::fault;
    output_size_ = 0;
    output_head_ = 0;
    wake_.notify_all();
    output_ready_.notify_all();
  }

  bool tick_locked(std::int64_t now_ns, std::unique_lock<std::mutex>& lock) {
    if (stopped_ || failed_) return false;
    if (now_ns <= 0 || now_ns > std::numeric_limits<std::int64_t>::max() || now_ns < last_tick_ns_) {
      latch_locked("native hand scheduler clock rollback");
      return false;
    }
    last_tick_ns_ = now_ns;
    const auto revision = session_revision_;
    if (reset_pending_) {
      reset_pending_ = false;
      const auto reset_session=last_session_;
      lock.unlock();
      try {
        if (left_reset_) left_reset_();
        if (right_reset_) right_reset_();
        solver_sequences_ = {};
        solver_timestamps_ = {};
      } catch (const std::exception& error) {
        lock.lock();
        latch_locked(std::string("native hand reset failed: ") + error.what());
        return false;
      } catch (...) {
        lock.lock();
        latch_locked("native hand reset failed");
        return false;
      }
      lock.lock();
      if (stopped_ || failed_ || revision != session_revision_) return false;
      if(reset_session.phase==HandPhase::idle) {
        completed_reset_=HandResetAck{reset_session.epoch,reset_session.sequence,
                                     reset_session.timestamp_ns,monotonic_now_ns()};
        output_ready_.notify_all();
      }
    }
    if (!latest_input_ || latest_input_->sequence <= last_processed_sequence_) return false;
    const HandInput input = *latest_input_;
    // Input may arrive after the tick clock sample (or during unlocked reset
    // callbacks). Leave it unclaimed until the next tick instead of emitting
    // inverted timestamps or advancing a stateful solver on that older clock.
    if (input.timestamp_ns > static_cast<std::uint64_t>(now_ns)) return false;
    // Claim before releasing the mutex: a new input supersedes only another
    // waiting input, never the frame already being solved.
    last_processed_sequence_ = input.sequence;
    HandCommand command;
    command.output_sequence = next_output_sequence_;
    command.input_sequence = input.sequence;
    command.input_timestamp_ns = input.timestamp_ns;
    command.epoch = session_seen_ ? last_session_.epoch : last_session_epoch_;
    command.scheduler_timestamp_ns = static_cast<std::uint64_t>(now_ns);
    command.valid_flags = input.flags;
    command.phase = session_seen_ ? last_session_.phase : HandPhase::idle;
    const bool fresh = now_ns >= static_cast<std::int64_t>(input.timestamp_ns) &&
                       now_ns - static_cast<std::int64_t>(input.timestamp_ns) <= config_.freshness_ns;
    // Do not pass an expired teleoperation frame through a stateful retarget,
    // filter or optimizer. Apart from being fail-closed at the output
    // boundary, this keeps a stale frame from advancing the native pipeline's
    // internal state and changing the first command after input recovers.
    if (command.phase == HandPhase::teleop && !fresh) {
      command.status = HandOutputStatus::stale;
    } else {
      lock.unlock();
      try {
        if (input.flags & 0x01)
          left_solver_(input.points.data() + kSchedulerPoints / 2, solver_sequence(0, input), command.positions.data());
        if (input.flags & 0x02)
          right_solver_(input.points.data(), solver_sequence(1, input), command.positions.data() + kSchedulerJoints);
      } catch (const std::exception& error) {
        lock.lock();
        latch_locked(std::string("native hand solver rejected input: ") + error.what());
        return false;
      } catch (...) {
        lock.lock();
        latch_locked("native hand solver rejected input");
        return false;
      }
      lock.lock();
      command.status = command.phase == HandPhase::teleop ? HandOutputStatus::command
                                                            : HandOutputStatus::processed;
    }
    if (stopped_ || failed_ || revision != session_revision_) return false;
    if (command.status == HandOutputStatus::stale) ++stats_.stale_inputs;
    if (output_size_ == output_ring_.size()) {
      latch_locked("native hand scheduler output queue overflow");
      return false;
    }
    const auto tail = (output_head_ + output_size_) % output_ring_.size();
    output_ring_[tail] = std::move(command);
    ++output_size_;
    ++next_output_sequence_;
    ++stats_.processed_inputs;
    ++stats_.output_commands;
    stats_.last_processed_sequence = input.sequence;
    stats_.last_output_sequence = next_output_sequence_ - 1;
    output_ready_.notify_all();
    return true;
  }

  void run() noexcept {
    try {
      auto next = std::chrono::steady_clock::now();
      while (true) {
        {
          std::unique_lock<std::mutex> lock(mutex_);
          if (stopped_ || failed_) {
            stopped_ = true;
            break;
          }
          const auto now = std::chrono::steady_clock::now();
          if (now >= next) {
            const auto now_ns = static_cast<std::int64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                now.time_since_epoch()).count());
            if (next != std::chrono::steady_clock::time_point{} && now > next + std::chrono::nanoseconds(config_.period_ns))
              ++stats_.late_ticks;
            lock.unlock();
            tick(now_ns);
            lock.lock();
            next += std::chrono::nanoseconds(config_.period_ns);
            if (now > next + std::chrono::nanoseconds(config_.period_ns * 4))
              next = now + std::chrono::nanoseconds(config_.period_ns);
            continue;
          }
          wake_.wait_until(lock, next, [this] { return stopped_ || failed_; });
        }
      }
    } catch (...) {
      std::lock_guard<std::mutex> lock(mutex_);
      failed_ = true;
      stopped_ = true;
      last_session_.phase = HandPhase::fault;
      failure_reason_ = "native hand scheduler loop failed";
    }
    output_ready_.notify_all();
  }

  // Called only by the serialized solver owner, not input/transport threads.
  std::uint64_t solver_sequence(std::size_t side, const HandInput& input) {
    if (!config_.filter_continuity_ns) return input.sequence;
    const auto previous = solver_timestamps_[side];
    const std::uint64_t increment = previous &&
        input.timestamp_ns - previous > static_cast<std::uint64_t>(config_.filter_continuity_ns) ? 2 : 1;
    if (solver_sequences_[side] > std::numeric_limits<std::uint64_t>::max() - increment)
      throw std::overflow_error("native hand solver sequence exhausted");
    solver_timestamps_[side] = input.timestamp_ns;
    return solver_sequences_[side] += increment;
  }
  std::array<std::uint64_t,2> solver_sequences_{}, solver_timestamps_{};
  HandSchedulerConfig config_;
  SolveFn left_solver_;
  SolveFn right_solver_;
  ResetFn left_reset_;
  ResetFn right_reset_;
  std::vector<HandCommand> output_ring_;
  std::size_t output_head_ = 0;
  std::size_t output_size_ = 0;
  mutable std::mutex mutex_;
  std::mutex tick_mutex_;
  std::mutex lifecycle_mutex_;
  std::condition_variable wake_;
  std::condition_variable output_ready_;
  std::thread thread_;
  bool started_ = false;
  bool stopped_ = false;
  bool failed_ = false;
  bool session_seen_ = false;
  bool input_seen_ = false;
  bool reset_pending_ = false;
  std::uint64_t session_revision_ = 0;
  std::optional<HandInput> latest_input_;
  HandSession last_session_;
  std::optional<HandResetAck> completed_reset_;
  std::int64_t last_session_epoch_ = 1;
  std::int64_t last_tick_ns_ = 0;
  std::uint64_t generation_ = 0;
  std::uint64_t last_input_sequence_ = 0;
  std::uint64_t last_input_timestamp_ns_ = 0;
  std::uint64_t last_processed_sequence_ = 0;
  std::uint64_t next_output_sequence_ = 1;
  std::string failure_reason_;
  HandSchedulerStats stats_;
};

}  // namespace tianji_hand
