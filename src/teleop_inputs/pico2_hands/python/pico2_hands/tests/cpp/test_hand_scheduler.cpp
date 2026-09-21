#include "../../native/hand/scheduler.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <future>

using namespace tianji_hand;

namespace {
HandInput input(std::uint64_t sequence, std::uint64_t timestamp,
                std::uint8_t flags = 3, std::uint64_t generation = 7) {
  HandInput value;
  value.flags = flags;
  value.sequence = sequence;
  value.timestamp_ns = timestamp;
  value.generation = generation;
  for (std::size_t i = 0; i < value.points.size(); ++i)
    value.points[i] = static_cast<double>(i) / 1000.0;
  return value;
}

HandSession session(std::uint64_t sequence, std::uint64_t timestamp,
                    HandPhase phase, std::int64_t epoch = 1) {
  return HandSession{epoch, sequence, timestamp, phase};
}
}

int main() {
  {
    std::vector<std::uint64_t> left, right;
    HandSchedulerConfig cfg;
    cfg.filter_continuity_ns = 200;
    HandScheduler continuous(cfg,
      [&](const double*, std::uint64_t seq, double*) { left.push_back(seq); },
      [&](const double*, std::uint64_t seq, double*) { right.push_back(seq); });
    HandCommand result;
    for (auto frame : {input(1,100), input(8,200), input(12,400), input(15,601)}) {
      assert(continuous.submit_input(frame));
      assert(continuous.tick(frame.timestamp_ns));
      assert(continuous.pop(result));
      assert(result.input_sequence == frame.sequence);
    }
    assert((left == std::vector<std::uint64_t>{1,2,3,5}));
    assert(right == left);
    // A missing side's continuity is independent of fresh opposite-hand data.
    assert(continuous.submit_input(input(16,700,2)));
    assert(continuous.tick(700));assert(continuous.pop(result));
    assert(continuous.submit_input(input(20,850)));
    assert(continuous.tick(850));assert(continuous.pop(result));
    assert(left.back()==7 && right.back()==7);
  }
  std::size_t resets = 0;
  HandScheduler scheduler(
      HandSchedulerConfig{5'000'000, 200'000'000, 4},
      [](const double* points, std::uint64_t sequence, double* joints) {
        for (std::size_t i = 0; i < kSchedulerJoints; ++i)
          joints[i] = points[0] + static_cast<double>(sequence) + static_cast<double>(i);
      },
      [](const double* points, std::uint64_t sequence, double* joints) {
        for (std::size_t i = 0; i < kSchedulerJoints; ++i)
          joints[i] = points[0] + static_cast<double>(sequence) + static_cast<double>(i);
      },
      [&] { ++resets; }, [&] { ++resets; });

  assert(scheduler.submit_session(session(1, 100, HandPhase::idle)));
  assert(scheduler.submit_input(input(1, 100)));
  assert(scheduler.tick(100));
  HandCommand command;
  assert(scheduler.pop(command));
  assert(command.status == HandOutputStatus::processed);
  assert(command.valid_flags == 3);
  assert(command.positions[0] == 1.063);
  assert(command.positions[kSchedulerJoints] == 1.0);

  // Latest-only admission is explicit and bounded: an intermediate input is
  // discarded before the next fixed-rate tick, never processed twice.
  assert(scheduler.submit_input(input(2, 200)));
  assert(scheduler.submit_input(input(3, 300)));
  assert(scheduler.tick(300));
  assert(scheduler.pop(command));
  assert(command.input_sequence == 3);
  assert(scheduler.stats().superseded_inputs == 1);

  assert(scheduler.submit_session(session(2, 400, HandPhase::teleop)));
  assert(scheduler.submit_input(input(4, 400)));
  assert(scheduler.tick(400));
  assert(scheduler.pop(command));
  assert(command.status == HandOutputStatus::command);
  assert(command.epoch == 1);

  // A reconnect/generation change is fail-closed and cannot silently reset the
  // filter or scheduler state.
  assert(!scheduler.submit_input(input(5, 500, 3, 8)));
  assert(scheduler.failed());
  assert(scheduler.phase() == HandPhase::fault);

  // A fresh scheduler reset is only driven by a new epoch; it never clears a
  // latched fault on the old instance.
  assert(resets == 0);
  assert(!scheduler.submit_session(session(3, 600, HandPhase::idle, 2)));

  HandScheduler epoch_scheduler(
      HandSchedulerConfig{5'000'000, 200'000'000, 2},
      [](const double*, std::uint64_t, double* joints) { for (std::size_t i = 0; i < kSchedulerJoints; ++i) joints[i] = 0.0; },
      [](const double*, std::uint64_t, double* joints) { for (std::size_t i = 0; i < kSchedulerJoints; ++i) joints[i] = 0.0; },
      [&] { ++resets; }, [&] { ++resets; });
  assert(epoch_scheduler.submit_session(session(1, 100, HandPhase::idle, 1)));
  assert(epoch_scheduler.submit_input(input(1, 100)));
  assert(epoch_scheduler.tick(100));
  assert(epoch_scheduler.submit_session(session(2, 200, HandPhase::idle, 2)));
  assert(!epoch_scheduler.tick(200));
  assert(resets == 2);

  // An expired teleoperation frame is emitted as stale without entering the
  // stateful native pipeline. A recovered frame can therefore start from the
  // last valid optimizer/filter state instead of inheriting stale input.
  std::size_t stale_solves = 0;
  HandScheduler stale_scheduler(
      HandSchedulerConfig{5'000'000, 200'000'000, 2},
      [&](const double*, std::uint64_t, double* joints) {
        ++stale_solves;
        for (std::size_t i = 0; i < kSchedulerJoints; ++i) joints[i] = 1.0;
      },
      [&](const double*, std::uint64_t, double* joints) {
        ++stale_solves;
        for (std::size_t i = 0; i < kSchedulerJoints; ++i) joints[i] = 2.0;
      });
  assert(stale_scheduler.submit_session(session(1, 100, HandPhase::teleop)));
  assert(stale_scheduler.submit_input(input(1, 100)));
  assert(stale_scheduler.tick(200'000'101));
  assert(stale_scheduler.pop(command));
  assert(command.status == HandOutputStatus::stale);
  assert(stale_solves == 0);
  assert(stale_scheduler.stats().stale_inputs == 1);
  assert(stale_scheduler.submit_input(input(2, 200'000'102)));
  assert(stale_scheduler.tick(200'000'102));
  assert(stale_scheduler.pop(command));
  assert(command.status == HandOutputStatus::command);
  assert(stale_solves == 2);
  // A solver fault must terminate the scheduler, even with no more input.
  HandScheduler failed_scheduler(
      HandSchedulerConfig{},
      [](const double*, std::uint64_t, double*) { throw std::runtime_error("solver failed"); },
      [](const double*, std::uint64_t, double*) {});
  assert(failed_scheduler.submit_input(input(1, 100)));
  failed_scheduler.start();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (!failed_scheduler.stopped() && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  assert(failed_scheduler.failed());
  assert(failed_scheduler.stopped());

  // A blocked optimizer must not block input/session admission. Its old
  // teleop result must be fenced when the return event arrives during solve.
  std::promise<void> entered, release;
  auto released = release.get_future().share();
  HandScheduler concurrent_scheduler(
      HandSchedulerConfig{},
      [&](const double*, std::uint64_t, double*) {
        entered.set_value();
        released.wait();
      }, [](const double*, std::uint64_t, double*) {});
  assert(concurrent_scheduler.submit_session(session(1, 100, HandPhase::teleop)));
  assert(concurrent_scheduler.submit_input(input(1, 100, 1)));
  auto solving = std::async(std::launch::async, [&] { return concurrent_scheduler.tick(100); });
  entered.get_future().wait();
  auto admission = std::async(std::launch::async, [&] {
    return concurrent_scheduler.submit_session(session(2, 200, HandPhase::returning)) &&
           concurrent_scheduler.submit_input(input(2, 200, 2));
  });
  const bool responsive = admission.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready;
  release.set_value();
  solving.get();
  assert(admission.get());
  assert(responsive);
  assert(!concurrent_scheduler.pop(command));
  assert(concurrent_scheduler.tick(200));
  assert(concurrent_scheduler.pop(command));
  assert(command.input_sequence == 2);
  assert(command.phase == HandPhase::returning);
  return 0;
}
