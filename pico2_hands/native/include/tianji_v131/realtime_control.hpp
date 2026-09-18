// Ported from TJ_arm_control_pico_ee_ik; see PORTING.md.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace tianji_v131 {

struct RealtimeControlConfig {
  bool enabled{false};
  bool required_for_qualification{false};
  int fifo_priority{60};
  int control_cpu{-1};
  bool lock_memory{true};
  std::size_t prefault_stack_kb{512U};
  std::size_t warmup_cycles{400U};
};

RealtimeControlConfig realtimeConfigForExecution(
    const RealtimeControlConfig& configured,
    bool process_has_late_allocations) noexcept;

struct RealtimeProcessLayout {
  bool valid{false};
  int control_cpu{-1};
  std::vector<int> control_sibling_cpus;
  std::vector<int> housekeeping_cpus;
  std::string detail;
};

struct RealtimeProcessPreparation {
  bool requested{false};
  bool housekeeping_affinity_applied{false};
  int affinity_error{0};
  RealtimeProcessLayout layout;
  std::string detail;
};

struct RealtimeThreadStatus {
  bool requested{false};
  bool control_affinity_applied{false};
  bool fifo_applied{false};
  bool memory_lock_requested{false};
  bool memory_locked{false};
  bool stack_prefaulted{false};
  int control_cpu{-1};
  int fifo_priority{0};
  int affinity_error{0};
  int scheduler_error{0};
  int memory_lock_error{0};
  std::size_t locked_memory_kb{0U};
  std::string detail;

  bool qualificationReady() const noexcept;
};

RealtimeProcessLayout selectRealtimeProcessLayout(
    const std::vector<int>& allowed_cpus,
    const std::vector<std::vector<int>>& sibling_groups,
    int requested_cpu);

RealtimeProcessPreparation prepareRealtimeProcess(
    const RealtimeControlConfig& config);

RealtimeThreadStatus configureCurrentThreadRealtime(
    const RealtimeControlConfig& config,
    const RealtimeProcessPreparation& preparation);

struct CycleTimingObservation {
  std::int64_t wake_lateness_ns{0};
  std::int64_t compute_time_ns{0};
  std::int64_t finish_overrun_ns{0};
  std::uint64_t skipped_periods{0U};
};

struct CycleTimingTotals {
  std::uint64_t deadline_misses{0U};
  std::uint64_t finish_overrun_count{0U};
  std::uint64_t skipped_periods{0U};
  std::int64_t maximum_compute_time_ns{0};
  std::int64_t maximum_wake_lateness_ns{0};
  std::int64_t maximum_finish_overrun_ns{0};
};

class PeriodicDeadlineTracker final {
 public:
  PeriodicDeadlineTracker(std::int64_t period_ns,
                          std::int64_t initial_release_ns);

  CycleTimingObservation finishCycle(std::int64_t cycle_start_ns,
                                     std::int64_t cycle_finish_ns);
  void reset(std::int64_t release_ns) noexcept;

  std::int64_t periodNs() const noexcept { return period_ns_; }
  std::int64_t nextReleaseNs() const noexcept { return release_ns_; }
  const CycleTimingTotals& totals() const noexcept { return totals_; }

 private:
  std::int64_t period_ns_{0};
  std::int64_t release_ns_{0};
  CycleTimingTotals totals_;
};

}  // namespace tianji_v131
