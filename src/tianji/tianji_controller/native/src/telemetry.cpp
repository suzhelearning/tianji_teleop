#include "tianji_qp_ik/telemetry.hpp"

#include <algorithm>
#include <cmath>

namespace tianji_qp_ik {

CycleTimeWindow::CycleTimeWindow(std::size_t capacity)
    : samples_(capacity, 0.0), scratch_(capacity, 0.0) {
  if (capacity == 0U) {
    throw std::invalid_argument("cycle-time window capacity must be positive");
  }
}

void CycleTimeWindow::add(double duration_us) noexcept {
  samples_[cursor_] = duration_us;
  cursor_ = (cursor_ + 1U) % samples_.size();
  size_ = std::min(size_ + 1U, samples_.size());
}

double CycleTimeWindow::percentile99() {
  if (size_ == 0U) {
    return 0.0;
  }
  std::copy_n(samples_.begin(), static_cast<std::ptrdiff_t>(size_), scratch_.begin());
  auto scratch_end = scratch_.begin() + static_cast<std::ptrdiff_t>(size_);
  std::sort(scratch_.begin(), scratch_end);
  const double position = 0.99 * static_cast<double>(size_ - 1U);
  const std::size_t lower = static_cast<std::size_t>(std::floor(position));
  const std::size_t upper = static_cast<std::size_t>(std::ceil(position));
  const double fraction = position - static_cast<double>(lower);
  return scratch_[lower] + fraction * (scratch_[upper] - scratch_[lower]);
}

}  // namespace tianji_qp_ik
