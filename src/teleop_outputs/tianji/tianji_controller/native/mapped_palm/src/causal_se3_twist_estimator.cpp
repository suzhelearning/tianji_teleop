#include "tianji_mapped_palm/causal_se3_twist_estimator.hpp"

#include "tianji_mapped_palm/so3.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/QR>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace tianji_mapped_palm {
namespace {

constexpr double kNsToSeconds = 1.0e-9;

bool finiteInput(const CausalSe3TwistInput& input) noexcept {
  return input.pose.position.allFinite() &&
         isProperRotation(input.pose.rotation) && input.sequence != 0U &&
         input.tracking_epoch != 0U && input.source_timestamp_ns > 0 &&
         std::isfinite(input.observation_age_s) &&
         input.observation_age_s >= 0.0 &&
         std::isfinite(input.control_dt_s) && input.control_dt_s > 0.0;
}

double decreasingFade(double value, double full, double zero) noexcept {
  if (value <= full) {
    return 1.0;
  }
  if (value >= zero) {
    return 0.0;
  }
  return (zero - value) / (zero - full);
}

template <int Rows, int Columns>
double conditionNumber(
    const Eigen::Matrix<double, Rows, Columns>& matrix) noexcept {
  const Eigen::Matrix<double, Columns, Columns> gram =
      matrix.transpose() * matrix;
  const Eigen::SelfAdjointEigenSolver<
      Eigen::Matrix<double, Columns, Columns>> solver(gram,
                                                       Eigen::EigenvaluesOnly);
  if (solver.info() != Eigen::Success) {
    return std::numeric_limits<double>::infinity();
  }
  const double minimum = solver.eigenvalues().minCoeff();
  const double maximum = solver.eigenvalues().maxCoeff();
  if (!std::isfinite(minimum) || !std::isfinite(maximum) ||
      minimum <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return std::sqrt(maximum / minimum);
}

}  // namespace

std::string_view toString(CausalSe3TwistFailure failure) noexcept {
  switch (failure) {
    case CausalSe3TwistFailure::kNone:
      return "none";
    case CausalSe3TwistFailure::kInsufficientHistory:
      return "insufficient_history";
    case CausalSe3TwistFailure::kNonFiniteInput:
      return "non_finite_input";
    case CausalSe3TwistFailure::kNonMonotonicSequence:
      return "non_monotonic_sequence";
    case CausalSe3TwistFailure::kNonMonotonicTimestamp:
      return "non_monotonic_timestamp";
    case CausalSe3TwistFailure::kEpochReset:
      return "epoch_reset";
    case CausalSe3TwistFailure::kStreamDiscontinuity:
      return "stream_discontinuity";
    case CausalSe3TwistFailure::kSourceDtOutlier:
      return "source_dt_outlier";
    case CausalSe3TwistFailure::kFitIllConditioned:
      return "fit_ill_conditioned";
    case CausalSe3TwistFailure::kPositionFitResidual:
      return "position_fit_residual";
    case CausalSe3TwistFailure::kOrientationFitResidual:
      return "orientation_fit_residual";
    case CausalSe3TwistFailure::kObservationAge:
      return "observation_age";
  }
  return "unknown";
}

CausalSe3TwistEstimator::CausalSe3TwistEstimator(
    CausalSe3TwistEstimatorConfig config)
    : config_(config) {
  if (config_.fit_window != 3 && config_.fit_window != 5 &&
      config_.fit_window != 7) {
    throw std::invalid_argument("causal SE(3) fit window must be 3, 5, or 7");
  }
}

void CausalSe3TwistEstimator::reset() noexcept {
  sample_count_ = 0;
  filtered_confidence_ = 0.0;
}

CausalSe3TwistResult CausalSe3TwistEstimator::fail(
    CausalSe3TwistFailure failure) noexcept {
  filtered_confidence_ = 0.0;
  CausalSe3TwistResult result;
  result.failure = failure;
  return result;
}

void CausalSe3TwistEstimator::append(
    const CausalSe3TwistInput& input) noexcept {
  if (sample_count_ == config_.fit_window) {
    for (int index = 1; index < sample_count_; ++index) {
      samples_[static_cast<std::size_t>(index - 1)] =
          samples_[static_cast<std::size_t>(index)];
    }
    --sample_count_;
  }
  Sample& sample = samples_[static_cast<std::size_t>(sample_count_)];
  sample.pose = input.pose;
  sample.sequence = input.sequence;
  sample.tracking_epoch = input.tracking_epoch;
  sample.source_timestamp_ns = input.source_timestamp_ns;
  ++sample_count_;
}

bool CausalSe3TwistEstimator::sourceDtIsOutlier(
    std::int64_t timestamp_ns) const noexcept {
  if (sample_count_ < 3) {
    return false;
  }
  std::array<std::int64_t, 6> intervals{};
  const int interval_count = std::clamp(sample_count_ - 1, 0, 6);
  for (int index = 0; index < interval_count; ++index) {
    intervals[static_cast<std::size_t>(index)] =
        samples_[static_cast<std::size_t>(index + 1)].source_timestamp_ns -
        samples_[static_cast<std::size_t>(index)].source_timestamp_ns;
  }
  for (int index = 1; index < 6; ++index) {
    if (index >= interval_count) {
      break;
    }
    const std::int64_t value = intervals[static_cast<std::size_t>(index)];
    int insertion = index;
    while (insertion > 0 &&
           intervals[static_cast<std::size_t>(insertion - 1)] > value) {
      intervals[static_cast<std::size_t>(insertion)] =
          intervals[static_cast<std::size_t>(insertion - 1)];
      --insertion;
    }
    intervals[static_cast<std::size_t>(insertion)] = value;
  }
  const std::int64_t median =
      intervals[static_cast<std::size_t>(interval_count / 2)];
  const std::int64_t newest_dt =
      timestamp_ns -
      samples_[static_cast<std::size_t>(sample_count_ - 1)].source_timestamp_ns;
  return median <= 0 || newest_dt * 2 < median || newest_dt > median * 2;
}

CausalSe3TwistResult CausalSe3TwistEstimator::update(
    const CausalSe3TwistInput& input) noexcept {
  if (!finiteInput(input)) {
    reset();
    return fail(CausalSe3TwistFailure::kNonFiniteInput);
  }
  if (sample_count_ > 0) {
    const Sample& latest =
        samples_[static_cast<std::size_t>(sample_count_ - 1)];
    if (input.tracking_epoch != latest.tracking_epoch) {
      reset();
      append(input);
      return fail(CausalSe3TwistFailure::kEpochReset);
    }
    if (input.stream_discontinuity) {
      reset();
      append(input);
      return fail(CausalSe3TwistFailure::kStreamDiscontinuity);
    }
    if (input.sequence <= latest.sequence) {
      reset();
      return fail(CausalSe3TwistFailure::kNonMonotonicSequence);
    }
    if (input.source_timestamp_ns <= latest.source_timestamp_ns) {
      reset();
      return fail(CausalSe3TwistFailure::kNonMonotonicTimestamp);
    }
    if (sourceDtIsOutlier(input.source_timestamp_ns)) {
      reset();
      append(input);
      return fail(CausalSe3TwistFailure::kSourceDtOutlier);
    }
  } else if (input.stream_discontinuity) {
    append(input);
    return fail(CausalSe3TwistFailure::kStreamDiscontinuity);
  }

  append(input);
  if (sample_count_ < config_.fit_window) {
    return fail(CausalSe3TwistFailure::kInsufficientHistory);
  }
  if (input.observation_age_s > config_.maximum_age_seconds) {
    return fail(CausalSe3TwistFailure::kObservationAge);
  }

  switch (config_.fit_window) {
    case 3:
      return fit<3>(input);
    case 5:
      return fit<5>(input);
    case 7:
      return fit<7>(input);
    default:
      return fail(CausalSe3TwistFailure::kFitIllConditioned);
  }
}

template <int Window>
CausalSe3TwistResult CausalSe3TwistEstimator::fit(
    const CausalSe3TwistInput& input) noexcept {
  try {
    using TranslationDesign = Eigen::Matrix<double, Window, 3>;
    using OrientationDesign = Eigen::Matrix<double, Window, 2>;
    using Observations = Eigen::Matrix<double, Window, 3>;
    const Sample& latest = samples_[static_cast<std::size_t>(Window - 1)];
    const double span_s =
        static_cast<double>(latest.source_timestamp_ns -
                            samples_[0].source_timestamp_ns) *
        kNsToSeconds;
    if (!std::isfinite(span_s) || span_s <= 0.0) {
      reset();
      return fail(CausalSe3TwistFailure::kFitIllConditioned);
    }

    TranslationDesign translation_design;
    OrientationDesign orientation_design;
    Observations positions;
    Observations rotations;
    for (int index = 0; index < Window; ++index) {
      const Sample& sample = samples_[static_cast<std::size_t>(index)];
      const double tau_s =
          static_cast<double>(sample.source_timestamp_ns -
                              latest.source_timestamp_ns) *
          kNsToSeconds;
      const double tau = tau_s / span_s;
      translation_design(index, 0) = 1.0;
      translation_design(index, 1) = tau;
      translation_design(index, 2) = 0.5 * tau * tau;
      orientation_design(index, 0) = tau;
      orientation_design(index, 1) = 0.5 * tau * tau;
      positions.row(index) = sample.pose.position.transpose();
      rotations.row(index) =
          so3Log(sample.pose.rotation * latest.pose.rotation.transpose())
              .transpose();
    }

    const double translation_condition = conditionNumber(translation_design);
    const double orientation_condition = conditionNumber(orientation_design);
    const double condition =
        std::max(translation_condition, orientation_condition);
    if (!std::isfinite(condition) ||
        condition > config_.maximum_condition_number) {
      reset();
      return fail(CausalSe3TwistFailure::kFitIllConditioned);
    }

    const Eigen::Matrix<double, 3, 3> translation_coefficients =
        translation_design.colPivHouseholderQr().solve(positions);
    const Eigen::Matrix<double, 2, 3> orientation_coefficients =
        orientation_design.colPivHouseholderQr().solve(rotations);
    const Observations position_error =
        positions - translation_design * translation_coefficients;
    const Observations orientation_error =
        rotations - orientation_design * orientation_coefficients;
    const double position_rms =
        std::sqrt(position_error.squaredNorm() / static_cast<double>(Window));
    const double orientation_rms = std::sqrt(
        orientation_error.squaredNorm() / static_cast<double>(Window));

    if (!std::isfinite(position_rms) ||
        position_rms >= config_.position_invalid_residual_m) {
      reset();
      return fail(CausalSe3TwistFailure::kPositionFitResidual);
    }
    if (!std::isfinite(orientation_rms) ||
        orientation_rms >= config_.orientation_invalid_residual_rad) {
      reset();
      return fail(CausalSe3TwistFailure::kOrientationFitResidual);
    }

    CausalSe3TwistResult result;
    result.valid = true;
    result.fit_window = Window;
    result.condition_number = condition;
    result.position_fit_rms_m = position_rms;
    result.orientation_fit_rms_rad = orientation_rms;
    result.observation_age_s = input.observation_age_s;
    result.source_twist.head<3>() =
        translation_coefficients.row(1).transpose() / span_s;
    result.source_acceleration.head<3>() =
        translation_coefficients.row(2).transpose() / (span_s * span_s);
    result.source_twist.tail<3>() =
        orientation_coefficients.row(0).transpose() / span_s;
    result.source_acceleration.tail<3>() =
        orientation_coefficients.row(1).transpose() / (span_s * span_s);
    const double alignment_s =
        input.observation_age_s + config_.extra_lead_seconds;
    result.aligned_twist =
        result.source_twist + alignment_s * result.source_acceleration;
    result.age_compensation = result.aligned_twist - result.source_twist;
    if (!result.source_twist.allFinite() ||
        !result.source_acceleration.allFinite() ||
        !result.aligned_twist.allFinite()) {
      reset();
      return fail(CausalSe3TwistFailure::kFitIllConditioned);
    }

    const double raw_confidence = std::min(
        {decreasingFade(std::log10(condition), 4.0,
                        std::log10(config_.maximum_condition_number)),
         decreasingFade(position_rms,
                        config_.position_full_confidence_residual_m,
                        config_.position_invalid_residual_m),
         decreasingFade(orientation_rms,
                        config_.orientation_full_confidence_residual_rad,
                        config_.orientation_invalid_residual_rad),
         decreasingFade(input.observation_age_s,
                        config_.age_full_confidence_seconds,
                        config_.maximum_age_seconds)});
    const double time_constant =
        raw_confidence >= filtered_confidence_
            ? config_.confidence_attack_seconds
            : config_.confidence_release_seconds;
    const double alpha = 1.0 - std::exp(-input.control_dt_s / time_constant);
    filtered_confidence_ += alpha * (raw_confidence - filtered_confidence_);
    filtered_confidence_ = std::clamp(filtered_confidence_, 0.0, 1.0);
    result.confidence = filtered_confidence_;
    result.failure = CausalSe3TwistFailure::kNone;
    return result;
  } catch (...) {
    reset();
    return fail(CausalSe3TwistFailure::kFitIllConditioned);
  }
}

template CausalSe3TwistResult CausalSe3TwistEstimator::fit<3>(
    const CausalSe3TwistInput&) noexcept;
template CausalSe3TwistResult CausalSe3TwistEstimator::fit<5>(
    const CausalSe3TwistInput&) noexcept;
template CausalSe3TwistResult CausalSe3TwistEstimator::fit<7>(
    const CausalSe3TwistInput&) noexcept;

}  // namespace tianji_mapped_palm
