#include "pico_odin/extrinsic_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <sstream>

#include <Eigen/SVD>
#include <Eigen/Cholesky>

namespace pico_odin {
namespace {

struct MotionPair {
  Pose3 odin_relative;
  Pose3 pelvis_relative;
};

struct OffsetScore {
  double mean_squared_error{std::numeric_limits<double>::infinity()};
  double correlation{-1.0};
  std::size_t pair_count{0};
};

double unwrap_near(double angle, double reference) {
  while (angle - reference > M_PI) angle -= 2.0 * M_PI;
  while (angle - reference < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

std::pair<double, double> pitch_yaw_ranges(const std::vector<HostTimedPose> & poses) {
  double min_pitch = std::numeric_limits<double>::infinity();
  double max_pitch = -std::numeric_limits<double>::infinity();
  double min_yaw = std::numeric_limits<double>::infinity();
  double max_yaw = -std::numeric_limits<double>::infinity();
  double previous_yaw = 0.0;
  bool first = true;
  for (const auto & sample : poses) {
    const Eigen::Vector3d forward = sample.pose.rotation * Eigen::Vector3d::UnitX();
    const double pitch = std::atan2(-forward.z(), std::hypot(forward.x(), forward.y()));
    double yaw = std::atan2(forward.y(), forward.x());
    if (!first) yaw = unwrap_near(yaw, previous_yaw);
    first = false;
    previous_yaw = yaw;
    min_pitch = std::min(min_pitch, pitch);
    max_pitch = std::max(max_pitch, pitch);
    min_yaw = std::min(min_yaw, yaw);
    max_yaw = std::max(max_yaw, yaw);
  }
  return {max_pitch - min_pitch, max_yaw - min_yaw};
}

std::optional<Pose3> interpolate_at(
  const std::vector<HostTimedPose> & samples,
  double stamp,
  double max_gap)
{
  if (samples.size() < 2 || stamp < samples.front().receipt_sec ||
      stamp > samples.back().receipt_sec) {
    return std::nullopt;
  }
  const auto upper = std::lower_bound(
    samples.begin(), samples.end(), stamp,
    [](const HostTimedPose & sample, double target) { return sample.receipt_sec < target; });
  if (upper == samples.begin()) return upper->pose;
  if (upper == samples.end()) return samples.back().pose;
  const auto lower = std::prev(upper);
  const double duration = upper->receipt_sec - lower->receipt_sec;
  if (duration <= 0.0 || duration > max_gap) return std::nullopt;
  return interpolate(lower->pose, upper->pose, (stamp - lower->receipt_sec) / duration);
}

OffsetScore offset_score(
  const std::vector<HostTimedPose> & pico,
  const std::vector<HostTimedPose> & odin,
  double offset,
  const SolverOptions & options)
{
  double squared_error = 0.0;
  double sum_pico = 0.0;
  double sum_odin = 0.0;
  double sum_pico_squared = 0.0;
  double sum_odin_squared = 0.0;
  double sum_cross = 0.0;
  std::size_t active_count = 0;
  for (std::size_t index = 2; index < pico.size(); index += 2) {
    const auto odin_previous = interpolate_at(
      odin, pico[index - 2].receipt_sec + offset, options.max_interpolation_gap_sec);
    const auto odin_current = interpolate_at(
      odin, pico[index].receipt_sec + offset, options.max_interpolation_gap_sec);
    if (!odin_previous || !odin_current) continue;
    const double pico_angle = angular_distance(
      pico[index - 2].pose.rotation, pico[index].pose.rotation);
    const double odin_angle = angular_distance(
      odin_previous->rotation, odin_current->rotation);
    if (std::max(pico_angle, odin_angle) < 1e-4) continue;
    const double difference = pico_angle - odin_angle;
    squared_error += difference * difference;
    sum_pico += pico_angle;
    sum_odin += odin_angle;
    sum_pico_squared += pico_angle * pico_angle;
    sum_odin_squared += odin_angle * odin_angle;
    sum_cross += pico_angle * odin_angle;
    ++active_count;
  }
  OffsetScore result;
  result.pair_count = active_count;
  if (active_count < options.min_receive_lag_pairs) return result;
  const double count = static_cast<double>(active_count);
  const double covariance = sum_cross - sum_pico * sum_odin / count;
  const double pico_variance = sum_pico_squared - sum_pico * sum_pico / count;
  const double odin_variance = sum_odin_squared - sum_odin * sum_odin / count;
  const double denominator = std::sqrt(std::max(0.0, pico_variance * odin_variance));
  if (denominator <= 1e-12) return result;
  result.mean_squared_error = squared_error / count;
  result.correlation = covariance / denominator;
  return result;
}

ReceiveLagEstimate estimate_receive_lag_impl(
  const std::vector<HostTimedPose> & pico,
  const std::vector<HostTimedPose> & odin,
  const SolverOptions & options)
{
  ReceiveLagEstimate best;
  double best_score = std::numeric_limits<double>::infinity();
  const int steps = static_cast<int>(
    std::ceil(2.0 * options.max_receive_lag_sec / options.receive_lag_step_sec));
  for (int step = 0; step <= steps; ++step) {
    const double offset =
      -options.max_receive_lag_sec + step * options.receive_lag_step_sec;
    const auto score = offset_score(pico, odin, offset, options);
    if (score.correlation >= options.min_receive_lag_correlation &&
        score.mean_squared_error < best_score) {
      best.valid = true;
      best_score = score.mean_squared_error;
      best.lag_sec = offset;
      best.correlation = score.correlation;
      best.pair_count = score.pair_count;
    }
  }
  best.at_boundary = best.valid &&
    std::abs(best.lag_sec) >=
      options.max_receive_lag_sec - 1.5 * options.receive_lag_step_sec;
  return best;
}

std::vector<std::pair<Pose3, Pose3>> synchronized_poses(
  const std::vector<HostTimedPose> & pico,
  const std::vector<HostTimedPose> & odin,
  double offset,
  double max_gap)
{
  std::vector<std::pair<Pose3, Pose3>> synchronized;
  synchronized.reserve(pico.size());
  for (const auto & sample : pico) {
    const auto odin_pose = interpolate_at(odin, sample.receipt_sec + offset, max_gap);
    if (odin_pose) synchronized.emplace_back(sample.pose, *odin_pose);
  }
  return synchronized;
}

std::vector<MotionPair> relative_motions(
  const std::vector<std::pair<Pose3, Pose3>> & synchronized,
  std::size_t stride)
{
  std::vector<MotionPair> motions;
  if (stride == 0 || synchronized.size() <= stride) return motions;
  motions.reserve(synchronized.size() - stride);
  for (std::size_t index = stride; index < synchronized.size(); ++index) {
    const auto & [pico_before, odin_before] = synchronized[index - stride];
    const auto & [pico_after, odin_after] = synchronized[index];
    const Pose3 a = compose(inverse(odin_before), odin_after);
    const Pose3 b = compose(inverse(pico_before), pico_after);
    if (angular_distance(a.rotation, Eigen::Quaterniond::Identity()) < 1e-4 &&
        a.translation.norm() < 1e-4) {
      continue;
    }
    motions.push_back({a, b});
  }
  return motions;
}

Eigen::Matrix4d left_quaternion_matrix(Eigen::Quaterniond quaternion) {
  quaternion = normalized(quaternion);
  if (quaternion.w() < 0.0) quaternion.coeffs() *= -1.0;
  Eigen::Matrix4d matrix;
  matrix <<
    quaternion.w(), -quaternion.x(), -quaternion.y(), -quaternion.z(),
    quaternion.x(), quaternion.w(), -quaternion.z(), quaternion.y(),
    quaternion.y(), quaternion.z(), quaternion.w(), -quaternion.x(),
    quaternion.z(), -quaternion.y(), quaternion.x(), quaternion.w();
  return matrix;
}

Eigen::Matrix4d right_quaternion_matrix(Eigen::Quaterniond quaternion) {
  quaternion = normalized(quaternion);
  if (quaternion.w() < 0.0) quaternion.coeffs() *= -1.0;
  Eigen::Matrix4d matrix;
  matrix <<
    quaternion.w(), -quaternion.x(), -quaternion.y(), -quaternion.z(),
    quaternion.x(), quaternion.w(), quaternion.z(), -quaternion.y(),
    quaternion.y(), -quaternion.z(), quaternion.w(), quaternion.x(),
    quaternion.z(), quaternion.y(), -quaternion.x(), quaternion.w();
  return matrix;
}

std::pair<Eigen::Quaterniond, double> solve_rotation(
  const std::vector<MotionPair> & motions)
{
  Eigen::MatrixXd system(4 * motions.size(), 4);
  for (std::size_t index = 0; index < motions.size(); ++index) {
    system.block<4, 4>(4 * index, 0) =
      left_quaternion_matrix(motions[index].odin_relative.rotation) -
      right_quaternion_matrix(motions[index].pelvis_relative.rotation);
  }
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(system, Eigen::ComputeFullV);
  const Eigen::Vector4d solution = svd.matrixV().col(3);
  const auto singular = svd.singularValues();
  // The smallest singular value is the expected quaternion null space.  A
  // unique hand-eye rotation requires the next singular direction to be
  // observable; otherwise single-axis motion can produce an arbitrary result.
  const double condition = singular.size() == 4 && singular(2) > 1e-12 ?
    singular(0) / singular(2) : std::numeric_limits<double>::infinity();
  return {
    normalized(Eigen::Quaterniond(
      solution(0), solution(1), solution(2), solution(3))),
    condition};
}

std::pair<Eigen::Vector3d, double> solve_translation(
  const std::vector<MotionPair> & motions,
  const Eigen::Quaterniond & rotation,
  double y_prior_sigma_m)
{
  const bool use_y_prior = std::isfinite(y_prior_sigma_m) && y_prior_sigma_m > 0.0;
  const Eigen::Index data_rows = static_cast<Eigen::Index>(3 * motions.size());
  const Eigen::Index row_count = data_rows + (use_y_prior ? 1 : 0);
  Eigen::MatrixXd system(row_count, 3);
  Eigen::VectorXd target(row_count);
  for (std::size_t index = 0; index < motions.size(); ++index) {
    system.block<3, 3>(3 * index, 0) =
      motions[index].odin_relative.rotation.toRotationMatrix() - Eigen::Matrix3d::Identity();
    target.segment<3>(3 * index) =
      rotation * motions[index].pelvis_relative.translation -
      motions[index].odin_relative.translation;
  }
  if (use_y_prior) {
    // X is T_odin_pelvis.  The stored mounting transform is inverse(X), whose
    // lateral component is -(R_X * UnitY)^T * t_X.  Penalize that component
    // without forcing it to exactly zero.
    const double weight = 1.0 / y_prior_sigma_m;
    system.row(data_rows) = weight * (rotation * Eigen::Vector3d::UnitY()).transpose();
    target(data_rows) = 0.0;
  }
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(
    system, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const auto singular = svd.singularValues();
  const double condition = singular.size() == 3 && singular(2) > 1e-12 ?
    singular(0) / singular(2) : std::numeric_limits<double>::infinity();
  return {svd.solve(target), condition};
}

double median(std::vector<double> values) {
  if (values.empty()) return 0.0;
  const auto middle = values.begin() + values.size() / 2;
  std::nth_element(values.begin(), middle, values.end());
  double result = *middle;
  if (values.size() % 2 == 0) {
    result = 0.5 * (result + *std::max_element(values.begin(), middle));
  }
  return result;
}

std::vector<MotionPair> reject_outliers(
  const std::vector<MotionPair> & motions,
  const Pose3 & odin_T_pelvis)
{
  std::vector<double> rotation_errors;
  std::vector<double> translation_errors;
  rotation_errors.reserve(motions.size());
  translation_errors.reserve(motions.size());
  for (const auto & motion : motions) {
    const Pose3 left = compose(motion.odin_relative, odin_T_pelvis);
    const Pose3 right = compose(odin_T_pelvis, motion.pelvis_relative);
    const Pose3 error = compose(inverse(left), right);
    rotation_errors.push_back(
      angular_distance(error.rotation, Eigen::Quaterniond::Identity()));
    translation_errors.push_back(error.translation.norm());
  }

  const double rotation_limit = std::max(4.0 * median(rotation_errors), 0.03);
  const double translation_limit = std::max(4.0 * median(translation_errors), 0.02);
  std::vector<MotionPair> accepted;
  accepted.reserve(motions.size());
  for (std::size_t index = 0; index < motions.size(); ++index) {
    if (rotation_errors[index] <= rotation_limit &&
        translation_errors[index] <= translation_limit) {
      accepted.push_back(motions[index]);
    }
  }
  return accepted;
}

Eigen::Vector3d rotation_vector(Eigen::Quaterniond rotation) {
  rotation = normalized(rotation);
  if (rotation.w() < 0.0) rotation.coeffs() *= -1.0;
  const double half_angle = std::acos(std::clamp(rotation.w(), -1.0, 1.0));
  const double sine = std::sin(half_angle);
  if (std::abs(sine) < 1e-10) return 2.0 * rotation.vec();
  return (2.0 * half_angle / sine) * rotation.vec();
}

Eigen::Matrix<double, 6, 1> hand_eye_residual(
  const MotionPair & motion,
  const Pose3 & odin_T_pelvis)
{
  const Pose3 left = compose(motion.odin_relative, odin_T_pelvis);
  const Pose3 right = compose(odin_T_pelvis, motion.pelvis_relative);
  const Pose3 error = compose(inverse(left), right);
  Eigen::Matrix<double, 6, 1> residual;
  residual.head<3>() = error.translation;
  residual.tail<3>() = rotation_vector(error.rotation);
  return residual;
}

Pose3 apply_increment(
  const Pose3 & pose,
  const Eigen::Matrix<double, 6, 1> & increment)
{
  const Eigen::Vector3d rotation_increment = increment.tail<3>();
  const double angle = rotation_increment.norm();
  const Eigen::Quaterniond rotation = angle < 1e-12 ?
    Eigen::Quaterniond::Identity() :
    Eigen::Quaterniond(Eigen::AngleAxisd(angle, rotation_increment / angle));
  return compose(Pose3{rotation, increment.head<3>()}, pose);
}

std::pair<Pose3, int> refine_hand_eye(
  const std::vector<MotionPair> & motions,
  Pose3 estimate,
  const SolverOptions & options)
{
  constexpr double epsilon = 1e-6;
  int completed_iterations = 0;
  for (int iteration = 0; iteration < options.max_refinement_iterations; ++iteration) {
    Eigen::Matrix<double, 6, 6> hessian =
      1e-8 * Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::Matrix<double, 6, 1> gradient =
      Eigen::Matrix<double, 6, 1>::Zero();

    for (const auto & motion : motions) {
      const auto residual = hand_eye_residual(motion, estimate);
      Eigen::Matrix<double, 6, 6> jacobian;
      for (int column = 0; column < 6; ++column) {
        Eigen::Matrix<double, 6, 1> delta =
          Eigen::Matrix<double, 6, 1>::Zero();
        delta(column) = epsilon;
        jacobian.col(column) =
          (hand_eye_residual(motion, apply_increment(estimate, delta)) - residual) /
          epsilon;
      }
      const double normalized_error = std::hypot(
        residual.head<3>().norm() / 0.04,
        residual.tail<3>().norm() / 0.05);
      const double weight = normalized_error <= options.refinement_huber_delta ? 1.0 :
        options.refinement_huber_delta / normalized_error;
      hessian.noalias() += weight * jacobian.transpose() * jacobian;
      gradient.noalias() += weight * jacobian.transpose() * residual;
    }

    const auto prior_residual = [&estimate, &options]() {
      Eigen::Matrix<double, 4, 1> residual;
      const Pose3 pelvis_T_odin = inverse(estimate);
      residual(0) = pelvis_T_odin.translation.y() / options.y_prior_sigma_m;
      const Eigen::Quaterniond rear_facing(
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
      residual.tail<3>() = rotation_vector(
        rear_facing.conjugate() * pelvis_T_odin.rotation) /
        options.rotation_prior_sigma_rad;
      return residual;
    }();
    Eigen::Matrix<double, 4, 6> prior_jacobian;
    for (int column = 0; column < 6; ++column) {
      Eigen::Matrix<double, 6, 1> delta =
        Eigen::Matrix<double, 6, 1>::Zero();
      delta(column) = epsilon;
      const Pose3 perturbed = apply_increment(estimate, delta);
      Eigen::Matrix<double, 4, 1> perturbed_residual;
      const Pose3 pelvis_T_odin = inverse(perturbed);
      perturbed_residual(0) = pelvis_T_odin.translation.y() / options.y_prior_sigma_m;
      const Eigen::Quaterniond rear_facing(
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
      perturbed_residual.tail<3>() = rotation_vector(
        rear_facing.conjugate() * pelvis_T_odin.rotation) /
        options.rotation_prior_sigma_rad;
      prior_jacobian.col(column) =
        (perturbed_residual - prior_residual) / epsilon;
    }
    hessian.noalias() += prior_jacobian.transpose() * prior_jacobian;
    gradient.noalias() += prior_jacobian.transpose() * prior_residual;

    const Eigen::Matrix<double, 6, 1> increment = -hessian.ldlt().solve(gradient);
    if (!increment.allFinite()) break;
    estimate = apply_increment(estimate, increment);
    ++completed_iterations;
    if (increment.norm() < 1e-8) break;
  }
  return {estimate, completed_iterations};
}

CalibrationMetrics calculate_metrics(
  const std::vector<MotionPair> & motions,
  const Pose3 & odin_T_pelvis,
  std::size_t sample_count,
  double pitch_range,
  double yaw_range,
  double condition)
{
  double rotation_squared = 0.0;
  double translation_squared = 0.0;
  for (const auto & motion : motions) {
    const Pose3 left = compose(motion.odin_relative, odin_T_pelvis);
    const Pose3 right = compose(odin_T_pelvis, motion.pelvis_relative);
    const Pose3 error = compose(inverse(left), right);
    const double rotation_error = angular_distance(error.rotation, Eigen::Quaterniond::Identity());
    rotation_squared += rotation_error * rotation_error;
    translation_squared += error.translation.squaredNorm();
  }
  CalibrationMetrics metrics;
  metrics.sample_count = static_cast<int>(sample_count);
  metrics.pitch_range_rad = pitch_range;
  metrics.yaw_range_rad = yaw_range;
  metrics.rotation_rms_rad = std::sqrt(rotation_squared / motions.size());
  metrics.translation_rms_m = std::sqrt(translation_squared / motions.size());
  metrics.condition_number = condition;
  return metrics;
}

bool final_neutral_is_consistent(
  const std::vector<std::pair<Pose3, Pose3>> & synchronized,
  const Pose3 & pelvis_T_odin,
  const SolverOptions & options)
{
  if (synchronized.empty()) return false;
  std::vector<Pose3> world_alignments;
  world_alignments.reserve(synchronized.size());
  for (const auto & [pico_pelvis, odin_sensor] : synchronized) {
    const Pose3 odin_pelvis = compose(odin_sensor, inverse(pelvis_T_odin));
    world_alignments.push_back(compose(pico_pelvis, inverse(odin_pelvis)));
  }
  const Pose3 alignment = mean_pose(world_alignments);
  const std::size_t tail_count = std::min<std::size_t>(10, synchronized.size());
  for (std::size_t index = synchronized.size() - tail_count;
       index < synchronized.size(); ++index) {
    const auto & [pico_pelvis, odin_sensor] = synchronized[index];
    const Pose3 predicted = compose(
      alignment, compose(odin_sensor, inverse(pelvis_T_odin)));
    if (angular_distance(predicted.rotation, pico_pelvis.rotation) >
        options.max_final_neutral_rotation_error_rad) {
      return false;
    }
    const Eigen::Vector3d up = pico_pelvis.rotation * Eigen::Vector3d::UnitZ();
    const double tilt = std::acos(std::clamp(up.z(), -1.0, 1.0));
    if (tilt > options.max_final_neutral_tilt_rad ||
        predicted.rotation.toRotationMatrix().determinant() <= 0.0) {
      return false;
    }
  }
  return true;
}

SolverResult failure(const std::string & message, double pitch = 0.0, double yaw = 0.0) {
  SolverResult result;
  result.message = message;
  result.metrics.pitch_range_rad = pitch;
  result.metrics.yaw_range_rad = yaw;
  return result;
}

}  // namespace

SolverResult solve_extrinsics(
  const std::vector<HostTimedPose> & pico,
  const std::vector<HostTimedPose> & odin,
  const SolverOptions & options)
{
  const double lag_steps =
    2.0 * options.max_receive_lag_sec / options.receive_lag_step_sec;
  if (!(options.min_pitch_range_rad > 0.0) ||
      !(options.min_yaw_range_rad > 0.0) ||
      options.min_pairs < 12 || !std::isfinite(options.max_receive_lag_sec) ||
      !(options.max_receive_lag_sec > 0.0) ||
      !std::isfinite(options.receive_lag_step_sec) ||
      !(options.receive_lag_step_sec > 0.0) ||
      !std::isfinite(lag_steps) || lag_steps > 1e6 ||
      !std::isfinite(options.min_receive_lag_correlation) ||
      options.min_receive_lag_correlation < -1.0 ||
      options.min_receive_lag_correlation > 1.0 ||
      options.min_receive_lag_pairs < 3 ||
      !(options.max_interpolation_gap_sec > 0.0) ||
      options.relative_motion_stride == 0 ||
      !std::isfinite(options.y_prior_sigma_m) ||
      !(options.y_prior_sigma_m > 0.0) ||
      !std::isfinite(options.max_rotation_rms_rad) ||
      !(options.max_rotation_rms_rad > 0.0) ||
      !std::isfinite(options.max_translation_rms_m) ||
      !(options.max_translation_rms_m > 0.0) ||
      !std::isfinite(options.max_condition_number) ||
      !(options.max_condition_number > 0.0) ||
      !(options.max_final_neutral_rotation_error_rad > 0.0) ||
      options.max_final_neutral_rotation_error_rad > M_PI ||
      !(options.max_final_neutral_tilt_rad > 0.0) ||
      options.max_final_neutral_tilt_rad > M_PI ||
      options.max_refinement_iterations < 0 ||
      !(options.refinement_huber_delta > 0.0) ||
      !std::isfinite(options.rotation_prior_sigma_rad) ||
      !(options.rotation_prior_sigma_rad > 0.0) ||
      !std::isfinite(options.mount_x_min_m) ||
      !std::isfinite(options.mount_x_max_m) ||
      !(options.mount_x_min_m < options.mount_x_max_m) ||
      !(options.mount_abs_y_max_m > 0.0) ||
      !(options.mount_abs_z_max_m > 0.0) ||
      !(options.mount_rear_angle_max_rad > 0.0) ||
      options.mount_rear_angle_max_rad > M_PI) {
    return failure("invalid solver options");
  }
  if (pico.size() < options.min_pairs || odin.size() < options.min_pairs) {
    return failure("insufficient input samples");
  }
  if (!std::is_sorted(pico.begin(), pico.end(), [](const auto & a, const auto & b) {
      return a.receipt_sec < b.receipt_sec;
    }) || !std::is_sorted(odin.begin(), odin.end(), [](const auto & a, const auto & b) {
      return a.receipt_sec < b.receipt_sec;
    })) {
    return failure("input timestamps are not monotonic");
  }
  for (const auto & sample : pico) {
    if (!std::isfinite(sample.receipt_sec) || !finite(sample.pose)) return failure("invalid PICO pose");
  }
  for (const auto & sample : odin) {
    if (!std::isfinite(sample.receipt_sec) || !finite(sample.pose)) return failure("invalid Odin pose");
  }

  const auto [pitch_range, yaw_range] = pitch_yaw_ranges(pico);
  if (pitch_range < options.min_pitch_range_rad) {
    return failure("insufficient pitch excitation", pitch_range, yaw_range);
  }
  if (yaw_range < options.min_yaw_range_rad) {
    return failure("insufficient yaw excitation", pitch_range, yaw_range);
  }

  const auto offset = estimate_receive_lag_impl(pico, odin, options);
  if (!offset.valid ||
      (options.reject_receive_lag_at_boundary && offset.at_boundary)) {
    return failure("time alignment correlation is invalid or at search boundary", pitch_range, yaw_range);
  }
  const auto synchronized = synchronized_poses(
    pico, odin, offset.lag_sec, options.max_interpolation_gap_sec);
  if (synchronized.size() < options.min_pairs) {
    return failure("insufficient synchronized pose pairs", pitch_range, yaw_range);
  }
  const auto motions = relative_motions(synchronized, options.relative_motion_stride);
  if (motions.size() < 12) {
    return failure("insufficient non-static relative motions", pitch_range, yaw_range);
  }

  std::vector<MotionPair> robust_motions = motions;
  Pose3 odin_T_pelvis;
  double condition = std::numeric_limits<double>::infinity();
  for (int iteration = 0; iteration < 3; ++iteration) {
    const auto rotation_solution = solve_rotation(robust_motions);
    odin_T_pelvis.rotation = rotation_solution.first;
    const auto translation_solution =
      solve_translation(
        robust_motions, odin_T_pelvis.rotation, options.y_prior_sigma_m);
    odin_T_pelvis.translation = translation_solution.first;
    condition = std::max(rotation_solution.second, translation_solution.second);
    const auto filtered = reject_outliers(motions, odin_T_pelvis);
    if (filtered.size() < 12 || filtered.size() == robust_motions.size()) break;
    robust_motions = filtered;
  }
  const auto refined = refine_hand_eye(robust_motions, odin_T_pelvis, options);
  odin_T_pelvis = refined.first;
  const Pose3 pelvis_T_odin = inverse(odin_T_pelvis);
  CalibrationMetrics metrics = calculate_metrics(
    robust_motions, odin_T_pelvis, synchronized.size(),
    pitch_range, yaw_range, condition);
  metrics.time_correlation = offset.correlation;
  metrics.refinement_iterations = refined.second;

  if (!std::isfinite(condition) || condition > options.max_condition_number) {
    return failure("ill-conditioned translation solve", pitch_range, yaw_range);
  }
  if (metrics.rotation_rms_rad > options.max_rotation_rms_rad) {
    return failure("rotation residual exceeds limit", pitch_range, yaw_range);
  }
  if (metrics.translation_rms_m > options.max_translation_rms_m) {
    return failure("translation residual exceeds limit", pitch_range, yaw_range);
  }
  if (!final_neutral_is_consistent(synchronized, pelvis_T_odin, options)) {
    return failure(
      "final neutral pose is inconsistent with the solved transform",
      pitch_range, yaw_range);
  }
  if (pelvis_T_odin.translation.x() < options.mount_x_min_m ||
      pelvis_T_odin.translation.x() > options.mount_x_max_m ||
      std::abs(pelvis_T_odin.translation.y()) > options.mount_abs_y_max_m ||
      std::abs(pelvis_T_odin.translation.z()) > options.mount_abs_z_max_m) {
    return failure("solved mounting translation is outside physical bounds", pitch_range, yaw_range);
  }
  const Eigen::Quaterniond rear_facing(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
  if (angular_distance(pelvis_T_odin.rotation, rear_facing) >
      options.mount_rear_angle_max_rad) {
    return failure("solved mounting rotation is outside rear-facing prior", pitch_range, yaw_range);
  }

  SolverResult result;
  result.success = true;
  result.message = "calibration accepted";
  result.pelvis_T_odin = pelvis_T_odin;
  result.metrics = metrics;
  result.receive_lag_sec = offset.lag_sec;
  return result;
}

ReceiveLagEstimate estimate_receive_lag(
  const std::vector<HostTimedPose> & pico,
  const std::vector<HostTimedPose> & odin,
  const SolverOptions & options)
{
  const double lag_steps =
    2.0 * options.max_receive_lag_sec / options.receive_lag_step_sec;
  if (!std::isfinite(options.max_receive_lag_sec) ||
      !(options.max_receive_lag_sec > 0.0) ||
      !std::isfinite(options.receive_lag_step_sec) ||
      !(options.receive_lag_step_sec > 0.0) ||
      !std::isfinite(options.min_receive_lag_correlation) ||
      options.min_receive_lag_correlation < -1.0 ||
      options.min_receive_lag_correlation > 1.0 ||
      options.min_receive_lag_pairs < 3 ||
      !std::isfinite(options.max_interpolation_gap_sec) ||
      !(options.max_interpolation_gap_sec > 0.0) ||
      !std::isfinite(lag_steps) || lag_steps > 1e6 ||
      !std::is_sorted(pico.begin(), pico.end(), [](const auto & a, const auto & b) {
        return a.receipt_sec < b.receipt_sec;
      }) ||
      !std::is_sorted(odin.begin(), odin.end(), [](const auto & a, const auto & b) {
        return a.receipt_sec < b.receipt_sec;
      })) {
    return {};
  }
  for (const auto & sample : pico) {
    if (!std::isfinite(sample.receipt_sec) || !finite(sample.pose)) return {};
  }
  for (const auto & sample : odin) {
    if (!std::isfinite(sample.receipt_sec) || !finite(sample.pose)) return {};
  }
  return estimate_receive_lag_impl(pico, odin, options);
}

}  // namespace pico_odin
