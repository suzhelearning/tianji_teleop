#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <mutex>
#include <string>

namespace py = pybind11;

namespace {
using Vec3 = std::array<double, 3>;
using Angles = std::array<double, 4>;
using Matrix3 = std::array<Vec3, 3>;
using Pose = std::array<Angles, 5>;
using Points = std::array<Vec3, 21>;
using Targets = std::array<Vec3, 5>;
using Directions = std::array<std::array<Vec3, 3>, 5>;
using Residual = std::array<double, 16>;
using Jacobian = std::array<Angles, 16>;
using Matrix4 = std::array<Angles, 4>;
using Movable = std::array<bool, 5>;
using NumericArray = py::array_t<double, py::array::c_style | py::array::forcecast>;

NumericArray numeric_array(py::handle value, std::initializer_list<py::ssize_t> shape,
                           const char* name) {
    std::string message = std::string(name) + " 必须是形状 (";
    bool first = true;
    for (auto extent : shape) {
        if (!first) message += ", ";
        message += std::to_string(extent);
        first = false;
    }
    if (shape.size() == 1) message += ",";
    message += ") 的有限数值数组";
    auto raw = py::array::ensure(value);
    if (!raw) throw py::value_error(message);
    const char kind = raw.dtype().kind();
    if (kind != 'i' && kind != 'u' && kind != 'f') throw py::value_error(message);
    if (raw.ndim() != static_cast<py::ssize_t>(shape.size())) throw py::value_error(message);
    std::size_t axis = 0;
    for (auto extent : shape) {
        if (raw.shape(axis++) != extent) throw py::value_error(message);
    }
    auto array = NumericArray::ensure(raw);
    if (!array) throw py::value_error(message);
    for (py::ssize_t i = 0; i < array.size(); ++i) {
        if (!std::isfinite(array.data()[i])) throw py::value_error(message);
    }
    return array;
}

Vec3 subtract(const Vec3& a, const Vec3& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

double norm(const Vec3& v) {
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

Vec3 rotate(const Matrix3& matrix, const Vec3& vector) {
    Vec3 result{};
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j) result[i] += matrix[i][j] * vector[j];
    return result;
}

Vec3 unrotate(const Matrix3& matrix, const Vec3& vector) {
    Vec3 result{};
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j) result[i] += matrix[j][i] * vector[j];
    return result;
}

double squared_cost(const Residual& residual) {
    double result = 0.0;
    for (double value : residual) result += value * value;
    return result;
}

Angles gradient_of(const Jacobian& jacobian, const Residual& residual) {
    Angles result{};
    for (std::size_t row = 0; row < 16; ++row)
        for (std::size_t joint = 0; joint < 4; ++joint)
            result[joint] += jacobian[row][joint] * residual[row];
    return result;
}

double max_abs(const Angles& values) {
    double result = 0.0;
    for (double value : values) {
        if (!std::isfinite(value)) return std::abs(value);
        result = std::max(result, std::abs(value));
    }
    return result;
}

Angles solve(Matrix4 matrix, Angles rhs) {
    // Partial pivoting keeps the damped active-set solve stable without an inverse.
    for (std::size_t column = 0; column < 4; ++column) {
        std::size_t pivot = column;
        for (std::size_t row = column; row < 4; ++row) {
            if (!std::isfinite(matrix[row][column]) || !std::isfinite(rhs[row]))
                throw py::value_error("姿态求解数值溢出");
            if (std::abs(matrix[row][column]) > std::abs(matrix[pivot][column])) pivot = row;
        }
        if (matrix[pivot][column] == 0.0) throw py::value_error("姿态求解矩阵奇异");
        if (pivot != column) {
            std::swap(matrix[pivot], matrix[column]);
            std::swap(rhs[pivot], rhs[column]);
        }
        for (std::size_t row = column + 1; row < 4; ++row) {
            const double factor = matrix[row][column] / matrix[column][column];
            matrix[row][column] = 0.0;
            for (std::size_t j = column + 1; j < 4; ++j)
                matrix[row][j] -= factor * matrix[column][j];
            rhs[row] -= factor * rhs[column];
        }
    }
    Angles result{};
    for (int row = 3; row >= 0; --row) {
        double value = rhs[row];
        for (std::size_t column = row + 1; column < 4; ++column)
            value -= matrix[row][column] * result[column];
        result[row] = value / matrix[row][row];
        if (!std::isfinite(result[row])) throw py::value_error("姿态求解数值溢出");
    }
    return result;
}

class MorphologyFitter {
public:
    MorphologyFitter(py::handle roots, py::handle bases, py::handle lengths,
                     py::handle limits, py::handle rotation, double direction_weight,
                     double temporal_weight, std::size_t max_iterations,
                     double pinch_distance, double pinch_full_contact, double pinch_target)
        : direction_weight_(direction_weight), temporal_weight_(temporal_weight),
          max_iterations_(max_iterations), pinch_distance_(pinch_distance),
          pinch_full_contact_(pinch_full_contact), pinch_target_(pinch_target) {
        const auto r = numeric_array(roots, {5, 3}, "roots");
        const auto b = numeric_array(bases, {5, 3, 3}, "bases");
        const auto l = numeric_array(lengths, {5, 3}, "lengths");
        const auto bounds = numeric_array(limits, {5, 4, 2}, "limits");
        const auto rotation_array = numeric_array(rotation, {3, 3}, "rotation");
        for (std::size_t f = 0; f < 5; ++f) {
            for (std::size_t i = 0; i < 3; ++i) {
                roots_[f][i] = r.data()[f * 3 + i];
                lengths_[f][i] = l.data()[f * 3 + i];
                for (std::size_t j = 0; j < 3; ++j)
                    bases_[f][i][j] = b.data()[f * 9 + i * 3 + j];
            }
            for (std::size_t j = 0; j < 4; ++j) {
                lower_[f][j] = bounds.data()[f * 8 + j * 2];
                upper_[f][j] = bounds.data()[f * 8 + j * 2 + 1];
            }
        }
        for (std::size_t i = 0; i < 3; ++i)
            for (std::size_t j = 0; j < 3; ++j)
                rotation_[i][j] = rotation_array.data()[i * 3 + j];
    }

    py::array_t<double> forward(py::handle pose, py::object wrist) const {
        const auto angles = numeric_array(pose, {5, 4}, "pose（弧度）");
        Pose input{};
        for (std::size_t f = 0; f < 5; ++f) {
            for (std::size_t j = 0; j < 4; ++j) {
                input[f][j] = angles.data()[f * 4 + j];
                if (input[f][j] < lower_[f][j] || input[f][j] > upper_[f][j])
                    throw py::value_error("pose 超出 joint_limits_rad");
            }
        }
        Vec3 origin{};
        if (!wrist.is_none()) {
            const auto array = numeric_array(wrist, {3}, "wrist（米）");
            std::copy_n(array.data(), 3, origin.begin());
        }
        return output(points(input, origin), "前向运动学数值溢出");
    }

    py::array_t<double> fit(py::handle keypoints) {
        const auto array = numeric_array(keypoints, {21, 3}, "keypoints（米）");
        Points input{};
        for (std::size_t i = 0; i < 21; ++i)
            std::copy_n(array.data() + i * 3, 3, input[i].begin());
        // Allocate while holding the GIL; numerical work only sees owned storage.
        py::array_t<double> result({21, 3});
        double* destination = result.mutable_data();
        {
            py::gil_scoped_release release;
            std::lock_guard<std::mutex> lock(history_mutex_);
            const Points fitted = fit_points(input);
            for (std::size_t i = 0; i < 21; ++i)
                std::copy(fitted[i].begin(), fitted[i].end(), destination + i * 3);
        }
        return result;
    }

private:
    Points fit_points(const Points& input) {
        Directions observed_directions{};
        Targets targets{};
        for (std::size_t f = 0; f < 5; ++f) {
            for (std::size_t s = 0; s < 3; ++s) {
                Vec3 segment = subtract(input[2 + f * 4 + s], input[1 + f * 4 + s]);
                const double length = norm(segment);
                if (!std::isfinite(length) || length <= 1e-9)
                    throw py::value_error("观测指链包含退化骨段");
                for (double& value : segment) value /= length;
                observed_directions[f][s] = unrotate(rotation_, segment);
            }
            targets[f] = unrotate(rotation_, subtract(input[4 + f * 4], input[0]));
            for (double value : targets[f])
                if (!std::isfinite(value)) throw py::value_error("观测目标超出数值求解范围");
        }
        const Pose observed_pose = optimize(targets, observed_directions,
                                           has_previous_ ? &previous_observed_pose_ : nullptr);
        Pose pose = observed_pose;
        Points fitted = points(pose, input[0]);
        if (pinch_distance_ > 0.0) {
            Movable movable{};
            Targets contact_targets = targets;
            bool any = false;
            for (std::size_t f = 1; f < 5; ++f) {
                const double source_gap = norm(subtract(input[4 + f * 4], input[4]));
                const double progress = std::clamp((pinch_distance_ - source_gap) /
                                                       (pinch_distance_ - pinch_full_contact_),
                                                   0.0, 1.0);
                const double strength = progress * progress * (3.0 - 2.0 * progress);
                const Vec3 tip_vector = subtract(fitted[4 + f * 4], fitted[4]);
                const double fitted_gap = norm(tip_vector);
                const double target_gap = (1.0 - strength) * fitted_gap + strength * pinch_target_;
                if (strength > 0.0 && fitted_gap > target_gap + 1e-6) {
                    Vec3 contact{};
                    for (std::size_t i = 0; i < 3; ++i)
                        contact[i] = fitted[4][i] + tip_vector[i] * (target_gap / fitted_gap);
                    contact_targets[f] = unrotate(rotation_, subtract(contact, input[0]));
                    movable[f] = true;
                    any = true;
                }
            }
            if (any) {
                pose = optimize(contact_targets, observed_directions,
                                has_previous_ ? &previous_pose_ : nullptr, &observed_pose, &movable);
                fitted = points(pose, input[0]);
            }
        }
        for (const auto& finger : pose)
            for (double value : finger)
                if (!std::isfinite(value)) throw py::value_error("姿态求解数值溢出");
        for (const auto& point : fitted)
            for (double value : point)
                if (!std::isfinite(value)) throw py::value_error("姿态求解数值溢出");
        // Both histories commit together only after all numerical checks succeed.
        previous_observed_pose_ = observed_pose;
        previous_pose_ = pose;
        has_previous_ = true;
        return fitted;
    }

public:
    py::object previous_pose(bool observed) const {
        Pose pose{};
        bool available;
        {
            py::gil_scoped_release release;
            std::lock_guard<std::mutex> lock(history_mutex_);
            available = has_previous_;
            if (available) pose = observed ? previous_observed_pose_ : previous_pose_;
        }
        if (!available) return py::none();
        py::array_t<double> result({5, 4});
        for (std::size_t f = 0; f < 5; ++f)
            std::copy(pose[f].begin(), pose[f].end(), result.mutable_data() + f * 4);
        return result;
    }

private:
    struct FingerDirections {
        std::array<Vec3, 3> values{};
        Vec3 sin_flex{};
        Vec3 cos_flex{};
        double sin_abd;
        double cos_abd;
    };

    FingerDirections directions(std::size_t finger, const Angles& pose) const {
        FingerDirections result{};
        result.sin_abd = std::sin(pose[0]);
        result.cos_abd = std::cos(pose[0]);
        double flexion = 0.0;
        for (std::size_t segment = 0; segment < 3; ++segment) {
            flexion += pose[segment + 1];
            result.sin_flex[segment] = std::sin(flexion);
            result.cos_flex[segment] = std::cos(flexion);
            result.values[segment] = rotate(bases_[finger],
                {-result.sin_abd * result.cos_flex[segment],
                  result.cos_abd * result.cos_flex[segment], result.sin_flex[segment]});
        }
        return result;
    }

    Points points(const Pose& pose, const Vec3& wrist) const {
        Points result{};
        result[0] = wrist;
        for (std::size_t f = 0; f < 5; ++f) {
            const auto dir = directions(f, pose[f]);
            Vec3 displacement{};
            for (std::size_t s = 0; s < 4; ++s) {
                Vec3 point = roots_[f];
                for (std::size_t i = 0; i < 3; ++i) {
                    if (s != 0) displacement[i] += lengths_[f][s - 1] * dir.values[s - 1][i];
                    point[i] += displacement[i];
                }
                point = rotate(rotation_, point);
                for (std::size_t i = 0; i < 3; ++i) result[1 + f * 4 + s][i] = point[i] + wrist[i];
            }
        }
        return result;
    }

    Residual residual(std::size_t f, const Angles& pose, const Vec3& target,
                      const std::array<Vec3, 3>& observed, const Angles* previous,
                      Jacobian* jacobian = nullptr) const {
        const auto dir = directions(f, pose);
        Residual result{};
        Vec3 endpoint{};
        if (jacobian) *jacobian = {};
        for (std::size_t s = 0; s < 3; ++s) {
            for (std::size_t i = 0; i < 3; ++i) {
                endpoint[i] += lengths_[f][s] * dir.values[s][i];
                result[3 + s * 3 + i] = direction_weight_ * (dir.values[s][i] - observed[s][i]);
            }
            if (!jacobian) continue;
            const Vec3 abd = rotate(bases_[f], {-dir.cos_abd * dir.cos_flex[s],
                                              -dir.sin_abd * dir.cos_flex[s], 0.0});
            const Vec3 flex = rotate(bases_[f], {dir.sin_abd * dir.sin_flex[s],
                                               -dir.cos_abd * dir.sin_flex[s], dir.cos_flex[s]});
            for (std::size_t i = 0; i < 3; ++i) {
                (*jacobian)[i][0] += lengths_[f][s] * abd[i];
                (*jacobian)[3 + s * 3 + i][0] = direction_weight_ * abd[i];
                for (std::size_t joint = 1; joint <= s + 1; ++joint) {
                    (*jacobian)[i][joint] += lengths_[f][s] * flex[i];
                    (*jacobian)[3 + s * 3 + i][joint] = direction_weight_ * flex[i];
                }
            }
        }
        for (std::size_t i = 0; i < 3; ++i) result[i] = roots_[f][i] + endpoint[i] - target[i];
        if (previous) {
            for (std::size_t joint = 0; joint < 4; ++joint) {
                result[12 + joint] = temporal_weight_ * (pose[joint] - (*previous)[joint]);
                if (jacobian) (*jacobian)[12 + joint][joint] = temporal_weight_;
            }
        }
        return result;
    }

    Angles direction_seed(std::size_t f, const std::array<Vec3, 3>& observed) const {
        std::array<Vec3, 3> local{};
        for (std::size_t s = 0; s < 3; ++s) local[s] = unrotate(bases_[f], observed[s]);
        const double abduction = std::atan2(-local[0][0], local[0][1]);
        const double sine = std::sin(abduction), cosine = std::cos(abduction);
        Vec3 flexion{};
        for (std::size_t s = 0; s < 3; ++s)
            flexion[s] = std::atan2(local[s][2], -sine * local[s][0] + cosine * local[s][1]);
        Angles result{abduction, flexion[0], 0.0, 0.0};
        for (std::size_t j = 0; j < 2; ++j) {
            const double increment = flexion[j + 1] - flexion[j];
            result[j + 2] = std::atan2(std::sin(increment), std::cos(increment));
        }
        return result;
    }

    Angles endpoint_seed(std::size_t f, const Vec3& target, std::size_t bend_joint, double sign) const {
        const Vec3 local = unrotate(bases_[f], subtract(target, roots_[f]));
        const double radius = norm(local);
        double proximal = 0.0, distal = 0.0;
        for (std::size_t s = 0; s < 3; ++s) {
            if (s < bend_joint) proximal += lengths_[f][s];
            else distal += lengths_[f][s];
        }
        const double cosine = (radius * radius - proximal * proximal - distal * distal) /
                              (2.0 * proximal * distal);
        const double bend = sign * std::acos(std::clamp(cosine, -1.0, 1.0));
        Angles result{};
        result[0] = std::atan2(-local[0], local[1]);
        result[1] = std::atan2(local[2], std::hypot(local[0], local[1])) -
                    std::atan2(distal * std::sin(bend), proximal + distal * std::cos(bend));
        result[bend_joint + 1] = bend;
        return result;
    }

    Angles clipped(std::size_t f, Angles pose) const {
        for (std::size_t j = 0; j < 4; ++j) pose[j] = std::clamp(pose[j], lower_[f][j], upper_[f][j]);
        return pose;
    }

    Pose optimize(const Targets& targets, const Directions& observed, const Pose* previous,
                  const Pose* initial = nullptr, const Movable* movable = nullptr) const {
        Pose pose{};
        std::array<double, 5> cost{}, damping{};
        for (std::size_t f = 0; f < 5; ++f) {
            const Angles seed = clipped(f, direction_seed(f, observed[f]));
            const Angles* history = previous ? &(*previous)[f] : nullptr;
            pose[f] = initial ? (*initial)[f] : (history ? *history : seed);
            Jacobian derivative{};
            const auto current = residual(f, pose[f], targets[f], observed[f], history, &derivative);
            cost[f] = squared_cost(current);
            // Only the first frame or a stationary gradient can restart a branch.
            const bool restart = (!previous || max_abs(gradient_of(derivative, current)) < 1e-11) &&
                                 (!movable || (*movable)[f]);
            if (restart) {
                for (std::size_t choice = 0; choice < 5; ++choice) {
                    const Angles candidate = choice == 0 ? seed : clipped(f, endpoint_seed(
                        f, targets[f], 1 + (choice - 1) / 2, choice % 2 == 1 ? 1.0 : -1.0));
                    const double candidate_cost = squared_cost(residual(f, candidate, targets[f], observed[f], history));
                    if (candidate_cost < cost[f]) {
                        pose[f] = candidate;
                        cost[f] = candidate_cost;
                    }
                }
            }
            if (!std::isfinite(cost[f])) throw py::value_error("观测目标超出数值求解范围");
            damping[f] = 1e-6;
        }
        for (std::size_t iteration = 0; iteration < max_iterations_; ++iteration) {
            std::array<Matrix4, 5> hessian{};
            Pose gradient{};
            bool stationary = true;
            for (std::size_t f = 0; f < 5; ++f) {
                Jacobian derivative{};
                const auto current = residual(f, pose[f], targets[f], observed[f],
                                              previous ? &(*previous)[f] : nullptr, &derivative);
                gradient[f] = gradient_of(derivative, current);
                std::array<bool, 4> free{};
                for (std::size_t j = 0; j < 4; ++j) {
                    const bool blocked = (pose[f][j] <= lower_[f][j] && gradient[f][j] > 0.0) ||
                                         (pose[f][j] >= upper_[f][j] && gradient[f][j] < 0.0) ||
                                         lower_[f][j] == upper_[f][j] || (movable && !(*movable)[f]);
                    free[j] = !blocked;
                    if (blocked) gradient[f][j] = 0.0;
                }
                if (!(max_abs(gradient[f]) < 1e-11)) stationary = false;
                for (std::size_t i = 0; i < 4; ++i) {
                    for (std::size_t j = 0; j < 4; ++j) {
                        if (free[i] && free[j])
                            for (std::size_t row = 0; row < 16; ++row)
                                hessian[f][i][j] += derivative[row][i] * derivative[row][j];
                    }
                    hessian[f][i][i] += damping[f];
                }
            }
            if (stationary) break;
            for (std::size_t f = 0; f < 5; ++f) {
                for (double& value : gradient[f]) value = -value;
                Angles step = solve(hessian[f], gradient[f]);
                const double scale = std::min(1.0, 0.5 / std::max(max_abs(step), 1e-15));
                Angles candidate{};
                for (std::size_t j = 0; j < 4; ++j) candidate[j] = pose[f][j] + step[j] * scale;
                candidate = clipped(f, candidate);
                const double candidate_cost = squared_cost(residual(f, candidate, targets[f], observed[f],
                                                                     previous ? &(*previous)[f] : nullptr));
                if (std::isfinite(candidate_cost) && candidate_cost < cost[f]) {
                    pose[f] = candidate;
                    cost[f] = candidate_cost;
                    damping[f] = std::max(damping[f] / 3.0, 1e-10);
                } else {
                    damping[f] = std::min(damping[f] * 10.0, 1e8);
                }
            }
        }
        return pose;
    }

    static py::array_t<double> output(const Points& points, const char* error) {
        py::array_t<double> result({21, 3});
        for (std::size_t i = 0; i < 21; ++i) {
            for (std::size_t j = 0; j < 3; ++j) {
                if (!std::isfinite(points[i][j])) throw py::value_error(error);
                result.mutable_data()[i * 3 + j] = points[i][j];
            }
        }
        return result;
    }

    Targets roots_{};
    std::array<Matrix3, 5> bases_{};
    std::array<Vec3, 5> lengths_{};
    Pose lower_{}, upper_{};
    Matrix3 rotation_{};
    double direction_weight_, temporal_weight_;
    std::size_t max_iterations_;
    double pinch_distance_, pinch_full_contact_, pinch_target_;
    mutable std::mutex history_mutex_;
    bool has_previous_ = false;
    Pose previous_pose_{}, previous_observed_pose_{};
};
}  // namespace

void bind_morphology(py::module_& module) {
    py::class_<MorphologyFitter>(module, "MorphologyFitter")
        .def(py::init<py::handle, py::handle, py::handle, py::handle, py::handle,
                      double, double, std::size_t, double, double, double>(),
             py::arg("roots"), py::arg("bases"), py::arg("lengths"), py::arg("limits"),
             py::arg("rotation"), py::arg("direction_weight"), py::arg("temporal_weight"),
             py::arg("max_iterations"), py::arg("pinch_distance"), py::arg("pinch_full_contact"),
             py::arg("pinch_target"))
        .def("fit", &MorphologyFitter::fit, py::arg("keypoints"))
        .def("forward", &MorphologyFitter::forward, py::arg("pose"), py::arg("wrist") = py::none())
        .def_property_readonly("previous_pose", [](const MorphologyFitter& self) {
            return self.previous_pose(false);
        })
        .def_property_readonly("previous_observed_pose", [](const MorphologyFitter& self) {
            return self.previous_pose(true);
        });
}
