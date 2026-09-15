// Copyright (c) 2025 WujiHand Technologies.
// Adapted from this package's MIT-licensed opt/adaptive_analytical.py; see ../LICENSE.
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <utility>

namespace py = pybind11;
using Array = py::array_t<double, py::array::c_style>;
using Fingers = std::array<py::ssize_t, 5>;

namespace {
constexpr double kEpsilon = 1e-8;

double norm(const std::array<double, 3>& v) {
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

double huber(double distance, double delta) {
    return distance <= delta ? 0.5 * distance * distance
                             : delta * (distance - 0.5 * delta);
}

double huber_gradient(double distance, double delta) {
    if (std::isnan(distance)) return distance;
    return distance <= delta ? distance : delta * (distance > 0.0 ? 1.0 : 0.0);
}

void matrix_shape(const Array& array, py::ssize_t rows, const char* name) {
    if (array.ndim() != 2 || array.shape(0) != rows || array.shape(1) != 3) {
        throw py::value_error(std::string(name) + " has an invalid shape");
    }
}

// Link indices are immutable model topology, converted once rather than on
// each NLopt callback. All frame arrays are borrowed contiguous float64 views.
class AdaptiveGeometry {
public:
    AdaptiveGeometry(Fingers origins, Fingers tips, Fingers pips, Fingers dips)
        : origins_(origins), tips_(tips), pips_(pips), dips_(dips) {
        for (const auto& indices : {origins_, tips_, pips_, dips_}) {
            for (auto index : indices) {
                if (index < 0) {
                    throw py::value_error("link indices must be nonnegative");
                }
                max_index_ = std::max(max_index_, index);
            }
        }
    }

    std::pair<double, Array> loss_gradient(
        const Array& positions, const Array& jacobians,
        const Array& target_tips, const Array& target_directions,
        const Array& target_full, const Array& alphas,
        double delta, double direction_delta, double w_position,
        double w_direction, double w_full, bool thumb_skip_pip) const {
        if (positions.ndim() != 2 || positions.shape(1) != 3 ||
            positions.shape(0) <= max_index_) {
            throw py::value_error("positions must have shape (num_links, 3) and contain every model link");
        }
        if (jacobians.ndim() != 3 || jacobians.shape(0) != positions.shape(0) ||
            jacobians.shape(1) != 3 || jacobians.shape(2) == 0) {
            throw py::value_error("jacobians must have shape (num_links, 3, num_joints)");
        }
        matrix_shape(target_tips, 5, "target_tips");
        matrix_shape(target_directions, 5, "target_directions");
        matrix_shape(target_full, 15, "target_full");
        if (alphas.ndim() != 1 || alphas.shape(0) != 5) {
            throw py::value_error("alphas must have shape (5,)");
        }

        const auto p = positions.unchecked<2>();
        const auto j = jacobians.unchecked<3>();
        const auto tip = target_tips.unchecked<2>();
        const auto direction = target_directions.unchecked<2>();
        const auto full = target_full.unchecked<2>();
        const auto alpha = alphas.unchecked<1>();
        const auto nq = jacobians.shape(2);
        Array gradient(nq);
        auto g = gradient.mutable_unchecked<1>();
        std::fill_n(gradient.mutable_data(), nq, 0.0);
        std::array<double, 5> position_losses{}, direction_losses{}, full_losses{};

        // Accumulate in the same term/finger order as the analytical Python
        // objective. Fixed three-vectors/matrices live on the stack; only the
        // returned gradient allocates. Epsilon and Huber conventions are exact.
        for (std::size_t finger = 0; finger < 5; ++finger) {
            std::array<double, 3> diff{};
            for (int axis = 0; axis < 3; ++axis) {
                diff[axis] = p(tips_[finger], axis) - p(origins_[finger], axis) - tip(finger, axis);
            }
            const double distance = norm(diff);
            position_losses[finger] = huber(distance, delta);
            const double coefficient = alpha(finger) * w_position * huber_gradient(distance, delta);
            for (auto& value : diff) value /= distance + kEpsilon;
            for (py::ssize_t q = 0; q < nq; ++q) {
                double dot = 0.0;
                for (int axis = 0; axis < 3; ++axis) {
                    dot += diff[axis] * (j(tips_[finger], axis, q) - j(origins_[finger], axis, q));
                }
                g(q) += coefficient * dot;
            }
        }

        for (std::size_t finger = 0; finger < 5; ++finger) {
            std::array<double, 3> unit{}, diff{}, normalized_gradient{};
            for (int axis = 0; axis < 3; ++axis) {
                unit[axis] = p(tips_[finger], axis) - p(dips_[finger], axis);
            }
            const double length = norm(unit);
            for (int axis = 0; axis < 3; ++axis) {
                unit[axis] /= length + kEpsilon;
                diff[axis] = unit[axis] - direction(finger, axis);
            }
            const double distance = norm(diff);
            direction_losses[finger] = huber(distance, direction_delta);
            const double coefficient = alpha(finger) * w_direction * huber_gradient(distance, direction_delta);
            for (auto& value : diff) value /= distance + kEpsilon;
            for (int column = 0; column < 3; ++column) {
                for (int row = 0; row < 3; ++row) {
                    const double jacobian = ((row == column ? 1.0 : 0.0) - unit[row] * unit[column]) /
                                            (length + kEpsilon);
                    normalized_gradient[column] += diff[row] * jacobian;
                }
            }
            for (py::ssize_t q = 0; q < nq; ++q) {
                double dot = 0.0;
                for (int axis = 0; axis < 3; ++axis) {
                    dot += normalized_gradient[axis] * (j(tips_[finger], axis, q) - j(dips_[finger], axis, q));
                }
                g(q) += coefficient * dot;
            }
        }

        const auto wrist = origins_[0];
        for (std::size_t finger = 0; finger < 5; ++finger) {
            const bool skip_pip = thumb_skip_pip && finger == 0;
            const double terms = skip_pip ? 2.0 : 3.0;
            const double coefficient = (1.0 - alpha(finger)) * w_full / terms;
            const std::array<py::ssize_t, 3> links{pips_[finger], dips_[finger], tips_[finger]};
            double finger_loss = 0.0;
            for (int term = 0; term < 3; ++term) {
                std::array<double, 3> diff{};
                for (int axis = 0; axis < 3; ++axis) {
                    diff[axis] = p(links[term], axis) - p(wrist, axis) - full(term * 5 + finger, axis);
                }
                const double distance = norm(diff);
                // Preserve the original mask multiplication, including its NaN
                // behavior. Only the gradient term is skipped for thumb PIP.
                finger_loss += (term == 0 && skip_pip ? 0.0 : 1.0) * huber(distance, delta);
                if (term == 0 && skip_pip) continue;
                const double weighted_gradient = coefficient * huber_gradient(distance, delta);
                for (auto& value : diff) value /= distance + kEpsilon;
                for (py::ssize_t q = 0; q < nq; ++q) {
                    double dot = 0.0;
                    for (int axis = 0; axis < 3; ++axis) {
                        dot += diff[axis] * (j(links[term], axis, q) - j(wrist, axis, q));
                    }
                    g(q) += weighted_gradient * dot;
                }
            }
            full_losses[finger] = finger_loss / terms;
        }

        double loss = 0.0;
        for (std::size_t finger = 0; finger < 5; ++finger) {
            loss += alpha(finger) * (w_position * position_losses[finger] + w_direction * direction_losses[finger]) +
                    (1.0 - alpha(finger)) * (w_full * full_losses[finger]);
        }
        return {loss, std::move(gradient)};
    }

private:
    Fingers origins_, tips_, pips_, dips_;
    py::ssize_t max_index_ = 0;
};
}  // namespace

PYBIND11_MODULE(_native, module) {
    module.doc() = "Allocation-bounded analytical hand objective geometry";
    py::class_<AdaptiveGeometry>(module, "AdaptiveGeometry")
        .def(py::init<Fingers, Fingers, Fingers, Fingers>(),
             py::arg("origins"), py::arg("tips"), py::arg("pips"), py::arg("dips"))
        .def("loss_gradient", &AdaptiveGeometry::loss_gradient,
             py::arg("positions").noconvert(), py::arg("jacobians").noconvert(),
             py::arg("target_tips").noconvert(), py::arg("target_directions").noconvert(),
             py::arg("target_full").noconvert(), py::arg("alphas").noconvert(),
             py::arg("delta"), py::arg("direction_delta"), py::arg("w_position"),
             py::arg("w_direction"), py::arg("w_full"), py::arg("thumb_skip_pip"));
}
