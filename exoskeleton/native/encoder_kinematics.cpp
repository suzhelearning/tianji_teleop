#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace py = pybind11;

namespace {

constexpr std::size_t channel_count = 21;
constexpr double pi = 3.141592653589793238462643383279502884;
constexpr double radians_per_degree = pi / 180.0;
constexpr double degrees_per_radian = 180.0 / pi;
using Point = std::array<double, 2>;

struct Geometry {
    Point B;
    Point C;
    double theta_A;
    double theta_D;
    double distance_BD;
};

double square(double value) {
    const double result = value * value;
    // Python's finite float ** 2 reports overflow instead of returning infinity.
    if (std::isinf(result) && std::isfinite(value)) {
        throw std::overflow_error("(34, 'Numerical result out of range')");
    }
    return result;
}

std::string distance_error(double distance, double limit, const char* comparison) {
    std::ostringstream message;
    message.imbue(std::locale::classic());
    message << "四连杆无法闭合：BD=" << std::fixed << std::setprecision(6)
            << distance << comparison << limit;
    return message.str();
}

class FourBarLinkage {
public:
    FourBarLinkage(double L_AD, double L_AB, double L_BC, double L_CD,
                   double theta_A0_deg, int branch)
        : L_AD_(L_AD), L_AB_(L_AB), L_BC_(L_BC), L_CD_(L_CD),
          theta_A0_(theta_A0_deg * radians_per_degree), branch_(branch),
          min_distance_(std::abs(L_BC - L_CD)), max_distance_(L_BC + L_CD),
          tolerance_(1e-10 * std::max({1.0, L_AD, L_AB, L_BC, L_CD})),
          BC_squared_(L_BC * L_BC), CD_squared_(L_CD * L_CD) {
        for (double length : {L_AD, L_AB, L_BC, L_CD}) {
            if (!std::isfinite(length) || length <= 0.0) {
                throw std::invalid_argument("所有杆长都必须是大于 0 的有限数值");
            }
        }
        if (!std::isfinite(theta_A0_deg)) {
            throw std::invalid_argument("theta_A0_deg 必须是有限数值");
        }
        if (branch != -1 && branch != 1) {
            throw std::invalid_argument("branch 只能是 +1 或 -1");
        }
        const Geometry initial = solve_geometry(theta_A0_);
        theta_D0_ = initial.theta_D;
        C0_ = initial.C;
    }

    double solve(double delta_A_deg) const {
        return solve_delta(delta_A_deg).second * degrees_per_radian;
    }

    py::dict solve_full(double delta_A_deg) const {
        const auto solved = solve_delta(delta_A_deg);
        const Geometry& current = solved.first;
        py::dict result;
        result["delta_A_deg"] = delta_A_deg;
        result["delta_D_deg"] = solved.second * degrees_per_radian;
        result["theta_A_deg"] = current.theta_A * degrees_per_radian;
        result["theta_D_deg"] = current.theta_D * degrees_per_radian;
        result["theta_A0_deg"] = theta_A0_ * degrees_per_radian;
        result["theta_D0_deg"] = theta_D0_ * degrees_per_radian;
        result["A"] = py::make_tuple(0.0, 0.0);
        result["B"] = py::make_tuple(current.B[0], current.B[1]);
        result["C"] = py::make_tuple(current.C[0], current.C[1]);
        result["D"] = py::make_tuple(L_AD_, 0.0);
        result["BD"] = current.distance_BD;
        return result;
    }

    double L_AD() const { return L_AD_; }
    double L_AB() const { return L_AB_; }
    double L_BC() const { return L_BC_; }
    double L_CD() const { return L_CD_; }
    double theta_A0() const { return theta_A0_; }
    double theta_D0() const { return theta_D0_; }
    int branch() const { return branch_; }
    py::tuple C0() const { return py::make_tuple(C0_[0], C0_[1]); }

private:
    Geometry solve_geometry(double theta_A) const {
        const Point B{L_AB_ * std::cos(theta_A), L_AB_ * std::sin(theta_A)};
        const double dx = L_AD_ - B[0];
        const double dy = 0.0 - B[1];
        const double distance_BD = std::hypot(dx, dy);
        if (distance_BD < tolerance_) {
            throw std::invalid_argument("机构处于退化位置：B 与 D 重合");
        }
        if (distance_BD > max_distance_ + tolerance_) {
            throw std::invalid_argument(distance_error(distance_BD, max_distance_, " > BC+CD="));
        }
        if (distance_BD < min_distance_ - tolerance_) {
            throw std::invalid_argument(distance_error(distance_BD, min_distance_, " < |BC-CD|="));
        }
        if (std::isinf(BC_squared_) || std::isinf(CD_squared_)) {
            throw std::overflow_error("(34, 'Numerical result out of range')");
        }
        const double unit_x = dx / distance_BD;
        const double unit_y = dy / distance_BD;
        const double chord_offset =
            (BC_squared_ - CD_squared_ + square(distance_BD)) / (2.0 * distance_BD);
        const double half_chord_squared = BC_squared_ - square(chord_offset);
        if (half_chord_squared < -tolerance_) {
            throw std::invalid_argument("四连杆无法闭合");
        }
        // Retain the original tolerance for roundoff at a tangent intersection.
        const double half_chord = std::sqrt(std::max(0.0, half_chord_squared));
        const Point midpoint{B[0] + chord_offset * unit_x, B[1] + chord_offset * unit_y};
        const Point C{midpoint[0] - branch_ * unit_y * half_chord,
                      midpoint[1] + branch_ * unit_x * half_chord};
        const double theta_D = std::atan2(C[1] - 0.0, C[0] - L_AD_);
        return {B, C, theta_A, theta_D, distance_BD};
    }

    std::pair<Geometry, double> solve_delta(double delta_A_deg) const {
        if (!std::isfinite(delta_A_deg)) {
            throw std::invalid_argument("delta_A_deg 必须是有限数值");
        }
        const Geometry current = solve_geometry(theta_A0_ + delta_A_deg * radians_per_degree);
        double wrapped = std::fmod(current.theta_D - theta_D0_ + pi, 2.0 * pi);
        if (wrapped < 0.0) {
            wrapped += 2.0 * pi;
        }
        return {current, wrapped - pi};
    }

    double L_AD_;
    double L_AB_;
    double L_BC_;
    double L_CD_;
    double theta_A0_;
    int branch_;
    double min_distance_;
    double max_distance_;
    double tolerance_;
    double BC_squared_;
    double CD_squared_;
    double theta_D0_;
    Point C0_;
};

using ChannelDirection = std::pair<int, int>;
using LinkageGroup = std::tuple<std::string, std::vector<ChannelDirection>, FourBarLinkage>;

class EncoderKinematics {
public:
    explicit EncoderKinematics(std::vector<LinkageGroup> groups) : groups_(std::move(groups)) {
        for (const auto& group : groups_) {
            for (const auto& channel : std::get<1>(group)) {
                if (channel.first < 0 || channel.first >= static_cast<int>(channel_count)) {
                    throw std::invalid_argument("编码器通道索引超出范围");
                }
                if (channel.second != -1 && channel.second != 1) {
                    throw std::invalid_argument("输入方向只能是 +1 或 -1");
                }
            }
        }
    }

    py::list convert_angles(const py::sequence& angles) const {
        if (py::len(angles) != channel_count) {
            throw std::invalid_argument("编码器帧必须包含 21 个角度");
        }
        std::array<double, channel_count> converted;
        for (std::size_t index = 0; index < channel_count; ++index) {
            converted[index] = py::float_(py::reinterpret_borrow<py::object>(angles[index])).cast<double>();
        }
        for (double value : converted) {
            if (!std::isfinite(value)) {
                throw std::invalid_argument("编码器角度必须是有限数值");
            }
        }
        // The GIL stays held: no Python callbacks occur in the mechanical loop,
        // and the small fixed-size frame needs no shared mutable native state.
        for (const auto& group : groups_) {
            for (const auto& channel : std::get<1>(group)) {
                try {
                    converted[channel.first] = std::get<2>(group).solve(
                        channel.second * converted[channel.first]);
                } catch (const std::invalid_argument& error) {
                    const std::string context = std::get<0>(group) + " 的 J" +
                        std::to_string(channel.first + 1) + " 转换失败：" + error.what();
                    PyErr_SetString(PyExc_ValueError, error.what());
                    py::error_already_set cause;
                    py::raise_from(cause, PyExc_ValueError, context.c_str());
                    throw py::error_already_set();
                }
            }
        }
        py::list result(channel_count);
        for (std::size_t index = 0; index < channel_count; ++index) {
            result[index] = converted[index];
        }
        return result;
    }

private:
    const std::vector<LinkageGroup> groups_;
};

}  // namespace

void bind_encoder_kinematics(py::module_& module) {
    py::class_<FourBarLinkage>(module, "FourBarLinkage")
        .def(py::init<double, double, double, double, double, int>(),
             py::arg("L_AD"), py::arg("L_AB"), py::arg("L_BC"), py::arg("L_CD"),
             py::arg("theta_A0_deg"), py::arg("branch") = 1)
        .def("solve", &FourBarLinkage::solve, py::arg("delta_A_deg"))
        .def("solve_full", &FourBarLinkage::solve_full, py::arg("delta_A_deg"))
        .def_property_readonly("L_AD", &FourBarLinkage::L_AD)
        .def_property_readonly("L_AB", &FourBarLinkage::L_AB)
        .def_property_readonly("L_BC", &FourBarLinkage::L_BC)
        .def_property_readonly("L_CD", &FourBarLinkage::L_CD)
        .def_property_readonly("theta_A0", &FourBarLinkage::theta_A0)
        .def_property_readonly("theta_D0", &FourBarLinkage::theta_D0)
        .def_property_readonly("branch", &FourBarLinkage::branch)
        .def_property_readonly("C0", &FourBarLinkage::C0);
    py::class_<EncoderKinematics>(module, "EncoderKinematics")
        .def(py::init<std::vector<LinkageGroup>>(), py::arg("groups"))
        .def("convert_angles", &EncoderKinematics::convert_angles, py::arg("encoder_angles_deg"));
}
