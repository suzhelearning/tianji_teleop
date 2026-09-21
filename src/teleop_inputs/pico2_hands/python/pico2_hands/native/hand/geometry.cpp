// Stateless official wrist-frame preprocessing. No SDK or command authority.
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

extern "C" int tianji_hand_geometry_abi() { return 1; }

// config: input signs(3), row-major rotation(9), wrist offset metres(3),
// thumb offset metres(3). Commit output only after complete validation.
extern "C" int tianji_hand_geometry_prepare(const double* input, std::size_t count,
    const double* config, std::size_t config_count, int left, double* output) noexcept {
    if (!input || !config || !output || count != 63 || config_count != 18 ||
        (left != 0 && left != 1)) return -1;
    try {
        for (std::size_t i=0; i<count; ++i) if (!std::isfinite(input[i])) return -1;
        for (std::size_t i=0; i<config_count; ++i) if (!std::isfinite(config[i])) return -1;
        for (int i=0; i<3; ++i) if (std::abs(config[i]) != 1.) return -1;
        using Points = Eigen::Matrix<double,21,3,Eigen::RowMajor>;
        using Rotation = Eigen::Matrix<double,3,3,Eigen::RowMajor>;
        const Eigen::Map<const Rotation> rotation(config+3);
        if (!(rotation*rotation.transpose()).isApprox(Eigen::Matrix3d::Identity(), 1e-10) ||
            std::abs(rotation.determinant()-1.) > 1e-10) return -1;
        Points points = Eigen::Map<const Points>(input);
        points.array().rowwise() *= Eigen::Map<const Eigen::RowVector3d>(config).array();
        const Eigen::RowVector3d origin = points.row(0);
        points.rowwise() -= origin;
        Eigen::Matrix3d plane;
        plane.row(0)=points.row(0); plane.row(1)=points.row(5); plane.row(2)=points.row(9);
        const Eigen::Vector3d x_vector = (plane.row(0)-plane.row(2)).transpose();
        const Eigen::RowVector3d mean=plane.colwise().mean();
        plane.rowwise() -= mean;
        if (!plane.allFinite()) return -1;
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(plane, Eigen::ComputeFullV);
        // A collinear/coincident wrist/index/middle triple has no unique frame.
        // Reject instead of passing arbitrary SVD axes into the optimizer.
        const auto singular = svd.singularValues();
        if (singular[0] <= 0 || singular[1] <= 64*std::numeric_limits<double>::epsilon()*singular[0]) return -1;
        Eigen::Vector3d normal=svd.matrixV().col(2);
        Eigen::Vector3d x=x_vector-x_vector.dot(normal)*normal;
        const double norm=x.norm();
        if (!std::isfinite(norm) || norm <= 0) return -1;
        x /= norm;
        Eigen::Vector3d z=x.cross(normal);
        if (z.dot((plane.row(1)-plane.row(2)).transpose()) < 0) {normal=-normal; z=-z;}
        Eigen::Matrix3d frame, mano;
        frame.col(0)=x; frame.col(1)=normal; frame.col(2)=z;
        mano << 0,0,-1, (left ? 1 : -1),0,0, 0,(left ? -1 : 1),0;
        Points result = ((points*frame).eval()*mano).eval();
        result = (result*rotation.transpose()).eval();
        for (int i=1; i<21; ++i)
            result.row(i) += Eigen::Map<const Eigen::RowVector3d>(config+(i<5 ? 15 : 12));
        if (!result.allFinite()) return -1;
        std::copy(result.data(), result.data()+63, output);
        return 0;
    } catch (...) { return -1; }
}
