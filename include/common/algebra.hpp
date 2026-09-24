/******************************************************************************
 * Copyright (c) 2023-2026,
 * The Intelligent Machine Research Lab (IMRL),
 * University of Macau
 *
 * License: MIT License
 *
 * Publication: M. Zhao, J. Wang, T. Gao, C. Xu and H. Kong,
 *              "FMCW-LIO: A Doppler LiDAR-Inertial Odometry,"
 *              IEEE Robotics and Automation Letters, 2024,
 *              DOI: 10.1109/LRA.2024.3396636.
 *
 *              M. Zhao, J. Wang, T. Gao, C. Xu and H. Kong,
 *              "Free-Init: Scan-Free, Motion-Free, and
 *              Correspondence-Free Initialization for
 *              Doppler LiDAR-Inertial Systems,"
 *              IEEE Robotics and Automation Letters, 2024,
 *              DOI: 10.1109/LRA.2024.3490395.
 *
 * IEEE Xplore Link: https://ieeexplore.ieee.org/document/10518074
 *                   https://ieeexplore.ieee.org/document/10740796
 * arXiv Paper Link:
 *
 * Code & Sequence : https://github.com/IMRL/FMCW-LIO
 *                   https://github.com/IMRL/Free-Init
 * Experiment Video: https://youtu.be/2yuZYw91AP8
 *                   https://youtu.be/FbyzvJ-4bHI
 *
 * Citation: @article{zhao2024fmcw-lio,
 *               title={{FMCW-LIO: A Doppler LiDAR-Inertial Odometry}},
 *               author={Zhao, Mingle and Wang, Jiahao and Gao, Tianxiao and
 *                       Xu, Chengzhong and Kong, Hui},
 *               journal={IEEE Robotics and Automation Letters},
 *               volume={9},
 *               number={6},
 *               pages={5727--5734},
 *               year={2024},
 *               publisher={IEEE}
 *           }
 *
 *           @article{zhao2024free-init,
 *               title={{Free-Init: Scan-Free, Motion-Free, and
 *                       Correspondence-Free Initialization for
 *                       Doppler LiDAR-Inertial Systems}},
 *               author={Zhao, Mingle and Wang, Jiahao and Gao, Tianxiao and
 *                       Xu, Chengzhong and Kong, Hui},
 *               journal={IEEE Robotics and Automation Letters},
 *               volume={9},
 *               number={12},
 *               pages={11329--11336},
 *               year={2024},
 *               publisher={IEEE}
 *           }
 ******************************************************************************/

#ifndef ALGEBRA_HPP
#define ALGEBRA_HPP

#include <Eigen/Dense>

// fmcw_lio
namespace fmcw_lio {

// SO(3)
class SO3 {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SO3() noexcept = default;

    explicit SO3(const Eigen::Quaterniond& q) : q_(q) {
        // normalize via quaternion normalization
        if (std::abs(q_.norm() - 1.0) >= 1.0e-9) {
            q_.normalize();
        }
    }

    explicit SO3(const Eigen::Matrix3d& R) : q_(R) {
        // normalize via quaternion normalization
        if (std::abs(q_.norm() - 1.0) >= 1.0e-9) {
            q_.normalize();
        }
    }

    ~SO3() = default;

    // SO(3) static function
    // skewVec: R^3 -> so(3)
    static Eigen::Matrix3d skewVec(const Eigen::Vector3d& phi) {
        Eigen::Matrix3d A;
        A <<      0.0, -phi.z(),  phi.y(),
              phi.z(),      0.0, -phi.x(),
             -phi.y(),  phi.x(),      0.0;

        return A;
    }

    // Exp: R^3 -> SO(3)
    static SO3 Exp(const Eigen::Vector3d& phi) {
        const auto phi_norm_half = 0.5 * phi.norm();
        const auto [q_w, q_vec] = ((phi_norm_half >= 1.0e-9) ?
                                   std::pair{std::cos(phi_norm_half), (std::sin(phi_norm_half) * phi.normalized()).eval()} :
                                   std::pair{1.0, (0.5 * phi).eval()});

        Eigen::Quaterniond q{q_w, q_vec.x(), q_vec.y(), q_vec.z()};
        q.normalize();

        return SO3(q);
    }

    // Log: SO(3) -> R^3
    static Eigen::Vector3d Log(const SO3& R) {
        const auto sign = std::copysign(1.0, R.q_.w());
        const auto q_w  = std::abs(R.q_.w());
        const auto q_vec = (sign * R.q_.vec()).eval();

        const auto vec_norm = q_vec.norm();
        Eigen::Vector3d phi = ((vec_norm >= 1.0e-9) ?
                               (2.0 * std::atan2(vec_norm, q_w) * (q_vec / vec_norm)).eval() :
                               ((2.0 / q_w) * q_vec * (1.0 - (((vec_norm / q_w) * (vec_norm / q_w)) / 3.0))).eval());

        return phi;
    }

    // Jl: R^3 -> R^{3 x 3}
    static Eigen::Matrix3d Jl(const Eigen::Vector3d& phi) {
        const auto phi_norm = phi.norm();

        return ((phi_norm >= 1.0e-9) ?
                ([&]() -> Eigen::Matrix3d {
                    const auto n = (phi / phi_norm).eval();

                    return  (((std::sin(phi_norm) / phi_norm) * Eigen::Matrix3d::Identity()) +
                             (((1.0 - std::cos(phi_norm)) / phi_norm) * skewVec(n)) +
                             ((1.0 - (std::sin(phi_norm) / phi_norm)) * n * n.transpose())).eval();
                }()) :
                (Eigen::Matrix3d::Identity() + (0.5 * skewVec(phi))).eval());
    }

    // SO(3) member function
    // quaternion form
    [[nodiscard]] Eigen::Quaterniond getQuat() const noexcept { return q_; }

    // rotation matrix form
    [[nodiscard]] Eigen::Matrix3d getMat() const noexcept { return q_.toRotationMatrix(); }

    // inverse SO(3) element
    [[nodiscard]] SO3 getInv() const noexcept { return SO3(q_.inverse()); }

    // overloaded operator
    // right * with an SO(3) element
    inline SO3 operator*(const SO3& R) const noexcept { return SO3(q_ * R.q_); }

    // right * with a rotation matrix
    inline SO3 operator*(const Eigen::Matrix3d& R_mat) const { return SO3(q_ * Eigen::Quaterniond(R_mat).normalized()); }

    // right * with a quaternion element
    inline SO3 operator*(const Eigen::Quaterniond& q) const { return SO3(q_ * q.normalized()); }

    // right * with a 3D vector
    inline Eigen::Vector3d operator*(const Eigen::Vector3d& v) const noexcept { return q_._transformVector(v); }

private:
    // quaternion
    Eigen::Quaterniond q_;
};

// non-member function
// left * with a rotation matrix
inline SO3 operator*(const Eigen::Matrix3d& R_mat_l, const SO3& R_r) noexcept { return (SO3(R_mat_l) * R_r); }

// left * with a quaternion element
inline SO3 operator*(const Eigen::Quaterniond& q_l, const SO3& R_r) noexcept { return (SO3(q_l) * R_r); }

} // namespace fmcw_lio

#endif // ALGEBRA_HPP
