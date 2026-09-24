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

#ifndef OPTIMIZER_BAG_HPP
#define OPTIMIZER_BAG_HPP

#include <ceres/ceres.h>

// fmcw_lio
namespace fmcw_lio {

// FactorAcceleration
class FactorAcceleration {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit FactorAcceleration(Eigen::Matrix3d R_wb,
                                Eigen::Vector3d f_wb_w,
                                Eigen::Vector3d a_wb_w,
                                const double gravity_scale) : R_wb_(std::move(R_wb)),
                                                              f_wb_w_(std::move(f_wb_w)),
                                                              a_wb_w_(std::move(a_wb_w)),
                                                              gravity_scale_(gravity_scale) {}

    // residual
    template <typename T>
    bool operator()(const T* const ba_b,
                    const T* const g_wb_w_unit,
                    T* residual) const {
        // parameter mapping
        const Eigen::Map<const Eigen::Matrix<T, 3, 1>> ba_b_mapped(ba_b);
        const Eigen::Map<const Eigen::Matrix<T, 3, 1>> g_wb_w_unit_mapped(g_wb_w_unit);
        Eigen::Map<Eigen::Matrix<T, 3, 1>> residual_mapped(residual);

        const Eigen::Matrix<T, 3, 1> a_wb_w_est = f_wb_w_.template cast<T>() -
                                                  (R_wb_.template cast<T>() * ba_b_mapped) +
                                                  (T(gravity_scale_) * g_wb_w_unit_mapped);
        const auto a_wb_w_T = a_wb_w_.template cast<T>();

        residual_mapped = a_wb_w_T - a_wb_w_est;

        return true;
    }

private:
    Eigen::Matrix3d R_wb_;
    Eigen::Vector3d f_wb_w_;
    Eigen::Vector3d a_wb_w_;
    double gravity_scale_;
};

// RegularizationAccelerometerBias
class RegularizationAccelerometerBias {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit RegularizationAccelerometerBias(const double (&ba_b_init)[3],
                                             double weight) : ba_b_init_(ba_b_init[0],
                                                                         ba_b_init[1],
                                                                         ba_b_init[2]),
                                                              weight_(weight) {}

    // residual
    template <typename T>
    bool operator()(const T* const ba_b,
                    T* residual) const {
        // parameter mapping
        const Eigen::Map<const Eigen::Matrix<T, 3, 1>> ba_b_mapped(ba_b);
        Eigen::Map<Eigen::Matrix<T, 3, 1>> residual_mapped(residual);

        const Eigen::Matrix<T, 3, 1> ba_b_init_T = ba_b_init_.template cast<T>();

        residual_mapped = T(weight_) * (ba_b_mapped - ba_b_init_T);

        return true;
    }

private:
    Eigen::Vector3d ba_b_init_;
    double weight_;
};

// OptimizerBaG
class OptimizerBaG {
public:
    explicit OptimizerBaG(const std::shared_ptr<const Config>& config_ptr,
                          const Eigen::Vector3d& ba_b = Eigen::Vector3d::Zero(),
                          const Eigen::Vector3d& g_wb_w = Eigen::Vector3d(0.0, 0.0, -9.81)) : config_ptr_(config_ptr),
                                                                                              loss_function_ptr_{nullptr},
                                                                                              accelerometer_bias_regularization_id_{nullptr},
                                                                                              ba_b_{},
                                                                                              g_wb_w_unit_{} {
        // set optimization option
        optimization_options_.minimizer_type = ceres::TRUST_REGION;
        optimization_options_.trust_region_strategy_type = ceres::DOGLEG;
        optimization_options_.linear_solver_type = ceres::DENSE_QR;
        optimization_options_.num_threads = static_cast<int>(config_ptr_->thread_num);
        optimization_options_.max_num_iterations = static_cast<int>(config_ptr_->max_iteration_num);
        optimization_options_.minimizer_progress_to_stdout = false;
        optimization_options_.max_solver_time_in_seconds = config_ptr_->max_solver_time_in_seconds;

        // set loss function
        loss_function_ptr_ = std::unique_ptr<ceres::LossFunction>(new ceres::CauchyLoss(config_ptr_->cauchy_loss_scale));

        // add parameter block
        problem_.AddParameterBlock(ba_b_, 3);
        problem_.AddParameterBlock(g_wb_w_unit_, 3);

        // S^2 manifold constrain for local gravity direction
        problem_.SetParameterization(g_wb_w_unit_,
                                     new ceres::HomogeneousVectorParameterization(3));

        // set optimization variable
        ba_b_[0] = ba_b.x();
        ba_b_[1] = ba_b.y();
        ba_b_[2] = ba_b.z();

        Eigen::Vector3d g_wb_w_unit = g_wb_w.normalized();
        g_wb_w_unit_[0] = g_wb_w_unit.x();
        g_wb_w_unit_[1] = g_wb_w_unit.y();
        g_wb_w_unit_[2] = g_wb_w_unit.z();
    }

    // add acceleration factor
    void addFactorAcceleration(const Eigen::Matrix3d& R_wb,
                               const Eigen::Vector3d& f_wb_w,
                               const Eigen::Vector3d& a_wb_w) {
        // cost function
        ceres::CostFunction* factor_acceleration =
                new ceres::AutoDiffCostFunction<FactorAcceleration, 3, 3, 3>(
                        new FactorAcceleration(R_wb,
                                               f_wb_w,
                                               a_wb_w,
                                               config_ptr_->gravity_scale));

        // add residual block
        problem_.AddResidualBlock(factor_acceleration,
                                  loss_function_ptr_.get(),
                                  ba_b_,
                                  g_wb_w_unit_);
    }

    // solve optimization problem
    void solveOptimization() {
        // add accelerometer bias regularization only once
        if (accelerometer_bias_regularization_id_ == nullptr) {
            ceres::CostFunction* accelerometer_bias_regularization =
                    new ceres::AutoDiffCostFunction<RegularizationAccelerometerBias, 3, 3>(
                            new RegularizationAccelerometerBias(ba_b_,
                                                                config_ptr_->accelerometer_bias_regularization_weight));

            // add residual block
            accelerometer_bias_regularization_id_ = problem_.AddResidualBlock(accelerometer_bias_regularization,
                                                                              nullptr,
                                                                              ba_b_);
        }

        // solve optimization
        ceres::Solver::Summary summary;
        ceres::Solve(optimization_options_, &problem_,
                     &summary);
    }

    // set initial value for optimization variable
    void setInitialValue(const Eigen::Vector3d& ba_b,
                         const Eigen::Vector3d& g_wb_w) {
        // set initial value
        ba_b_[0] = ba_b.x();
        ba_b_[1] = ba_b.y();
        ba_b_[2] = ba_b.z();

        Eigen::Vector3d g_wb_w_unit = g_wb_w.normalized();
        g_wb_w_unit_[0] = g_wb_w_unit.x();
        g_wb_w_unit_[1] = g_wb_w_unit.y();
        g_wb_w_unit_[2] = g_wb_w_unit.z();
    }

    // get accelerometer bias
    [[nodiscard]] Eigen::Vector3d getAccelerometerBias() const {
        return {ba_b_[0],
                ba_b_[1],
                ba_b_[2]};
    }

    // get gravity
    [[nodiscard]] Eigen::Vector3d getGravity() const {
        return (config_ptr_->gravity_scale * Eigen::Vector3d(g_wb_w_unit_[0],
                                                             g_wb_w_unit_[1],
                                                             g_wb_w_unit_[2]));
    }

private:
    // configuration pointer
    const std::shared_ptr<const Config> config_ptr_;

    // optimization option configuration
    ceres::Solver::Options optimization_options_;
    // optimization loss function
    std::unique_ptr<ceres::LossFunction> loss_function_ptr_;
    // optimization problem
    ceres::Problem problem_;

    // ID to avoid adding the same regularization multiple times
    ceres::ResidualBlockId accelerometer_bias_regularization_id_;

    // optimization variable
    // accelerometer bias
    double ba_b_[3];
    // gravity direction
    double g_wb_w_unit_[3];
};

} // namespace fmcw_lio

#endif // OPTIMIZER_BAG_HPP
