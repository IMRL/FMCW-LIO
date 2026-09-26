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
 * arXiv Paper Link: https://arxiv.org/abs/2609.29374
 *                   https://arxiv.org/abs/2609.29375
 * Code & Dataset  : https://github.com/IMRL/FMCW-LIO
 *                   https://github.com/IMRL/Free-Init
 * Experiment Video: https://youtu.be/2yuZYw91AP8
 *                   https://youtu.be/FbyzvJ-4bHI
 *
 * Citation: @article{zhao2024fmcw-lio,
 *               title={FMCW-LIO: A Doppler LiDAR-Inertial Odometry},
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
 *               title={Free-Init: Scan-Free, Motion-Free, and
 *                      Correspondence-Free Initialization for
 *                      Doppler LiDAR-Inertial Systems},
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

#ifndef PROPAGATOR_HPP
#define PROPAGATOR_HPP

#include "bithub/bithub.hpp"
#include "compensator/compensator.hpp"

// fmcw_lio
namespace fmcw_lio {

// LoggerPropagation
class LoggerPropagation {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double propagation_time{};
    Eigen::VectorXd delta;
};

// Propagator
class Propagator {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit Propagator(const std::shared_ptr<const Config>& config_ptr);

    // initialize propagator
    void initializePropagator(const std::shared_ptr<StateBase>& state_ptr,
                              const std::shared_ptr<MeasPackLI>& meas_ptr);

    // propagate state and covariance using IMU data
    void propagateStateAndCov(const std::shared_ptr<StateBase>& state_ptr, Eigen::MatrixXd& P,
                              const std::shared_ptr<MeasPackLI>& meas_ptr,
                              const std::shared_ptr<const BitHub>& bithub_ptr);

    // get IMU at scan end time: (angular rate, specific force)
    [[nodiscard]] std::pair<Eigen::Vector3d, Eigen::Vector3d> getIMUScanEnd() const noexcept {
        return std::make_pair(omg_wb_b_meas_scan_end_, f_wb_b_meas_scan_end_);
    }

    // get compensated scan in LiDAR frame
    [[nodiscard]] PointCloudType::Ptr getScanCompensatedInLiDAR() const noexcept {
        return scan_compensated_lidar_ptr_;
    }

    // get compensation logger
    [[nodiscard]] std::shared_ptr<LoggerCompensation> getLoggerCompensation() const noexcept {
        return compensator_ptr_->logger_compensation_ptr;
    }

private:
    // propagate nominal state
    void propagateNominalState(const std::shared_ptr<StateBase>& state_ptr,
                               const Eigen::Vector3d& omg_wb_b_head, const Eigen::Vector3d& f_wb_b_head,
                               const Eigen::Vector3d& omg_wb_b_tail, const Eigen::Vector3d& f_wb_b_tail,
                               double dt);

    // propagate covariance
    void propagateCovariance(const std::shared_ptr<StateBase>& state_ptr, Eigen::MatrixXd& P,
                             const Eigen::Vector3d& omg_wb_b_head, const Eigen::Vector3d& f_wb_b_head,
                             const Eigen::Vector3d& omg_wb_b_tail, const Eigen::Vector3d& f_wb_b_tail,
                             double dt);

    // state transition matrix
    static Eigen::MatrixXd buildStateMatrix(const std::shared_ptr<StateBase>& state_ptr,
                                            const Eigen::Vector3d& omg_wb_b_unbiased, const Eigen::Vector3d& f_wb_b_unbiased,
                                            double dt);

    // input matrix
    static Eigen::MatrixXd buildInputMatrix(const std::shared_ptr<StateBase>& state_ptr,
                                            const Eigen::Vector3d& omg_wb_b_unbiased, const Eigen::Vector3d& f_wb_b_unbiased,
                                            double dt);

    // propagate nominal state using 2-nd order Runge-Kutta method
    static Eigen::VectorXd integrateRungeKutta2(const std::shared_ptr<const StateBase>& state_ptr,
                                                const Eigen::Vector3d& omg_wb_b_head_unbiased, const Eigen::Vector3d& f_wb_b_head_unbiased,
                                                const Eigen::Vector3d& omg_wb_b_tail_unbiased, const Eigen::Vector3d& f_wb_b_tail_unbiased,
                                                double dt);

    // propagate nominal state using 3-rd order Runge-Kutta method
    static Eigen::VectorXd integrateRungeKutta3(const std::shared_ptr<const StateBase>& state_ptr,
                                                const Eigen::Vector3d& omg_wb_b_head_unbiased, const Eigen::Vector3d& f_wb_b_head_unbiased,
                                                const Eigen::Vector3d& omg_wb_b_tail_unbiased, const Eigen::Vector3d& f_wb_b_tail_unbiased,
                                                double dt);

    // propagate nominal state using 4-th order Runge-Kutta method
    static Eigen::VectorXd integrateRungeKutta4(const std::shared_ptr<const StateBase>& state_ptr,
                                                const Eigen::Vector3d& omg_wb_b_head_unbiased, const Eigen::Vector3d& f_wb_b_head_unbiased,
                                                const Eigen::Vector3d& omg_wb_b_tail_unbiased, const Eigen::Vector3d& f_wb_b_tail_unbiased,
                                                double dt);

    // propagate nominal state using two-sample mechanization method
    static Eigen::VectorXd integrateMechanization(const std::shared_ptr<const StateBase>& state_ptr,
                                                  const Eigen::Vector3d& omg_wb_b_head_unbiased, const Eigen::Vector3d& f_wb_b_head_unbiased,
                                                  const Eigen::Vector3d& omg_wb_b_tail_unbiased, const Eigen::Vector3d& f_wb_b_tail_unbiased,
                                                  double dt);

    // propagate nominal state using analytic combined IMU integration (ACI2) method:
    // Yang et al., "Analytic Combined IMU Integration (ACI2) For Visual Inertial Navigation.", IEEE ICRA, 2020.
    static Eigen::VectorXd integrateACI2(const std::shared_ptr<const StateBase>& state_ptr,
                                         const Eigen::Vector3d& omg_wb_b_head_unbiased, const Eigen::Vector3d& f_wb_b_head_unbiased,
                                         const Eigen::Vector3d& omg_wb_b_tail_unbiased, const Eigen::Vector3d& f_wb_b_tail_unbiased,
                                         double dt);

    // preintegrated IMU measurement Jacobian matrix for ACI2 method
    static void assembleXiMatrixPack(Eigen::Matrix<double, 3, 6>& Xi_packed,
                                     const Eigen::Vector3d& omg_wb_b_avg_unbiased,
                                     double dt);

private:
    // integration method and integrate function table
    inline static const IntegrationTable integration_table_ = {
        // 2-nd order Runge-Kutta method
        {
            IntegrationMethod::RungeKutta2,
            &integrateRungeKutta2
        },
        // 3-th order Runge-Kutta method
        {
            IntegrationMethod::RungeKutta3,
            &integrateRungeKutta3
        },
        // 4-th order Runge-Kutta method
        {
            IntegrationMethod::RungeKutta4,
            &integrateRungeKutta4
        },
        // mechanization method
        {
            IntegrationMethod::Mechanization,
            &integrateMechanization
        },
        // ACI2 method
        {
            IntegrationMethod::ACI2,
            &integrateACI2
        }
    };

public:
    std::shared_ptr<LoggerPropagation> logger_propagation_ptr;

private:
    // configuration pointer
    const std::shared_ptr<const Config> config_ptr_;

    // compensator pointer
    const std::shared_ptr<Compensator> compensator_ptr_;

    // last scan end time
    double last_scan_end_time_;

    // last IMU message pointer
    sensor_msgs::Imu::Ptr last_imu_ptr_;

    // compensated scan in LiDAR frame
    PointCloudType::Ptr scan_compensated_lidar_ptr_;

    // IMU measurement at scan end
    Eigen::Vector3d omg_wb_b_meas_scan_end_;
    Eigen::Vector3d f_wb_b_meas_scan_end_;

    // last unbiased angular rate and specific force
    Eigen::Vector3d omg_wb_b_unbiased_last_;
    Eigen::Vector3d f_wb_b_unbiased_last_;

    // propagation state and measurement for motion compensation
    std::shared_ptr<std::vector<std::shared_ptr<const StateBase>>> states_propagation_ptr_;

    // integrate function
    IntegrateFunction integrate_func_;

    // input covariance matrix
    Eigen::MatrixXd Q_;
};

} // namespace fmcw_lio

#endif // PROPAGATOR_HPP
