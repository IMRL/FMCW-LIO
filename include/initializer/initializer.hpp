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

#ifndef INITIALIZER_HPP
#define INITIALIZER_HPP

#include "estimator/filter.hpp"

// fmcw_lio
namespace fmcw_lio {

// LoggerInitialization
class LoggerInitialization {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LoggerInitialization() : is_zero_velocity(false),
                             initialization_time(0.0),
                             imu_number(0) {}

    ~LoggerInitialization() = default;

public:
    bool is_zero_velocity;
    double initialization_time;
    std::size_t imu_number;
    Eigen::Vector3d omg_wb_b_0;
    Eigen::Vector3d v_wb_b_0;
};

// StaticInit
class StaticInit {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit StaticInit(const std::shared_ptr<const Config>& config_ptr);

    ~StaticInit() = default;

    // initialize state, covariance, and map
    [[nodiscard]] std::size_t initializeStateCovMap(const std::shared_ptr<MeasPackLI>& meas_ptr);

    // get state
    [[nodiscard]] std::shared_ptr<StateBase> getState() const noexcept { return state_ptr_; }

    // get covariance
    [[nodiscard]] Eigen::MatrixXd getCovariance() const noexcept { return P_; }

    // get static point cloud map in world frame
    [[nodiscard]] PointCloudType::Ptr getMap() const noexcept { return map_ptr_; }

    // get angular velocity
    [[nodiscard]] static Eigen::Vector3d getAngularVelocity() noexcept { return Eigen::Vector3d::Zero(); }

    // get body velocity
    [[nodiscard]] static Eigen::Vector3d getBodyVelocity() noexcept { return Eigen::Vector3d::Zero(); }

    // get is zero velocity in initialization
    [[nodiscard]] static bool getIsZeroVelocityInit() noexcept { return true; }

    // get poses during initialization
    [[nodiscard]] std::shared_ptr<std::vector<std::shared_ptr<StatePoseQuat>>> getPoses() const noexcept {
        return states_pose_ptr_;
    }

private:
    // add pose and map point
    void addPoseAndMapPoint(const std::shared_ptr<MeasPackLI>& meas_ptr);

private:
    // configuration pointer
    const std::shared_ptr<const Config> config_ptr_;

    // maximum number of IMU data for initialization
    const std::size_t max_imu_num_;
    // number of collected IMU data
    std::size_t imu_num_;

    // mean of collected angular rate
    Eigen::Vector3d angular_rate_mean_;
    // mean of collected specific force
    Eigen::Vector3d specific_force_mean_;

    // pose (during initialization) vector pointer
    std::shared_ptr<std::vector<std::shared_ptr<StatePoseQuat>>> states_pose_ptr_;

    // initialized state
    const std::shared_ptr<StateBase> state_ptr_;
    // initialized covariance
    Eigen::MatrixXd P_;
    // initialized static point cloud map in world frame
    PointCloudType::Ptr map_ptr_;
};

// Initializer
class Initializer {
public:
    explicit Initializer(const std::shared_ptr<const Config>& config_ptr);

    ~Initializer() = default;

    // initialize system: system state, filter and covariance, and map
    [[nodiscard]] bool initializeSystem(const std::shared_ptr<StateBase>& state_ptr,
                                        const std::shared_ptr<Filter>& filter_ptr,
                                        const std::shared_ptr<MeasPackLI>& meas_ptr) const;

    // estimate initial velocity for Free Init method
    void bootstrapFreeInit(const std::shared_ptr<MeasPackLI>& meas_ptr) const;

    // get static point cloud map in world frame
    [[nodiscard]] PointCloudType::Ptr getMap() const noexcept;

    // get pose during initialization
    [[nodiscard]] std::shared_ptr<std::vector<std::shared_ptr<StatePoseQuat>>> getPoses() const noexcept;

public:
    // logger for initialization
    std::shared_ptr<LoggerInitialization> logger_initialization_ptr;

private:
    // configuration pointer
    const std::shared_ptr<const Config> config_ptr_;

    // initialization method pointer
    InitMethodPtrVariant init_method_ptr_;
};

} // namespace fmcw_lio

#endif // INITIALIZER_HPP
