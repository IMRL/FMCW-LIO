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

#include "initializer/initializer.hpp"

#ifdef BUILD_FREE_INIT
#include "initializer/free_init/free_init.hpp"
#endif

// fmcw_lio
namespace fmcw_lio {

// StaticInit
StaticInit::StaticInit(const std::shared_ptr<const Config>& config_ptr) : config_ptr_(config_ptr),
                                                                          max_imu_num_(config_ptr_->max_imu_num),
                                                                          imu_num_(0),
                                                                          angular_rate_mean_(Eigen::Vector3d::Zero()),
                                                                          specific_force_mean_(Eigen::Vector3d::Zero()),
                                                                          states_pose_ptr_(std::make_shared<std::vector<std::shared_ptr<StatePoseQuat>>>()),
                                                                          state_ptr_(std::make_shared<StateSystem>(config_ptr_)) {
    // initialize point cloud
    map_ptr_.reset(new PointCloudType());
}

std::size_t StaticInit::initializeStateCovMap(const std::shared_ptr<MeasPackLI>& meas_ptr) {
    // IMU data for initialization
    for (const auto& imu : meas_ptr->imu_msg_ptr_queue) {
        if (imu == meas_ptr->imu_msg_ptr_queue.back()) {
            break;
        }
        imu_num_ += 1;

        Eigen::Vector3d curr_angular_rate{imu->angular_velocity.x,
                                          imu->angular_velocity.y,
                                          imu->angular_velocity.z};
        Eigen::Vector3d curr_specific_force{imu->linear_acceleration.x,
                                            imu->linear_acceleration.y,
                                            imu->linear_acceleration.z};

        angular_rate_mean_ += (curr_angular_rate - angular_rate_mean_) / imu_num_;
        specific_force_mean_ += (curr_specific_force - specific_force_mean_) / imu_num_;
    }

    if (imu_num_ > max_imu_num_) {
        Eigen::Matrix3d R_wb_0 = Eigen::Matrix3d::Identity();
        Eigen::Vector3d v_wb_w_0 = Eigen::Vector3d::Zero();
        Eigen::Vector3d p_wb_w_0 = Eigen::Vector3d::Zero();

        Eigen::Vector3d bg_b_0 = angular_rate_mean_;

        Eigen::Vector3d unit_mean_acc = specific_force_mean_.normalized();
        Eigen::Vector3d g_wb_w_0 = -config_ptr_->gravity_scale * unit_mean_acc;

        Eigen::Vector3d ba_b_0;
        if (config_ptr_->init_method == InitializationMethod::StaticWithoutBias) {
            ba_b_0 = Eigen::Vector3d::Zero();
        } else if (config_ptr_->init_method == InitializationMethod::StaticWithBias) {
            ba_b_0 = specific_force_mean_ + g_wb_w_0;
        }

        // set state and state time
        state_ptr_->setState(R_wb_0,
                             v_wb_w_0,
                             p_wb_w_0,
                             bg_b_0,
                             ba_b_0,
                             g_wb_w_0);
        state_ptr_->setTime(meas_ptr->scan_end_time);

        // set covariance
        const auto dim_state = static_cast<Eigen::Index>(state_ptr_->getDim());
        P_ = Eigen::MatrixXd::Identity(dim_state, dim_state);
        // initial rotation covariance
        P_.block<3, 3>(0, 0) *= 1.0e-5;
        // initial velocity covariance
        P_.block<3, 3>(3, 3) *= 1.0e-5;
        // initial position covariance
        P_.block<3, 3>(6, 6) *= 1.0e-5;
        // initial gyroscope bias covariance
        P_.block<3, 3>(9, 9) *= 1.0e-4;
        // initial accelerometer bias covariance
        P_.block<3, 3>(12, 12) *= 1.0e-3;
        // initial gravity covariance
        P_.block<3, 3>(15, 15) *= 1.0e-5;
    } 

    // store pose and map point during initialization
    addPoseAndMapPoint(meas_ptr);

    return imu_num_;
}

void StaticInit::addPoseAndMapPoint(const std::shared_ptr<MeasPackLI>& meas_ptr) {
    // store poses during initialization
    auto state_pose = std::make_shared<StatePoseQuat>(meas_ptr->scan_end_time,
                                                      Eigen::Quaterniond::Identity(),
                                                      Eigen::Vector3d::Zero());
    states_pose_ptr_->push_back(state_pose);

    std::size_t map_size = map_ptr_->size();
    std::size_t scan_size = meas_ptr->scan_ptr->size();

    map_ptr_->resize(map_size + scan_size);

    // initial map points in world frame
    std::vector<size_t> point_indices(scan_size);
    std::for_each(point_indices.begin(), point_indices.end(), [n = 0](std::size_t& idx) mutable -> void { idx = n++; });
    std::for_each(std::execution::par,
                  point_indices.begin(), point_indices.end(),
                  [&](const auto point_idx) -> void {
        // scan point in LiDAR frame
        const auto& point = meas_ptr->scan_ptr->points[point_idx];
        Eigen::Vector3d p_lp_l{point.x, point.y, point.z};
        Eigen::Vector3d p_wp_w = (config_ptr_->R_bl * p_lp_l) + config_ptr_->p_bl_b;

        map_ptr_->points[map_size + point_idx].curvature = point.curvature;
        map_ptr_->points[map_size + point_idx].x = static_cast<float>(p_wp_w.x());
        map_ptr_->points[map_size + point_idx].y = static_cast<float>(p_wp_w.y());
        map_ptr_->points[map_size + point_idx].z = static_cast<float>(p_wp_w.z());
        map_ptr_->points[map_size + point_idx].intensity = point.intensity;
    });
}

void Initializer::bootstrapFreeInit(const std::shared_ptr<MeasPackLI>& meas_ptr) const {
    // bootstrap Free-Init method
    std::visit([&](auto&& init_method_ptr) -> void {
        using T = std::decay_t<decltype(init_method_ptr)>;

        if constexpr (std::is_same_v<T, std::shared_ptr<FreeInit>>) {
            init_method_ptr->bootstrapVelocity(meas_ptr);
        }
    }, init_method_ptr_);
}

// Initializer
Initializer::Initializer(const std::shared_ptr<const Config>& config_ptr) : logger_initialization_ptr(std::make_shared<LoggerInitialization>()),
                                                                            config_ptr_(config_ptr) {
    // set initialization method
    switch (config_ptr_->init_method) {
        case InitializationMethod::StaticWithoutBias:
        case InitializationMethod::StaticWithBias: {
            // StaticInit: static initialization method
            init_method_ptr_ = std::make_shared<StaticInit>(config_ptr_);

            break;
        }

#ifdef BUILD_FREE_INIT
        case InitializationMethod::FreeInit: {
            // FreeInit: Free-Init method
            init_method_ptr_ = std::make_shared<FreeInit>(config_ptr_);

            break;
        }
#endif

        default: {
            // StaticInit: static initialization method
            init_method_ptr_ = std::make_shared<StaticInit>(config_ptr_);

            break;
        }
    }
}

bool Initializer::initializeSystem(const std::shared_ptr<StateBase>& state_ptr,
                                   const std::shared_ptr<Filter>& filter_ptr,
                                   const std::shared_ptr<MeasPackLI>& meas_ptr) const {
    // initialize system via corresponding initialization method
    return std::visit([&](auto&& init_method_ptr) -> bool {
        // start timer
        std::chrono::steady_clock::time_point t1, t2;
        t1 = std::chrono::steady_clock::now();

        // initialize state, covariance, and map
        const auto imu_num = init_method_ptr->initializeStateCovMap(meas_ptr);

        // stop timer
        t2 = std::chrono::steady_clock::now();
        logger_initialization_ptr->initialization_time += static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()) * 1.0e-3;

        if (imu_num > config_ptr_->max_imu_num) {
            logger_initialization_ptr->is_zero_velocity = init_method_ptr->getIsZeroVelocityInit();
            logger_initialization_ptr->imu_number = imu_num;
            logger_initialization_ptr->omg_wb_b_0 = init_method_ptr->getAngularVelocity();
            logger_initialization_ptr->v_wb_b_0 = init_method_ptr->getBodyVelocity();

            const auto init_state_ptr = init_method_ptr->getState();
            // set state and time
            state_ptr->setState(init_state_ptr->getRotation(),
                                init_state_ptr->getVelocity(),
                                init_state_ptr->getPosition(),
                                init_state_ptr->getGyroscopeBias(),
                                init_state_ptr->getAccelerometerBias(),
                                init_state_ptr->getGravity());
            state_ptr->setTime(meas_ptr->scan_end_time);

            // initialize filter and covariance
            filter_ptr->initializeFilter(state_ptr,
                                         init_method_ptr->getCovariance(),
                                         meas_ptr);

            return true;
        } else {
            // set time
            state_ptr->setTime(meas_ptr->scan_end_time);

            return false;
        }
    }, init_method_ptr_);
}

PointCloudType::Ptr Initializer::getMap() const noexcept {
    // get static point cloud in world frame
    return std::visit([&](auto&& init_method_ptr) -> PointCloudType::Ptr {
        return init_method_ptr->getMap();
    }, init_method_ptr_);
}

std::shared_ptr<std::vector<std::shared_ptr<StatePoseQuat>>> Initializer::getPoses() const noexcept {
    // get poses during initialization
    return std::visit([&](auto&& init_method_ptr) -> std::shared_ptr<std::vector<std::shared_ptr<StatePoseQuat>>> {
        return init_method_ptr->getPoses();
    }, init_method_ptr_);
}

} // namespace fmcw_lio
