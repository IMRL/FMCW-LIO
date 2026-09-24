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

#include "system/fmcw_lio.hpp"

// fmcw_lio
namespace fmcw_lio {

// FMCWLIO
FMCWLIO::FMCWLIO(const std::shared_ptr<Config>& config_ptr) : config_ptr_(config_ptr),
                                                              state_system_ptr_(std::make_shared<StateSystem>(config_ptr_)),
                                                              initializer_ptr_(std::make_shared<Initializer>(config_ptr_)),
                                                              filter_ptr_(std::make_shared<Filter>(config_ptr_)),
                                                              map_ptr_(std::make_shared<Map>(config_ptr_)),
                                                              velocimeter_ptr_(std::make_shared<Velocimeter>(config_ptr_)),
                                                              is_init_success_(false),
                                                              is_first_scan_sys_(true),
                                                              time_first_scan_sys_{},
                                                              is_first_scan_lio_(true),
                                                              vel_update_time_ratio_{0.1},
                                                              correspondence_num_{},
                                                              time_processing_{},
                                                              correspond_num_thresh_mapping_4d_(state_system_ptr_->getDim()) {
    // initialize point clouds
    map_init_ptr_.reset(new PointCloudType());
    scan_compensated_lidar_ptr_.reset(new PointCloudType());
    scan_down_lidar_ptr_.reset(new PointCloudType());
    scan_dynamic_lidar_ptr_.reset(new PointCloudType());
    scan_down_world_ptr_.reset(new PointCloudType());

    // set voxel filter
    voxel_filter_scan_.setLeafSize(static_cast<float>(config_ptr_->voxel_filter_size_scan),
                                   static_cast<float>(config_ptr_->voxel_filter_size_scan),
                                   static_cast<float>(config_ptr_->voxel_filter_size_scan));

    vel_update_time_diff_thresh_ = vel_update_time_ratio_ / config_ptr_->scan_rate;
}

void FMCWLIO::evolveSystem(const std::shared_ptr<MeasPackLI>& meas_ptr) {
    // FMCW-LIO system evolves
    if (is_first_scan_sys_) {
        initializer_ptr_->bootstrapFreeInit(meas_ptr);
        is_first_scan_sys_ = false;
        state_system_ptr_->setTime(meas_ptr->scan_end_time);
        time_first_scan_sys_ = meas_ptr->scan_beg_time;

        return;
    }

    scan_compensated_lidar_ptr_->clear();
    scan_down_lidar_ptr_->clear();
    scan_down_world_ptr_->clear();
    scan_dynamic_lidar_ptr_->clear();

    // start timer
    std::chrono::steady_clock::time_point t1, t2;
    t1 = std::chrono::steady_clock::now();

    // dial forward LiDAR and IMU measurement
    dialForwardMeasLI(meas_ptr);

    // initialization
    if (!is_init_success_) {
        is_init_success_ = initializer_ptr_->initializeSystem(state_system_ptr_,
                                                              filter_ptr_,
                                                              meas_ptr);

        if (is_init_success_) {
            // switch point select interval between initialization and LIO phase
            config_ptr_->point_select_interval = config_ptr_->point_select_interval_lio;

            // initialization mapping
            map_init_ptr_ = initializer_ptr_->getMap();
            if (config_ptr_->init_mapping &&
                !map_init_ptr_->points.empty() &&
                map_ptr_->isNeedInitializeMap()) {
                // initialize map
                map_ptr_->initializeMap(state_system_ptr_,
                                        map_init_ptr_,
                                        true);
            }
        }

        return;
    }

    // filter propagation
    filter_ptr_->propagateFilter(state_system_ptr_,
                                 meas_ptr);

    // motion compensation for LiDAR scan
    scan_compensated_lidar_ptr_ = filter_ptr_->getScanCompensatedInLiDAR();

    // update local map boundary
    map_ptr_->updateMapBoundary(state_system_ptr_);

    // downsample point cloud
    voxel_filter_scan_.setInputCloud(scan_compensated_lidar_ptr_);
    voxel_filter_scan_.filter(*scan_down_lidar_ptr_);
    if (scan_down_lidar_ptr_->points.size() <= 5) {
        return;
    }

    // estimate velocity in LiDAR frame
    bool is_vel_update_first_scan_lio = !(config_ptr_->init_mapping && is_first_scan_lio_);
    if (config_ptr_->use_doppler &&
        is_vel_update_first_scan_lio) {
        auto imu_scan_end = filter_ptr_->getIMUScanEnd();
        auto v_wb_b = (state_system_ptr_->getRotation().transpose() * state_system_ptr_->getVelocity()).eval();
        auto omg_wb_b_unbiased = (imu_scan_end.first - state_system_ptr_->getGyroscopeBias()).eval();

        // predicted velocity in LiDAR frame
        Eigen::Vector3d v_wl_l_prior = config_ptr_->R_bl.transpose() * (v_wb_b + omg_wb_b_unbiased.cross(config_ptr_->p_bl_b));

        // velocity estimation via Doppler
        bool is_vel_success = velocimeter_ptr_->estimateVelocity(scan_down_lidar_ptr_,
                                                                 v_wl_l_prior,
                                                                 config_ptr_->use_ransac);

        // state update based on measured and estimated LiDAR velocity
        if (is_vel_success &&
            (std::abs(velocimeter_ptr_->getTime() - (state_system_ptr_->getTime() - meas_ptr->scan_beg_time)) <= vel_update_time_diff_thresh_)) {
            Eigen::Vector3d v_wl_l_est = velocimeter_ptr_->getVelocityEstimated();
            Eigen::Matrix3d R_v_est = velocimeter_ptr_->getCovariance();

            filter_ptr_->updateByVelocity(state_system_ptr_,
                                          v_wl_l_est, R_v_est);
        }

        // dynamic point removal
        if (config_ptr_->use_dynamic_point_removal) {
            auto v_wb_b_updated = (state_system_ptr_->getRotation().transpose() * state_system_ptr_->getVelocity()).eval();
            auto omg_wb_b_unbiased_updated = (imu_scan_end.first - state_system_ptr_->getGyroscopeBias()).eval();
            auto v_wl_l_updated = (config_ptr_->R_bl.transpose() * (v_wb_b_updated + omg_wb_b_unbiased_updated.cross(config_ptr_->p_bl_b))).eval();

            // remove dynamic points based on estimated velocity from updated state
            // static point cloud
            PointCloudType::Ptr scan_static_lidar_ptr(new PointCloudType());
            velocimeter_ptr_->removeDynamicPoint(scan_down_lidar_ptr_,
                                                 scan_static_lidar_ptr,
                                                 scan_dynamic_lidar_ptr_,
                                                 v_wl_l_updated);
            *scan_down_lidar_ptr_ = *scan_static_lidar_ptr;
        }
    }

    // initialize map
    if (map_ptr_->isNeedInitializeMap()) {
        map_ptr_->initializeMap(state_system_ptr_,
                                scan_down_lidar_ptr_);

        return;
    }

    // state update based on point-plane distance
    nearest_points_.resize(scan_down_lidar_ptr_->points.size());
    correspondence_num_ = filter_ptr_->updateByPlane(state_system_ptr_,
                                                     scan_down_lidar_ptr_, config_ptr_->point_cov,
                                                     map_ptr_, nearest_points_,
                                                     correspond_num_thresh_mapping_4d_);

    // transform current point cloud to world frame
    if (!config_ptr_->use_doppler ||
        (correspondence_num_ >= correspond_num_thresh_mapping_4d_)) {
        scan_down_world_ptr_ = map_ptr_->addMapPoints(state_system_ptr_, scan_down_lidar_ptr_,
                                                      nearest_points_);
    }

    if (is_first_scan_lio_) {
        is_first_scan_lio_ = false;
    }

    // stop timer
    t2 = std::chrono::steady_clock::now();
    time_processing_ = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()) * 1.0e-3;
}

void FMCWLIO::dialForwardMeasLI(const std::shared_ptr<MeasPackLI>& meas_ptr) {
    // LiDAR scan
    auto it_first_point_keep = std::lower_bound(meas_ptr->scan_ptr->points.begin(), meas_ptr->scan_ptr->points.end(),
                                                state_system_ptr_->getTime() - meas_ptr->scan_beg_time,
                                                [](const PointType& pt, const double t) -> bool {
        return ((pt.curvature * 1.0e-3) < t);
    });
    // erase invalid points before state time
    meas_ptr->scan_ptr->erase(meas_ptr->scan_ptr->points.begin(), it_first_point_keep);

    // IMU measurement
    auto it_first_imu_keep = std::lower_bound(meas_ptr->imu_msg_ptr_queue.begin(), meas_ptr->imu_msg_ptr_queue.end(),
                                              state_system_ptr_->getTime(),
                                              [](const sensor_msgs::Imu::ConstPtr& imu_msg_ptr, const double t) -> bool {
        return (imu_msg_ptr->header.stamp.toSec() < t);
    });
    // erase invalid IMU measurements before state time
    meas_ptr->imu_msg_ptr_queue.erase(meas_ptr->imu_msg_ptr_queue.begin(), it_first_imu_keep);
}

} // namespace fmcw_lio
