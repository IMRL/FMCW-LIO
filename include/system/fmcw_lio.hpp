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

#ifndef FMCW_LIO_HPP
#define FMCW_LIO_HPP

#include <pcl/filters/voxel_grid.h>

#include "common/neon.hpp"
#include "initializer/initializer.hpp"
#include "velocimeter/velocimeter.hpp"

// fmcw_lio
namespace fmcw_lio {

// FMCWLIO
class FMCWLIO {
public:
    explicit FMCWLIO(const std::shared_ptr<Config>& config_ptr);

    ~FMCWLIO() = default;

    // FMCW-LIO system evolution
    void evolveSystem(const std::shared_ptr<MeasPackLI>& meas_ptr);

    // whether system is initialized
    [[nodiscard]] bool isInitialized() const noexcept { return is_init_success_; }

    // set callback function for imu-rate tf publish
    void setTFPublishFunction(TFPublishFunction tf_publish_func) {
        filter_ptr_->setTFPublishFunction(std::move(tf_publish_func));
    }

    // get system state
    [[nodiscard]] std::shared_ptr<StateBase> getState() const noexcept { return state_system_ptr_; }

    // get covariance
    [[nodiscard]] Eigen::MatrixXd getCovariance() const noexcept { return filter_ptr_->getCovariance(); }

    // get initial map
    [[nodiscard]] PointCloudType::Ptr getMapInit() const noexcept { return map_init_ptr_; }

    // get full scan in LiDAR frame
    [[nodiscard]] PointCloudType::Ptr getScanFullInLiDAR() const noexcept { return scan_compensated_lidar_ptr_; }

    // get down sampled scan in LiDAR frame
    [[nodiscard]] PointCloudType::Ptr getScanDownSampledInLiDAR() const noexcept { return scan_down_lidar_ptr_; }

    // get dynamic scan in LiDAR frame
    [[nodiscard]] PointCloudType::Ptr getScanDynamicInLiDAR() const noexcept { return scan_dynamic_lidar_ptr_; }

    // get down sampled scan in world frame
    [[nodiscard]] PointCloudType::Ptr getScanDownSampledInWorld() const noexcept { return scan_down_world_ptr_; }

    // get first scan time in system after launch
    [[nodiscard]] double getTimeFirstScanSys() const noexcept { return time_first_scan_sys_; }

    // get processing time
    [[nodiscard]] double getTimeProcessing() const noexcept { return time_processing_; }

    // get estimated velocity in LiDAR frame from velocimeter
    [[nodiscard]] Eigen::Vector3d getVelocityEstimated() const noexcept {
        return velocimeter_ptr_->getVelocityEstimated();
    }

    // get poses during initialization
    [[nodiscard]] std::shared_ptr<std::vector<std::shared_ptr<StatePoseQuat>>> getPosesInit() const noexcept {
        return initializer_ptr_->getPoses();
    }

    // get initialization logger
    [[nodiscard]] std::shared_ptr<LoggerInitialization> getLoggerInitialization() const noexcept {
        return initializer_ptr_->logger_initialization_ptr;
    }

    // get compensation logger
    [[nodiscard]] std::shared_ptr<LoggerCompensation> getLoggerCompensation() const noexcept {
        return filter_ptr_->getLoggerCompensation();
    }

    // get propagation logger
    [[nodiscard]] std::shared_ptr<LoggerPropagation> getLoggerPropagation() const noexcept {
        return filter_ptr_->getLoggerPropagation();
    }

    // get update logger
    [[nodiscard]] std::shared_ptr<LoggerUpdate> getLoggerUpdate() const noexcept {
        return filter_ptr_->getLoggerUpdate();
    }

    // get velocimetry logger
    [[nodiscard]] std::shared_ptr<LoggerVelocimetry> getLoggerVelocimetry() const noexcept {
        return velocimeter_ptr_->logger_velocimetry_ptr;
    }

private:
    // dial forward LiDAR and IMU measurement according to state time
    void dialForwardMeasLI(const std::shared_ptr<MeasPackLI>& meas_ptr);

public:
    // FMCW-LIO system neon
    inline static const Neon neon{
        {"F", {241,  38,  29}},
        {"M", {255, 146,   0}},
        {"C", {253, 219,  42}},
        {"W", { 77, 228,  43}},
        {"-", { 15, 246, 146}},
        {"L", { 11, 216, 249}},
        {"I", { 44,  34, 251}},
        {"O", {195,  45, 243}}
    };

private:
    // configuration pointer
    const std::shared_ptr<Config> config_ptr_;

    // system state pointer
    const std::shared_ptr<StateBase> state_system_ptr_;

    // initializer pointer
    const std::shared_ptr<Initializer> initializer_ptr_;
    // filter pointer
    const std::shared_ptr<Filter> filter_ptr_;
    // map pointer
    const std::shared_ptr<Map> map_ptr_;
    // velocimeter pointer
    const std::shared_ptr<Velocimeter> velocimeter_ptr_;

    // system initialized flag
    bool is_init_success_;

    // first scan in system after launch
    bool is_first_scan_sys_;
    double time_first_scan_sys_;
    // first scan in LIO after initialization
    bool is_first_scan_lio_;

    // scan and map point cloud pointer
    PointCloudType::Ptr map_init_ptr_;
    PointCloudType::Ptr scan_compensated_lidar_ptr_;
    PointCloudType::Ptr scan_down_lidar_ptr_;
    PointCloudType::Ptr scan_dynamic_lidar_ptr_;
    PointCloudType::Ptr scan_down_world_ptr_;

    // point cloud filter
    pcl::VoxelGrid<PointType> voxel_filter_scan_;

    // time difference threshold for velocity update
    const double vel_update_time_ratio_;
    double vel_update_time_diff_thresh_;

    // nearest points
    PointVec2D nearest_points_;
    // correspondence number
    std::size_t correspondence_num_;

    // record processing time
    double time_processing_;

    // correspondence number threshold for mapping when using 4D Doppler LiDAR
    const std::size_t correspond_num_thresh_mapping_4d_;
};

} // namespace fmcw_lio

#endif // FMCW_LIO_HPP
