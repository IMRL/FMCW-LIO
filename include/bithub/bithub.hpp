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

#ifndef BITHUB_HPP
#define BITHUB_HPP

#include <condition_variable>

#include <nav_msgs/Path.h>
#include <topic_tools/shape_shifter.h>
#include <pcl/filters/voxel_grid.h>

#include "common/sensor.hpp"
#include "state/state.hpp"

// fmcw_lio
namespace fmcw_lio {

// FMCWLIO
class FMCWLIO;

// ConverterIMU
class ConverterIMU {
public:
    explicit ConverterIMU(const std::shared_ptr<const Config>& config_ptr);

    ~ConverterIMU() = default;

    // convert IMU message
    void convertIMU(const sensor_msgs::Imu::ConstPtr& msg_ptr_in,
                    sensor_msgs::Imu::Ptr& msg_ptr_out) const;

private:
    // configuration pointer
    const std::shared_ptr<const Config> config_ptr_;
};

// ConverterLiDAR
class ConverterLiDAR {
public:
    explicit ConverterLiDAR(const std::shared_ptr<const Config>& config_ptr);

    ~ConverterLiDAR() = default;

    // convert LiDAR scan point cloud message
    void convertLiDAR(const MsgPtrVariant& msg_ptr_in,
                      PointCloudType::Ptr& scan_ptr_out);

private:
    // configuration pointer
    const std::shared_ptr<const Config> config_ptr_;

    // LiDAR pointer
    const std::shared_ptr<LiDAR> lidar_ptr_;

    // filtered scan point cloud pointer
    PointCloudType::Ptr scan_filtered_ptr_;
};

// BitHub
class BitHub {
public:
    explicit BitHub(ros::NodeHandle& nh,
                    const std::shared_ptr<const Config>& config_ptr,
                    const std::shared_ptr<const FMCWLIO>& fmcw_lio_ptr);

    ~BitHub() = default;

    // IMU callback function
    void callIMU(const sensor_msgs::Imu::ConstPtr& msg_ptr_in);

    // LiDAR callback function
    void callLiDAR(const topic_tools::ShapeShifter::ConstPtr& msg_ptr_in);

    // pack LiDAR-inertial measurement
    bool packMeasLI();

    // get LiDAR-inertial measurement package
    std::shared_ptr<MeasPackLI> getMeasPackLI() const noexcept { return meas_pack_ptr_; }

    // publish data
    void publishData();

    // publish transformation
    static void publishTF(const std::shared_ptr<const StateBase>& state_ptr);

    // save map
    void dumpMap();

private:
    // read pose
    static void readPose(const std::string& path, std::deque<StatePoseQuat>& poses_stamp);

    // save pose
    void dumpPose();

    // publish estimated path during initialization
    void publishPathInit();

    // publish estimated path
    void publishPath();

    // publish ground truth path
    void publishPathGT();

    // publish odometry
    void publishOdometry();

    // publish established map during initialization
    void publishMapInit();

    // publish scan point cloud in world frame
    void publishScanInWorld();

    // publish scan point cloud in body frame
    void publishScanInBody();

    // publish dynamic scan point cloud in body frame
    void publishDynamicScanInBody();

    // transform a point from LiDAR frame to world frame
    void transformPointLiDARToWorld(const std::shared_ptr<StateBase>& state_ptr,
                                    const PointType& pt_in,
                                    PointType& pt_out);

    // transform a point from LiDAR frame to body frame
    void transformPointLiDARToBody(const PointType& pt_in,
                                   PointType& pt_out);

    // get CPU type
    static std::string getCPUType();

    // print system information into terminal
    void printSystemInfo();

private:
    // configuration pointer
    const std::shared_ptr<const Config> config_ptr_;

    // FMCW-LIO system pointer
    const std::shared_ptr<const FMCWLIO> fmcw_lio_ptr_;

    // IMU converter pointer
    const std::shared_ptr<ConverterIMU> converter_imu_ptr_;
    // LiDAR converter pointer
    const std::shared_ptr<ConverterLiDAR> converter_lidar_ptr_;

    // LiDAR-inertial measurement package pointer
    std::shared_ptr<MeasPackLI> meas_pack_ptr_;

    // mutex and condition variable
    std::mutex mtx_;
    std::condition_variable cv_meas_;

    // time, IMU, and LiDAR scan buffer
    std::deque<double> time_buffer_;
    std::deque<sensor_msgs::Imu::Ptr> imu_ptr_buffer_;
    std::deque<PointCloudType::Ptr> scan_ptr_buffer_;

    // scan pushed flag
    bool is_scan_pushed_;

    // timestamp variable
    double scan_end_time_;
    double last_imu_timestamp_;
    double last_scan_timestamp_;
    double lidar_scan_time_mean_;

    // map point cloud filter
    pcl::VoxelGrid<PointType> voxel_filter_map_;

    // IMU and LiDAR subscriber
    ros::Subscriber sub_imu_;
    ros::Subscriber sub_lidar_;
    // odometry and path publisher
    ros::Publisher pub_odom_;
    ros::Publisher pub_path_init_;
    ros::Publisher pub_path_;
    ros::Publisher pub_path_gt_;
    // point cloud and map publisher
    ros::Publisher pub_map_init_;
    ros::Publisher pub_point_cloud_body_;
    ros::Publisher pub_point_cloud_world_;
    ros::Publisher pub_dynamic_point_cloud_body_;

    // path
    nav_msgs::Path path_init_;
    nav_msgs::Path path_;
    nav_msgs::Path path_gt_;

    // ground truth pose
    std::deque<StatePoseQuat> poses_gt_;
    std::deque<StatePoseQuat>::iterator poses_gt_it_;

    // pose dumping file
    std::ofstream poses_file_;

    // map dumping
    std::string map_file_path_;
    std::size_t dumped_map_downsample_interval_;
    PointCloudType::Ptr map_wait_dump_ptr_;
};

} // namespace fmcw_lio

#endif // BITHUB_HPP
