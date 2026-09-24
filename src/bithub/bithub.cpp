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

#include <filesystem>

#include <omp.h>
#include <sys/times.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include "bithub/bithub.hpp"

// fmcw_lio
namespace fmcw_lio {

// ConverterIMU
ConverterIMU::ConverterIMU(const std::shared_ptr<const Config>& config_ptr) : config_ptr_(config_ptr) {}

void ConverterIMU::convertIMU(const sensor_msgs::Imu::ConstPtr& msg_ptr_in,
                              sensor_msgs::Imu::Ptr& msg_ptr_out) const {
    // scale IMU data
    msg_ptr_out->linear_acceleration.x = msg_ptr_in->linear_acceleration.x * config_ptr_->gravity_scale;
    msg_ptr_out->linear_acceleration.y = msg_ptr_in->linear_acceleration.y * config_ptr_->gravity_scale;
    msg_ptr_out->linear_acceleration.z = msg_ptr_in->linear_acceleration.z * config_ptr_->gravity_scale;
}

// ConverterLiDAR
ConverterLiDAR::ConverterLiDAR(const std::shared_ptr<const Config>& config_ptr) : config_ptr_(config_ptr),
                                                                                  lidar_ptr_(std::make_shared<LiDAR>(config_ptr_->lidar_type)) {
    // initialize scan point cloud
    scan_filtered_ptr_.reset(new PointCloudType());
}

void ConverterLiDAR::convertLiDAR(const MsgPtrVariant& msg_ptr_in,
                                  PointCloudType::Ptr& scan_ptr_out) {
    // clear previous point cloud
    scan_filtered_ptr_->clear();

    // point cloud conversion
    std::visit([&](auto&& msg_lambda) -> void {
        // decayed message pointer type
        using MsgPtrType = std::decay_t<decltype(msg_lambda)>;

        // get point number and field offset
        constexpr bool is_livox_msg = std::is_same_v<MsgPtrType, livox_ros_driver::CustomMsg::Ptr>;

        std::size_t point_size;
        if constexpr (!is_livox_msg) {
            point_size = msg_lambda->width * msg_lambda->height;
            lidar_ptr_->setOffset(msg_lambda->fields);
        } else {
            point_size = msg_lambda->point_num;
        }

        if (point_size == 0) {
            return;
        }
        scan_filtered_ptr_->reserve(point_size);

        // collect valid point from message
        const std::size_t start_idx = (is_livox_msg ? 1 : 0);
        const std::size_t livox_scan_line_num = 6;

        if constexpr (!is_livox_msg) {
            lidar_ptr_->parseFirstPointTimestamp(msg_lambda);
        }

        double msg_timestamp_correction = 0.0;
        for (std::size_t i = start_idx; i < point_size; ++i) {
            // field:  timestamp, x, y, z, intensity
            float timestamp, x, y, z, intensity;

            if constexpr (is_livox_msg) {
                timestamp = static_cast<float>(msg_lambda->points[i].offset_time);
                x = msg_lambda->points[i].x;
                y = msg_lambda->points[i].y;
                z = msg_lambda->points[i].z;
                intensity = msg_lambda->points[i].reflectivity;
            } else {
                lidar_ptr_->parsePointField(msg_lambda,
                                            i,
                                            timestamp, x, y, z, intensity);
            }
            timestamp = timestamp * lidar_ptr_->getTimeUnitScale();

            if (config_ptr_->scan_end_timestamp &&
                (msg_timestamp_correction < timestamp)) {
                msg_timestamp_correction = static_cast<double>(timestamp);
            }

            bool condition = true;
            if constexpr (is_livox_msg) {
                condition = (msg_lambda->points[i].line < livox_scan_line_num) &&
                            (((msg_lambda->points[i].tag & 0x30) == 0x10) ||
                             ((msg_lambda->points[i].tag & 0x30) == 0x00));
            }

            if (condition &&
                ((i % config_ptr_->point_select_interval) == 0)) {
                double range_squared = (x * x) + (y * y) + (z * z);

                if (range_squared >= config_ptr_->skip_radius_squared) {
                    PointType added_pt;

                    added_pt.curvature = timestamp;
                    added_pt.x = x;
                    added_pt.y = y;
                    added_pt.z = z;
                    added_pt.intensity = intensity;

                    scan_filtered_ptr_->points.push_back(added_pt);
                }
            }
        }

        // sort point via timestamp
        *scan_ptr_out = *scan_filtered_ptr_;
        std::sort(scan_ptr_out->points.begin(), scan_ptr_out->points.end(),
                  [](PointType& x, PointType& y) -> bool { return (x.curvature < y.curvature); });

        // if scan message timestamp is at scan end then correct it to scan begin
        if (config_ptr_->scan_end_timestamp) {
            msg_lambda->header.stamp = ros::Time(msg_lambda->header.stamp.toSec() - (msg_timestamp_correction * 1.0e-3));
        }
    }, msg_ptr_in);
}

// BitHub
BitHub::BitHub(ros::NodeHandle& nh,
               const std::shared_ptr<const Config>& config_ptr,
               const std::shared_ptr<const FMCWLIO>& fmcw_lio_ptr) : config_ptr_(config_ptr),
                                                                     fmcw_lio_ptr_(fmcw_lio_ptr),
                                                                     converter_imu_ptr_(std::make_shared<ConverterIMU>(config_ptr_)),
                                                                     converter_lidar_ptr_(std::make_shared<ConverterLiDAR>(config_ptr_)),
                                                                     meas_pack_ptr_(std::make_shared<MeasPackLI>()),
                                                                     is_scan_pushed_{false},
                                                                     scan_end_time_{0.0},
                                                                     last_imu_timestamp_{0.0},
                                                                     last_scan_timestamp_{0.0},
                                                                     lidar_scan_time_mean_{0.0},
                                                                     dumped_map_downsample_interval_{4} {
    // average LiDAR scan time
    lidar_scan_time_mean_ = 1.0 / config_ptr_->scan_rate;

    // voxel filter
    voxel_filter_map_.setLeafSize(static_cast<float>(config_ptr_->voxel_filter_size_map),
                                  static_cast<float>(config_ptr_->voxel_filter_size_map),
                                  static_cast<float>(config_ptr_->voxel_filter_size_map));

    // subscriber and publisher
    sub_imu_ = nh.subscribe(config_ptr_->topic_imu, 1000,
                            &BitHub::callIMU, this);
    sub_lidar_ = nh.subscribe(config_ptr_->topic_lidar, 1000,
                              &BitHub::callLiDAR, this);

    pub_odom_ = nh.advertise<nav_msgs::Odometry>("/odometry", 100000);
    pub_path_init_ = nh.advertise<nav_msgs::Path>("/path_init", 100000);
    pub_path_ = nh.advertise<nav_msgs::Path>("/path", 100000);
    pub_path_gt_ = nh.advertise<nav_msgs::Path>("/path_gt", 100000);

    pub_map_init_ = nh.advertise<sensor_msgs::PointCloud2>("/map_init", 100000);
    pub_point_cloud_body_ = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud_body", 100000);
    pub_point_cloud_world_ = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud_world", 100000);
    pub_dynamic_point_cloud_body_ = nh.advertise<sensor_msgs::PointCloud2>("/dynamic_point_cloud_body", 100000);

    // ground truth pose
    if (config_ptr_->display_gt) {
        readPose(config_ptr_->gt_file_path, poses_gt_);
        poses_gt_it_ = poses_gt_.begin();
    }

    // prepare recording file
    if (config_ptr_->dump_pose) {
        auto now = std::chrono::system_clock::now();
        std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
        std::tm now_tm{};

        localtime_r(&now_time_t, &now_tm);
        std::ostringstream timestamp;
        timestamp << std::put_time(&now_tm, "%Y-%m-%d-%H-%M-%S");

        std::ostringstream full_pose_path, full_map_path;
        full_pose_path << config_ptr_->pose_dir_path;
        full_map_path << config_ptr_->map_dir_path;
        if (!config_ptr_->pose_dir_path.empty() &&
            (config_ptr_->pose_dir_path.back() != '/') &&
            (config_ptr_->pose_dir_path.back() != '\\')) {
            full_pose_path << "/";
        }
        if (!config_ptr_->map_dir_path.empty() &&
            (config_ptr_->map_dir_path.back() != '/') &&
            (config_ptr_->map_dir_path.back() != '\\')) {
            full_map_path << "/";
        }
        full_pose_path << "fmcw_lio_pose_" << timestamp.str() << ".txt";
        full_map_path << "fmcw_lio_map_" << timestamp.str() << ".pcd";

        std::string pose_file_path = full_pose_path.str();
        map_file_path_ = full_map_path.str();

        if (std::filesystem::exists(pose_file_path)) {
            std::filesystem::remove(pose_file_path);
        }
        if (std::filesystem::exists(map_file_path_)) {
            std::filesystem::remove(map_file_path_);
        }

        std::filesystem::create_directories(std::filesystem::path(config_ptr_->pose_dir_path.c_str()));
        poses_file_.open(pose_file_path.c_str());
    }

    // initialize map point cloud
    map_wait_dump_ptr_.reset(new PointCloudType());
}

void BitHub::callIMU(const sensor_msgs::Imu::ConstPtr& msg_ptr_in) {
    // get IMU message and push it into buffer
    mtx_.lock();

    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_ptr_in));
    msg->header.stamp = ros::Time().fromSec(msg_ptr_in->header.stamp.toSec() + config_ptr_->time_diff_bl);
    double timestamp = msg->header.stamp.toSec();

    // scale specific force
    if (config_ptr_->scaling) {
        converter_imu_ptr_->convertIMU(msg_ptr_in,
                                       msg);
    }

    if (timestamp < last_imu_timestamp_) {
        imu_ptr_buffer_.clear();
    }

    last_imu_timestamp_ = timestamp;
    imu_ptr_buffer_.push_back(msg);

    mtx_.unlock();
    cv_meas_.notify_all();
}

void BitHub::callLiDAR(const topic_tools::ShapeShifter::ConstPtr& msg_ptr_in) {
    // get LiDAR message and push it into buffer
    mtx_.lock();

    MsgPtrVariant msg_ptr_variant;
    if (config_ptr_->lidar_type != LiDARType::Livox) {
        msg_ptr_variant = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2(*msg_ptr_in->instantiate<sensor_msgs::PointCloud2>()));
    } else {
        livox_ros_driver::CustomMsg::Ptr livox_custom_msg(new livox_ros_driver::CustomMsg());

        const auto size_msg = msg_ptr_in->size();
        std::vector<std::uint8_t> buffer_temp(size_msg);

        ros::serialization::OStream ostream_temp(buffer_temp.data(), size_msg);
        msg_ptr_in->write(ostream_temp);

        ros::serialization::IStream istream_temp(buffer_temp.data(), size_msg);
        ros::serialization::deserialize(istream_temp, *livox_custom_msg);

        msg_ptr_variant = livox_custom_msg;
    }

    PointCloudType::Ptr ptr(new PointCloudType());
    converter_lidar_ptr_->convertLiDAR(msg_ptr_variant,
                                       ptr);

    std::visit([&](auto&& msg_lambda) -> void {
        if (msg_lambda->header.stamp.toSec() < last_scan_timestamp_) {
            scan_ptr_buffer_.clear();
        }

        last_scan_timestamp_ = msg_lambda->header.stamp.toSec();
    }, msg_ptr_variant);

    scan_ptr_buffer_.push_back(ptr);
    time_buffer_.push_back(last_scan_timestamp_);

    mtx_.unlock();
    cv_meas_.notify_all();
}

bool BitHub::packMeasLI() {
    std::unique_lock<std::mutex> lock(mtx_);

    if (time_buffer_.empty() ||
        imu_ptr_buffer_.empty() ||
        scan_ptr_buffer_.empty()) {
        return false;
    }

    if (!meas_pack_ptr_) {
        meas_pack_ptr_ = std::make_shared<MeasPackLI>();
    }

    // set scan end time
    if (!is_scan_pushed_) {
        meas_pack_ptr_->scan_ptr = scan_ptr_buffer_.front();
        meas_pack_ptr_->scan_beg_time = time_buffer_.front();

        if (meas_pack_ptr_->scan_ptr->points.empty()) {
            scan_ptr_buffer_.pop_front();
            time_buffer_.pop_front();

            return false;
        }

        if ((meas_pack_ptr_->scan_ptr->points.back().curvature * 1.0e-3) > (1.5 * lidar_scan_time_mean_)) {
            scan_ptr_buffer_.pop_front();
            time_buffer_.pop_front();

            return false;
        } else {
            scan_end_time_ = meas_pack_ptr_->scan_beg_time + meas_pack_ptr_->scan_ptr->points.back().curvature * 1.0e-3;
        }
        meas_pack_ptr_->scan_end_time = scan_end_time_;

        is_scan_pushed_ = true;
    }

    if (last_imu_timestamp_ < scan_end_time_) {
        return false;
    }

    // pack IMU and LiDAR scan data
    meas_pack_ptr_->imu_msg_ptr_queue.clear();
    while (!imu_ptr_buffer_.empty()) {
        const double imu_time = imu_ptr_buffer_.front()->header.stamp.toSec();
        if (imu_time > scan_end_time_) {
            break;
        }

        meas_pack_ptr_->imu_msg_ptr_queue.emplace_back(imu_ptr_buffer_.front());
        imu_ptr_buffer_.pop_front();
    }

    while (imu_ptr_buffer_.empty() ||
           (imu_ptr_buffer_.front()->header.stamp.toSec() < scan_end_time_)) {
        cv_meas_.wait(lock, [this]() -> bool { return (!imu_ptr_buffer_.empty()); });

        if (!imu_ptr_buffer_.empty() &&
            (imu_ptr_buffer_.front()->header.stamp.toSec() < scan_end_time_)) {
            meas_pack_ptr_->imu_msg_ptr_queue.emplace_back(imu_ptr_buffer_.front());
            imu_ptr_buffer_.pop_front();
        }
    }

    meas_pack_ptr_->imu_msg_ptr_queue.emplace_back(imu_ptr_buffer_.front());

    scan_ptr_buffer_.pop_front();
    time_buffer_.pop_front();

    is_scan_pushed_ = false;

    return true;
}

void BitHub::publishData() {
    // only publish data after system is initialized
    if (!fmcw_lio_ptr_->isInitialized()) {
        return;
    }

    if (path_init_.poses.empty()) {
        publishPathInit();

        if (config_ptr_->init_mapping ||
            (config_ptr_->init_method == InitializationMethod::FreeInit)) {
            publishMapInit();
        }
    }

    // dump pose
    if (config_ptr_->dump_pose) {
        dumpPose();
    }

    // publish path
    publishPath();

    // publish ground truth path
    if (config_ptr_->display_gt) {
        publishPathGT();
    }

    // publish odometry
    publishOdometry();

    // publish scan point cloud in body frame
    publishScanInBody();

    // publish dynamic scan point cloud in body frame
    if (config_ptr_->use_doppler &&
        config_ptr_->use_dynamic_point_removal) {
        publishDynamicScanInBody();
    }

    // publish scan point cloud in world frame
    publishScanInWorld();

    // print system information into terminal
    printSystemInfo();
}

void BitHub::publishTF(const std::shared_ptr<const StateBase>& state_ptr) {
    // publish transformation from world frame to body frame
    Eigen::Quaterniond q_wb(state_ptr->getRotation());
    Eigen::Vector3d p_wb_w = state_ptr->getPosition();

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(p_wb_w.x(), p_wb_w.y(), p_wb_w.z()));
    q.setW(q_wb.w());
    q.setX(q_wb.x());
    q.setY(q_wb.y());
    q.setZ(q_wb.z());
    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform, ros::Time().fromSec(state_ptr->getTime()), "world", "body"));
}

void BitHub::dumpMap() {
    // downsampled map
    PointCloudType::Ptr map_point_cloud(new PointCloudType());
    voxel_filter_map_.setInputCloud(map_wait_dump_ptr_);
    voxel_filter_map_.filter(*map_point_cloud);

    std::cout << "The whole map with point size: " << map_wait_dump_ptr_->points.size()
              << " , is saved to " << map_file_path_ << " ." << std::endl;

    pcl::PCDWriter map_writer;
    map_writer.writeBinary(map_file_path_, *map_point_cloud);
}

void BitHub::readPose(const std::string& path, std::deque<StatePoseQuat>& poses_stamp) {
    // check file path
    std::ifstream txtFile(path.c_str());
    if (!txtFile.is_open()) {
        std::cerr << "Cannot open file: " << path << " ." << std::endl;

        std::exit(-1);
    }

    std::string line;
    while (std::getline(txtFile, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }

        double t;
        Eigen::Quaterniond q;
        Eigen::Vector3d p;

        std::stringstream ss(line);
        ss >> t;
        ss >> p.x();
        ss >> p.y();
        ss >> p.z();
        ss >> q.x();
        ss >> q.y();
        ss >> q.z();
        ss >> q.w();

        StatePoseQuat pose(t, q, p);
        poses_stamp.push_back(pose);
    }

    std::cout << "Successfully read: " << poses_stamp.size() << " ground truth poses." << std::endl;
}

void BitHub::dumpPose() {
    // get estimated pose in world frame
    const auto state_ptr = fmcw_lio_ptr_->getState();
    const auto q_wb = Eigen::Quaterniond(state_ptr->getRotation());
    const auto p_wb_w = state_ptr->getPosition();

    poses_file_.precision(8);
    poses_file_.setf(std::ios::fixed, std::ios::floatfield);
    poses_file_ << state_ptr->getTime() << " ";
    poses_file_.precision(10);
    poses_file_ << p_wb_w.x() << " " << p_wb_w.y() << " " << p_wb_w.z() << " "
                << q_wb.x() << " " << q_wb.y() << " " << q_wb.z() << " " << q_wb.w();
    poses_file_ << "\n";
    poses_file_.flush();
}

void BitHub::publishPathInit() {
    // get estimated poses during initialization in world frame
    const auto states_poses_ptr = fmcw_lio_ptr_->getPosesInit();
    for (const auto& state_pose : *states_poses_ptr) {
        const auto R_wb = state_pose->getRotation();
        const auto p_wb_w = state_pose->getPosition();

        geometry_msgs::PoseStamped msg_pose;

        auto q_wb = Eigen::Quaterniond(R_wb);
        msg_pose.pose.orientation.x = q_wb.x();
        msg_pose.pose.orientation.y = q_wb.y();
        msg_pose.pose.orientation.z = q_wb.z();
        msg_pose.pose.orientation.w = q_wb.w();

        msg_pose.pose.position.x = p_wb_w.x();
        msg_pose.pose.position.y = p_wb_w.y();
        msg_pose.pose.position.z = p_wb_w.z();

        msg_pose.header.stamp = ros::Time().fromSec(state_pose->getTime());
        msg_pose.header.frame_id = "world";

        path_init_.poses.push_back(msg_pose);
    }
    path_init_.header.frame_id = "world";
    path_init_.header.stamp = ros::Time().fromSec(states_poses_ptr->back()->getTime());

    pub_path_init_.publish(path_init_);
}

void BitHub::publishPath() {
    // get estimated pose in world frame
    const auto state_ptr = fmcw_lio_ptr_->getState();
    const auto R_wb = state_ptr->getRotation();
    const auto p_wb_w = state_ptr->getPosition();

    geometry_msgs::PoseStamped msg_pose;

    auto q_wb = Eigen::Quaterniond(R_wb);
    msg_pose.pose.orientation.x = q_wb.x();
    msg_pose.pose.orientation.y = q_wb.y();
    msg_pose.pose.orientation.z = q_wb.z();
    msg_pose.pose.orientation.w = q_wb.w();

    msg_pose.pose.position.x = p_wb_w.x();
    msg_pose.pose.position.y = p_wb_w.y();
    msg_pose.pose.position.z = p_wb_w.z();

    msg_pose.header.stamp = ros::Time().fromSec(state_ptr->getTime());
    msg_pose.header.frame_id = "world";

    path_.header.frame_id = "world";
    path_.header.stamp = ros::Time().fromSec(state_ptr->getTime());
    path_.poses.push_back(msg_pose);

    pub_path_.publish(path_);
}

void BitHub::publishPathGT() {
    // get ground truth pose in world frame
    const auto state_ptr = fmcw_lio_ptr_->getState();
    while ((poses_gt_it_->getTime() - state_ptr->getTime()) < -0.01) {
        poses_gt_it_ += 1;

        if (poses_gt_it_ == poses_gt_.end()) {
            return;
        }
    }

    const auto q_wb = poses_gt_it_->getRotationQuat();
    const auto p_wb_w = poses_gt_it_->getPosition();

    geometry_msgs::PoseStamped msg_pose;

    msg_pose.pose.orientation.x = q_wb.x();
    msg_pose.pose.orientation.y = q_wb.y();
    msg_pose.pose.orientation.z = q_wb.z();
    msg_pose.pose.orientation.w = q_wb.w();

    msg_pose.pose.position.x = p_wb_w.x();
    msg_pose.pose.position.y = p_wb_w.y();
    msg_pose.pose.position.z = p_wb_w.z();

    msg_pose.header.stamp = ros::Time().fromSec(state_ptr->getTime());
    msg_pose.header.frame_id = "world";

    static std::size_t gt_path_count = 0;
    gt_path_count += 1;
    if ((gt_path_count % 10) == 1) {
        path_gt_.header.frame_id = "world";
        path_gt_.header.stamp = ros::Time().fromSec(state_ptr->getTime());
        path_gt_.poses.push_back(msg_pose);

        pub_path_gt_.publish(path_gt_);
    }
}

void BitHub::publishOdometry() {
    // get estimated pose and body velocity
    const auto state_ptr = fmcw_lio_ptr_->getState();
    const auto R_wb = state_ptr->getRotation();
    const auto v_wb_b = (state_ptr->getRotation().transpose() * state_ptr->getVelocity()).eval();
    const auto p_wb_w = state_ptr->getPosition();
    const auto P = fmcw_lio_ptr_->getCovariance();

    nav_msgs::Odometry odom;

    odom.header.stamp = ros::Time().fromSec(state_ptr->getTime());
    odom.header.frame_id = "world";
    odom.child_frame_id = "body";

    auto q_wb = Eigen::Quaterniond(R_wb);
    odom.pose.pose.orientation.x = q_wb.x();
    odom.pose.pose.orientation.y = q_wb.y();
    odom.pose.pose.orientation.z = q_wb.z();
    odom.pose.pose.orientation.w = q_wb.w();

    odom.pose.pose.position.x = p_wb_w.x();
    odom.pose.pose.position.y = p_wb_w.y();
    odom.pose.pose.position.z = p_wb_w.z();

    for (int i = 0; i < 6; ++i) {
        int k = ((i < 3) ? (i + 6) : (i - 3));
        odom.pose.covariance[(i * 6) + 0] = P(k, 6);
        odom.pose.covariance[(i * 6) + 1] = P(k, 7);
        odom.pose.covariance[(i * 6) + 2] = P(k, 8);
        odom.pose.covariance[(i * 6) + 3] = P(k, 0);
        odom.pose.covariance[(i * 6) + 4] = P(k, 1);
        odom.pose.covariance[(i * 6) + 5] = P(k, 2);
    }

    odom.twist.twist.linear.x = v_wb_b.x();
    odom.twist.twist.linear.y = v_wb_b.y();
    odom.twist.twist.linear.z = v_wb_b.z();

    pub_odom_.publish(odom);

    publishTF(state_ptr);
}

void BitHub::publishMapInit() {
    // get map established during initialization
    const auto map_init_ptr = fmcw_lio_ptr_->getMapInit();

    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(*map_init_ptr, map_msg);
    map_msg.header.stamp = ros::Time().fromSec(fmcw_lio_ptr_->getState()->getTime());
    map_msg.header.frame_id = "world";

    pub_map_init_.publish(map_msg);
}

void BitHub::publishScanInWorld() {
    // transform LiDAR scan point cloud to world frame
    const auto state_ptr = fmcw_lio_ptr_->getState();
    PointCloudType::Ptr scan_pub_lidar(config_ptr_->publish_dense_scan ? fmcw_lio_ptr_->getScanFullInLiDAR() :
                                                                         fmcw_lio_ptr_->getScanDownSampledInLiDAR());
    const auto pt_pub_size = scan_pub_lidar->points.size();
    PointCloudType::Ptr scan_pub_world(new PointCloudType(pt_pub_size, 1));

    std::vector<std::size_t> point_indices(pt_pub_size);
    std::for_each(point_indices.begin(), point_indices.end(), [n = 0](std::size_t& idx) mutable -> void { idx = n++; });
    std::for_each(std::execution::par,
                  point_indices.begin(), point_indices.end(),
                  [&](auto point_idx) -> void {
        transformPointLiDARToWorld(state_ptr,
                                   scan_pub_lidar->points[point_idx],
                                   scan_pub_world->points[point_idx]);
    });

    sensor_msgs::PointCloud2 scan_msg;
    pcl::toROSMsg(*scan_pub_world, scan_msg);
    scan_msg.header.stamp = ros::Time().fromSec(state_ptr->getTime());
    scan_msg.header.frame_id = "world";

    pub_point_cloud_world_.publish(scan_msg);

    // dump map
    if (config_ptr_->dump_map) {
        // accumulate point cloud in world frame
        const auto scan_pub_world_ptr = fmcw_lio_ptr_->getScanDownSampledInWorld();

        static std::size_t scan_wait_num = 0;
        scan_wait_num += 1;
        if ((scan_wait_num % dumped_map_downsample_interval_) == 0) {
            *map_wait_dump_ptr_ += *scan_pub_world_ptr;
        }
    }
}

void BitHub::publishScanInBody() {
    // transform LiDAR scan point cloud to body frame
    const auto state_ptr = fmcw_lio_ptr_->getState();
    const auto scan_compensated_ptr = fmcw_lio_ptr_->getScanFullInLiDAR();
    const auto pt_comp_size = scan_compensated_ptr->points.size();
    PointCloudType::Ptr scan_pub_body(new PointCloudType(pt_comp_size, 1));

    std::vector<std::size_t> point_indices(pt_comp_size);
    std::for_each(point_indices.begin(), point_indices.end(), [n = 0](std::size_t& idx) mutable -> void { idx = n++; });
    std::for_each(std::execution::par,
                  point_indices.begin(), point_indices.end(),
                  [&](auto point_idx) -> void {
        transformPointLiDARToBody(scan_compensated_ptr->points[point_idx],
                                  scan_pub_body->points[point_idx]);
    });

    sensor_msgs::PointCloud2 scan_msg;
    pcl::toROSMsg(*scan_pub_body, scan_msg);
    scan_msg.header.stamp = ros::Time().fromSec(state_ptr->getTime());
    scan_msg.header.frame_id = "body";

    pub_point_cloud_body_.publish(scan_msg);
}

void BitHub::publishDynamicScanInBody() {
    // transform dynamic LiDAR scan point cloud to body frame
    const auto state_ptr = fmcw_lio_ptr_->getState();
    const auto scan_dynamic_lidar_ptr = fmcw_lio_ptr_->getScanDynamicInLiDAR();
    const auto pt_dyna_size = scan_dynamic_lidar_ptr->points.size();
    PointCloudType::Ptr scan_pub_body(new PointCloudType(pt_dyna_size, 1));

    std::vector<std::size_t> point_indices(pt_dyna_size);
    std::for_each(point_indices.begin(), point_indices.end(), [n = 0](std::size_t& idx) mutable -> void { idx = n++; });
    std::for_each(std::execution::par,
                  point_indices.begin(), point_indices.end(),
                  [&](auto point_idx) -> void {
        transformPointLiDARToBody(scan_dynamic_lidar_ptr->points[point_idx],
                                  scan_pub_body->points[point_idx]);
    });

    sensor_msgs::PointCloud2 scan_msg;
    pcl::toROSMsg(*scan_pub_body, scan_msg);
    scan_msg.header.stamp = ros::Time().fromSec(state_ptr->getTime());
    scan_msg.header.frame_id = "body";

    pub_dynamic_point_cloud_body_.publish(scan_msg);
}

void BitHub::transformPointLiDARToWorld(const std::shared_ptr<StateBase>& state_ptr,
                                        const PointType& pt_in,
                                        PointType& pt_out) {
    // get rotation and position of body frame in world frame
    const auto R_wb = state_ptr->getRotation();
    const auto p_wb_w = state_ptr->getPosition();

    // transform point from LiDAR frame to world frame
    Eigen::Vector3d p_lp_l{pt_in.x, pt_in.y, pt_in.z};
    Eigen::Vector3d p_wp_w = (R_wb * ((config_ptr_->R_bl * p_lp_l) + config_ptr_->p_bl_b)) + p_wb_w;

    pt_out.curvature = pt_in.curvature;
    pt_out.x = static_cast<float>(p_wp_w(0));
    pt_out.y = static_cast<float>(p_wp_w(1));
    pt_out.z = static_cast<float>(p_wp_w(2));
    pt_out.intensity = pt_in.intensity;
}

void BitHub::transformPointLiDARToBody(const PointType& pt_in,
                                       PointType& pt_out) {
    // get point position in LiDAR frame
    const Eigen::Vector3d p_lp_l{pt_in.x, pt_in.y, pt_in.z};
    // transform point from LiDAR frame to body frame
    const Eigen::Vector3d p_bp_b = (config_ptr_->R_bl * p_lp_l) + config_ptr_->p_bl_b;

    pt_out.curvature = pt_in.curvature;
    pt_out.x = static_cast<float>(p_bp_b(0));
    pt_out.y = static_cast<float>(p_bp_b(1));
    pt_out.z = static_cast<float>(p_bp_b(2));
    pt_out.intensity = pt_in.intensity;
}

std::string BitHub::getCPUType() {
    // get CPU type string
    std::string cpu_type;

#ifdef HAS_CPUID
    unsigned int CPUInfo[4] = {0, 0, 0, 0};
    __cpuid(0x80000000, CPUInfo[0], CPUInfo[1], CPUInfo[2], CPUInfo[3]);
    unsigned int nExIds = CPUInfo[0];
    for (unsigned int i = 0x80000000; i <= nExIds; ++i) {
        __cpuid(i, CPUInfo[0], CPUInfo[1], CPUInfo[2], CPUInfo[3]);
        if (i == 0x80000002)
            memcpy(CPUBrandString, CPUInfo, sizeof(CPUInfo));
        else if (i == 0x80000003)
            memcpy(CPUBrandString + 16, CPUInfo, sizeof(CPUInfo));
        else if (i == 0x80000004)
            memcpy(CPUBrandString + 32, CPUInfo, sizeof(CPUInfo));
    }
    this->cpu_type = CPUBrandString;
    boost::trim(this->cpu_type);
#endif

    return cpu_type;
}

void BitHub::printSystemInfo() {
    // print system information into terminal
    const auto state_ptr = fmcw_lio_ptr_->getState();
    static double distance_traveled = 0.0;
    static Eigen::Vector3d p_wb_w_curr = Eigen::Vector3d::Zero();
    static Eigen::Vector3d p_wb_w_last = Eigen::Vector3d::Zero();

    if (state_ptr->getPosition() == Eigen::Vector3d::Zero().eval()) {
        p_wb_w_last = state_ptr->getPosition();

        return;
    }

    p_wb_w_curr = state_ptr->getPosition();
    double l = std::sqrt(std::pow(p_wb_w_curr[0] - p_wb_w_last[0], 2) +
                         std::pow(p_wb_w_curr[1] - p_wb_w_last[1], 2) +
                         std::pow(p_wb_w_curr[2] - p_wb_w_last[2], 2));
    if (l >= 0.1) {
        distance_traveled += l;
        p_wb_w_last = p_wb_w_curr;
    }

    auto pushHistory = [](std::vector<double>& v,
                          double x,
                          std::size_t max_keep) -> void {
        v.push_back(x);
        if (v.size() > max_keep) {
            v.erase(v.begin(), v.begin() + static_cast<long>(v.size() - max_keep));
        }
    };

    constexpr std::size_t kMaxKeep = 36000;

    // average propagation time
    auto propagation_logger_ptr = fmcw_lio_ptr_->getLoggerPropagation();
    static std::vector<double> prop_time;
    pushHistory(prop_time,
                propagation_logger_ptr->propagation_time,
                kMaxKeep);
    double avg_prop_time = 0.0;
    if (!prop_time.empty()) {
        avg_prop_time = std::accumulate(prop_time.begin(), prop_time.end(),
                                        0.0) / static_cast<double>(prop_time.size());
    }

    // average motion compensation time
    auto compensation_logger_ptr = fmcw_lio_ptr_->getLoggerCompensation();
    static std::vector<double> motion_comp_time;
    pushHistory(motion_comp_time,
                compensation_logger_ptr->compensation_time,
                kMaxKeep);
    double avg_motion_comp_time = 0.0;
    if (!motion_comp_time.empty()) {
        avg_motion_comp_time = std::accumulate(motion_comp_time.begin(), motion_comp_time.end(),
                                               0.0) / static_cast<double>(motion_comp_time.size());
    }

    // average LiDAR velocity-based state update time
    auto update_logger_ptr = fmcw_lio_ptr_->getLoggerUpdate();
    static std::vector<double> velocity_update_time;
    pushHistory(velocity_update_time,
                update_logger_ptr->velocity_based_update_time,
                kMaxKeep);
    double avg_velocity_update_time = 0.0;
    if (!velocity_update_time.empty()) {
        avg_velocity_update_time = std::accumulate(velocity_update_time.begin(), velocity_update_time.end(),
                                                   0.0) / static_cast<double>(velocity_update_time.size());
    }

    // average dynamic point removal time
    auto velocimeter_logger_ptr = fmcw_lio_ptr_->getLoggerVelocimetry();
    static std::vector<double> dynamic_point_removal_time;
    pushHistory(dynamic_point_removal_time,
                velocimeter_logger_ptr->dynamic_point_removal_time,
                kMaxKeep);
    double avg_dynamic_points_removal_time = 0.0;
    if (!dynamic_point_removal_time.empty()) {
        avg_dynamic_points_removal_time = std::accumulate(dynamic_point_removal_time.begin(), dynamic_point_removal_time.end(),
                                                          0.0) / static_cast<double>(dynamic_point_removal_time.size());
    }

    // average LiDAR velocity estimation time
    static std::vector<double> vel_est_time;
    pushHistory(vel_est_time,
                velocimeter_logger_ptr->vel_est_time,
                kMaxKeep);
    double avg_vel_est_time = 0.0;
    if (!vel_est_time.empty()) {
        avg_vel_est_time = std::accumulate(vel_est_time.begin(), vel_est_time.end(),
                                           0.0) / static_cast<double>(vel_est_time.size());
    }

    // average plane search time
    static std::vector<double> plane_search_time;
    pushHistory(plane_search_time,
                std::accumulate(update_logger_ptr->plane_search_time.begin(), update_logger_ptr->plane_search_time.end(), 0.0),
                kMaxKeep);
    double avg_plane_search_time = 0.0;
    if (!plane_search_time.empty()) {
        avg_plane_search_time = std::accumulate(plane_search_time.begin(), plane_search_time.end(),
                                                0.0) / static_cast<double>(plane_search_time.size());
    }

    // average point-plane-distance-based state update time
    static std::vector<double> geometric_update_time;
    pushHistory(geometric_update_time,
                std::accumulate(update_logger_ptr->update_time.begin(), update_logger_ptr->update_time.end(), 0.0),
                kMaxKeep);
    double avg_geometric_update_time = 0.0;
    if (!geometric_update_time.empty()) {
        avg_geometric_update_time = std::accumulate(geometric_update_time.begin(), geometric_update_time.end(),
                                                    0.0) / static_cast<double>(geometric_update_time.size());
    }

    // average processing time
    static std::vector<double> proc_time;
    pushHistory(proc_time,
                fmcw_lio_ptr_->getTimeProcessing(),
                kMaxKeep);
    double avg_proc_time = 0.0;
    if (!proc_time.empty()) {
        avg_proc_time = std::accumulate(proc_time.begin(), proc_time.end(),
                                        0.0) / static_cast<double>(proc_time.size());
    }

    // RAM usage
    double resident_set = 0.0;

    // get information from proc directory
    std::ifstream stat_stream("/proc/self/stat", std::ios_base::in);

    std::string pid, comm, state, ppid, pgrp, session, tty_nr;
    std::string tpgid, flags, minflt, cminflt, majflt, cmajflt;
    std::string utime, stime, cutime, cstime, priority, nice;
    std::string thread_num, itrealvalue, starttime;
    unsigned long vsize;
    long rss;

    stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr >> tpgid >> flags >> minflt >> cminflt
                >> majflt >> cmajflt >> utime >> stime >> cutime >> cstime >> priority >> nice >> thread_num
                >> itrealvalue >> starttime >> vsize >> rss;
    stat_stream.close();

    long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024;
    resident_set = static_cast<double>(rss * page_size_kb);

    // CPU usage
    struct tms time_sample{};
    clock_t now;
    double cpu_percent;

    // get CPU usage of current process
    now = times(&time_sample);

    static clock_t last_cpu = now;
    // CPU time spent in kernel mode when last called
    static clock_t last_sys_cpu = time_sample.tms_stime;
    // CPU time spent in user mode when last called
    static clock_t last_user_cpu = time_sample.tms_utime;

    if ((now <= last_cpu) ||
        (time_sample.tms_stime < last_sys_cpu) ||
        (time_sample.tms_utime < last_user_cpu)) {
        cpu_percent = -1.0;
    } else {
        cpu_percent = static_cast<double>((time_sample.tms_stime - last_sys_cpu) + (time_sample.tms_utime - last_user_cpu));
        cpu_percent /= static_cast<double>(now - last_cpu);
        // get CPU core number
        cpu_percent /= static_cast<double>(sysconf(_SC_NPROCESSORS_ONLN));
        cpu_percent *= 100.0;
    }

    last_cpu = now;
    last_sys_cpu = time_sample.tms_stime;
    last_user_cpu = time_sample.tms_utime;

    static std::vector<double> cpu_percents;
    pushHistory(cpu_percents,
                cpu_percent,
                kMaxKeep);
    double avg_cpu_usage = 0.0;
    if (!cpu_percents.empty()) {
        avg_cpu_usage = std::accumulate(cpu_percents.begin(), cpu_percents.end(),
                                        0.0) / static_cast<double>(cpu_percents.size());
    }

    // print to terminal
    printf("\033[2J\033[1;1H");

    // terminal output format
    const int total_space = 95;
    const int value_width = 8;

    // system neon
    const auto& spectrum = fmcw_lio_ptr_->neon.getRGBVec();

    constexpr double kSpeedAbsurd = 1000.0;

    // lambda function
    auto visibleWidth = [&](const std::string& str) -> std::size_t {
        std::string clean;
        bool in_escape = false;

        for (char c : str) {
            if (!in_escape) {
                if (c == '\033') {
                    in_escape = true;
                } else {
                    clean += c;
                }
            } else {
                if (c == 'm') {
                    in_escape = false;
                }
            }
        }

        std::size_t width = 0;
        try {
            std::wstring_convert<std::codecvt_utf8<wchar_t>> conv;
            std::wstring ws = conv.from_bytes(clean);

            for (wchar_t ch : ws) {
                int w = wcwidth(ch);
                width += ((w > 0) ? w : 1);
            }
        } catch (...) {
            width = clean.size();
        }

        return width;
    };

    auto convertValueToString = [](auto a_value, int n) -> std::string {
        double scale = std::pow(10.0, n);
        double truncated = std::trunc(a_value * scale) / scale;

        std::ostringstream out;
        out << std::fixed << std::setprecision(n) << truncated;

        return out.str();
    };

    auto formatValue = [](double val, int width, int precision) -> std::string {
        if (!std::isfinite(val)) {
            std::ostringstream oss;
            oss << std::right << std::setw(width) << "nan";

            return oss.str();
        }

        std::ostringstream oss;
        oss << std::fixed << std::right << std::setw(width)
            << std::setprecision(precision) << val;

        return oss.str();
    };

    auto printLine = [&](const std::string& label,
                         const std::vector<std::string>& values) -> void {
        std::ostringstream oss;
        oss << "| " << std::left << label << "= ";
        for (const auto& value : values) {
            oss << value << " ";
        }

        const auto current_len = static_cast<int>(visibleWidth(oss.str()));
        const int padding = std::max(0, total_space - current_len + 1);

        oss << std::string(static_cast<std::size_t>(padding), ' ') << "|";
        std::cout << oss.str() << std::endl;
    };

    auto printTime = [&](const std::string& label,
                         const std::vector<double>& data,
                         const double average_time,
                         int total_width) -> void {
        if (data.empty()) {
            return;
        }

        double last = data.back();
        double avg = average_time;
        double max = *std::max_element(data.begin(), data.end());

        std::ostringstream oss;
        oss << std::right << std::setprecision(2) << std::fixed;
        oss << " " << label << " = "
            << std::setw(value_width) << last
            << "  (Avg: "
            << std::setw(8) << avg
            << "  |  Max: "
            << std::setw(8) << max
            << ")";

        std::string content = oss.str();
        int padding = std::max(0, total_width - static_cast<int>(content.length()));
        content += std::string(static_cast<std::size_t>(padding), ' ');

        std::cout << "|" << content << "|" << std::endl;
    };

    auto printValue = [&](const std::string& label,
                          auto value,
                          int precision,
                          int value_width,
                          int total_width) -> void {
        std::ostringstream oss;
        oss << " " << label << " = ";

        using ValueType = std::decay_t<decltype(value)>;

        if constexpr (std::is_floating_point_v<ValueType>) {
            oss << std::right << std::setw(value_width)
                << convertValueToString(value, precision);
        } else {
            oss << std::right << std::setw(value_width)
                << value;
        }

        std::string content = oss.str();
        int padding = std::max(0, total_width - static_cast<int>(content.length()));
        content += std::string(static_cast<std::size_t>(padding), ' ');

        std::cout << "|" << content << "|" << std::endl;
    };

    auto colorizeFixedWidth = [&](const std::string& label,
                                  const std::string& label_format,
                                  const std::string& status,
                                  const std::string& color_code,
                                  int total_width,
                                  const std::string& extra) -> std::string {
        std::ostringstream oss;

        std::string colored_label = label_format + label + "\033[0m";
        std::string colored_status = color_code + status + "\033[0m";

        oss << colored_label << " ▸ [[ " << colored_status << extra << " ]]" << "\033[0m";
        std::string line = oss.str();

        std::size_t visual_length = visibleWidth(line);

        if (visual_length < static_cast<std::size_t>(total_width)) {
            line += std::string(static_cast<std::size_t>(total_width) - visual_length, ' ');
        }

        return ("| " + line + " |");
    };

    auto makeSpeedDashboard = [&](double speed,
                                  bool success) -> std::string {
        const double speed_min = 0.0;
        const double speed_max = 30.0;

        const int bar_width = 28;

        double v = speed;
        if (!std::isfinite(v)) {
            v = 0.0;
        }
        if (v < speed_min) {
            v = speed_min;
        }
        if (v > speed_max) {
            v = speed_max;
        }

        int filled = static_cast<int>(std::round((v - speed_min) / (speed_max - speed_min) * bar_width));
        filled = std::max(0, std::min(bar_width, filled));

        auto lerpRGB = [&](double t) -> std::array<std::uint8_t, 3> {
            if (spectrum.size() <= 1) {
                return spectrum.front();
            }

            double u = t * static_cast<double>(spectrum.size() - 1);
            int idx = static_cast<int>(std::floor(u));
            idx = std::max(0, std::min(static_cast<int>(spectrum.size() - 2), idx));
            double local = u - static_cast<double>(idx);

            const auto& a = spectrum[idx];
            const auto& b = spectrum[idx + 1];

            auto lerp = [&](std::uint8_t ca, std::uint8_t cb) -> std::uint8_t {
                double vv = static_cast<double>(ca) + (static_cast<double>(cb) - static_cast<double>(ca)) * local;

                return static_cast<std::uint8_t>(std::round(vv));
            };

            return {lerp(a[0], b[0]),
                    lerp(a[1], b[1]),
                    lerp(a[2], b[2])};
        };

        std::ostringstream bar;
        bar << "[";

        for (int i = 0; i < bar_width; ++i) {
            if (!success) {
                if (i < filled) {
                    bar << "\033[38;2;160;160;160m" << "█" << "\033[0m";
                } else {
                    bar << "\033[38;2;160;160;160m" << "░" << "\033[0m";
                }
            } else {
                if (i < filled) {
                    double t = static_cast<double>(i) / static_cast<double>(bar_width - 1);
                    auto rgb = lerpRGB(t);

                    bar << "\033[38;2;"
                        << static_cast<int>(rgb[0]) << ";"
                        << static_cast<int>(rgb[1]) << ";"
                        << static_cast<int>(rgb[2]) << "m"
                        << "█"
                        << "\033[0m";
                } else {
                    bar << "\033[38;2;160;160;160m" << "░" << "\033[0m";
                }
            }
        }

        bar << " " << std::fixed << std::setprecision(2) << std::setw(5) << v << " m/s";
        bar << "]";

        return bar.str();
    };

    auto printHeaderWithSpeedBar = [&](const std::string& left_part,
                                       const std::string& speed_bar) -> void {
        std::string left_core;
        {
            if (left_part.size() >= 2) {
                std::size_t start = left_part.find("| ");
                std::size_t end = left_part.rfind('|');
                if ((start != std::string::npos) &&
                    (end != std::string::npos) &&
                    (end > (start + 2))) {
                    left_core = left_part.substr(start + 2, end - (start + 2));
                } else {
                    left_core = left_part;
                }
            } else {
                left_core = left_part;
            }
        }

        const int inner_width = total_space - 1;

        std::string core = left_core;

        // remove trailing spaces in core (avoid pushing speed bar out of the border)
        while (!core.empty() && core.back() == ' ') {
            core.pop_back();
        }

        std::size_t w_left = visibleWidth(core);
        std::size_t w_bar = visibleWidth(speed_bar);

        int gap = static_cast<int>(inner_width) - static_cast<int>(w_left) - static_cast<int>(w_bar) - 1;
        if (gap < 1) {
            gap = 1;
        }

        std::string inner = core + std::string(static_cast<std::size_t>(gap), ' ') + speed_bar + " ";

        std::size_t w_inner = visibleWidth(inner);
        if (w_inner < static_cast<std::size_t>(inner_width)) {
            inner += std::string(static_cast<std::size_t>(inner_width) - w_inner, ' ');
        }

        std::cout << "| " << inner << "|" << std::endl;
    };

    // FMCW-LIO
    std::string spectrum_output;
    for (const auto& text_color : fmcw_lio_ptr_->neon.getStringRGBVec()) {
        auto [r, g, b] = text_color.second;

        spectrum_output += "\033[1;38;2;" +
                           std::to_string(r) + ";" +
                           std::to_string(g) + ";" +
                           std::to_string(b) + "m" +
                           text_color.first;
    }
    spectrum_output += "\033[0m";

    std::string content = spectrum_output;

    const int content_length = static_cast<int>(visibleWidth(content));
    const int inner = std::max(0, total_space - content_length);

    const int left_padding  = inner - (inner / 2);
    const int right_padding = inner / 2;

    std::cout << std::endl
              << "+" << std::string(static_cast<std::size_t>(total_space), '-') << "+" << std::endl;
    std::cout << "|" << std::string(static_cast<std::size_t>(left_padding), '-')
              << content
              << std::string(static_cast<std::size_t>(right_padding), '-') << "|" << std::endl;
    std::cout << "+" << std::string(static_cast<std::size_t>(total_space), '-') << "+" << std::endl;

    static ros::Time start_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();
    ros::Duration run_time = current_time - start_time;

    auto time_t_value_first = static_cast<std::time_t>(fmcw_lio_ptr_->getTimeFirstScanSys());
    std::tm time_info_first{};
    localtime_r(&time_t_value_first, &time_info_first);

    std::ostringstream oss;
    oss << "Data Time: " << std::put_time(&time_info_first, "%Y-%m-%d %H:%M:%S");
    std::string asc_time = oss.str();

    double seconds = run_time.toSec();
    std::ostringstream run_oss;
    run_oss << "Run Time: "
            << std::setw(11) << std::fixed << std::setprecision(4)
            << seconds << " s ";
    std::string run_str = run_oss.str();

    int space_count = static_cast<int>(total_space - asc_time.length() - run_str.length() - 1);
    space_count = std::max(0, space_count);

    std::cout << "| " << asc_time << std::string(static_cast<std::size_t>(space_count), ' ') << run_str << "|" << std::endl;

    // get cores and CPU type
    auto core_num = omp_get_num_procs();
    std::string cpu_type = getCPUType();

    if (!cpu_type.empty()) {
        std::cout << "| " << std::left << std::setfill(' ') << std::setw(total_space)
                  << cpu_type + " x " + std::to_string(core_num) << " |" << std::endl;
    }

    static std::vector<double> cores_utilized;
    pushHistory(cores_utilized,
                cpu_percent * core_num * 1.0e-2,
                kMaxKeep);
    double avg_cores_utilized = 0.0;
    if (!cores_utilized.empty()) {
        avg_cores_utilized = std::accumulate(cores_utilized.begin(), cores_utilized.end(),
                                             0.0) / static_cast<double>(cores_utilized.size());
    }

    std::cout << "|" << std::string(static_cast<std::size_t>(total_space), '=') << "|" << std::endl;

    // state information
    int state_value_width = 10;
    int state_precision = 4;
    std::cout << "| " << std::left << std::setfill(' ')
              << "\033[1;39m"
              << "\033[1;38;2;241;38;29m"
              << std::setw(total_space - 1)
              << "STATE"
              << "\033[0m"
              << "|"
              << std::endl;
    auto q_wb = Eigen::Quaterniond(state_ptr->getRotation());
    printLine("    Rotation           {q_wb}   [xyzw]          ",
              {formatValue(q_wb.x(), state_value_width, state_precision),
               formatValue(q_wb.y(), state_value_width, state_precision),
               formatValue(q_wb.z(), state_value_width, state_precision),
               formatValue(q_wb.w(), state_value_width, state_precision)});

    printLine("    Velocity           {v_wb_w} [xyz ] (m/s)    ",
              {formatValue(state_ptr->getVelocity().x(), state_value_width, state_precision),
               formatValue(state_ptr->getVelocity().y(), state_value_width, state_precision),
               formatValue(state_ptr->getVelocity().z(), state_value_width, state_precision)});

    printLine("    Position           {p_wb_w} [xyz ] (m)      ",
              {formatValue(state_ptr->getPosition().x(), state_value_width, state_precision),
               formatValue(state_ptr->getPosition().y(), state_value_width, state_precision),
               formatValue(state_ptr->getPosition().z(), state_value_width, state_precision)});

    printLine("    Gyroscope Bias     {bg_b}   [xyz ] (rad/s)  ",
              {formatValue(state_ptr->getGyroscopeBias().x(), state_value_width, state_precision),
               formatValue(state_ptr->getGyroscopeBias().y(), state_value_width, state_precision),
               formatValue(state_ptr->getGyroscopeBias().z(), state_value_width, state_precision)});

    printLine("    Accelerometer Bias {ba_b}   [xyz ] (m/s^2)  ",
              {formatValue(state_ptr->getAccelerometerBias().x(), state_value_width, state_precision),
               formatValue(state_ptr->getAccelerometerBias().y(), state_value_width, state_precision),
               formatValue(state_ptr->getAccelerometerBias().z(), state_value_width, state_precision)});

    printLine("    Gravity            {g_wb_w} [xyz ] (m/s^2)  ",
              {formatValue(state_ptr->getGravity().x(), state_value_width, state_precision),
               formatValue(state_ptr->getGravity().y(), state_value_width, state_precision),
               formatValue(state_ptr->getGravity().z(), state_value_width, state_precision)});

    std::cout << "|" << std::string(static_cast<std::size_t>(total_space), ' ') << "|" << std::endl;

    // initializer
    const auto initialization_logger_ptr = fmcw_lio_ptr_->getLoggerInitialization();
    if (config_ptr_->init_method == InitializationMethod::FreeInit) {
        std::string is_zero_vel_init = (initialization_logger_ptr->is_zero_velocity ? "Zero Velocity ✔" :
                                                                                      "Zero Velocity ✘");
        std::string is_zero_vel_color = (initialization_logger_ptr->is_zero_velocity ? "\033[38;2;100;255;200m" :
                                                                                       "\033[38;2;255;100;0m");

        std::string initialization_part = colorizeFixedWidth("INITIALIZER",
                                                             "\033[1;38;2;255;146;0m",
                                                             is_zero_vel_init,
                                                             is_zero_vel_color,
                                                             0,
                                                             "");

        double init_speed = initialization_logger_ptr->v_wb_b_0.norm();
        if ((!std::isfinite(init_speed)) || (init_speed > kSpeedAbsurd)) {
            init_speed = 0.0;
        }

        std::string init_speed_bar = makeSpeedDashboard(init_speed,
                                                        true);

        printHeaderWithSpeedBar(initialization_part,
                                init_speed_bar);

        printLine("    Angular Velocity   {ω_wb_b} [xyz] (rad/s)   ",
                  {formatValue(initialization_logger_ptr->omg_wb_b_0.x(), state_value_width, state_precision),
                   formatValue(initialization_logger_ptr->omg_wb_b_0.y(), state_value_width, state_precision),
                   formatValue(initialization_logger_ptr->omg_wb_b_0.z(), state_value_width, state_precision)});
        printLine("    Body Velocity      {v_wb_b} [xyz] (m/s)     ",
                  {formatValue(initialization_logger_ptr->v_wb_b_0.x(), state_value_width, state_precision),
                   formatValue(initialization_logger_ptr->v_wb_b_0.y(), state_value_width, state_precision),
                   formatValue(initialization_logger_ptr->v_wb_b_0.z(), state_value_width, state_precision)});
    } else {
        std::cout << "| " << std::left << std::setfill(' ')
                  << "\033[1;38;2;255;146;0m"
                  << std::setw(total_space - 1)
                  << "INITIALIZER"
                  << "\033[0m"
                  << "|"
                  << std::endl;
    }

    printValue("    Initialization Time (ms)                   ",
               initialization_logger_ptr->initialization_time,
               2,
               value_width,
               total_space);
    printValue("    IMU Measurement Number                     ",
               initialization_logger_ptr->imu_number,
               2,
               value_width,
               total_space);

    std::cout << "|" << std::string(static_cast<std::size_t>(total_space), ' ') << "|" << std::endl;

    // propagator
    std::cout << "| " << std::left << std::setfill(' ')
              << "\033[1;38;2;253;219;42m"
              << std::setw(total_space - 1)
              << "PROPAGATOR"
              << "\033[0m"
              << "|"
              << std::endl;
    printTime("    Propagation Time (ms)                      ",
              prop_time,
              avg_prop_time,
              total_space);

    std::cout << "|" << std::string(static_cast<std::size_t>(total_space), ' ') << "|" << std::endl;

    // compensator
    std::cout << "| " << std::left << std::setfill(' ')
              << "\033[1;38;2;77;228;43m"
              << std::setw(total_space - 1)
              << "COMPENSATOR"
              << "\033[0m"
              << "|"
              << std::endl;
    printTime("    Motion Compensation Time (ms)              ",
              motion_comp_time,
              avg_motion_comp_time,
              total_space);

    std::cout << "|" << std::string(static_cast<std::size_t>(total_space), ' ') << "|" << std::endl;

    // velocimeter
    if (config_ptr_->use_doppler) {
        std::string status = (velocimeter_logger_ptr->success ? "Success ✔" :
                                                                "Failure ✘");
        std::string status_color = (velocimeter_logger_ptr->success ? "\033[38;2;100;255;200m" :
                                                                      "\033[38;2;255;100;0m");

        std::string left_part = colorizeFixedWidth("VELOCIMETER",
                                                   "\033[1;38;2;15;246;216m",
                                                   status,
                                                   status_color,
                                                   0,
                                                   "");

        Eigen::Vector3d v_wl_l = fmcw_lio_ptr_->getVelocityEstimated();
        double speed = v_wl_l.norm();
        if ((!std::isfinite(speed)) || (speed > kSpeedAbsurd)) {
            v_wl_l.setZero();
            speed = 0.0;
        }

        std::string speed_bar = makeSpeedDashboard(speed,
                                                   velocimeter_logger_ptr->success);

        printHeaderWithSpeedBar(left_part,
                                speed_bar);

        printLine("    Estimated Velocity {v_wl_l} [xyz] (m/s)     ",
                  {formatValue(v_wl_l.x(), state_value_width, state_precision),
                   formatValue(v_wl_l.y(), state_value_width, state_precision),
                   formatValue(v_wl_l.z(), state_value_width, state_precision)});

        printTime("    Velocity Estimation Time (ms)              ",
                  vel_est_time,
                  avg_vel_est_time,
                  total_space);

        if (config_ptr_->use_dynamic_point_removal) {
            printTime("    Dynamic Point Removal Time (ms)            ",
                      dynamic_point_removal_time,
                      avg_dynamic_points_removal_time,
                      total_space);

            std::string static_point_size = std::to_string(velocimeter_logger_ptr->static_point_num);
            std::string dynamic_point_size = std::to_string(velocimeter_logger_ptr->dynamic_point_num);
            printValue("    Static Point Number                        ",
                       static_point_size,
                       0,
                       value_width,
                       total_space);
            printValue("    Dynamic Point Number                       ",
                       dynamic_point_size,
                       0,
                       value_width,
                       total_space);
        }

        std::cout << "|" << std::string(static_cast<std::size_t>(total_space), ' ') << "|" << std::endl;
    }

    // updater
    std::cout << "| " << std::left << std::setfill(' ')
              << "\033[1;38;2;11;216;249m"
              << std::setw(total_space - 1)
              << "UPDATER"
              << "\033[0m"
              << "|"
              << std::endl;

    if (config_ptr_->use_doppler) {
        printTime("    Update Time (Velocity Observation, ms)     ",
                  velocity_update_time,
                  avg_velocity_update_time,
                  total_space);
    }

    printTime("    Update Time (Geometric Observation, ms)    ",
              geometric_update_time,
              avg_geometric_update_time,
              total_space);

    printTime("    Point-Plane Correspondence Search Time (ms)",
              plane_search_time,
              avg_plane_search_time,
              total_space);

    const auto scan_down_lidar_ptr = fmcw_lio_ptr_->getScanDownSampledInLiDAR();
    std::size_t actual_iterations = update_logger_ptr->actual_iterations;

    oss.str("");
    oss.clear();
    oss << "|     Point-Plane Correspondence Number           = " << std::setw(value_width) << update_logger_ptr->eff_pts[actual_iterations];
    oss << "  (Total Point Number: " << std::setw(11) << scan_down_lidar_ptr->points.size() << ")";
    std::size_t current_len = oss.str().size();

    int padding = static_cast<int>(total_space - current_len + 1);
    padding = std::max(0, padding);

    oss << std::string(static_cast<std::size_t>(padding), ' ') << "|";
    std::cout << oss.str() << std::endl;

    printValue("    Point-Plane Residual Norm (m)              ",
               update_logger_ptr->res[actual_iterations],
               2,
               value_width,
               total_space);
    printValue("    Point-Plane Update Iteration Number        ",
               actual_iterations,
               0,
               value_width,
               total_space);

    std::cout << "|" << std::string(static_cast<std::size_t>(total_space), ' ') << "|" << std::endl;

    // metric
    std::cout << "| " << std::left << std::setfill(' ')
              << "\033[1;38;2;195;45;243m"
              << std::setw(total_space - 1)
              << "METRIC"
              << "\033[0m"
              << "|"
              << std::endl;

    static Eigen::Vector3d p_wb_w_0 = Eigen::Vector3d::Zero();
    printValue("    Distance (Traveled, m)                     ",
               distance_traveled,
               2,
               value_width,
               total_space);
    printValue("    Distance (End-to-End, m)                   ",
               (state_ptr->getPosition() - p_wb_w_0).norm(),
               2,
               value_width,
               total_space);

    std::cout << std::right << std::setprecision(2) << std::fixed;

    printTime("    Computation Time (ms)                      ",
              proc_time,
              avg_proc_time,
              total_space);
    printTime("    Cores Utilized (cores)                     ",
              cores_utilized,
              avg_cores_utilized,
              total_space);
    printTime("    CPU Load (%)                               ",
              cpu_percents,
              avg_cpu_usage,
              total_space);

    printValue("    RAM Allocation (MB)                        ",
               resident_set * 1.0e-3,
               2,
               value_width,
               total_space);

    std::cout << "+" << std::string(static_cast<std::size_t>(total_space), '-') << "+" << std::endl;
}

} // namespace fmcw_lio
