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

#ifndef CONFIG_HPP
#define CONFIG_HPP

#include "common/type.hpp"

// fmcw_lio
namespace fmcw_lio {

// Config
class Config {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit Config(ros::NodeHandle& nh) {
        // load configuration
        // pose input and output
        nh.param<bool>("pose_io/dump", dump_pose, false);
        nh.param<std::string>("pose_io/dir_path", pose_dir_path, "/tmp");

        // map input and output
        nh.param<bool>("map_io/dump", dump_map, false);
        nh.param<std::string>("map_io/dir_path", map_dir_path, "/tmp");

        // ground truth input and output
        nh.param<bool>("ground_truth_io/display", display_gt, false);
        nh.param<std::string>("ground_truth_io/file_path", gt_file_path, "/tmp/ground_truth.txt");

        // common
        nh.param<std::string>("common/topic_imu", topic_imu, "/imu/data");
        nh.param<std::string>("common/topic_lidar", topic_lidar, "/aeva/AeriesII/point_cloud_frame");
        // time difference of LiDAR w.r.t. IMU
        nh.param<double>("common/time_diff_bl", time_diff_bl, 0.0);
        // extrinsic parameter of LiDAR frame w.r.t. IMU frame
        std::vector<double> R_bl_vec;
        nh.param<std::vector<double>>("common/R_bl", R_bl_vec, std::vector<double>(9));
        R_bl = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(R_bl_vec.data());
        std::vector<double> p_bl_b_vec;
        nh.param<std::vector<double>>("common/p_bl_b", p_bl_b_vec, std::vector<double>(3));
        p_bl_b = Eigen::Map<const Eigen::Vector3d>(p_bl_b_vec.data());
        // use Doppler velocity
        nh.param<bool>("common/use_doppler", use_doppler, false);

        // IMU preprocess
        nh.param<double>("imu_preprocess/gravity_scale", gravity_scale, 9.81);
        nh.param<bool>("imu_preprocess/scaling", scaling, false);

        // scan preprocess
        // { Aeva = 1, Hesai = 2, RoboSense = 3, Ouster = 4, Velodyne = 5, Livox = 6 }
        int lidar_temp;
        nh.param<int>("scan_preprocess/lidar_type", lidar_temp, 1);
        lidar_type = static_cast<LiDARType>(lidar_temp);
        nh.param<double>("scan_preprocess/scan_rate", scan_rate, 10);
        nh.param<double>("scan_preprocess/skip_radius", skip_radius, 0.5);
        skip_radius_squared = skip_radius * skip_radius;
        int point_select_interval_lio_temp;
        nh.param<int>("scan_preprocess/point_select_interval_lio", point_select_interval_lio_temp, 6);
        point_select_interval_lio = static_cast<std::size_t>(point_select_interval_lio_temp);
        // is the scan message timestamp the scan end timestamp
        nh.param<bool>("scan_preprocess/scan_end_timestamp", scan_end_timestamp, false);

        // initialization
        // { StaticWithoutBias = 1, StaticWithBias = 2, FreeInit = 3 }
        int init_method_temp;
        nh.param<int>("initialization/init_method", init_method_temp, 2);
        init_method = static_cast<InitializationMethod>(init_method_temp);
        int max_imu_num_temp;
        nh.param<int>("initialization/max_imu_num", max_imu_num_temp, 100);
        max_imu_num = static_cast<std::size_t>(max_imu_num_temp);
        int point_select_interval_init_temp;
        nh.param<int>("initialization/point_select_interval_init", point_select_interval_init_temp, 6);
        point_select_interval_init = static_cast<std::size_t>(point_select_interval_init_temp);
        point_select_interval = point_select_interval_init;
        nh.param<bool>("initialization/init_mapping", init_mapping, false);
        // Free-Init
        nh.param<int>("initialization/point_num_thresh", point_num_thresh, 36);
        // velocity bootstrap
        nh.param<bool>("initialization/use_bootstrap_velocity", use_bootstrap_velocity, false);
        nh.param<double>("initialization/voxel_size_bootstrap", voxel_size_bootstrap, 2.0);
        int iteration_num_bootstrap_temp;
        nh.param<int>("initialization/iteration_num_bootstrap", iteration_num_bootstrap_temp, 3);
        iteration_num_bootstrap = static_cast<std::size_t>(iteration_num_bootstrap_temp);
        nh.param<double>("initialization/max_cond_num_bootstrap", max_cond_num_bootstrap, 1000.0);
        nh.param<double>("initialization/detect_dynamic_threshold_bootstrap", detect_dynamic_threshold_bootstrap, 1.5);
        // prior IMU bias
        std::vector<double> gyroscope_bias_prior_vec;
        nh.param<std::vector<double>>("initialization/gyroscope_bias_prior", gyroscope_bias_prior_vec, std::vector<double>(3));
        gyroscope_bias_prior = Eigen::Map<const Eigen::Vector3d>(gyroscope_bias_prior_vec.data());
        std::vector<double> accelerometer_bias_prior_vec;
        nh.param<std::vector<double>>("initialization/accelerometer_bias_prior", accelerometer_bias_prior_vec, std::vector<double>(3));
        accelerometer_bias_prior = Eigen::Map<const Eigen::Vector3d>(accelerometer_bias_prior_vec.data());
        // DIV initial covariance
        std::vector<double> angular_velocity_cov_init_div_vec;
        nh.param<std::vector<double>>("initialization/angular_velocity_cov_init_div", angular_velocity_cov_init_div_vec, std::vector<double>(3));
        angular_velocity_cov_init_div = Eigen::Map<const Eigen::Vector3d>(angular_velocity_cov_init_div_vec.data());
        std::vector<double> velocity_cov_init_div_vec;
        nh.param<std::vector<double>>("initialization/body_velocity_cov_init_div", velocity_cov_init_div_vec, std::vector<double>(3));
        body_velocity_cov_init_div = Eigen::Map<const Eigen::Vector3d>(velocity_cov_init_div_vec.data());
        std::vector<double> gyroscope_bias_cov_init_div_vec;
        nh.param<std::vector<double>>("initialization/gyroscope_bias_cov_init_div", gyroscope_bias_cov_init_div_vec, std::vector<double>(3));
        gyroscope_bias_cov_init_div = Eigen::Map<const Eigen::Vector3d>(gyroscope_bias_cov_init_div_vec.data());
        // DIV propagation covariance
        std::vector<double> angular_velocity_cov_prop_div_vec;
        nh.param<std::vector<double>>("initialization/angular_velocity_cov_prop_div", angular_velocity_cov_prop_div_vec, std::vector<double>(3));
        angular_velocity_cov_prop_div = Eigen::Map<const Eigen::Vector3d>(angular_velocity_cov_prop_div_vec.data());
        std::vector<double> velocity_cov_prop_div_vec;
        nh.param<std::vector<double>>("initialization/body_velocity_cov_prop_div", velocity_cov_prop_div_vec, std::vector<double>(3));
        body_velocity_cov_prop_div = Eigen::Map<const Eigen::Vector3d>(velocity_cov_prop_div_vec.data());
        std::vector<double> gyroscope_bias_cov_prop_div_vec;
        nh.param<std::vector<double>>("initialization/gyroscope_bias_cov_prop_div", gyroscope_bias_cov_prop_div_vec, std::vector<double>(3));
        gyroscope_bias_cov_prop_div = Eigen::Map<const Eigen::Vector3d>(gyroscope_bias_cov_prop_div_vec.data());
        // DIV update covariance
        std::vector<double> angular_rate_cov_div_vec;
        nh.param<std::vector<double>>("initialization/angular_rate_cov_div", angular_rate_cov_div_vec, std::vector<double>(3));
        angular_rate_cov_div = Eigen::Map<const Eigen::Vector3d>(angular_rate_cov_div_vec.data());
        nh.param<double>("initialization/doppler_velocity_cov_div", doppler_velocity_cov_div, 0.1);
        // IMU measurement number threshold for world frame definition
        int imu_num_thresh_world_frame_temp;
        nh.param<int>("initialization/imu_num_thresh_world_frame", imu_num_thresh_world_frame_temp, 20);
        imu_num_thresh_world_frame = static_cast<std::size_t>(imu_num_thresh_world_frame_temp);
        // zero velocity detection
        nh.param<double>("initialization/zero_angular_velocity_thresh", zero_angular_velocity_thresh_init, 0.15);
        nh.param<double>("initialization/zero_velocity_thresh", zero_velocity_thresh_init, 0.3);
        // IMU correlation time (s)
        double imu_correlation_time;
        nh.param<double>("initialization/imu_correlation_time", imu_correlation_time, 3600.0);
        imu_correlation_time_inv = 1.0 / imu_correlation_time;
        // dynamic point removal
        nh.param<bool>("initialization/use_dynamic_point_removal", use_dynamic_point_removal_init, true);
        nh.param<double>("initialization/remove_dynamic_threshold", remove_dynamic_threshold_init, 1.0);
        // optimization
        nh.param<double>("initialization/accelerometer_bias_regularization_weight", accelerometer_bias_regularization_weight, 10.0);
        nh.param<double>("initialization/cauchy_loss_scale", cauchy_loss_scale, 100.0);
        int thread_num_temp;
        nh.param<int>("initialization/thread_num", thread_num_temp, 4);
        thread_num = static_cast<std::size_t>(thread_num_temp);
        int max_iteration_num_temp;
        nh.param<int>("initialization/max_iteration_num", max_iteration_num_temp, 50);
        max_iteration_num = static_cast<std::size_t>(max_iteration_num_temp);
        nh.param<double>("initialization/max_solver_time_in_seconds", max_solver_time_in_seconds, 0.1);

        // propagation
        // { RungeKutta2 = 1, RungeKutta3 = 2, RungeKutta4 = 3, Mechanization = 4, ACI2 = 5 }
        int integration_method_temp;
        nh.param<int>("propagation/integration_method", integration_method_temp, 1);
        integration_method = static_cast<IntegrationMethod>(integration_method_temp);
        // { RungeKutta2 = 1, RungeKutta3 = 2, RungeKutta4 = 3, Mechanization = 4, ACI2 = 5 }
        int compensation_method_temp;
        nh.param<int>("propagation/compensation_method", compensation_method_temp, 1);
        compensation_method = static_cast<IntegrationMethod>(compensation_method_temp);
        // { NearestNeighbor = 1, LinearInterpolation = 2, RightEndpoint = 3 }
        int point_weight_method_temp;
        nh.param<int>("propagation/point_weight_method", point_weight_method_temp, 1);
        point_weight_method = static_cast<PointWeightMethod>(point_weight_method_temp);
        // IMU parameter
        nh.param<double>("propagation/gyroscope_cov", gyroscope_cov, 0.1);
        nh.param<double>("propagation/accelerometer_cov", accelerometer_cov, 0.1);
        nh.param<double>("propagation/gyroscope_bias_cov", gyroscope_bias_cov, 0.0001);
        nh.param<double>("propagation/accelerometer_bias_cov", accelerometer_bias_cov, 0.0001);

        // update
        int iteration_num_temp;
        nh.param<int>("update/iteration_num", iteration_num_temp, 3);
        iteration_num = static_cast<std::size_t>(iteration_num_temp);
        int nearest_point_num_temp;
        nh.param<int>("update/nearest_point_num", nearest_point_num_temp, 5);
        nearest_point_num = static_cast<std::size_t>(nearest_point_num_temp);
        nh.param<double>("update/point_cov", point_cov, 0.001);

        // mapping
        // { ikdTree = 1, iVox = 2 }
        int map_structure_type_temp;
        nh.param<int>("mapping/map_structure_type", map_structure_type_temp, 1);
        map_structure_type = static_cast<MapStructureType>(map_structure_type_temp);
        nh.param<double>("mapping/voxel_filter_size_scan", voxel_filter_size_scan, 0.5);
        // ikd-Tree parameter
        nh.param<double>("mapping/detection_range", detection_range, 150.0);
        nh.param<double>("mapping/map_region_length", map_region_length, 1000.0);
        nh.param<double>("mapping/voxel_filter_size_map", voxel_filter_size_map, 0.5);
        // iVox parameter
        nh.param<double>("mapping/ivox_resolution", ivox_resolution, 0.5);
        // { CENTER = 1, NEARBY6 = 2, NEARBY18 = 3, NEARBY26 = 4 }
        int ivox_nearby_type_temp;
        nh.param<int>("mapping/nearby_type", ivox_nearby_type_temp, 2);
        ivox_nearby_type = static_cast<IVoxType::NearbyType>(ivox_nearby_type_temp);

        // publishing
        nh.param<bool>("publishing/publish_dense_scan", publish_dense_scan, false);
        nh.param<bool>("publishing/publish_tf_imu_rate", publish_tf_imu_rate, false);

        // velocimeter
        // min and max point distance from LiDAR origin
        nh.param<double>("velocimeter/min_point_dist", min_point_dist, 0.5);
        nh.param<double>("velocimeter/max_point_dist", max_point_dist, 150.0);
        // min and max point height (z value) in LiDAR frame
        nh.param<double>("velocimeter/min_point_z", min_point_z, -100.0);
        nh.param<double>("velocimeter/max_point_z", max_point_z, 100.0);
        // threshold for valid azimuth and elevation angle (degree)
        nh.param<double>("velocimeter/azimuth_thresh_deg", azimuth_thresh_deg, 55);
        nh.param<double>("velocimeter/elevation_thresh_deg", elevation_thresh_deg, 15);
        // zero velocity detection and update
        nh.param<double>("velocimeter/zero_velocity_thresh", zero_velocity_thresh, 0.10);
        std::vector<double> zero_velocity_sigma_vec;
        nh.param<std::vector<double>>("velocimeter/zero_velocity_sigma", zero_velocity_sigma_vec, std::vector<double>(3));
        zero_velocity_sigma = Eigen::Map<const Eigen::Vector3d>(zero_velocity_sigma_vec.data());
        // result filtering
        nh.param<double>("velocimeter/max_cond_num", max_cond_num, 1000.0);
        std::vector<double> max_sigma_vec;
        nh.param<std::vector<double>>("velocimeter/max_sigma", max_sigma_vec, std::vector<double>(3));
        max_sigma = Eigen::Map<const Eigen::Vector3d>(max_sigma_vec.data());
        // LSQ parameter
        nh.param<bool>("velocimeter/use_cos_weight", use_cos_weight, false);
        // { None = 1, Cauchy = 2 }
        int robust_kernel_temp;
        nh.param<int>("velocimeter/robust_kernel", robust_kernel_temp, 1);
        robust_kernel = static_cast<RobustKernelType>(robust_kernel_temp);
        // sigma offset
        std::vector<double> vel_sigma_offset_vec;
        nh.param<std::vector<double>>("velocimeter/vel_sigma_offset", vel_sigma_offset_vec, std::vector<double>(3));
        vel_sigma_offset = Eigen::Map<const Eigen::Vector3d>(vel_sigma_offset_vec.data());
        // RANSAC parameter
        nh.param<bool>("velocimeter/use_ransac", use_ransac, false);
        nh.param<double>("velocimeter/outlier_prob", outlier_prob, 0.3);
        nh.param<double>("velocimeter/success_prob", success_prob, 0.999);
        int ransac_point_num_temp;
        nh.param<int>("velocimeter/ransac_point_num", ransac_point_num_temp, 3);
        ransac_point_num = static_cast<std::size_t>(ransac_point_num_temp);
        nh.param<double>("velocimeter/inlier_thresh", inlier_thresh, 0.1);
        // { Uniform = 1, CosWeight = 2 }
        int sampling_method_temp;
        nh.param<int>("velocimeter/sampling_method", sampling_method_temp, 2);
        sampling_method = static_cast<SamplingMethod>(sampling_method_temp);
        int azimuth_interval_num_temp, elevation_interval_num_temp;
        nh.param<int>("velocimeter/azimuth_interval_num", azimuth_interval_num_temp, 10);
        nh.param<int>("velocimeter/elevation_interval_num", elevation_interval_num_temp, 3);
        azimuth_interval_num = static_cast<std::size_t>(azimuth_interval_num_temp);
        elevation_interval_num = static_cast<std::size_t>(elevation_interval_num_temp);
        // dynamic point removal
        nh.param<bool>("velocimeter/use_dynamic_point_removal", use_dynamic_point_removal, true);
        nh.param<bool>("velocimeter/pre_remove", pre_remove, false);
        nh.param<double>("velocimeter/remove_dynamic_threshold", remove_dynamic_threshold, 1.0);
    }

    ~Config() = default;

public:
    // pose input and output
    bool dump_pose{};
    std::string pose_dir_path;

    // map input and output
    bool dump_map{};
    std::string map_dir_path;

    // ground truth input and output
    bool display_gt{};
    std::string gt_file_path;

    // common
    std::string topic_imu;
    std::string topic_lidar;
    // time difference of LiDAR w.r.t. IMU
    double time_diff_bl{};
    // extrinsic parameter of LiDAR frame w.r.t. IMU frame
    Eigen::Matrix3d R_bl;
    Eigen::Vector3d p_bl_b;
    // use Doppler velocity
    bool use_doppler{};

    // IMU preprocess
    double gravity_scale{};
    bool scaling{};

    // scan preprocess
    // { Aeva = 1, Hesai = 2, RoboSense = 3, Ouster = 4, Velodyne = 5, Livox = 6 }
    LiDARType lidar_type{};
    double scan_rate{};
    double skip_radius{};
    double skip_radius_squared{};
    std::size_t point_select_interval_lio{};
    std::size_t point_select_interval{};
    // is the scan message timestamp the scan end timestamp
    bool scan_end_timestamp{};

    // initialization
    // { StaticWithoutBias = 1, StaticWithBias = 2, FreeInit = 3 }
    InitializationMethod init_method{};
    std::size_t max_imu_num{};
    std::size_t point_select_interval_init{};
    bool init_mapping{};
    // Free-Init
    int point_num_thresh{};
    // velocity bootstrap
    bool use_bootstrap_velocity{};
    double voxel_size_bootstrap{};
    std::size_t iteration_num_bootstrap{};
    double max_cond_num_bootstrap{};
    double detect_dynamic_threshold_bootstrap{};
    // prior IMU bias
    Eigen::Vector3d gyroscope_bias_prior;
    Eigen::Vector3d accelerometer_bias_prior;
    Eigen::Vector3d gyroscope_bias_cov_init_div;
    // initial covariance
    Eigen::Vector3d angular_velocity_cov_init_div;
    Eigen::Vector3d body_velocity_cov_init_div;
    // DIV propagation covariance
    Eigen::Vector3d angular_velocity_cov_prop_div;
    Eigen::Vector3d body_velocity_cov_prop_div;
    Eigen::Vector3d gyroscope_bias_cov_prop_div;
    // DIV update covariance
    Eigen::Vector3d angular_rate_cov_div;
    double doppler_velocity_cov_div{};
    // IMU measurement number threshold for world frame definition
    std::size_t imu_num_thresh_world_frame{};
    // zero velocity detection
    double zero_angular_velocity_thresh_init{};
    double zero_velocity_thresh_init{};
    // IMU correlation time (s)
    double imu_correlation_time_inv{};
    // dynamic point removal
    bool use_dynamic_point_removal_init{};
    double remove_dynamic_threshold_init{};
    // optimization
    double accelerometer_bias_regularization_weight{};
    double cauchy_loss_scale{};
    std::size_t thread_num{};
    std::size_t max_iteration_num{};
    double max_solver_time_in_seconds{};

    // propagation
    // { RungeKutta2 = 1, RungeKutta3 = 2, RungeKutta4 = 3, Mechanization = 4, ACI2 = 5 }
    IntegrationMethod integration_method{};
    // { RungeKutta2 = 1, RungeKutta3 = 2, RungeKutta4 = 3, Mechanization = 4, ACI2 = 5 }
    IntegrationMethod compensation_method{};
    // { NearestNeighbor = 1, LinearInterpolation = 2, RightEndpoint = 3 }
    PointWeightMethod point_weight_method{};
    // IMU parameter
    double gyroscope_cov{};
    double accelerometer_cov{};
    double gyroscope_bias_cov{};
    double accelerometer_bias_cov{};

    // update
    std::size_t iteration_num{};
    std::size_t nearest_point_num{};
    double point_cov{};

    // mapping
    // { ikdTree = 1, iVox = 2 }
    MapStructureType map_structure_type{};
    double voxel_filter_size_scan{};
    // ikd-Tree parameter
    double detection_range{};
    double map_region_length{};
    double voxel_filter_size_map{};
    // iVox parameter
    double ivox_resolution{};
    // { CENTER = 1, NEARBY6 = 2, NEARBY18 = 3, NEARBY26 = 4 }
    IVoxType::NearbyType ivox_nearby_type{};

    // publishing
    bool publish_dense_scan{};
    bool publish_tf_imu_rate{};

    // velocimeter
    // min and max point distance from LiDAR origin
    double min_point_dist{};
    double max_point_dist{};
    // min and max point height (z value) in LiDAR frame
    double min_point_z{};
    double max_point_z{};
    // threshold for valid azimuth and elevation angle (degree)
    double azimuth_thresh_deg{};
    double elevation_thresh_deg{};
    // zero velocity detection and update
    double zero_velocity_thresh{};
    Eigen::Vector3d zero_velocity_sigma;
    // result filtering
    double max_cond_num{};
    Eigen::Vector3d max_sigma;
    // LSQ parameter
    bool use_cos_weight{};
    // { None = 1, Cauchy = 2 }
    RobustKernelType robust_kernel{};
    // sigma offset
    Eigen::Vector3d vel_sigma_offset;
    // RANSAC parameter
    bool use_ransac{};
    double outlier_prob{};
    double success_prob{};
    std::size_t ransac_point_num{};
    double inlier_thresh{};
    // { Uniform = 1, CosWeight = 2 }
    SamplingMethod sampling_method{};
    std::size_t azimuth_interval_num{};
    std::size_t elevation_interval_num{};
    // dynamic point removal
    bool use_dynamic_point_removal{};
    bool pre_remove{};
    double remove_dynamic_threshold{};
};

} // namespace fmcw_lio

#endif // CONFIG_HPP
