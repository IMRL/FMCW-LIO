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

#ifndef TYPE_HPP
#define TYPE_HPP

#include <variant>

#include <livox_ros_driver/CustomMsg.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include "lib/ikd-Tree/ikd_Tree.h"
#include "lib/ivox3d/ivox3d.h"

// fmcw_lio
namespace fmcw_lio {

// enum type
// LiDARType
enum class LiDARType {
    Aeva = 1,
    Hesai = 2,
    RoboSense = 3,
    Ouster = 4,
    Velodyne = 5,
    Livox = 6
};
// InitializationMethod
enum class InitializationMethod {
    StaticWithoutBias = 1,
    StaticWithBias = 2,
    FreeInit = 3
};
// IntegrationMethod
enum class IntegrationMethod {
    RungeKutta2 = 1,
    RungeKutta3 = 2,
    RungeKutta4 = 3,
    Mechanization = 4,
    ACI2 = 5
};
// PointWeightMethod
enum class PointWeightMethod {
    NearestNeighbor = 1,
    LinearInterpolation = 2,
    RightEndpoint = 3
};
// MapStructureType
enum class MapStructureType {
    ikdTree = 1,
    iVox = 2
};
// SamplingMethod
enum class SamplingMethod {
    Uniform = 1,
    CosWeight = 2
};
// RobustKernelType
enum class RobustKernelType {
    None = 1,
    Cauchy = 2
};

// point and point cloud related type
// PCL point type and point cloud type
using PointType = pcl::PointXYZINormal;
using PointCloudType = pcl::PointCloud<PointType>;
// point vector 1D and 2D
using PointVec1D = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
using PointVec2D = std::vector<std::vector<PointType, Eigen::aligned_allocator<PointType>>>;

// variant type
// message pointer variant
using MsgPtrVariant = std::variant<sensor_msgs::PointCloud2::Ptr,
                                   livox_ros_driver::CustomMsg::Ptr>;
// time field variant
using TimeFieldVariant = std::variant<double,
                                      float,
                                      std::uint32_t,
                                      std::int32_t>;
// initialization method variant
class StaticInit;
class FreeInit;
using InitMethodPtrVariant = std::variant<std::shared_ptr<StaticInit>
#ifdef BUILD_FREE_INIT
                                                                     ,
                                          std::shared_ptr<FreeInit>
#endif
                                                                   >;
// map structure variant
using IkdTreeType = KD_TREE<PointType>;
using IVoxType = IVox<3, IVoxNodeType::DEFAULT, PointType>;
using MapStructurePtrVariant = std::variant<std::shared_ptr<IkdTreeType>,
                                            std::shared_ptr<IVoxType>>;

// function type
// integrate function
class StateBase;
using IntegrateFunction = std::function<Eigen::VectorXd(const std::shared_ptr<const StateBase>&,
                                                        const Eigen::Vector3d&, const Eigen::Vector3d&,
                                                        const Eigen::Vector3d&, const Eigen::Vector3d&,
                                                        const double)>;
// point weight function
using PointWeightFunction = std::function<double(const double)>;
// sample function
using SampleFunction = std::function<std::vector<std::size_t>(const std::vector<std::unordered_map<std::string, double>>&,
                                                              const Eigen::Vector3d&)>;

// tuple type
// LiDAR tuple -> (0: time field name,
//                 1: time field type,
//                 2: time field unit scale,
//                 3: is time offset to scan begin,
//                 4: field name need to be stored in PointType intensity field)
using LiDARTuple = std::tuple<std::string,
                              TimeFieldVariant,
                              float,
                              bool,
                              std::string>;

// table type
// LiDAR table
using LiDARTable = std::unordered_map<LiDARType,
                                      std::shared_ptr<LiDARTuple>>;
// integration table
using IntegrationTable = std::unordered_map<IntegrationMethod,
                                            IntegrateFunction>;
// point weight table
using PointWeightTable = std::unordered_map<PointWeightMethod,
                                            PointWeightFunction>;
// sampling table
using SamplingTable = std::unordered_map<SamplingMethod,
                                         SampleFunction>;

// class type
// MeasPackLI
class MeasPackLI {
public:
    explicit MeasPackLI() : scan_beg_time(0.0),
                            scan_end_time(0.0) {
        // reset scan pointer
        this->scan_ptr.reset(new PointCloudType());
    };

    ~MeasPackLI() = default;

public:
    // scan begin timestamp
    double scan_beg_time;
    // scan end timestamp
    double scan_end_time;

    // IMU queue pointer
    std::deque<sensor_msgs::Imu::ConstPtr> imu_msg_ptr_queue;

    // LiDAR scan pointer
    PointCloudType::Ptr scan_ptr;
};

} // namespace fmcw_lio

#endif // TYPE_HPP
