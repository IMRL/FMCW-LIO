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

#ifndef COMPENSATOR_HPP
#define COMPENSATOR_HPP

#include "state/state.hpp"

// fmcw_lio
namespace fmcw_lio {

// LoggerCompensation
class LoggerCompensation {
public:
    LoggerCompensation() = default;

    ~LoggerCompensation() = default;

public:
    double compensation_time;
};

// Compensator
class Compensator {
public:
    explicit Compensator(const std::shared_ptr<const Config>& config_ptr,
                         IntegrateFunction compensate_func);

    ~Compensator() = default;

    // compensate scan point cloud using extrinsic parameter and IMU data
    void compensateScan(const std::shared_ptr<StateBase>& state_ptr,
                        const std::shared_ptr<const std::vector<std::shared_ptr<const StateBase>>>& propagation_states_ptr,
                        const PointCloudType::Ptr& scan_compensated_ptr);

private:
    // point weight function
    // nearest neighbor
    static double weightNearestNeighbor(const double time_ratio) { return ((time_ratio > 0.5) ? 1.0 : 0.0); }
    // linear interpolation
    static double weightLinearInterpolation(const double time_ratio) { return time_ratio; }
    // right endpoint
    static double weightRightEndpoint(const double time_ratio) { return 1.0; }

    // weight IMU angular rate and specific force measurement for point
    inline std::pair<Eigen::Vector3d, Eigen::Vector3d> weightIMUMeas(const Eigen::Vector3d& omg_head, const Eigen::Vector3d& f_head,
                                                                     const Eigen::Vector3d& omg_tail, const Eigen::Vector3d& f_tail,
                                                                     const double dt_imu = 5.0e-3,
                                                                     const double dt = 0.0) {
        // time duration ratio
        double time_ratio = dt / dt_imu;
        double weight = point_weight_func_(time_ratio);

        Eigen::Vector3d omg_i = ((1.0 - weight) * omg_head) + (weight * omg_tail);
        Eigen::Vector3d f_i = ((1.0 - weight) * f_head) + (weight * f_tail);

        return {omg_i, f_i};
    }

private:
    // point weight function table
    inline static const PointWeightTable point_weight_table_ = {
        // nearest neighbor
        {
            PointWeightMethod::NearestNeighbor,
            &weightNearestNeighbor
        },
        // linear interpolation
        {
            PointWeightMethod::LinearInterpolation,
            &weightLinearInterpolation
        },
        // right endpoint
        {
            PointWeightMethod::RightEndpoint,
            &weightRightEndpoint
        }
    };

public:
    std::shared_ptr<LoggerCompensation> logger_compensation_ptr;

private:
    // configuration pointer
    const std::shared_ptr<const Config> config_ptr_;

    // integrate function
    IntegrateFunction integrate_func_;

    // point weight function
    PointWeightFunction point_weight_func_;
};

} // namespace fmcw_lio

#endif // COMPENSATOR_HPP
