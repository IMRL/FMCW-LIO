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

#ifndef UPDATER_HPP
#define UPDATER_HPP

#include "map/map.hpp"

// fmcw_lio
namespace fmcw_lio {

// LoggerUpdate
class LoggerUpdate {
public:
    explicit LoggerUpdate(const std::size_t iterations) {
        // initialize and resize
        std::size_t logger_size = iterations + 1;

        velocity_based_update_time = 0.0;
        actual_iterations = 0;
        res.resize(logger_size);
        eff_pts.resize(logger_size);
        update_time.resize(logger_size);
        plane_search_time.resize(logger_size);
    }

    ~LoggerUpdate() = default;

public:
    double velocity_based_update_time;
    std::size_t actual_iterations;
    std::vector<double> res;
    std::vector<std::size_t> eff_pts;
    std::vector<double> update_time;
    std::vector<double> plane_search_time;
};

// Updater
class Updater {
public:
    explicit Updater(const std::shared_ptr<const Config>& config_ptr);

    ~Updater() = default;

    // LiDAR velocity observation: update state by LiDAR velocity
    void updateByVelocity(const std::shared_ptr<StateBase>& state_ptr, Eigen::MatrixXd& P,
                          const Eigen::Vector3d& v_wl_l, const Eigen::Matrix3d& R,
                          const Eigen::Vector3d& omg_wb_b_scan_end_time);

    // LiDAR geometric observation: update state by point-plane distance
    std::size_t updateByPlane(const std::shared_ptr<StateBase>& state_ptr, Eigen::MatrixXd& P,
                              const PointCloudType::Ptr& scan_down_lidar_ptr, double R_point,
                              const std::shared_ptr<Map>& map_ptr, PointVec2D& nearest_points,
                              std::size_t correspondence_thresh);

private:
    // estimate local plane parameter
    [[nodiscard]] bool estimatePlane(Eigen::Vector4d& plane_param,
                                     const PointVec1D& points_near,
                                     double threshold) const;

public:
    std::shared_ptr<LoggerUpdate> logger_update_ptr;

private:
    // configuration pointer
    const std::shared_ptr<const Config> config_ptr_;
};

} // namespace fmcw_lio

#endif // UPDATER_HPP
