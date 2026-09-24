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

#ifndef FILTER_HPP
#define FILTER_HPP

#include "estimator/propagator.hpp"
#include "estimator/updater.hpp"

// fmcw_lio
namespace fmcw_lio {

// Filter
class Filter {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit Filter(const std::shared_ptr<const Config>& config_ptr) : propagator_ptr_(std::make_shared<Propagator>(config_ptr)),
                                                                       updater_ptr_(std::make_shared<Updater>(config_ptr)) {}

    ~Filter() = default;

    // initialize filter
    void initializeFilter(const std::shared_ptr<StateBase>& state_ptr, const Eigen::MatrixXd& P,
                          const std::shared_ptr<MeasPackLI>& meas_ptr) {
        // set initial covariance
        P_ = P;
        // initialize propagator with last measurement
        propagator_ptr_->initializePropagator(state_ptr,
                                              meas_ptr);
    }

    // filter propagation
    void propagateFilter(const std::shared_ptr<StateBase>& state_ptr,
                         const std::shared_ptr<MeasPackLI>& meas_ptr) {
        // state and covariance propagation via propagator
        propagator_ptr_->propagateStateAndCov(state_ptr, P_,
                                              meas_ptr);
    }

    // state update based on LiDAR velocity observation
    void updateByVelocity(const std::shared_ptr<StateBase>& state_ptr,
                          const Eigen::Vector3d& v_wl_l, const Eigen::Matrix3d& R_v) {
        // state update based on LiDAR velocity observation from Doppler velocity
        updater_ptr_->updateByVelocity(state_ptr, P_,
                                       v_wl_l, R_v,
                                       propagator_ptr_->getIMUScanEnd().first);
    }

    // state update based on LiDAR geometric observation
    std::size_t updateByPlane(const std::shared_ptr<StateBase>& state_ptr,
                              const PointCloudType::Ptr& scan_down_lidar_ptr, const double R_point,
                              const std::shared_ptr<Map>& map_ptr, PointVec2D& nearest_points,
                              const std::size_t correspondence_thresh) {
        // state update based on LiDAR geometric observation from point-plane distance
        return updater_ptr_->updateByPlane(state_ptr, P_,
                                           scan_down_lidar_ptr, R_point,
                                           map_ptr, nearest_points,
                                           correspondence_thresh);
    }

    // set callback function for imu-rate state publish
    void setTFPublishFunction(TFPublishFunction tf_publish_func) {
        // function for publishing imu-rate state
        propagator_ptr_->setTFPublishFunction(std::move(tf_publish_func));
    }

    // set covariance
    void setCovariance(const Eigen::MatrixXd& P) { P_ = P; }

    // get covariance
    [[nodiscard]] const Eigen::MatrixXd& getCovariance() const noexcept { return P_; }

    // get extrapolated IMU at scan end
    [[nodiscard]] std::pair<Eigen::Vector3d, Eigen::Vector3d> getIMUScanEnd() const noexcept {
        return propagator_ptr_->getIMUScanEnd();
    }

    // get compensated scan in LiDAR frame
    [[nodiscard]] PointCloudType::Ptr getScanCompensatedInLiDAR() const noexcept {
        return propagator_ptr_->getScanCompensatedInLiDAR();
    }

    // get propagation logger
    [[nodiscard]] std::shared_ptr<LoggerPropagation> getLoggerPropagation() const noexcept {
        return propagator_ptr_->logger_propagation_ptr;
    }

    // get compensation logger
    [[nodiscard]] std::shared_ptr<LoggerCompensation> getLoggerCompensation() const noexcept {
        return propagator_ptr_->getLoggerCompensation();
    }

    // get update logger
    [[nodiscard]] std::shared_ptr<LoggerUpdate> getLoggerUpdate() const noexcept {
        return updater_ptr_->logger_update_ptr;
    }

private:
    // propagator pointer
    const std::shared_ptr<Propagator> propagator_ptr_;
    // updater pointer
    const std::shared_ptr<Updater> updater_ptr_;

    // state covariance matrix
    Eigen::MatrixXd P_;
};

} // namespace fmcw_lio

#endif // FILTER_HPP
