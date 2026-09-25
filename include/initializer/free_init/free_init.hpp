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

#ifndef FREE_INIT_HPP
#define FREE_INIT_HPP

#include <pcl/filters/uniform_sampling.h>

#include "initializer/free_init/div.hpp"
#include "initializer/free_init/optimizer_bag.hpp"

// fmcw_lio
namespace fmcw_lio {

// FreeInit
class FreeInit {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit FreeInit(const std::shared_ptr<const Config>& config_ptr);

    ~FreeInit() = default;

    // bootstrap velocity with given measurement
    void bootstrapVelocity(const std::shared_ptr<MeasPackLI>& meas_ptr);

    // solve least square problem to estimate velocity and covariance
    bool solveLSQ(const Eigen::MatrixXd& lsq_data, const Eigen::VectorXd& weight,
                  Eigen::Vector3d& v_wl_l, Eigen::Matrix3d& R_v);

    // initialize state, covariance, and map
    [[nodiscard]] std::size_t initializeStateCovMap(const std::shared_ptr<MeasPackLI>& meas_ptr);

    // get state
    [[nodiscard]] std::shared_ptr<StateBase> getState() const noexcept { return state_ptr_; }

    // get covariance
    [[nodiscard]] Eigen::MatrixXd getCovariance() const noexcept { return P_; }

    // get static point cloud map in world frame
    [[nodiscard]] PointCloudType::Ptr getMap() const noexcept { return map_ptr_; }

    // get angular velocity
    [[nodiscard]] Eigen::Vector3d getAngularVelocity() const noexcept { 
        return div_ptr_->getAngularVelocity(); 
    }

    // get body velocity
    [[nodiscard]] Eigen::Vector3d getBodyVelocity() const noexcept {
        return div_ptr_->getBodyVelocity();
    }

    // get poses during initialization
    [[nodiscard]] std::shared_ptr<std::vector<std::shared_ptr<StatePoseQuat>>> getPoses() const noexcept {
        return pose_ptr_vec_ptr_;
    }

    // get is zero velocity in initialization
    [[nodiscard]] bool getIsZeroVelocityInit() const noexcept {
        return (div_ptr_->getStateType() == StateDIV::StateType::GyroBias);
    }

private:
    // span frame with two given axis direction
    [[nodiscard]] static std::array<Eigen::Vector3d, 3> spanFrame(const Eigen::Vector3d& axis_z,
                                                                  const Eigen::Vector3d& axis_x_ref);

    // sort measurement
    std::shared_ptr<std::vector<std::shared_ptr<MeasDIV>>> sortMeas(const std::shared_ptr<MeasPackLI>& meas_ptr);

    // estimate accelerometer bias and gravity
    void estimateAccelerometerBiasAndGravity();

    // set initialized state and covariance in new defined world frame
    void setStateAndCov(const Eigen::Matrix3d& R_align_mat);

    // transform pose and map point
    void transformPosesAndMapPoints(const Eigen::Matrix3d& R_align_mat);

private:
    // configuration pointer
    const std::shared_ptr<const Config> config_ptr_;

    // point cloud filter
    pcl::UniformSampling<PointType> uniform_filter_scan_;

    // DIV pointer
    const std::shared_ptr<DIV> div_ptr_;

    // accelerometer bias and gravity optimizer
    const std::shared_ptr<OptimizerBaG> optimizer_bag_ptr_;

    // maximum number of IMU data for initialization
    const std::size_t max_imu_num_;

    // pose (during initialization) vector pointer
    std::shared_ptr<std::vector<std::shared_ptr<StatePoseQuat>>> pose_ptr_vec_ptr_;

    // initialized state
    const std::shared_ptr<StateBase> state_ptr_;
    // initialized covariance
    Eigen::MatrixXd P_;
    // initialized static point cloud map in world frame aligned with gravity
    PointCloudType::Ptr map_ptr_;
};

} // namespace fmcw_lio

#endif // FREE_INIT_HPP
