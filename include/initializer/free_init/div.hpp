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

#ifndef DIV_HPP
#define DIV_HPP

#include "state/state.hpp"

// fmcw_lio
namespace fmcw_lio {

// MeasDIV
class MeasDIV {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // MeasType
    enum class MeasType {
        // angular rate from gyroscope
        AngularRate = 1,
        // Doppler velocity from Doppler LiDAR point
        DopplerVelocity = 2
    };

public:
    MeasDIV() : meas_type{},
                timestamp{} {}

    ~MeasDIV() = default;

public:
    // measurement type
    MeasType meas_type;

    // timestamp in milliseconds
    double timestamp;
    // angular rate from gyroscope
    Eigen::Vector3d angular_rate;
    // specific force from accelerometer
    Eigen::Vector3d specific_force;
    // 4D LiDAR point from Doppler LiDAR
    PointType point;
};

// StateDIV
class StateDIV {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // StateType
    enum class StateType {
        // R^3: gyroscope bias
        GyroBias = 1,
        // R^6: angular velocity, body velocity
        AngVelBodyVel = 2,
        // R^9: angular velocity, body velocity, gyroscope bias
        AngVelBodyVelGyroBias = 3
    };

public:
    explicit StateDIV(StateType state_type,
                      double time = 0.0,
                      Eigen::Vector3d omg_wb_b = Eigen::Vector3d::Zero(),
                      Eigen::Vector3d v_wb_b = Eigen::Vector3d::Zero(),
                      Eigen::Vector3d bg_b = Eigen::Vector3d::Zero());

    ~StateDIV() = default;

    // clone DIV state
    [[nodiscard]] std::shared_ptr<StateDIV> cloneStateDIV() const { return std::make_shared<StateDIV>(*this); }

    // add state with a delta vector
    void addDelVec(const Eigen::VectorXd& delta);

    // set state type
    void setStateType(StateType state_type);

    // set state time
    void setTime(const double time) { time_ = time; }

    // set state with gyroscope bias
    void setState(const Eigen::Vector3d& bg_b);

    // set state with angular velocity and body velocity
    void setState(const Eigen::Vector3d& omg_wb_b, 
                  const Eigen::Vector3d& v_wb_b);

    // set state with angular velocity, body velocity, gyroscope bias
    void setState(const Eigen::Vector3d& omg_wb_b, 
                  const Eigen::Vector3d& v_wb_b, 
                  const Eigen::Vector3d& bg_b);

    // state getter
    // get state type
    [[nodiscard]] StateType getStateType() const noexcept { return state_type_; }

    // get state dimension
    [[nodiscard]] std::size_t getDim() const noexcept { return dim_; }

    // get state time
    [[nodiscard]] double getTime() const noexcept { return time_; }

    // get angular velocity
    [[nodiscard]] Eigen::Vector3d getAngularVelocity() const noexcept { return omg_wb_b_; }

    // get body velocity
    [[nodiscard]] Eigen::Vector3d getBodyVelocity() const noexcept { return v_wb_b_; }

    // get gyroscope bias
    [[nodiscard]] Eigen::Vector3d getGyroscopeBias() const noexcept { return bg_b_; }

private:
    // state type
    StateType state_type_;

    // state dimension
    std::size_t dim_;

    // state time
    double time_;
    // DIV state
    // angular velocity
    Eigen::Vector3d omg_wb_b_;
    // body velocity
    Eigen::Vector3d v_wb_b_;
    // gyroscope bias
    Eigen::Vector3d bg_b_;
};

// PropagatorDIV
class PropagatorDIV {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit PropagatorDIV(const std::shared_ptr<const Config>& config_ptr);

    ~PropagatorDIV() = default;

    // propagate state and covariance
    void propagateStateAndCov(const std::shared_ptr<StateDIV>& state_div_ptr, Eigen::MatrixXd& P,
                              const double dt) const {
        // propagate covariance
        propagateCovariance(state_div_ptr, P, 
                            dt);
        // propagate nominal state
        propagateNominalState(state_div_ptr, 
                              dt);
        // set time after propagation
        state_div_ptr->setTime(state_div_ptr->getTime() + dt);
    }

private:
    // propagate nominal state
    void propagateNominalState(const std::shared_ptr<StateDIV>& state_div_ptr, 
                               double dt) const;

    // propagate covariance
    void propagateCovariance(const std::shared_ptr<StateDIV>& state_div_ptr, Eigen::MatrixXd& P, 
                             double dt) const;

private:
    // configuration pointer
    const std::shared_ptr<const Config> config_ptr_;

    // input covariance matrix
    Eigen::MatrixXd Q_;
};

// UpdaterDIV
class UpdaterDIV {
public:
    explicit UpdaterDIV(const std::shared_ptr<const Config>& config_ptr);
    
    ~UpdaterDIV() = default;

    // update state by angular rate
    void updateByAngularRate(const std::shared_ptr<StateDIV>& state_div_ptr, Eigen::MatrixXd& P,
                             const Eigen::Vector3d& angular_rate, const Eigen::Matrix3d& R);

    // update state by Doppler velocity
    void updateByDoppler(const std::shared_ptr<StateDIV>& state_div_ptr, Eigen::MatrixXd& P,
                         const PointType& point, double R);

private:
    // configuration pointer
    const std::shared_ptr<const Config> config_ptr_;
};

// FilterDIV
class FilterDIV {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit FilterDIV(const std::shared_ptr<const Config>& config_ptr) : propagator_div_ptr_(std::make_shared<PropagatorDIV>(config_ptr)),
                                                                          updater_div_ptr_(std::make_shared<UpdaterDIV>(config_ptr)) {}

    ~FilterDIV() = default;

    // DIV filter propagation
    void propagateFilterDIV(const std::shared_ptr<StateDIV>& state_div_ptr,
                            double dt) {
        // state and covariance propagation via propagator
        propagator_div_ptr_->propagateStateAndCov(state_div_ptr, P_,
                                                  dt);
    }

    // state update based on angular rate observation
    void updateByAngularRate(const std::shared_ptr<StateDIV>& state_div_ptr,
                             const Eigen::Vector3d& angular_rate, const Eigen::Matrix3d& R_omg) {
        // state update based on angular rate observation from gyroscope
        updater_div_ptr_->updateByAngularRate(state_div_ptr, P_,
                                              angular_rate, R_omg);
    }

    // state update based on Doppler velocity observation
    void updateByDoppler(const std::shared_ptr<StateDIV>& state_div_ptr,
                         const PointType& point, const double R_d) {
        // state update based on Doppler velocity observation from Doppler LiDAR point
        updater_div_ptr_->updateByDoppler(state_div_ptr, P_,
                                          point, R_d);
    }

    // set covariance
    void setCovariance(const Eigen::MatrixXd& P) { P_ = P; }

    // get covariance
    [[nodiscard]] Eigen::MatrixXd getCovariance() const noexcept { return P_; }

private:
    // propagator pointer
    const std::shared_ptr<PropagatorDIV> propagator_div_ptr_;
    // updater pointer
    const std::shared_ptr<UpdaterDIV> updater_div_ptr_;

    // state covariance matrix
    Eigen::MatrixXd P_;
};

// DIV (Doppler-Inertial Velocimeter)
class DIV {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit DIV(const std::shared_ptr<const Config>& config_ptr);

    ~DIV() = default;

    // DIV initialization: initialize DIV state and covariance
    void initializeDIV(double time,
                       const Eigen::Vector3d& omg_wb_b_unbiased, const Eigen::Vector3d& v_wb_b);

    // DIV evolution: DIV state and covariance estimation, pose integration, and point mapping
    std::size_t evolveDIV(const std::shared_ptr<std::vector<std::shared_ptr<MeasDIV>>>& meas_div_vec_ptr);

    // get DIV state type
    [[nodiscard]] StateDIV::StateType getStateType() const noexcept {
        return state_div_ptr_->getStateType();
    }

    // get DIV state time
    [[nodiscard]] double getTime() const noexcept { return state_div_ptr_->getTime(); }

    // get DIV state
    [[nodiscard]] std::shared_ptr<StateDIV> getState() const noexcept { return state_div_ptr_; }

    // get angular velocity
    [[nodiscard]] Eigen::Vector3d getAngularVelocity() const noexcept {
        return (last_is_zero_velocity_imu_ ? Eigen::Vector3d::Zero() :
                                             state_div_ptr_->getAngularVelocity());
    }

    // get body velocity
    [[nodiscard]] Eigen::Vector3d getBodyVelocity() const noexcept { 
        return (last_is_zero_velocity_imu_ ? Eigen::Vector3d::Zero() :
                                             state_div_ptr_->getBodyVelocity());
    }

    // get gyroscope bias
    [[nodiscard]] Eigen::Vector3d getGyroscopeBias() const noexcept {
        return state_div_ptr_->getGyroscopeBias();
    }

    // get accumulated kinematics states from DIV integration start time
    [[nodiscard]] std::shared_ptr<std::vector<std::shared_ptr<StateKinematics>>> getStatesKinematics() const noexcept {
        return state_kin_ptr_vec_ptr_;
    }

    // get body velocity at DIV integration start time
    [[nodiscard]] Eigen::Vector3d getBodyVelocityIntegrationStart() const noexcept {
        return v_wb_b_integration_start_;
    }

    // get pose at last IMU time
    [[nodiscard]] std::pair<Eigen::Matrix3d, Eigen::Vector3d> getPose() const noexcept {
        return {last_state_kin_imu_ptr_->getRotation(), last_state_kin_imu_ptr_->getPosition()};
    }

    // get static point cloud map
    [[nodiscard]] PointCloudType::Ptr getMap() const noexcept { return map_div_ptr_; }

private:
    // update and integrate state by angular rate
    void updateAndintegrateByAngularRate(const std::shared_ptr<MeasDIV> &meas_div_ptr);

    // update and integrate state, and mapping by Doppler LiDAR point
    void updateAndintegrateAndmappingByDoppler(const std::shared_ptr<MeasDIV>& meas_div_ptr);

    // dynamic point detection
    bool isDynamicPoint(const PointType& point);

    // integrate pose on SE(3)
    static Eigen::VectorXd integratePose(const Eigen::Vector3d& omg_wb_b_head, const Eigen::Vector3d& v_wb_b_head,
                                         const Eigen::Vector3d& omg_wb_b_tail, const Eigen::Vector3d& v_wb_b_tail,
                                         double dt);

    // enable DIV integration
    void enableIntegration(const std::shared_ptr<MeasDIV>& meas_div_ptr);

    // detect zero velocity
    bool detectZeroVelocity();

    // add static point to map
    void addMapPoint(const Eigen::Matrix3d& R_wb, const Eigen::Vector3d& p_wb_w,
                     const PointType& point);

private:
    // configuration pointer
    const std::shared_ptr<const Config> config_ptr_;

    // DIV state pointer
    const std::shared_ptr<StateDIV> state_div_ptr_;

    // DIV filter pointer
    const std::shared_ptr<FilterDIV> filter_div_ptr_;

    // number of IMU data to enable DIV integration and mapping
    const std::size_t imu_num_integration_start_;
    // number of collected IMU data
    std::size_t imu_num_;

    // last DIV state time
    double last_state_time_div_;

    // velocity bootstrap success flag
    bool is_bootstrap_success_;
    // DIV launch flag
    bool is_div_launch_;
    // DIV integration start flag
    bool is_integration_start_;

    // last zero velocity detection flag at IMU time
    bool last_is_zero_velocity_imu_;
    // last zero velocity detection flag at LiDAR point time
    bool last_is_zero_velocity_point_;

    // body velocity at DIV integration start time in DIV integration world frame
    // DIV integration world frame is defined as body frame at DIV integration start time
    Eigen::Vector3d v_wb_b_integration_start_;

    // last kinematics state at IMU time
    std::shared_ptr<StateKinematics> last_state_kin_imu_ptr_;
    // last kinematics state at LiDAR point time
    std::shared_ptr<StateKinematics> last_state_kin_point_ptr_;

    // kinematics state vector for accelerometer bias and gravity estimation
    std::shared_ptr<std::vector<std::shared_ptr<StateKinematics>>> state_kin_ptr_vec_ptr_;

    // static point cloud map in DIV integration world frame
    PointCloudType::Ptr map_div_ptr_;
    // dynamic scan point cloud in LiDAR frame
    PointCloudType::Ptr dynamic_scan_div_lidar_ptr_;
};

} // namespace fmcw_lio

#endif // DIV_HPP
