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

#include "initializer/free_init/div.hpp"

// fmcw_lio
namespace fmcw_lio {

// StateDIV
StateDIV::StateDIV(StateType state_type,
                   double time,
                   Eigen::Vector3d omg_wb_b,
                   Eigen::Vector3d v_wb_b,
                   Eigen::Vector3d bg_b) : state_type_{},
                                           dim_{},
                                           time_(time),
                                           omg_wb_b_(std::move(omg_wb_b)),
                                           v_wb_b_(std::move(v_wb_b)),
                                           bg_b_(std::move(bg_b)) {
    // set state type and state dimension
    this->setStateType(state_type);
}

void StateDIV::addDelVec(const Eigen::VectorXd& delta) {
    // add state with a delta vector
    switch (state_type_) {
        case StateType::GyroBias: {
            // state order: bg_b
            this->bg_b_ += delta.block<3, 1>(0, 0).eval();

            break;
        }

        case StateType::AngVelBodyVel: {
            // state order: omg_wb_b, v_wb_b
            this->omg_wb_b_ += delta.block<3, 1>(0, 0).eval();
            this->v_wb_b_ += delta.block<3, 1>(3, 0).eval();

            break;
        }

        case StateType::AngVelBodyVelGyroBias: {
            // state order:  omg_wb_b, v_wb_b, bg_b
            this->omg_wb_b_ += delta.block<3, 1>(0, 0).eval();
            this->v_wb_b_ += delta.block<3, 1>(3, 0).eval();
            this->bg_b_ += delta.block<3, 1>(6, 0).eval();

            break;
        }
    }
}

void StateDIV::setStateType(const StateDIV::StateType state_type) {
    // set state type
    state_type_ = state_type;

    // set state dimension
    switch (state_type_) {
        case StateType::GyroBias: {
            // R^3: gyroscope bias
            dim_ = 3;

            break;
        }

        case StateType::AngVelBodyVel: {
            // R^6: angular velocity, body velocity
            dim_ = 6;

            break;
        }

        case StateType::AngVelBodyVelGyroBias: {
            // R^9: angular velocity, body velocity, gyroscope bias
            dim_ = 9;

            break;
        }
    }
}

void StateDIV::setState(const Eigen::Vector3d& bg_b) {
    // set state
    this->bg_b_ = bg_b;
}

void StateDIV::setState(const Eigen::Vector3d& omg_wb_b, 
                        const Eigen::Vector3d& v_wb_b) {
    // set state
    this->omg_wb_b_ = omg_wb_b;
    this->v_wb_b_ = v_wb_b;
}

void StateDIV::setState(const Eigen::Vector3d& omg_wb_b, 
                        const Eigen::Vector3d& v_wb_b, 
                        const Eigen::Vector3d& bg_b) {
    // set state
    this->omg_wb_b_ = omg_wb_b;
    this->v_wb_b_ = v_wb_b;
    this->bg_b_ = bg_b;
}

// PropagatorDIV
PropagatorDIV::PropagatorDIV(const std::shared_ptr<const Config>& config_ptr) : config_ptr_(config_ptr) {
    // initialize input covariance matrix
    Q_ = Eigen::MatrixXd::Zero(9, 9);
    Q_.block<3, 3>(0, 0) = config_ptr_->angular_velocity_cov_prop_div.asDiagonal();
    Q_.block<3, 3>(3, 3) = config_ptr_->body_velocity_cov_prop_div.asDiagonal();
    Q_.block<3, 3>(6, 6) = config_ptr_->gyroscope_bias_cov_prop_div.asDiagonal();
}

void PropagatorDIV::propagateNominalState(const std::shared_ptr<StateDIV>& state_div_ptr, 
                                          const double dt) const {
    // propagate nominal state
    const auto dim_state = state_div_ptr->getDim();
    const auto dim_mat = static_cast<Eigen::Index>(dim_state);

    switch (state_div_ptr->getStateType()) {
        case StateDIV::StateType::AngVelBodyVel: {
            break;
        }

        case StateDIV::StateType::AngVelBodyVelGyroBias:
        case StateDIV::StateType::GyroBias: {
            const auto bg_b = state_div_ptr->getGyroscopeBias();
            const auto idx_bg = static_cast<Eigen::Index>(dim_mat - bg_b.rows());

            // 1-st order Gauss-Markov process
            const auto delta_bg = ((-dt * config_ptr_->imu_correlation_time_inv) * bg_b).eval();

            // gyroscope bias index
            Eigen::VectorXd delta = Eigen::VectorXd::Zero(dim_mat);
            delta.segment<3>(idx_bg) = delta_bg;

            // add state delta
            state_div_ptr->addDelVec(delta);

            break;
        }
    }
}

void PropagatorDIV::propagateCovariance(const std::shared_ptr<StateDIV>& state_div_ptr, Eigen::MatrixXd& P, 
                                        const double dt) const {
    // propagate covariance
    const auto dim_state = state_div_ptr->getDim();
    const auto dim_mat = static_cast<Eigen::Index>(dim_state);

    // state transition matrix
    Eigen::MatrixXd F_x = Eigen::MatrixXd::Identity(dim_mat, dim_mat);
    // input matrix
    Eigen::MatrixXd F_w = dt * Eigen::MatrixXd::Identity(dim_mat, dim_mat);
    // noise covariance matrix
    Eigen::MatrixXd Q;

    switch (state_div_ptr->getStateType()) {
        case StateDIV::StateType::AngVelBodyVel: {
            Q = Q_.block(0, 0, dim_mat, dim_mat).eval();

            break;
        }

        case StateDIV::StateType::AngVelBodyVelGyroBias:
        case StateDIV::StateType::GyroBias: {
            // 1-st order Gauss-Markov process
            const auto block_bg = ((1.0 - (dt * config_ptr_->imu_correlation_time_inv)) * Eigen::Matrix3d::Identity()).eval();
            const auto dim_bg = static_cast<Eigen::Index>(block_bg.rows());

            F_x.block(dim_mat - dim_bg, dim_mat - dim_bg, dim_bg, dim_bg) = block_bg;

            const auto dim_Q = static_cast<Eigen::Index>(Q_.rows());
            Q = Q_.block(dim_Q - dim_mat, dim_Q - dim_mat, dim_mat, dim_mat).eval();

            break;
        }
    }

    P = ((F_x * P * F_x.transpose()) + (F_w * Q * F_w.transpose())).eval();
}

// UpdaterDIV
UpdaterDIV::UpdaterDIV(const std::shared_ptr<const Config>& config_ptr) : config_ptr_(config_ptr) {}

void UpdaterDIV::updateByAngularRate(const std::shared_ptr<StateDIV>& state_div_ptr, Eigen::MatrixXd& P,
                                     const Eigen::Vector3d& angular_rate, const Eigen::Matrix3d& R) {
    // state type and state dimension
    const auto state_type = state_div_ptr->getStateType();
    const auto dim_state = state_div_ptr->getDim();

    // residual and Jacobian matrix
    Eigen::Vector3d r = Eigen::Vector3d::Zero();
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, static_cast<Eigen::Index>(dim_state));
    if (state_type == StateDIV::StateType::AngVelBodyVelGyroBias) {
        r = angular_rate - (state_div_ptr->getAngularVelocity() + state_div_ptr->getGyroscopeBias());
        H.block<3, 3>(0, 0).setIdentity();
        H.block<3, 3>(0, 6).setIdentity();
    } else if (state_type == StateDIV::StateType::AngVelBodyVel) {
        r = angular_rate - config_ptr_->gyroscope_bias_prior - state_div_ptr->getAngularVelocity();
        H.block<3, 3>(0, 0).setIdentity();
    } else if (state_type == StateDIV::StateType::GyroBias) {
        r = angular_rate - (state_div_ptr->getAngularVelocity() + state_div_ptr->getGyroscopeBias());
        H.block<3, 3>(0, 0).setIdentity();
    }

    // update lambda function
    auto update_state_and_cov = [&, this]() -> void {
        // update covariance matrix
        Eigen::MatrixXd PHT = P * H.transpose();
        Eigen::MatrixXd S = (H * PHT) + R;
        Eigen::MatrixXd S_inv = S.inverse();

        // Kalman gain
        Eigen::MatrixXd K = PHT * S_inv;
        // optimal updated error state dx of current iteration step
        Eigen::VectorXd dx = K * r;

        // update state estimate
        state_div_ptr->addDelVec(dx);
        // update covariance
        P -= K * H * P;
    };

    // update state and covariance
    update_state_and_cov();
}

void UpdaterDIV::updateByDoppler(const std::shared_ptr<StateDIV>& state_div_ptr, Eigen::MatrixXd& P,
                                 const PointType& point, const double R) {
    // state dimension
    const auto dim_state = state_div_ptr->getDim();
    // angular velocity and body velocity
    const auto omg_wb_b = state_div_ptr->getAngularVelocity();
    const auto v_wb_b = state_div_ptr->getBodyVelocity();

    // point direction unit vector
    Eigen::Vector3d d_lp_l = Eigen::Vector3d(point.x, point.y, point.z).normalized();
    Eigen::Vector3d d_lp_b = config_ptr_->R_bl * d_lp_l;

    // residual and Jacobian matrix
    // residual
    double r = point.intensity + d_lp_l.dot(config_ptr_->R_bl.transpose() * (v_wb_b + omg_wb_b.cross(config_ptr_->p_bl_b)));

    // Jacobian matrix
    Eigen::RowVectorXd H = Eigen::RowVectorXd::Zero(static_cast<Eigen::Index>(dim_state));
    H.block<1, 3>(0, 0) = -(config_ptr_->p_bl_b.cross(d_lp_b)).transpose();
    H.block<1, 3>(0, 3) = -d_lp_b.transpose();
    
    // update lambda function
    auto update_state_and_cov = [&, this]() -> void {
        // update covariance matrix
        Eigen::VectorXd PHT = P * H.transpose();
        double S = H.dot(PHT) + R;
        double S_inv = 1.0 / S;

        // Kalman gain
        Eigen::VectorXd K = PHT * S_inv;
        // optimal updated error state dx of current iteration step
        Eigen::VectorXd dx = K * r;

        // update state estimate
        state_div_ptr->addDelVec(dx);
        // update covariance
        P -= K * H * P;
    };

    // update state and covariance
    update_state_and_cov();
}

// DIV (Doppler-Inertial Velocimeter)
DIV::DIV(const std::shared_ptr<const Config>& config_ptr) : config_ptr_(config_ptr),
                                                            state_div_ptr_(std::make_shared<StateDIV>(StateDIV::StateType::AngVelBodyVel,
                                                                                                      0.0,
                                                                                                      Eigen::Vector3d::Zero(),
                                                                                                      Eigen::Vector3d::Zero(),
                                                                                                      config_ptr_->gyroscope_bias_prior)),
                                                            filter_div_ptr_(std::make_shared<FilterDIV>(config_ptr_)),
                                                            imu_num_integration_start_(config_ptr_->imu_num_thresh_world_frame),
                                                            imu_num_(0),
                                                            last_state_time_div_(std::numeric_limits<double>::quiet_NaN()),
                                                            is_bootstrap_success_(false),
                                                            is_div_launch_(false),
                                                            is_integration_start_(false),
                                                            last_is_zero_velocity_imu_(false),
                                                            last_is_zero_velocity_point_(false),
                                                            v_wb_b_integration_start_(Eigen::Vector3d::Zero()),
                                                            last_state_kin_imu_ptr_(std::make_shared<StateKinematics>()),
                                                            last_state_kin_point_ptr_(std::make_shared<StateKinematics>()),
                                                            state_kin_ptr_vec_ptr_(std::make_shared<std::vector<std::shared_ptr<StateKinematics>>>()) {
    // initialize filter covariance matrix
    const auto dim_state = static_cast<Eigen::Index>(state_div_ptr_->getDim());
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(dim_state, dim_state);
    P.block<3, 3>(0, 0) = config_ptr_->angular_velocity_cov_init_div.asDiagonal();
    P.block<3, 3>(3, 3) = config_ptr_->body_velocity_cov_init_div.asDiagonal();
    filter_div_ptr_->setCovariance(P);

    // initialize point cloud
    map_div_ptr_.reset(new PointCloudType());
    dynamic_scan_div_lidar_ptr_.reset(new PointCloudType());
}

void DIV::initializeDIV(double time,
                        const Eigen::Vector3d& omg_wb_b_unbiased, const Eigen::Vector3d& v_wb_b) {
    // set DIV state and time
    state_div_ptr_->setStateType(StateDIV::StateType::AngVelBodyVel);
    state_div_ptr_->setTime(time);
    state_div_ptr_->setState(omg_wb_b_unbiased,
                             v_wb_b);

    last_state_time_div_ = time;
    is_div_launch_ = true;
    is_bootstrap_success_ = true;
    last_state_kin_imu_ptr_ = std::make_shared<StateKinematics>(time,
                                                                Eigen::Matrix3d::Identity(),
                                                                v_wb_b,
                                                                Eigen::Vector3d::Zero(),
                                                                omg_wb_b_unbiased,
                                                                Eigen::Vector3d::Zero());
    last_state_kin_point_ptr_ = std::dynamic_pointer_cast<StateKinematics>(last_state_kin_imu_ptr_->cloneState());
}

std::size_t DIV::evolveDIV(const std::shared_ptr<std::vector<std::shared_ptr<MeasDIV>>>& meas_div_vec_ptr) {
    // clear dynamic scan point cloud
    dynamic_scan_div_lidar_ptr_->points.clear();

    // launch DIV
    if (!is_div_launch_) {
        is_div_launch_ = true;
        state_div_ptr_->setTime(meas_div_vec_ptr->front()->timestamp);
        last_state_time_div_ = state_div_ptr_->getTime();
    }

    const auto& meas_div_vec = *meas_div_vec_ptr;
    const auto last_it = std::prev(meas_div_vec.end());
    for (auto it = meas_div_vec.begin(); it != meas_div_vec.end(); ++it) {
        const auto& meas_div_ptr = *it;
        double curr_time = meas_div_ptr->timestamp;

        // dynamic point detection
        if ((is_bootstrap_success_ || is_integration_start_) &&
            (meas_div_ptr->meas_type == MeasDIV::MeasType::DopplerVelocity) &&
            config_ptr_->use_dynamic_point_removal_init &&
            isDynamicPoint(meas_div_ptr->point)) {
            // store dynamic point
            dynamic_scan_div_lidar_ptr_->points.push_back(meas_div_ptr->point);

            continue;
        }

        if (curr_time > last_state_time_div_) {
            // propagate DIV state and filter covariance
            filter_div_ptr_->propagateFilterDIV(state_div_ptr_,
                                                curr_time - last_state_time_div_);
            // update last state time
            last_state_time_div_ = curr_time;
        }

        // update and integrate DIV state, and mapping point
        switch (meas_div_ptr->meas_type) {
            case MeasDIV::MeasType::AngularRate: {
                // update and integrate DIV state by angular rate
                updateAndintegrateByAngularRate(meas_div_ptr);

                if (it != last_it) {
                    imu_num_ += 1;

                    // enable DIV integration
                    enableIntegration(meas_div_ptr);
                }

                break;
            }

            case MeasDIV::MeasType::DopplerVelocity: {
                // update and integrate DIV state, and mapping by Doppler LiDAR point
                updateAndintegrateAndmappingByDoppler(meas_div_ptr);

                break;
            }
        }
    }

    return imu_num_;
}

void DIV::updateAndintegrateByAngularRate(const std::shared_ptr<MeasDIV>& meas_div_ptr) {
    // update DIV state by angular rate
    const Eigen::Matrix3d R_omg = config_ptr_->angular_rate_cov_div.asDiagonal();
    filter_div_ptr_->updateByAngularRate(state_div_ptr_, 
                                         meas_div_ptr->angular_rate, R_omg);

    // DIV integration is started
    if (is_integration_start_) {
        // detect zero velocity
        bool is_zero_velocity = (state_div_ptr_->getStateType() == StateDIV::StateType::GyroBias) ||
                                detectZeroVelocity();

        std::shared_ptr<StateKinematics> state_kin_ptr;
        // zero velocity
        if (is_zero_velocity &&
            last_is_zero_velocity_imu_) {
            // store integrated kinematics state and measurement
            state_kin_ptr = std::make_shared<StateKinematics>(state_div_ptr_->getTime(),
                                                              last_state_kin_imu_ptr_->getRotation(),
                                                              last_state_kin_imu_ptr_->getVelocity(),
                                                              last_state_kin_imu_ptr_->getPosition(),
                                                              last_state_kin_imu_ptr_->getAngRotRate(),
                                                              meas_div_ptr->specific_force);
            state_kin_ptr_vec_ptr_->push_back(state_kin_ptr);
        } else {
            // integrate pose from last IMU time to current IMU time
            double dt_imu = state_div_ptr_->getTime() - last_state_kin_imu_ptr_->getTime();
            auto delta = integratePose(last_state_kin_imu_ptr_->getAngRotRate(), last_state_kin_imu_ptr_->getVelocity(),
                                       state_div_ptr_->getAngularVelocity(), state_div_ptr_->getBodyVelocity(),
                                       dt_imu);
            Eigen::Matrix3d R_wb_k = (last_state_kin_imu_ptr_->getRotation() *
                                      SO3::Exp(delta.segment<3>(0).eval())).getMat();
            Eigen::Vector3d p_wb_w_k = last_state_kin_imu_ptr_->getPosition() +
                                       (SO3(last_state_kin_imu_ptr_->getRotation()) * delta.segment<3>(3).eval());

            // store integrated kinematics state and measurement
            state_kin_ptr = std::make_shared<StateKinematics>(state_div_ptr_->getTime(),
                                                              R_wb_k,
                                                              state_div_ptr_->getBodyVelocity(),
                                                              p_wb_w_k,
                                                              state_div_ptr_->getAngularVelocity(),
                                                              meas_div_ptr->specific_force);
            state_kin_ptr_vec_ptr_->push_back(state_kin_ptr);
        }

        last_is_zero_velocity_imu_ = is_zero_velocity;
        last_is_zero_velocity_point_ = is_zero_velocity;

        // integrated kinematics state and measurement
        last_state_kin_imu_ptr_ = std::dynamic_pointer_cast<StateKinematics>(state_kin_ptr->cloneState());
        last_state_kin_point_ptr_ = std::dynamic_pointer_cast<StateKinematics>(state_kin_ptr->cloneState());
    }
}

void DIV::updateAndintegrateAndmappingByDoppler(const std::shared_ptr<MeasDIV> &meas_div_ptr) {
    // update DIV state by Doppler velocity
    if (state_div_ptr_->getStateType() == StateDIV::StateType::AngVelBodyVelGyroBias ||
        state_div_ptr_->getStateType() == StateDIV::StateType::AngVelBodyVel) {
        const double R_d = config_ptr_->doppler_velocity_cov_div;
        filter_div_ptr_->updateByDoppler(state_div_ptr_, 
                                         meas_div_ptr->point, R_d);
    }

    // DIV integration is started
    if (is_integration_start_) {
        // detect zero velocity
        bool is_zero_velocity = (state_div_ptr_->getStateType() == StateDIV::StateType::GyroBias) ||
                                detectZeroVelocity();

        // zero velocity
        if (is_zero_velocity && 
            last_is_zero_velocity_point_) {
            // integrated kinematics state
            last_state_kin_point_ptr_ = std::make_shared<StateKinematics>(state_div_ptr_->getTime(),
                                                                          last_state_kin_point_ptr_->getRotation(),
                                                                          last_state_kin_point_ptr_->getVelocity(),
                                                                          last_state_kin_point_ptr_->getPosition(),
                                                                          last_state_kin_point_ptr_->getAngRotRate(),
                                                                          last_state_kin_imu_ptr_->getLinVelRate());

            // add point to map in DIV integration world frame
            addMapPoint(last_state_kin_point_ptr_->getRotation(), last_state_kin_point_ptr_->getPosition(),
                        meas_div_ptr->point);
        } else {
            // integrate pose from last time to current time
            double dt_point = state_div_ptr_->getTime() - last_state_kin_point_ptr_->getTime();
            auto delta = integratePose(last_state_kin_point_ptr_->getAngRotRate(), last_state_kin_point_ptr_->getVelocity(),
                                       state_div_ptr_->getAngularVelocity(), state_div_ptr_->getBodyVelocity(),
                                       dt_point);
            Eigen::Matrix3d R_wb_j = (last_state_kin_point_ptr_->getRotation() *
                                      SO3::Exp(delta.segment<3>(0).eval())).getMat();
            Eigen::Vector3d p_wb_w_j = last_state_kin_point_ptr_->getPosition() +
                                       (SO3(last_state_kin_point_ptr_->getRotation()) * delta.segment<3>(3).eval());

            // integrated kinematics state
            last_state_kin_point_ptr_ = std::make_shared<StateKinematics>(state_div_ptr_->getTime(),
                                                                          R_wb_j,
                                                                          state_div_ptr_->getBodyVelocity(),
                                                                          p_wb_w_j,
                                                                          state_div_ptr_->getAngularVelocity(),
                                                                          last_state_kin_imu_ptr_->getLinVelRate());

            // add point to map in DIV integration world frame
            addMapPoint(R_wb_j, p_wb_w_j,
                        meas_div_ptr->point);
        }

        last_is_zero_velocity_point_ = is_zero_velocity;
    }
}

bool DIV::isDynamicPoint(const PointType& point) {
    // point direction unit vector
    Eigen::Vector3d d_lp_l = Eigen::Vector3d(point.x, point.y, point.z).normalized();

    // LiDAR velocity
    Eigen::Vector3d v_wl_l = config_ptr_->R_bl.transpose() * (state_div_ptr_->getBodyVelocity() +
                                                              state_div_ptr_->getAngularVelocity().cross(config_ptr_->p_bl_b));
    Eigen::Vector3d v_wl_l_normalized = v_wl_l.normalized();

    // predicted Doppler velocity
    double doppler_pred = -d_lp_l.dot(v_wl_l);
    double error_doppler = std::abs(doppler_pred - point.intensity);

    double v_wl_l_norm = v_wl_l.norm();

    // use cosine weight or for small velocity case, do not scale threshold
    double cos_weight_or_not = (v_wl_l_norm > config_ptr_->zero_velocity_thresh_init) ?
                                std::abs(d_lp_l.dot(v_wl_l_normalized)) :
                                1.0;
    if (error_doppler < (cos_weight_or_not * config_ptr_->remove_dynamic_threshold_init)) {
        return false;
    } else {
        return true;
    }
}

Eigen::VectorXd DIV::integratePose(const Eigen::Vector3d& omg_wb_b_head, const Eigen::Vector3d& v_wb_b_head,
                                   const Eigen::Vector3d& omg_wb_b_tail, const Eigen::Vector3d& v_wb_b_tail,
                                   const double dt) {
    // integrate pose on SE(3)
    // average angular velocity
    Eigen::Vector3d omg_wb_b_avg = 0.5 * (omg_wb_b_head + omg_wb_b_tail);
    // average body velocity
    Eigen::Vector3d v_wb_b_avg = 0.5 * (v_wb_b_head + v_wb_b_tail);

    // delta theta
    Eigen::Vector3d dtheta = omg_wb_b_avg * dt;

    // DIV integration increment
    Eigen::VectorXd delta = Eigen::VectorXd::Zero(6);
    delta.block<3, 1>(0, 0) = dtheta;
    delta.block<3, 1>(3, 0) = SO3::Jl(dtheta) * v_wb_b_avg * dt;

    return delta;
}

void DIV::enableIntegration(const std::shared_ptr<MeasDIV>& meas_div_ptr) {
    // check and define DIV integration world frame
    if (!is_integration_start_ &&
        (imu_num_ >= imu_num_integration_start_)) {
        // start DIV integration
        is_integration_start_ = true;

        // body velocity at DIV integration start time in DIV integration world frame via identity transformation
        v_wb_b_integration_start_ = state_div_ptr_->getBodyVelocity();

        // zero velocity detection at DIV integration start time
        if (detectZeroVelocity()) {
            // set state type, state, and filter covariance
            state_div_ptr_->setStateType(StateDIV::StateType::GyroBias);
            state_div_ptr_->setState(Eigen::Vector3d::Zero(),
                                     Eigen::Vector3d::Zero(),
                                     config_ptr_->gyroscope_bias_prior);
            filter_div_ptr_->setCovariance(config_ptr_->gyroscope_bias_cov_init_div.asDiagonal());

            last_is_zero_velocity_imu_ = true;
            last_is_zero_velocity_point_ = true;
        } else {
            // set state type, state, and filter covariance
            state_div_ptr_->setStateType(StateDIV::StateType::AngVelBodyVelGyroBias);
            state_div_ptr_->setState(state_div_ptr_->getAngularVelocity(),
                                     state_div_ptr_->getBodyVelocity(),
                                     config_ptr_->gyroscope_bias_prior);
            Eigen::MatrixXd P = Eigen::MatrixXd::Zero(9, 9);
            P.block<3, 3>(0, 0) = config_ptr_->angular_velocity_cov_init_div.asDiagonal();
            P.block<3, 3>(3, 3) = config_ptr_->body_velocity_cov_init_div.asDiagonal();
            P.block<3, 3>(6, 6) = config_ptr_->gyroscope_bias_cov_init_div.asDiagonal();
            filter_div_ptr_->setCovariance(P);
        }

        // integrated kinematics state and measurement
        last_state_kin_imu_ptr_ = std::make_shared<StateKinematics>(state_div_ptr_->getTime(),
                                                                    Eigen::Matrix3d::Identity(),
                                                                    state_div_ptr_->getBodyVelocity(),
                                                                    Eigen::Vector3d::Zero(),
                                                                    state_div_ptr_->getAngularVelocity(),
                                                                    meas_div_ptr->specific_force);
        last_state_kin_point_ptr_ = std::make_shared<StateKinematics>(state_div_ptr_->getTime(),
                                                                      Eigen::Matrix3d::Identity(),
                                                                      state_div_ptr_->getBodyVelocity(),
                                                                      Eigen::Vector3d::Zero(),
                                                                      state_div_ptr_->getAngularVelocity(),
                                                                      meas_div_ptr->specific_force);

        // clear accumulated integrated kinematics state and measurement vector
        state_kin_ptr_vec_ptr_->clear();
    }
}

bool DIV::detectZeroVelocity() {
    // detect zero velocity via angular velocity and body velocity estimate
    return ((state_div_ptr_->getBodyVelocity().norm() <= config_ptr_->zero_velocity_thresh_init) &&
            (state_div_ptr_->getAngularVelocity().norm() <= config_ptr_->zero_angular_velocity_thresh_init));
}

void DIV::addMapPoint(const Eigen::Matrix3d& R_wb, const Eigen::Vector3d& p_wb_w,
                      const PointType& point) {
    // transform point to DIV integration world frame
    Eigen::Vector3d p_lp_l{point.x, point.y, point.z};
    Eigen::Vector3d p_wp_w = (R_wb * ((config_ptr_->R_bl * p_lp_l) + config_ptr_->p_bl_b)) + p_wb_w;

    PointType point_integ_w;
    point_integ_w.curvature = point.curvature;
    point_integ_w.x = static_cast<float>(p_wp_w.x());
    point_integ_w.y = static_cast<float>(p_wp_w.y());
    point_integ_w.z = static_cast<float>(p_wp_w.z());
    point_integ_w.intensity = point.intensity;

    map_div_ptr_->points.push_back(point_integ_w);
}

} // namespace fmcw_lio
