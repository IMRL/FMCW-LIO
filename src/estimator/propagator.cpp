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

#include "estimator/propagator.hpp"

// fmcw_lio
namespace fmcw_lio {

// Propagator
Propagator::Propagator(const std::shared_ptr<const Config>& config_ptr) : logger_propagation_ptr(std::make_shared<LoggerPropagation>()),
                                                                          config_ptr_(config_ptr),
                                                                          compensator_ptr_(std::make_shared<Compensator>(config_ptr_,
                                                                                                                         integration_table_.at(config_ptr_->compensation_method))),
                                                                          last_scan_end_time_(0.0),
                                                                          last_imu_ptr_(nullptr),
                                                                          scan_compensated_lidar_ptr_(new PointCloudType()),
                                                                          omg_wb_b_meas_scan_end_(0.0, 0.0, 0.0),
                                                                          f_wb_b_meas_scan_end_(0.0, 0.0, 0.0),
                                                                          omg_wb_b_unbiased_last_(0.0, 0.0, 0.0),
                                                                          f_wb_b_unbiased_last_(0.0, 0.0, 0.0),
                                                                          states_propagation_ptr_(std::make_shared<std::vector<std::shared_ptr<const StateBase>>>()),
                                                                          integrate_func_(integration_table_.at(config_ptr_->integration_method)) {
    // initialize input covariance matrix
    Q_ = Eigen::MatrixXd::Zero(12, 12);
    Q_.block<3, 3>(0, 0) = config_ptr_->gyroscope_cov * Eigen::Matrix3d::Identity();
    Q_.block<3, 3>(3, 3) = config_ptr_->accelerometer_cov * Eigen::Matrix3d::Identity();
    Q_.block<3, 3>(6, 6) = config_ptr_->gyroscope_bias_cov * Eigen::Matrix3d::Identity();
    Q_.block<3, 3>(9, 9) = config_ptr_->accelerometer_bias_cov * Eigen::Matrix3d::Identity();
}

void Propagator::initializePropagator(const std::shared_ptr<StateBase>& state_ptr,
                                      const std::shared_ptr<MeasPackLI>& meas_ptr) {
    // for propagation and motion compensation
    last_scan_end_time_ = meas_ptr->scan_end_time;

    // interpolate IMU data at scan end
    auto imu_head_ptr = meas_ptr->imu_msg_ptr_queue.end() - 2;
    auto imu_tail_ptr = meas_ptr->imu_msg_ptr_queue.end() - 1;

    double dt_head_tail = imu_tail_ptr->get()->header.stamp.toSec() - imu_head_ptr->get()->header.stamp.toSec();
    double dt_head_scan = meas_ptr->scan_end_time - imu_head_ptr->get()->header.stamp.toSec();
    double interp_weight = dt_head_scan / dt_head_tail;

    sensor_msgs::Imu::Ptr interp_imu_ptr(new sensor_msgs::Imu());
    interp_imu_ptr->header.stamp = ros::Time().fromSec(meas_ptr->scan_end_time);
    interp_imu_ptr->angular_velocity.x = ((1.0 - interp_weight) * imu_head_ptr->get()->angular_velocity.x) +
                                         (interp_weight * imu_tail_ptr->get()->angular_velocity.x);
    interp_imu_ptr->angular_velocity.y = ((1.0 - interp_weight) * imu_head_ptr->get()->angular_velocity.y) +
                                         (interp_weight * imu_tail_ptr->get()->angular_velocity.y);
    interp_imu_ptr->angular_velocity.z = ((1.0 - interp_weight) * imu_head_ptr->get()->angular_velocity.z) +
                                         (interp_weight * imu_tail_ptr->get()->angular_velocity.z);
    interp_imu_ptr->linear_acceleration.x = ((1.0 - interp_weight) * imu_head_ptr->get()->linear_acceleration.x) +
                                            (interp_weight * imu_tail_ptr->get()->linear_acceleration.x);
    interp_imu_ptr->linear_acceleration.y = ((1.0 - interp_weight) * imu_head_ptr->get()->linear_acceleration.y) +
                                            (interp_weight * imu_tail_ptr->get()->linear_acceleration.y);
    interp_imu_ptr->linear_acceleration.z = ((1.0 - interp_weight) * imu_head_ptr->get()->linear_acceleration.z) +
                                            (interp_weight * imu_tail_ptr->get()->linear_acceleration.z);
    last_imu_ptr_ = interp_imu_ptr;

    Eigen::Vector3d omg_wb_b(last_imu_ptr_->angular_velocity.x,
                             last_imu_ptr_->angular_velocity.y,
                             last_imu_ptr_->angular_velocity.z);
    Eigen::Vector3d f_wb_b(last_imu_ptr_->linear_acceleration.x,
                           last_imu_ptr_->linear_acceleration.y,
                           last_imu_ptr_->linear_acceleration.z);

    omg_wb_b_meas_scan_end_ = omg_wb_b;
    f_wb_b_meas_scan_end_ = f_wb_b;

    omg_wb_b_unbiased_last_ = omg_wb_b - state_ptr->getGyroscopeBias();
    f_wb_b_unbiased_last_ = f_wb_b - state_ptr->getAccelerometerBias();
}

void Propagator::propagateStateAndCov(const std::shared_ptr<StateBase>& state_ptr, Eigen::MatrixXd& P,
                                      const std::shared_ptr<MeasPackLI>& meas_ptr) {
    // start timer
    std::chrono::steady_clock::time_point t1, t2;
    t1 = std::chrono::steady_clock::now();

    const double scan_beg_time = meas_ptr->scan_beg_time;
    const double scan_end_time = meas_ptr->scan_end_time;

    states_propagation_ptr_->clear();
    states_propagation_ptr_->emplace_back(std::make_shared<const StateKinematics>(last_scan_end_time_ - scan_beg_time,
                                                                                  state_ptr->getRotation(),
                                                                                  state_ptr->getVelocity(),
                                                                                  state_ptr->getPosition(),
                                                                                  omg_wb_b_unbiased_last_,
                                                                                  f_wb_b_unbiased_last_,
                                                                                  state_ptr->getGravity()));

    auto last_state_ptr = state_ptr->cloneState();
    for (std::size_t i = 0; i < meas_ptr->imu_msg_ptr_queue.size(); ++i) {
        // get current IMU data
        sensor_msgs::Imu::Ptr curr_imu_ptr(new sensor_msgs::Imu(*meas_ptr->imu_msg_ptr_queue[i]));

        double dt;
        if (i != (meas_ptr->imu_msg_ptr_queue.size() - 1)) {
            dt = curr_imu_ptr->header.stamp.toSec() - state_ptr->getTime();
        } else {
            dt = scan_end_time - state_ptr->getTime();
            double dt_scan_end_curr = curr_imu_ptr->header.stamp.toSec() - scan_end_time;
            double interp_weight = dt / (dt + dt_scan_end_curr);

            sensor_msgs::Imu::Ptr interp_imu_ptr(new sensor_msgs::Imu());
            interp_imu_ptr->header.stamp = ros::Time().fromSec(scan_end_time);
            interp_imu_ptr->angular_velocity.x = ((1.0 - interp_weight) * last_imu_ptr_->angular_velocity.x) +
                                                 (interp_weight * curr_imu_ptr->angular_velocity.x);
            interp_imu_ptr->angular_velocity.y = ((1.0 - interp_weight) * last_imu_ptr_->angular_velocity.y) +
                                                 (interp_weight * curr_imu_ptr->angular_velocity.y);
            interp_imu_ptr->angular_velocity.z = ((1.0 - interp_weight) * last_imu_ptr_->angular_velocity.z) +
                                                 (interp_weight * curr_imu_ptr->angular_velocity.z);
            interp_imu_ptr->linear_acceleration.x = ((1.0 - interp_weight) * last_imu_ptr_->linear_acceleration.x) +
                                                    (interp_weight * curr_imu_ptr->linear_acceleration.x);
            interp_imu_ptr->linear_acceleration.y = ((1.0 - interp_weight) * last_imu_ptr_->linear_acceleration.y) +
                                                    (interp_weight * curr_imu_ptr->linear_acceleration.y);
            interp_imu_ptr->linear_acceleration.z = ((1.0 - interp_weight) * last_imu_ptr_->linear_acceleration.z) +
                                                    (interp_weight * curr_imu_ptr->linear_acceleration.z);
            curr_imu_ptr = interp_imu_ptr;
        }

        // prepare IMU data for integration
        Eigen::Vector3d omg_wb_b_head, f_wb_b_head;
        Eigen::Vector3d omg_wb_b_tail, f_wb_b_tail;

        omg_wb_b_head = Eigen::Vector3d(last_imu_ptr_->angular_velocity.x,
                                        last_imu_ptr_->angular_velocity.y,
                                        last_imu_ptr_->angular_velocity.z);
        omg_wb_b_tail = Eigen::Vector3d(curr_imu_ptr->angular_velocity.x,
                                        curr_imu_ptr->angular_velocity.y,
                                        curr_imu_ptr->angular_velocity.z);

        f_wb_b_head = Eigen::Vector3d(last_imu_ptr_->linear_acceleration.x,
                                      last_imu_ptr_->linear_acceleration.y,
                                      last_imu_ptr_->linear_acceleration.z);
        f_wb_b_tail = Eigen::Vector3d(curr_imu_ptr->linear_acceleration.x,
                                      curr_imu_ptr->linear_acceleration.y,
                                      curr_imu_ptr->linear_acceleration.z);

        // forward propagation from last time to current time
        // propagate covariance
        propagateCovariance(state_ptr, P,
                            omg_wb_b_head, f_wb_b_head,
                            omg_wb_b_tail, f_wb_b_tail,
                            dt);
        // propagate nominal state
        propagateNominalState(state_ptr,
                              omg_wb_b_head, f_wb_b_head,
                              omg_wb_b_tail, f_wb_b_tail,
                              dt);
        // set time after propagation
        state_ptr->setTime(curr_imu_ptr->header.stamp.toSec());

        omg_wb_b_unbiased_last_ = omg_wb_b_tail - state_ptr->getGyroscopeBias();
        f_wb_b_unbiased_last_ = f_wb_b_tail - state_ptr->getAccelerometerBias();

        double offset_time = curr_imu_ptr->header.stamp.toSec() - scan_beg_time;
        states_propagation_ptr_->emplace_back(std::make_shared<const StateKinematics>(offset_time,
                                                                                      state_ptr->getRotation(),
                                                                                      state_ptr->getVelocity(),
                                                                                      state_ptr->getPosition(),
                                                                                      omg_wb_b_unbiased_last_,
                                                                                      f_wb_b_unbiased_last_,
                                                                                      state_ptr->getGravity()));

        // get angular rate and specific force at scan end for motion compensation
        if (i == (meas_ptr->imu_msg_ptr_queue.size() - 1)) {
            omg_wb_b_meas_scan_end_ = omg_wb_b_tail;
            f_wb_b_meas_scan_end_ = f_wb_b_tail;
        } else if (config_ptr_->publish_tf_imu_rate) {
            tf_publish_func_(state_ptr);
        }

        // update last IMU data
        last_imu_ptr_ = curr_imu_ptr;
    }

    // update end time of last scan
    last_scan_end_time_ = scan_end_time;

    // temporal-spatial motion compensation
    scan_compensated_lidar_ptr_ = meas_ptr->scan_ptr;
    compensator_ptr_->compensateScan(state_ptr,
                                     states_propagation_ptr_,
                                     scan_compensated_lidar_ptr_);

    // stop timer
    t2 = std::chrono::steady_clock::now();
    logger_propagation_ptr->propagation_time = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()) * 1.0e-3;
    logger_propagation_ptr->delta = state_ptr->subState(last_state_ptr);
}

void Propagator::propagateNominalState(const std::shared_ptr<StateBase>& state_ptr,
                                       const Eigen::Vector3d& omg_wb_b_head, const Eigen::Vector3d& f_wb_b_head,
                                       const Eigen::Vector3d& omg_wb_b_tail, const Eigen::Vector3d& f_wb_b_tail,
                                       const double dt) {
    // unbiased measurement
    Eigen::Vector3d bg = state_ptr->getGyroscopeBias();
    Eigen::Vector3d ba = state_ptr->getAccelerometerBias();

    Eigen::Vector3d omg_wb_b_head_unbiased = omg_wb_b_head - bg;
    Eigen::Vector3d f_wb_b_head_unbiased = f_wb_b_head - ba;
    Eigen::Vector3d omg_wb_b_tail_unbiased = omg_wb_b_tail - bg;
    Eigen::Vector3d f_wb_b_tail_unbiased = f_wb_b_tail - ba;

    // integrate
    const auto& delta = integrate_func_(state_ptr,
                                        omg_wb_b_head_unbiased, f_wb_b_head_unbiased,
                                        omg_wb_b_tail_unbiased, f_wb_b_tail_unbiased,
                                        dt);

    // add integrated state delta
    state_ptr->addDelVec(delta);
}

void Propagator::propagateCovariance(const std::shared_ptr<StateBase>& state_ptr, Eigen::MatrixXd& P,
                                     const Eigen::Vector3d& omg_wb_b_head, const Eigen::Vector3d& f_wb_b_head,
                                     const Eigen::Vector3d& omg_wb_b_tail, const Eigen::Vector3d& f_wb_b_tail,
                                     const double dt) {
    // unbiased measurement
    Eigen::Vector3d bg = state_ptr->getGyroscopeBias();
    Eigen::Vector3d ba = state_ptr->getAccelerometerBias();

    Eigen::Vector3d omg_wb_b_head_unbiased = omg_wb_b_head - bg;
    Eigen::Vector3d f_wb_b_head_unbiased = f_wb_b_head - ba;
    Eigen::Vector3d omg_wb_b_tail_unbiased = omg_wb_b_tail - bg;
    Eigen::Vector3d f_wb_b_tail_unbiased = f_wb_b_tail - ba;

    Eigen::Vector3d omg_wb_b_unbiased_avg = 0.5 * (omg_wb_b_head_unbiased + omg_wb_b_tail_unbiased);
    Eigen::Vector3d f_wb_b_unbiased_avg = 0.5 * (f_wb_b_head_unbiased + f_wb_b_tail_unbiased);

    // state transition matrix and input matrix
    const auto F_x = buildStateMatrix(state_ptr,
                                      omg_wb_b_unbiased_avg, f_wb_b_unbiased_avg,
                                      dt);
    const auto F_w = buildInputMatrix(state_ptr,
                                      omg_wb_b_unbiased_avg, f_wb_b_unbiased_avg,
                                      dt);

    // propagate covariance
    P = ((F_x * P * F_x.transpose()) + (F_w * Q_ * F_w.transpose())).eval();
}

Eigen::MatrixXd Propagator::buildStateMatrix(const std::shared_ptr<StateBase>& state_ptr,
                                             const Eigen::Vector3d& omg_wb_b_unbiased, const Eigen::Vector3d& f_wb_b_unbiased,
                                             const double dt) {
    // state transition matrix
    const auto dim_state = static_cast<Eigen::Index>(state_ptr->getDim());
    Eigen::MatrixXd F_x = Eigen::MatrixXd::Identity(dim_state, dim_state);

    // R_wb | R_wb
    F_x.block<3, 3>(0, 0) = SO3::Exp(-omg_wb_b_unbiased * dt).getMat();
    // R_wb | bg_b
    F_x.block<3, 3>(0, 9) = -SO3::Jl(omg_wb_b_unbiased * dt).transpose() * dt;
    // v_wb_w | R_wb
    F_x.block<3, 3>(3, 0) = -state_ptr->getRotation() * SO3::skewVec(f_wb_b_unbiased) * dt;
    // v_wb_w | ba_b
    F_x.block<3, 3>(3, 12) = -state_ptr->getRotation() * dt;
    // v_wb_w | g_wb_w
    F_x.block<3, 3>(3, 15) = Eigen::Matrix3d::Identity() * dt;
    // p_wb_w | v_wb_w
    F_x.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity() * dt;

    return F_x;
}

Eigen::MatrixXd Propagator::buildInputMatrix(const std::shared_ptr<StateBase>& state_ptr,
                                             const Eigen::Vector3d& omg_wb_b_unbiased, const Eigen::Vector3d& f_wb_b_unbiased,
                                             const double dt) {
    // input matrix
    Eigen::MatrixXd F_w = Eigen::MatrixXd::Zero(static_cast<Eigen::Index>(state_ptr->getDim()), 12);

    // R_wb | n_g
    F_w.block<3, 3>(0, 0) = -SO3::Jl(omg_wb_b_unbiased * dt).transpose() * dt;
    // v_wb_w | n_a
    F_w.block<3, 3>(3, 3) = -state_ptr->getRotation() * dt;
    // bg_b | n_bg
    F_w.block<3, 3>(9, 6) = Eigen::Matrix3d::Identity() * dt;
    // ba_b | n_ba
    F_w.block<3, 3>(12, 9) = Eigen::Matrix3d::Identity() * dt;

    return F_w;
}

Eigen::VectorXd Propagator::integrateRungeKutta2(const std::shared_ptr<const StateBase>& state_ptr,
                                                 const Eigen::Vector3d& omg_wb_b_head_unbiased, const Eigen::Vector3d& f_wb_b_head_unbiased,
                                                 const Eigen::Vector3d& omg_wb_b_tail_unbiased, const Eigen::Vector3d& f_wb_b_tail_unbiased,
                                                 const double dt) {
    // using 2-nd order Runge-Kutta method to propagate nominal state
    // navigation state at previous time step
    auto R_wb_i_minus_1 = state_ptr->getRotation();
    auto v_wb_w_i_minus_1 = state_ptr->getVelocity();
    auto g_wb_w_i_minus_1 = state_ptr->getGravity();

    // average angular rate
    Eigen::Vector3d omg_wb_b_avg_unbiased = 0.5 * (omg_wb_b_head_unbiased + omg_wb_b_tail_unbiased);
    // delta theta
    Eigen::Vector3d dtheta = omg_wb_b_avg_unbiased * dt;

    auto R_wb_i = (R_wb_i_minus_1 * SO3::Exp(dtheta)).getMat();
    Eigen::Vector3d a_wb_w = (0.5 * ((R_wb_i_minus_1 * f_wb_b_head_unbiased) + (R_wb_i * f_wb_b_tail_unbiased))) + g_wb_w_i_minus_1;

    // integration increment for state space on direct product manifold
    Eigen::VectorXd delta = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(state_ptr->getDim()));
    Eigen::Vector3d a_dt = a_wb_w * dt;
    // rotation increment
    delta.block<3, 1>(0, 0) = dtheta;
    // velocity increment
    delta.block<3, 1>(3, 0) = a_dt;
    // position increment
    delta.block<3, 1>(6, 0) = (v_wb_w_i_minus_1 * dt) +
                              (0.5 * a_dt * dt);

    return delta;
}

Eigen::VectorXd Propagator::integrateRungeKutta3(const std::shared_ptr<const StateBase>& state_ptr,
                                                 const Eigen::Vector3d& omg_wb_b_head_unbiased, const Eigen::Vector3d& f_wb_b_head_unbiased,
                                                 const Eigen::Vector3d& omg_wb_b_tail_unbiased, const Eigen::Vector3d& f_wb_b_tail_unbiased,
                                                 const double dt) {
    // using 3-rd order Runge-Kutta method to propagate nominal state
    // navigation state at previous time step
    auto R_wb_i_minus_1 = state_ptr->getRotation();
    auto v_wb_w_i_minus_1 = state_ptr->getVelocity();
    auto g_wb_w_i_minus_1 = state_ptr->getGravity();

    // average angular velocity
    Eigen::Vector3d omg_wb_b_avg_unbiased = 0.5 * (omg_wb_b_head_unbiased + omg_wb_b_tail_unbiased);
    // delta theta
    Eigen::Vector3d dtheta = omg_wb_b_avg_unbiased * dt;

    // rotation at current time step
    auto R_wb_i = (R_wb_i_minus_1 * SO3::Exp(dtheta)).getMat();

    // jerk
    Eigen::Vector3d a_wb_w_i_minus_1 = (R_wb_i_minus_1 * f_wb_b_head_unbiased) + g_wb_w_i_minus_1;
    Eigen::Vector3d A_a_dt = ((R_wb_i * f_wb_b_tail_unbiased) - (R_wb_i_minus_1 * f_wb_b_head_unbiased));

    // integration increment for state space on direct product manifold
    Eigen::VectorXd delta = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(state_ptr->getDim()));
    double dt_square = dt * dt;
    // rotation increment
    delta.block<3, 1>(0, 0) = dtheta;
    // velocity increment
    delta.block<3, 1>(3, 0) = (a_wb_w_i_minus_1 * dt) +
                              (0.5 * A_a_dt * dt);
    // position increment
    delta.block<3, 1>(6, 0) = (v_wb_w_i_minus_1 * dt) +
                              (0.5 * a_wb_w_i_minus_1 * dt_square) +
                              (A_a_dt * dt_square / 6.0);

    return delta;
}

Eigen::VectorXd Propagator::integrateRungeKutta4(const std::shared_ptr<const StateBase>& state_ptr,
                                                 const Eigen::Vector3d& omg_wb_b_head_unbiased, const Eigen::Vector3d& f_wb_b_head_unbiased,
                                                 const Eigen::Vector3d& omg_wb_b_tail_unbiased, const Eigen::Vector3d& f_wb_b_tail_unbiased,
                                                 const double dt) {
    // using 4-th order Runge-Kutta method to propagate nominal state
    // lambda function
    auto multiplyQuaternionsJPL = [](const Eigen::Vector4d& quat1, const Eigen::Vector4d& quat2) -> Eigen::Vector4d {
        Eigen::Vector4d quat_result;
        Eigen::Matrix<double, 4, 4> Qm;

        // construct multiplication matrix
        Qm.block<3, 3>(0, 0) = (quat1(3, 0) * Eigen::Matrix3d::Identity()) - SO3::skewVec(quat1.block<3, 1>(0, 0));
        Qm.block<3, 1>(0, 3) = quat1.block<3, 1>(0, 0);
        Qm.block<1, 3>(3, 0) = -quat1.block<3, 1>(0, 0).transpose();
        Qm(3, 3) = quat1(3, 0);
        quat_result = Qm * quat2;

        // ensure unique by forcing q_4 to be > 0
        if (quat_result(3, 0) < 0.0) {
            quat_result *= -1.0;
        }

        // normalize and return
        return quat_result.normalized();
    };

    // Omega matrix for JPL quaternion multiplication
    auto deriveOmegaMatrixJPL = [](const Eigen::Vector3d& omg) -> Eigen::Matrix4d {
        Eigen::Matrix4d Omega;

        Omega.block<3, 3>(0, 0) = -SO3::skewVec(omg);
        Omega.block<1, 3>(3, 0) = -omg.transpose();
        Omega.block<3, 1>(0, 3) = omg;
        Omega(3, 3) = 0.0;

        return Omega;
    };

    // normalize a quaternion for JPL quaternion
    auto normalizeQuaternionJPL = [](Eigen::Vector4d quat) -> Eigen::Vector4d {
        if (quat(3, 0) < 0.0) {
            quat *= -1.0;
        }

        return quat.normalized();
    };

    // transform quaternion to rotation matrix for JPL quaternion
    auto transQuatToRotJPL = [](const Eigen::Vector4d& quat) -> Eigen::Matrix3d {
        Eigen::Vector3d quat_real_part = quat.block<3, 1>(0, 0);
        double quat_imaginary_part = quat(3);

        Eigen::Matrix3d quat_real_part_x = SO3::skewVec(quat_real_part);
        Eigen::Matrix3d rotation = ((2.0 * quat_imaginary_part * quat_imaginary_part - 1.0) * Eigen::Matrix3d::Identity()) -
                                   (2.0 * quat_imaginary_part * quat_real_part_x) +
                                   (2.0 * quat_real_part * quat_real_part.transpose());

        return rotation;
    };

    // transform rotation matrix to quaternion for JPL quaternion
    auto transRotToQuatJPL = [](const Eigen::Matrix3d& rotation) -> Eigen::Vector4d {
        Eigen::Vector4d quat;

        double rot_trace = rotation.trace();
        if ((rotation(0, 0) >= rot_trace) &&
            (rotation(0, 0) >= rotation(1, 1)) &&
            (rotation(0, 0) >= rotation(2, 2))) {
            quat(0) = std::sqrt((1.0 + (2.0 * rotation(0, 0)) - rot_trace) / 4.0);
            quat(1) = (1.0 / (4.0 * quat(0))) * (rotation(0, 1) + rotation(1, 0));
            quat(2) = (1.0 / (4.0 * quat(0))) * (rotation(0, 2) + rotation(2, 0));
            quat(3) = (1.0 / (4.0 * quat(0))) * (rotation(1, 2) - rotation(2, 1));
        } else if ((rotation(1, 1) >= rot_trace) &&
                   (rotation(1, 1) >= rotation(0, 0)) &&
                   (rotation(1, 1) >= rotation(2, 2))) {
            quat(1) = std::sqrt((1.0 + (2.0 * rotation(1, 1)) - rot_trace) / 4.0);
            quat(0) = (1.0 / (4.0 * quat(1))) * (rotation(0, 1) + rotation(1, 0));
            quat(2) = (1.0 / (4.0 * quat(1))) * (rotation(1, 2) + rotation(2, 1));
            quat(3) = (1.0 / (4.0 * quat(1))) * (rotation(2, 0) - rotation(0, 2));
        } else if ((rotation(2, 2) >= rot_trace) &&
                   (rotation(2, 2) >= rotation(0, 0)) &&
                   (rotation(2, 2) >= rotation(1, 1))) {
            quat(2) = std::sqrt((1.0 + (2.0 * rotation(2, 2)) - rot_trace) / 4.0);
            quat(0) = (1.0 / (4.0 * quat(2))) * (rotation(0, 2) + rotation(2, 0));
            quat(1) = (1.0 / (4.0 * quat(2))) * (rotation(1, 2) + rotation(2, 1));
            quat(3) = (1.0 / (4.0 * quat(2))) * (rotation(0, 1) - rotation(1, 0));
        } else {
            quat(3) = std::sqrt((1.0 + rot_trace) / 4.0);
            quat(0) = (1.0 / (4.0 * quat(3))) * (rotation(1, 2) - rotation(2, 1));
            quat(1) = (1.0 / (4.0 * quat(3))) * (rotation(2, 0) - rotation(0, 2));
            quat(2) = (1.0 / (4.0 * quat(3))) * (rotation(0, 1) - rotation(1, 0));
        }

        if (quat(3) < 0.0) {
            quat = -quat;
        }

        return quat.normalized();
    };

    // navigation state at previous time step
    auto R_wb_i_minus_1 = state_ptr->getRotation();
    auto q_bw_vec_i_minus_1 = transRotToQuatJPL(R_wb_i_minus_1.transpose());
    auto v_wb_w_i_minus_1 = state_ptr->getVelocity();
    auto g_wb_w_i_minus_1 = state_ptr->getGravity();

    // rotation
    // angular acceleration
    Eigen::Vector3d A_omg;
    if (dt < 1.0e-8) {
        A_omg = Eigen::Vector3d::Zero();
    } else {
        A_omg = (omg_wb_b_tail_unbiased - omg_wb_b_head_unbiased) / dt;
    }
    // angular velocity point
    Eigen::Vector3d omg_point = omg_wb_b_head_unbiased;
    // quaternion k1
    Eigen::Vector4d dq_0{0.0, 0.0, 0.0, 1.0};
    Eigen::Vector4d q0_dot = 0.5 * deriveOmegaMatrixJPL(omg_point) * dq_0;
    Eigen::Vector4d k1_q = q0_dot * dt;
    // quaternion k2
    omg_point += (0.5 * A_omg * dt);
    Eigen::Vector4d dq_1 = normalizeQuaternionJPL(dq_0 + (0.5 * k1_q));
    Eigen::Vector4d q1_dot = 0.5 * deriveOmegaMatrixJPL(omg_point) * dq_1;
    Eigen::Vector4d k2_q = q1_dot * dt;
    // quaternion k3
    Eigen::Vector4d dq_2 = normalizeQuaternionJPL(dq_0 + (0.5 * k2_q));
    Eigen::Vector4d q2_dot = 0.5 * deriveOmegaMatrixJPL(omg_point) * dq_2;
    Eigen::Vector4d k3_q = q2_dot * dt;
    // quaternion k4
    omg_point += (0.5 * A_omg * dt);
    Eigen::Vector4d dq_3 = normalizeQuaternionJPL(dq_0 + k3_q);
    Eigen::Vector4d q3_dot = 0.5 * deriveOmegaMatrixJPL(omg_point) * dq_3;
    Eigen::Vector4d k4_q = q3_dot * dt;
    // final rotation increment
    Eigen::Vector4d dq = normalizeQuaternionJPL(dq_0 +
                                                ((1.0 / 6.0) * k1_q) +
                                                ((1.0 / 3.0) * k2_q) +
                                                ((1.0 / 3.0) * k3_q) +
                                                ((1.0 / 6.0) * k4_q));

    // rotation update for velocity and position
    Eigen::Vector4d q_bw_vec_i = multiplyQuaternionsJPL(dq, q_bw_vec_i_minus_1);
    Eigen::Matrix3d R_wb_i = transQuatToRotJPL(q_bw_vec_i).transpose();

    // velocity and position
    // jerk
    Eigen::Vector3d A_a;
    if (dt < 1.0e-8) {
        A_a = Eigen::Vector3d::Zero();
    } else {
        A_a = ((R_wb_i * f_wb_b_tail_unbiased) - (R_wb_i_minus_1 * f_wb_b_head_unbiased)) / dt;
    }
    // acceleration point
    Eigen::Vector3d a_point = (R_wb_i_minus_1 * f_wb_b_head_unbiased) + g_wb_w_i_minus_1;
    // velocity and position k1
    Eigen::Vector3d v0_dot = a_point;
    const Eigen::Vector3d& p0_dot = v_wb_w_i_minus_1;
    Eigen::Vector3d k1_v = v0_dot * dt;
    Eigen::Vector3d k1_p = p0_dot * dt;
    // velocity and position k2
    a_point += (0.5 * A_a * dt);
    Eigen::Vector3d v_1 = v_wb_w_i_minus_1 + (0.5 * k1_v);
    Eigen::Vector3d v1_dot = a_point;
    const Eigen::Vector3d& p1_dot = v_1;
    Eigen::Vector3d k2_v = v1_dot * dt;
    Eigen::Vector3d k2_p = p1_dot * dt;
    // velocity and position k3
    Eigen::Vector3d v_2 = v_wb_w_i_minus_1 + (0.5 * k2_v);
    Eigen::Vector3d v2_dot = a_point;
    const Eigen::Vector3d& p2_dot = v_2;
    Eigen::Vector3d k3_v = v2_dot * dt;
    Eigen::Vector3d k3_p = p2_dot * dt;
    // velocity and position k4
    a_point += (0.5 * A_a * dt);
    Eigen::Vector3d v_3 = v_wb_w_i_minus_1 + k3_v;
    Eigen::Vector3d v3_dot = a_point;
    const Eigen::Vector3d& p3_dot = v_3;
    Eigen::Vector3d k4_v = v3_dot * dt;
    Eigen::Vector3d k4_p = p3_dot * dt;

    // integration increment for state space on direct product manifold
    Eigen::VectorXd delta = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(state_ptr->getDim()));
    // rotation increment
    delta.block<3, 1>(0, 0) = SO3::Log(SO3(transQuatToRotJPL(dq).transpose()));
    // velocity increment
    delta.block<3, 1>(3, 0) = ((1.0 / 6.0) * k1_v) +
                              ((1.0 / 3.0) * k2_v) +
                              ((1.0 / 3.0) * k3_v) +
                              ((1.0 / 6.0) * k4_v);
    // position increment
    delta.block<3, 1>(6, 0) = ((1.0 / 6.0) * k1_p) +
                              ((1.0 / 3.0) * k2_p) +
                              ((1.0 / 3.0) * k3_p) +
                              ((1.0 / 6.0) * k4_p);

    return delta;
}

Eigen::VectorXd Propagator::integrateMechanization(const std::shared_ptr<const StateBase>& state_ptr,
                                                   const Eigen::Vector3d& omg_wb_b_head_unbiased, const Eigen::Vector3d& f_wb_b_head_unbiased,
                                                   const Eigen::Vector3d& omg_wb_b_tail_unbiased, const Eigen::Vector3d& f_wb_b_tail_unbiased,
                                                   const double dt) {
    // using mechanization method to propagate nominal state
    // navigation state at previous time step
    auto R_wb_i_minus_1 = state_ptr->getRotation();
    auto v_wb_w_i_minus_1 = state_ptr->getVelocity();
    auto g_wb_w_i_minus_1 = state_ptr->getGravity();

    // delta theta and v for previous time step
    Eigen::Vector3d dtheta_i_minus_1 = omg_wb_b_head_unbiased * dt;
    Eigen::Vector3d dv_i_minus_1 = f_wb_b_head_unbiased * dt;
    // delta theta and v for current time step
    Eigen::Vector3d dtheta_i = omg_wb_b_tail_unbiased * dt;
    Eigen::Vector3d dv_i = f_wb_b_tail_unbiased * dt;

    // error compensation term
    // 2-nd order coning compensation
    Eigen::Vector3d coning_comp = dtheta_i_minus_1.cross(dtheta_i) / 12.0;
    // rotation compensation
    Eigen::Vector3d rotation_comp = 0.5 * dtheta_i.cross(dv_i);
    // sculling compensation
    Eigen::Vector3d sculling_comp = (dtheta_i_minus_1.cross(dv_i) + dv_i_minus_1.cross(dtheta_i)) / 12.0;

    // integration increment for state space on direct product manifold
    Eigen::VectorXd delta = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(state_ptr->getDim()));
    // rotation increment
    delta.block<3, 1>(0, 0) = dtheta_i +
                              coning_comp;
    // velocity increment
    Eigen::Vector3d delta_v_wb_w_i = (R_wb_i_minus_1 * (dv_i + rotation_comp + sculling_comp)) +
                                     (g_wb_w_i_minus_1 * dt);
    delta.block<3, 1>(3, 0) = delta_v_wb_w_i;
    // position increment
    delta.block<3, 1>(6, 0) = (v_wb_w_i_minus_1 + (0.5 * delta_v_wb_w_i)) * dt;

    return delta;
}

Eigen::VectorXd Propagator::integrateACI2(const std::shared_ptr<const StateBase>& state_ptr,
                                          const Eigen::Vector3d& omg_wb_b_head_unbiased, const Eigen::Vector3d& f_wb_b_head_unbiased,
                                          const Eigen::Vector3d& omg_wb_b_tail_unbiased, const Eigen::Vector3d& f_wb_b_tail_unbiased,
                                          const double dt) {
    // using ACI2 method to propagate nominal state
    // navigation state at previous time step
    auto R_wb_i_minus_1 = state_ptr->getRotation();
    auto v_wb_w_i_minus_1 = state_ptr->getVelocity();
    auto g_wb_w_i_minus_1 = state_ptr->getGravity();

    // average angular rate and specific force
    Eigen::Vector3d omg_wb_b_avg_unbiased = 0.5 * (omg_wb_b_head_unbiased + omg_wb_b_tail_unbiased);
    Eigen::Vector3d f_wb_b_avg_unbiased = 0.5 * (f_wb_b_head_unbiased + f_wb_b_tail_unbiased);

    // integrated term
    Eigen::Matrix<double, 3, 6> Xi_packed = Eigen::Matrix<double, 3, 6>::Zero(3, 6);
    assembleXiMatrixPack(Xi_packed, omg_wb_b_avg_unbiased, dt);
    // Xi matrix block
    Eigen::Matrix3d Xi_1 = Xi_packed.block(0, 0, 3, 3);
    Eigen::Matrix3d Xi_2 = Xi_packed.block(0, 3, 3, 3);

    // integration increment for state space on direct product manifold
    Eigen::VectorXd delta = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(state_ptr->getDim()));
    Eigen::Vector3d g_dt = g_wb_w_i_minus_1 * dt;
    // rotation increment
    delta.block<3, 1>(0, 0) = omg_wb_b_avg_unbiased * dt;
    // velocity increment
    delta.block<3, 1>(3, 0) = (R_wb_i_minus_1 * Xi_1 * f_wb_b_avg_unbiased) +
                              g_dt;
    // position increment
    delta.block<3, 1>(6, 0) = (v_wb_w_i_minus_1 * dt) +
                              (R_wb_i_minus_1 * Xi_2 * f_wb_b_avg_unbiased) +
                              (0.5 * g_dt * dt);

    return delta;
}

void Propagator::assembleXiMatrixPack(Eigen::Matrix<double, 3, 6>& Xi_packed,
                                      const Eigen::Vector3d& omg_wb_b_avg_unbiased,
                                      const double dt) {
    // decompose angular velocity
    double omg_norm = omg_wb_b_avg_unbiased.norm();
    double dtheta = omg_norm * dt;
    Eigen::Vector3d omg_normalized = Eigen::Vector3d::Zero();
    if (omg_norm > 1.0e-12) {
        omg_normalized = omg_wb_b_avg_unbiased / omg_norm;
    }

    // temporal matrix and vector
    double dt_square = dt * dt;
    double omg_norm_square = omg_norm * omg_norm;
    double cos_dtheta = std::cos(dtheta);
    double sin_dtheta = std::sin(dtheta);
    Eigen::Matrix3d omg_normalized_x = SO3::skewVec(omg_normalized);
    Eigen::Matrix3d omg_normalized_x_square = omg_normalized_x * omg_normalized_x;

    // integration component
    Eigen::Matrix3d Xi_1, Xi_2;
    // begin to integrate each component
    bool is_small_omg = (omg_norm < (0.5 * M_PI / 180.0));
    if (!is_small_omg) {
        // 1-st order rotation integration with constant omega
        Xi_1 = (dt * Eigen::Matrix3d::Identity()) + ((1.0 - cos_dtheta) / omg_norm * omg_normalized_x) +
               ((dt - sin_dtheta / omg_norm) * omg_normalized_x_square);
        // 2-nd order rotation integration with constant omega
        Xi_2 = (0.5 * dt_square * Eigen::Matrix3d::Identity()) + ((dtheta - sin_dtheta) / omg_norm_square * omg_normalized_x) +
               (((0.5 * dt_square) - ((1.0 - cos_dtheta) / omg_norm_square)) * omg_normalized_x_square);
    } else {
        // 1-st order rotation integration with constant omega
        Xi_1 = dt * (Eigen::Matrix3d::Identity() + ((1.0 - cos_dtheta) * omg_normalized_x_square) +
                     (sin_dtheta * omg_normalized_x));
        // 2-nd order rotation integration with constant omega
        Xi_2 = 0.5 * dt * Xi_1;
    }

    // store integrated parameter
    Xi_packed.setZero();
    Xi_packed.block<3, 3>(0, 0) = Xi_1;
    Xi_packed.block<3, 3>(0, 3) = Xi_2;
}

} // namespace fmcw_lio
