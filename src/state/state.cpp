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

#include "state/state.hpp"

// fmcw_lio
namespace fmcw_lio {

// StatePoseQuat
StatePoseQuat::StatePoseQuat(double t,
                             const Eigen::Quaterniond& q,
                             Eigen::Vector3d p) : StateBase(6),
                                                  rotation_quat_(q),
                                                  position_(std::move(p)) {
    // set state time
    setTime(t);
}

void StatePoseQuat::setState(const Eigen::Matrix3d& R,
                             const Eigen::Vector3d& p,
                             const Eigen::Vector3d& vec3_2,
                             const Eigen::Vector3d& vec3_3,
                             const Eigen::Vector3d& vec3_4,
                             const Eigen::Vector3d& vec3_5) {
    // set state
    rotation_quat_ = Eigen::Quaterniond(R);
    position_ = p;
}

void StatePoseQuat::addDelVec(const Eigen::VectorXd& delta) {
    // invalid function
    throw std::logic_error("Error: Invalid function.");
}

Eigen::VectorXd StatePoseQuat::subState(const std::shared_ptr<StateBase>& state_ptr) const {
    // invalid function
    throw std::logic_error("Error: Invalid function.");
}

// StateKinematics
StateKinematics::StateKinematics(double t,
                                 Eigen::Matrix3d R,
                                 Eigen::Vector3d v,
                                 Eigen::Vector3d p,
                                 Eigen::Vector3d omg,
                                 Eigen::Vector3d a_or_f,
                                 Eigen::Vector3d g) : StateBase(18),
                                                      rotation_(std::move(R)),
                                                      velocity_(std::move(v)),
                                                      position_(std::move(p)),
                                                      angular_rotation_rate_(std::move(omg)),
                                                      linear_velocity_rate_(std::move(a_or_f)),
                                                      gravity_(std::move(g)) {
    // set state time
    setTime(t);
}

void StateKinematics::setState(const Eigen::Matrix3d& R,
                               const Eigen::Vector3d& v,
                               const Eigen::Vector3d& p,
                               const Eigen::Vector3d& omg,
                               const Eigen::Vector3d& a_or_f,
                               const Eigen::Vector3d& g) {
    // set state
    rotation_ = R;
    velocity_ = v;
    position_ = p;
    angular_rotation_rate_ = omg;
    linear_velocity_rate_ = a_or_f;
    gravity_ = g;
}

void StateKinematics::addDelVec(const Eigen::VectorXd& delta) {
    // state order: rotation, velocity, position
    // rotation
    rotation_ = (rotation_ * SO3::Exp(delta.segment<3>(0).eval())).getMat();
    // velocity
    velocity_ += delta.segment<3>(3).eval();
    // position
    position_ += delta.segment<3>(6).eval();
}

Eigen::VectorXd StateKinematics::subState(const std::shared_ptr<StateBase>& state_ptr) const {
    // invalid function
    throw std::logic_error("Error: Invalid function.");
}

// StateSystem
StateSystem::StateSystem(const std::shared_ptr<const Config>& config_ptr) : StateBase(18),
                                                                            R_wb_(Eigen::Matrix3d::Identity()),
                                                                            v_wb_w_(Eigen::Vector3d::Zero()),
                                                                            p_wb_w_(Eigen::Vector3d::Zero()),
                                                                            bg_b_(Eigen::Vector3d::Zero()),
                                                                            ba_b_(Eigen::Vector3d::Zero()),
                                                                            g_wb_w_(Eigen::Vector3d{0.0, 0.0, -config_ptr->gravity_scale}) {}

StateSystem::StateSystem(double t,
                         SO3 R,
                         Eigen::Vector3d v,
                         Eigen::Vector3d p,
                         Eigen::Vector3d bg,
                         Eigen::Vector3d ba,
                         Eigen::Vector3d g) : StateBase(18),
                                              R_wb_(std::move(R)),
                                              v_wb_w_(std::move(v)),
                                              p_wb_w_(std::move(p)),
                                              bg_b_(std::move(bg)),
                                              ba_b_(std::move(ba)),
                                              g_wb_w_(std::move(g)) {
    // set state time
    setTime(t);
}

void StateSystem::setState(const Eigen::Matrix3d& R_wb,
                           const Eigen::Vector3d& v_wb_w,
                           const Eigen::Vector3d& p_wb_w,
                           const Eigen::Vector3d& bg_b,
                           const Eigen::Vector3d& ba_b,
                           const Eigen::Vector3d& g_wb_w) {
    // set state
    R_wb_ = SO3(R_wb);
    v_wb_w_ = v_wb_w;
    p_wb_w_ = p_wb_w;
    bg_b_ = bg_b;
    ba_b_ = ba_b;
    g_wb_w_ = g_wb_w;
}

void StateSystem::addDelVec(const Eigen::VectorXd& delta) {
    // state order: R_wb, v_wb_w, p_wb_w, bg_b, ba_b, g_wb_w
    // rotation
    R_wb_ = R_wb_ * SO3::Exp(delta.segment<3>(0).eval());
    // velocity
    v_wb_w_ += delta.segment<3>(3).eval();
    // position
    p_wb_w_ += delta.segment<3>(6).eval();
    // gyroscope bias
    bg_b_ += delta.segment<3>(9).eval();
    // accelerometer bias
    ba_b_ += delta.segment<3>(12).eval();
    // gravity
    g_wb_w_ += delta.segment<3>(15).eval();
}

Eigen::VectorXd StateSystem::subState(const std::shared_ptr<StateBase>& state_ptr) const {
    // downcast to the derived state and throw std::bad_cast on type mismatch
    const auto& other = dynamic_cast<const StateSystem&>(*state_ptr);

    // state difference
    Eigen::VectorXd delta = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(getDim()));

    // rotation
    delta.segment<3>(0) = SO3::Log(other.R_wb_.getInv() * this->R_wb_);
    // velocity
    delta.segment<3>(3) = this->v_wb_w_ - other.v_wb_w_;
    // position
    delta.segment<3>(6) = this->p_wb_w_ - other.p_wb_w_;
    // gyroscope bias
    delta.segment<3>(9) = this->bg_b_ - other.bg_b_;
    // accelerometer bias
    delta.segment<3>(12) = this->ba_b_ - other.ba_b_;
    // gravity
    delta.segment<3>(15) = this->g_wb_w_ - other.g_wb_w_;

    return delta;
}

} // namespace fmcw_lio
