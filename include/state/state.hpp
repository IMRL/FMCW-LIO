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

#ifndef STATE_HPP
#define STATE_HPP

#include "common/algebra.hpp"
#include "common/config.hpp"

// fmcw_lio
namespace fmcw_lio {

// StateBase
class StateBase {
public:
    explicit StateBase(const std::size_t dim) noexcept : dim_(dim),
                                                         time_{} {}

    virtual ~StateBase() = default;

    // clone state
    [[nodiscard]] virtual std::shared_ptr<StateBase> cloneState() const = 0;

    // add and subtract state
    virtual void addDelVec(const Eigen::VectorXd& delta) = 0;

    [[nodiscard]] virtual Eigen::VectorXd subState(const std::shared_ptr<StateBase>& state_ptr) const = 0;

    // state setter
    void setTime(const double time) noexcept { time_ = time; }

    virtual void setState(const Eigen::Matrix3d& mat3,
                          const Eigen::Vector3d& vec3_1,
                          const Eigen::Vector3d& vec3_2,
                          const Eigen::Vector3d& vec3_3,
                          const Eigen::Vector3d& vec3_4,
                          const Eigen::Vector3d& vec3_5) = 0;

    // state getter
    [[nodiscard]] std::size_t getDim() const noexcept { return dim_; }

    [[nodiscard]] double getTime() const noexcept { return time_; }

    [[nodiscard]] virtual Eigen::Matrix3d getRotation() const = 0;

    [[nodiscard]] virtual Eigen::Vector3d getVelocity() const = 0;

    [[nodiscard]] virtual Eigen::Vector3d getPosition() const = 0;

    [[nodiscard]] virtual Eigen::Vector3d getGyroscopeBias() const = 0;

    [[nodiscard]] virtual Eigen::Vector3d getAccelerometerBias() const = 0;

    [[nodiscard]] virtual Eigen::Vector3d getGravity() const = 0;

private:
    // state dimension
    const std::size_t dim_;

    // state time
    double time_;
};

// StatePoseQuat
class StatePoseQuat : public StateBase {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // additional constructor
    explicit StatePoseQuat(double t = 0.0,
                           const Eigen::Quaterniond& q = Eigen::Quaterniond::Identity(),
                           Eigen::Vector3d p = Eigen::Vector3d::Zero());

    ~StatePoseQuat() override = default;

    // clone state
    [[nodiscard]] std::shared_ptr<StateBase> cloneState() const override { return std::make_shared<StatePoseQuat>(*this); }

    // add state with a delta vector for state space on direct product manifold
    void addDelVec(const Eigen::VectorXd& delta) override;

    // state subtract another state for state space on direct product manifold
    [[nodiscard]] Eigen::VectorXd subState(const std::shared_ptr<StateBase>& state_ptr) const override;

    // set state with rotation and position
    void setState(const Eigen::Matrix3d& R,
                  const Eigen::Vector3d& p,
                  const Eigen::Vector3d& vec3_2,
                  const Eigen::Vector3d& vec3_3,
                  const Eigen::Vector3d& vec3_4,
                  const Eigen::Vector3d& vec3_5) override;

    // state getter
    // get rotation matrix
    [[nodiscard]] Eigen::Matrix3d getRotation() const override { return rotation_quat_.toRotationMatrix(); }

    // get velocity vector
    [[nodiscard]] Eigen::Vector3d getVelocity() const override { throw std::logic_error("Error: Invalid function."); }

    // get position vector
    [[nodiscard]] Eigen::Vector3d getPosition() const override { return position_.matrix(); }

    // get gyroscope bias vector
    [[nodiscard]] Eigen::Vector3d getGyroscopeBias() const override { throw std::logic_error("Error: Invalid function."); }

    // get accelerometer bias vector
    [[nodiscard]] Eigen::Vector3d getAccelerometerBias() const override { throw std::logic_error("Error: Invalid function."); }

    // get gravity vector in world frame
    [[nodiscard]] Eigen::Vector3d getGravity() const override { throw std::logic_error("Error: Invalid function."); }

    // additional getter
    // get rotation quaternion
    [[nodiscard]] Eigen::Quaterniond getRotationQuat() const noexcept { return rotation_quat_; }

private:
    // pose state
    Eigen::Quaterniond rotation_quat_;
    Eigen::Vector3d position_;
};

// StateKinematics
class StateKinematics : public StateBase {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // additional constructor
    explicit StateKinematics(double t = 0.0,
                             Eigen::Matrix3d R = Eigen::Matrix3d::Identity(),
                             Eigen::Vector3d v = Eigen::Vector3d::Zero(),
                             Eigen::Vector3d p = Eigen::Vector3d::Zero(),
                             Eigen::Vector3d omg = Eigen::Vector3d::Zero(),
                             Eigen::Vector3d a_or_f = Eigen::Vector3d::Zero(),
                             Eigen::Vector3d g = Eigen::Vector3d{0.0, 0.0, -9.81});

    ~StateKinematics() override = default;

    // clone state
    [[nodiscard]] std::shared_ptr<StateBase> cloneState() const override { return std::make_shared<StateKinematics>(*this); }

    // add state with a delta vector for state space on direct product manifold
    void addDelVec(const Eigen::VectorXd& delta) override;

    // state subtract another state for state space on direct product manifold
    [[nodiscard]] Eigen::VectorXd subState(const std::shared_ptr<StateBase>& state_ptr) const override;

    // set state with rotation, velocity, position, angular rate, specific force, gravity
    void setState(const Eigen::Matrix3d& R,
                  const Eigen::Vector3d& v,
                  const Eigen::Vector3d& p,
                  const Eigen::Vector3d& omg,
                  const Eigen::Vector3d& a_or_f,
                  const Eigen::Vector3d& g) override;

    // state getter
    // get rotation matrix
    [[nodiscard]] Eigen::Matrix3d getRotation() const override { return rotation_.matrix(); }

    // get velocity vector
    [[nodiscard]] Eigen::Vector3d getVelocity() const override { return velocity_.matrix(); }

    // get position vector
    [[nodiscard]] Eigen::Vector3d getPosition() const override { return position_.matrix(); }

    // get gyroscope bias vector
    [[nodiscard]] Eigen::Vector3d getGyroscopeBias() const override { throw std::logic_error("Error: Invalid function."); }

    // get accelerometer bias vector
    [[nodiscard]] Eigen::Vector3d getAccelerometerBias() const override { throw std::logic_error("Error: Invalid function."); }

    // get gravity vector in world frame
    [[nodiscard]] Eigen::Vector3d getGravity() const override { return gravity_.matrix(); }

    // additional getter
    // get angular rotation rate (angular velocity or angular rate)
    [[nodiscard]] Eigen::Vector3d getAngRotRate() const noexcept { return angular_rotation_rate_.matrix(); }

    // get linear velocity rate (acceleration or specific force)
    [[nodiscard]] Eigen::Vector3d getLinVelRate() const noexcept { return linear_velocity_rate_.matrix(); }

private:
    // state in propagation
    Eigen::Matrix3d rotation_;
    Eigen::Vector3d velocity_;
    Eigen::Vector3d position_;
    Eigen::Vector3d angular_rotation_rate_;
    Eigen::Vector3d linear_velocity_rate_;
    Eigen::Vector3d gravity_;
};

// StateSystem
class StateSystem : public StateBase {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // additional constructor
    explicit StateSystem(const std::shared_ptr<const Config>& config_ptr);

    explicit StateSystem(double t = 0.0,
                         const SO3& R = SO3(Eigen::Matrix3d::Identity()),
                         Eigen::Vector3d v = Eigen::Vector3d::Zero(),
                         Eigen::Vector3d p = Eigen::Vector3d::Zero(),
                         Eigen::Vector3d bg = Eigen::Vector3d::Zero(),
                         Eigen::Vector3d ba = Eigen::Vector3d::Zero(),
                         Eigen::Vector3d g = Eigen::Vector3d{0.0, 0.0, -9.81});

    ~StateSystem() override = default;

    // clone state
    [[nodiscard]] std::shared_ptr<StateBase> cloneState() const override { return std::make_shared<StateSystem>(*this); }

    // add state with a delta vector for state space on direct product manifold
    void addDelVec(const Eigen::VectorXd& delta) override;

    // state subtract another state for state space on direct product manifold
    [[nodiscard]] Eigen::VectorXd subState(const std::shared_ptr<StateBase>& state_ptr) const override;

    // set state with rotation, velocity, position, gyroscope bias, accelerometer bias, and gravity
    void setState(const Eigen::Matrix3d& R_wb,
                  const Eigen::Vector3d& v_wb_w,
                  const Eigen::Vector3d& p_wb_w,
                  const Eigen::Vector3d& bg_b,
                  const Eigen::Vector3d& ba_b,
                  const Eigen::Vector3d& g_wb_w) override;

    // state getter
    // get rotation matrix
    [[nodiscard]] Eigen::Matrix3d getRotation() const override { return R_wb_.getMat(); }

    // get velocity vector
    [[nodiscard]] Eigen::Vector3d getVelocity() const override { return v_wb_w_.matrix(); }

    // get position vector
    [[nodiscard]] Eigen::Vector3d getPosition() const override { return p_wb_w_.matrix(); }

    // get gyroscope bias vector
    [[nodiscard]] Eigen::Vector3d getGyroscopeBias() const override { return bg_b_.matrix(); }

    // get accelerometer bias vector
    [[nodiscard]] Eigen::Vector3d getAccelerometerBias() const override { return ba_b_.matrix(); }

    // get gravity vector in world frame
    [[nodiscard]] Eigen::Vector3d getGravity() const override { return g_wb_w_.matrix(); }

    // additional getter
    // get rotation on SO(3)
    [[nodiscard]] SO3 getRotationSO3() const noexcept { return R_wb_; }

    // get rotation in quaternion
    [[nodiscard]] Eigen::Quaterniond getRotationQuat() const noexcept { return R_wb_.getQuat(); }

private:
    // navigation state
    SO3 R_wb_;
    Eigen::Vector3d v_wb_w_;
    Eigen::Vector3d p_wb_w_;
    // IMU bias
    Eigen::Vector3d bg_b_;
    Eigen::Vector3d ba_b_;
    // gravity
    Eigen::Vector3d g_wb_w_;
};

} // namespace fmcw_lio

#endif // STATE_HPP
