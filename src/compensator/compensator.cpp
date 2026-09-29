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

#include "compensator/compensator.hpp"

// fmcw_lio
namespace fmcw_lio {

// Compensator
Compensator::Compensator(const std::shared_ptr<const Config>& config_ptr,
                         IntegrateFunction compensate_func) : logger_compensation_ptr(std::make_shared<LoggerCompensation>()),
                                                              config_ptr_(config_ptr),
                                                              integrate_func_(std::move(compensate_func)),
                                                              point_weight_func_(point_weight_table_.at(config_ptr_->point_weight_method)) {}

void Compensator::compensateScan(const std::shared_ptr<StateBase>& state_ptr,
                                 const std::shared_ptr<const std::vector<std::shared_ptr<const StateBase>>>& propagation_states_ptr,
                                 const PointCloudType::Ptr& scan_compensated_ptr) {
    // start timer
    std::chrono::steady_clock::time_point t1, t2;
    t1 = std::chrono::steady_clock::now();

    // compare state time lambda
    auto compareStateTime = [](const std::shared_ptr<const StateBase>& propagation_state_ptr,
                               const double t) -> bool {
        return (propagation_state_ptr->getTime() <= t);
    };

    if (scan_compensated_ptr->points.empty() ||
        propagation_states_ptr->empty()) {
        return;
    }

    // propagated state estimate at scan end time
    auto R_wb_k = state_ptr->getRotation();
    auto v_wb_w_k = state_ptr->getVelocity();
    auto p_wb_w_k = state_ptr->getPosition();

    // angular rate at scan end time
    auto prop_state_end_ptr = std::dynamic_pointer_cast<const StateKinematics>(propagation_states_ptr->back());
    auto omg_scan_end_time = prop_state_end_ptr->getAngRotRate();

    // LiDAR velocity w.r.t. world frame in world frame at scan end time
    Eigen::Vector3d v_wl_w_k = v_wb_w_k + (R_wb_k * omg_scan_end_time.cross(config_ptr_->p_bl_b));
    // LiDAR position w.r.t. world frame in world frame at scan end time
    Eigen::Vector3d p_wl_w_k = p_wb_w_k + (R_wb_k * config_ptr_->p_bl_b);

    // point time at scan end time
    auto point_time_scan_end = scan_compensated_ptr->points.back().curvature;

    // kinematics states for motion compensation
    std::vector<std::shared_ptr<const StateKinematics>> prop_kin_states;
    prop_kin_states.reserve(propagation_states_ptr->size());
    for (const auto& propagation_state_ptr : *propagation_states_ptr) {
        prop_kin_states.push_back(std::dynamic_pointer_cast<const StateKinematics>(propagation_state_ptr));
    }

    // unique id of current compensation for thread-local cache of integrated state
    static std::atomic<std::uint64_t> compensation_count{0};
    const std::uint64_t compensation_id = ++compensation_count;

    // compensation via propagation
    std::vector<std::size_t> point_indices(scan_compensated_ptr->points.size());
    std::iota(point_indices.begin(), point_indices.end(), std::size_t{0});
    std::for_each(std::execution::par,
                  point_indices.begin(), point_indices.end(),
                  [&](std::size_t idx) -> void {
        auto& point = scan_compensated_ptr->points[idx];

        // integrated state at point sampling time
        thread_local std::uint64_t cached_compensation_id = 0;
        thread_local float cached_point_curvature = 0.0f;
        thread_local Eigen::Matrix3d R_wb_i, R_wl_i;
        thread_local Eigen::Vector3d v_wb_w_i, p_wb_w_i, v_wl_w_i, p_wl_w_i;
        if ((cached_compensation_id != compensation_id) ||
            !((cached_point_curvature == point.curvature) &&
            (std::signbit(cached_point_curvature) == std::signbit(point.curvature)))) {
            // search head and tail propagation state for point
            float point_time = point.curvature * 1.0e-3f;
            const auto it = std::lower_bound(propagation_states_ptr->begin(), propagation_states_ptr->end(),
                                             point_time,
                                             compareStateTime);
            auto tail_idx = static_cast<std::size_t>(it - propagation_states_ptr->begin());
            if (it == propagation_states_ptr->end()) {
                tail_idx -= 1;
            } else if (tail_idx == 0) {
                // point is slightly earlier than first propagation state
                tail_idx = 1;
            }

            // kinematics state for motion compensation
            const auto& prop_state_head = prop_kin_states[tail_idx - 1];
            const auto& prop_state_tail = prop_kin_states[tail_idx];

            // unbiased angular rate and specific force of head and tail propagated state
            Eigen::Vector3d omg_wb_b_head = prop_state_head->getAngRotRate();
            Eigen::Vector3d f_wb_b_head = prop_state_head->getLinVelRate();

            Eigen::Vector3d omg_wb_b_tail = prop_state_tail->getAngRotRate();
            Eigen::Vector3d f_wb_b_tail = prop_state_tail->getLinVelRate();

            // time duration between two adjacent propagation states
            double dt_prop = prop_state_tail->getTime() - prop_state_head->getTime();
            // time duration from head propagation state
            double dt_i = point_time - prop_state_head->getTime();

            // weight unbiased angular rate and specific force at point sampling time
            Eigen::Vector3d omg_wb_b_i, f_wb_b_i;
            std::tie(omg_wb_b_i, f_wb_b_i) = weightIMUMeas(omg_wb_b_head, f_wb_b_head,
                                                           omg_wb_b_tail, f_wb_b_tail,
                                                           dt_prop,
                                                           dt_i);

            // integrate state to point sampling time
            thread_local Eigen::VectorXd delta;
            StateKinematics state_i(*prop_state_head);
            integrate_func_((*propagation_states_ptr)[tail_idx - 1],
                            omg_wb_b_head, f_wb_b_head,
                            omg_wb_b_i, f_wb_b_i,
                            dt_i,
                            delta);
            state_i.addDelVec(delta);
            // rotation, velocity, position at point sampling time
            R_wb_i = state_i.getRotation();
            v_wb_w_i = state_i.getVelocity();
            p_wb_w_i = state_i.getPosition();

            if (config_ptr_->use_doppler) {
                // LiDAR rotation w.r.t. world frame at point sampling time
                R_wl_i = R_wb_i * config_ptr_->R_bl;
                // LiDAR velocity w.r.t. world frame in world frame at point sampling time
                v_wl_w_i = v_wb_w_i + (R_wb_i * omg_wb_b_i.cross(config_ptr_->p_bl_b));
                // LiDAR position w.r.t. world frame in world frame at point sampling time
                p_wl_w_i = p_wb_w_i + (R_wb_i * config_ptr_->p_bl_b);
            }

            cached_compensation_id = compensation_id;
            cached_point_curvature = point.curvature;
        }

        // point in LiDAR frame at sampling time
        Eigen::Vector3d p_lp_l_i{point.x, point.y, point.z};

        // point position compensation
        Eigen::Vector3d p_lp_b_i = (config_ptr_->R_bl * p_lp_l_i) + config_ptr_->p_bl_b;
        Eigen::Vector3d p_lp_l_k = config_ptr_->R_bl.transpose() * ((R_wb_k.transpose() *
                                   ((R_wb_i * p_lp_b_i) + p_wb_w_i - p_wb_w_k)) - config_ptr_->p_bl_b);

        // temporal-spatial compensation: timestamp, x, y, z (and Doppler velocity)
        point.curvature = point_time_scan_end;
        point.x = static_cast<float>(p_lp_l_k.x());
        point.y = static_cast<float>(p_lp_l_k.y());
        point.z = static_cast<float>(p_lp_l_k.z());

        // Doppler velocity compensation
        if (config_ptr_->use_doppler) {
            // point position w.r.t. LiDAR frame in world frame
            Eigen::Vector3d p_lp_w_i = R_wl_i * p_lp_l_i;

            // point Doppler velocity at point sampling time
            double v_d_i = point.intensity;
            // compensated Doppler velocity
            double v_d_k = ((p_lp_l_i.norm() / p_lp_l_k.norm()) * v_d_i) - ((1.0 / p_lp_l_k.norm()) *
                           (p_lp_w_i.dot((v_wl_w_k - v_wl_w_i)) + (p_wl_w_i - p_wl_w_k).dot(v_wl_w_k)));

            point.intensity = static_cast<float>(v_d_k);
        }
    });

    // stop timer
    t2 = std::chrono::steady_clock::now();
    logger_compensation_ptr->compensation_time = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()) * 1.0e-3;
}

} // namespace fmcw_lio
