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

#include "estimator/updater.hpp"

// fmcw_lio
namespace fmcw_lio {

// Updater
Updater::Updater(const std::shared_ptr<const Config>& config_ptr) : logger_update_ptr(std::make_shared<LoggerUpdate>(config_ptr->iteration_num)),
                                                                    config_ptr_(config_ptr) {}

void Updater::updateByVelocity(const std::shared_ptr<StateBase>& state_ptr, Eigen::MatrixXd& P,
                               const Eigen::Vector3d& v_wl_l, const Eigen::Matrix3d& R,
                               const Eigen::Vector3d& omg_wb_b_scan_end_time) {
    // start timer
    std::chrono::steady_clock::time_point t1, t2;
    t1 = std::chrono::steady_clock::now();

    const auto dim_state = static_cast<Eigen::Index>(state_ptr->getDim());

    // residual and Jacobian matrix
    Eigen::Vector3d r = Eigen::Vector3d::Zero();
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, dim_state);

    // rotation
    auto R_wb = state_ptr->getRotation();
    // velocity
    auto v_wb_w = state_ptr->getVelocity();

    // angular velocity
    Eigen::Vector3d omg_wb_b_unbiased = omg_wb_b_scan_end_time - state_ptr->getGyroscopeBias();

    Eigen::Vector3d v_wb_b = R_wb.transpose() * v_wb_w;
    Eigen::Matrix3d omg_wb_b_unbiased_x = SO3::skewVec(omg_wb_b_unbiased);
    Eigen::Vector3d v_wl_l_estimated = config_ptr_->R_bl.transpose() * (v_wb_b + omg_wb_b_unbiased_x * config_ptr_->p_bl_b);

    // residual
    r.block(0, 0, 3, 1) = v_wl_l - v_wl_l_estimated;

    // Jacobian matrix
    H.block(0, 0, 3, 3) = config_ptr_->R_bl.transpose() * SO3::skewVec(v_wb_b);
    H.block(0, 3, 3, 3) = config_ptr_->R_bl.transpose() * R_wb.transpose();
    H.block(0, 9, 3, 3) = config_ptr_->R_bl.transpose() * SO3::skewVec(config_ptr_->p_bl_b);

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
        state_ptr->addDelVec(dx);
        // update covariance
        P = (Eigen::MatrixXd::Identity(dim_state, dim_state) - (K * H)) * P;

        // projection Jacobian for covariance of posterior state point
        Eigen::MatrixXd L = Eigen::MatrixXd::Identity(dim_state, dim_state);
        L.block(0, 0, 3, 3) = SO3::Jl(dx.segment(0, 3).eval()).transpose();
        P = L * P * L.transpose();
    };

    // update state and covariance
    update_state_and_cov();

    // stop timer
    t2 = std::chrono::steady_clock::now();
    logger_update_ptr->velocity_based_update_time = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()) * 1.0e-3;
}

std::size_t Updater::updateByPlane(const std::shared_ptr<StateBase>& state_ptr, Eigen::MatrixXd& P,
                                   const PointCloudType::Ptr& scan_down_lidar_ptr, const double R_point,
                                   const std::shared_ptr<Map>& map_ptr, PointVec2D& nearest_points,
                                   const std::size_t correspondence_thresh) {
    // valid point flag
    const std::size_t size_flag = 100000;
    std::array<bool, size_flag> valid_point_flags{{false}};

    // point cloud with valid point and corresponding local plane parameter
    auto point_down_size = scan_down_lidar_ptr->size();
    PointCloudType::Ptr plane_params_ptr(new PointCloudType());
    PointCloudType::Ptr valid_points_ptr(new PointCloudType());
    plane_params_ptr->resize(point_down_size);
    valid_points_ptr->resize(point_down_size);

    // convergence flag
    bool is_convergence = true;
    std::size_t convergence_num = 0;

    Eigen::MatrixXd P_prop = P;
    const auto state_prop = state_ptr->cloneState();
    const auto dim_state = static_cast<Eigen::Index>(state_ptr->getDim());

    for (std::size_t i = 0; i <= config_ptr_->iteration_num; ++i) {
        // flag for iteration
        bool is_valid_iteration = true;

        plane_params_ptr->clear();
        valid_points_ptr->clear();

        // start timer
        std::chrono::steady_clock::time_point t1, t2, t3;
        t1 = std::chrono::steady_clock::now();

        PointCloudType::Ptr valid_plane_params_ptr(new PointCloudType());
        valid_plane_params_ptr->resize(point_down_size);

        // get current state estimate for transforming points
        auto R_wb = state_ptr->getRotation();
        auto p_wb_w = state_ptr->getPosition();

        // search nearest points in world frame and find plane
        std::vector<std::size_t> point_indices(point_down_size);
        std::for_each(point_indices.begin(), point_indices.end(), [n = 0](std::size_t& idx) mutable -> void { idx = n++; });
        std::for_each(std::execution::par,
                      point_indices.begin(), point_indices.end(),
                      [&](auto point_idx) -> void {
            PointType& point_lidar = scan_down_lidar_ptr->points[point_idx];
            Eigen::Vector3d p_lp_l(point_lidar.x, point_lidar.y, point_lidar.z);
            Eigen::Vector3d p_wp_w(R_wb * (config_ptr_->R_bl * p_lp_l + config_ptr_->p_bl_b) + p_wb_w);

            PointType point_world;
            point_world.curvature = point_lidar.curvature;
            point_world.x = static_cast<float>(p_wp_w(0));
            point_world.y = static_cast<float>(p_wp_w(1));
            point_world.z = static_cast<float>(p_wp_w(2));
            point_world.intensity = point_lidar.intensity;

            // storing squared distance of point to nearest points
            std::vector<float> nearest_points_sq_dist(config_ptr_->nearest_point_num);
            // vector in ascending order according to distance to point_world
            auto& points_near = nearest_points[point_idx];
            if (is_convergence) {
                map_ptr->searchNearestPoints(point_world, config_ptr_->nearest_point_num,
                                             points_near, nearest_points_sq_dist);
                // valid point if number of nearest points > threshold and largest distance < threshold
                valid_point_flags[point_idx] = ((points_near.size() >= config_ptr_->nearest_point_num) &&
                                                (nearest_points_sq_dist[config_ptr_->nearest_point_num - 1] <= 5.0));
            }

            if (!valid_point_flags[point_idx]) {
                return;
            }

            Eigen::Vector4d plane_param;
                          valid_point_flags[point_idx] = false;

            // point-plane distance
            if (estimatePlane(plane_param,
                              points_near,
                              0.1)) {
                // point-plane distance
                double point2plane_dist = (plane_param(0) * point_world.x) +
                                          (plane_param(1) * point_world.y) +
                                          (plane_param(2) * point_world.z) +
                                          plane_param(3);

                double score = 1.0 - (0.9 * std::fabs(point2plane_dist) / std::sqrt(p_lp_l.norm()));
                if (score > 0.9) {
                    valid_point_flags[point_idx] = true;

                    // normal vector and point-plane distance
                    plane_params_ptr->points[point_idx].x = static_cast<float>(plane_param(0));
                    plane_params_ptr->points[point_idx].y = static_cast<float>(plane_param(1));
                    plane_params_ptr->points[point_idx].z = static_cast<float>(plane_param(2));
                    plane_params_ptr->points[point_idx].intensity = static_cast<float>(point2plane_dist);
                }
            }
        });

        std::size_t effect_pts = 0;
        for (std::size_t point_idx = 0; point_idx < point_down_size; ++point_idx) {
            // valid points and corresponding plane parameters
            if (valid_point_flags[point_idx]) {
                valid_points_ptr->points[effect_pts] = scan_down_lidar_ptr->points[point_idx];
                valid_plane_params_ptr->points[effect_pts] = plane_params_ptr->points[point_idx];

                effect_pts += 1;
            }
        }

        // stop timer
        t2 = std::chrono::steady_clock::now();
        // if enough valid point
        if (effect_pts < (config_ptr_->use_doppler ? correspondence_thresh : 1)) {
            is_valid_iteration = false;

            logger_update_ptr->actual_iterations = i;
            logger_update_ptr->res[i] = 0.0;
            logger_update_ptr->eff_pts[i] = effect_pts;
            logger_update_ptr->update_time[i] = 0.0;
            logger_update_ptr->plane_search_time[i] = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()) * 1.0e-3;

            return effect_pts;
        }

        // residual and Jacobian matrix
        Eigen::VectorXd r = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(effect_pts));
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(static_cast<Eigen::Index>(effect_pts), dim_state);

        std::vector<std::size_t> valid_point_indices(effect_pts);
        std::for_each(valid_point_indices.begin(), valid_point_indices.end(), [n = 0](std::size_t& idx) mutable -> void { idx = n++; });
        std::for_each(std::execution::par,
                      valid_point_indices.begin(), valid_point_indices.end(),
                      [&](auto valid_point_idx) -> void {
            Eigen::Vector3d p_lp_l(valid_points_ptr->points[valid_point_idx].x,
                                   valid_points_ptr->points[valid_point_idx].y,
                                   valid_points_ptr->points[valid_point_idx].z);
            Eigen::Vector3d p_bp_b = config_ptr_->R_bl * p_lp_l + config_ptr_->p_bl_b;
            Eigen::Matrix3d p_bp_b_times = SO3::skewVec(p_bp_b);

            auto& plane_param = valid_plane_params_ptr->points[valid_point_idx];
            Eigen::Vector3d normal(plane_param.x, plane_param.y, plane_param.z);

            // Jacobian matrix
            // Jacobian of residual w.r.t. position p_wb_w
            Eigen::Vector3d dr_dp_wb_w = normal;
            // Jacobian of residual w.r.t. extrinsic position p_bl_b
            Eigen::Vector3d dr_dp_bl_b = R_wb.transpose() * normal;
            // Jacobian of residual w.r.t. rotation R_wb
            Eigen::Vector3d dr_dR_wb = p_bp_b_times * dr_dp_bl_b;

            H.block(valid_point_idx, 0, 1, 3) = dr_dR_wb.transpose();
            H.block(valid_point_idx, 6, 1, 3) = dr_dp_wb_w.transpose();

            // residual
            r(valid_point_idx) = static_cast<double>(-plane_param.intensity);
        });

        if (!is_valid_iteration) {
            continue;
        }

        // matrix dimension reduction via QR decomposition with orthogonal triangularization
        auto applyOrthogonalTriangularization =
                [](auto& A, auto& b, bool use_col_pivoting = true) -> void {
            // decayed scalar type
            using Scalar = typename std::decay_t<decltype(A)>::Scalar;

            // ensure A and b use same scalar type
            static_assert(std::is_same_v<Scalar, typename std::decay_t<decltype(b)>::Scalar>,
                          "A and b must have same scalar type.");

            // get matrix dimension
            const auto row_dim = static_cast<std::size_t>(A.rows());
            const auto col_dim = static_cast<std::size_t>(A.cols());

            // only reduce when A is overdetermined (more rows than columns)
            if (row_dim <= col_dim){
                return;
            }

            const std::size_t dim = col_dim;
            if (use_col_pivoting) {
                // perform QR decomposition with column pivoting: A * P = Q * R
                Eigen::ColPivHouseholderQR<Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>> qr(A);
                // get column permutation matrix
                const auto& P = qr.colsPermutation();
                // extract full upper triangular matrix R from QR decomposition
                const auto& R_full = qr.matrixQR().template triangularView<Eigen::Upper>().toDenseMatrix();
                // take top-left col_dim x col_dim block of R
                Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> R_n = R_full.topRows(dim).leftCols(dim);
                // compute y = Q^{\top} * b, then take top col_dim rows as reduced right-hand side
                Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> y = (qr.householderQ().adjoint() * b).topRows(dim);

                A = std::move(R_n * P.transpose());
                b = std::move(y);
            } else {
                // perform standard QR decomposition without pivoting: A = Q * R
                Eigen::HouseholderQR<Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>> qr(A);
                // extract full upper triangular matrix R
                const auto R_full = qr.matrixQR().template triangularView<Eigen::Upper>().toDenseMatrix();
                // take top-left col_dim x col_dim block of R
                Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> R_n = R_full.topRows(dim).leftCols(dim);
                // compute y = Q^{\top} * b, then take top col_dim rows
                Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> y = (qr.householderQ().adjoint() * b).topRows(dim);

                A = std::move(R_n);
                b = std::move(y);
            }
        };

        // observation matrix decomposition
        applyOrthogonalTriangularization(H, r);
        Eigen::MatrixXd R_p = R_point * Eigen::MatrixXd::Identity(static_cast<Eigen::Index>(H.rows()),
                                                                  static_cast<Eigen::Index>(H.rows()));

        // error state update
        Eigen::VectorXd delta_x_pre = state_ptr->subState(state_prop);

        // projection Jacobian
        Eigen::MatrixXd J = Eigen::MatrixXd::Identity(dim_state, dim_state);
        J.block(0, 0, 3, 3) = SO3::Jl(delta_x_pre.segment(0, 3).eval()).transpose();

        // Kalman gain
        Eigen::MatrixXd P_pre = J * P_prop * J.transpose();

        Eigen::MatrixXd PHT = P_pre * H.transpose();
        Eigen::MatrixXd S = (H * PHT) + R_p;
        Eigen::MatrixXd S_inv = S.inverse();

        Eigen::MatrixXd K = PHT * S_inv;
        Eigen::MatrixXd KH = K * H;

        // optimal updated error state dx of current iteration step
        Eigen::VectorXd delta_x_post = (K * r) +
                                       ((KH - Eigen::MatrixXd::Identity(dim_state, dim_state)) * J * delta_x_pre);

        // update state estimate
        state_ptr->addDelVec(delta_x_post);

        // convergence judge
        is_convergence = true;
        for (int j = 0; j < delta_x_post.rows(); ++j) {
            // check convergence
            if (std::fabs(delta_x_post[j]) > 1.0e-3) {
                is_convergence = false;

                break;
            }
        }

        if (is_convergence) {
            convergence_num += 1;
        }

        if (!convergence_num &&
            (i == (config_ptr_->iteration_num - 1))) {
            is_convergence = true;
        }

        // covariance update
        if ((convergence_num > 1) ||
            (i == config_ptr_->iteration_num)) {
            P_pre = (Eigen::MatrixXd::Identity(dim_state, dim_state) - KH) * P_pre;

            // projection Jacobian for covariance reset
            Eigen::MatrixXd L = Eigen::MatrixXd::Identity(dim_state, dim_state);
            L.block(0, 0, 3, 3) = SO3::Jl(delta_x_post.segment(0, 3).eval()).transpose();

            P = L * P_pre * L.transpose();
        } else {
            P = P_pre;
        }

        // stop timer
        t3 = std::chrono::steady_clock::now();
        logger_update_ptr->actual_iterations = i;
        logger_update_ptr->res[i] = r.norm();
        logger_update_ptr->eff_pts[i] = effect_pts;
        logger_update_ptr->update_time[i] = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count()) * 1.0e-3;
        logger_update_ptr->plane_search_time[i] = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()) * 1.0e-3;

        if ((convergence_num > 1) ||
            (i == config_ptr_->iteration_num)) {
            return effect_pts;
        }
    }
}

bool Updater::estimatePlane(Eigen::Vector4d& plane_param,
                            const PointVec1D& points_near,
                            const double threshold) const {
    // estimate plane parameter
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(static_cast<Eigen::Index>(config_ptr_->nearest_point_num), 3);
    Eigen::VectorXd b = -Eigen::VectorXd::Ones(static_cast<Eigen::Index>(config_ptr_->nearest_point_num));

    for (int j = 0; j < config_ptr_->nearest_point_num; ++j) {
        A(j, 0) = points_near[j].x;
        A(j, 1) = points_near[j].y;
        A(j, 2) = points_near[j].z;
    }

    Eigen::Vector3d norm_vec = A.colPivHouseholderQr().solve(b);

    double n = norm_vec.norm();
    plane_param(0) = norm_vec(0) / n;
    plane_param(1) = norm_vec(1) / n;
    plane_param(2) = norm_vec(2) / n;
    plane_param(3) = 1.0 / n;

    for (int j = 0; j < config_ptr_->nearest_point_num; ++j) {
        if (std::fabs(plane_param(0) * points_near[j].x +
                      plane_param(1) * points_near[j].y +
                      plane_param(2) * points_near[j].z +
                      plane_param(3)) > threshold) {
            return false;
        }
    }

    return true;
}

} // namespace fmcw_lio
