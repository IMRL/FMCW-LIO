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

#include <random>

#include <angles/angles.h>

#include "velocimeter/velocimeter.hpp"

// fmcw_lio
namespace fmcw_lio {

// Velocimeter
Velocimeter::Velocimeter(const std::shared_ptr<const Config>& config_ptr) : logger_velocimetry_ptr(std::make_shared<LoggerVelocimetry>()),
                                                                            config_ptr_(config_ptr),
                                                                            valid_point_num_thresh_{10},
                                                                            cond_ratio_{0.4},
                                                                            sigma_ratio_{10.0},
                                                                            vel_sigma_offset_few_(1.0, 3.0, 5.0),
                                                                            success_{},
                                                                            time_{} {
    // RANSAC iteration number
    ransac_iteration_num_ = std::ceil((std::log(1.0 - config_ptr_->success_prob)) /
                                      std::log(1.0 - std::pow(1.0 - config_ptr_->outlier_prob,
                                                              config_ptr_->ransac_point_num)));

    // set sample function
    sample_func_ = sampling_table_.at(config_ptr_->sampling_method);
}

bool Velocimeter::estimateVelocity(const PointCloudType::Ptr& scan_ptr,
                                   const Eigen::Vector3d& v_wl_l,
                                   const bool use_ransac) {
    // start timer
    std::chrono::steady_clock::time_point t1, t2;
    t1 = std::chrono::steady_clock::now();

    // velocity solution success flag
    success_ = false;

    // store points within FoV and set detection range
    std::vector<std::unordered_map<std::string, double>> valid_points;

    // preprocess LiDAR scan
    PointCloudType::Ptr scan_pre_ptr(new PointCloudType());
    if (config_ptr_->use_dynamic_point_removal &&
        config_ptr_->pre_remove) {
        PointCloudType::Ptr scan_dynamic_pre_ptr(new PointCloudType());
        removeDynamicPoint(scan_ptr,
                           scan_pre_ptr,
                           scan_dynamic_pre_ptr,
                           v_wl_l);
    } else {
        scan_pre_ptr = scan_ptr;
    }

    auto scan_point_size = scan_pre_ptr->points.size();
    std::vector<std::unordered_map<std::string, double>> all_points;
    all_points.resize(scan_point_size);

    std::vector<std::size_t> point_indices(scan_point_size);
    std::for_each(point_indices.begin(), point_indices.end(), [n = 0](std::size_t& idx) mutable -> void { idx = n++; });
    std::for_each(std::execution::par,
                  point_indices.begin(), point_indices.end(),
                  [&](auto point_idx) -> void {
        // select valid points
        auto& point = scan_pre_ptr->points[point_idx];
        Eigen::Vector3d p_lp_l = Eigen::Vector3d(point.x, point.y, point.z);
        double dist = p_lp_l.norm();
        double inv_dist = 1.0 / dist;

        double azimuth = std::atan2(point.y, point.x);
        double elevation = std::atan2(point.z, std::sqrt(point.x * point.x + point.y * point.y));

        // filter points based on distance and angle
        if ((dist > config_ptr_->min_point_dist) &&
            (dist < config_ptr_->max_point_dist) &&
            (std::fabs(azimuth) < angles::from_degrees(config_ptr_->azimuth_thresh_deg)) &&
            (std::fabs(elevation) < angles::from_degrees(config_ptr_->elevation_thresh_deg)) &&
            (p_lp_l.z() > config_ptr_->min_point_z) &&
            (p_lp_l.z() < config_ptr_->max_point_z)) {
            // key: x, y, z, azimuth, elevation, x_normalized, y_normalized, z_normalized, doppler_velocity
            std::unordered_map<std::string, double> point_info;

            point_info["x"] = point.x;
            point_info["y"] = point.y;
            point_info["z"] = point.z;
            point_info["azimuth"] = azimuth;
            point_info["elevation"] = elevation;
            point_info["x_normalized"] = point.x * inv_dist;
            point_info["y_normalized"] = point.y * inv_dist;
            point_info["z_normalized"] = point.z * inv_dist;
            point_info["doppler_velocity"] = -point.intensity;

            all_points[point_idx] = point_info;
        }
    });

    for (const auto& item : all_points) {
        if (!item.empty()) {
            valid_points.push_back(item);
        }
    }

    if (valid_points.size() < 3) {
        time_ = scan_ptr->points.back().curvature * 1.0e-3;
        v_wl_l_.setZero();

        return success_;
    }

    Eigen::Index idx = 0;
    Eigen::MatrixXd lsq_data(valid_points.size(), 4);
    for (const auto& valid_point : valid_points) {
        lsq_data.row(idx++) = Eigen::Vector4d(valid_point.at("x_normalized"),
                                              valid_point.at("y_normalized"),
                                              valid_point.at("z_normalized"),
                                              valid_point.at("doppler_velocity"));
    }

    // solve optimization problem
    v_wl_l_ = v_wl_l;
    if (use_ransac &&
        (valid_points.size() > valid_point_num_thresh_)) {
        // enough valid points and use RANSAC to estimate velocity in LiDAR frame
        // get indices of sampling points
        if (v_wl_l.norm() > config_ptr_->zero_velocity_thresh) {
            sampled_indices_ = sample_func_(valid_points, v_wl_l);
        } else {
            sampled_indices_ = sampleUniform(valid_points, v_wl_l);
        }

        std::vector<std::size_t> inlier_idx_best;
        success_ = solveRANSAC(lsq_data, 
                               v_wl_l_, R_v_,
                               inlier_idx_best);
    } else {
        // few valid points to use LSQ to estimate velocity in LiDAR frame
        bool few_points_flag = (valid_points.size() <= valid_point_num_thresh_);
        success_ = solveLSQ(lsq_data,
                            v_wl_l_, R_v_,
                            true, few_points_flag);
    }

    // zero velocity detection
    if (success_ &&
        (v_wl_l_.norm() <= config_ptr_->zero_velocity_thresh)) {
        v_wl_l_.setZero();
        R_v_.setIdentity();
        R_v_.diagonal() = config_ptr_->zero_velocity_sigma.array().square();
    }

    // set velocity estimation time
    time_ = scan_ptr->points.back().curvature * 1.0e-3;

    // stop timer
    t2 = std::chrono::steady_clock::now();
    logger_velocimetry_ptr->success = success_;
    logger_velocimetry_ptr->vel_est_time = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()) * 1.0e-3;

    return success_;
}

void Velocimeter::removeDynamicPoint(const PointCloudType::Ptr& scan_ptr,
                                     const PointCloudType::Ptr& scan_static_ptr,
                                     const PointCloudType::Ptr& scan_dynamic_ptr,
                                     const Eigen::Vector3d& v_wl_l) {
    // start timer
    std::chrono::steady_clock::time_point t1, t2;
    t1 = std::chrono::steady_clock::now();

    std::size_t point_size = scan_ptr->points.size();
    scan_static_ptr->points.reserve(point_size);
    scan_dynamic_ptr->points.reserve(point_size);
    for (std::size_t i = 0; i < point_size; ++i) {
        const auto& point = scan_ptr->points[i];
        Eigen::Vector3d point_normalized = Eigen::Vector3d(point.x, point.y, point.z).normalized();
        Eigen::Vector3d v_wl_l_normalized = v_wl_l.normalized();

        // predicted Doppler velocity
        double pred_doppler = -point_normalized.dot(v_wl_l);
        double doppler_error = std::abs(pred_doppler - point.intensity);

        double v_wl_l_norm = v_wl_l.norm();

        // use cosine weight (do not scale threshold for near-zero velocity case)
        double cos_weight_or_not = ((v_wl_l_norm > config_ptr_->zero_velocity_thresh) ?
                                    std::abs(point_normalized.dot(v_wl_l_normalized)) :
                                    1.0);
        if (doppler_error < (config_ptr_->remove_dynamic_threshold * cos_weight_or_not)) {
            scan_static_ptr->points.push_back(point);
        } else {
            scan_dynamic_ptr->points.push_back(point);
        }
    }

    // stop timer
    t2 = std::chrono::steady_clock::now();
    logger_velocimetry_ptr->dynamic_point_removal_time = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()) * 1.0e-3;
    logger_velocimetry_ptr->static_point_num = scan_static_ptr->points.size();
    logger_velocimetry_ptr->dynamic_point_num = scan_dynamic_ptr->points.size();
}

std::vector<std::size_t> Velocimeter::sampleUniform(const std::vector<std::unordered_map<std::string, double>>& valid_points,
                                                    const Eigen::Vector3d& v_wl_l) const {
    // divide points into azimuth and elevation regions
    std::map<std::pair<std::size_t, std::size_t>, std::vector<std::size_t>> region_points_map;
    for (std::size_t i = 0; i < valid_points.size(); ++i) {
        const std::unordered_map<std::string, double>& valid_point = valid_points[i];
        double azimuth = valid_point.at("azimuth");
        double elevation = valid_point.at("elevation");

        // interval each point belongs
        auto azimuth_index = std::min(static_cast<std::size_t>((azimuth + M_PI_2) / M_PI *
                                      static_cast<double>(config_ptr_->azimuth_interval_num)),
                                      config_ptr_->azimuth_interval_num - 1);
        auto elevation_index = std::min(static_cast<std::size_t>((elevation + M_PI_2) / M_PI *
                                        static_cast<double>(config_ptr_->elevation_interval_num)),
                                        config_ptr_->elevation_interval_num - 1);

        region_points_map[std::make_pair(azimuth_index, elevation_index)].push_back(i);
    }

    // create a list of region indices containing azimuth and elevation region indices
    std::vector<std::pair<std::size_t, std::size_t>> region_indices;
    for (std::size_t azimuth_index = 0; azimuth_index < config_ptr_->azimuth_interval_num; ++azimuth_index) {
        for (std::size_t elevation_index = 0; elevation_index < config_ptr_->elevation_interval_num; ++elevation_index) {
            region_indices.emplace_back(azimuth_index, elevation_index);
        }
    }

    // random number generator
    std::random_device rd;
    std::default_random_engine gen(rd());
    // erase point check via point number
    bool erase_point = true;
    if (valid_points.size() < (ransac_iteration_num_ * config_ptr_->ransac_point_num)) {
        // not enough valid points for sampling
        erase_point = false;
    }

    // randomly select points from different regions until enough points are selected
    std::vector<std::size_t> selected_indices;
    while (selected_indices.size() < (ransac_iteration_num_ * config_ptr_->ransac_point_num)) {
        if (region_indices.empty()) {
            for (std::size_t azimuth_index = 0; azimuth_index < config_ptr_->azimuth_interval_num; ++azimuth_index) {
                for (std::size_t elevation_index = 0; elevation_index < config_ptr_->elevation_interval_num; ++elevation_index) {
                    region_indices.emplace_back(azimuth_index, elevation_index);
                }
            }
        }

        // select a random region
        std::uniform_int_distribution<std::size_t> uniform_distribution_region(0, region_indices.size() - 1);
        auto region_index = static_cast<long>(uniform_distribution_region(gen));
        const auto& azimuth_elevation_indices = region_indices[region_index];

        // select a random point from selected region
        const auto& region_points_map_iter = region_points_map.find(azimuth_elevation_indices);
        if ((region_points_map_iter != region_points_map.end()) &&
            !region_points_map_iter->second.empty()) {
            // find all indices of points in selected region
            const auto& point_indices = region_points_map_iter->second;
            std::uniform_int_distribution<std::size_t> uniform_distribution_point(0, point_indices.size() - 1);
            auto random_point_index = static_cast<long>(uniform_distribution_point(gen));
            selected_indices.push_back(point_indices[random_point_index]);

            // remove selected point index
            if (erase_point) {
                region_points_map[azimuth_elevation_indices].erase(region_points_map[azimuth_elevation_indices].begin() + random_point_index);
            }
        }

        // remove selected region
        region_indices.erase(region_indices.begin() + region_index);
    }

    return selected_indices;
}

std::vector<std::size_t> Velocimeter::sampleCosWeight(const std::vector<std::unordered_map<std::string, double>>& valid_points,
                                                      const Eigen::Vector3d& v_wl_l) const {
    // count weight (absolute value of cos_theta) and index of each point
    std::vector<std::pair<double, std::size_t>> weights_indices;
    weights_indices.resize(valid_points.size());

    std::vector<std::size_t> valid_point_indices(valid_points.size());
    std::for_each(valid_point_indices.begin(), valid_point_indices.end(), [n = 0](std::size_t& idx) mutable -> void { idx = n++; });
    std::for_each(std::execution::par,
                  valid_point_indices.begin(), valid_point_indices.end(),
                  [&](auto point_idx) -> void {
        // use cosine between point direction and velocity vector
        Eigen::Vector3d p_lp_l(valid_points[point_idx].at("x"),
                               valid_points[point_idx].at("y"),
                               valid_points[point_idx].at("z"));

        weights_indices[point_idx].first = weightCosine(p_lp_l, v_wl_l);
        weights_indices[point_idx].second = point_idx;
    });

    // cumulative weight
    std::vector<double> cumulative_weights(weights_indices.size());
    std::inclusive_scan(weights_indices.begin(), weights_indices.end(),
                        cumulative_weights.begin(),
                        [](double sum, const std::pair<double, std::size_t>& p) -> double { return (sum + p.first); },
                        0.0);

    // random number generator
    std::random_device rd;
    std::default_random_engine gen(rd());
    // erase point check via point number
    bool erase_point = true;
    if (valid_points.size() < (ransac_iteration_num_ * config_ptr_->ransac_point_num)) {
        // not enough valid points for sampling
        erase_point = false;
    }

    std::vector<std::size_t> selected_indices;
    double total_weight = cumulative_weights.back();
    if (total_weight == 0.0) {
        return selected_indices;
    }

    // randomly select points based on weights until enough points are selected
    while (selected_indices.size() < (ransac_iteration_num_ * config_ptr_->ransac_point_num)) {
        // sum of remaining weight
        std::vector<double> cumulative_remaining_weights(weights_indices.size());
        std::inclusive_scan(weights_indices.begin(), weights_indices.end(),
                            cumulative_remaining_weights.begin(),
                            [](double sum, const std::pair<double, std::size_t>& p) -> double { return (sum + p.first); },
                            0.0);
        double total_weight = cumulative_remaining_weights.back();

        std::uniform_real_distribution<double> uniform_distribution_weight(0.0, total_weight);
        auto sampled_value = uniform_distribution_weight(gen);
        auto it = std::lower_bound(cumulative_remaining_weights.begin(), cumulative_remaining_weights.end(),
                                   sampled_value);
        auto weight_index = std::distance(cumulative_remaining_weights.begin(), it);

        // store selected indices
        selected_indices.push_back(weights_indices[weight_index].second);

        // remove selected point
        if (erase_point) {
            weights_indices.erase(weights_indices.begin() + weight_index);
        }
    }

    return selected_indices;
}


bool Velocimeter::weightMatrix(Eigen::VectorXd& weight, 
                               const Eigen::MatrixXd& lsq_data, const Eigen::Vector3d& v_wl_l) const {
    // flag for using weight
    bool use_weight = false;
    weight = Eigen::VectorXd::Ones(lsq_data.rows());

    // cosine weight
    if (config_ptr_->use_cos_weight &&
        (v_wl_l.norm() > config_ptr_->zero_velocity_thresh)) {
        std::vector<std::size_t> lsq_data_indices(lsq_data.rows());
        std::for_each(lsq_data_indices.begin(), lsq_data_indices.end(), [n = 0](std::size_t& idx) mutable -> void { idx = n++; });
        std::for_each(std::execution::par,
                      lsq_data_indices.begin(), lsq_data_indices.end(),
                      [&](auto lsq_data_idx) -> void {
            auto row_idx = static_cast<Eigen::Index>(lsq_data_idx);
            weight(row_idx) *= weightCosine(lsq_data.row(row_idx).head<3>(), v_wl_l);
        });

        use_weight = true;
    }

    // robust kernel weight
    if (config_ptr_->robust_kernel == RobustKernelType::Cauchy) {
        std::vector<std::size_t> lsq_data_indices(lsq_data.rows());
        std::for_each(lsq_data_indices.begin(), lsq_data_indices.end(), [n = 0](std::size_t& idx) mutable -> void { idx = n++; });
        std::for_each(std::execution::par,
                      lsq_data_indices.begin(), lsq_data_indices.end(),
                      [&](auto lsq_data_idx) -> void {
            auto row_idx = static_cast<Eigen::Index>(lsq_data_idx);
            weight(row_idx) *= weightRobustKernelCauchy(lsq_data.row(row_idx), v_wl_l);
        });

        use_weight = true;
    }

    return use_weight;
}

double Velocimeter::weightCosine(const Eigen::Vector3d& p_lp_l, const Eigen::Vector3d& v_wl_l) {
    // cosine between point direction and velocity vector
    auto d_lp_l = p_lp_l.normalized();
    auto v_wl_l_normalized = v_wl_l.normalized();
    double cos_theta = d_lp_l.dot(v_wl_l_normalized);

    return std::abs(cos_theta);
}

double Velocimeter::weightRobustKernelCauchy(const Eigen::Vector4d& lsq_data_row, const Eigen::Vector3d& v_wl_l) {
    // Cauchy scale square
    double cauchy_scale_square = 100.0;
    double residual = lsq_data_row.head<3>().dot(v_wl_l) - lsq_data_row(3);
    double residual_square = residual * residual;

    // Cauchy robust kernel weight
    return (cauchy_scale_square / (cauchy_scale_square + residual_square));
}

bool Velocimeter::solveLSQ(const Eigen::MatrixXd& lsq_data,
                           Eigen::Vector3d& v_wl_l, Eigen::Matrix3d& R_v,
                           bool estimate_cov, bool few_valid_points) {
    // construct least square data matrix
    Eigen::MatrixXd D(lsq_data.rows(), 3);
    D.col(0) = lsq_data.col(0);
    D.col(1) = lsq_data.col(1);
    D.col(2) = lsq_data.col(2);

    Eigen::VectorXd V = lsq_data.col(3);

    // weight matrix
    Eigen::VectorXd weight = Eigen::VectorXd::Ones(lsq_data.rows());
    bool use_weight = weightMatrix(weight, 
                                   lsq_data, v_wl_l);
    if (use_weight) {
        // apply weight to LSQ data
        Eigen::VectorXd sqrt_weight = weight.array().sqrt();
        D = D.array().colwise() * sqrt_weight.array();
        V = V.cwiseProduct(sqrt_weight);
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
    Eigen::MatrixXd D_ori = D;
    Eigen::VectorXd V_ori = V;

    applyOrthogonalTriangularization(D, V);
    Eigen::MatrixXd DTD = D.transpose() * D;
    Eigen::VectorXd DTV = D.transpose() * V;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(DTD);
    double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);

    double cond_ratio = few_valid_points ? cond_ratio_ : 1.0;
    if (std::fabs(cond) < (cond_ratio * config_ptr_->max_cond_num)) {
        // solve LSQ
        v_wl_l = (DTD).ldlt().solve(DTV);

        if (estimate_cov) {
            Eigen::VectorXd e = D_ori * v_wl_l - V_ori;
            R_v = (e.transpose() * e).x() * (DTD).inverse() / (D_ori.rows() - 3);
            Eigen::Vector3d sigma_v_wl_l = Eigen::Vector3d(R_v(0, 0), R_v(1, 1), R_v(2, 2));

            if ((sigma_v_wl_l.x() >= 0.0) &&
                (sigma_v_wl_l.y() >= 0.0) &&
                (sigma_v_wl_l.z() >= 0.0)) {
                // sigma value
                sigma_v_wl_l = sigma_v_wl_l.array().sqrt();

                double sigma_ratio = (few_valid_points ? sigma_ratio_ : 1.0);
                if ((sigma_v_wl_l.x() < (sigma_ratio * config_ptr_->max_sigma.x())) &&
                    (sigma_v_wl_l.y() < (sigma_ratio * config_ptr_->max_sigma.y())) &&
                    (sigma_v_wl_l.z() < (sigma_ratio * config_ptr_->max_sigma.z()))) {
                    // sigma offset
                    Eigen::Vector3d vel_sigma_offset = few_valid_points ? vel_sigma_offset_few_ : config_ptr_->vel_sigma_offset;
                    Eigen::Vector3d offset = vel_sigma_offset.array().square();

                    // covariance offset
                    R_v += offset.asDiagonal();

                    return true;
                }
            }
        } else {
            return true;
        }
    }

    return false;
}

bool Velocimeter::solveRANSAC(const Eigen::MatrixXd& lsq_data,
                              Eigen::Vector3d& v_wl_l, Eigen::Matrix3d& R_v,
                              std::vector<std::size_t>& inlier_idx_best) {
    // construct least square data matrix
    Eigen::MatrixXd D(lsq_data.rows(), 3);
    D.col(0) = lsq_data.col(0);
    D.col(1) = lsq_data.col(1);
    D.col(2) = lsq_data.col(2);

    Eigen::VectorXd V = lsq_data.col(3);

    // parallel RANSAC iteration
    std::mutex best_mutex;
    std::atomic<bool> early_stop(false);
    std::vector<std::size_t> ransac_iters(ransac_iteration_num_);
    std::for_each(ransac_iters.begin(), ransac_iters.end(), [n = 0](std::size_t& idx) mutable -> void { idx = n++; });
    std::for_each(std::execution::par,
                  ransac_iters.begin(), ransac_iters.end(),
                  [&](std::size_t k) -> void {
        // early stop check
        if (early_stop.load()) {
            return;
        }

        Eigen::MatrixXd lsq_data_iter(config_ptr_->ransac_point_num, 4);
        for (std::size_t i = 0; i < config_ptr_->ransac_point_num; ++i) {
            lsq_data_iter.row(static_cast<Eigen::Index>(i)) = lsq_data.row(static_cast<Eigen::Index>(sampled_indices_[k * config_ptr_->ransac_point_num + i]));
        }

        Eigen::Vector3d local_v_wl_l;
        Eigen::Matrix3d local_R_v;
        std::vector<std::size_t> local_inlier_idx;
        bool is_success = solveLSQ(lsq_data_iter,
                                   local_v_wl_l, local_R_v,
                                   false);
        if (is_success) {
            const Eigen::VectorXd err = (V - D * local_v_wl_l).array().abs();

            for (std::size_t j = 0; j < err.size(); ++j) {
                if (err(static_cast<Eigen::Index>(j)) < config_ptr_->inlier_thresh) {
                    local_inlier_idx.push_back(j);
                }
            }

            if (static_cast<double>(local_inlier_idx.size()) > ((1.0 - config_ptr_->outlier_prob) * static_cast<double>(lsq_data.rows()))) {
                // early iteration termination
                early_stop.store(true);
            }

            // update best inliers if current inliers better
            std::lock_guard<std::mutex> lock(best_mutex);
            if (local_inlier_idx.size() > inlier_idx_best.size()) {
                inlier_idx_best = std::move(local_inlier_idx);
                v_wl_l = local_v_wl_l;
                R_v = local_R_v;
            }
        }
    });

    // refine model with best inliers
    if (!inlier_idx_best.empty()) {
        Eigen::MatrixXd lsq_data_inlier(inlier_idx_best.size(), 4);
        for (std::size_t i = 0; i < inlier_idx_best.size(); ++i) {
            lsq_data_inlier.row(static_cast<Eigen::Index>(i)) = lsq_data.row(static_cast<Eigen::Index>(inlier_idx_best.at(i)));
        }

        return solveLSQ(lsq_data_inlier,
                        v_wl_l, R_v,
                        true);
    }

    return false;
}

} // namespace fmcw_lio
