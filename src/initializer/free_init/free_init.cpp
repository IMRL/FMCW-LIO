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

#include "initializer/free_init/free_init.hpp"

// fmcw_lio
namespace fmcw_lio {

// FreeInit
FreeInit::FreeInit(const std::shared_ptr<const Config>& config_ptr) : config_ptr_(config_ptr),
                                                                      div_ptr_(std::make_shared<DIV>(config_ptr_)),
                                                                      optimizer_bag_ptr_(std::make_shared<OptimizerBaG>(config_ptr_)),
                                                                      max_imu_num_(config_ptr_->max_imu_num),
                                                                      pose_ptr_vec_ptr_(std::make_shared<std::vector<std::shared_ptr<StatePoseQuat>>>()),
                                                                      state_ptr_(std::make_shared<StateSystem>(config_ptr_)) {
    // initialize point cloud
    map_ptr_.reset(new PointCloudType());

    uniform_filter_scan_.setRadiusSearch(config_ptr_->voxel_filter_size_scan);
}

void FreeInit::bootstrapVelocity(const std::shared_ptr<MeasPackLI>& meas_ptr) {
    // check data for bootstrap
    if (!config_ptr_->use_bootstrap_velocity || 
        meas_ptr->scan_ptr->empty() || 
        meas_ptr->imu_msg_ptr_queue.empty()) {

        return;
    }

    // divide LiDAR points into different voxels
    Eigen::MatrixXd lsq_data(meas_ptr->scan_ptr->points.size(), 4);
    std::vector<std::array<int, 3>> voxel_indices;
    voxel_indices.reserve(meas_ptr->scan_ptr->points.size());
    std::map<std::array<int, 3>, std::size_t> voxel_counts;
    for (Eigen::Index i = 0; i < meas_ptr->scan_ptr->points.size(); ++i) {
        const auto& point = meas_ptr->scan_ptr->points[i];
        const Eigen::Vector3d p_lp_l(point.x, point.y, point.z);
        const Eigen::Vector3d d_lp_l = p_lp_l.normalized();

        lsq_data.row(i) << -d_lp_l.x(),
                           -d_lp_l.y(),
                           -d_lp_l.z(),
                           point.intensity;

        // compute voxel index
        const std::array<int, 3> voxel_index = {static_cast<int>(std::floor(point.x / config_ptr_->voxel_size_bootstrap)),
                                                static_cast<int>(std::floor(point.y / config_ptr_->voxel_size_bootstrap)),
                                                static_cast<int>(std::floor(point.z / config_ptr_->voxel_size_bootstrap))};
        voxel_indices.push_back(voxel_index);

        ++voxel_counts[voxel_index];
    }

    // set base weight according to point number in each voxel
    Eigen::VectorXd base_weight = Eigen::VectorXd::Ones(lsq_data.rows());
    for (Eigen::Index i = 0; i < lsq_data.rows(); ++i) {
        const std::array<int, 3>& voxel_index = voxel_indices[i];
        base_weight(i) = 1.0 / static_cast<double>(voxel_counts[voxel_index]);
    }

    // iterated least square to estimate body velocity
    Eigen::Vector3d v_wl_l = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R_v = Eigen::Matrix3d::Zero();
    Eigen::VectorXd weight = base_weight;
    bool success = false;
    for (std::size_t iteration = 0; iteration < config_ptr_->iteration_num_bootstrap; ++iteration) {
        // calculate weight matrix
        if (iteration != 0) {
            for (Eigen::Index i = 0; i < lsq_data.rows(); ++i) {
                const double residual = std::abs(lsq_data.row(i).head<3>().dot(v_wl_l) - lsq_data(i, 3));
                const double weight_i = residual > config_ptr_->detect_dynamic_threshold_bootstrap ? config_ptr_->detect_dynamic_threshold_bootstrap / residual : 1.0;
                weight(i) = base_weight(i) * weight_i;
            }
        }

        // solve least square
        success = solveLSQ(lsq_data,
                           weight,
                           v_wl_l, R_v);

        if (!success) {
            break;
        }

        // solve least square with inlier points only for the last iteration
        std::map<std::array<int, 3>, std::size_t> inlier_voxel_counts;
        if (iteration == config_ptr_->iteration_num_bootstrap - 1) {
            std::vector<Eigen::Index> inlier_indices;

            for (Eigen::Index i = 0; i < lsq_data.rows(); ++i) {
                const double residual = std::abs(lsq_data.row(i).head<3>().dot(v_wl_l) - lsq_data(i, 3));

                if (residual < config_ptr_->detect_dynamic_threshold_bootstrap) {
                    inlier_indices.push_back(i);

                    ++inlier_voxel_counts[voxel_indices[i]];
                }
            }

            if (inlier_indices.size() < 4) {
                success = false;

                break;
            }

            Eigen::MatrixXd inlier_lsq_data(inlier_indices.size(), 4);
            Eigen::VectorXd inlier_weight(inlier_indices.size());
            for (Eigen::Index i = 0; i < inlier_indices.size(); ++i) {
                inlier_lsq_data.row(i) = lsq_data.row(inlier_indices[i]);
                inlier_weight(i) = 1.0 / static_cast<double>(inlier_voxel_counts[voxel_indices[inlier_indices[i]]]);
            }

            success = solveLSQ(inlier_lsq_data,
                               inlier_weight,
                               v_wl_l, R_v);
        }
    }

    // interpolate inertial measurement at scan end
    if (success) {
        Eigen::Vector3d omg_wb_b_interp_unbiased;
        if (meas_ptr->imu_msg_ptr_queue.size() >= 2) {
            auto imu_head_ptr = meas_ptr->imu_msg_ptr_queue.end() - 2;
            auto imu_tail_ptr = meas_ptr->imu_msg_ptr_queue.end() - 1;

            double dt_head_tail = imu_tail_ptr->get()->header.stamp.toSec() - imu_head_ptr->get()->header.stamp.toSec();
            double dt_head_scan = meas_ptr->scan_end_time - imu_head_ptr->get()->header.stamp.toSec();
            double interp_weight = dt_head_scan / dt_head_tail;

            Eigen::Vector3d omg_wb_b_head = Eigen::Vector3d(imu_head_ptr->get()->angular_velocity.x,
                                                            imu_head_ptr->get()->angular_velocity.y,
                                                            imu_head_ptr->get()->angular_velocity.z);
            Eigen::Vector3d omg_wb_b_tail = Eigen::Vector3d(imu_tail_ptr->get()->angular_velocity.x,
                                                            imu_tail_ptr->get()->angular_velocity.y,
                                                            imu_tail_ptr->get()->angular_velocity.z);
            Eigen::Vector3d omg_wb_b_interp = (((1.0 - interp_weight) * omg_wb_b_head) +
                                               (interp_weight * omg_wb_b_tail)).eval();

            omg_wb_b_interp_unbiased = omg_wb_b_interp - config_ptr_->gyroscope_bias_prior;
        } else {
            auto imu_tail_ptr = meas_ptr->imu_msg_ptr_queue.end() - 1;

            Eigen::Vector3d omg_wb_b_tail = Eigen::Vector3d(imu_tail_ptr->get()->angular_velocity.x,
                                                            imu_tail_ptr->get()->angular_velocity.y,
                                                            imu_tail_ptr->get()->angular_velocity.z);

            omg_wb_b_interp_unbiased = omg_wb_b_tail - config_ptr_->gyroscope_bias_prior;
        }

        // initialize DIV with estimated body velocity and interpolated angular rate
        Eigen::Vector3d v_wb_b_scan_end = (config_ptr_->R_bl * v_wl_l) - omg_wb_b_interp_unbiased.cross(config_ptr_->p_bl_b);
        div_ptr_->initializeDIV(meas_ptr->scan_end_time,
                                omg_wb_b_interp_unbiased, v_wb_b_scan_end);
    }
}

bool FreeInit::solveLSQ(const Eigen::MatrixXd& lsq_data, const Eigen::VectorXd& weight, 
                         Eigen::Vector3d& v_wl_l, Eigen::Matrix3d& R_v) {
    // construct least square data matrix
    Eigen::MatrixXd D(lsq_data.rows(), 3);
    D.col(0) = lsq_data.col(0);
    D.col(1) = lsq_data.col(1);
    D.col(2) = lsq_data.col(2);

    Eigen::VectorXd V = lsq_data.col(3);
    // apply weight to LSQ data
    Eigen::VectorXd sqrt_weight = weight.array().sqrt();
    D = D.array().colwise() * sqrt_weight.array();
    V = V.cwiseProduct(sqrt_weight);

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

    if (std::fabs(cond) < (config_ptr_->max_cond_num_bootstrap)) {
        // solve LSQ
        v_wl_l = (DTD).ldlt().solve(DTV);

        Eigen::VectorXd e = D_ori * v_wl_l - V_ori;
        R_v = (e.transpose() * e).x() * (DTD).inverse() / (D_ori.rows() - 3);
        Eigen::Vector3d sigma_v_wl_l = Eigen::Vector3d(R_v(0, 0), R_v(1, 1), R_v(2, 2));

        if ((sigma_v_wl_l.x() >= 0.0) &&
            (sigma_v_wl_l.y() >= 0.0) &&
            (sigma_v_wl_l.z() >= 0.0)) {
            return true;
        }
    }

    return false;
}

std::size_t FreeInit::initializeStateCovMap(const std::shared_ptr<MeasPackLI>& meas_ptr) {
    // sort sensor measurement
    const auto meas_div_ptr_vec_ptr = sortMeas(meas_ptr);

    // DIV evolves
    const std::size_t imu_num = div_ptr_->evolveDIV(meas_div_ptr_vec_ptr);

    if (imu_num > max_imu_num_) {
        // estimate accelerometer bias and gravity with stored state estimate and IMU measurement
        estimateAccelerometerBiasAndGravity();

        // align z axis with estimated gravity direction
        auto gravity = optimizer_bag_ptr_->getGravity();
        auto x_ref = (div_ptr_->getStateType() == StateDIV::StateType::GyroBias) ?
                     Eigen::Vector3d::UnitX() :
                     div_ptr_->getBodyVelocityIntegrationStart();
        auto axis_xyz = spanFrame(-gravity,
                                  x_ref);

        Eigen::Matrix3d R_align_mat;
        R_align_mat.row(0) = axis_xyz[0].transpose();
        R_align_mat.row(1) = axis_xyz[1].transpose();
        R_align_mat.row(2) = axis_xyz[2].transpose();

        // set initial state and covariance
        setStateAndCov(R_align_mat);

        // transform pose and map point to new world frame
        transformPosesAndMapPoints(R_align_mat);
    } else {
        const auto R_p_wb = div_ptr_->getPose();
        const auto pose = std::make_shared<StatePoseQuat>(meas_ptr->scan_end_time,
                                                          Eigen::Quaterniond(R_p_wb.first),
                                                          R_p_wb.second);
        pose_ptr_vec_ptr_->push_back(pose);
    }

    return imu_num;
}

std::array<Eigen::Vector3d, 3> FreeInit::spanFrame(const Eigen::Vector3d& axis_z,
                                                   const Eigen::Vector3d& axis_x_ref) {
    // normalized main direction
    const double nz = axis_z.norm();
    const double nr = axis_x_ref.norm();

    const Eigen::Vector3d z = (nz > 1.0e-9) ? (axis_z / nz).eval() : Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d r = (nr > 1.0e-9) ? (axis_x_ref / nr).eval() : Eigen::Vector3d::UnitX();
    const double cos_theta = z.dot(r);

    // nearly collinear case
    Eigen::Vector3d r_xyz = r;
    if (std::fabs(std::fabs(cos_theta) - 1.0) < 1.0e-6) {
        if ((std::abs(z.x()) <= std::abs(z.y())) &&
            (std::abs(z.x()) <= std::abs(z.z()))) {
            r_xyz = Eigen::Vector3d::UnitX();
        } else if (std::abs(z.y()) <= std::abs(z.z())) {
            r_xyz = Eigen::Vector3d::UnitY();
        } else {
            r_xyz = Eigen::Vector3d::UnitZ();
        }
    }

    // span a right-handed frame {x, y, z}
    const Eigen::Vector3d y = z.cross(r_xyz).normalized();
    const Eigen::Vector3d x = y.cross(z);

    return {x, y, z};
}

std::shared_ptr<std::vector<std::shared_ptr<MeasDIV>>> FreeInit::sortMeas(const std::shared_ptr<MeasPackLI>& meas_ptr) {
    // downsample LiDAR scan
    PointCloudType::Ptr scan_down_ptr;
    scan_down_ptr.reset(new PointCloudType());
    uniform_filter_scan_.setInputCloud(meas_ptr->scan_ptr);
    uniform_filter_scan_.filter(*scan_down_ptr);
    if (scan_down_ptr->points.size() < config_ptr_->point_num_thresh) {
        *scan_down_ptr = *meas_ptr->scan_ptr;
    }

    // push IMU measurement and LiDAR point
    const std::size_t imu_num = meas_ptr->imu_msg_ptr_queue.size();
    const std::size_t point_num = scan_down_ptr->points.size();

    auto meas_div_vec_ptr = std::make_shared<std::vector<std::shared_ptr<MeasDIV>>>();
    meas_div_vec_ptr->resize(imu_num + point_num - 1);

    // LiDAR point
    std::vector<std::size_t> point_indices(point_num);
    std::for_each(point_indices.begin(), point_indices.end(), [n = 0](std::size_t& idx) mutable -> void { idx = n++; });
    std::for_each(std::execution::par,
                  point_indices.begin(), point_indices.end(),
                  [&](const auto point_idx) -> void {
        // 4D Doppler LiDAR point measurement
        const auto& point = scan_down_ptr->points[point_idx];

        std::shared_ptr<MeasDIV> meas_div_ptr = std::make_shared<MeasDIV>();
        meas_div_ptr->meas_type = MeasDIV::MeasType::DopplerVelocity;
        meas_div_ptr->timestamp = (point.curvature * 1.0e-3) + meas_ptr->scan_beg_time;
        meas_div_ptr->point = point;

        meas_div_vec_ptr->at(point_idx) = meas_div_ptr;
    });

    // IMU measurement
    std::vector<std::size_t> imu_indices(imu_num);
    std::for_each(imu_indices.begin(), imu_indices.end(), [n = 0](std::size_t& idx) mutable -> void { idx = n++; });
    std::for_each(std::execution::par,
                  imu_indices.begin(), imu_indices.end(),
                  [&](const auto imu_idx) -> void {
        // angular rate and specific force measurement
        // the last IMU data
        if (imu_idx == (imu_num - 1)) {
            return;
        }

        std::shared_ptr<MeasDIV> meas_div_ptr = std::make_shared<MeasDIV>();
        meas_div_ptr->meas_type = MeasDIV::MeasType::AngularRate;
        meas_div_ptr->timestamp = meas_ptr->imu_msg_ptr_queue[imu_idx]->header.stamp.toSec();
        meas_div_ptr->angular_rate = Eigen::Vector3d(meas_ptr->imu_msg_ptr_queue[imu_idx]->angular_velocity.x,
                                                     meas_ptr->imu_msg_ptr_queue[imu_idx]->angular_velocity.y,
                                                     meas_ptr->imu_msg_ptr_queue[imu_idx]->angular_velocity.z);
        meas_div_ptr->specific_force = Eigen::Vector3d(meas_ptr->imu_msg_ptr_queue[imu_idx]->linear_acceleration.x,
                                                       meas_ptr->imu_msg_ptr_queue[imu_idx]->linear_acceleration.y,
                                                       meas_ptr->imu_msg_ptr_queue[imu_idx]->linear_acceleration.z);

        meas_div_vec_ptr->at(point_num + imu_idx) = meas_div_ptr;
    });

    // sort measurement via timestamp
    std::sort(meas_div_vec_ptr->begin(), meas_div_vec_ptr->end(),
              [](const std::shared_ptr<MeasDIV>& a, const std::shared_ptr<MeasDIV>& b) -> bool { return (a->timestamp < b->timestamp); });

    if (imu_num >= 2) {
        const auto imu_head_ptr = meas_ptr->imu_msg_ptr_queue.end() - 2;
        const auto imu_tail_ptr = meas_ptr->imu_msg_ptr_queue.end() - 1;

        double dt_head_tail = imu_tail_ptr->get()->header.stamp.toSec() - imu_head_ptr->get()->header.stamp.toSec();
        double dt_head_scan = meas_ptr->scan_end_time - imu_head_ptr->get()->header.stamp.toSec();
        double interp_weight = dt_head_scan / dt_head_tail;

        std::shared_ptr<MeasDIV> meas_div_ptr = std::make_shared<MeasDIV>();
        meas_div_ptr->meas_type = MeasDIV::MeasType::AngularRate;
        meas_div_ptr->timestamp = meas_ptr->scan_end_time;
        meas_div_ptr->angular_rate = Eigen::Vector3d(((1.0 - interp_weight) * imu_head_ptr->get()->angular_velocity.x) +
                                                     (interp_weight * imu_tail_ptr->get()->angular_velocity.x),
                                                     ((1.0 - interp_weight) * imu_head_ptr->get()->angular_velocity.y) +
                                                     (interp_weight * imu_tail_ptr->get()->angular_velocity.y),
                                                     ((1.0 - interp_weight) * imu_head_ptr->get()->angular_velocity.z) +
                                                     (interp_weight * imu_tail_ptr->get()->angular_velocity.z));
        meas_div_ptr->specific_force = Eigen::Vector3d(((1.0 - interp_weight) * imu_head_ptr->get()->linear_acceleration.x) +
                                                       (interp_weight * imu_tail_ptr->get()->linear_acceleration.x),
                                                       ((1.0 - interp_weight) * imu_head_ptr->get()->linear_acceleration.y) +
                                                       (interp_weight * imu_tail_ptr->get()->linear_acceleration.y),
                                                       ((1.0 - interp_weight) * imu_head_ptr->get()->linear_acceleration.z) +
                                                       (interp_weight * imu_tail_ptr->get()->linear_acceleration.z));

        meas_div_vec_ptr->push_back(meas_div_ptr);
    } else {
        std::shared_ptr<MeasDIV> meas_div_ptr = std::make_shared<MeasDIV>();
        meas_div_ptr->meas_type = MeasDIV::MeasType::AngularRate;
        meas_div_ptr->timestamp = meas_ptr->scan_end_time;
        meas_div_ptr->angular_rate = Eigen::Vector3d(meas_ptr->imu_msg_ptr_queue.back()->angular_velocity.x,
                                                     meas_ptr->imu_msg_ptr_queue.back()->angular_velocity.y,
                                                     meas_ptr->imu_msg_ptr_queue.back()->angular_velocity.z);
        meas_div_ptr->specific_force = Eigen::Vector3d(meas_ptr->imu_msg_ptr_queue.back()->linear_acceleration.x,
                                                       meas_ptr->imu_msg_ptr_queue.back()->linear_acceleration.y,
                                                       meas_ptr->imu_msg_ptr_queue.back()->linear_acceleration.z);

        meas_div_vec_ptr->push_back(meas_div_ptr);
    }

    return meas_div_vec_ptr;
}

void FreeInit::estimateAccelerometerBiasAndGravity() {
    // get kinematics states for accelerometer bias and gravity estimation
    auto state_kin_ptr_vec = *(div_ptr_->getStatesKinematics());

    // use differentiated acceleration to compute initial gravity and accelerometer bias
    using Mat3dVec = std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>>;
    using Vec3dVec = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;

    // differentiated acceleration at last time
    Eigen::Vector3d last_a_wb_w = ((state_kin_ptr_vec[1]->getRotation() * state_kin_ptr_vec[1]->getVelocity()) -
                                   (state_kin_ptr_vec[0]->getRotation() * state_kin_ptr_vec[0]->getVelocity())) /
                                  (state_kin_ptr_vec[1]->getTime() - state_kin_ptr_vec[0]->getTime());

    // data accumulation for estimation
    Mat3dVec R_wb_vec;
    Vec3dVec f_wb_w_vec, a_wb_w_vec, g_wb_w_vec;
    for (std::size_t i = 1; i < state_kin_ptr_vec.size() - 1; ++i) {
        double dt = state_kin_ptr_vec[i + 1]->getTime() - state_kin_ptr_vec[i - 1]->getTime();

        if (dt < 1.0e-9) {
            a_wb_w_vec.push_back(last_a_wb_w);

            continue;
        }

        Eigen::Vector3d delta_v_wb_w_i_minus_1 = (state_kin_ptr_vec[i + 1]->getRotation() * state_kin_ptr_vec[i + 1]->getVelocity()) -
                                                 (state_kin_ptr_vec[i - 1]->getRotation() * state_kin_ptr_vec[i - 1]->getVelocity());
        Eigen::Vector3d a_wb_w = delta_v_wb_w_i_minus_1 / dt;

        R_wb_vec.push_back(state_kin_ptr_vec[i]->getRotation());
        f_wb_w_vec.emplace_back(state_kin_ptr_vec[i]->getRotation() * state_kin_ptr_vec[i]->getLinVelRate());
        a_wb_w_vec.push_back(a_wb_w);

        last_a_wb_w = a_wb_w;

        // gravity in defined world frame
        Eigen::Vector3d ba_w = state_kin_ptr_vec[i]->getRotation() * config_ptr_->accelerometer_bias_prior;
        Eigen::Vector3d g_wb_w = a_wb_w - f_wb_w_vec.back() + ba_w;
        g_wb_w_vec.push_back(g_wb_w);
    }

    // initial accelerometer bias and gravity via mean
    Eigen::Vector3d ba_b = config_ptr_->accelerometer_bias_prior;
    Eigen::Vector3d g_wb_w{0.0, 0.0, -config_ptr_->gravity_scale};
    if (!g_wb_w_vec.empty()) {
        g_wb_w = config_ptr_->gravity_scale * (std::accumulate(g_wb_w_vec.begin(), g_wb_w_vec.end(),
                                                               Eigen::Vector3d::Zero().eval()) /
                                               static_cast<double>(g_wb_w_vec.size())).normalized();
    }

    // optimization
    optimizer_bag_ptr_->setInitialValue(ba_b,
                                        g_wb_w);

    for (std::size_t i = 0; i < R_wb_vec.size(); ++i) {
        optimizer_bag_ptr_->addFactorAcceleration(R_wb_vec[i],
                                                  f_wb_w_vec[i],
                                                  a_wb_w_vec[i]);
    }

    optimizer_bag_ptr_->solveOptimization();
}

void FreeInit::setStateAndCov(const Eigen::Matrix3d& R_align_mat) {
    // get pose
    const auto R_p_wb_frame_def = div_ptr_->getPose();

    // rotation of previous world frame w.r.t. new world frame
    SO3 R_w_pre(R_align_mat);
    SO3 R_wb_0 = R_w_pre * R_p_wb_frame_def.first;

    // initialized state in new world frame (z axis aligned with estimated gravity)
    Eigen::Matrix3d R_wb_0_mat = R_wb_0.getMat();
    Eigen::Vector3d v_wb_w_0 = R_wb_0 * div_ptr_->getBodyVelocity();
    Eigen::Vector3d p_wb_w_0 = R_w_pre * R_p_wb_frame_def.second;
    Eigen::Vector3d bg_b_0 = div_ptr_->getGyroscopeBias();
    Eigen::Vector3d ba_b_0 = optimizer_bag_ptr_->getAccelerometerBias();
    Eigen::Vector3d g_wb_w_0 = -config_ptr_->gravity_scale * Eigen::Vector3d::UnitZ();

    // set state
    state_ptr_->setState(R_wb_0_mat,
                         v_wb_w_0,
                         p_wb_w_0,
                         bg_b_0,
                         ba_b_0,
                         g_wb_w_0);
    state_ptr_->setTime(div_ptr_->getTime());

    // set covariance
    bool is_zero_velocity_init = div_ptr_->getStateType() == StateDIV::StateType::GyroBias;
    auto dim_state = static_cast<Eigen::Index>(state_ptr_->getDim());
    P_ = Eigen::MatrixXd::Identity(dim_state, dim_state);
    if (is_zero_velocity_init) {
        // initial rotation covariance
        P_.block<3, 3>(0, 0) *= 1.0e-5;
        // initial velocity covariance
        P_.block<3, 3>(3, 3) *= 1.0e-5;
        // initial position covariance
        P_.block<3, 3>(6, 6) *= 1.0e-5;
        // initial gyroscope bias covariance
        P_.block<3, 3>(9, 9) *= 1.0e-4;
        // initial accelerometer bias covariance
        P_.block<3, 3>(12, 12) *= 1.0e-3;
        // initial gravity covariance
        P_.block<3, 3>(15, 15) *= 1.0e-5;
    } else {
        // initial covariance
        P_ *= 1.0e-4;
    }
}

void FreeInit::transformPosesAndMapPoints(const Eigen::Matrix3d& R_align_mat) {
    // rotation on SO(3)
    const SO3 R_align_SO3(R_align_mat);

    // transform pose during initialization
    for (auto& state_pose_ptr : *pose_ptr_vec_ptr_) {
        const auto q_i = (R_align_SO3.getQuat() * state_pose_ptr->getRotationQuat()).normalized();
        const auto p_i = R_align_SO3 * state_pose_ptr->getPosition();

        state_pose_ptr = std::make_shared<StatePoseQuat>(state_pose_ptr->getTime(),
                                                         q_i,
                                                         p_i);
    }

    const auto R_p_wb_frame_def = div_ptr_->getPose();
    const auto q_wb = (R_align_SO3 * R_p_wb_frame_def.first).getQuat();
    const auto p_wb_w = R_align_SO3 * R_p_wb_frame_def.second;
    pose_ptr_vec_ptr_->push_back(std::make_shared<StatePoseQuat>(div_ptr_->getTime(),
                                                                 q_wb,
                                                                 p_wb_w));

    // transform map point
    map_ptr_ = div_ptr_->getMap();
    auto& points = map_ptr_->points;
    std::for_each(std::execution::par,
                  points.begin(), points.end(),
                  [&](PointType& point) -> void {
        Eigen::Vector3d p_wp_w = R_align_SO3 * Eigen::Vector3d(point.x, point.y, point.z);
        point.x = static_cast<float>(p_wp_w.x());
        point.y = static_cast<float>(p_wp_w.y());
        point.z = static_cast<float>(p_wp_w.z());
    });
}

} // namespace fmcw_lio
