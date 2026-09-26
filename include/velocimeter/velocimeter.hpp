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

#ifndef VELOCIMETER_HPP
#define VELOCIMETER_HPP

#include "estimator/filter.hpp"

// fmcw_lio
namespace fmcw_lio {

// LoggerVelocimetry
class LoggerVelocimetry {
public:
    LoggerVelocimetry() : success(false),
                          vel_est_time(0.0),
                          dynamic_point_removal_time(0.0),
                          static_point_num(0),
                          dynamic_point_num(0) {}

public:
    bool success;
    double vel_est_time;
    double dynamic_point_removal_time;
    std::size_t static_point_num;
    std::size_t dynamic_point_num;
};

// Velocimeter
class Velocimeter {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit Velocimeter(const std::shared_ptr<const Config>& config_ptr);

    // estimate LiDAR velocity in LiDAR frame
    bool estimateVelocity(const PointCloudType::Ptr& scan_ptr,
                          const Eigen::Vector3d& v_wl_l = Eigen::Vector3d::Zero(),
                          bool use_ransac = false);

    // dynamic point removal
    void removeDynamicPoint(const PointCloudType::Ptr& scan_ptr,
                            const PointCloudType::Ptr& scan_static_ptr,
                            const PointCloudType::Ptr& scan_dynamic_ptr,
                            const Eigen::Vector3d& v_wl_l);

    // get velocity estimation time
    [[nodiscard]] double getTime() const noexcept { return time_; }

    // get estimated velocity in LiDAR frame
    [[nodiscard]] Eigen::Vector3d getVelocityEstimated() const noexcept { return v_wl_l_; }

    // get estimated velocity covariance
    [[nodiscard]] Eigen::Matrix3d getCovariance() const noexcept { return R_v_; }

private:
    // solve LSQ problem
    [[nodiscard]] bool solveLSQ(const Eigen::MatrixXd& lsq_data,
                                Eigen::Vector3d& v_wl_l, Eigen::Matrix3d& R_v,
                                bool estimate_sigma = true, bool few_valid_points = false);

    // solve optimization via RANSAC
    [[nodiscard]] bool solveRANSAC(const Eigen::MatrixXd& lsq_data,
                                   Eigen::Vector3d& v_wl_l, Eigen::Matrix3d& R_v,
                                   std::vector<std::size_t>& inlier_idx_best);

    // uniform sampling
    [[nodiscard]] std::vector<std::size_t> sampleUniform(const std::vector<std::unordered_map<std::string, double>>& valid_points,
                                                         const Eigen::Vector3d& v_wl_l) const;

    // cosine weight sampling
    [[nodiscard]] std::vector<std::size_t> sampleCosWeight(const std::vector<std::unordered_map<std::string, double>>& valid_points,
                                                           const Eigen::Vector3d& v_wl_l) const;

    // weight matrix
    bool weightMatrix(Eigen::VectorXd& weight, 
                      const Eigen::MatrixXd& lsq_data,
                      const Eigen::Vector3d& v_wl_l) const;

    // cosine weight
    static double weightCosine(const Eigen::Vector3d& p_lp_l,
                               const Eigen::Vector3d& v_wl_l);

    // Cauchy robust kernel weight
    static double weightRobustKernelCauchy(const Eigen::Vector4d& lsq_data_row,
                                           const Eigen::Vector3d& v_wl_l);

private:
    // sampling method and sample function table
    const SamplingTable sampling_table_ = {
        // uniform sampling
        {
            SamplingMethod::Uniform,
            [this](auto&& pts,
                   auto&& vel) -> std::vector<std::size_t> {
                return this->sampleUniform(std::forward<decltype(pts)>(pts),
                                           std::forward<decltype(vel)>(vel));
            }
        },
        // cosine weighted sampling
        {
            SamplingMethod::CosWeight,
            [this](auto&& pts,
                   auto&& vel) -> std::vector<std::size_t> {
                return this->sampleCosWeight(std::forward<decltype(pts)>(pts),
                                             std::forward<decltype(vel)>(vel));
            }
        }
    };

public:
    std::shared_ptr<LoggerVelocimetry> logger_velocimetry_ptr;

private:
    // configuration pointer
    const std::shared_ptr<const Config> config_ptr_;

    // threshold for valid point number
    const std::size_t valid_point_num_thresh_;
    // ratio for condition number
    const double cond_ratio_;
    // ratio for sigma threshold when few valid points
    const double sigma_ratio_;
    // sigma offset when few valid points
    const Eigen::Vector3d vel_sigma_offset_few_;

    // RANSAC iteration number
    std::size_t ransac_iteration_num_;
    // sampled point indices
    std::vector<std::size_t> sampled_indices_;
    // sample function
    SampleFunction sample_func_;

    // velocity solution success flag
    bool success_;

    // velocity estimation time
    double time_;
    // estimated velocity in LiDAR frame
    Eigen::Vector3d v_wl_l_;
    // estimated velocity covariance
    Eigen::Matrix3d R_v_;
};

} // namespace fmcw_lio

#endif // VELOCIMETER_HPP
