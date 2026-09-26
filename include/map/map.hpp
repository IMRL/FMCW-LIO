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

#ifndef MAP_HPP
#define MAP_HPP

#include "state/state.hpp"

// fmcw_lio
namespace fmcw_lio {

// Map
class Map {
public:
    explicit Map(const std::shared_ptr<const Config>& config_ptr);

    // initialize map
    void initializeMap(const std::shared_ptr<StateBase>& state_ptr,
                       const PointCloudType::Ptr& scan_ptr,
                       bool is_transformed = false);

    // update local map boundary and remove points outside boundary
    void updateMapBoundary(const std::shared_ptr<StateBase>& state_ptr);

    // add points to map
    PointCloudType::Ptr addMapPoints(const std::shared_ptr<StateBase>& state_ptr, const PointCloudType::Ptr& scan_ptr,
                                     const PointVec2D& nearest_points, bool is_transformed = false);

    // search nearest points in map
    void searchNearestPoints(const PointType& point_world, int k_nearest,
                             PointVec1D& nearest_points, std::vector<float>& point_distance);

    // transform a point from LiDAR frame to world frame
    void transPointLiDARToWorld(const std::shared_ptr<StateBase>& state_ptr,
                                const PointType& pt_in, PointType& pt_out);

    // if need to initialize map
    [[nodiscard]] bool isNeedInitializeMap() const;

private:
    // configuration pointer
    const std::shared_ptr<const Config> config_ptr_;

    // map structure pointer
    MapStructurePtrVariant map_structure_ptr_;

    // map initialized flag
    bool is_local_map_initialized_;

    // local map boundary
    BoxPointType local_map_points_;

    // for counting points need to be removed
    std::vector<BoxPointType> box_need_remove_;
};

} // namespace fmcw_lio

#endif // MAP_HPP
