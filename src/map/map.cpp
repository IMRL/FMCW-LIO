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

#include "map/map.hpp"

// fmcw_lio
namespace fmcw_lio {

// Map
Map::Map(const std::shared_ptr<const Config>& config_ptr) : config_ptr_(config_ptr),
                                                            is_local_map_initialized_(false),
                                                            local_map_points_{} {
    // initialize map structure
    switch (config_ptr_->map_structure_type) {
        case MapStructureType::ikdTree: {
            // ikd-Tree
            map_structure_ptr_ = std::make_shared<IkdTreeType>();

            auto& ikd_tree_ptr = std::get<std::shared_ptr<IkdTreeType>>(map_structure_ptr_);
            ikd_tree_ptr->set_downsample_param(static_cast<float>(config_ptr_->voxel_filter_size_map));

            break;
        }

        case MapStructureType::iVox: {
            // iVox
            IVoxType::Options ivox_options;
            ivox_options.resolution_ = static_cast<float>(config_ptr_->ivox_resolution);
            ivox_options.nearby_type_ = config_ptr->ivox_nearby_type;

            map_structure_ptr_ = std::make_shared<IVoxType>(ivox_options);

            break;
        }
    }
}

void Map::initializeMap(const std::shared_ptr<StateBase>& state_ptr,
                        const PointCloudType::Ptr& scan_ptr,
                        bool is_transformed) {
    // initialize map
    std::visit([&](auto&& map_structure_ptr) -> void {
        if (is_transformed) {
            map_structure_ptr->Build(scan_ptr->points);
        } else {
            auto scan_size = scan_ptr->points.size();
            PointCloudType::Ptr scan_world_ptr(new PointCloudType());
            scan_world_ptr->points.resize(scan_size);

            std::vector<std::size_t> point_indices(scan_size);
            std::for_each(point_indices.begin(), point_indices.end(), [n = 0](std::size_t& idx) mutable -> void { idx = n++; });
            std::for_each(std::execution::par,
                          point_indices.begin(), point_indices.end(),
                          [&](auto point_idx) -> void {
                transPointLiDARToWorld(state_ptr,
                                       scan_ptr->points[point_idx], scan_world_ptr->points[point_idx]);
            });

            map_structure_ptr->Build(scan_world_ptr->points);
        }
    }, map_structure_ptr_);
}

void Map::updateMapBoundary(const std::shared_ptr<StateBase>& state_ptr) {
    // update map boundary
    std::visit([&](auto&& map_structure_ptr) -> void {
        // decayed type of map structure
        using MapStructurePtrType = std::decay_t<decltype(map_structure_ptr)>;

        constexpr bool is_ikd_tree_type = std::is_same_v<MapStructurePtrType, std::shared_ptr<IkdTreeType>>;
        if constexpr (is_ikd_tree_type) {
            // get position of LiDAR in world frame
            const auto p_wl_w = (state_ptr->getPosition() +
                                 state_ptr->getRotation() * config_ptr_->p_bl_b).eval();

            box_need_remove_.clear();

            if (!is_local_map_initialized_) {
                for (int i = 0; i < 3; ++i) {
                    local_map_points_.vertex_min[i] = static_cast<float>(p_wl_w(i) - (config_ptr_->map_region_length / 2.0));
                    local_map_points_.vertex_max[i] = static_cast<float>(p_wl_w(i) + (config_ptr_->map_region_length / 2.0));
                }
                is_local_map_initialized_ = true;

                return;
            }

            std::array<std::array<double, 2>, 3> dist_to_map_boundary{};
            bool is_need_move = false;
            for (int i = 0; i < 3; ++i) {
                dist_to_map_boundary[i][0] = std::fabs(p_wl_w(i) - local_map_points_.vertex_min[i]);
                dist_to_map_boundary[i][1] = std::fabs(p_wl_w(i) - local_map_points_.vertex_max[i]);

                // move local map if distance to map boundary less than threshold
                if ((dist_to_map_boundary[i][0] <= (1.5 * config_ptr_->detection_range)) ||
                    (dist_to_map_boundary[i][1] <= (1.5 * config_ptr_->detection_range)))
                    is_need_move = true;
            }
            if (!is_need_move) {
                return;
            }

            BoxPointType new_local_map_points = local_map_points_;
            double mov_dist = std::max((config_ptr_->map_region_length - (2.0 * 1.5 * config_ptr_->detection_range)) * 0.5 * 0.9,
                                       config_ptr_->detection_range * 0.5);

            for (int i = 0; i < 3; ++i) {
                // points outside new map boundary to be removed
                BoxPointType tmp_box_points = local_map_points_;
                if (dist_to_map_boundary[i][0] <= (1.5 * config_ptr_->detection_range)) {
                    new_local_map_points.vertex_max[i] -= static_cast<float>(mov_dist);
                    new_local_map_points.vertex_min[i] -= static_cast<float>(mov_dist);

                    tmp_box_points.vertex_min[i] = local_map_points_.vertex_max[i] - static_cast<float>(mov_dist);

                    box_need_remove_.push_back(tmp_box_points);
                } else if (dist_to_map_boundary[i][1] <= (1.5 * config_ptr_->detection_range)) {
                    new_local_map_points.vertex_max[i] += static_cast<float>(mov_dist);
                    new_local_map_points.vertex_min[i] += static_cast<float>(mov_dist);

                    tmp_box_points.vertex_max[i] = local_map_points_.vertex_min[i] + static_cast<float>(mov_dist);

                    box_need_remove_.push_back(tmp_box_points);
                }
            }
            local_map_points_ = new_local_map_points;

            PointVec1D points_history;
            map_structure_ptr->acquire_removed_points(points_history);
            if (!box_need_remove_.empty()) {
                map_structure_ptr->Delete_Point_Boxes(box_need_remove_);
            }
        }
    }, map_structure_ptr_);
}

PointCloudType::Ptr Map::addMapPoints(const std::shared_ptr<StateBase>& state_ptr, const PointCloudType::Ptr& scan_ptr,
                                      const PointVec2D& nearest_points, bool is_transformed) {
    // add points to map
    const auto size_scan = scan_ptr->points.size();
    PointCloudType::Ptr scan_world_ptr(new PointCloudType());
    scan_world_ptr->resize(size_scan);

    PointVec1D points_to_add, points_no_need_downsample;
    points_to_add.reserve(size_scan);
    points_no_need_downsample.reserve(size_scan);

    for (int i = 0; i < size_scan; ++i) {
        if (is_transformed) {
            scan_world_ptr->points[i] = scan_ptr->points[i];
        } else {
            // transform point from LiDAR frame to world frame
            transPointLiDARToWorld(state_ptr,
                                   scan_ptr->points[i], scan_world_ptr->points[i]);
        }
        if (!nearest_points[i].empty()) {
            const auto& points_near = nearest_points[i];
            bool is_need_add = true;

            PointType center_point;
            center_point.x = static_cast<float>((std::floor(scan_world_ptr->points[i].x / config_ptr_->voxel_filter_size_map) *
                                                 config_ptr_->voxel_filter_size_map) + (0.5 * config_ptr_->voxel_filter_size_map));
            center_point.y = static_cast<float>((std::floor(scan_world_ptr->points[i].y / config_ptr_->voxel_filter_size_map) *
                                                 config_ptr_->voxel_filter_size_map) + (0.5 * config_ptr_->voxel_filter_size_map));
            center_point.z = static_cast<float>((std::floor(scan_world_ptr->points[i].z / config_ptr_->voxel_filter_size_map) *
                                                 config_ptr_->voxel_filter_size_map) + (0.5 * config_ptr_->voxel_filter_size_map));

            if ((std::fabs(points_near[0].x - center_point.x) > (0.5 * config_ptr_->voxel_filter_size_map)) &&
                (std::fabs(points_near[0].y - center_point.y) > (0.5 * config_ptr_->voxel_filter_size_map)) &&
                (std::fabs(points_near[0].z - center_point.z) > (0.5 * config_ptr_->voxel_filter_size_map))) {
                points_no_need_downsample.push_back(scan_world_ptr->points[i]);

                continue;
            }

            // distance between two points
            auto calDist = [](PointType p1, PointType p2) -> float {
                return ((p1.x - p2.x) * (p1.x - p2.x) +
                        (p1.y - p2.y) * (p1.y - p2.y) +
                        (p1.z - p2.z) * (p1.z - p2.z));
            };

            auto dist = calDist(scan_world_ptr->points[i], center_point);
            for (int re_add_i = 0; re_add_i < config_ptr_->nearest_point_num; ++re_add_i) {
                // add point if nearest point number less than matching point number
                if (points_near.size() < config_ptr_->nearest_point_num) {
                    break;
                }

                if (calDist(points_near[re_add_i], center_point) < dist) {
                    is_need_add = false;

                    break;
                }
            }

            if (is_need_add) {
                points_to_add.push_back(scan_world_ptr->points[i]);
            }
        } else {
            points_to_add.push_back(scan_world_ptr->points[i]);
        }
    }

    // add point
    std::visit([&](auto&& map_structure_ptr) -> void {
        map_structure_ptr->Add_Points(points_to_add, true);
        map_structure_ptr->Add_Points(points_no_need_downsample, false);
    }, map_structure_ptr_);

    return scan_world_ptr;
}

void Map::searchNearestPoints(const PointType& point_world, const int k_nearest,
                              PointVec1D& nearest_points, std::vector<float>& point_distance) {
    // search nearest points
    std::visit([&](auto&& map_structure_ptr) -> void {
        map_structure_ptr->Nearest_Search(point_world, k_nearest,
                                          nearest_points, point_distance);
    }, map_structure_ptr_);
}

void Map::transPointLiDARToWorld(const std::shared_ptr<StateBase>& state_ptr,
                                 const PointType& pt_in, PointType& pt_out) {
    // get pose of body frame in world frame
    auto R_wb = state_ptr->getRotation();
    auto p_wb_w = state_ptr->getPosition();

    // transform point from LiDAR frame to world frame
    Eigen::Vector3d p_lp_l{pt_in.x, pt_in.y, pt_in.z};
    Eigen::Vector3d p_wp_w = (R_wb * ((config_ptr_->R_bl * p_lp_l) + config_ptr_->p_bl_b)) + p_wb_w;

    pt_out.curvature = pt_in.curvature;
    pt_out.x = static_cast<float>(p_wp_w(0));
    pt_out.y = static_cast<float>(p_wp_w(1));
    pt_out.z = static_cast<float>(p_wp_w(2));
    pt_out.intensity = pt_in.intensity;
}

bool Map::isNeedInitializeMap() const {
    // if need to initialize map
    return std::visit([&](auto&& map_structure_ptr) -> bool {
        // decayed type of map structure
        using MapStructurePtrType = std::decay_t<decltype(map_structure_ptr)>;

        constexpr bool is_ikd_tree_type = std::is_same_v<MapStructurePtrType, std::shared_ptr<IkdTreeType>>;
        if constexpr (is_ikd_tree_type) {
            return (!map_structure_ptr ||
                    (map_structure_ptr->Root_Node == nullptr));
        } else {
            return (!(map_structure_ptr->NumValidGrids() > 0));
        }
    }, map_structure_ptr_);
}

} // namespace fmcw_lio
