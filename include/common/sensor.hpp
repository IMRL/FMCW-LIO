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

#ifndef SENSOR_HPP
#define SENSOR_HPP

#include "common/type.hpp"

// fmcw_lio
namespace fmcw_lio {

// LiDAR
class LiDAR {
public:
    explicit LiDAR(const LiDARType lidar_type) : lidar_tuple_ptr_(lidar_table_.at(lidar_type)),
                                                 is_get_offset_{false},
                                                 offset_time_{0},
                                                 offset_x_{0},
                                                 offset_y_{0},
                                                 offset_z_{0},
                                                 offset_intensity_{0} {}

    ~LiDAR() = default;

    // set field data offset
    void setOffset(const std::vector<sensor_msgs::PointField>& fields) {
        // check get offset
        if (is_get_offset_) {
            return;
        }

        // get field data offset
        for (const auto& field : fields) {
            if (field.name == std::get<0>(*lidar_tuple_ptr_)) {
                // time: index 0 in tuple
                offset_time_ = field.offset;
            } else if (field.name == "x") {
                // x
                offset_x_ = field.offset;
            } else if (field.name == "y") {
                // y
                offset_y_ = field.offset;
            } else if (field.name == "z") {
                // z
                offset_z_ = field.offset;
            } else if (field.name == std::get<4>(*lidar_tuple_ptr_)) {
                // intensity: index 4 in tuple
                offset_intensity_ = field.offset;
            }
        }

        is_get_offset_ = true;
    }

    // parse LiDAR scan first point timestamp
    void parseFirstPointTimestamp(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        // get scan first point timestamp via time variant
        std::visit([&](auto& scan_beg_time_lambda) -> void {
            // decayed time type
            using TimeType = std::decay_t<decltype(scan_beg_time_lambda)>;

            // time field type and is time offset to scan begin: index 1 and 3 in tuple
            if (std::get<3>(*lidar_tuple_ptr_)) {
                std::memcpy(&scan_beg_time_lambda, msg->data.data() + offset_time_, sizeof(TimeType));
            }
        }, std::get<1>(*lidar_tuple_ptr_));
    }

    // parse point field data
    void parsePointField(const sensor_msgs::PointCloud2::ConstPtr& msg,
                         const std::size_t& index,
                         float& timestamp, float& x, float& y, float& z, float& intensity) const {
        // get point data in every field
        const auto row = index / msg->width;
        const auto col = index % msg->width;

        const auto base = (row * msg->row_step) + (col * msg->point_step);
        const std::uint8_t* msg_data_ptr = msg->data.data() + base;
        std::memcpy(&x, msg_data_ptr + offset_x_, sizeof(float));
        std::memcpy(&y, msg_data_ptr + offset_y_, sizeof(float));
        std::memcpy(&z, msg_data_ptr + offset_z_, sizeof(float));
        std::memcpy(&intensity, msg_data_ptr + offset_intensity_, sizeof(float));

        std::visit([&](auto&& time_lambda) -> void {
            // decayed time type
            using TimeType = std::decay_t<decltype(time_lambda)>;

            // time field type: index 1 in tuple
            TimeType time_value;
            std::memcpy(&time_value, msg_data_ptr + offset_time_, sizeof(TimeType));
            timestamp = static_cast<float>(time_value - std::get<TimeType>(std::get<1>(*lidar_tuple_ptr_)));
        }, std::get<1>(*lidar_tuple_ptr_));
    }

    // timestamp unit scale: index 2 in tuple
    [[nodiscard]] float getTimeUnitScale() const { return std::get<2>(*lidar_tuple_ptr_); }

private:
    // LiDAR table:
    // LiDAR type -> (0: time field name,
    //                1: time field type,
    //                2: time field unit scale,
    //                3: is time offset to scan begin,
    //                4: field name need to be stored in PointType intensity field)
    inline static const LiDARTable lidar_table_ = {
        // Aeva
        {
            LiDARType::Aeva,
            std::make_shared<LiDARTuple>("time_offset_ns",
                                         std::int32_t{0},
                                         1.0e-6f,
                                         false,
                                         "velocity")
        },
        // Hesai
        {
            LiDARType::Hesai,
            std::make_shared<LiDARTuple>("timestamp",
                                         double{0},
                                         1.0e+3f,
                                         true,
                                         "intensity")
        },
        // RoboSense
        {
            LiDARType::RoboSense,
            std::make_shared<LiDARTuple>("timestamp",
                                         double{0},
                                         1.0e+3f,
                                         true,
                                         "intensity")
        },
        // Ouster
        {
            LiDARType::Ouster,
            std::make_shared<LiDARTuple>("t",
                                         std::uint32_t{0},
                                         1.0e-6f,
                                         false,
                                         "intensity")
        },
        // Velodyne
        {
            LiDARType::Velodyne,
            std::make_shared<LiDARTuple>("time",
                                         float{0},
                                         1.0e-3f,
                                         false,
                                         "intensity")
        },
        // Livox
        {
            LiDARType::Livox,
            std::make_shared<LiDARTuple>("offset_time",
                                         std::uint32_t{0},
                                         1.0e-6f,
                                         false,
                                         "reflectivity")
        }
    };

private:
    // LiDAR tuple pointer
    const std::shared_ptr<LiDARTuple> lidar_tuple_ptr_;

    // get offset
    bool is_get_offset_;

    // field offset
    std::uint32_t offset_time_;
    std::uint32_t offset_x_;
    std::uint32_t offset_y_;
    std::uint32_t offset_z_;
    std::uint32_t offset_intensity_;
};

} // namespace fmcw_lio

#endif // SENSOR_HPP
