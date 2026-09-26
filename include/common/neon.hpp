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

#ifndef NEON_HPP
#define NEON_HPP

#include <optional>

// fmcw_lio
namespace fmcw_lio {

// Neon
class Neon {
public:
    using RGB = std::array<std::uint8_t, 3>;

    Neon(std::initializer_list<std::pair<std::string, RGB>> init) {
        resetNeon(std::vector<std::pair<std::string, RGB>>(init));
    }

    explicit Neon(std::vector<std::pair<std::string, RGB>> vec) {
        resetNeon(std::move(vec));
    }

    // reset string to RGB vector
    void resetNeon(std::vector<std::pair<std::string, RGB>> vec) {
        str_rgb_vec_ = std::move(vec);

        for (auto& item : lut_) {
            item.reset();
        }

        for (auto& item : str_rgb_vec_) {
            const std::string& key = item.first;
            if (key.size() != 1) {
                continue;
            }

            auto idx = static_cast<unsigned char>(key[0]);
            lut_[idx] = item.second;
        }
    }

    // get string
    [[nodiscard]] std::string getString() const {
        std::string str;
        str.reserve(str_rgb_vec_.size());

        for (auto const& item : str_rgb_vec_) {
            if (!item.first.empty()) {
                str += item.first;
            }
        }

        return str;
    }

    // get RGB vector
    [[nodiscard]] std::vector<RGB> getRGBVec() const {
        std::vector<RGB> rgb_vec;
        rgb_vec.reserve(str_rgb_vec_.size());

        for (auto const& item : str_rgb_vec_) {
            rgb_vec.push_back(item.second);
        }

        return rgb_vec;
    }

    // get string to RGB vector
    [[nodiscard]] std::vector<std::pair<std::string, RGB>> const& getStringRGBVec() const noexcept {
        return str_rgb_vec_;
    }

    // get RGB via a given string
    [[nodiscard]] std::optional<RGB> getRGB(std::string const& key) const {
        if (key.size() != 1) {
            return std::nullopt;
        }

        auto idx = static_cast<unsigned char>(key[0]);

        return lut_[idx];
    }

    // get RGB via a given string using index
    [[nodiscard]] std::optional<RGB> operator[](std::string const& key) const {
        return getRGB(key);
    }

private:
    // string to RGB vector
    std::vector<std::pair<std::string, RGB>> str_rgb_vec_;
    // string to RGB lookup table
    std::array<std::optional<RGB>, 256> lut_;
};

} // namespace fmcw_lio

#endif // NEON_HPP
