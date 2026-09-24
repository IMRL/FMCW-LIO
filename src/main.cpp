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

#include "bithub/bithub.hpp"

// main
int main(int argc, char* argv[]) {
    // launch FMCW-LIO
    ros::init(argc, argv, "FMCW-LIO");
    ros::NodeHandle nh;

    // configuration pointer
    const auto config_ptr = std::make_shared<fmcw_lio::Config>(nh);
    // FMCW-LIO system pointer
    const auto fmcw_lio_ptr = std::make_shared<fmcw_lio::FMCWLIO>(config_ptr);
    fmcw_lio_ptr->setTFPublishFunction(&fmcw_lio::BitHub::publishTF);
    // BitHub pointer
    const auto bithub_ptr = std::make_shared<fmcw_lio::BitHub>(nh,
                                                               config_ptr,
                                                               fmcw_lio_ptr);

    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::Rate rate(5000);

    // FMCW-LIO evolves and publish data
    while (ros::ok()) {
        if (bithub_ptr->packMeasLI()) {
            fmcw_lio_ptr->evolveSystem(bithub_ptr->getMeasPackLI());
            bithub_ptr->publishData();
        }

        rate.sleep();
    }

    // dump map
    if (config_ptr->dump_map) {
        bithub_ptr->dumpMap();
    }

    ros::waitForShutdown();
    ros::shutdown();

    return 0;
}
