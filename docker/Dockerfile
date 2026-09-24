FROM ros:noetic-perception

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
        git libceres-dev ros-noetic-rviz \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/livox_ws
RUN mkdir src \
    && git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver.git src/livox_ros_driver \
    && source /opt/ros/noetic/setup.bash \
    && catkin_make -DCMAKE_BUILD_TYPE=Release

WORKDIR /root/fmcw_lio_ws
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && source /opt/livox_ws/devel/setup.bash && exec \"$@\"", "--"]
CMD ["bash"]
