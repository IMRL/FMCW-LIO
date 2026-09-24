cmake_minimum_required(VERSION 3.12)
project(fmcw_lio)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

add_definitions(-w)
add_definitions(-DROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

# Free-Init
option(BUILD_FREE_INIT "Build WITH Free-Init and Ceres Solver" ON)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    pcl_ros
    tf
    roscpp
    rospy
    message_generation
    eigen_conversions
    livox_ros_driver
)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(TBB REQUIRED)
find_package(OpenMP REQUIRED)

file(GLOB_RECURSE FMCW_LIO_SRCS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/*.cpp"
)
list(FILTER FMCW_LIO_SRCS EXCLUDE REGEX ".*/main\\.cpp$")
file(GLOB_RECURSE FREE_INIT_SRCS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/initializer/free_init/*.cpp"
)
list(REMOVE_ITEM FMCW_LIO_SRCS ${FREE_INIT_SRCS})

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
    DEPENDS Eigen3 PCL
)

add_executable(fmcw_lio
    src/main.cpp
    ${FMCW_LIO_SRCS}
)
add_dependencies(fmcw_lio ${catkin_EXPORTED_TARGETS})

target_compile_options(fmcw_lio PRIVATE -pthread -fexceptions)
target_compile_definitions(fmcw_lio PRIVATE ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_include_directories(fmcw_lio PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(fmcw_lio PRIVATE
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    TBB::tbb
)
target_link_libraries(fmcw_lio PRIVATE OpenMP::OpenMP_CXX)

if(BUILD_FREE_INIT)
    message(STATUS "BUILD_FREE_INIT=ON: Building WITH Free-Init and Ceres Solver")

    find_package(Ceres REQUIRED)
    target_compile_definitions(fmcw_lio PRIVATE BUILD_FREE_INIT)
    target_sources(fmcw_lio PRIVATE ${FREE_INIT_SRCS})
    target_include_directories(fmcw_lio PRIVATE ${CERES_INCLUDE_DIRS})
    target_link_libraries(fmcw_lio PRIVATE ${CERES_LIBRARIES})
else()
    message(STATUS "BUILD_FREE_INIT=OFF: Building WITHOUT Free-Init and Ceres Solver")
endif()
