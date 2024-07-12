cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

project(trajlo LANGUAGES C CXX)

if (NOT CXX_MARCH)
    set(CXX_MARCH native)
endif ()

add_subdirectory(thirdparty)

include_directories(include)

add_definitions("-D SOPHUS_USE_BASIC_LOGGING")
include_directories(thirdparty/Sophus)

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(TBB REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

# Include ROS 2 directories
include_directories(${rclcpp_INCLUDE_DIRS})
include_directories(${sensor_msgs_INCLUDE_DIRS})
include_directories(${geometry_msgs_INCLUDE_DIRS})
include_directories(${TBB_INCLUDE_DIRS})

add_executable(trajlo src/run_trajlo.cpp
        src/src/io/data_loader.cpp
        src/src/utils/config.cpp
        src/src/core/odometry.cpp
        src/src/core/map_manager.cpp
)

ament_target_dependencies(trajlo
        rclcpp
        sensor_msgs
        geometry_msgs
        TBB
        yaml-cpp
        Eigen3
)

target_link_libraries(trajlo 
        yaml-cpp::yaml-cpp
        Eigen3::Eigen
        ${TBB_LIBRARIES}
        rosbag
        pthread
        tsl::robin_map
        TBB::tbb
)

install(TARGETS trajlo
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()

