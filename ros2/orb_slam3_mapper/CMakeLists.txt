cmake_minimum_required(VERSION 3.5)
project(orb_slam3_mapper)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/")

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PCL_DIR "~/libraries/libpcl-1.12.0/share/pcl-1.12/")
find_package(PCL 1.12.0 REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(point_cloud_msg_wrapper REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(ORB_SLAM3 REQUIRED)

link_directories(lib)

include_directories(
        include
        ${Pangolin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/../../
        ${PROJECT_SOURCE_DIR}/../../include
        ${PROJECT_SOURCE_DIR}/../../include/CameraModels
        ${PROJECT_SOURCE_DIR}/../../Thirdparty/Sophus
)

add_executable(mapper_node
        src/Mapper.cpp
        src/mapper_node.cpp)

target_link_libraries(mapper_node
        ${Pangolin_LIBRARIES}
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        )

ament_target_dependencies(mapper_node
        rclcpp
        sensor_msgs
        cv_bridge
        message_filters
        Pangolin
        Sophus
        ORB_SLAM3
        tf2
        tf2_ros
        point_cloud_msg_wrapper
        PCL
        )

install(
        DIRECTORY ${PROJECT_SOURCE_DIR}/../../lib/
        DESTINATION lib
)

install(
        DIRECTORY ${PROJECT_SOURCE_DIR}/../../Thirdparty/DBoW2/lib/
        DESTINATION lib
)

install(
        DIRECTORY ${PROJECT_SOURCE_DIR}/../../Thirdparty/g2o/lib/
        DESTINATION lib
)

install(TARGETS mapper_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        launch
        config
        DESTINATION share/${PROJECT_NAME}/
        )

ament_package()
