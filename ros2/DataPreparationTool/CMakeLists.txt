cmake_minimum_required(VERSION 3.5)
project(data_preparation_tool)

set(GeographicLib_DIR "~/libraries/GeographicLib-2.1.2/")
find_package (GeographicLib REQUIRED)

find_package(PCL 1.13.0 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(applanix_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)

include_directories(
        include
        ${PCL_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${GeographicLib_INCLUDE_DIRS}
)

add_executable(data_preparation_tool
        src/data_preparation_tool_node.cpp
        src/DataPreparationTool.cpp)

target_link_libraries(data_preparation_tool
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBRARIES}
        ${GeographicLib_LIBRARIES}
)

ament_target_dependencies(data_preparation_tool
        cv_bridge
        pcl_conversions
        ament_index_cpp
        pluginlib
        sensor_msgs
        applanix_msgs
        rclcpp
        rcpputils
        rcutils
        rmw
        rmw_implementation
        rosbag2_storage
        rosbag2_cpp
        rosidl_runtime_c
        rosidl_runtime_cpp
        rosidl_typesupport_cpp
        rosidl_typesupport_introspection_cpp
        )

install(TARGETS data_preparation_tool
        DESTINATION lib/${PROJECT_NAME})

ament_package()