//
// Created by bzeren on 27.03.2023.
//

#ifndef ORB_SLAM3_RGBL_DATA_PREPARATION_TOOL_H
#define ORB_SLAM3_RGBL_DATA_PREPARATION_TOOL_H

#include <iostream>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "applanix_msgs/msg/navigation_solution_gsof49.hpp"

#include "cv_bridge/cv_bridge.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>

#include <pcl_conversions/pcl_conversions.h>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/UTMUPS.hpp>

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#include "rosbag2_cpp/bag_events.hpp"
#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/visibility_control.hpp"
#include "rosbag2_cpp/reader.hpp"

#include "rosbag2_storage/bag_metadata.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/storage_filter.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>

struct ImageData {
    rcutils_time_point_value_t timestamp;
    std::string topic_name;
    sensor_msgs::msg::Image data;
    bool is_used;
};

struct PointCloudData {
    rcutils_time_point_value_t timestamp;
    std::string topic_name;
    sensor_msgs::msg::PointCloud2 data;
    bool is_used;
};

struct GnssData {
    rcutils_time_point_value_t timestamp;
    std::string topic_name;
    applanix_msgs::msg::NavigationSolutionGsof49 data;
    bool is_used;
};

class DataPreparationTool : public rclcpp::Node {
public:
    DataPreparationTool();

    void readParameters();

private:

};

class BagReader : public rclcpp::Node {
public:
    BagReader(std::string bag_path, std::string dataset_path);

    void createMatrices();

    void parseBag(const std::string &bag_path);

    void saveData(ImageData image_data, PointCloudData point_cloud_data, const GnssData &gnss_data);

    void visualizer(ImageData image_data, PointCloudData &point_cloud_data, const GnssData &gnss_data);

    void tf_publisher();

    Eigen::Matrix4d calculate_gnss(const GnssData &gnss_data);

    template<typename T>
    inline T deg_to_rad(T deg)
    {
        constexpr double multiplier = M_PI / 180.0;
        return static_cast<T>(deg * multiplier);
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_point_cloud_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_image_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr m_pose_array_publisher;

    geometry_msgs::msg::PoseArray m_pose_array;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    geometry_msgs::msg::TransformStamped trans_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> mTransformBroadcaster;

    int m_name_counter;

    std::string m_bag_path;
    std::string m_dataset_path;

    cv::Mat m_camera_matrix;
    cv::Mat m_distortion_coefficients;

    std::ofstream m_gnss_file;
    std::ofstream m_timestamp_file;

    long m_first_timestamp;

    bool m_is_first_gnss;

    GeographicLib::Geocentric m_local_earth;
    GeographicLib::LocalCartesian m_local_origin;

    GnssData m_first_gnss_data;

    Eigen::Matrix4d mLidarToCamera;
    Eigen::Matrix4d mProjectionMatrix;
};

#endif //ORB_SLAM3_RGBL_DATA_PREPARATION_TOOL_H
