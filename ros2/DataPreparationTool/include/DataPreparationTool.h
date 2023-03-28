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
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_point_cloud_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_image_publisher;
};

class BagReader {
public:
    BagReader(std::string bag_path, std::string dataset_path);

    void parseBag(const std::string &bag_path);

    void saveData(const ImageData &image_data, const PointCloudData &point_cloud_data, const GnssData &gnss_data);

    template<typename T>
    inline T deg_to_rad(T deg)
    {
        constexpr double multiplier = M_PI / 180.0;
        return static_cast<T>(deg * multiplier);
    }

private:
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
};

#endif //ORB_SLAM3_RGBL_DATA_PREPARATION_TOOL_H
