//
// Created by bzeren on 27.03.2023.
//

#ifndef DATA_PREPARATION_TOOL_BAGREADER_H
#define DATA_PREPARATION_TOOL_BAGREADER_H

#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "applanix_msgs/msg/navigation_solution_gsof49.hpp"

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

class BagReader : public rclcpp::Node {
public:
    BagReader();

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_point_cloud_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_image_publisher;
private:

};

#endif //DATA_PREPARATION_TOOL_BAGREADER_H
