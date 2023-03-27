//
// Created by bzeren on 27.03.2023.
//

#include "BagReader.h"

BagReader::BagReader() : Node("bag_reader") {
    m_point_cloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);
    m_image_publisher = this->create_publisher<sensor_msgs::msg::Image>("image", 10);

    std::unique_ptr<rosbag2_cpp::reader_interfaces::BaseReaderInterface> reader_impl = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
    rosbag2_cpp::Reader reader(std::move(reader_impl));
    reader.open("/media/bzeren/PortableSSD/YTU-DATA/21.03.2023/rosbag2_2023_03_21-19_16_54/");

    // Get all topics and their types
    auto topics_and_types = reader.get_all_topics_and_types();
    for (const auto &topic_and_type: topics_and_types) {
        std::cout << topic_and_type.name << " " << topic_and_type.type << std::endl;
    }

    ImageData image_data;
    PointCloudData point_cloud_data;
    GnssData gnss_data;

    while (reader.has_next() && rclcpp::ok()) {
        auto bag_message = reader.read_next();

        if (bag_message->topic_name == "/my_camera/pylon_ros2_camera_node/image_raw") {
//            std::cout << "image" << std::endl;
            rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
            rclcpp::SerializedMessage extracted_message(*bag_message->serialized_data);
            serialization.deserialize_message(&extracted_message, &image_data.data);
            image_data.timestamp = bag_message->time_stamp;
            image_data.topic_name = bag_message->topic_name;
            image_data.is_used = false;
        } else if (bag_message->topic_name == "/velodyne_points") {
//            std::cout << "point_cloud" << std::endl;
            rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
            rclcpp::SerializedMessage extracted_message(*bag_message->serialized_data);
            serialization.deserialize_message(&extracted_message, &point_cloud_data.data);
            point_cloud_data.timestamp = bag_message->time_stamp;
            point_cloud_data.topic_name = bag_message->topic_name;
            point_cloud_data.is_used = false;
        } else if (bag_message->topic_name == "/applanix/lvx_client/gsof/ins_solution_49") {
//            std::cout << "gnss" << std::endl;
            rclcpp::Serialization<applanix_msgs::msg::NavigationSolutionGsof49> serialization;
            rclcpp::SerializedMessage extracted_message(*bag_message->serialized_data);
            serialization.deserialize_message(&extracted_message, &gnss_data.data);
            gnss_data.timestamp = bag_message->time_stamp;
            gnss_data.topic_name = bag_message->topic_name;
            gnss_data.is_used = false;
        }

        if (image_data.is_used || point_cloud_data.is_used || gnss_data.is_used) {
            continue;
        }

//        std::cout << "image_timestamp: " << image_data.timestamp << std::endl;
//        std::cout << "point_cloud_timestamp: " << point_cloud_data.timestamp << std::endl;
//        std::cout << "gnss_timestamp: " << gnss_data.timestamp << std::endl;
//        std::cout << "---------------------------------" << std::endl;

        if (image_data.timestamp <= gnss_data.timestamp && image_data.timestamp <= point_cloud_data.timestamp) {
            if (gnss_data.timestamp - image_data.timestamp < 10000000 && point_cloud_data.timestamp - image_data.timestamp < 10000000) {
                // TODO: sync
                std::cout << "sync" << std::endl;
                image_data.is_used = true;
                point_cloud_data.is_used = true;
                gnss_data.is_used = true;
                continue;
            }
        } else if (point_cloud_data.timestamp <= gnss_data.timestamp && point_cloud_data.timestamp <= image_data.timestamp) {
            if (gnss_data.timestamp - point_cloud_data.timestamp < 10000000 && image_data.timestamp - point_cloud_data.timestamp < 10000000) {
                // TODO: sync
                std::cout << "sync" << std::endl;
                image_data.is_used = true;
                point_cloud_data.is_used = true;
                gnss_data.is_used = true;
                continue;
            }
        } else {
            if (point_cloud_data.timestamp - gnss_data.timestamp < 10000000 && image_data.timestamp - gnss_data.timestamp < 10000000) {
                // TODO: sync
                std::cout << "sync" << std::endl;
                image_data.is_used = true;
                point_cloud_data.is_used = true;
                gnss_data.is_used = true;
                continue;
            }
        }
        std::cout << "anything" << std::endl;
    }
    reader.close();
}