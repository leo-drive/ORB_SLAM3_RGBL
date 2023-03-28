//
// Created by bzeren on 28.03.2023.
//

#include "DataPreparationTool.h"

DataPreparationTool::DataPreparationTool() : Node("data_preparation_tool") {
//    m_point_cloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);
//    m_image_publisher = this->create_publisher<sensor_msgs::msg::Image>("image", 10);
    readParameters();
    BagReader bag_reader{"/media/bzeren/PortableSSD/YTU-DATA/21.03.2023/rosbag2_2023_03_21-19_16_54/",
                         "/media/bzeren/PortableSSD/YTU-DATA/21.03.2023/dataset/"};
}

void DataPreparationTool::readParameters() {

}

BagReader::BagReader(std::string bag_path, std::string dataset_path) : m_bag_path(std::move(bag_path)),
                                                                       m_dataset_path(std::move(dataset_path)),
                                                                       m_name_counter{0},
                                                                       m_is_first_gnss{true},
                                                                       m_local_earth(
                                                                               GeographicLib::Constants::WGS84_a(),
                                                                               GeographicLib::Constants::WGS84_f()) {
    m_camera_matrix = (cv::Mat_<double>(3, 3)
            << 1051.301058, 0.000000, 746.065934, 0.000000, 1057.779841, 555.827273, 0.000000, 0.000000, 1.000000);
    m_distortion_coefficients = (cv::Mat_<double>(1, 5) << -0.335374, 0.107449, -0.000934, -0.000695, 0.000000);

    m_gnss_file.open(m_dataset_path + "gnss.txt");
    m_timestamp_file.open(m_dataset_path + "times.txt");

    parseBag("/media/bzeren/PortableSSD/YTU-DATA/21.03.2023/rosbag2_2023_03_21-19_16_54/");
}

void BagReader::parseBag(const std::string &bag_path) {
    std::unique_ptr<rosbag2_cpp::reader_interfaces::BaseReaderInterface> reader_impl = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
    rosbag2_cpp::Reader reader(std::move(reader_impl));
    reader.open(bag_path);

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
            rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
            rclcpp::SerializedMessage extracted_message(*bag_message->serialized_data);
            serialization.deserialize_message(&extracted_message, &image_data.data);
            image_data.timestamp = bag_message->time_stamp;
            image_data.topic_name = bag_message->topic_name;
            image_data.is_used = false;
        } else if (bag_message->topic_name == "/velodyne_points") {
            rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
            rclcpp::SerializedMessage extracted_message(*bag_message->serialized_data);
            serialization.deserialize_message(&extracted_message, &point_cloud_data.data);
            point_cloud_data.timestamp = bag_message->time_stamp;
            point_cloud_data.topic_name = bag_message->topic_name;
            point_cloud_data.is_used = false;
        } else if (bag_message->topic_name == "/applanix/lvx_client/gsof/ins_solution_49") {
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

        if (image_data.timestamp <= gnss_data.timestamp && image_data.timestamp <= point_cloud_data.timestamp) {
            if (gnss_data.timestamp - image_data.timestamp < 10000000 &&
                point_cloud_data.timestamp - image_data.timestamp < 10000000) {
                saveData(image_data, point_cloud_data, gnss_data);
                image_data.is_used = true;
                point_cloud_data.is_used = true;
                gnss_data.is_used = true;
                continue;
            }
        } else if (point_cloud_data.timestamp <= gnss_data.timestamp &&
                   point_cloud_data.timestamp <= image_data.timestamp) {
            if (gnss_data.timestamp - point_cloud_data.timestamp < 10000000 &&
                image_data.timestamp - point_cloud_data.timestamp < 10000000) {
                saveData(image_data, point_cloud_data, gnss_data);
                image_data.is_used = true;
                point_cloud_data.is_used = true;
                gnss_data.is_used = true;
                continue;
            }
        } else {
            if (point_cloud_data.timestamp - gnss_data.timestamp < 10000000 &&
                image_data.timestamp - gnss_data.timestamp < 10000000) {
                saveData(image_data, point_cloud_data, gnss_data);
                image_data.is_used = true;
                point_cloud_data.is_used = true;
                gnss_data.is_used = true;
                continue;
            }
        }
        std::cout << "No sync" << std::endl;
    }
    reader.close();
}

void
BagReader::saveData(const ImageData &image_data, const PointCloudData &point_cloud_data, const GnssData &gnss_data) {

    if (m_is_first_gnss) {
        m_local_origin = GeographicLib::LocalCartesian(gnss_data.data.lla.latitude, gnss_data.data.lla.longitude,
                                                       gnss_data.data.lla.altitude, m_local_earth);
        m_first_timestamp = gnss_data.timestamp;
        m_is_first_gnss = false;
    }

    // undistort image and save image
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(image_data.data, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        std::cout << "cv_bridge error: " << e.what() << std::endl;
        return;
    }
    cv::Mat undisorted_image;
    cv::undistort(cv_ptr->image, undisorted_image, m_camera_matrix, m_distortion_coefficients);
    std::stringstream ss;
    ss << m_dataset_path << "images/" << std::setfill('0') << std::setw(6) << m_name_counter << ".png";
    cv::imwrite(ss.str(), undisorted_image);

    // save point cloud as binary
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(point_cloud_data.data, *point_cloud);

    std::stringstream out_file;
    out_file << m_dataset_path << "clouds/" << std::setfill('0') << std::setw(6) << m_name_counter << ".bin";
    std::ofstream bin_file(out_file.str(), std::ios::out | std::ios::binary | std::ios::app);
    if (!bin_file.good()) std::cout << "Couldn't open " << out_file.str() << std::endl;
    for (auto &point: point_cloud->points) {
        bin_file.write((char *) &point.x, 3 * sizeof(float));
        bin_file.write((char *) &point.intensity, sizeof(float));
    }
    bin_file.close();

    // save gnss
    double x;
    double y;
    double z;
    m_local_origin.Forward(gnss_data.data.lla.latitude, gnss_data.data.lla.longitude, gnss_data.data.lla.altitude, x, y,
                           z);
    Eigen::Quaternionf q;
    q = Eigen::AngleAxisf(deg_to_rad<double>(gnss_data.data.roll + 180), Eigen::Vector3f::UnitX()) *
        Eigen::AngleAxisf(deg_to_rad<double>(gnss_data.data.pitch), Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(deg_to_rad<double>(gnss_data.data.heading - 90), Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose.block<3, 3>(0, 0) = q.toRotationMatrix();
    pose(0, 3) = x;
    pose(1, 3) = y;
    pose(2, 3) = z;

    m_gnss_file << pose(0, 0) << " " << pose(0, 1) << " " << pose(0, 2) << " " << pose(0, 3) << " "
                << pose(1, 0) << " " << pose(1, 1) << " " << pose(1, 2) << " " << pose(1, 3) << " "
                << pose(2, 0) << " " << pose(2, 1) << " " << pose(2, 2) << " " << pose(2, 3) << std::endl;

    // calculate time as second from start
    double time = (gnss_data.timestamp - m_first_timestamp) / 1000000000.0;
    m_timestamp_file << time << std::endl;

    m_name_counter++;
}