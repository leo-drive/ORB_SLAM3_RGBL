//
// Created by bzeren on 28.03.2023.
//

#include "DataPreparationTool.h"

DataPreparationTool::DataPreparationTool() : Node("data_preparation_tool") {
//    m_point_cloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);
//    m_image_publisher = this->create_publisher<sensor_msgs::msg::Image>("image", 10);
    readParameters();
    BagReader bag_reader{"/home/bzeren/projects/data_collection/bags/rosbag2_2023_03_30-19_20_10",
                         "/home/bzeren/projects/data_collection/bags/dataset/"};
}

void DataPreparationTool::readParameters() {

}

BagReader::BagReader(std::string bag_path, std::string dataset_path) : Node("data_preparation_tool_2"),
                                                                       m_bag_path(std::move(bag_path)),
                                                                       m_dataset_path(std::move(dataset_path)),
                                                                       m_name_counter{0},
                                                                       m_is_first_gnss{true},
                                                                       m_local_earth(
                                                                               GeographicLib::Constants::WGS84_a(),
                                                                               GeographicLib::Constants::WGS84_f()),
                                                                       mProjectionMatrix{
                                                                               Eigen::MatrixXd::Zero(4, 4)} {
    m_point_cloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);
    m_image_publisher = this->create_publisher<sensor_msgs::msg::Image>("image", 10);
    m_pose_array_publisher = this->create_publisher<geometry_msgs::msg::PoseArray>("pose_array", 10);

    mTransformBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);


    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

//    trans_.header.stamp = this->get_clock()->now();
//    trans_.header.frame_id = "map";
//    trans_.child_frame_id = "velodyne";
//    trans_.transform.rotation.set__w(1.0);
//    trans_.transform.translation.set__x(0.0);
//    trans_.transform.translation.set__y(0.0);
//    tf_static_broadcaster_->sendTransform(trans_);

    tf_publisher();
    createMatrices();

    m_camera_matrix = (cv::Mat_<double>(3, 3)
            << 1051.301058, 0.000000, 746.065934, 0.000000, 1057.779841, 555.827273, 0.000000, 0.000000, 1.000000);
    m_distortion_coefficients = (cv::Mat_<double>(1, 5) << -0.335374, 0.107449, -0.000934, -0.000695, 0.000000);

    m_gnss_file.open(m_dataset_path + "gnss.txt");
    m_timestamp_file.open(m_dataset_path + "times.txt");

    parseBag("/home/bzeren/projects/data_collection/bags/rosbag2_2023_03_30-19_20_10");
}

void BagReader::createMatrices() {
    Eigen::MatrixXd CamIntrinsic(3, 3);
    CamIntrinsic << 1051.301058, 0.000000, 746.065934, 0.000000, 1057.779841, 555.827273, 0.000000, 0.000000, 1.000000;

    Eigen::MatrixXd temp = CamIntrinsic * mLidarToCamera.topLeftCorner(3, 4);
    mProjectionMatrix.topLeftCorner(3, 4) = temp;
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
    }
    reader.close();
}

void BagReader::saveData(ImageData image_data, PointCloudData point_cloud_data, const GnssData &gnss_data) {

    if (m_is_first_gnss) {
        std::cout << std::setprecision(15) << "First GNSS: " << gnss_data.data.roll << " " << gnss_data.data.pitch
                  << " " << gnss_data.data.heading << std::endl;

        m_first_gnss_data = gnss_data;

        m_local_origin = GeographicLib::LocalCartesian(gnss_data.data.lla.latitude, gnss_data.data.lla.longitude,
                                                       gnss_data.data.lla.altitude, m_local_earth);
        m_first_timestamp = gnss_data.timestamp;
        m_is_first_gnss = false;
    }

    // Visualizer
    visualizer(image_data, point_cloud_data, gnss_data);

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
    cv::Mat cropped_image = undisorted_image(cv::Rect(0, 0, 1440, 720));
    std::stringstream ss;
    ss << m_dataset_path << "images/" << std::setfill('0') << std::setw(6) << m_name_counter << ".png";
    cv::imwrite(ss.str(), cropped_image);

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

    // save gnss data

    Eigen::Matrix4d gnss_pose = calculate_gnss(gnss_data);

    m_gnss_file << gnss_pose(0, 0) << " " << gnss_pose(0, 1) << " " << gnss_pose(0, 2) << " " << gnss_pose(0, 3) << " "
                << gnss_pose(1, 0) << " " << gnss_pose(1, 1) << " " << gnss_pose(1, 2) << " " << gnss_pose(1, 3) << " "
                << gnss_pose(2, 0) << " " << gnss_pose(2, 1) << " " << gnss_pose(2, 2) << " " << gnss_pose(2, 3) << std::endl;

    // calculate time as second from start
    double time = (gnss_data.timestamp - m_first_timestamp) / 1000000000.0;
    m_timestamp_file << time << std::endl;

    m_name_counter++;
}

//void BagReader::visualizer(ImageData image_data, PointCloudData &point_cloud_data, const GnssData &gnss_data) {
//
//    geometry_msgs::msg::Pose pose_msg;
//
//    double x;
//    double y;
//    double z;
//    m_local_origin.Forward(gnss_data.data.lla.latitude, gnss_data.data.lla.longitude, gnss_data.data.lla.altitude, x, y,
//                           z);
//
//    // Ned to Enu-gnss
//    Eigen::Quaterniond q_ned_to_enu;
//    q_ned_to_enu = Eigen::AngleAxisd(deg_to_rad<double>(-90), Eigen::Vector3d::UnitZ()) *
//                   Eigen::AngleAxisd(deg_to_rad<double>(0), Eigen::Vector3d::UnitY()) *
//                   Eigen::AngleAxisd(deg_to_rad<double>(180), Eigen::Vector3d::UnitX());
//
//    geometry_msgs::msg::TransformStamped ned_to_enu_t;
//    ned_to_enu_t.header.stamp = rclcpp::Clock().now();
//    ned_to_enu_t.header.frame_id = "ned-gnss";
//    ned_to_enu_t.child_frame_id = "enu";
//    ned_to_enu_t.transform.translation.x = 0;
//    ned_to_enu_t.transform.translation.y = 0;
//    ned_to_enu_t.transform.translation.z = 0;
//    ned_to_enu_t.transform.rotation.x = q_ned_to_enu.x();
//    ned_to_enu_t.transform.rotation.y = q_ned_to_enu.y();
//    ned_to_enu_t.transform.rotation.z = q_ned_to_enu.z();
//    ned_to_enu_t.transform.rotation.w = q_ned_to_enu.w();
//
//    tf_static_broadcaster_->sendTransform(ned_to_enu_t);
//
//    Eigen::Affine3d ned_to_enu = tf2::transformToEigen(ned_to_enu_t);
//    Eigen::Matrix4d ned_to_enu_mat = ned_to_enu.matrix();
//    std::cout << "Ned-gnssToEnu: " << ned_to_enu_mat << std::endl;
//
//    // Enu to Velodyne
//    Eigen::Quaterniond q_enu_to_velodyne;
//    q_enu_to_velodyne = Eigen::AngleAxisd(deg_to_rad<double>(0.742711), Eigen::Vector3d::UnitZ()) *
//                        Eigen::AngleAxisd(deg_to_rad<double>(-0.464242), Eigen::Vector3d::UnitY()) *
//                        Eigen::AngleAxisd(deg_to_rad<double>(179.715196), Eigen::Vector3d::UnitX());
//
//    geometry_msgs::msg::TransformStamped enu_to_velodyne_t;
//    enu_to_velodyne_t.header.stamp = rclcpp::Clock().now();
//    enu_to_velodyne_t.header.frame_id = "enu";
//    enu_to_velodyne_t.child_frame_id = "velodyne_raw";
//    enu_to_velodyne_t.transform.translation.x = 0.0;
//    enu_to_velodyne_t.transform.translation.y = 0.0;
//    enu_to_velodyne_t.transform.translation.z = 0.1;
//    enu_to_velodyne_t.transform.rotation.x = q_enu_to_velodyne.x();
//    enu_to_velodyne_t.transform.rotation.y = q_enu_to_velodyne.y();
//    enu_to_velodyne_t.transform.rotation.z = q_enu_to_velodyne.z();
//    enu_to_velodyne_t.transform.rotation.w = q_enu_to_velodyne.w();
//
//    tf_static_broadcaster_->sendTransform(enu_to_velodyne_t);
//
//    Eigen::Affine3d enu_to_velodyne = tf2::transformToEigen(enu_to_velodyne_t);
//    Eigen::Matrix4d enu_to_velodyne_mat = enu_to_velodyne.matrix();
//    std::cout << "EnuToVelodyne: " << enu_to_velodyne_mat << std::endl;
//
//    //Velodyne to Lidar_ros
//    Eigen::Quaterniond q_velodyne_to_lidar_ros;
//    q_velodyne_to_lidar_ros = Eigen::AngleAxisd(deg_to_rad<double>(90), Eigen::Vector3d::UnitZ()) *
//                              Eigen::AngleAxisd(deg_to_rad<double>(0), Eigen::Vector3d::UnitY()) *
//                              Eigen::AngleAxisd(deg_to_rad<double>(0), Eigen::Vector3d::UnitX());
//
//    geometry_msgs::msg::TransformStamped velodyne_to_lidar_ros_t;
//    velodyne_to_lidar_ros_t.header.stamp = rclcpp::Clock().now();
//    velodyne_to_lidar_ros_t.header.frame_id = "velodyne_raw";
//    velodyne_to_lidar_ros_t.child_frame_id = "velodyne";
//    velodyne_to_lidar_ros_t.transform.translation.x = 0.0;
//    velodyne_to_lidar_ros_t.transform.translation.y = 0.0;
//    velodyne_to_lidar_ros_t.transform.translation.z = 0.0;
//    velodyne_to_lidar_ros_t.transform.rotation.x = q_velodyne_to_lidar_ros.x();
//    velodyne_to_lidar_ros_t.transform.rotation.y = q_velodyne_to_lidar_ros.y();
//    velodyne_to_lidar_ros_t.transform.rotation.z = q_velodyne_to_lidar_ros.z();
//    velodyne_to_lidar_ros_t.transform.rotation.w = q_velodyne_to_lidar_ros.w();
//
//    tf_static_broadcaster_->sendTransform(velodyne_to_lidar_ros_t);
//
//    Eigen::Affine3d velodyne_to_lidar_ros = tf2::transformToEigen(velodyne_to_lidar_ros_t);
//    Eigen::Matrix4d velodyne_to_lidar_ros_mat = velodyne_to_lidar_ros.matrix();
//    std::cout << "VelodyneToLidarRos: " << velodyne_to_lidar_ros_mat << std::endl;
//
//    // Lidar_ros to Camera
//    Eigen::Quaterniond q_lidar_ros_to_camera;
//    q_lidar_ros_to_camera = Eigen::AngleAxisd(3.1055002572281754, Eigen::Vector3d::UnitZ()) *
//                            Eigen::AngleAxisd(-0.011186821744700943, Eigen::Vector3d::UnitY()) *
//                            Eigen::AngleAxisd(-1.613769866234427, Eigen::Vector3d::UnitX());
//
//    geometry_msgs::msg::TransformStamped lidar_ros_to_camera_t;
//    lidar_ros_to_camera_t.header.stamp = rclcpp::Clock().now();
//    lidar_ros_to_camera_t.header.frame_id = "velodyne";
//    lidar_ros_to_camera_t.child_frame_id = "camera";
//    lidar_ros_to_camera_t.transform.translation.x = 0.015976527214191717;
//    lidar_ros_to_camera_t.transform.translation.y = -0.14058677051467006;
//    lidar_ros_to_camera_t.transform.translation.z = -0.08027769563042886;
//    lidar_ros_to_camera_t.transform.rotation.x = q_lidar_ros_to_camera.x();
//    lidar_ros_to_camera_t.transform.rotation.y = q_lidar_ros_to_camera.y();
//    lidar_ros_to_camera_t.transform.rotation.z = q_lidar_ros_to_camera.z();
//    lidar_ros_to_camera_t.transform.rotation.w = q_lidar_ros_to_camera.w();
//
//    tf_static_broadcaster_->sendTransform(lidar_ros_to_camera_t);
//
//    Eigen::Affine3d lidar_ros_to_camera = tf2::transformToEigen(lidar_ros_to_camera_t);
//    Eigen::Matrix4d lidar_ros_to_camera_mat = lidar_ros_to_camera.matrix();
//    std::cout << "LidarRosToCamera: " << lidar_ros_to_camera_mat << std::endl;
//
//    // Pose
////    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
////    Eigen::Quaterniond q_pose = Eigen::AngleAxisd(deg_to_rad(gnss_data.data.roll), Eigen::Vector3d::UnitX()) *
////                                Eigen::AngleAxisd(deg_to_rad(-gnss_data.data.pitch), Eigen::Vector3d::UnitY()) *
////                                Eigen::AngleAxisd(deg_to_rad(-gnss_data.data.heading), Eigen::Vector3d::UnitZ());
////    pose.block<3, 3>(0, 0) = q_pose.toRotationMatrix();
////    pose(0, 3) = x;
////    pose(1, 3) = y;
////    pose(2, 3) = z;
//
////    pose = first_pose.inverse() * pose;
////    pose = pose * enu_to_ned_mat;
////    pose = pose * gnss_to_velodyne_mat;
////    pose = pose * velodyne_to_lidar_ros_mat;
////    pose = pose * lidar_ros_to_camera_mat;
//
////    Eigen::Quaterniond q_pose_transformed(pose.block<3, 3>(0, 0));
////
////    pose_msg.position.x = pose(0, 3);
////    pose_msg.position.y = pose(1, 3);
////    pose_msg.position.z = pose(2, 3);
////    pose_msg.orientation.x = q_pose_transformed.x();
////    pose_msg.orientation.y = q_pose_transformed.y();
////    pose_msg.orientation.z = q_pose_transformed.z();
////    pose_msg.orientation.w = q_pose_transformed.w();
//
//    Eigen::Quaterniond q_pose = Eigen::AngleAxisd(deg_to_rad(gnss_data.data.heading), Eigen::Vector3d::UnitZ()) *
//                                Eigen::AngleAxisd(deg_to_rad(gnss_data.data.pitch), Eigen::Vector3d::UnitY()) *
//                                Eigen::AngleAxisd(deg_to_rad(gnss_data.data.roll), Eigen::Vector3d::UnitX());
//    Eigen::Matrix4d pose_mat = Eigen::Matrix4d::Identity();
//    pose_mat.block<3, 3>(0, 0) = q_pose.toRotationMatrix();
//    pose_mat(0, 3) = x;
//    pose_mat(1, 3) = y;
//    pose_mat(2, 3) = z;
//    pose_msg.position.x = x;
//    pose_msg.position.y = y;
//    pose_msg.position.z = z;
//    pose_msg.orientation.x = q_pose.x();
//    pose_msg.orientation.y = q_pose.y();
//    pose_msg.orientation.z = q_pose.z();
//    pose_msg.orientation.w = q_pose.w();
//
//    tf2::doTransform<geometry_msgs::msg::Pose>(pose_msg, pose_msg, ned_to_enu_t);
//    tf2::doTransform<geometry_msgs::msg::Pose>(pose_msg, pose_msg, enu_to_velodyne_t);
//    tf2::doTransform<geometry_msgs::msg::Pose>(pose_msg, pose_msg, velodyne_to_lidar_ros_t);
//    tf2::doTransform<geometry_msgs::msg::Pose>(pose_msg, pose_msg, lidar_ros_to_camera_t);
//
//    pose_mat = ned_to_enu_mat * pose_mat;
//    pose_mat = enu_to_velodyne_mat * pose_mat;
//    pose_mat = velodyne_to_lidar_ros_mat * pose_mat;
//    pose_mat = lidar_ros_to_camera_mat * pose_mat;
//
//    m_pose_array.poses.push_back(pose_msg);
//    m_pose_array.header.frame_id = "camera";
//    m_pose_array_publisher->publish(m_pose_array);
//
//    // Point Cloud
//    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZI>);
//    pcl::fromROSMsg(point_cloud_data.data, *cloud);
//    pcl::transformPointCloud(*cloud, *cloud, lidar_ros_to_camera_mat);
////    pcl::transformPointCloud(*cloud, *cloud, pose_mat);
//
//    sensor_msgs::msg::PointCloud2 point_cloud_data_transformed;
//    pcl::toROSMsg(*cloud, point_cloud_data_transformed);
//    point_cloud_data_transformed.header.frame_id = "camera";
//    m_point_cloud_publisher->publish(point_cloud_data_transformed);
//}

void BagReader::visualizer(ImageData image_data, PointCloudData &point_cloud_data, const GnssData &gnss_data) {
    // -------------------- GNSS  PART --------------------
    double x;
    double y;
    double z;
    m_local_origin.Forward(gnss_data.data.lla.latitude, gnss_data.data.lla.longitude, gnss_data.data.lla.altitude, x, y,
                           z);

    Eigen::Matrix4d pose_mat = Eigen::Matrix4d::Identity();
    pose_mat(0, 3) = x;
    pose_mat(1, 3) = y;
    pose_mat(2, 3) = z;


    // NED TO ENU
    Eigen::Quaterniond q_ned_to_enu;
    q_ned_to_enu = Eigen::AngleAxisd(deg_to_rad<double>(gnss_data.data.roll + 180), Eigen::Vector3d::UnitX()) *
                   Eigen::AngleAxisd(deg_to_rad<double>(gnss_data.data.pitch), Eigen::Vector3d::UnitY()) *
                   Eigen::AngleAxisd(deg_to_rad<double>(gnss_data.data.heading - 90), Eigen::Vector3d::UnitZ());
    q_ned_to_enu = q_ned_to_enu.inverse();
    pose_mat.block<3, 3>(0, 0) = q_ned_to_enu.toRotationMatrix();

    // GNSS TO LIDAR
    Eigen::Matrix4d gnss_to_lidar = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond gnss_to_lidar_q;
    gnss_to_lidar_q = Eigen::AngleAxisd(deg_to_rad<double>(179.715196), Eigen::Vector3d::UnitX()) *
                      Eigen::AngleAxisd(deg_to_rad<double>(-0.464242), Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(deg_to_rad<double>(90.742711), Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond gnss_to_lidar_q2 = gnss_to_lidar_q;
    gnss_to_lidar.block<3, 3>(0, 0) = gnss_to_lidar_q2.toRotationMatrix();
    gnss_to_lidar(0, 3) = 0.0; // x
    gnss_to_lidar(1, 3) = 0.0; // y
    gnss_to_lidar(2, 3) = 0.1; // z
    pose_mat = pose_mat * gnss_to_lidar;

    // LIDAR TO CAMERA
    Eigen::Matrix4d lidar_to_camera = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond lidar_to_camera_q;
    lidar_to_camera_q = Eigen::AngleAxisd(-3.116503125405655, Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(0.008405523479203703, Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(-1.5775131156436477, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond lidar_to_camera_q2 = lidar_to_camera_q;
    lidar_to_camera.block<3, 3>(0, 0) = lidar_to_camera_q2.toRotationMatrix();
    lidar_to_camera(0, 3) = -0.016989523180533185; // x
    lidar_to_camera(1, 3) = -0.10956637435592069; // y
    lidar_to_camera(2, 3) = -0.18908994499947854; // z
    pose_mat = lidar_to_camera * pose_mat;

    Eigen::Quaterniond q_last = Eigen::Quaterniond(pose_mat.block<3, 3>(0, 0));
    geometry_msgs::msg::Pose pose;
    pose.position.x = pose_mat(0, 3);
    pose.position.y = pose_mat(1, 3);
    pose.position.z = pose_mat(2, 3);
    pose.orientation.x = q_last.x();
    pose.orientation.y = q_last.y();
    pose.orientation.z = q_last.z();
    pose.orientation.w = q_last.w();

    m_pose_array.poses.push_back(pose);
    m_pose_array.header.frame_id = "pylon_camera_link";
    m_pose_array_publisher->publish(m_pose_array);
    // -------------------- GNSS  PART --------------------

    // -------------------- POINT CLOUD PART --------------------
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(point_cloud_data.data, *cloud);
    pcl::transformPointCloud(*cloud, *cloud, pose_mat);

    sensor_msgs::msg::PointCloud2 point_cloud_data_transformed;
    pcl::toROSMsg(*cloud, point_cloud_data_transformed);
    point_cloud_data_transformed.header.frame_id = "pylon_camera_link";
    m_point_cloud_publisher->publish(point_cloud_data_transformed);
    // -------------------- POINT CLOUD PART --------------------
}

void BagReader::tf_publisher() {

    // Publish transforme between enu and ned
    Eigen::Quaternionf q_enu_to_ned;
    q_enu_to_ned = Eigen::AngleAxisf(deg_to_rad<double>(180), Eigen::Vector3f::UnitX()) *
                   Eigen::AngleAxisf(deg_to_rad<double>(0), Eigen::Vector3f::UnitY()) *
                   Eigen::AngleAxisf(deg_to_rad<double>(-90), Eigen::Vector3f::UnitZ());

    geometry_msgs::msg::TransformStamped enu_to_ned;
    enu_to_ned.header.stamp = rclcpp::Clock().now();
    enu_to_ned.header.frame_id = "enu";
    enu_to_ned.child_frame_id = "ned";
    enu_to_ned.transform.translation.x = 0;
    enu_to_ned.transform.translation.y = 0;
    enu_to_ned.transform.translation.z = 0;
    enu_to_ned.transform.rotation.x = q_enu_to_ned.x();
    enu_to_ned.transform.rotation.y = q_enu_to_ned.y();
    enu_to_ned.transform.rotation.z = q_enu_to_ned.z();
    enu_to_ned.transform.rotation.w = q_enu_to_ned.w();

    tf_static_broadcaster_->sendTransform(enu_to_ned);

    // Publish transforme between ned to gnss
    geometry_msgs::msg::TransformStamped ned_to_gnss;
    ned_to_gnss.header.stamp = rclcpp::Clock().now();
    ned_to_gnss.header.frame_id = "ned";
    ned_to_gnss.child_frame_id = "gnss";
    ned_to_gnss.transform.translation.x = 0;
    ned_to_gnss.transform.translation.y = 0;
    ned_to_gnss.transform.translation.z = 0;
    ned_to_gnss.transform.rotation.x = 0;
    ned_to_gnss.transform.rotation.y = 0;
    ned_to_gnss.transform.rotation.z = 0;
    ned_to_gnss.transform.rotation.w = 1;

    tf_static_broadcaster_->sendTransform(ned_to_gnss);

    // Publish transforme between gnss to velodyne
    Eigen::Quaternionf q_gnss_to_velodyne;
    q_gnss_to_velodyne = Eigen::AngleAxisf(deg_to_rad<double>(179.715196), Eigen::Vector3f::UnitX()) *
                         Eigen::AngleAxisf(deg_to_rad<double>(-0.464242), Eigen::Vector3f::UnitY()) *
                         Eigen::AngleAxisf(deg_to_rad<double>(90.742711), Eigen::Vector3f::UnitZ());
    geometry_msgs::msg::TransformStamped gnss_to_velodyne;
    gnss_to_velodyne.header.stamp = rclcpp::Clock().now();
    gnss_to_velodyne.header.frame_id = "gnss";
    gnss_to_velodyne.child_frame_id = "velodyne";
    gnss_to_velodyne.transform.translation.x = 0.1;
    gnss_to_velodyne.transform.translation.y = 0;
    gnss_to_velodyne.transform.translation.z = 0;
    gnss_to_velodyne.transform.rotation.x = q_gnss_to_velodyne.x();
    gnss_to_velodyne.transform.rotation.y = q_gnss_to_velodyne.y();
    gnss_to_velodyne.transform.rotation.z = q_gnss_to_velodyne.z();
    gnss_to_velodyne.transform.rotation.w = q_gnss_to_velodyne.w();

    tf_static_broadcaster_->sendTransform(gnss_to_velodyne);

    // Publish transforme between velodyne to lidar_ros
    Eigen::Quaternionf q_velodyne_to_lidar_ros;
    q_velodyne_to_lidar_ros = Eigen::AngleAxisf(deg_to_rad<double>(0), Eigen::Vector3f::UnitX()) *
                              Eigen::AngleAxisf(deg_to_rad<double>(0), Eigen::Vector3f::UnitY()) *
                              Eigen::AngleAxisf(deg_to_rad<double>(90), Eigen::Vector3f::UnitZ());
    geometry_msgs::msg::TransformStamped velodyne_to_lidar_ros;
    velodyne_to_lidar_ros.header.stamp = rclcpp::Clock().now();
    velodyne_to_lidar_ros.header.frame_id = "velodyne";
    velodyne_to_lidar_ros.child_frame_id = "lidar_ros";
    velodyne_to_lidar_ros.transform.translation.x = 0;
    velodyne_to_lidar_ros.transform.translation.y = 0;
    velodyne_to_lidar_ros.transform.translation.z = 0;
    velodyne_to_lidar_ros.transform.rotation.x = q_velodyne_to_lidar_ros.x();
    velodyne_to_lidar_ros.transform.rotation.y = q_velodyne_to_lidar_ros.y();
    velodyne_to_lidar_ros.transform.rotation.z = q_velodyne_to_lidar_ros.z();
    velodyne_to_lidar_ros.transform.rotation.w = q_velodyne_to_lidar_ros.w();

    tf_static_broadcaster_->sendTransform(velodyne_to_lidar_ros);

    // Publish transforme between lidar_ros to camera
    Eigen::Quaternionf q_lidar_ros_to_camera;
    q_lidar_ros_to_camera = Eigen::AngleAxisf(3.1055002572281754, Eigen::Vector3f::UnitZ()) *
                            Eigen::AngleAxisf(-0.011186821744700943, Eigen::Vector3f::UnitY()) *
                            Eigen::AngleAxisf(-1.613769866234427, Eigen::Vector3f::UnitX());
    geometry_msgs::msg::TransformStamped lidar_ros_to_camera;
    lidar_ros_to_camera.header.stamp = rclcpp::Clock().now();
    lidar_ros_to_camera.header.frame_id = "lidar_ros";
    lidar_ros_to_camera.child_frame_id = "pylon_camera_link";
    lidar_ros_to_camera.transform.translation.x = 0.015976527214191717;
    lidar_ros_to_camera.transform.translation.y = -0.14058677051467006;
    lidar_ros_to_camera.transform.translation.z = -0.08027769563042886;
    lidar_ros_to_camera.transform.rotation.x = q_lidar_ros_to_camera.x();
    lidar_ros_to_camera.transform.rotation.y = q_lidar_ros_to_camera.y();
    lidar_ros_to_camera.transform.rotation.z = q_lidar_ros_to_camera.z();
    lidar_ros_to_camera.transform.rotation.w = q_lidar_ros_to_camera.w();

    tf_static_broadcaster_->sendTransform(lidar_ros_to_camera);

    Eigen::Affine3d affine_velo_to_cam_ = tf2::transformToEigen(lidar_ros_to_camera);

    mLidarToCamera = affine_velo_to_cam_.matrix();

    std::cout << "mLidarToCamera: " << mLidarToCamera << std::endl;
}

Eigen::Matrix4d BagReader::calculate_gnss(const GnssData &gnss_data) {

    double x;
    double y;
    double z;
    m_local_origin.Forward(gnss_data.data.lla.latitude, gnss_data.data.lla.longitude, gnss_data.data.lla.altitude, x, y,
                           z);

    Eigen::Matrix4d pose_mat = Eigen::Matrix4d::Identity();
    pose_mat(0, 3) = x;
    pose_mat(1, 3) = y;
    pose_mat(2, 3) = z;

    // NED TO ENU
    Eigen::Quaterniond q_ned_to_enu;
    q_ned_to_enu = Eigen::AngleAxisd(deg_to_rad<double>(gnss_data.data.roll + 180), Eigen::Vector3d::UnitX()) *
                   Eigen::AngleAxisd(deg_to_rad<double>(gnss_data.data.pitch), Eigen::Vector3d::UnitY()) *
                   Eigen::AngleAxisd(deg_to_rad<double>(gnss_data.data.heading - 90), Eigen::Vector3d::UnitZ());
    q_ned_to_enu = q_ned_to_enu.inverse();
    pose_mat.block<3, 3>(0, 0) = q_ned_to_enu.toRotationMatrix();

    // GNSS TO LIDAR
    Eigen::Matrix4d gnss_to_lidar = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond gnss_to_lidar_q;
    gnss_to_lidar_q = Eigen::AngleAxisd(deg_to_rad<double>(179.715196), Eigen::Vector3d::UnitX()) *
                      Eigen::AngleAxisd(deg_to_rad<double>(-0.464242), Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(deg_to_rad<double>(90.742711), Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond gnss_to_lidar_q2 = gnss_to_lidar_q;
    gnss_to_lidar.block<3, 3>(0, 0) = gnss_to_lidar_q2.toRotationMatrix();
    gnss_to_lidar(0, 3) = 0.0; // x
    gnss_to_lidar(1, 3) = 0.0; // y
    gnss_to_lidar(2, 3) = 0.1; // z
    pose_mat = pose_mat * gnss_to_lidar;

    // LIDAR TO CAMERA
    Eigen::Matrix4d lidar_to_camera = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond lidar_to_camera_q;
    lidar_to_camera_q = Eigen::AngleAxisd(-3.116503125405655, Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(0.008405523479203703, Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(-1.5775131156436477, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond lidar_to_camera_q2 = lidar_to_camera_q;
    lidar_to_camera.block<3, 3>(0, 0) = lidar_to_camera_q2.toRotationMatrix();
    lidar_to_camera(0, 3) = -0.016989523180533185; // x
    lidar_to_camera(1, 3) = -0.10956637435592069; // y
    lidar_to_camera(2, 3) = -0.18908994499947854; // z
    pose_mat = lidar_to_camera * pose_mat;

    return pose_mat;
}