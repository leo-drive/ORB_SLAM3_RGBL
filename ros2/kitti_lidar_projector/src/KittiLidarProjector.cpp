//
// Created by bzeren on 08.12.2022.
//

#include "kitti_lidar_projector/KittiLidarProjector.h"

namespace kitti_lidar_projector {

    KittiLidarProjectorNode::KittiLidarProjectorNode(int argc, char **argv) : Node("kitti_lidar_projector"),
                                                                              mSLAM("/home/bzeren/leo/ORB_SLAM3_RGBL/Vocabulary/ORBvoc.txt",
                                                                                    "/home/bzeren/leo/ORB_SLAM3_RGBL/Examples/RGB-L/KITTI00-02.yaml",
                                                                                    ORB_SLAM3::System::RGBL, true),
                                                                              mProjectionMatrix{
                                                                                      Eigen::MatrixXd::Zero(3, 4)} {
        RCLCPP_INFO(this->get_logger(), "KittiLidarProjectorNode has been initialized.");

        readParams();
        createProjectionMatrix();

        mPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("kitti_lidar_projector/point_cloud", 10);
        mProjectedPublisher = this->create_publisher<sensor_msgs::msg::Image>("kitti_lidar_projector/projected_image",
                                                                              10);
        mPosePublisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("kitti_lidar_projector/pose", 10);

        mTransformBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);


        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        trans_.header.stamp = this->get_clock()->now();
        trans_.header.frame_id = "map";
        trans_.child_frame_id = "xyzi";
        trans_.transform.rotation.set__w(1.0);
        trans_.transform.translation.set__x(0.0);
        trans_.transform.translation.set__y(0.0);
        tf_static_broadcaster_->sendTransform(trans_);

        std::string dataset_path = mPathToSequence.c_str();
        LoadImages(dataset_path, mvstrImageFilenamesRGB, mvstrPcdFilenames, mvTimestamps);
        RCLCPP_INFO(this->get_logger(), "Loaded %d images and %d pointclouds.", mvstrImageFilenamesRGB.size(),
                    mvstrPcdFilenames.size());

        cv::Mat imRGB, pcd;
        sensor_msgs::msg::PointCloud2 pc2msg;
        for (int ni = 0; ni < mvstrImageFilenamesRGB.size(); ni++) {

            imRGB = cv::imread(mvstrImageFilenamesRGB[ni], cv::IMREAD_UNCHANGED);

            LoadPointcloudBinary(mvstrPcdFilenames[ni], pcd, pc2msg);
            double tframe = mvTimestamps[ni];

            Sophus::SE3f Tcw = mSLAM.TrackRGBL(imRGB, pcd, tframe);

            publishPose(Tcw, tframe);
            publishTransform(Tcw, tframe);
//            publishPointCloud(pc2msg, tframe);

            auto key_points = mSLAM.GetTrackedKeyPointsUn();
            auto map_points = mSLAM.GetTrackedMapPoints();
//            visualizeProjection(imRGB, pc2msg, key_points, map_points);

//            last_map_point->SetWorldPos()

//            RCLCPP_INFO(this->get_logger(), "Tracking %d/%d", ni, mvstrImageFilenamesRGB.size());
        }

        RCLCPP_INFO(this->get_logger(), "SLAM Shutdown");
        mSLAM.Shutdown();
    }

    KittiLidarProjectorNode::~KittiLidarProjectorNode() {
        RCLCPP_INFO(this->get_logger(), "KittiLidarProjectorNode has been destroyed.");
    }

    void KittiLidarProjectorNode::readParams() {
        this->declare_parameter("voc_path");
        this->declare_parameter("settings_path");
        this->declare_parameter("sequence_path");
        this->declare_parameter("camera.intrinsic_matrix.data");
        this->declare_parameter("camera.distortion_coefficients.data");
        this->declare_parameter("camera.height");
        this->declare_parameter("camera.width");
        this->declare_parameter("lidar_camera_transform.data");
        this->declare_parameter("projection.min_dist");
        this->declare_parameter("projection.max_dist");

        this->get_parameter("voc_path", mPathToVocabulary);
        this->get_parameter("settings_path", mPathToSettings);
        this->get_parameter("sequence_path", mPathToSequence);
        this->get_parameter("camera.intrinsic_matrix.data", mCameraIntrinsicMatrix);
        this->get_parameter("camera.distortion_coefficients.data", mCameraDistortionCoefficients);
        this->get_parameter("camera.height", mCameraHeight);
        this->get_parameter("camera.width", mCameraWidth);
        this->get_parameter("lidar_camera_transform.data", mLidarCameraTransform);
        this->get_parameter("projection.min_dist", mMinDistance);
        this->get_parameter("projection.max_dist", mMaxDistance);

        RCLCPP_INFO(this->get_logger(), "Vocabulary path: %s", mPathToVocabulary.c_str());
        RCLCPP_INFO(this->get_logger(), "Settings path: %s", mPathToSettings.c_str());
        RCLCPP_INFO(this->get_logger(), "Sequence path: %s", mPathToSequence.c_str());
        RCLCPP_INFO(this->get_logger(), "Camera intrinsic matrix: %f %f %f %f %f %f %f %f %f",
                    mCameraIntrinsicMatrix[0], mCameraIntrinsicMatrix[1], mCameraIntrinsicMatrix[2],
                    mCameraIntrinsicMatrix[3], mCameraIntrinsicMatrix[4], mCameraIntrinsicMatrix[5],
                    mCameraIntrinsicMatrix[6], mCameraIntrinsicMatrix[7], mCameraIntrinsicMatrix[8]);
        RCLCPP_INFO(this->get_logger(), "Camera distortion coefficients: %f %f %f %f %f",
                    mCameraDistortionCoefficients[0], mCameraDistortionCoefficients[1],
                    mCameraDistortionCoefficients[2], mCameraDistortionCoefficients[3],
                    mCameraDistortionCoefficients[4]);
        RCLCPP_INFO(this->get_logger(), "Camera height: %d", mCameraHeight);
        RCLCPP_INFO(this->get_logger(), "Camera width: %d", mCameraWidth);
        RCLCPP_INFO(this->get_logger(), "Lidar camera transform: %f %f %f %f %f %f %f %f %f %f %f %f",
                    mLidarCameraTransform[0], mLidarCameraTransform[1], mLidarCameraTransform[2],
                    mLidarCameraTransform[3], mLidarCameraTransform[4], mLidarCameraTransform[5],
                    mLidarCameraTransform[6], mLidarCameraTransform[7], mLidarCameraTransform[8],
                    mLidarCameraTransform[9], mLidarCameraTransform[10], mLidarCameraTransform[11]);
        RCLCPP_INFO(this->get_logger(), "Min distance: %d", mMinDistance);
        RCLCPP_INFO(this->get_logger(), "Max distance: %d", mMaxDistance);
    }

    void KittiLidarProjectorNode::LoadPointcloudBinary(const std::string &FilePath, cv::Mat &point_cloud,
                                                       sensor_msgs::msg::PointCloud2 &point_cloud2) {
        // Initialization
        int32_t num = 1000000; // maximum Number of points to allocate
        float *data = (float *) malloc(num * sizeof(float));
        float *px = data + 0;
        float *py = data + 1;
        float *pz = data + 2;
        float *pr = data + 3;

        // load point cloud from file
        std::FILE *stream;
        stream = fopen(FilePath.c_str(), "rb");

        // Save data to variable
        num = fread(data, sizeof(float), num, stream) / 4;

        // Clear Pointcloud variable
        point_cloud = cv::Mat::zeros(cv::Size(num, 4), CV_32F);

        sensor_msgs::msg::PointCloud2 msg_cloud_xyzi;
        CloudModifierXYZI modifier{ msg_cloud_xyzi, "xyzi" };
        modifier.reserve(num);

        // Format data as desired
        for (int32_t i = 0; i < num; i++) {
            point_cloud.at<float>(0, i) = (float) *px;
            point_cloud.at<float>(1, i) = (float) *py;
            point_cloud.at<float>(2, i) = (float) *pz;
            point_cloud.at<float>(3, i) = (float) 1;

            PointXYZI point;
            point.x = *px;
            point.y = *py;
            point.z = *pz;
            point.intensity = *pr;
            modifier.push_back(point);

            px += 4;
            py += 4;
            pz += 4;
            pr += 4;
        }
        // print point cloud data
         std::cout << modifier[12].x << std::endl;

        mPublisher->publish(msg_cloud_xyzi);

        // Close Stream and Free Memory
        fclose(stream);
        free(data);
    }

    void KittiLidarProjectorNode::LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenamesRGB,
                                             vector<string> &vstrPcdFilenames, vector<double> &vTimestamps) {
        cout << "Start Loading TimeStamps" << endl;
        ifstream fTimes;
        string strPathTimeFile = strPathToSequence + "times.txt";
        fTimes.open(strPathTimeFile);
        cout << strPathTimeFile << endl;
        while (!fTimes.eof()) {
            string s;
            getline(fTimes, s);
            if (!s.empty()) {
                stringstream ss;
                ss << s;
                double t;
                ss >> t;
                vTimestamps.push_back(t);
            }
        }

        cout << "Start Loading Images" << endl;
        string strPrefixRGB = strPathToSequence + "image_2/";
        string strPrefixPcd = strPathToSequence + "velodyne/";

        const int nTimes = vTimestamps.size();
        vstrImageFilenamesRGB.resize(nTimes);
        vstrPcdFilenames.resize(nTimes);

        for (int i = 0; i < nTimes; i++) {
            stringstream ss;
            ss << setfill('0') << setw(6) << i;
            vstrImageFilenamesRGB[i] = strPrefixRGB + ss.str() + ".png";
            vstrPcdFilenames[i] = strPrefixPcd + ss.str() + ".bin";
        }
    }

    void KittiLidarProjectorNode::createProjectionMatrix() {

        Eigen::MatrixXd cam_intrinsic_3x3(3, 3);
        cam_intrinsic_3x3 << mCameraIntrinsicMatrix[0], mCameraIntrinsicMatrix[1], mCameraIntrinsicMatrix[2],
                mCameraIntrinsicMatrix[3], mCameraIntrinsicMatrix[4], mCameraIntrinsicMatrix[5],
                mCameraIntrinsicMatrix[6], mCameraIntrinsicMatrix[7], mCameraIntrinsicMatrix[8];

        Eigen::MatrixXd lidar_to_cam_3x4(3, 4);
        lidar_to_cam_3x4 << mLidarCameraTransform[0], mLidarCameraTransform[1], mLidarCameraTransform[2],
                mLidarCameraTransform[3],
                mLidarCameraTransform[4], mLidarCameraTransform[5], mLidarCameraTransform[6],
                mLidarCameraTransform[7],
                mLidarCameraTransform[8], mLidarCameraTransform[9], mLidarCameraTransform[10],
                mLidarCameraTransform[11];

        Eigen::MatrixXd temp = cam_intrinsic_3x3 * lidar_to_cam_3x4;
        mProjectionMatrix.topLeftCorner(3, 4) = temp;

        std::cout << "Projection Matrix: " << std::endl;
        std::cout << mProjectionMatrix << std::endl;
    }

    void
    KittiLidarProjectorNode::visualizeProjection(cv::Mat &t_imRGB, sensor_msgs::msg::PointCloud2 &t_lidar_points,
                                                 std::vector<cv::KeyPoint> &t_key_points,
                                                 std::vector<ORB_SLAM3::MapPoint *> t_map_points) {
        cv::Mat imRGB = t_imRGB.clone();
        sensor_msgs::msg::PointCloud2 lidar_points = t_lidar_points;

        // VISUALIZE KEY POINTS
        for (auto &keypoint: t_key_points) {
            cv::circle(imRGB, keypoint.pt, 1, cv::Scalar(0, 255, 0), 1);
        }

        // VISUALIZE MAP POINTS
        for (auto i = 0; i < t_map_points.size(); i++) {
            if (t_map_points[i]) {
                cv::circle(imRGB, cv::Point(t_map_points[i]->mTrackProjX, t_map_points[i]->mTrackProjY), 2,
                           cv::Scalar(255, 0, 0), 2);
            }
        }

        // VISUALIZE LIDAR POINTS
        sensor_msgs::PointCloud2Iterator<float> iter_x(lidar_points, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(lidar_points, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(lidar_points, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_r(lidar_points, "intensity");

        for (int i = 0; i < lidar_points.width; i++) {

            Eigen::Vector4d point;
            point << *iter_x, *iter_y, *iter_z, 1;
            Eigen::Vector3d projected_point = mProjectionMatrix.topLeftCorner(3, 4) * point;
            projected_point(0) = projected_point(0) / projected_point(2);
            projected_point(1) = projected_point(1) / projected_point(2);

            if (projected_point(0) > 0 && projected_point(0) < imRGB.cols && projected_point(1) > 0 &&
                projected_point(1) < imRGB.rows) {

                if (projected_point(2) > mMinDistance && projected_point(2) < mMaxDistance) {
                    cv::circle(imRGB, cv::Point2f(projected_point(0), projected_point(1)), 0.5, cv::Scalar(0, 0, 255),
                               0.5);
                }
            }
            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++iter_r;
        }

        publishImage(imRGB);
    }

    void KittiLidarProjectorNode::publishPose(Sophus::SE3f pose, double t_frame_time) {
        Sophus::SE3f TWc = pose.inverse();

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.frame_id = "odom";
        pose_msg.header.stamp = rclcpp::Time(t_frame_time);

        pose_msg.pose.position.x = TWc.translation().x();
        pose_msg.pose.position.y = TWc.translation().y();
        pose_msg.pose.position.z = TWc.translation().z();

        pose_msg.pose.orientation.w = TWc.unit_quaternion().coeffs().w();
        pose_msg.pose.orientation.x = TWc.unit_quaternion().coeffs().x();
        pose_msg.pose.orientation.y = TWc.unit_quaternion().coeffs().y();
        pose_msg.pose.orientation.z = TWc.unit_quaternion().coeffs().z();

        mPosePublisher->publish(pose_msg);
    }

    void KittiLidarProjectorNode::publishTransform(Sophus::SE3f pose, double t_frame_time) {
        Sophus::SE3f TWc = pose.inverse();

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = rclcpp::Time(t_frame_time);
        t.header.frame_id = "map";
        t.child_frame_id = "odom";
        t.transform.translation.x = TWc.translation().x();
        t.transform.translation.y = TWc.translation().y();
        t.transform.translation.z = TWc.translation().z();
        t.transform.rotation.w = TWc.unit_quaternion().coeffs().w();
        t.transform.rotation.x = TWc.unit_quaternion().coeffs().x();
        t.transform.rotation.y = TWc.unit_quaternion().coeffs().y();
        t.transform.rotation.z = TWc.unit_quaternion().coeffs().z();

        mTransformBroadcaster->sendTransform(t);
    }

    void KittiLidarProjectorNode::publishImage(cv::Mat &t_image) {
        cv_bridge::CvImage cv_image;
        cv_image.image = t_image;
        cv_image.encoding = sensor_msgs::image_encodings::BGR8;
        mProjectedPublisher->publish(*cv_image.toImageMsg());
    }

    void
    KittiLidarProjectorNode::publishPointCloud(sensor_msgs::msg::PointCloud2 &t_lidar_points, double t_frame_time) {
        t_lidar_points.header.stamp = rclcpp::Time(t_frame_time);
        t_lidar_points.header.frame_id = "map";
        mPublisher->publish(t_lidar_points);
    }
}