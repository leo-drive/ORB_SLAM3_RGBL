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

        std::string dataset_path = "/home/bzeren/datasets/KITTI/deneme/00/";
        LoadImages(dataset_path, mvstrImageFilenamesRGB, mvstrPcdFilenames, mvTimestamps);
        RCLCPP_INFO(this->get_logger(), "Loaded %d images and %d pointclouds.", mvstrImageFilenamesRGB.size(),
                    mvstrPcdFilenames.size());

        cv::Mat imRGB, pcd;
        sensor_msgs::msg::PointCloud2 pc2msg;
        for (int ni = 0; ni < mvstrImageFilenamesRGB.size(); ni++) {

            imRGB = cv::imread(mvstrImageFilenamesRGB[ni], cv::IMREAD_UNCHANGED);

            LoadPointcloudBinary(mvstrPcdFilenames[ni], pcd, pc2msg);
            double tframe = mvTimestamps[ni];

            mSLAM.TrackRGBL(imRGB, pcd, tframe);

            pc2msg.header.stamp = rclcpp::Time(tframe);
            pc2msg.header.frame_id = "map";
            mPublisher->publish(pc2msg);

            auto temp = mSLAM.GetTrackedKeyPointsUn();
            visualizeProjection(imRGB, pc2msg, temp);

//            RCLCPP_INFO(this->get_logger(), "Tracking %d/%d", ni, mvstrImageFilenamesRGB.size());
        }
    }

    KittiLidarProjectorNode::~KittiLidarProjectorNode() {
        RCLCPP_INFO(this->get_logger(), "KittiLidarProjectorNode has been destroyed.");
    }

    void KittiLidarProjectorNode::readParams() {
        this->declare_parameter("camera.intrinsic_matrix.data");
        this->declare_parameter("camera.distortion_coefficients.data");
        this->declare_parameter("camera.height");
        this->declare_parameter("camera.width");
        this->declare_parameter("lidar_camera_transform.data");
        this->declare_parameter("projection.min_dist");
        this->declare_parameter("projection.max_dist");

        this->get_parameter("camera.intrinsic_matrix.data", mCameraIntrinsicMatrix);
        this->get_parameter("camera.distortion_coefficients.data", mCameraDistortionCoefficients);
        this->get_parameter("camera.height", mCameraHeight);
        this->get_parameter("camera.width", mCameraWidth);
        this->get_parameter("lidar_camera_transform.data", mLidarCameraTransform);
        this->get_parameter("projection.min_dist", mMinDistance);
        this->get_parameter("projection.max_dist", mMaxDistance);

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

        sensor_msgs::PointCloud2Modifier modifier(point_cloud2);
        modifier.resize(num);
        modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
        sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud2, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud2, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud2, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_r(point_cloud2, "intensity");

        // Format data as desired
        for (int32_t i = 0; i < num; i++) {
            point_cloud.at<float>(0, i) = (float) *px;
            point_cloud.at<float>(1, i) = (float) *py;
            point_cloud.at<float>(2, i) = (float) *pz;
            point_cloud.at<float>(3, i) = (float) 1;

            *iter_x = (float) *px;
            *iter_y = (float) *py;
            *iter_z = (float) *pz;
            *iter_r = (float) *pr;

            px += 4;
            py += 4;
            pz += 4;
            pr += 4;
            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++iter_r;
        }

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
                                                 std::vector<cv::KeyPoint> &t_tracked_points) {
        cv::Mat imRGB = t_imRGB.clone();
        sensor_msgs::msg::PointCloud2 lidar_points = t_lidar_points;

        // Draw keypoints
        for (auto &keypoint: t_tracked_points) {
            cv::circle(imRGB, keypoint.pt, 2, cv::Scalar(0, 255, 0), 2);
        }

        // Project point cloud and draw points
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

        // publish image
        cv_bridge::CvImage cv_image;
        cv_image.image = imRGB;
        cv_image.encoding = sensor_msgs::image_encodings::BGR8;
        mProjectedPublisher->publish(*cv_image.toImageMsg());
    }
}