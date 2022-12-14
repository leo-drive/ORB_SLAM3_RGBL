//
// Created by bzeren on 08.12.2022.
//

#ifndef ORB_SLAM3_KITTILIDARPROJECTOR_H
#define ORB_SLAM3_KITTILIDARPROJECTOR_H

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"

#include "System.h"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "cv_bridge/cv_bridge.h"

namespace kitti_lidar_projector {

    class KittiLidarProjectorNode : public rclcpp::Node {
    public:
        KittiLidarProjectorNode(int argc, char **argv);

        ~KittiLidarProjectorNode();

        void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenamesRGB, vector<string> &vstrPcdFilenames, vector<double> &vTimestamps);
        void LoadPointcloudBinary(const std::string& FilePath, cv::Mat& point_cloud, sensor_msgs::msg::PointCloud2 &point_cloud2);
        void readParams();
        void createProjectionMatrix();
        void visualizeProjection(cv::Mat &t_imRGB, sensor_msgs::msg::PointCloud2 &t_lidar_points, std::vector<cv::KeyPoint> &t_tracked_points);

    private:
        ORB_SLAM3::System mSLAM;

        vector<std::string> mvstrImageFilenamesRGB;
        vector<std::string> mvstrPcdFilenames;
        vector<double> mvTimestamps;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mPublisher;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mProjectedPublisher;

        int mCameraHeight;
        int mCameraWidth;

        double mMinDistance;
        double mMaxDistance;

        vector<double> mCameraIntrinsicMatrix;
        vector<double> mCameraDistortionCoefficients;
        vector<double> mLidarCameraTransform;

        Eigen::Matrix4d mProjectionMatrix;
    };

}

#endif //ORB_SLAM3_KITTILIDARPROJECTOR_H
