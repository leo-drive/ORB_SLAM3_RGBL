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

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include "System.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/PointField.h>
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "cv_bridge/cv_bridge.h"

#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>

#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>

typedef pcl::PointXYZI PointT;

namespace mapper {

    class Mapper : public rclcpp::Node {
    public:
        Mapper(int argc, char **argv);

        ~Mapper();

        void readParams();

        void LoadGroundTruth(std::string strPathToSequence, std::vector<Sophus::SE3f> &vPosesGT);

        void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenamesRGB,
                        vector<string> &vstrPcdFilenames, vector<double> &vTimestamps);

        void LoadPointcloudBinary(const std::string &FilePath, cv::Mat &point_cloud,
                                  sensor_msgs::msg::PointCloud2 &point_cloud2);

        void addFrameToMap(vector<ORB_SLAM3::MapPoint *> vpMapPoints, Sophus::SE3f &pose, Sophus::SE3f &Tcw);

        void setMapPointsPose(vector<ORB_SLAM3::MapPoint *> vpMapPoints, Sophus::SE3f &pose, Sophus::SE3f &Tcw);

        void visualizer(const Sophus::SE3f &Tcw, Sophus::SE3f &poseGT);

    private:
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mCurrCameraMarkerPublisher{};
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mKeyFramesMarkerPublisher{};
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mKeyFramesGTMarkerPublisher{};
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mPosesMarkerPublisher{};

        ORB_SLAM3::System mSLAM;

        vector<std::string> mvstrImageFilenamesRGB;
        vector<std::string> mvstrPcdFilenames;
        vector<double> mvTimestamps;
        vector<Sophus::SE3f> mvPosesGT;

        std::string mPathToVocabulary;
        std::string mPathToSettings;
        std::string mPathToSequence;

        pcl::PointCloud<PointT>::Ptr mMap;

        // Visualizer
        vector<Sophus::SE3f> mvKeyFrames;
        vector<Sophus::SE3f> mvKeyFramesGT;
    };

}

#endif //ORB_SLAM3_KITTILIDARPROJECTOR_H
