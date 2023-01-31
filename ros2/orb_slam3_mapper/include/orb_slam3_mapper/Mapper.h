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

    template<typename T>
    bool rel_eq(const T &a, const T &b) {
        static_assert(
                std::is_floating_point<T>::value, "Float comparisons only support floating point types.");

        const auto delta = std::abs(a - b);
        const auto larger = std::max(std::abs(a), std::abs(b));
        const auto max_rel_delta = (larger * std::numeric_limits<T>::epsilon());
        return delta <= max_rel_delta;
    }

    class Mapper : public rclcpp::Node {
    public:
        struct PointXYZI {
            float x;
            float y;
            float z;
            float intensity;

            friend bool operator==(const PointXYZI &p1, const PointXYZI &p2) noexcept {
                return rel_eq(p1.x, p2.x) && rel_eq(p1.y, p2.y) && rel_eq(p1.z, p2.z) &&
                       rel_eq(p1.intensity, p2.intensity);
            }
        };

        using CloudModifierXYZI = point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZI>;

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

        void pointCloudVisualizer(sensor_msgs::msg::PointCloud2 &point_cloud2, vector<ORB_SLAM3::MapPoint *> mapPoints, Sophus::SE3f curr_pose);

    private:
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mCurrCameraMarkerPublisher{};
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mKeyFramesMarkerPublisher{};
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mKeyFramesGTMarkerPublisher{};
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mPosesMarkerPublisher{};

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mPublisher1{};
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mPublisher2{};

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

        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
        geometry_msgs::msg::TransformStamped trans_;

        std::unique_ptr<tf2_ros::TransformBroadcaster> mTransformBroadcaster;
    };

}

#endif //ORB_SLAM3_KITTILIDARPROJECTOR_H
