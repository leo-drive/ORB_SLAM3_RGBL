//
// Created by bzeren on 08.12.2022.
//

#include "orb_slam3_mapper/Mapper.h"

namespace mapper {

    Mapper::Mapper(int argc, char **argv) : Node("kitti_lidar_projector"),
                                            mSLAM("/home/bzeren/leo/ORB_SLAM3_RGBL/Vocabulary/ORBvoc.txt",
                                                  "/home/bzeren/leo/ORB_SLAM3_RGBL/Examples/RGB-L/KITTI00-02.yaml",
                                                  ORB_SLAM3::System::RGBL, true) {
        RCLCPP_INFO(this->get_logger(), "KittiLidarProjectorNode has been initialized.");

        readParams();

        mCurrCameraMarkerPublisher = this->create_publisher<visualization_msgs::msg::Marker>(
                "/orb_slam3_mapper/camera_marker", 1);
        mKeyFramesMarkerPublisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "/orb_slam3_mapper/keyframes_marker", 1);
        mKeyFramesGTMarkerPublisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "/orb_slam3_mapper/keyframes_gt_marker", 1);
        mPosesMarkerPublisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "/orb_slam3_mapper/poses_marker", 1);

        mPublisher1 = this->create_publisher<sensor_msgs::msg::PointCloud2>("orb_slam3_mapper/point_cloud1", 10);
        mPublisher2 = this->create_publisher<sensor_msgs::msg::PointCloud2>("orb_slam3_mapper/point_cloud2", 10);

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
        LoadGroundTruth(dataset_path, mvPosesGT);
        RCLCPP_INFO(this->get_logger(), "Loaded %d images, %d pointclouds and %d ground truths.",
                    mvstrImageFilenamesRGB.size(),
                    mvstrPcdFilenames.size(), mvPosesGT.size());

        mMap = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);

        int localization_counter = 0;

        vector<Sophus::SE3f> poses;

        cv::Mat imRGB, pcd;
        sensor_msgs::msg::PointCloud2 pc2msg;
        for (int ni = 0; ni < mvstrImageFilenamesRGB.size(); ni++) {
            imRGB = cv::imread(mvstrImageFilenamesRGB[ni], cv::IMREAD_UNCHANGED);
            LoadPointcloudBinary(mvstrPcdFilenames[ni], pcd, pc2msg);

            double tframe = mvTimestamps[ni];

            Sophus::SE3f Tcw = mSLAM.TrackRGBL(imRGB, pcd, tframe, mvPosesGT[ni]);
            poses.push_back(Tcw);
//            cout << "All MapPoints: " << mSLAM.GetAtlas()->GetAllMapPoints().size() << endl;

            // Add map points to map.
            auto pose = mvPosesGT[ni];
            auto map_points = mSLAM.GetTrackedMapPoints();
            addFrameToMap(map_points, pose, Tcw);

            // TODO: Ground truth map.
//            setMapPointsPose(mSLAM.GetTrackedMapPoints(), mvPosesGT[ni], Tcw);

            // TODO: Visualize pangolin things with rviz.
            visualizer(Tcw, pose);
            pointCloudVisualizer(pc2msg, mSLAM.GetCurrentMapPoints(), Tcw);
//             Activate localization mode.
//            if (localization_counter == 50) {
//                mSLAM.ActivateLocalizationMode();
//            }
//            localization_counter++;
        }

        // write poses to file
        std::ofstream f;
        f.open("LastPoses.txt");
        for (auto pose : poses) {
            Eigen::Matrix4f m = pose.matrix().inverse();
            f << m(0,0) << " " << m(0,1) << " " << m(0,2) << " " << m(0,3) << " " << m(1, 0) << " " << m(1, 1) << " " << m(1, 2) << " " << m(1, 3) << " " << m(2, 0) << " " << m(2, 1) << " " << m(2, 2) << " " << m(2, 3) << std::endl;
        }

        RCLCPP_INFO(this->get_logger(), "SLAM Shutdown");

        auto map_points = mSLAM.GetTrackedMapPoints();
        std::cout << "Map points size: " << map_points.size() << std::endl;

        pcl::io::savePCDFileASCII("deneme.pcd", *mMap);

        mSLAM.Shutdown();
        mSLAM.SaveTrajectoryKITTI("KITTI.txt");
    }

    Mapper::~Mapper() {
        RCLCPP_INFO(this->get_logger(), "KittiLidarProjectorNode has been destroyed.");
    }

    void Mapper::readParams() {
        this->declare_parameter("voc_path");
        this->declare_parameter("settings_path");
        this->declare_parameter("sequence_path");

        this->get_parameter("voc_path", mPathToVocabulary);
        this->get_parameter("settings_path", mPathToSettings);
        this->get_parameter("sequence_path", mPathToSequence);

        RCLCPP_INFO(this->get_logger(), "Vocabulary path: %s", mPathToVocabulary.c_str());
        RCLCPP_INFO(this->get_logger(), "Settings path: %s", mPathToSettings.c_str());
        RCLCPP_INFO(this->get_logger(), "Sequence path: %s", mPathToSequence.c_str());
    }

    void Mapper::LoadPointcloudBinary(const std::string &FilePath, cv::Mat &point_cloud,
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
            point.intensity = 0.5;
            modifier.push_back(point);

            px += 4;
            py += 4;
            pz += 4;
            pr += 4;
        }
        mPublisher1->publish(msg_cloud_xyzi);

        // Close Stream and Free Memory
        fclose(stream);
        free(data);
    }

    void Mapper::LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenamesRGB,
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


        // Half-Sequence
//        const int nTimes = vTimestamps.size();
//        vstrImageFilenamesRGB.resize(nTimes);
//        vstrPcdFilenames.resize(nTimes);
//
//        for (int i = 1001; i < nTimes+1001; i++) {
//            stringstream ss;
//            ss << setfill('0') << setw(6) << i;
//            vstrImageFilenamesRGB[i-1001] = strPrefixRGB + ss.str() + ".png";
//            vstrPcdFilenames[i-1001] = strPrefixPcd + ss.str() + ".bin";
//        }

        // Full-Sequence
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

    void Mapper::LoadGroundTruth(std::string strPathToSequence, std::vector<Sophus::SE3f> &vPosesGT) {
        std::ifstream fPoses;
        std::string strPathPoseFile = strPathToSequence + "poses.txt";
        fPoses.open(strPathPoseFile);
        cout << strPathPoseFile << endl;
        while (!fPoses.eof()) {
            std::string s;
            getline(fPoses, s);
            if (!s.empty()) {
                std::stringstream ss;
                ss << s;
                float data[12];
                for (int i = 0; i < 12; i++) {
                    ss >> data[i];
                }
                Eigen::Matrix4f T;
                T << data[0], data[1], data[2], data[3],
                        data[4], data[5], data[6], data[7],
                        data[8], data[9], data[10], data[11],
                        0, 0, 0, 1;
                Sophus::SE3f pose(T);
                vPosesGT.push_back(pose);
            }
        }
    }

    void Mapper::addFrameToMap(vector<ORB_SLAM3::MapPoint *> vpMapPoints, Sophus::SE3f &pose,
                               Sophus::SE3f &Tcw) {

        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

        for (auto i = 0; i < vpMapPoints.size(); i++) {
            if (vpMapPoints[i]) {

                PointT p;
                p.x = vpMapPoints[i]->GetWorldPos()(0, 0);
                p.y = vpMapPoints[i]->GetWorldPos()(1, 0);
                p.z = vpMapPoints[i]->GetWorldPos()(2, 0);

                p.intensity = vpMapPoints[i]->Observations();
                cloud->points.push_back(p);
            }
        }
        Eigen::Matrix4f tcw = Tcw.matrix();
        Eigen::Matrix4f GT = pose.matrix();
        pcl::transformPointCloud(*cloud, *cloud, tcw);
        pcl::transformPointCloud(*cloud, *cloud, GT);

        *mMap += *cloud;
    }

    void Mapper::visualizer(const Sophus::SE3f &Tcw, Sophus::SE3f &poseGT) {

        // Aktif frame görselleştirme (yeşil) --  Ros marker ile  +++
        // Bütün keyframeler görselleştirme  (kırmızı) --  Ros marker array ile  +++
        // GT poselarını görselleştirme (mavi)  --  Ros marker array ile  +++
        // Bütün 3d noktaları görselleştirme  --  Nav  path ile
        // Aktif 3d noktaları görselleştirme  --  point cloud ile

        // Visualize active frame
        Eigen::Matrix4f tcw = Tcw.inverse().matrix();
        geometry_msgs::msg::Pose pose;
        pose.position.x = tcw(0, 3);
        pose.position.y = tcw(1, 3);
        pose.position.z = tcw(2, 3);
        Eigen::Quaternionf q(tcw.block<3, 3>(0, 0));
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        std_msgs::msg::ColorRGBA color;
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;
        color.a = 1.0;
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = rclcpp::Clock().now();
        marker.ns = "active_frame";
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.id = 0;
        marker.pose = pose;
        marker.color = color;
        marker.scale.x = 4.0;
        marker.scale.y = 4.0;
        marker.scale.z = 4.0;
        mCurrCameraMarkerPublisher->publish(marker);

        // Visualize current poses
        visualization_msgs::msg::MarkerArray markerArrayPoses;
        mvKeyFrames.push_back(Tcw);
        for (int i = 0; i < mvKeyFrames.size(); i++) {
            Eigen::Matrix4f tcw = mvKeyFrames[i].inverse().matrix();
            geometry_msgs::msg::Pose pose;
            pose.position.x = tcw(0, 3);
            pose.position.y = tcw(1, 3);
            pose.position.z = tcw(2, 3);
            Eigen::Quaternionf q(tcw.block<3, 3>(0, 0));
            pose.orientation.x = q.x();
            pose.orientation.y = q.y();
            pose.orientation.z = q.z();
            pose.orientation.w = q.w();
            std_msgs::msg::ColorRGBA color;
            color.r = 0.0;
            color.g = 1.0;
            color.b = 1.0;
            color.a = 1.0;
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = rclcpp::Clock().now();
            marker.ns = "keyframes";
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.id = i;
            marker.pose = pose;
            marker.color = color;
            marker.scale.x = 0.4;
            marker.scale.y = 0.4;
            marker.scale.z = 0.4;
            markerArrayPoses.markers.push_back(marker);
        }
        mPosesMarkerPublisher->publish(markerArrayPoses);


        // Visualize keyframes
        visualization_msgs::msg::MarkerArray markerArray;
        vector<ORB_SLAM3::KeyFrame*> vpKFs = mSLAM.GetAtlas()->GetCurrentMap()->GetAllKeyFrames();
        for (auto i = 0; i < vpKFs.size(); i++) {
            Eigen::Matrix4f Tcw = vpKFs[i]->GetPose().inverse().matrix();
            geometry_msgs::msg::Pose pose;
            pose.position.x = Tcw(0, 3);
            pose.position.y = Tcw(1, 3);
            pose.position.z = Tcw(2, 3);
            Eigen::Quaternionf q(Tcw.block<3, 3>(0, 0));
            pose.orientation.x = q.x();
            pose.orientation.y = q.y();
            pose.orientation.z = q.z();
            pose.orientation.w = q.w();
            std_msgs::msg::ColorRGBA color;
            color.r = 1.0;
            color.g = 0.0;
            color.b = 0.0;
            color.a = 1.0;

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = rclcpp::Clock().now();
            marker.ns = "current_poses";
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.id = i;
            marker.pose = pose;
            marker.color = color;
            marker.scale.x = 0.4;
            marker.scale.y = 0.4;
            marker.scale.z = 0.4;
            markerArray.markers.push_back(marker);
        }
        mKeyFramesMarkerPublisher->publish(markerArray);

        // Visualize GT poses
        visualization_msgs::msg::MarkerArray markerArrayGT;
        mvKeyFramesGT.push_back(poseGT);
        for (auto i = 0; i < mvKeyFramesGT.size(); i++) {
            Eigen::Matrix4f Tcw = mvKeyFramesGT[i].matrix();
            geometry_msgs::msg::Pose pose;
            pose.position.x = Tcw(0, 3);
            pose.position.y = Tcw(1, 3);
            pose.position.z = Tcw(2, 3);
            Eigen::Quaternionf q(Tcw.block<3, 3>(0, 0));
            pose.orientation.x = q.x();
            pose.orientation.y = q.y();
            pose.orientation.z = q.z();
            pose.orientation.w = q.w();
            std_msgs::msg::ColorRGBA color;
            color.r = 0.0;
            color.g = 0.0;
            color.b = 1.0;
            color.a = 1.0;
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = rclcpp::Clock().now();
            marker.ns = "gt_poses";
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.id = i;
            marker.pose = pose;
            marker.color = color;
            marker.scale.x = 0.4;
            marker.scale.y = 0.4;
            marker.scale.z = 0.4;
            markerArrayGT.markers.push_back(marker);
        }
        mKeyFramesGTMarkerPublisher->publish(markerArrayGT);

        // Visualize all 3d points

    }

    void Mapper::pointCloudVisualizer(sensor_msgs::msg::PointCloud2 &point_cloud2, vector<ORB_SLAM3::MapPoint *> mapPoints, Sophus::SE3f curr_pose)
    {
        sensor_msgs::msg::PointCloud2 point_cloud;

        sensor_msgs::msg::PointCloud2 msg_cloud_xyzi;
        CloudModifierXYZI modifier{ msg_cloud_xyzi, "xyzi" };
        modifier.reserve(1000000);

        Eigen::Matrix4f lidar_to_camera = Eigen::Matrix4f::Identity();
        lidar_to_camera(0, 0) = 4.276802385584e-04;
        lidar_to_camera(0, 1) = -9.999672484946e-01;
        lidar_to_camera(0, 2) = -8.084491683471e-03;
        lidar_to_camera(0, 3) = -1.198459927713e-02;
        lidar_to_camera(1, 0) = -7.210626507497e-03;
        lidar_to_camera(1, 1) = 8.081198471645e-03;
        lidar_to_camera(1, 2) = -9.999413164504e-01;
        lidar_to_camera(1, 3) = -5.403984729748e-02;
        lidar_to_camera(2, 0) = 9.999738645903e-01;
        lidar_to_camera(2, 1) = 4.859485810390e-04;
        lidar_to_camera(2, 2) = -7.206933692422e-03;
        lidar_to_camera(2, 3) = -2.921968648686e-01;
        lidar_to_camera = lidar_to_camera.inverse();

        Eigen::Matrix4f pose = curr_pose.matrix();
        for (auto pMP : mapPoints) {
            if (pMP)
            {
                Eigen::Vector4f p;
                p(0, 0) = pMP->GetWorldPos()(0, 0);
                p(1, 0) = pMP->GetWorldPos()(1, 0);
                p(2, 0) = pMP->GetWorldPos()(2, 0);
                p(3, 0) = 1.0f;

                Eigen::Vector4f p2 = pose * p;
                Eigen::Vector4f p3 = lidar_to_camera * p2;

                PointXYZI point;
                point.x = p3(0, 0);
                point.y = p3(1, 0);
                point.z = p3(2, 0);
                point.intensity = 1.0;
                modifier.push_back(point);

//                PointXYZI point;
//                point.x = pMP->GetWorldPos()(0, 0);
//                point.y = pMP->GetWorldPos()(1, 0);
//                point.z = pMP->GetWorldPos()(2, 0);
//                point.intensity = 1.0;
//                modifier.push_back(point);
            }
        }
        if (msg_cloud_xyzi.width != 0)
            mPublisher2->publish(msg_cloud_xyzi);

    }

    void Mapper::setMapPointsPose(vector<ORB_SLAM3::MapPoint *> vpMapPoints, Sophus::SE3f &pose,
                                  Sophus::SE3f &Tcw) {

//        for (auto i = 0; i < vpMapPoints.size(); i++) {
//            if (vpMapPoints[i]) {
//                Eigen::Vector3f p;
//                p(0, 0) = vpMapPoints[i]->GetWorldPos()(0, 0);
//                p(1, 0) = vpMapPoints[i]->GetWorldPos()(1, 0);
//                p(2, 0) = vpMapPoints[i]->GetWorldPos()(2, 0);
//
//                Eigen::Vector3f pGT = Tcw * p;
//                Eigen::Vector3f pSLAM = pose * pGT;
//
//                vpMapPoints[i]->SetWorldPos(pSLAM);
//            }
//        }

//        ORB_SLAM3::Atlas *mpAtlas = mSLAM.GetAtlas();
//        ORB_SLAM3::Map *mpMap = mpAtlas->GetCurrentMap();
//
//        vector<ORB_SLAM3::MapPoint *> vpMapPointsMap = mpMap->GetAllMapPoints();
//        vector<ORB_SLAM3::MapPoint *> vpRefMPs = mpMap->GetReferenceMapPoints();
//        vector<ORB_SLAM3::KeyFrame *> vpKeyFrames = mpMap->GetAllKeyFrames();

//        for (auto i = counter_keyframes; i < vpKeyFrames.size(); i++) {
//            if (vpKeyFrames[i]) {
//                vpKeyFrames[i]->SetPose(pose);
//
//                counter_keyframes++;
//            }
//        }

//        for (auto i = counter_map_points; i < vpMapPointsMap.size(); i++) {
//            if (vpMapPointsMap[i]) {
//                Eigen::Vector3f p;
//                p(0, 0) = vpMapPointsMap[i]->GetWorldPos()(0, 0);
//                p(1, 0) = vpMapPointsMap[i]->GetWorldPos()(1, 0);
//                p(2, 0) = vpMapPointsMap[i]->GetWorldPos()(2, 0);
//
//                Eigen::Vector3f pGT = Tcw * p;
//                Eigen::Vector3f pSLAM = pose * pGT;
//
//                vpMapPointsMap[i]->SetWorldPos(pSLAM);
//
//                counter_map_points++;
//            }
//        }

    }
}