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

        std::string dataset_path = mPathToSequence.c_str();
        LoadImages(dataset_path, mvstrImageFilenamesRGB, mvstrPcdFilenames, mvTimestamps);
        LoadGroundTruth(dataset_path, mvPosesGT);
        RCLCPP_INFO(this->get_logger(), "Loaded %d images, %d pointclouds and %d ground truths.",
                    mvstrImageFilenamesRGB.size(),
                    mvstrPcdFilenames.size(), mvPosesGT.size());

        mMap = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);

        int localization_counter = 0;

        cv::Mat imRGB, pcd;
        sensor_msgs::msg::PointCloud2 pc2msg;
        for (int ni = 0; ni < mvstrImageFilenamesRGB.size(); ni++) {
            imRGB = cv::imread(mvstrImageFilenamesRGB[ni], cv::IMREAD_UNCHANGED);
            LoadPointcloudBinary(mvstrPcdFilenames[ni], pcd, pc2msg);

            double tframe = mvTimestamps[ni];

            Sophus::SE3f Tcw = mSLAM.TrackRGBL(imRGB, pcd, tframe);

            // TODO: Ground truth map.
//            setMapPointsPose(mSLAM.GetTrackedMapPoints(), mvPosesGT[ni], Tcw);

            // Add map points to map.
            auto pose = mvPosesGT[ni];
            auto map_points = mSLAM.GetTrackedMapPoints();
            addFrameToMap(map_points, pose, Tcw);

            // Activate localization mode.
            if (localization_counter == 30) {
                mSLAM.ActivateLocalizationMode();
            }
            localization_counter++;
        }
        mSLAM.SaveTrajectoryKITTI("KITTI.txt");
        RCLCPP_INFO(this->get_logger(), "SLAM Shutdown");

        auto map_points = mSLAM.GetTrackedMapPoints();
        std::cout << "Map points size: " << map_points.size() << std::endl;

        pcl::io::savePCDFileASCII("deneme.pcd", *mMap);

        mSLAM.Shutdown();
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