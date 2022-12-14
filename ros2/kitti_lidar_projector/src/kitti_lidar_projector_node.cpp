//
// Created by bzeren on 08.12.2022.
//

#include "kitti_lidar_projector/KittiLidarProjector.h"

int main(int argc, char **argv)
{
    if(argc < 4)
    {
        std::cerr << "\nUsage: ros2 launch kitti_lidar_projector kitti_lidar_projector.launch.py" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<kitti_lidar_projector::KittiLidarProjectorNode>(argc, argv));
    rclcpp::shutdown();
    return 0;
}