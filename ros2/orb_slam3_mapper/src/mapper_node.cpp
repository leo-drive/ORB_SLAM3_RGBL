//
// Created by bzeren on 08.12.2022.
//

#include "orb_slam3_mapper/Mapper.h"

int main(int argc, char **argv)
{
    if(argc < 4)
    {
        std::cerr << "\nUsage: ros2 launch kitti_lidar_projector mapper.launch.py" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mapper::Mapper>(argc, argv));
    rclcpp::shutdown();
    return 0;
}