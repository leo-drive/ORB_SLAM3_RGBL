//
// Created by bzeren on 27.03.2023.
//

#include "DataPreparationTool.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DataPreparationTool>());
    rclcpp::shutdown();
    return 0;
}