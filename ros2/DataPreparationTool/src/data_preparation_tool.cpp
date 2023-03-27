//
// Created by bzeren on 27.03.2023.
//

#include "data_preparation_tool.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BagReader>());
    rclcpp::shutdown();
    return 0;
}