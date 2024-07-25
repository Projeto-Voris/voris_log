//
// Created by daniel on 24/07/24.
//

#include "pose_register.hpp"

int main(int argc, char **argv){

    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseSubscriberLog>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}