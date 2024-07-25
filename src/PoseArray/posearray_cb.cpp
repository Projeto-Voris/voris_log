//
// Created by daniel on 25/07/24.
//

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <memory>
#include <fstream>

class PoseArrayLog : public rclcpp::Node {

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_subscription_;
    std::ofstream pose_data;
public:
    PoseArrayLog() : Node("posearray_log") {
        // Open the file
        pose_data.open("gt_pose_log.csv");
        if (!pose_data.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: vision_pose_sim.csv");
        } else {
            // Write header
            pose_data << "pose_x, pose_y, pose_z, pose_q_w, pose_q_x, pose_q_y, pose_q_z" << std::endl;
        }

        (void) pose_subscription_;

        pose_subscription_ = create_subscription<geometry_msgs::msg::PoseArray>(
                "/model/orca4/pose", 10, [this](const geometry_msgs::msg::PoseArray::ConstSharedPtr msg) {
                    if (!pose_data.is_open()) {
                        RCLCPP_ERROR(this->get_logger(), "Failed to write to file: file is not open.");
                        return;
                    }
                    // Write data
                    pose_data << msg->poses[0].position.x << "," << msg->poses[0].position.y << "," << msg->poses[0].position.z
                              << ","
                              << msg->poses[0].orientation.w << "," << msg->poses[0].orientation.x << ","
                              << msg->poses[0].orientation.y << ","
                              << msg->poses[0].orientation.z << std::endl;
                });
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseArrayLog>());
    rclcpp::shutdown();
    return 0;
}
