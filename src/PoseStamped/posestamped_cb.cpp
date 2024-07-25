//
// Created by daniel on 25/07/24.
//

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <fstream>

class PoseStampedLog : public rclcpp::Node {

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
    std::ofstream pose_data;
public:
    PoseStampedLog() : Node("posestamped_log") {
        // Open the file
        pose_data.open("vision_pose_sim.csv");
        if (!pose_data.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: vision_pose_sim.csv");
        } else {
            // Write header
            pose_data << "pose_x, pose_y, pose_z, pose_q_w, pose_q_x, pose_q_y, pose_q_z" << std::endl;
        }

        (void) pose_subscription_;

        pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
                "/mavros/vision_pose/pose", 10, [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
                    if (!pose_data.is_open()) {
                        RCLCPP_ERROR(this->get_logger(), "Failed to write to file: file is not open.");
                        return;
                    }


                    // Write data
                    pose_data << msg->pose.position.x << "," << msg->pose.position.y << "," << msg->pose.position.z
                              << ","
                              << msg->pose.orientation.w << "," << msg->pose.orientation.x << ","
                              << msg->pose.orientation.y << ","
                              << msg->pose.orientation.z << std::endl;
                });
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseStampedLog>());
    rclcpp::shutdown();
    return 0;
}
