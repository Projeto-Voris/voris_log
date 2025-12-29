#include "pose_register.hpp"

PoseSubscriberLog::PoseSubscriberLog() : Node("pose_sim_log") {

    slam_pose = std::make_shared<
            message_filters::Subscriber<geometry_msgs::msg::PoseStamped>>(this,
                                                                          "/orb_slam2_stereo_node/pose");
    gt_odom = std::make_shared<
            message_filters::Subscriber<geometry_msgs::msg::PoseArray>>(this,
                                                                        "/model/orca4/pose");

    // Initialize synchronizer
    syncApproximate = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::PoseArray>>>(
        message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::PoseArray>(
                    100), *slam_pose, *gt_odom);
    syncApproximate->registerCallback(&PoseSubscriberLog::callback, this);
    //, std::placeholders::_1, std::placeholders::_2));


    // Open the file
    pose_data.open("pose_sim_data.csv");
    if (!pose_data.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open file: pose_sim_data.csv");
    } else {
        // Write header
        pose_data
                << "gt_x, gt_y, gt_z, gt_q_w, gt_q_x, gt_q_y, gt_q_z, slam_x, slam_y, slam_z, slam_q_w, slam_q_x, slam_q_y, slam_q_z"
                << std::endl;
    }
}


PoseSubscriberLog::~PoseSubscriberLog() {
    if (pose_data.is_open()) {
        pose_data.close();
    }
}

void PoseSubscriberLog::callback(const geometry_msgs::msg::PoseStamped::SharedPtr &slam_msg,
                                 const geometry_msgs::msg::PoseArray::SharedPtr &gt_msg) {
    if (!pose_data.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write to file: file is not open.");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Saving data - %s / %s", gt_msg->header.frame_id.c_str(),
                slam_msg->header.frame_id.c_str());
    pose_data << gt_msg->poses[0].position.x << "," << gt_msg->poses[0].position.y << "," << gt_msg->poses[0].position.z
              << ","
              << gt_msg->poses[0].orientation.w << "," << gt_msg->poses[0].orientation.x << ","
              << gt_msg->poses[0].orientation.y
              << ","
              << gt_msg->poses[0].orientation.z << "," << slam_msg->pose.position.x << "," << slam_msg->pose.position.y
              << ","
              << slam_msg->pose.position.z << "," << slam_msg->pose.orientation.w << "," << slam_msg->pose.orientation.x
              << "," << slam_msg->pose.orientation.y << "," << slam_msg->pose.orientation.z << "," << std::endl;
}