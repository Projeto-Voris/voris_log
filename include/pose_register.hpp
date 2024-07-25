//
// Created by daniel on 24/07/24.
//

#ifndef VORIS_LOG_POSE_REGISTER_HPP
#define VORIS_LOG_POSE_REGISTER_HPP


#include "fstream"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "geometry_msgs/msg/pose_array.hpp"


class PoseSubscriberLog : public rclcpp::Node {
public:
    PoseSubscriberLog();

    ~PoseSubscriberLog();

    void callback(const geometry_msgs::msg::PoseStamped::SharedPtr &slam_msg,
                  const geometry_msgs::msg::PoseArray::SharedPtr &gt_msg);

private:

    std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>> slam_pose;
    std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PoseArray>> gt_odom;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::PoseArray>>> syncApproximate;

    std::ofstream pose_data;
};

#endif //VORIS_LOG_POSE_REGISTER_HPP
