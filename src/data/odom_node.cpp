#include <memory>
#include <fstream>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class OdomSaver : public rclcpp::Node
{
public:
  OdomSaver()
  : Node("odom_saver")
  {
    // Declare parameter for file path
    this->declare_parameter<std::string>("file_path", "odom_data.csv");
    this->get_parameter("file_path", file_path_);

    // Initialize CSV file with headers
    std::ofstream csv_file;
    csv_file.open(file_path_, std::ios::out | std::ios::trunc);
    if (csv_file.is_open()) {
      csv_file << "timestamp,p_x,p_y,p_z,o_w,o_x,o_y,o_z\n";
      csv_file.close();
      RCLCPP_INFO(this->get_logger(), "Initialized CSV file at: %s", file_path_.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", file_path_.c_str());
    }

    // Create subscription
    // Topic: "odom" (can be remapped)
    // Queue size: 10
    auto qos_profile = rclcpp::SensorDataQoS();
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", qos_profile, std::bind(&OdomSaver::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const
  {
    std::ofstream csv_file;
    // Open in append mode
    csv_file.open(file_path_, std::ios::out | std::ios::app);

    if (csv_file.is_open()) {
      // Timestamp in seconds.nanoseconds format
      csv_file << msg->header.stamp.sec << "." << msg->header.stamp.nanosec << ","
               << msg->pose.pose.position.x << ","
               << msg->pose.pose.position.y << ","
               << msg->pose.pose.position.z << ","
               << msg->pose.pose.orientation.w << ","
               << msg->pose.pose.orientation.x << ","
               << msg->pose.pose.orientation.y << ","
               << msg->pose.pose.orientation.z << "\n";
      csv_file.close();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unable to write to file.");
    }
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  std::string file_path_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomSaver>());
  rclcpp::shutdown();
  return 0;
}