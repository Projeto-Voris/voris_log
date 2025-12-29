#include <memory>
#include <fstream>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class PoseSaver : public rclcpp::Node
{
public:
  PoseSaver()
  : Node("pose_saver")
  {
    // Declare parameter for file path
    this->declare_parameter<std::string>("file_path", "pose_data.csv");
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
    // Topic: "pose" (can be remapped)
    // Message Type: PoseStamped (contains header for timestamp)
    auto qos_profile = rclcpp::SensorDataQoS();

    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "pose", qos_profile, std::bind(&PoseSaver::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const
  {
    std::ofstream csv_file;
    // Open in append mode
    csv_file.open(file_path_, std::ios::out | std::ios::app);

    if (csv_file.is_open()) {
      // Timestamp in seconds.nanoseconds format
      csv_file << msg->header.stamp.sec << "." << msg->header.stamp.nanosec << ","
               << msg->pose.position.x << ","
               << msg->pose.position.y << ","
               << msg->pose.position.z << ","
               << msg->pose.orientation.w << ","
               << msg->pose.orientation.x << ","
               << msg->pose.orientation.y << ","
               << msg->pose.orientation.z << "\n";
      csv_file.close();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unable to write to file.");
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  std::string file_path_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseSaver>());
  rclcpp::shutdown();
  return 0;
}