#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

namespace depth_log
{

class DepthNode : public rclcpp::Node
{
public:
  explicit DepthNode(const rclcpp::NodeOptions & options) : rclcpp::Node("depth_msg", options)
  
  {
    depth_publisher = this->create_publisher<geometry_msgs::msg::PointStamped>(
      "/depth", 10);

    subscriber_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
      "/mavros/imu/static_pressure", 10,
      std::bind(&DepthNode::on_message_received, this, std::placeholders::_1));
  }

private:
  void on_message_received(const sensor_msgs::msg::FluidPressure::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Pressão do fluído recebida. Convertendo para Altitude...");

    /*=========================== PUBLICANDO DEPTH MSG =================================*/

    auto depth_msg = geometry_msgs::msg::PointStamped();

    double pressure = msg->fluid_pressure;  
    const double rho = 1000.0;      
    const double g = 9.81;
    const double p_surface = 101325.0; 

    double depth = (pressure - p_surface) / (rho * g);

    depth_msg.header.stamp = msg->header.stamp; 
    depth_msg.header.frame_id = "base_link";
    depth_msg.point.x = 0.0;
    depth_msg.point.y = 0.0; 
    depth_msg.point.z = depth;


    RCLCPP_INFO(this->get_logger(), "Publicando mensagem de profundidade");
    depth_publisher->publish(depth_msg);

  }

  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr depth_publisher;

};

}

RCLCPP_COMPONENTS_REGISTER_NODE(depth_log::DepthNode)