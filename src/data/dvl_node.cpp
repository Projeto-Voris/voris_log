#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "marine_acoustic_msgs/msg/dvl.hpp"

namespace dvl_log
{

class DVLNode : public rclcpp::Node
{
public:
  explicit DVLNode(const rclcpp::NodeOptions & options) : rclcpp::Node("dvl_msg_converter", options)
  
  {
    // Instancia o publisher com o tipo correto
    publisher_dvl_filtered = this->create_publisher<geometry_msgs::msg::TwistWithCovariance>(
      "dvl_twist", 10);

    // Instancia o subscriber com o tipo e callback corretos
    subscriber_dvl_sim = this->create_subscription<marine_acoustic_msgs::msg::Dvl>(
      "/model/bluerov2/dvl/velocity", 10,
      std::bind(&DVLNode::on_message_received, this, std::placeholders::_1));
  }

private:
  void on_message_received(const marine_acoustic_msgs::msg::Dvl::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Recebido DVL. Convertendo para Twist...");

    auto nova_mensagem = geometry_msgs::msg::TwistWithCovariance();
    
    // Copia as velocidades lineares do DVL
    nova_mensagem.twist.linear.x = msg->velocity.x;
    nova_mensagem.twist.linear.y = msg->velocity.y;
    nova_mensagem.twist.linear.z = msg->velocity.z;

    // Zera as velocidades angulares 
    nova_mensagem.twist.angular.x = 0.0;
    nova_mensagem.twist.angular.y = 0.0;
    nova_mensagem.twist.angular.z = 0.0;

    nova_mensagem.covariance.fill(0.0);

    // Preenche os dados de covariancia --> pequenas alterações pois o formato das mensagens não é compatível 
    nova_mensagem.covariance[0]  = msg->velocity_covar[0];
    nova_mensagem.covariance[1]  = msg->velocity_covar[1];
    nova_mensagem.covariance[2]  = msg->velocity_covar[2];

    // Linha 2
    nova_mensagem.covariance[6]  = msg->velocity_covar[3];
    nova_mensagem.covariance[7]  = msg->velocity_covar[4];
    nova_mensagem.covariance[8]  = msg->velocity_covar[5];

    // Linha 3
    nova_mensagem.covariance[12] = msg->velocity_covar[6];
    nova_mensagem.covariance[13] = msg->velocity_covar[7];
    nova_mensagem.covariance[14] = msg->velocity_covar[8];

    double high_uncertainty = 1e6;
    nova_mensagem.covariance[21] = high_uncertainty; 
    nova_mensagem.covariance[28] = high_uncertainty; 
    nova_mensagem.covariance[35] = high_uncertainty;  

    RCLCPP_INFO(this->get_logger(), "Publicando Twist Filtrada");
    publisher_dvl_filtered->publish(nova_mensagem);
  }

    rclcpp::Subscription<marine_acoustic_msgs::msg::Dvl>::SharedPtr subscriber_dvl_sim;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovariance>::SharedPtr publisher_dvl_filtered;
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(dvl_log::DVLNode)