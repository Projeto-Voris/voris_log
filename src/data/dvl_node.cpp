#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "marine_acoustic_msgs/msg/dvl.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Transform.h"

namespace dvl_log
{

class DVLNode : public rclcpp::Node
{
public:
  explicit DVLNode(const rclcpp::NodeOptions & options) : rclcpp::Node("dvl_msg_converter", options)
  
  {

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Instancia o publisher com o tipo correto
    publisher_dvl_filtered = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/mavros/vision_speed/speed_twist_cov", 10);

    // Instancia o subscriber com o tipo e callback corretos
    subscriber_dvl_sim = this->create_subscription<marine_acoustic_msgs::msg::Dvl>(
      "/waterlinked_dvl_driver/velocity_report", 10,
      std::bind(&DVLNode::on_message_received, this, std::placeholders::_1));
  }

private:
  void on_message_received(const marine_acoustic_msgs::msg::Dvl::SharedPtr msg)
  {
    if (!tf_static_cached_) {
          std::string base_frame_ = "base_link";
          std::string dvl_frame_ = msg->header.frame_id; // Supondo que o frame do DVL esteja correto no header da mensagem
          try {
              auto tf_base_dvl = tf_buffer_->lookupTransform(
                  base_frame_, dvl_frame_, tf2::TimePointZero);
              tf2::fromMsg(tf_base_dvl.transform, T_base_dvl_);
              tf_static_cached_ = true;
              RCLCPP_INFO(this->get_logger(), "Static transform [%s -> %s] successfully cached!", base_frame_.c_str(), dvl_frame_.c_str());
          } catch (const tf2::TransformException& ex) {
              RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                  "Wait static TF to be available: %s", ex.what());
              return; // Retorna cedo pois não podemos publicar path/poses corretos sem essa TF
          }
      }
    auto nova_mensagem = geometry_msgs::msg::TwistWithCovarianceStamped();

    nova_mensagem.header = msg->header; // Copia o header do DVL
    nova_mensagem.header.frame_id = "base_link"; // Copia o header do DVL
    nova_mensagem.header.stamp = this->get_clock()->now();
    
    tf2::Vector3 vel_dvl(msg->velocity.x, msg->velocity.y, msg->velocity.z);

    // 2. Rotacionar o vetor usando apenas a base da transformada (Matriz de Rotação 3x3)
    tf2::Vector3 vel_base = T_base_dvl_.getBasis() * vel_dvl;

    // 3. Atribuir as velocidades rotacionadas à nova mensagem
    nova_mensagem.twist.twist.linear.x = vel_base.x();
    nova_mensagem.twist.twist.linear.y = vel_base.y();
    nova_mensagem.twist.twist.linear.z = vel_base.z();

    // Zera as velocidades angulares 
    nova_mensagem.twist.twist.angular.x = 0.0;
    nova_mensagem.twist.twist.angular.y = 0.0;
    nova_mensagem.twist.twist.angular.z = 0.0;

    nova_mensagem.twist.covariance.fill(0.0);

    tf2::Matrix3x3 R = T_base_dvl_.getBasis();
    tf2::Matrix3x3 C_dvl(
      msg->velocity_covar[0], msg->velocity_covar[1], msg->velocity_covar[2],
      msg->velocity_covar[3], msg->velocity_covar[4], msg->velocity_covar[5],
      msg->velocity_covar[6], msg->velocity_covar[7], msg->velocity_covar[8]
    );
    
    tf2::Matrix3x3 C_base = R * C_dvl * R.transpose();

    // Preenche os dados de covariancia rotacionados na matriz 6x6 do Twist
    // Linha 0 (Linear X)
    nova_mensagem.twist.covariance[0]  = C_base[0][0];
    nova_mensagem.twist.covariance[1]  = C_base[0][1];
    nova_mensagem.twist.covariance[2]  = C_base[0][2];

    // Linha 1 (Linear Y)
    nova_mensagem.twist.covariance[6]  = C_base[1][0];
    nova_mensagem.twist.covariance[7]  = C_base[1][1];
    nova_mensagem.twist.covariance[8]  = C_base[1][2];

    // Linha 2 (Linear Z)
    nova_mensagem.twist.covariance[12] = C_base[2][0];
    nova_mensagem.twist.covariance[13] = C_base[2][1];
    nova_mensagem.twist.covariance[14] = C_base[2][2];

    // Incerteza alta para a parte angular que não possuímos
    double high_uncertainty = 1e6;
    nova_mensagem.twist.covariance[21] = high_uncertainty; // Angular X
    nova_mensagem.twist.covariance[28] = high_uncertainty; // Angular Y
    nova_mensagem.twist.covariance[35] = high_uncertainty; // Angular Z

    // RCLCPP_INFO(this->get_logger(), "Publicando Twist Filtrada");
    publisher_dvl_filtered->publish(nova_mensagem);
  }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    bool tf_static_cached_{false};
    tf2::Transform T_base_dvl_;
    rclcpp::Subscription<marine_acoustic_msgs::msg::Dvl>::SharedPtr subscriber_dvl_sim;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr publisher_dvl_filtered;
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(dvl_log::DVLNode)