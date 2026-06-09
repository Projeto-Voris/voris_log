#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "marine_acoustic_msgs/msg/dvl.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>

namespace voris_log
{

class DVL2MavrosNode : public rclcpp::Node
{
public:
  explicit DVL2MavrosNode(const rclcpp::NodeOptions & options) : rclcpp::Node("dvl_msg_converter", options)
  
  {
    this->declare_parameter<std::string>("parent_frame", "base_link");

    rclcpp::QoS qos_profile(2); // QoS Best Effort para sensores
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    auto cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cb_group;

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("dvl/twist_cov", 5);

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("dvl/pose_cov", 5);

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("dvl/pose", qos_profile, std::bind(&DVL2MavrosNode::pose_cb, this, std::placeholders::_1), sub_options);

    dvl_sub_ = this->create_subscription<marine_acoustic_msgs::msg::Dvl>(
      "dvl/velocity", rclcpp::SensorDataQoS(), std::bind(&DVL2MavrosNode::dvl_cb, this, std::placeholders::_1), sub_options);
  }

private:

  bool update_tf_cache(const std::string& target_frame, const std::string& source_frame)
    {
      if (transform_cached_ && target_frame == target_frame_ && source_frame == source_frame_) {
        return true;
      }
      if(!transform_cached_){
        try {
          auto tf_msg = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
          target_frame_ = target_frame;
          source_frame_ = source_frame;
          tf2::fromMsg(tf_msg.transform, T_target_source_);
          transform_cached_ = true;
          RCLCPP_INFO(this->get_logger(), "Transform  [%s -> %s] cached!", 
                      source_frame_.c_str(), target_frame.c_str());
          return true;
        } catch (const tf2::TransformException& ex) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Waiting TF: %s", ex.what());
          return false;
        }
      }
      return true;
    }

  void dvl_cb(const marine_acoustic_msgs::msg::Dvl::SharedPtr msg)
  {

    // Instanciar a mensagem correta de Twist com Covariância Stamped
    auto transform_msg = geometry_msgs::msg::TwistWithCovarianceStamped();
    std::string target_frame = this->get_parameter("parent_frame").as_string();
    // Executa sua lógica de lookup/cache para obter a T_target_source_
    if(!update_tf_cache(target_frame, msg->header.frame_id)){
      return;
    }

    // 1. Extrair a Matriz de Rotação 3x3 pura do cache
    tf2::Matrix3x3 R(T_target_source_.getRotation());

    // 2. Rotacionar as Velocidades Lineares (Velocidade é vetor livre: não sofre translação!)
    tf2::Vector3 vel_in(msg->velocity.x, msg->velocity.y, msg->velocity.z);
    tf2::Vector3 vel_out = R * vel_in;

    transform_msg.twist.twist.linear.x = vel_out.x();
    transform_msg.twist.twist.linear.y = vel_out.y();
    transform_msg.twist.twist.linear.z = vel_out.z();

    // Como o DVL padrão (A50, etc) não mede velocidade angular, garantimos que ela seja zerada
    transform_msg.twist.twist.angular.x = 0.0;
    transform_msg.twist.twist.angular.y = 0.0;
    transform_msg.twist.twist.angular.z = 0.0;

    // 3. Montar e Rotacionar o Bloco de Covariância Linear 3x3 (R * Cov * R^T)
    tf2::Matrix3x3 Cov(
      msg->velocity_covar[0], msg->velocity_covar[1], msg->velocity_covar[2],
      msg->velocity_covar[6], msg->velocity_covar[7], msg->velocity_covar[8],
      msg->velocity_covar[12], msg->velocity_covar[13], msg->velocity_covar[14]
    );
    
    tf2::Matrix3x3 transformed_cov = R * Cov * R.transpose(); 

    // 4. Mapear os dados transformados de volta para o array de 36 posições do Twist
    // Inicializa o array com zeros para evitar lixo de memória
    std::fill(transform_msg.twist.covariance.begin(), transform_msg.twist.covariance.end(), 0.0);

    transform_msg.twist.covariance[0]  = transformed_cov[0][0];
    transform_msg.twist.covariance[1]  = transformed_cov[0][1];
    transform_msg.twist.covariance[2]  = transformed_cov[0][2];
    
    transform_msg.twist.covariance[6]  = transformed_cov[1][0];
    transform_msg.twist.covariance[7]  = transformed_cov[1][1];
    transform_msg.twist.covariance[8]  = std::abs(transformed_cov[1][2]);
    
    transform_msg.twist.covariance[12] = transformed_cov[2][0];
    transform_msg.twist.covariance[13] = std::abs(transformed_cov[2][1]);
    transform_msg.twist.covariance[14] = transformed_cov[2][2];

    transform_msg.twist.covariance[21] = -1; // Alta incerteza para as velocidades angulares, já que o DVL não as mede
    transform_msg.twist.covariance[28] = -1;
    transform_msg.twist.covariance[35] = -1;

    // 5. Configurar Cabeçalhos de Sincronismo
    transform_msg.header.stamp = msg->header.stamp; 
    transform_msg.header.frame_id = target_frame_;

    // Publicar mensagem transformada
    twist_pub_->publish(transform_msg);

  }

  void pose_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    std::string target_frame = this->get_parameter("parent_frame").as_string();
    source_frame_ = msg->header.frame_id;
    auto transformed_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
    if(!update_tf_cache(target_frame, msg->header.frame_id)){
      RCLCPP_WARN(this->get_logger(), "Cannot transform from %s to %s..", source_frame_.c_str(), target_frame.c_str());
      return;
    }

    // Criar a mensagem de saída limpa
    auto transform_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
    transform_msg.header.stamp = msg->header.stamp; // Preserva o sincronismo temporal do hardware original
    transform_msg.header.frame_id = target_frame_;

    // 1. Posição XYZ: Rotaciona o vetor do sensor e soma a translação (offset físico no ROV)
    tf2::Matrix3x3 R_t(T_target_source_.getRotation());
    tf2::Vector3 msg_pos(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    tf2::Vector3 transformed_pos = R_t * msg_pos + T_target_source_.getOrigin();
    
    tf2::Quaternion q_msg(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, 
                          msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 msg_ori(q_msg);

    // 3. MUDANÇA DE BASE CINEMÁTICA: R_target = R_t * R_source * R_t^T
    // Isola o vetor de atitude do elo Down e rebate o quatérnio para Z Up (FLU)
    tf2::Matrix3x3 transformed_ori = R_t * msg_ori * R_t.transpose();

    // 4. Converter a matriz de rotação final de volta para Quatérnio ROS estável
    tf2::Quaternion q_final;
    transformed_ori.getRotation(q_final);
    q_final.normalize();

    // 5. Montar a Transformada Homogênea Final da Pose do Veículo
    tf2::Transform T_world_base;
    T_world_base.setOrigin(transformed_pos);
    T_world_base.setRotation(q_final);

    // Preencher os dados espaciais na mensagem ROS
    transform_msg.pose.pose.position.x = T_world_base.getOrigin().x();
    transform_msg.pose.pose.position.y = T_world_base.getOrigin().y();
    transform_msg.pose.pose.position.z = T_world_base.getOrigin().z();
    transform_msg.pose.pose.orientation = tf2::toMsg(T_world_base.getRotation());

    // 6. Rotacionar a matriz de covariância linear 3x3 (Mudar incerteza FRD para FLU)
    tf2::Matrix3x3 Cov_in(
      msg->pose.covariance[0],  msg->pose.covariance[1],  msg->pose.covariance[2],
      msg->pose.covariance[6],  msg->pose.covariance[7],  msg->pose.covariance[8],
      msg->pose.covariance[12], msg->pose.covariance[13], msg->pose.covariance[14]
    );
    tf2::Matrix3x3 Cov_out = R_t * Cov_in * R_t.transpose();

    // Copia os dados antigos e sobrescreve as posições lineares transformadas
    transform_msg.pose.covariance = msg->pose.covariance;
    transform_msg.pose.covariance[0]  = Cov_out[0][0]; transform_msg.pose.covariance[1]  = Cov_out[0][1]; transform_msg.pose.covariance[2]  = Cov_out[0][2];
    transform_msg.pose.covariance[6]  = Cov_out[1][0]; transform_msg.pose.covariance[7]  = Cov_out[1][1]; transform_msg.pose.covariance[8]  = std::abs(Cov_out[1][2]);
    transform_msg.pose.covariance[12] = Cov_out[2][0]; transform_msg.pose.covariance[13] = std::abs(Cov_out[2][1]); transform_msg.pose.covariance[14] = Cov_out[2][2];

    transform_msg.pose.covariance[21] = -1; // Alta incerteza para as rotações, já que o DVL não mede orientação
    transform_msg.pose.covariance[28] = -1;
    transform_msg.pose.covariance[35] = -1;

    // Publicar o dado completamente rebatido com Z Up  
    pose_pub_->publish(transform_msg);  

  }


  bool transform_cached_ = false;
  std::string target_frame_ = "base_link";
  std::string source_frame_;
  tf2::Transform T_target_source_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<marine_acoustic_msgs::msg::Dvl>::SharedPtr dvl_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;

};

}

RCLCPP_COMPONENTS_REGISTER_NODE(voris_log::DVL2MavrosNode)