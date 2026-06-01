#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <filesystem>
#include <iomanip>
#include <sstream>
#include <string>
#include <fstream>

namespace voris_log
{

class DataSaverNode : public rclcpp::Node
{
public:
  explicit DataSaverNode(const rclcpp::NodeOptions & options)
  : Node("data_saver_node", options), msg_counter_(0)
  {
    // 1. Declaração e leitura do parâmetro do caminho para salvar
    this->declare_parameter<std::string>("save_directory", "/tmp/ros_data/");
    save_path_ = this->get_parameter("save_directory").as_string();

    // Cria o diretório se não existir
    if (!std::filesystem::exists(save_path_)) {
      std::filesystem::create_directories(save_path_);
      std::filesystem::create_directories(save_path_ + "/left");
      std::filesystem::create_directories(save_path_ + "/right");
      std::filesystem::create_directories(save_path_ + "/sonar");
      std::string odom_file_path = save_path_ + "/odom_data.txt";
      odom_file_.open(odom_file_path, std::ios::out);
        if (!odom_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot Open file");
        } else {
            // Escreve um cabeçalho opcional no arquivo para organizar os dados
            odom_file_ << "Timestamp, x, y, z, w, x, y, z\n";
        }

    }

    RCLCPP_INFO(this->get_logger(), "Salvando dados no diretório: %s", save_path_.c_str());

    // 2. Configuração dos Subscribers com message_filters
    // Usando rmw_qos_profile_sensor_data (Best Effort) que é comum para sensores
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odometry", rclcpp::SensorDataQoS(), std::bind(&DataSaverNode::odomCallback, this, std::placeholders::_1));

    sub_img_left_.subscribe(this, "camera/left", qos_profile);
    sub_img_right_.subscribe(this, "camera/right", qos_profile);
    sub_sonar_pc_.subscribe(this, "sonar_point_cloud", qos_profile);

    // 3. Configuração do Sincronizador ApproximateTime
    // O tamanho da fila (queue_size) é definido como 10
    sync_ = std::make_shared<Sync>(
      SyncPolicy(100), sub_img_left_, sub_img_right_, sub_sonar_pc_);
    
    sync_->registerCallback(
      std::bind(&DataSaverNode::syncCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    get_data_srv_ = this->create_service<std_srvs::srv::Trigger>("save_data",  std::bind(&DataSaverNode::get_data_srv, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  // Definição dos tipos para o sincronizador
  using ImageMsg = sensor_msgs::msg::Image;
  using Pc2Msg = sensor_msgs::msg::PointCloud2;
  
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg, Pc2Msg>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;

  message_filters::Subscriber<ImageMsg> sub_img_left_;
  message_filters::Subscriber<ImageMsg> sub_img_right_;
  message_filters::Subscriber<Pc2Msg> sub_sonar_pc_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  
  std::shared_ptr<Sync> sync_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_data_srv_;

  std::string save_path_;
  bool get_data{false};
  int msg_counter_;
  std::ofstream odom_file_;
  nav_msgs::msg::Odometry::ConstSharedPtr last_odom_;

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
  {
    last_odom_ = msg;
  }


  // Callback chamado quando as 3 mensagens estão sincronizadas
  void syncCallback(const ImageMsg::ConstSharedPtr & msg_img_l, const ImageMsg::ConstSharedPtr & msg_img_r, const Pc2Msg::ConstSharedPtr & msg_pc2)
  {
    if(!get_data){
      return; 
    }

    // Formata o contador com zeros à esquerda (ex: 000, 001, ..., 999)
    std::ostringstream ss;
    ss << std::setw(3) << std::setfill('0') << msg_counter_;
    std::string counter_str = ss.str();

    try {
        cv::Mat cv_img_l = cv_bridge::toCvShare(msg_img_l, msg_img_l->encoding)->image;
        cv::cvtColor(cv_img_l, cv_img_l, cv::COLOR_BayerBG2BGR);
        std::string path_l = save_path_ + "/left/L" + counter_str + ".png";
        cv::imwrite(path_l, cv_img_l);
      } catch (cv_bridge::Exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Erro no cv_bridge (Esquerda): %s", e.what());
      }
      
      // Salvar a Imagem Direita (R###.png)
      try {
        cv::Mat cv_img_r = cv_bridge::toCvShare(msg_img_r, msg_img_r->encoding)->image;
        cv::cvtColor(cv_img_r, cv_img_r, cv::COLOR_BayerBG2BGR);
        std::string path_r = save_path_ + "/right/R" + counter_str + ".png";
      cv::imwrite(path_r, cv_img_r);
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Erro no cv_bridge (Direita): %s", e.what());
    }

    // Salvar a PointCloud2 (SONAR###.ply)
    try {
      pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
      pcl::fromROSMsg(*msg_pc2, pcl_cloud);
      std::string path_sonar = save_path_ + "/sonar/SONAR" + counter_str + ".ply";
      
      // Salva em formato ASCII ou Binário (Binário é menor e mais rápido)
      pcl::io::savePLYFileBinary(path_sonar, pcl_cloud);
    } catch (std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Erro ao salvar PLY: %s", e.what());
    }

    if(odom_file_.is_open()){
            odom_file_ << last_odom_->header.stamp.sec << "." << last_odom_->header.stamp.nanosec << ","
                    << last_odom_->pose.pose.position.x << ","
                    << last_odom_->pose.pose.position.y << ","
                    << last_odom_->pose.pose.position.z << ","
                    << last_odom_->pose.pose.orientation.w << ","
                    << last_odom_->pose.pose.orientation.x << ","
                    << last_odom_->pose.pose.orientation.y << ","
                    << last_odom_->pose.pose.orientation.z << "\n";

            odom_file_.flush();   
        } else {
            RCLCPP_ERROR(this->get_logger(), "File is not open");
        }


    RCLCPP_INFO(this->get_logger(), "Save data as: %s", counter_str.c_str());
    msg_counter_++;
    get_data = false;
  }

  void get_data_srv(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    // Este serviço pode ser usado para retornar o caminho do último arquivo salvo ou para acionar uma ação específica

    get_data = true;
    response->success = true;
    response->message = "Request data acquisition, last one: " + std::to_string(msg_counter_ - 1);
  }
};

} RCLCPP_COMPONENTS_REGISTER_NODE(voris_log::DataSaverNode)
