#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <filesystem>
#include <iomanip>
#include <sstream>
#include <string>

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
    }

    RCLCPP_INFO(this->get_logger(), "Salvando dados no diretório: %s", save_path_.c_str());

    // 2. Configuração dos Subscribers com message_filters
    // Usando rmw_qos_profile_sensor_data (Best Effort) que é comum para sensores
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

    sub_img_left_.subscribe(this, "camera/left", qos_profile);
    sub_img_right_.subscribe(this, "camera/right", qos_profile);
    sub_sonar_pc_.subscribe(this, "sonar_point_cloud", qos_profile);

    // 3. Configuração do Sincronizador ApproximateTime
    // O tamanho da fila (queue_size) é definido como 10
    sync_ = std::make_shared<Sync>(
      SyncPolicy(100), sub_img_left_, sub_img_right_, sub_sonar_pc_);
    
    sync_->registerCallback(
      std::bind(&DataSaverNode::syncCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
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
  
  std::shared_ptr<Sync> sync_;

  std::string save_path_;
  int msg_counter_;

  // Callback chamado quando as 3 mensagens estão sincronizadas
  void syncCallback(
    const ImageMsg::ConstSharedPtr & msg_img_l,
    const ImageMsg::ConstSharedPtr & msg_img_r,
    const Pc2Msg::ConstSharedPtr & msg_pc2)
  {
    RCLCPP_INFO(this->get_logger(), "Saving images and sonar data");
    // Formata o contador com zeros à esquerda (ex: 000, 001, ..., 999)
    std::ostringstream ss;
    ss << std::setw(3) << std::setfill('0') << msg_counter_;
    std::string counter_str = ss.str();

    // Salvar a Imagem Esquerda (L###.png)
    try {
      cv::Mat cv_img_l = cv_bridge::toCvShare(msg_img_l, "bgr8")->image;
      std::string path_l = save_path_ + "/L" + counter_str + ".png";
      cv::imwrite(path_l, cv_img_l);
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Erro no cv_bridge (Esquerda): %s", e.what());
    }

    // Salvar a Imagem Direita (R###.png)
    try {
      cv::Mat cv_img_r = cv_bridge::toCvShare(msg_img_r, "bgr8")->image;
      std::string path_r = save_path_ + "/R" + counter_str + ".png";
      cv::imwrite(path_r, cv_img_r);
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Erro no cv_bridge (Direita): %s", e.what());
    }

    // Salvar a PointCloud2 (SONAR###.ply)
    try {
      pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
      pcl::fromROSMsg(*msg_pc2, pcl_cloud);
      std::string path_sonar = save_path_ + "/SONAR" + counter_str + ".ply";
      
      // Salva em formato ASCII ou Binário (Binário é menor e mais rápido)
      pcl::io::savePLYFileBinary(path_sonar, pcl_cloud);
    } catch (std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Erro ao salvar PLY: %s", e.what());
    }

    RCLCPP_INFO(this->get_logger(), "Dados salvos com índice: %s", counter_str.c_str());
    msg_counter_++;
  }
};

}  // namespace voris_log

// Registra o nó como um componente instanciável
RCLCPP_COMPONENTS_REGISTER_NODE(voris_log::DataSaverNode)