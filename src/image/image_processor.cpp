#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace voris_log
{

class ImageProcessorNode : public rclcpp::Node
{
public:
  // Construtor compatível com Composable Node
  explicit ImageProcessorNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("image_processor", options)
  {
    // Parâmetros de redimensionamento
    this->declare_parameter<int>("resize_width", 640);
    this->declare_parameter<int>("resize_height", 480);
    // Nota: A qualidade JPEG é gerida pelo parametro do plugin do image_transport
    // ex: _jpeg_quality:=50 no launch

    // 1. Subscriber Padrão (Melhor para Zero-Copy com Spinnaker)
    // Usamos SensorDataQoS para garantir compatibilidade com drivers de câmera (Best Effort)
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "input/image", 
      rclcpp::SensorDataQoS(), 
      std::bind(&ImageProcessorNode::imageCallback, this, std::placeholders::_1));

    // 2. Publisher via ImageTransport
    // Isso cria automaticamente os tópicos "output/image", "output/image/compressed", etc.
    // Usamos a função estática create_publisher que aceita o ponteiro 'this' cru
    pub_ = image_transport::create_publisher(this, "output/image");

    RCLCPP_INFO(this->get_logger(), "Image Processor (Resizer) iniciado.");
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    try {
      // Zero-Copy check: Se estivermos no mesmo processo, isso é apenas um ponteiro
      // Mas para OpenCV precisamos de acesso aos dados. toCvShare é eficiente.
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);

      int target_width = this->get_parameter("resize_width").as_int();
      int target_height = this->get_parameter("resize_height").as_int();

      // Se a imagem já for pequena, não faz nada
      if (cv_ptr->image.cols <= target_width && cv_ptr->image.rows <= target_height) {
         pub_.publish(msg);
         return;
      }

      // Redimensiona
      cv::Mat resized;
      cv::resize(cv_ptr->image, resized, cv::Size(target_width, target_height), 0, 0, cv::INTER_LINEAR);

      // Publica
      // Criamos um novo CvImage. O header mantém o timestamp original para sincronia!
      cv_bridge::CvImage out_msg;
      out_msg.header = msg->header; 
      out_msg.encoding = msg->encoding; // Mantém a codificação (ex: bayer_rg8 ou bgr8)
      out_msg.image = resized;

      pub_.publish(out_msg.toImageMsg());

    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Erro no cv_bridge: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  image_transport::Publisher pub_;
};

}  // namespace voris_log

// Registro do Componente
RCLCPP_COMPONENTS_REGISTER_NODE(voris_log::ImageProcessorNode)