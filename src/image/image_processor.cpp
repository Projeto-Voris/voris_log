#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp> // <--- Importante
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace voris_log
{

class ImageProcessorNode : public rclcpp::Node
{
public:
  explicit ImageProcessorNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("image_processor", options)
  {
    // Parâmetros
    this->declare_parameter<int>("resize_width", 640);
    this->declare_parameter<int>("resize_height", 480);
    this->declare_parameter<int>("jpeg_quality", 80); // Controle manual da qualidade

    // 1. Subscriber (Entrada Raw)
    // Mantemos sensor_msgs::msg::Image na entrada para pegar do driver
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "input/image", 
      rclcpp::SensorDataQoS(), // QoS Best Effort (bom para câmeras)
      std::bind(&ImageProcessorNode::imageCallback, this, std::placeholders::_1));

    // 2. Publisher (Saída SOMENTE Comprimida)
    // Mudamos para CompressedImage direto
    pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("output/compressed_image", 10);

    RCLCPP_INFO(this->get_logger(), "Image Processor (JPEG Output Only) iniciado.");
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    try {
      // 1. Conversão para BGR8
      // O driver da Spinnaker geralmente manda BayerRG8. 
      // O JPEG precisa de cor (BGR) ou Mono. "bgr8" força a conversão correta.
      // Se já vier BGR, ele só repassa o ponteiro (zero-copy-ish).
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);

      int target_width = this->get_parameter("resize_width").as_int();
      int target_height = this->get_parameter("resize_height").as_int();
      int jpeg_quality = this->get_parameter("jpeg_quality").as_int();

      // 2. Redimensionamento
      cv::Mat processed_img;
      // Se a imagem for maior que o alvo, redimensiona. Caso contrário, usa a original.
      if (cv_ptr->image.cols > target_width || cv_ptr->image.rows > target_height) {
          cv::resize(cv_ptr->image, processed_img, cv::Size(target_width, target_height), 0, 0, cv::INTER_LINEAR);
      } else {
          processed_img = cv_ptr->image;
      }

      // 3. Compressão Manual (JPEG)
      std::vector<uchar> buffer;
      std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality};
      
      // Codifica a matriz para o buffer de bytes
      cv::imencode(".jpg", processed_img, buffer, compression_params);

      // 4. Montagem da Mensagem CompressedImage
      sensor_msgs::msg::CompressedImage out_msg;
      out_msg.header = msg->header; // Mantém timestamp e frame_id originais
      out_msg.format = "jpeg";
      out_msg.data = buffer; // Move o buffer para a mensagem

      // Publica somente o comprimido
      pub_->publish(out_msg);

    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Erro no cv_bridge: %s", e.what());
    } catch (cv::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Erro no OpenCV: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_; // Tipo alterado
};

}  // namespace voris_log

RCLCPP_COMPONENTS_REGISTER_NODE(voris_log::ImageProcessorNode)
