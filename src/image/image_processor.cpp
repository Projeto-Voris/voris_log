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
    this->declare_parameter<int>("resize_width", 612);
    this->declare_parameter<int>("resize_height", 512);
    this->declare_parameter<int>("jpeg_quality", 40); // Controle manual da qualidade
    this->declare_parameter<bool>("apply_clahe", true);
    this->declare_parameter<int>("clahe_climp", 5);
    this->declare_parameter<int>("clahe_tile", 5);

    if(this->get_parameter("apply_clahe").as_bool()){
        int clip_limit = this->get_parameter("clahe_climp").as_int();
        int tile_grid_size = this->get_parameter("clahe_tile").as_int();
        clahe_ = cv::createCLAHE(clip_limit, cv::Size(tile_grid_size, tile_grid_size));
        RCLCPP_INFO(this->get_logger(), "CLAHE ON: clip_limit=%d, tile_grid_size=%dx%d", clip_limit, tile_grid_size, tile_grid_size);
    }


    rclcpp::QoS qos_profile(2); // QoS Best Effort para sensores
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    auto cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cb_group;

    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "input/image", 
      rclcpp::SensorDataQoS(), // QoS Best Effort (bom para câmeras)
      std::bind(&ImageProcessorNode::imageCallback, this, std::placeholders::_1), sub_options);


    pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("output/compressed_image", qos_profile); // Publica CompressedImage

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
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);

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

      if(this->get_parameter("apply_clahe").as_bool()){
        
        processed_img = applyCLAHEtoColor(processed_img);
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
    cv::Mat applyCLAHEtoColor(const cv::Mat& input_bgr)
    {
        cv::Mat lab_image;
        cv::cvtColor(input_bgr, lab_image, cv::COLOR_BGR2Lab);

        // 2. Separar a imagem nos 3 canais (L, A, B)
        std::vector<cv::Mat> lab_channels(3);
        cv::split(lab_image, lab_channels);

        // Aplicar o CLAHE exclusivamente no canal L (lab_channels[0])
        clahe_->apply(lab_channels[0], lab_channels[0]);

        // 4. Juntar os canais modificados de volta em uma única imagem LAB
        cv::Mat processed_lab;
        cv::merge(lab_channels, processed_lab);

        // 5. Converter de volta para o padrão BGR
        cv::Mat output_bgr;
        cv::cvtColor(processed_lab, output_bgr, cv::COLOR_Lab2BGR);

        return output_bgr;
    }
  cv::Ptr<cv::CLAHE> clahe_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_; // Tipo alterado
};

}  // namespace voris_log

RCLCPP_COMPONENTS_REGISTER_NODE(voris_log::ImageProcessorNode)
