#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/core.hpp>

class CompressedToRaw : public rclcpp::Node
{
public:
    CompressedToRaw() : Node("compressed_to_raw")
    {
        // Subscription to compressed image
        sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "image/compressed", 10,
            std::bind(&CompressedToRaw::callback, this, std::placeholders::_1));

        // Publisher for raw image
        pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
    }

private:
    void callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        try
        {
            // Decode compressed image -> cv::Mat
            cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);

            if (image.empty()) {
                RCLCPP_WARN(this->get_logger(), "Decoded image is empty!");
                return;
            }

            // Convert cv::Mat -> ROS2 Image
            std_msgs::msg::Header header;
            header.stamp = msg->header.stamp;
            header.frame_id = msg->header.frame_id;

            sensor_msgs::msg::Image::SharedPtr img_msg =
                cv_bridge::CvImage(header, "bgr8", image).toImageMsg();

            pub_->publish(*img_msg);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert image: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CompressedToRaw>());
    rclcpp::shutdown();
    return 0;
}
