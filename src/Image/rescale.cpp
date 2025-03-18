#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>

class ImageRescaler : public rclcpp::Node
{
public:
    ImageRescaler() : Node("image_rescaler")
    {
        // Create a subscription to the right image topic
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_in", 10,
            std::bind(&ImageRescaler::imageCallback, this, std::placeholders::_1));

        // Create a publisher for the rescaled image
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_out", 10);
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert ROS Image message to OpenCV image
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Rescale the image to 800x600
        cv::Mat resized_image;
        cv::resize(cv_ptr->image, resized_image, cv::Size(800, 600), cv::INTER_LINEAR);

        // Convert the rescaled OpenCV image back to a ROS Image message
        auto rescaled_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, resized_image).toImageMsg();

        // Publish the rescaled image
        rescaled_msg->header.stamp = this->now();
        publisher_->publish(*rescaled_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageRescaler>());
    rclcpp::shutdown();
    return 0;
}