#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <mutex>

class ImageRescaler : public rclcpp::Node
{
public:
    ImageRescaler() : Node("image_rescaler")
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_in", 10,
            std::bind(&ImageRescaler::image_callback, this, std::placeholders::_1));

        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera_info_in", 10,
            std::bind(&ImageRescaler::camera_info_callback, this, std::placeholders::_1));

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_out", 10);
        camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info_out", 10);
    }

private:
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_camera_info_ = *msg;
        got_camera_info_ = true;
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!got_camera_info_) {
            RCLCPP_WARN(this->get_logger(), "Waiting for camera info...");
            return;
        }

        // Convert ROS Image message to OpenCV image
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Rescale the image to half of its original size
        int new_width = cv_ptr->image.cols / 2;
        int new_height = cv_ptr->image.rows / 2;
        cv::Mat resized_image;
        cv::resize(cv_ptr->image, resized_image, cv::Size(new_width, new_height), 0, 0, cv::INTER_LINEAR);

        // Publish the rescaled image
        auto rescaled_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, resized_image).toImageMsg();
        image_pub_->publish(*rescaled_msg);

        // Update and publish CameraInfo
        sensor_msgs::msg::CameraInfo new_info;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            new_info = last_camera_info_;
        }
        double scale_x = static_cast<double>(new_width) / static_cast<double>(new_info.width);
        double scale_y = static_cast<double>(new_height) / static_cast<double>(new_info.height);

        new_info.header = msg->header;
        new_info.width = new_width;
        new_info.height = new_height;
        // Scale K (intrinsic matrix)
        new_info.k[0] *= scale_x; // fx
        new_info.k[2] *= scale_x; // cx
        new_info.k[4] *= scale_y; // fy
        new_info.k[5] *= scale_y; // cy
        // Scale P (projection matrix)
        new_info.p[0] *= scale_x; // fx'
        new_info.p[2] *= scale_x; // cx'
        new_info.p[5] *= scale_y; // fy'
        new_info.p[6] *= scale_y; // cy'

        camera_info_pub_->publish(new_info);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

    sensor_msgs::msg::CameraInfo last_camera_info_;
    bool got_camera_info_ = false;
    std::mutex mutex_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageRescaler>());
    rclcpp::shutdown();
    return 0;
}