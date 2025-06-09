#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class RectifyImageNode : public rclcpp::Node
{
public:
    RectifyImageNode()
    : Node("rectify_image_node")
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", 10,
            std::bind(&RectifyImageNode::image_callback, this, std::placeholders::_1));
        info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera_info", 10,
            std::bind(&RectifyImageNode::info_callback, this, std::placeholders::_1));
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_rect", 10);
        camera_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info_rect", 10);
    }

private:
    void info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (!got_camera_info_) {
            camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
            dist_coeffs_ = cv::Mat(msg->d.size(), 1, CV_64F, const_cast<double*>(msg->d.data())).clone();
            image_size_ = cv::Size(msg->width, msg->height);
            got_camera_info_ = true;
            RCLCPP_INFO(this->get_logger(), "Received camera info.");
        }
        camera_info_pub->publish(*msg);

    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!got_camera_info_) {
            RCLCPP_WARN(this->get_logger(), "Waiting for camera info...");
            return;
        }

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat rectified;
        cv::undistort(cv_ptr->image, rectified, camera_matrix_, dist_coeffs_);

        auto out_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::RGB8, rectified).toImageMsg();
        image_pub_->publish(*out_msg);

    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub;

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Size image_size_;
    bool got_camera_info_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RectifyImageNode>());
    rclcpp::shutdown();
    return 0;
}