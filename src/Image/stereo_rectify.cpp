#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <image_transport/image_transport.hpp>
#include <mutex>

class StereoRectifyNode : public rclcpp::Node
{
public:
    StereoRectifyNode()
    : Node("stereo_rectify_node")
    {
        using namespace std::placeholders;

        left_image_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "left/image_raw");
        right_image_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "right/image_raw");

        sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), *left_image_sub_, *right_image_sub_);
        sync_->registerCallback(std::bind(&StereoRectifyNode::stereo_callback, this, _1, _2));

        left_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "left/camera_info", 10,
            std::bind(&StereoRectifyNode::left_info_callback, this, _1));
        right_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "right/camera_info", 10,
            std::bind(&StereoRectifyNode::right_info_callback, this, _1));

        left_rect_pub_ = image_transport::create_publisher(this, "left/image_rect");
        right_rect_pub_ = image_transport::create_publisher(this, "right/image_rect");

        left_camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("left/camera_info_rect", 10);
        right_camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("right/camera_info_rect", 10);
    }

private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,
        sensor_msgs::msg::Image>;

    std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_image_sub_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> right_image_sub_;
    std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_sub_;

    image_transport::Publisher left_rect_pub_;
    image_transport::Publisher right_rect_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_camera_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_camera_info_pub_;

    sensor_msgs::msg::CameraInfo::ConstSharedPtr last_left_info_;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr last_right_info_;
    std::mutex info_mutex_;

    void left_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(info_mutex_);
        last_left_info_ = msg;
    }

    void right_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(info_mutex_);
        last_right_info_ = msg;
    }

    void stereo_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr &left_img,
        const sensor_msgs::msg::Image::ConstSharedPtr &right_img)
    {
        sensor_msgs::msg::CameraInfo::ConstSharedPtr left_info, right_info;
        {
            std::lock_guard<std::mutex> lock(info_mutex_);
            left_info = last_left_info_;
            right_info = last_right_info_;
        }

        if (!left_info || !right_info) {
            RCLCPP_WARN(this->get_logger(), "Waiting for camera info...");
            return;
        }

        cv::Mat left_rect, right_rect;
        if (!rectify_images(left_img, right_img, left_info, right_info, left_rect, right_rect)) {
            RCLCPP_WARN(this->get_logger(), "Rectification failed");
            return;
        }

        auto left_rect_msg = cv_bridge::CvImage(left_img->header, "bgr8", left_rect).toImageMsg();
        auto right_rect_msg = cv_bridge::CvImage(right_img->header, "bgr8", right_rect).toImageMsg();
        publish_camera_info(left_info, right_info);
        left_rect_pub_.publish(left_rect_msg);
        right_rect_pub_.publish(right_rect_msg);
    }
    bool publish_camera_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info_l,
                             const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info_r)
    {
        if (!info_l || !info_r) {
            RCLCPP_WARN(this->get_logger(), "Camera info is null");
            return false;
        }

        sensor_msgs::msg::CameraInfo info_l_msg = *info_l;
        sensor_msgs::msg::CameraInfo info_r_msg = *info_r;
        // Set distortion coefficients to zeros
        info_l_msg.d.assign(info_l_msg.d.size(), 0.0);
        info_r_msg.d.assign(info_r_msg.d.size(), 0.0);

        // Set rectification matrix R to identity
        info_l_msg.r = {1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0};
        info_r_msg.r = {1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0};
        auto time = this->now();
        info_l_msg.header.stamp = time;
        info_r_msg.header.stamp = time;
        left_camera_info_pub_->publish(info_l_msg);
        right_camera_info_pub_->publish(info_l_msg);

        return true;
    }

    bool rectify_images(
        const sensor_msgs::msg::Image::ConstSharedPtr &left_img,
        const sensor_msgs::msg::Image::ConstSharedPtr &right_img,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr &left_info,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr &right_info,
        cv::Mat &left_rect,
        cv::Mat &right_rect)
    {
        if (!left_img || !right_img || !left_info || !right_info) {
            return false;
        }

        cv_bridge::CvImageConstPtr left_cv_ptr, right_cv_ptr;
        try {
            left_cv_ptr = cv_bridge::toCvShare(left_img, "bgr8");
            right_cv_ptr = cv_bridge::toCvShare(right_img, "bgr8");
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return false;
        }

        cv::Mat R1(3, 3, CV_64F, const_cast<double*>(left_info->r.data()));
        cv::Mat R2(3, 3, CV_64F, const_cast<double*>(right_info->r.data()));
        cv::Mat P1(3, 4, CV_64F, const_cast<double*>(left_info->p.data()));
        cv::Mat P2(3, 4, CV_64F, const_cast<double*>(right_info->p.data()));
        cv::Mat K1(3, 3, CV_64F, const_cast<double*>(left_info->k.data()));
        cv::Mat K2(3, 3, CV_64F, const_cast<double*>(right_info->k.data()));
        cv::Mat D1(1, 5, CV_64F, const_cast<double*>(left_info->d.data()));
        cv::Mat D2(1, 5, CV_64F, const_cast<double*>(right_info->d.data()));


        cv::Size image_size(left_img->width, left_img->height);


        cv::Mat map1x, map1y, map2x, map2y;
        cv::initUndistortRectifyMap(K1, D1, R1, P1, image_size, CV_32FC1, map1x, map1y);
        cv::initUndistortRectifyMap(K2, D2, R2, P2, image_size, CV_32FC1, map2x, map2y);

        cv::remap(left_cv_ptr->image, left_rect, map1x, map1y, cv::INTER_LINEAR);
        cv::remap(right_cv_ptr->image, right_rect, map2x, map2y, cv::INTER_LINEAR);


        return true;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StereoRectifyNode>());
    rclcpp::shutdown();
    return 0;
}