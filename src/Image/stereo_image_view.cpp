//
// Created by daniel on 14/08/24.
//

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>

class StereoImageView : public rclcpp::Node {
public:
    StereoImageView()
        : Node("stereo_image_view") {

        // Initialize subscribers
        left_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/camera_1/image_raw");
        right_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/camera_2/image_raw");

        // Initialize synchronizer with ApproximateTime policy
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), *left_sub_, *right_sub_);

        // Register callback
        sync_->registerCallback(std::bind(&StereoImageView::imageCallback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
                       const sensor_msgs::msg::Image::ConstSharedPtr& right_msg) {
        try {


            cv::Mat left_image = cv_bridge::toCvCopy(left_msg, left_msg->encoding)->image;
            cv::Mat right_image = cv_bridge::toCvCopy(right_msg, right_msg->encoding)->image;
            // cv::cvtColor(left_image, left_image, cv::COLOR_BayerBG2BGR);
            // cv::cvtColor(right_image, right_image, cv::COLOR_BayerBG2BGR);

            if (left_image.empty() || right_image.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Received empty images.");
                return;
            }

            if (left_image.size() != right_image.size()) {
                RCLCPP_ERROR(this->get_logger(), "Image sizes do not match: left = [%d x %d], right = [%d x %d]",
                             left_image.cols, left_image.rows, right_image.cols, right_image.rows);
                return;
            }

            RCLCPP_INFO(this->get_logger(), "GOT images");
            // Concatenate the images horizontally
            cv::namedWindow("stereo", cv::WINDOW_NORMAL);
            cv::Mat concatenated;
            cv::hconcat(left_image, right_image, concatenated);
            cv::resizeWindow("stereo", concatenated.cols / 4, concatenated.rows / 4);

            cv::imshow("stereo", concatenated);
            cv::waitKey(1);

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        } catch (cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
        }
    }


    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> right_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StereoImageView>());
    rclcpp::shutdown();
    return 0;
}
