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
#include <rclcpp/wait_for_message.hpp>

#include <std_srvs/srv/set_bool.hpp>

class StereoImageView : public rclcpp::Node {
public:
    StereoImageView(sensor_msgs::msg::CameraInfo info_left,sensor_msgs::msg::CameraInfo info_right)
        : Node("stereo_image_view") {
        // Read the parameter for show_window_ from the launch file
        this->declare_parameter<bool>("show_window", false);  // Declare the parameter with a default value
        this->get_parameter("show_window", show_window_);
        this->declare_parameter<bool>("rectificate_image",false);
        this->get_parameter("rectificate_image",rectificate_);
        
        auto left_camera_info = sensor_msgs::msg::CameraInfo();
        auto right_camera_info = sensor_msgs::msg::CameraInfo();
        left_camera_info = info_left;
        right_camera_info = info_right;

        // Initialize subscribers
        left_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/camera_1/image_raw");
        right_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/camera_2/image_raw");


        // Initialize synchronizer with ApproximateTime policy
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), *left_sub_, *right_sub_);

        // Register callback
        sync_->registerCallback(std::bind(&StereoImageView::imageCallback, this, std::placeholders::_1, std::placeholders::_2));

        // Serviço para ativar/desativar a janela
        service_show_window = this->create_service<std_srvs::srv::SetBool>("show_window",
            std::bind(&StereoImageView::showImagesCallback, this, std::placeholders::_1, std::placeholders::_2)
        );
        service_rectificate_image = this->create_service<std_srvs::srv::SetBool>("rectificate_image",
            std::bind(&StereoImageView::rectificatedImageCallback, this, std::placeholders::_1, std::placeholders::_2)
        );
    }

private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
                       const sensor_msgs::msg::Image::ConstSharedPtr& right_msg) {
        try {
            if (!show_window_) {
                cv::destroyAllWindows();
                return;
            }

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

    void showImagesCallback(const std_srvs::srv::SetBool::Request::SharedPtr request,
                              std_srvs::srv::SetBool::Response::SharedPtr response) {
        show_window_ = request->data;
        response->success = true;
        response->message = show_window_ ? "Window enabled." : "Window disabled.";
        RCLCPP_INFO(this->get_logger(), "Window display set to: %s", show_window_ ? "ON" : "OFF");
    }
    void rectificatedImageCallback(const std_srvs::srv::SetBool::Request::SharedPtr request,
                                std_srvs::srv::SetBool::Response::SharedPtr response){
        rectificate_ = request -> data;
        response->success = true;
        response->message = rectificate_ ? "image rectificated":"image raw";


    }

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> right_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_show_window;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_rectificate_image;
    sensor_msgs::msg::CameraInfo left_camera_info;
    sensor_msgs::msg::CameraInfo right_camera_info;
    bool show_window_; // Controle da exibição da janela
    bool rectificate_; // Controle da rectificaçao da imagem 
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    if (argc<3) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Usage: StereoImageView_node <left_info_topic> <right_info_topic>");
    }
    std::string left_info_topic = argv[1];
    std::string right_info_topic = argv[2];

    auto left_camera_info = sensor_msgs::msg::CameraInfo();
    auto right_camera_info = sensor_msgs::msg::CameraInfo();

    auto left_camera_info_received = false;
    auto right_camera_info_received = false;

    auto info_node = std::make_shared<rclcpp::Node>("camera_info_subscriber");
    right_camera_info_received = rclcpp::wait_for_message<sensor_msgs::msg::CameraInfo>(right_camera_info,info_node,right_info_topic, std::chrono::seconds(5));
    left_camera_info_received = rclcpp::wait_for_message<sensor_msgs::msg::CameraInfo>(left_camera_info,info_node,left_info_topic, std::chrono::seconds(5));
    
    
    if(left_camera_info_received && right_camera_info_received){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Camera info received");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Left info topic: %s", left_info_topic.c_str());
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Right info topic: %s", right_info_topic.c_str());
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Right Projection Matrix P[3] value: %f", right_camera_info.p[3]);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No camera info provided");
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Left info topic: %s", left_info_topic.c_str());
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Right info topic: %s", right_info_topic.c_str());
    }


    rclcpp::spin(std::make_shared<StereoImageView>(left_camera_info,right_camera_info));
    rclcpp::shutdown();
    return 0;
}
