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
#include <opencv2/imgproc.hpp>
#include <std_srvs/srv/set_bool.hpp>

class StereoImageView : public rclcpp::Node {
public:
    StereoImageView(sensor_msgs::msg::CameraInfo info_left,sensor_msgs::msg::CameraInfo info_right)
        : Node("stereo_image_view"), left_camera_info(info_left),right_camera_info(info_right) {
        // Read the parameter for show_window_ from the launch file
        this->declare_parameter<bool>("show_window", false);  // Declare the parameter with a default value
        this->get_parameter("show_window", show_window_);
        this->declare_parameter<bool>("rectificate_image",false);
        this->get_parameter("rectificate_image",rectificate_);
        
        

        // Initialize subscribers
        left_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image> >(this, "/camera_1/image_raw");
        right_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image> >(this, "/camera_2/image_raw");


        left_rect_pub = this->create_publisher<sensor_msgs::msg::Image>("/camera_1/image_rect", 10);
        right_rect_pub = this->create_publisher<sensor_msgs::msg::Image>("/camera_2/image_rect", 10);

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
            
            left_image = cv_bridge::toCvCopy(left_msg, left_msg->encoding)->image;
            right_image = cv_bridge::toCvCopy(right_msg, right_msg->encoding)->image;

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
            RectfyimgCallback(left_image,right_image);

            if(rectificate_){
                auto left_rectified_msg = cv_bridge::CvImage(left_msg->header, left_msg->encoding, left_image_rectified).toImageMsg();
                auto right_rectified_msg = cv_bridge::CvImage(right_msg->header, right_msg->encoding, right_image_rectified).toImageMsg();
                left_rect_pub->publish(*left_rectified_msg);
                right_rect_pub->publish(*right_rectified_msg);
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
        RCLCPP_INFO(this->get_logger(), "rectification set to: %s", rectificate_ ? "ON" : "OFF");

    }
    void RectfyimgCallback(const cv::Mat img_left,
                            const cv::Mat img_right){
        if(rectificate_){
        cv::Mat intrinsics_left(3, 3, cv::DataType<double>::type);
        cv::Mat dist_coeffs_left(5,1, cv::DataType<double>::type);
        cv::Mat intrinsics_right(3, 3, cv::DataType<double>::type);
        cv::Mat dist_coeffs_right(5,1, cv::DataType<double>::type);
        cv::Mat rotation(3, 3, cv::DataType<double>::type);
        cv::Mat translation(3,1,cv::DataType<double>::type);
        cv::Mat rectification_matrix_left(3,3,cv::DataType<double>::type);
        cv::Mat rectification_matrix_right(3,3,cv::DataType<double>::type);
        cv::Mat R1, R2, P1, P2, Q;
        cv::Size siz;
        

        if(first_rectification){
        
        intrinsics_left.at<double>(0, 0) = left_camera_info.k[0]; //fx
        intrinsics_left.at<double>(0, 2) = left_camera_info.k[2]; //cx
        intrinsics_left.at<double>(1, 1) = left_camera_info.k[4]; //fy
        intrinsics_left.at<double>(1, 2) = left_camera_info.k[5]; //cy
        intrinsics_left.at<double>(2, 2) = 1;

        dist_coeffs_left.at<double>(0) = left_camera_info.d[0];
        dist_coeffs_left.at<double>(1) = left_camera_info.d[1];
        dist_coeffs_left.at<double>(2) = left_camera_info.d[2];
        dist_coeffs_left.at<double>(3) = left_camera_info.d[3];
        dist_coeffs_left.at<double>(4) = left_camera_info.d[4];

        intrinsics_right.at<double>(0, 0) = right_camera_info.k[0]; //fx
        intrinsics_right.at<double>(0, 2) = right_camera_info.k[2]; //cx
        intrinsics_right.at<double>(1, 1) = right_camera_info.k[4]; //fy
        intrinsics_right.at<double>(1, 2) = right_camera_info.k[5]; //cy
        intrinsics_right.at<double>(2, 2) = 1;

        dist_coeffs_right.at<double>(0) = right_camera_info.d[0];
        dist_coeffs_right.at<double>(1) = right_camera_info.d[1];
        dist_coeffs_right.at<double>(2) = right_camera_info.d[2];
        dist_coeffs_right.at<double>(3) = right_camera_info.d[3];
        dist_coeffs_right.at<double>(4) = right_camera_info.d[4];

        siz.width = left_camera_info.width;
        siz.height = left_camera_info.height;

        rotation.at<double>(0, 0) = right_camera_info.p[0];
        rotation.at<double>(0, 1) = right_camera_info.p[1];
        rotation.at<double>(0, 2) = right_camera_info.p[2];
        rotation.at<double>(1, 0) = right_camera_info.p[4];
        rotation.at<double>(1, 1) = right_camera_info.p[5];
        rotation.at<double>(1, 2) = right_camera_info.p[6];
        rotation.at<double>(2, 0) = right_camera_info.p[8];
        rotation.at<double>(2, 1) = right_camera_info.p[9];
        rotation.at<double>(2, 2) = right_camera_info.p[10];


        translation.at<double>(0) = right_camera_info.p[3];
        translation.at<double>(1) = right_camera_info.p[7];
        translation.at<double>(2) = right_camera_info.p[11];

        cv::stereoRectify(intrinsics_left,dist_coeffs_left,intrinsics_right,dist_coeffs_right,siz,rotation,translation,R1,R2,P1,P2,Q);
        
        
        cv::Mat new_cameramatrix_left = cv::getOptimalNewCameraMatrix(intrinsics_left,dist_coeffs_left,siz,1,siz,0);
        cv::Mat new_cameramatrix_right = cv::getOptimalNewCameraMatrix(intrinsics_right,dist_coeffs_right,siz,1,siz,0);
        
        cv::initUndistortRectifyMap(intrinsics_left,dist_coeffs_left,R1,new_cameramatrix_left,siz,CV_32FC1,map1_left,map2_left);
        cv::initUndistortRectifyMap(intrinsics_right,dist_coeffs_right,R1,new_cameramatrix_right,siz,CV_32FC1,map1_right,map2_right);
        
        first_rectification = false;   
        }

        cv::remap(img_left,left_image_rectified,map1_left,map2_left, cv::INTER_LINEAR);
        cv::remap(img_right,right_image_rectified,map1_right,map2_right, cv::INTER_LINEAR);
        left_image = left_image_rectified;
        right_image = right_image_rectified;


        }
    }

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> right_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_rect_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_rect_pub;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_show_window;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_rectificate_image;
    sensor_msgs::msg::CameraInfo left_camera_info;
    sensor_msgs::msg::CameraInfo right_camera_info;
    cv::Mat left_image_rectified;
    cv::Mat right_image_rectified;
    cv::Mat left_image;
    cv::Mat right_image;
    cv::Mat map1_left,map1_right;
    cv::Mat map2_left,map2_right;
    bool show_window_; // Controle da exibição da janela
    bool rectificate_; // Controle da rectificaçao da imagem 
    bool first_rectification = true;
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
