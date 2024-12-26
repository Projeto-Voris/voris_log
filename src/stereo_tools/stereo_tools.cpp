#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <fstream>

using namespace std::chrono_literals;

int numDisparities = 12;
int blockSize = 75;
int preFilterType = 0;
int preFilterSize = 25;
int preFilterCap = 31;
int minDisparity = 1;
int textureThreshold = 61;
int uniquenessRatio = 0;
int speckleRange = 50;
int speckleWindowSize = 38;
int disp12MaxDiff = 20;

int lambda = 8000;
int sigma = 2;

using std::placeholders::_1;

void save_params_to_yaml() {
    // Save the parameters to a .yaml file
    cv::FileStorage fs("stereo_params.yaml", cv::FileStorage::WRITE);

    fs << "numDisparities" << numDisparities*16;
    fs << "blockSize" << blockSize*2+5;
    fs << "preFilterType" << preFilterType;
    fs << "preFilterSize" << preFilterSize*2+5;
    fs << "preFilterCap" << preFilterCap;
    fs << "minDisparity" << minDisparity;
    fs << "textureThreshold" << textureThreshold;
    fs << "uniquenessRatio" << uniquenessRatio;
    fs << "speckleRange" << speckleRange;
    fs << "speckleWindowSize" << speckleWindowSize*2;
    fs << "disp12MaxDiff" << disp12MaxDiff;
    fs << "lambda" << lambda;
    fs << "sigma" << sigma;

    fs.release();
    RCLCPP_INFO(rclcpp::get_logger("stereo_tools"), "Parameters saved to stereo_params.yaml");
}

class StereoTunner : public rclcpp::Node
{
  public:
    StereoTunner()
    : Node("stereo_tools"), count_(0)
    {
      stereo_params_publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("stereo_params", 10);
      filter_params_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("filter_params", 10);
      timer_ = this->create_wall_timer(1000ms, std::bind(&StereoTunner::timer_callback, this));
      disparity_image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("disparity_image", 10, std::bind(&StereoTunner::image_callback, this, _1));
      create_trackbars();
    }

    void create_trackbars() {
        static const std::string OPENCV_WINDOW_D = "Image D window";
        cv::namedWindow(OPENCV_WINDOW_D, cv::WINDOW_NORMAL);
        cv::resizeWindow(OPENCV_WINDOW_D, 800, 600);

        // Trackbars for stereo parameters
        cv::createTrackbar("numDisparities", OPENCV_WINDOW_D, &numDisparities, 18);
        cv::createTrackbar("blockSize", OPENCV_WINDOW_D, &blockSize, 50);
        cv::createTrackbar("preFilterType", OPENCV_WINDOW_D, &preFilterType, 1);
        cv::createTrackbar("preFilterSize", OPENCV_WINDOW_D, &preFilterSize, 25);
        cv::createTrackbar("preFilterCap", OPENCV_WINDOW_D, &preFilterCap, 62);
        cv::createTrackbar("textureThreshold", OPENCV_WINDOW_D, &textureThreshold, 100);
        cv::createTrackbar("uniquenessRatio", OPENCV_WINDOW_D, &uniquenessRatio, 100);
        cv::createTrackbar("speckleRange", OPENCV_WINDOW_D, &speckleRange, 100);
        cv::createTrackbar("speckleWindowSize", OPENCV_WINDOW_D, &speckleWindowSize, 25);
        cv::createTrackbar("disp12MaxDiff", OPENCV_WINDOW_D, &disp12MaxDiff, 25);
        cv::createTrackbar("minDisparity", OPENCV_WINDOW_D, &minDisparity, 25);
        cv::createTrackbar("Lambda", OPENCV_WINDOW_D, &lambda, 25);
        cv::createTrackbar("Sigma", OPENCV_WINDOW_D, &sigma, 25);
    }

  private:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & imgmsg)
    {
        try
        {
            cv_ptrImg = cv_bridge::toCvShare(imgmsg, "mono16");
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        cv::imshow("Disparity Image", cv_ptrImg->image);
        cv::waitKey(1);
    }

    void timer_callback()
    {
        auto stereo_params_message = std_msgs::msg::Int16MultiArray();
        auto filter_params_message = std_msgs::msg::Float32MultiArray();

        stereo_params_message.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        stereo_params_message.layout.dim[0].size = 15;
        stereo_params_message.layout.dim[0].label = "length";
        stereo_params_message.layout.dim[0].stride = 1;

        stereo_params_message.data.push_back(preFilterCap);
        stereo_params_message.data.push_back(preFilterSize * 2 + 5);
        stereo_params_message.data.push_back(preFilterType);
        stereo_params_message.data.push_back(textureThreshold);
        stereo_params_message.data.push_back(uniquenessRatio);
        stereo_params_message.data.push_back(numDisparities * 16);
        stereo_params_message.data.push_back(blockSize * 2 + 5);
        stereo_params_message.data.push_back(speckleRange);
        stereo_params_message.data.push_back(speckleWindowSize * 2);
        stereo_params_message.data.push_back(disp12MaxDiff);
        stereo_params_message.data.push_back(minDisparity);

        filter_params_message.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        filter_params_message.layout.dim[0].size = 2;
        filter_params_message.layout.dim[0].label = "length";
        filter_params_message.layout.dim[0].stride = 1;
        filter_params_message.data.push_back(lambda);
        filter_params_message.data.push_back(sigma);

        stereo_params_publisher_->publish(stereo_params_message);
        filter_params_publisher_->publish(filter_params_message);

        // Check for keyboard input to save parameters
        if (cv::waitKey(1) == 's') {
            save_params_to_yaml();
        }
    }

    cv_bridge::CvImageConstPtr cv_ptrImg;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr stereo_params_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr filter_params_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr disparity_image_subscriber_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StereoTunner>());
  rclcpp::shutdown();
  return 0;
}
