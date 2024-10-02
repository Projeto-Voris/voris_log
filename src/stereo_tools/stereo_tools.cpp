#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

#include "std_msgs/msg/multi_array_dimension.hpp"

#include <sensor_msgs/msg/image.hpp>

#include<opencv2/core/core.hpp>
#include<opencv2/opencv.hpp>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

int numDisparities = 12*16;
int blockSize = 75;
int preFilterType = 0;
int preFilterSize = 25;
int preFilterCap = 15;
int minDisparity = 1;
int textureThreshold = 61;
int uniquenessRatio = 0;
int speckleRange = 50;
int speckleWindowSize = 38;
int disp12MaxDiff = 20;

float lambda = 8000;
float sigma = 1.5;


class StereoTunner : public rclcpp::Node
{
  public:
    StereoTunner()
    : Node("stereo_tunner"), count_(0)
    {
      stereo_params_publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("stereo_params", 10);
      //filter_params_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("filter_params", 10);
      timer_ = this->create_wall_timer(
      100ms, std::bind(&StereoTunner::timer_callback, this));
      create_trackbars();
    }

    void create_trackbars() {
        static const std::string OPENCV_WINDOW_D = "Image D window";
        cv::namedWindow(OPENCV_WINDOW_D,cv::WINDOW_NORMAL);
        cv::resizeWindow(OPENCV_WINDOW_D, 800, 600);
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
    }

  private:
    
    void timer_callback()
    {
        auto stereo_params_message = std_msgs::msg::Int16MultiArray();
        auto filter_params_message = std_msgs::msg::Int16MultiArray();

        stereo_params_message.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());

        stereo_params_message.layout.dim[0].size = 15;
        stereo_params_message.layout.dim[0].label = "lenght";
        stereo_params_message.layout.dim[0].stride = 1;

        stereo_params_message.data.push_back(numDisparities);
        stereo_params_message.data.push_back(blockSize);
        stereo_params_message.data.push_back(preFilterType);
        stereo_params_message.data.push_back(preFilterSize);
        stereo_params_message.data.push_back(preFilterCap);
        stereo_params_message.data.push_back(minDisparity);
        stereo_params_message.data.push_back(textureThreshold);
        stereo_params_message.data.push_back(uniquenessRatio);
        stereo_params_message.data.push_back(numDisparities);
        stereo_params_message.data.push_back(blockSize);
        stereo_params_message.data.push_back(speckleRange);
        stereo_params_message.data.push_back(speckleWindowSize);
        stereo_params_message.data.push_back(disp12MaxDiff);
        stereo_params_message.data.push_back(minDisparity);
        stereo_params_message.data.push_back(minDisparity);

        stereo_params_publisher_->publish(stereo_params_message);
        cv::waitKey(1);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr stereo_params_publisher_;
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