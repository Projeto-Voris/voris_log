#ifndef STEREO_IMG_SAVE_HPP
#define STEREO_IMG_SAVE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <thread>
#include <filesystem>

class ImageSaver : public rclcpp::Node {
public:
    ImageSaver();

    ~ImageSaver();

private:
    void verify_path();

    void imagesCB(const sensor_msgs::msg::Image::ConstSharedPtr &msgLeft,
                  const sensor_msgs::msg::Image::ConstSharedPtr &msgRight);

    void keyLoop();

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > left_sub;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > right_sub;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> sync_pol;

    message_filters::Synchronizer<sync_pol> * sync_;

    std::string path_;
    int counter_;
    bool save_images;
    std::mutex mutex_;
    std::thread key_thread_;
};

#endif // STEREO_IMG_SAVE_HPP
