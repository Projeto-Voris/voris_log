#ifndef STEREO_IMG_SAVE_HPP
#define STEREO_IMG_SAVE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include "voris_log/srv/save_images.hpp"

class ImageSaver : public rclcpp::Node {
public:
    ImageSaver();

    ~ImageSaver();

private:
    bool verify_path(const std::string& path_string, int counter);

    void images_cb(const sensor_msgs::msg::Image::ConstSharedPtr &msgLeft,
                  const sensor_msgs::msg::Image::ConstSharedPtr &msgRight);


    bool save_image_cb(const std::shared_ptr<voris_log::srv::SaveImages::Request> &req,
                        const std::shared_ptr<voris_log::srv::SaveImages::Response> &res);


    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > left_sub;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > right_sub;

    rclcpp::Service<voris_log::srv::SaveImages>::SharedPtr save_image_srv;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;

    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;


    sensor_msgs::msg::Image::ConstSharedPtr msgLeft_, msgRight_;
    std::string path_;
    int counter_;
    bool save_images;

};

#endif // STEREO_IMG_SAVE_HPP
