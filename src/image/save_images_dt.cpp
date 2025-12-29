#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <chrono>

using namespace std::chrono_literals;

class ImageSaverNode : public rclcpp::Node
{
public:
    ImageSaverNode()
        : Node("image_saver_node"), save_interval_(200ms), counter_(0)
    {
        this->declare_parameter("saving_path", "~/Pictures/images");
        this->declare_parameter("save_rate", 5); // Default save rate
        int save_rate;
        this->get_parameter("save_rate", save_rate);
        this->get_parameter("saving_path", path_);
    
        // Verify or create the path directories
        if (!verify_path(path_, 0)) {
            RCLCPP_ERROR(this->get_logger(), "Error creating directory");
            rclcpp::shutdown();
        }
        save_interval_ = std::chrono::milliseconds(static_cast<int>(1000.0 / save_rate));


        image1_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/camera_1/image_raw");
        image2_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/camera_2/image_raw");
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *image1_sub_, *image2_sub_);
        sync_->registerCallback(std::bind(&ImageSaverNode::imageCallback, this, std::placeholders::_1, std::placeholders::_2));

        // Timer for periodic saving
        timer_ = this->create_wall_timer(save_interval_, std::bind(&ImageSaverNode::saveImages, this));

        RCLCPP_INFO(this->get_logger(), "ImageSaverNode started.");
    }

private:

bool verify_path(const std::string &path_string, int counter = 0) {
    // Check if the path exists
    if (!std::filesystem::exists(path_string)) {
        // Path does not exist, create the directory
        if (std::filesystem::create_directory(path_string)) {
            RCLCPP_INFO(this->get_logger(), "Directory created successfully.");
            
            if (std::filesystem::create_directory(path_string + "/left") &&
            std::filesystem::create_directory(path_string + "/right")) {
                RCLCPP_INFO(this->get_logger(), "Left and Right folders created successfully.");
                path_ = path_string;
                return true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Error creating Left and Right directories.");
                return false;
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Error creating main directory.");
            return false;
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Directory '%s' already exists.", path_string.c_str());
        counter++;
        RCLCPP_INFO(this->get_logger(), "Increase counter to %i", counter);
        
        // Create a new path by appending the counter
        std::string new_path = path_string + "_" + std::to_string(counter);
        
        return verify_path(new_path, counter);
    }
}

void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &image1, const sensor_msgs::msg::Image::ConstSharedPtr &image2)
{
    std::lock_guard<std::mutex> lock(mutex_);
    last_image1_ = image1;
    last_image2_ = image2;
}

void saveImages()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!last_image1_ || !last_image2_)
    {
        RCLCPP_WARN(this->get_logger(), "No synchronized images available to save.");
        return;
    }
    
    cv::Mat img_left, img_right;
    try {
        img_left = cv_bridge::toCvCopy(last_image1_, last_image1_->encoding)->image;
        img_right = cv_bridge::toCvCopy(last_image2_, last_image2_->encoding)->image;
        cv::cvtColor(img_left, img_left, cv::COLOR_BayerBG2BGR);
        cv::cvtColor(img_right, img_right, cv::COLOR_BayerBG2BGR);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
        
    // Save images
    std::string img_counter_;
    if (counter_ < 10) {
        img_counter_ = "00";
    } else if (counter_ >= 10 && counter_ < 100) {
        img_counter_ = "0";
    } else if (counter_ >= 100 && counter_ < 1000) {
        img_counter_ = "";
    }

    std::string path_L = path_ + "/left/L" + img_counter_ + std::to_string(counter_) + ".png";
    std::string path_R = path_ + "/right/R" + img_counter_ + std::to_string(counter_) + ".png";
    cv::imwrite(path_L, img_left);
    cv::imwrite(path_R, img_right);
    RCLCPP_INFO(this->get_logger(), "Saved images %s and %s", path_L.c_str(), path_R.c_str());
    counter_++;

    // Clear the last images to wait for new ones
    last_image1_.reset();
    last_image2_.reset();
}

using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
// Parameters
std::string path_;
std::chrono::milliseconds save_interval_;

// Message filters
std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>  image1_sub_;
std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>  image2_sub_;
std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

// Synchronized images
sensor_msgs::msg::Image::ConstSharedPtr last_image1_;
sensor_msgs::msg::Image::ConstSharedPtr last_image2_;
std::mutex mutex_;

// Timer
rclcpp::TimerBase::SharedPtr timer_;

// Image counter
size_t counter_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSaverNode>());
    rclcpp::shutdown();
    return 0;
}
