#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <filesystem>
#include <voris_log/srv/save_images.hpp>

class MultiImageSaver : public rclcpp::Node {
public:
    MultiImageSaver()
        : Node("multi_image_saver"), image_count_(0) {
        // Declare a parameter for the save directory
        this->declare_parameter<std::string>("save_directory", "/home/daniel/Pictures/images");

        // Get the save directory from the parameter
        this->get_parameter("save_directory", path_);

        // Verify or create the path directories
        if (!verify_path(path_, 0)) {
            RCLCPP_ERROR(this->get_logger(), "Error creating directory");
            rclcpp::shutdown();
        }

        // Subscribe to the image topic
        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "flir_camera/image_raw", 10,
            std::bind(&MultiImageSaver::image_callback, this, std::placeholders::_1));

        image_save_srv_ = this->create_service<voris_log::srv::SaveImages>("save_single_image",
            std::bind(&MultiImageSaver::service_cb, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    bool verify_path(const std::string &path_string, int counter = 0) {
        // Check if the path exists
        if (!std::filesystem::exists(path_string)) {
            // Path does not exist, create the directory
            if (std::filesystem::create_directory(path_string)) {
                RCLCPP_INFO(this->get_logger(), "Directory created successfully.");
                return true;

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

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
       image_msg_ = *msg;
    }

    void service_cb(const std::shared_ptr<voris_log::srv::SaveImages::Request> req,
                    std::shared_ptr<voris_log::srv::SaveImages::Response> res) {
        if (req->reset_counter) {
            image_count_ = 0;
        }
        // Convert ROS2 image message to OpenCV image
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(image_msg_, image_msg_.encoding);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Save images
        std::string img_counter_;
        if (image_count_ < 10) {
            img_counter_ = "00";
        } else if (image_count_ >= 10 && image_count_ < 100) {
            img_counter_ = "0";
        }

        // Create a unique filename for each image
        std::string filename = path_ + "/image_" + img_counter_ + std::to_string(image_count_) + ".png";

        // Save the image to the specified path
        cv::imwrite(filename, cv_ptr->image);
        RCLCPP_INFO(this->get_logger(), "Image %d saved at: %s", image_count_, filename.c_str());

        // Increment the image counter
        image_count_++;
        res->result = true;
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Service<voris_log::srv::SaveImages>::SharedPtr image_save_srv_;

    sensor_msgs::msg::Image image_msg_;
    std::string path_;
    int image_count_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiImageSaver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
