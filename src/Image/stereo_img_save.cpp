#include "stereo_img_save.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSaver>());
    rclcpp::shutdown();
    return 0;
}

ImageSaver::ImageSaver()
    : Node("image_saver"), save_images(false), counter_(0) {
    // Declare and get the path parameter from the launch file
    this->declare_parameter("saving_path", "/tmp/images");
    this->get_parameter("saving_path", path_);

    // Verify or create the path directories
    verify_path();

    // Initialize the subscribers using message filters
    left_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image> >(
        shared_from_this(), "/camera_0/image_raw");
    right_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image> >(
        shared_from_this(), "/camera_1/image_raw");

    // Set up the synchronizer with the ApproximateTime policy
    sync_ = new message_filters::Synchronizer<sync_pol>(sync_pol(10), *left_sub, *right_sub);
    sync_->registerCallback(std::bind(&ImageSaver::imagesCB, this,
  std::placeholders::_1, std::placeholders::_2));

    // Start a separate thread to monitor keyboard input
    key_thread_ = std::thread(&ImageSaver::keyLoop, this);
}

ImageSaver::~ImageSaver() {
    key_thread_.join(); // Ensure the thread is properly joined on destruction
}

void ImageSaver::verify_path() {
    // Check if the path exists
    if (!std::filesystem::exists(path_)) {
        // Path does not exist, create the directory
        if (std::filesystem::create_directory(path_)) {
            RCLCPP_INFO(this->get_logger(), "Directory created successfully.");

            if (std::filesystem::create_directory(path_ + "/left") &&
                std::filesystem::create_directory(path_ + "/right")) {
                RCLCPP_INFO(this->get_logger(), "Left and Right folders created successfully.");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Error creating Left and Right directories.");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Error creating main directory.");
            rclcpp::shutdown(); // Shutdown the node if the directory cannot be created
        }
    } else {
        RCLCPP_INFO(this->get_logger(), "Directory already exists.");
    }
}

void ImageSaver::imagesCB(const sensor_msgs::msg::Image::ConstSharedPtr &msgLeft,
                          const sensor_msgs::msg::Image::ConstSharedPtr &msgRight) {
    std::lock_guard<std::mutex> lock(mutex_);

    cv::Mat img_left, img_right;
    std::string img_counter_ = std::to_string(counter_);

    // Padding zeros for consistent file naming
    while (img_counter_.length() < 3) {
        img_counter_ = "0" + img_counter_;
    }

    try {
        img_left = cv_bridge::toCvCopy(msgLeft, "bgr8")->image;
        img_right = cv_bridge::toCvCopy(msgRight, "bgr8")->image;
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (save_images) {
        std::string path_L = path_ + "/left/L" + img_counter_ + ".png";
        std::string path_R = path_ + "/right/R" + img_counter_ + ".png";
        cv::imwrite(path_L, img_left);
        cv::imwrite(path_R, img_right);
        RCLCPP_INFO(this->get_logger(), "Image pair %03d saved", counter_);
        counter_++;
    }
}

void ImageSaver::keyLoop() {
    while (rclcpp::ok()) {
        char key = cv::waitKey(100);
        if (key == 's') {
            std::lock_guard<std::mutex> lock(mutex_);
            save_images = true;
            RCLCPP_INFO(this->get_logger(), "Saving next images...");
        } else if (key == 'q') {
            RCLCPP_INFO(this->get_logger(), "Exiting...");
            rclcpp::shutdown();
        } else {
            std::lock_guard<std::mutex> lock(mutex_);
            save_images = false;
        }
    }
}
