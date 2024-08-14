#include "stereo_img_save.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSaver>());
    rclcpp::shutdown();
    return 0;
}

ImageSaver::ImageSaver()
    : Node("image_saver"), save_images(false), counter_(0), visualize(true) {
    // Declare and get the path parameter from the launch file
    this->declare_parameter("saving_path", "/home/daniel/Pictures/images");
    this->declare_parameter("visualize", false);
    this->get_parameter("saving_path", path_);
    this->get_parameter("visualize", visualize);

    // Verify or create the path directories
    verify_path();

    // Initialize the subscribers using the node's interface directly
    left_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image> >(this, "left/image_raw");
    right_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image> >(this, "right/image_raw");

    // list_folder_srv = create_service<voris_log::srv::ListFolders>("list_folders",
    //                                                               std::bind(&ImageSaver::list_folder_cb, this,
    //                                                                         std::placeholders::_1,
    //                                                                         std::placeholders::_2));
    save_image_srv = this->create_service<voris_log::srv::SaveImages>("/save_images",
                                                                      std::bind(&ImageSaver::save_image_cb, this,
                                                                          std::placeholders::_1,
                                                                          std::placeholders::_2));
    // Initialize the sync_ object after initializing subscribers
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy> >(
        SyncPolicy(10), *left_sub, *right_sub);

    // Register the callback
    sync_->registerCallback(std::bind(&ImageSaver::images_cb, this, std::placeholders::_1, std::placeholders::_2));


    // Start a separate thread to monitor keyboard input
}

ImageSaver::~ImageSaver() {

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
        RCLCPP_ERROR(this->get_logger(), "Directory already exists.");
        path_ = path_ + "-1";
        verify_path();
        // rclcpp::shutdown(); // Shutdown the node if the directory cannot be created
    }
}

void ImageSaver::images_cb(const sensor_msgs::msg::Image::ConstSharedPtr &msgLeft,
                           const sensor_msgs::msg::Image::ConstSharedPtr &msgRight) {
    msgRight_ = msgRight;
    msgLeft_ = msgLeft;
}

bool ImageSaver::save_image_cb(const std::shared_ptr<voris_log::srv::SaveImages::Request> &req,
                               const std::shared_ptr<voris_log::srv::SaveImages::Response> &res) {
    if (req->reset_counter) {
        counter_ = 0;
    }

    cv::Mat img_left, img_right;
    try {
        img_left = cv_bridge::toCvCopy(msgLeft_, msgLeft_->encoding)->image;
        img_right = cv_bridge::toCvCopy(msgRight_, msgRight_->encoding)->image;
        cv::cvtColor(img_left, img_left, cv::COLOR_BayerBG2BGR);
        cv::cvtColor(img_right, img_right, cv::COLOR_BayerBG2BGR);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        res->result = false;
        return false;
    }
    //


    // Save images
    std::string img_counter_;
    if (counter_ < 10) {
        img_counter_ = "00";
    } else if (counter_ >= 10 && counter_ < 100) {
        img_counter_ = "0";
    }

    std::string path_L = path_ + "/left/L" + img_counter_ + std::to_string(counter_) + ".png";
    std::string path_R = path_ + "/right/R" + img_counter_ + std::to_string(counter_) + ".png";
    cv::imwrite(path_L, img_left);
    cv::imwrite(path_R, img_right);
    RCLCPP_INFO(this->get_logger(), "Saved images %s and %s", path_L.c_str(), path_R.c_str());
    counter_++;
    res->result = true;
    return true;
}
