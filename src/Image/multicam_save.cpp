#include "multicam_save.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSaver>());
    rclcpp::shutdown();
    return 0;
}

ImageSaver::ImageSaver()
    : Node("image_saver"), save_images(false), counter_(0) {
    // Declare and get the path parameter from the launch file
    this->declare_parameter("saving_path", "~/Pictures/images");
    this->get_parameter("saving_path", path_);

    // Verify or create the path directories
    if (!verify_path(path_, 0)) {
        RCLCPP_ERROR(this->get_logger(), "Error creating directory");
        rclcpp::shutdown();
    }

    // Initialize the subscribers using the node's interface directly
    stereo1_left_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image> >(this, "/stereo1/camera_1/image_raw");
    stereo1_right_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image> >(this, "/stereo1/camera_2/image_raw");

    stereo2_left_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image> >(this, "/stereo2/camera_1/image_raw");
    stereo2_right_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image> >(this, "/stereo2/camera_2/image_raw");

    // Initialize service
    save_image_srv = this->create_service<voris_log::srv::SaveImages>("/save_images",
                                                                      std::bind(&ImageSaver::save_image_cb, this,
                                                                          std::placeholders::_1,
                                                                          std::placeholders::_2));
    // Initialize the sync_ object after initializing subscribers
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy> >(
        SyncPolicy(10), *stereo1_left_sub, *stereo1_right_sub, *stereo2_left_sub, *stereo2_right_sub);

    // Register the callback
    sync_->registerCallback(std::bind(&ImageSaver::images_cb, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
}

ImageSaver::~ImageSaver() {
}

bool ImageSaver::verify_path(const std::string &path_string, int counter = 0) {
    // Check if the path exists
    if (!std::filesystem::exists(path_string)) {
        // Path does not exist, create the directory
        if (std::filesystem::create_directory(path_string)) {
            RCLCPP_INFO(this->get_logger(), "Directory created successfully.");
            std::filesystem::create_directory(path_string + "/S1");
            std::filesystem::create_directory(path_string + "/S2");
            if (std::filesystem::create_directory(path_string + "/S1/left") &&
                std::filesystem::create_directory(path_string + "/S1/right") &&
                std::filesystem::create_directory(path_string + "/S2/left") &&
                std::filesystem::create_directory(path_string + "/S2/right")) {
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

void ImageSaver::images_cb(const sensor_msgs::msg::Image::ConstSharedPtr &msgS1Left,
                           const sensor_msgs::msg::Image::ConstSharedPtr &msgS1Right,
                           const sensor_msgs::msg::Image::ConstSharedPtr &msgS2Left,
                           const sensor_msgs::msg::Image::ConstSharedPtr &msgS2Right) {
    msgS1Right_ = msgS1Right;
    msgS1Left_ = msgS1Left;
    msgS2Left_ = msgS2Left;
    msgS2Right_ = msgS2Right;
}

bool ImageSaver::save_image_cb(const std::shared_ptr<voris_log::srv::SaveImages::Request> &req,
                               const std::shared_ptr<voris_log::srv::SaveImages::Response> &res) {
    if (req->reset_counter) {
        counter_ = 0;
    }

    cv::Mat s1_img_left, s1_img_right, s2_img_left, s2_img_right;
    try {
        s1_img_left = cv_bridge::toCvCopy(msgS1Left_, msgS1Left_->encoding)->image;
        s1_img_right = cv_bridge::toCvCopy(msgS1Right_, msgS1Right_->encoding)->image;
        s2_img_left = cv_bridge::toCvCopy(msgS2Left_, msgS2Left_->encoding)->image;
        s2_img_right = cv_bridge::toCvCopy(msgS2Right_, msgS2Right_->encoding)->image;

        cv::cvtColor(s1_img_left, s1_img_left, cv::COLOR_BayerBG2BGR);
        cv::cvtColor(s1_img_right, s1_img_right, cv::COLOR_BayerBG2BGR);
        cv::cvtColor(s2_img_left, s2_img_left, cv::COLOR_BayerBG2BGR);
        cv::cvtColor(s2_img_right, s2_img_right, cv::COLOR_BayerBG2BGR);

    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        res->message = "cv_bridge exception";
        res->result = false;
        return res->result;
    }
    //


    // Save images
    std::string img_counter_;
    if (counter_ < 10) {
        img_counter_ = "00";
    } else if (counter_ >= 10 && counter_ < 100) {
        img_counter_ = "0";
    }else if(counter_ >= 100 && counter_ < 1000) {
        img_counter_ = "";
    }

    std::string s1_path_L = path_ + "S1/left/L" + img_counter_ + std::to_string(counter_) + ".png";
    std::string s1_path_R = path_ + "S1/right/R" + img_counter_ + std::to_string(counter_) + ".png";
    cv::imwrite(s1_path_L, s1_img_left);
    cv::imwrite(s1_path_R, s1_img_right);
    std::string s2_path_L = path_ + "S2/left/L" + img_counter_ + std::to_string(counter_) + ".png";
    std::string s2_path_R = path_ + "S2/right/R" + img_counter_ + std::to_string(counter_) + ".png";
    cv::imwrite(s2_path_L, s2_img_left);
    cv::imwrite(s2_path_R, s2_img_right);
    RCLCPP_INFO(this->get_logger(), "Saved images %s and %s", s1_path_L.c_str(), s1_path_R.c_str());
    char buffer[50];
    sprintf(buffer, "Images saved n° %i", counter_);
    res->message = buffer;
    res->result = true;
    counter_++;
    return res->result;
}
