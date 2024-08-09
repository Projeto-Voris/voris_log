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
    this->get_parameter("saving_path", path_);

    // Verify or create the path directories
    verify_path();

    // Initialize the subscribers using the node's interface directly
    left_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "left/image_raw");
    right_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "right/image_raw");

    // Initialize the sync_ object after initializing subscribers
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), *left_sub, *right_sub);

    // Register the callback
    sync_->registerCallback(std::bind(&ImageSaver::imagesCB, this, std::placeholders::_1, std::placeholders::_2));



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
        RCLCPP_ERROR(this->get_logger(), "Directory already exists.");
        rclcpp::shutdown(); // Shutdown the node if the directory cannot be created

    }
}

void ImageSaver::imagesCB(const sensor_msgs::msg::Image::ConstSharedPtr &msgLeft,
                          const sensor_msgs::msg::Image::ConstSharedPtr &msgRight) {
    std::lock_guard<std::mutex> lock(mutex_);

    // char input;
    // std::cin >> input;
    //
    // if (input == 's') {
    //     save_images = true;
    //     // Perform the desired action here
    // } else if (input == 'q') {
    //     RCLCPP_ERROR(this->get_logger(),"Quitting program.");
    //     rclcpp::shutdown(); // Shutdown the node if the directory cannot be created
    // }


    cv::Mat img_left, img_right;
    try {
        img_left = cv_bridge::toCvCopy(msgLeft, "bayer_rggb8")->image;
        img_right = cv_bridge::toCvCopy(msgRight, "bayer_rggb8")->image;
        cv::cvtColor(img_left, img_left, cv::COLOR_BayerRG2BGR);
        cv::cvtColor(img_right, img_right, cv::COLOR_BayerRG2BGR);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    if (visualize) {
        cv::namedWindow("left", cv::WINDOW_NORMAL);
        cv::namedWindow("right", cv::WINDOW_NORMAL);
        cv::resizeWindow("left", 800, 600);
        cv::resizeWindow("right", 800, 600);
        cv::imshow("left", img_left);
        cv::imshow("right", img_right);
        cv::waitKey(10);
    }

    // Save images
    std::string img_counter_;
    if (counter_ < 10) {
        img_counter_ = "00";
    } else if (counter_ >= 10 && counter_ < 100) {
        img_counter_ = "0";
    }
    if (save_images) {
        std::string path_L = path_ + "/left/L" + img_counter_ + std::to_string(counter_) + ".png";
        std::string path_R = path_ + "/right/R" + img_counter_ + std::to_string(counter_) + ".png";
        cv::imwrite(path_L, img_left);
        cv::imwrite(path_R, img_right);
        RCLCPP_INFO(this->get_logger(), "Saved images %s and %s", path_L.c_str(), path_R.c_str());

        counter_++;
    }
}

void ImageSaver::keyLoop() {
    RCLCPP_INFO(this->get_logger(), "Started key loop");

    while (rclcpp::ok()) {
        char key = cv::waitKey(10);

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