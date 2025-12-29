#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>  // Import the standard service
#include <voris_log/srv/save_images.hpp> // Import your custom service

using namespace std::chrono_literals;

class MultiServiceNode : public rclcpp::Node {
public:
    MultiServiceNode(int max_requests, int period_sec)
        : Node("multi_service_node"), max_requests_(max_requests), request_count_(0) {
        // Create two clients for the services
        auto period = std::chrono::duration<double>(period_sec);

        client1_ = this->create_client<std_srvs::srv::SetBool>("/pattern_change");
        client2_ = this->create_client<voris_log::srv::SaveImages>("/save_images");

        // Create a service provided by this node
        this->service_ = this->create_service<std_srvs::srv::SetBool>(
            "multi_service_trigger",
            std::bind(&MultiServiceNode::handle_request, this, std::placeholders::_1, std::placeholders::_2)
        );

        // Create a timer to periodically call both services every 3 seconds
        timer_ = this->create_wall_timer(period, std::bind(&MultiServiceNode::periodic_service_requests, this)
        );
    }

private:
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client1_;
    rclcpp::Client<voris_log::srv::SaveImages>::SharedPtr client2_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr timer_;

    int max_requests_; // Maximum number of requests to send
    int request_count_; // Counter for the number of requests sent

    void handle_request(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Multi-service trigger received");

        // Reset the request count and restart the timer when the service is called
        request_count_ = 0;
        timer_->reset(); // Restart the timer

        // Respond to the service request
        response->success = true;
    }

    void periodic_service_requests() {
        if (request_count_ < max_requests_) {
            RCLCPP_INFO(this->get_logger(), "Periodic service request triggered.");
            call_both_services();
            request_count_++;
        } else {
            RCLCPP_INFO(this->get_logger(), "Reached maximum number of requests: %d", max_requests_);
            timer_->cancel(); // Stop the timer after reaching the maximum number of requests
        }
    }

    void call_both_services() {
        // Create requests for both services
        auto req1 = std::make_shared<std_srvs::srv::SetBool::Request>();
        req1->data = true; // Example request data

        auto req2 = std::make_shared<voris_log::srv::SaveImages::Request>();
        req2->reset_counter = false; // Example request data

        // Check if service clients are available
        if (!client1_->wait_for_service(1s) || !client2_->wait_for_service(1s)) {
            RCLCPP_ERROR(this->get_logger(), "One or more services are not available.");
            return;
        }

        // Asynchronously send requests and attach callbacks directly
        auto future1 = client1_->async_send_request(
            req1, [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
                try {
                    RCLCPP_INFO(this->get_logger(), "Service 1 response: %s", future.get()->message.c_str());
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Service 1 call failed: %s", e.what());
                }
            });

        auto future2 = client2_->async_send_request(
            req2, [this](rclcpp::Client<voris_log::srv::SaveImages>::SharedFuture future) {
                try {
                    RCLCPP_INFO(this->get_logger(), "Service 2 response received.");
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Service 2 call failed: %s", e.what());
                }
            });
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Specify the number of times the services should be requested
    int max_requests = 100; // Example: 5 requests
    int period = 6;
    auto node = std::make_shared<MultiServiceNode>(max_requests, period);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
