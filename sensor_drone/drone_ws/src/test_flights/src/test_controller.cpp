#include "rclcpp/rclcpp.hpp"
#include "geographic_msgs/msg/geo_point_stamped.hpp"
#include "ardupilot_msgs/srv/mode_switch.hpp"
#include <chrono>
#include <memory>

class TestController: public rclcpp::Node {
public:
    TestController(): Node("test_controller") {
        // QoS profile
        rclcpp::QoS qos_settings(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
        qos_settings.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

        // subscriptions
        gps_subscription_ = this->create_subscription<geographic_msgs::msg::GeoPointStamped>(
            "/ap/gps_global_origin/filtered", qos_settings,
            std::bind(&TestController::topic_callback, this, std::placeholders::_1));

        // clients
        mode_switch_client_ = this->create_client<ardupilot_msgs::srv::ModeSwitch>("/ap/mode_switch");
        
        this->switch_mode();
    }
private:
    void topic_callback(const geographic_msgs::msg::GeoPointStamped::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), 
        "Current location: Latitude: %f, Longitude: %f, Altitude: %f",
        msg->position.latitude, msg->position.longitude, msg->position.altitude);
    }

    void switch_mode() {
        if (!mode_switch_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Mode switch service not available");
            return;
        }

        auto request = std::make_shared<ardupilot_msgs::srv::ModeSwitch::Request>();
        request->mode = 4; // GUIDED mode
        
        auto result = mode_switch_client_->async_send_request(request);
        auto future_result = mode_switch_client_->async_send_request(request,
            std::bind(&TestController::handle_mode_switch_response, this, std::placeholders::_1));
    }

    void handle_mode_switch_response(rclcpp::Client<ardupilot_msgs::srv::ModeSwitch>::SharedFuture future) {
        auto response = future.get();
        if (response->status) {
            RCLCPP_INFO(this->get_logger(), "Mode successfully switched to GUIDED");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to switch mode");
        }
    }
    
    rclcpp::Subscription<geographic_msgs::msg::GeoPointStamped>::SharedPtr gps_subscription_;
    rclcpp::Client<ardupilot_msgs::srv::ModeSwitch>::SharedPtr mode_switch_client_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestController>());
    rclcpp::shutdown();
    return 0;
}