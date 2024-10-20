#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <chrono>

class BasicFlightController :  public rclcpp::Node {
public:
    BasicFlightController() : Node("basic_flight_controller") {
        client_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        service_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode", rmw_qos_profile_services_default, client_callback_group_);
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming", rmw_qos_profile_services_default, client_callback_group_);
        takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff", rmw_qos_profile_services_default, client_callback_group_);
        flight_service_ = this->create_service<mavros_msgs::srv::CommandBool>(
            "trigger_flight", 
            std::bind(&BasicFlightController::trigger_flight_callback, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default, service_callback_group_);
    }

private:
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::Service<mavros_msgs::srv::CommandBool>::SharedPtr flight_service_;

    rclcpp::CallbackGroup::SharedPtr client_callback_group_;
    rclcpp::CallbackGroup::SharedPtr service_callback_group_;

    void trigger_flight_callback(const std::shared_ptr<mavros_msgs::srv::CommandBool::Request> request,
                                 std::shared_ptr<mavros_msgs::srv::CommandBool::Response> response) {
        if (request->value) {
            RCLCPP_INFO(this->get_logger(), "Flight sequence initiated.");
            if (!set_mode("GUIDED")) {
                RCLCPP_ERROR(this->get_logger(), "Failed to set mode to GUIDED.");
                response->success = false;
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Flight sequence completed.");
            response->success = true;
        } else {
            RCLCPP_WARN(this->get_logger(), "Flight sequence aborted.");
            response->success = false;
        }
    }

    bool set_mode(const std::string& mode) {
        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->base_mode = 0;
        request->custom_mode = mode;

        RCLCPP_INFO(this->get_logger(), "Setting mode to %s", mode.c_str());

        auto future = set_mode_client_->async_send_request(request);
        auto result = future.wait_for(std::chrono::seconds(5));
        
        if (result == std::future_status::timeout) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set mode: timeout.");
            return false;
        }

        auto response = future.get();
        if (!response->mode_sent) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set mode: mode not sent.");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Mode set to %s", mode.c_str());
        return true;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<BasicFlightController>();
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}