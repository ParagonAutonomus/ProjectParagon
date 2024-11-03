#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <chrono>

/**
 * @class BasicFlightController
 * @brief Basic flight controller that interacts with MAVROS services.
 * 
 * @author Brendan Waldrop
 * @date October 2024
 * @version 1.0
 */
class BasicFlightController : public rclcpp::Node {
public:
    /**
     * @brief Constructor for BasicFlightController.
     * 
     * Initializes the node, creates clients for MAVROS services, and creates a service for triggering flight.
     */
    BasicFlightController() : Node("basic_flight_controller") {
        client_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        service_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode", rmw_qos_profile_services_default, client_callback_group_);
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming", rmw_qos_profile_services_default, client_callback_group_);
        takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff", rmw_qos_profile_services_default, client_callback_group_);
        flight_service_ = this->create_service<mavros_msgs::srv::CommandBool>(
            "/auto_uav/basic_flight_controller/trigger_flight", 
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

    /**
     * @brief Callback for triggering the flight sequence.
     * 
     * @param request Service request (command_bool) to trigger the flight sequence.
     * @param response Service response (command_bool) to indicate success or failure.
     */
    void trigger_flight_callback(const std::shared_ptr<mavros_msgs::srv::CommandBool::Request> request,
                                 std::shared_ptr<mavros_msgs::srv::CommandBool::Response> response) {
        if (request->value) {
            RCLCPP_INFO(this->get_logger(), "Flight sequence initiated.");

            // Set mode to GUIDED
            if (!set_mode("GUIDED")) {
                RCLCPP_ERROR(this->get_logger(), "Failed to set mode to GUIDED.");
                response->success = false;
                return;
            }

            // Arm throttle
            if (!arm_throttle()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to arm throttle.");
                response->success = false;
                return;
            }

            // Takeoff 10m
            if (!takeoff(10)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to takeoff.");
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

    /**
     * @brief Set the mode of the drone.
     * 
     * @param mode The mode to set the drone to. (e.g. GUIDED)
     * @return true if the mode was set successfully, false otherwise.
     */
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

    /**
     * @brief Arm the throttle of the drone.
     * 
     * @return true if the throttle was armed successfully, false otherwise.
     */
    bool arm_throttle() {
        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = true;

        RCLCPP_INFO(this->get_logger(), "Arming throttle.");

        auto future = arming_client_->async_send_request(request);
        auto result = future.wait_for(std::chrono::seconds(5));

        if (result == std::future_status::timeout) {
            RCLCPP_ERROR(this->get_logger(), "Failed to arm throttle: timeout.");
            return false;
        }

        auto response = future.get();
        if (!response->success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to arm throttle: not successful.");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Throttle armed.");
        return true;
    }

    /**
     * @brief Takeoff the drone to a specified altitude.
     * 
     * @param altitude The altitude to takeoff to.
     * @return true if the drone took off successfully, false otherwise.
     */
    bool takeoff(float altitude) {
        auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        request->min_pitch = 0.0;
        request->yaw = 0.0;
        request->latitude = 0.0;
        request->longitude = 0.0;
        request->altitude = altitude;

        RCLCPP_INFO(this->get_logger(), "Taking off.");

        auto future = takeoff_client_->async_send_request(request);
        auto result = future.wait_for(std::chrono::seconds(5));

        if (result == std::future_status::timeout) {
            RCLCPP_ERROR(this->get_logger(), "Failed to takeoff: timeout.");
            return false;
        }

        auto response = future.get();
        if (!response->success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to takeoff: not successful.");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Takeoff successful.");
        return true;
    }
};

/**
 * @brief Main function for the basic flight controller node.
 * 
 * Initializes the node and executor for the basic flight controller.
 */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<BasicFlightController>();
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}