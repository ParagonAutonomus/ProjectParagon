#include <rclcpp/rclcpp.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <auto_uav_msgs/srv/update_next_target.hpp>
#include <auto_uav_msgs/srv/set_mission_state.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include "drone_state.hpp"

/**
 * @class Navigator
 * @brief Navigator node to manage the drone's flight path and execution of missions.
 * 
 * @author Brendan Waldrop
 * @date November 2024
 * @version 1.0
 */
class Navigator : public rclcpp::Node {
public:
    /**
     * @brief Constructor for Navigator.
     */
    Navigator() : Node("navigator") {
        client_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        current_target_sub_ = this->create_subscription<geographic_msgs::msg::GeoPoseStamped>(
            "/auto_uav/mission_controller/current_target",
            10, std::bind(&Navigator::current_target_callback, this, std::placeholders::_1));
        rclcpp::QoS qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::BestEffort);
        current_position_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/mavros/global_position/global",
            qos, std::bind(&Navigator::current_position_callback, this, std::placeholders::_1));
        current_state_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/auto_uav/mission_controller/state",
            10, std::bind(&Navigator::current_state_callback, this, std::placeholders::_1));

        set_state_client_ = this->create_client<auto_uav_msgs::srv::SetMissionState>(
            "/auto_uav/mission_controller/set_state", 
            rmw_qos_profile_services_default, 
            client_callback_group_);
        update_next_target_client_ = this->create_client<auto_uav_msgs::srv::UpdateNextTarget>(
            "/auto_uav/mission_controller/update_next_target", 
            rmw_qos_profile_services_default, 
            client_callback_group_);

        set_global_position_pub_ = this->create_publisher<geographic_msgs::msg::GeoPoseStamped>(
            "/mavros/setpoint_position/global", 10);

        RCLCPP_INFO(this->get_logger(), "Flight path initialized.");
    }
 
private:
    rclcpp::Subscription<geographic_msgs::msg::GeoPoseStamped>::SharedPtr current_target_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr current_position_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr current_state_sub_;
    
    rclcpp::Client<auto_uav_msgs::srv::SetMissionState>::SharedPtr set_state_client_;
    rclcpp::Client<auto_uav_msgs::srv::UpdateNextTarget>::SharedPtr update_next_target_client_;
    
    rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr set_global_position_pub_;

    rclcpp::CallbackGroup::SharedPtr client_callback_group_;

    geographic_msgs::msg::GeoPoseStamped current_target_;
    sensor_msgs::msg::NavSatFix current_position_;

    void current_target_callback(const geographic_msgs::msg::GeoPoseStamped::SharedPtr msg) {
        current_target_ = *msg;
    }

    void current_position_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        current_position_ = *msg;
    }

    void current_state_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
        if (msg->data == DroneState::READY) {
            RCLCPP_INFO(this->get_logger(), "Drone is READY, updating next target.");
        
            auto update_request = std::make_shared<auto_uav_msgs::srv::UpdateNextTarget::Request>();
            
            auto update_future = update_next_target_client_->async_send_request(update_request);
            auto update_result = update_future.wait_for(std::chrono::seconds(5));

            if (update_result == std::future_status::timeout) {
                RCLCPP_ERROR(this->get_logger(), "Failed to update next target: timeout.");
                return;
            }

            auto update_response = update_future.get();
            if (update_response->success) {
                RCLCPP_INFO(this->get_logger(), "Next target updated successfully. Setting state to MOVING.");
                
                if (!set_state(DroneState::MOVING)) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to set state to MOVING.");
                }

            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to update the next target. Setting state back to WAITING.");

                if (!set_state(DroneState::WAITING)) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to set state to WAITING.");
                }
            }
        }
    }

    bool set_state(DroneState state) {
        auto request = std::make_shared<auto_uav_msgs::srv::SetMissionState::Request>();
        request->state = state;

        auto future = set_state_client_->async_send_request(request);
        auto result = future.wait_for(std::chrono::seconds(5));

        if (result == std::future_status::timeout) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set state: timeout.");
            return false;
        }

        auto response = future.get();
        if (response->success) {
            return true;
        } else {
            return false;
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<Navigator>();
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}