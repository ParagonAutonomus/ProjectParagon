#include <rclcpp/rclcpp.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <auto_uav_msgs/srv/update_next_target.hpp>
#include <auto_uav_msgs/srv/set_mission_state.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <chrono>
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
     * @brief Constructor for navigator node.
     */
    Navigator() : Node("navigator"), position_margin_(19.5) {
        // callback group for service clients
        client_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // topic subscribers
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

        // service clients
        set_state_client_ = this->create_client<auto_uav_msgs::srv::SetMissionState>(
            "/auto_uav/mission_controller/set_state", 
            rmw_qos_profile_services_default, client_callback_group_);
        update_next_target_client_ = this->create_client<auto_uav_msgs::srv::UpdateNextTarget>(
            "/auto_uav/mission_controller/update_next_target", 
            rmw_qos_profile_services_default, client_callback_group_);
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode", 
            rmw_qos_profile_services_default, client_callback_group_);
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming", 
            rmw_qos_profile_services_default, client_callback_group_);
        takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff", 
            rmw_qos_profile_services_default, client_callback_group_);

        // publishers
        set_global_position_pub_ = this->create_publisher<geographic_msgs::msg::GeoPoseStamped>(
            "/mavros/setpoint_position/global", 10);

        RCLCPP_INFO(this->get_logger(), "Flight path initialized.");
    }
 
private:
    // subscribers
    rclcpp::Subscription<geographic_msgs::msg::GeoPoseStamped>::SharedPtr current_target_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr current_position_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr current_state_sub_;

    // clients
    rclcpp::Client<auto_uav_msgs::srv::SetMissionState>::SharedPtr set_state_client_;
    rclcpp::Client<auto_uav_msgs::srv::UpdateNextTarget>::SharedPtr update_next_target_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    
    // publishers
    rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr set_global_position_pub_;

    // callback groups
    rclcpp::CallbackGroup::SharedPtr client_callback_group_;

    geographic_msgs::msg::GeoPoseStamped current_target_;
    sensor_msgs::msg::NavSatFix current_position_;
    uint8_t current_state_;
    double position_margin_;

    /**
     * @brief Callback for current target subscriber.
     * 
     * @param msg
     */
    void current_target_callback(const geographic_msgs::msg::GeoPoseStamped::SharedPtr msg) {
        current_target_ = *msg;
    }

    /**
     * @brief Callback for current position subscriber; change state to WAITING if target reached.
     * 
     * @param msg
     */
    void current_position_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        current_position_ = *msg;
        if (current_state_ == DroneState::MOVING) {
            if (is_target_reached()) {
                RCLCPP_INFO(this->get_logger(), "Target reached. Setting state to WAITING.");
                if (!set_state(DroneState::WAITING)) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to set state to WAITING.");
                }
            }
        }
    }

    /**
     * @brief Check if the drone has reached the target.
     * 
     * @return true if the drone has reached the target within margin, false otherwise
     */
    bool is_target_reached() {
        double distance = calculate_distance(
            current_position_.latitude, 
            current_position_.longitude, 
            current_position_.altitude, 
            current_target_.pose.position.latitude, 
            current_target_.pose.position.longitude, 
            current_target_.pose.position.altitude
        );

        RCLCPP_INFO(this->get_logger(), "Distance to target: %.2f", distance);
        return  distance < position_margin_;
    }

    /**
     * @brief Calculate the distance between two points.
     * 
     * @param lat1 latitude of point 1
     * @param lon1 longitude of point 1
     * @param alt1 altitude of point 1
     * @param lat2 latitude of point 2
     * @param lon2 longitude of point 2
     * @param alt2 altitude of point 2
     * @return the distance between the two points
     */
    double calculate_distance(double lat1, double lon1, double alt1, double lat2, double lon2, double alt2) {
        double dist_lat = lat2 - lat1;
        double dist_lon = lon2 - lon1;
        double dist_alt = alt2 - alt1;

        // !FIX! innacurate distance conversions
        double horizontal_dist = sqrt(pow(dist_lat * 111139, 2) + pow(dist_lon * 111139, 2));
        return sqrt(pow(horizontal_dist, 2) + pow(dist_alt, 2));
    }

    /**
     * @brief Callback for current state subscriber; check if the drone is READY and executes the next target.
     * 
     * @param msg
     */
    void current_state_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
        current_state_ = msg->data;
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
                
                current_target_ = update_response->updated_target;
                RCLCPP_INFO(
                    this->get_logger(), 
                    "Next target updated: Latitude = %.6f, Longitude = %.6f, Altitude = %.2f", 
                    current_target_.pose.position.latitude, 
                    current_target_.pose.position.longitude, 
                    current_target_.pose.position.altitude
                );

                if (!set_state(DroneState::MOVING)) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to set state to MOVING.");
                }

                set_global_position_pub_->publish(current_target_);

            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to update the next target. Setting state back to WAITING.");

                if (!set_state(DroneState::WAITING)) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to set state to WAITING.");
                }
            }
        }
    }

    /**
     * @brief Set the state of the mission controller.
     * 
     * @param state the state to set
     * @return true if the state was set successfully, false otherwise
     */
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

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<Navigator>();
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}