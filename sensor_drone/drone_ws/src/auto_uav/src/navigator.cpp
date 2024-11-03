#include <rclcpp/rclcpp.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <auto_uav_msgs/srv/update_next_target.hpp>

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
        current_target_sub_ = this->create_subscription<geographic_msgs::msg::GeoPoseStamped>(
            "/auto_uav/mission_controller/current_waypoint",
            10, std::bind(&Navigator::current_target_callback, this, std::placeholders::_1));

        update_next_target_client_ = this->create_client<auto_uav_msgs::srv::UpdateNextTarget>(
            "/auto_uav/mission_controller/update_next_target");

        global_position_pub_ = this->create_publisher<geographic_msgs::msg::GeoPoseStamped>(
            "/mavros/setpoint_position/global", 10);

        RCLCPP_INFO(this->get_logger(), "Flight path initialized.");
    }
 
private:
    rclcpp::Subscription<geographic_msgs::msg::GeoPoseStamped>::SharedPtr current_target_sub_;
    rclcpp::Client<auto_uav_msgs::srv::UpdateNextTarget>::SharedPtr update_next_target_client_;
    rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr global_position_pub_;

    geographic_msgs::msg::GeoPoseStamped current_target_;

    /**
     * @brief Callback for updating the current target.
     * 
     * @param msg The current target to update.
     */
    void current_target_callback(const geographic_msgs::msg::GeoPoseStamped::SharedPtr msg) {
        current_target_ = *msg;
        RCLCPP_INFO(
            this->get_logger(), 
            "Next target updated: Latitude = %.6f, Longitude = %.6f, Altitude = %.2f",
            current_target_.pose.position.latitude, 
            current_target_.pose.position.longitude, 
            current_target_.pose.position.altitude
        );
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto flight_path = std::make_shared<Navigator>();
    rclcpp::spin(flight_path);
    rclcpp::shutdown();
    return 0;
}