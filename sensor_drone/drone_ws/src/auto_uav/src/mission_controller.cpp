#include <rclcpp/rclcpp.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <auto_uav_msgs/srv/update_next_target.hpp>
#include <auto_uav_msgs/srv/set_mission_state.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <queue>

enum DroneState {
    IDLE = 0,
    TAKEOFF = 1,
    READY = 2,
    MOVING = 3,
    WAITING = 4,
    LANDING = 5
};

/**
 * @class MissionController
 * @brief Mission controller node to manage the drone's missions.
 * 
 * @author Brendan Waldrop
 * @date November 2024
 * @version 1.0
 */
class MissionController : public rclcpp::Node {
public:
    /**
     * @brief Constructor for MissionController.
     */
    MissionController() : Node("mission_controller"), current_state_(DroneState::IDLE) {
        current_target_pub_ = this->create_publisher<geographic_msgs::msg::GeoPoseStamped>("/auto_uav/mission_controller/current_target", 10);
        current_state_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/auto_uav/mission_controller/state", 10);

        set_state_service_ = this->create_service<auto_uav_msgs::srv::SetMissionState>(
            "/auto_uav/mission_controller/set_state",
            std::bind(&MissionController::set_state_callback, this, std::placeholders::_1, std::placeholders::_2));

        update_next_target_service_ = this->create_service<auto_uav_msgs::srv::UpdateNextTarget>(
            "/auto_uav/mission_controller/update_next_target",
            std::bind(&MissionController::update_next_target_callback, this, std::placeholders::_1, std::placeholders::_2));
        
        state_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MissionController::publish_current_state, this));
        target_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MissionController::publish_current_target, this));

        RCLCPP_INFO(this->get_logger(), "Mission controller initialized.");
    }

    /**
     * @brief Add a waypoint to the mission queue. TESTING ONLY
     * 
     * @param waypoint The waypoint to add to the mission queue.
     */
    void add_waypoint(const geographic_msgs::msg::GeoPoseStamped& waypoint) {
        waypoint_queue_.push(waypoint);
    }

private:
    std::queue<geographic_msgs::msg::GeoPoseStamped> waypoint_queue_;
    geographic_msgs::msg::GeoPoseStamped current_target_;
    uint8_t current_state_;

    rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr current_target_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr current_state_pub_;
    rclcpp::Service<auto_uav_msgs::srv::UpdateNextTarget>::SharedPtr update_next_target_service_;
    rclcpp::Service<auto_uav_msgs::srv::SetMissionState>::SharedPtr set_state_service_;
    rclcpp::TimerBase::SharedPtr target_timer_;
    rclcpp::TimerBase::SharedPtr state_timer_;

    /**
     * @brief Publish the current waypoint to the current waypoint topic.
     */
    void publish_current_target() {
        current_target_pub_->publish(current_target_);
    }

    void publish_current_state() {
        std_msgs::msg::UInt8 state_msg;
        state_msg.data = current_state_;
        current_state_pub_->publish(state_msg);
    }

    /**
     * @brief Callback for updating the next target.
     * 
     * @param request Service request (update_next_target) to update the next target.
     * @param response Service response (update_next_target) to indicate success or failure.
     */
    void update_next_target_callback(const std::shared_ptr<auto_uav_msgs::srv::UpdateNextTarget::Request> request,
                                     std::shared_ptr<auto_uav_msgs::srv::UpdateNextTarget::Response> response) {
        if (waypoint_queue_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No waypoints in queue.");
            response->success = false;
            return;
        }

        current_target_ = waypoint_queue_.front();
        waypoint_queue_.pop();
        publish_current_target();

        RCLCPP_INFO(
            this->get_logger(), 
            "Next target updated: Latitude = %.6f, Longitude = %.6f, Altitude = %.2f", 
            current_target_.pose.position.latitude, 
            current_target_.pose.position.longitude, 
            current_target_.pose.position.altitude
        );
        response->success = true;
    }

    void set_state_callback(const std::shared_ptr<auto_uav_msgs::srv::SetMissionState::Request> request,
                            std::shared_ptr<auto_uav_msgs::srv::SetMissionState::Response> response) {
        current_state_ = request->state;
        publish_current_state();

        response->success = true;
        response->message = "State updated successfully.";
        RCLCPP_INFO(this->get_logger(), "State set to %u", current_state_);
    }
};

/**
 * @brief Main function for the mission controller node.
 * 
 * Initializes the node and executor for the mission controller.
 */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto mission_controller = std::make_shared<MissionController>();

    // test points
    geographic_msgs::msg::GeoPoseStamped waypoint1;
    waypoint1.pose.position.latitude = -35.36284814;
    waypoint1.pose.position.longitude = 149.16516125;
    waypoint1.pose.position.altitude = 600;
    mission_controller->add_waypoint(waypoint1);

    geographic_msgs::msg::GeoPoseStamped waypoint2;
    waypoint2.pose.position.latitude = -35.36275018;
    waypoint2.pose.position.longitude = 149.16559226;
    waypoint2.pose.position.altitude = 600;
    mission_controller->add_waypoint(waypoint2);

    rclcpp::spin(mission_controller);
    rclcpp::shutdown();
    return 0;
}