#include <rclcpp/rclcpp.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <auto_uav_msgs/srv/update_next_target.hpp>
#include <auto_uav_msgs/srv/set_mission_state.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <queue>
#include "drone_state.hpp"

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
     * @brief Constructor for mission controller node.
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
        waiting_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MissionController::check_for_waypoints, this));
        waiting_timer_->cancel();

        RCLCPP_INFO(this->get_logger(), "Mission controller initialized.");
    }

    /**
     * @brief !TESTING ONLY! Add a waypoint to the mission queue.
     * 
     * @param waypoint
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
    rclcpp::TimerBase::SharedPtr waiting_timer_;

    /**
     * @brief Publish the current target.
     */
    void publish_current_target() {
        current_target_pub_->publish(current_target_);
    }

    /**
     * @brief Publish the current state.
     */
    void publish_current_state() {
        std_msgs::msg::UInt8 state_msg;
        state_msg.data = current_state_;
        current_state_pub_->publish(state_msg);
    }

    /**
     * @brief Update the next target in the waypoint queue.
     * 
     * @param request
     * @param response
     */
    void update_next_target_callback(const std::shared_ptr<auto_uav_msgs::srv::UpdateNextTarget::Request>,
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
        response->updated_target = current_target_;
    }

    /**
     * @brief Set the current state of the drone.
     * 
     * @param request
     * @param response
     */
    void set_state_callback(const std::shared_ptr<auto_uav_msgs::srv::SetMissionState::Request> request,
                            std::shared_ptr<auto_uav_msgs::srv::SetMissionState::Response> response) {
        current_state_ = request->state;
        publish_current_state();

        // start polling for waypoints if WAITING
        if (current_state_ == DroneState::WAITING) {
            waiting_timer_->reset();
        } else {
            waiting_timer_->cancel();
        }

        response->success = true;
        response->message = "State updated successfully.";
        RCLCPP_INFO(this->get_logger(), "State set to %s.", drone_state_to_string(static_cast<DroneState>(current_state_)).c_str());
    }

    /**
     * @brief Check for waypoints in the queue.
     */
    void check_for_waypoints() {
        RCLCPP_INFO(this->get_logger(), "Checking for waypoints...");
        if (current_state_ == DroneState::WAITING && !waypoint_queue_.empty()) {
            current_state_ = DroneState::READY;
            publish_current_state();
            waiting_timer_->cancel();
            RCLCPP_INFO(this->get_logger(), "Waypoint found. Drone set to READY.");
        }
    }
};

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

    geographic_msgs::msg::GeoPoseStamped waypoint3;
    waypoint3.pose.position.latitude = -35.36290000;
    waypoint3.pose.position.longitude = 149.16520000;
    waypoint3.pose.position.altitude = 600;
    mission_controller->add_waypoint(waypoint3);

    geographic_msgs::msg::GeoPoseStamped waypoint4;
    waypoint4.pose.position.latitude = -35.36270000;
    waypoint4.pose.position.longitude = 149.16530000;
    waypoint4.pose.position.altitude = 600;
    mission_controller->add_waypoint(waypoint4);

    geographic_msgs::msg::GeoPoseStamped waypoint5;
    waypoint5.pose.position.latitude = -35.36282000;
    waypoint5.pose.position.longitude = 149.16516000;
    waypoint5.pose.position.altitude = 600;
    mission_controller->add_waypoint(waypoint5);

    geographic_msgs::msg::GeoPoseStamped waypoint6;
    waypoint6.pose.position.latitude = -35.36278000;
    waypoint6.pose.position.longitude = 149.16525000;
    waypoint6.pose.position.altitude = 600;
    mission_controller->add_waypoint(waypoint6);

    rclcpp::spin(mission_controller);
    rclcpp::shutdown();
    return 0;
}