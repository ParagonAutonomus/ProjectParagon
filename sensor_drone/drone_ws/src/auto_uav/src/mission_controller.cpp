#include <rclcpp/rclcpp.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <queue>

/**
 * @class MissionController
 * @brief A mission controller node to manage the drone's missions.
 * 
 * @author Brendan Waldrop
 * @date November 2024
 * @version 1.0
 */
class MissionController : public rclcpp::Node {
public:
    MissionController() : Node("mission_controller") {

    }

private:
    std::queue<geographic_msgs::msg::GeoPoseStamped> waypoing_queue_;
    geographic_msgs::msg::GeoPoseStamped current_waypoint_;

    rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr current_waypoint_pub_;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto mission_controller = std::make_shared<MissionController>();
    rclcpp::spin(mission_controller);
    rclcpp::shutdown();
    return 0;
}