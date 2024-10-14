#include "rclcpp/rclcpp.hpp"
#include "geographic_msgs/msg/geo_point_stamped.hpp"

class TestController: public rclcpp::Node {
public:
    TestController(): Node("test_controller") {
        rclcpp::QoS qos_settings(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
        qos_settings.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

        gps_subscription_ = this->create_subscription<geographic_msgs::msg::GeoPointStamped>(
            "/ap/gps_global_origin/filtered", qos_settings,
            std::bind(&TestController::topic_callback, this, std::placeholders::_1));
    }
private:
    void topic_callback(const geographic_msgs::msg::GeoPointStamped::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), 
        "Current location: Latitude: %f, Longitude: %f, Altitude: %f",
        msg->position.latitude, msg->position.longitude, msg->position.altitude);
    }
    rclcpp::Subscription<geographic_msgs::msg::GeoPointStamped>::SharedPtr gps_subscription_;

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestController>());
    rclcpp::shutdown();
    return 0;
}