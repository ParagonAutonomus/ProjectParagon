#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs/laserscan.pb.h>

/**
 * @class LidarBridge
 * @brief Bridge node to convert Gazebo LaserScan messages to ROS PointCloud2 messages.
 * 
 * @author Brendan Waldrop
 * @date November 2024
 * @version 1.0
 */
class LidarBridge : public rclcpp::Node {
public:
    LidarBridge() : Node("lidar_bridge") {
        ros_lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/auto_uav/lidar", 10);
        gz_node_.Subscribe("/lidar", &LidarBridge::gz_lidar_callback, this);
        RCLCPP_INFO(this->get_logger(), "Lidar bridge initialized.");
    }

    ~LidarBridge() noexcept override = default;

private:
    gz::transport::Node gz_node_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ros_lidar_pub_;

    /**
     * @brief Callback for Gazebo lidar messages.
     * Converts Gazebo LaserScan messages to ROS PointCloud2 messages.
     * 
     * @param gz_msg
     */
    void gz_lidar_callback(const gz::msgs::LaserScan &gz_msg) {
        sensor_msgs::msg::PointCloud2 ros_msg;
        ros_msg.header.stamp = this->now();
        ros_msg.header.frame_id = "lidar_frame"; 

        ros_msg.height = 1;
        ros_msg.width = gz_msg.count();
        ros_msg.is_dense = false;
        ros_msg.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(ros_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(gz_msg.count());

        sensor_msgs::PointCloud2Iterator<float> iter_x(ros_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(ros_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(ros_msg, "z");

        float angle = gz_msg.angle_min();

        for (google::protobuf::uint32 i = 0; i < gz_msg.count(); i++, angle += gz_msg.angle_step()) {
            float range = gz_msg.ranges(i);

            if (std::isfinite(range)) {
                *iter_x = range * std::cos(angle);
                *iter_y = range * std::sin(angle);
                *iter_z = 0.0;
            } else {
                *iter_x = std::numeric_limits<float>::quiet_NaN();
                *iter_y = std::numeric_limits<float>::quiet_NaN();
                *iter_z = std::numeric_limits<float>::quiet_NaN();
            }

            ++iter_x;
            ++iter_y;
            ++iter_z;
        }

        ros_lidar_pub_->publish(ros_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto lidar_bridge = std::make_shared<LidarBridge>();
    rclcpp::spin(lidar_bridge);
    rclcpp::shutdown();
    return 0;
}