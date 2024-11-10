#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs/pointcloud_packed.pb.h>

class LidarBridge : public rclcpp::Node {
public:
    LidarBridge() : Node("lidar_bridge") {
        lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/auto_uav/lidar", 10);
        gz_node_.Subscribe("lidar/points", &LidarBridge::lidar_callback, this);
    }

    ~LidarBridge() noexcept override = default;

private:
    gz::transport::Node gz_node_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;

    void lidar_callback(const gz::msgs::PointCloudPacked &gz_msg) {
        auto ros_msg = convert_to_ros(gz_msg);
        lidar_pub_->publish(ros_msg);
    }

    sensor_msgs::msg::PointCloud2 convert_to_ros(const gz::msgs::PointCloudPacked &gz_msg) {
        sensor_msgs::msg::PointCloud2 ros_msg;
        
        ros_msg.header.stamp = this->now();
        ros_msg.header.frame_id = "lidar";

        ros_msg.height = 1;
        ros_msg.width = gz_msg.width();
        ros_msg.is_dense = false;
        ros_msg.is_bigendian = false;
        ros_msg.point_step = gz_msg.point_step();
        ros_msg.row_step = gz_msg.point_step() * gz_msg.width();

        for (const auto &gz_field : gz_msg.field()) {
            sensor_msgs::msg::PointField ros_field;
            ros_field.name = gz_field.name();
            ros_field.offset = gz_field.offset();
            ros_field.datatype = gz_field.datatype();
            ros_field.count = gz_field.count();
            ros_msg.fields.push_back(ros_field);
        }

        ros_msg.data.resize(gz_msg.data().size());
        std::copy(gz_msg.data().begin(), gz_msg.data().end(), ros_msg.data.begin());

        return ros_msg;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto lidar_bridge = std::make_shared<LidarBridge>();
    rclcpp::spin(lidar_bridge);
    rclcpp::shutdown();
    return 0;
}