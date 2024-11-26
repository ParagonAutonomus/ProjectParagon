#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/camera_info.pb.h>

/**
 * @class ThermalCameraBridge
 * @brief Bridge node to convert Gazebo Image messages to ROS Image messages.
 * 
 * @author Brendan Waldrop
 * @date November 2024
 * @version 1.0
 */
class ThermalCameraBridge : public rclcpp::Node {
public:
    ThermalCameraBridge() : Node("thermal_camera_bridge") {
        // ROS publishers
        ros_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/auto_uav/thermal_camera/image", 10);
        ros_camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/auto_uav/thermal_camera/camera_info", 10);

        // Gazebo topic subscribers
        gz_node_.Subscribe("/thermal_camera/image", &ThermalCameraBridge::gz_image_callback, this);
        gz_node_.Subscribe("/thermal_camera/camera_info", &ThermalCameraBridge::gz_camera_info_callback, this);
        
        RCLCPP_INFO(this->get_logger(), "Thermal camera bridge initialized.");
    }

    ~ThermalCameraBridge() noexcept override = default;
private:
    gz::transport::Node gz_node_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ros_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr ros_camera_info_pub_;

    /**
     * @brief Callback for Gazebo image messages.
     * Converts Gazebo Image messages to ROS Image messages.
     * 
     * @param gz_msg
     */
    void gz_image_callback(const gz::msgs::Image &gz_msg) {
        auto ros_msg = std::make_shared<sensor_msgs::msg::Image>();

        ros_msg->header.stamp = this->now();
        ros_msg->header.frame_id = "thermal_camera_frame";

        ros_msg->height = gz_msg.height();
        ros_msg->width = gz_msg.width();

        // Determine the encoding and step size based on the Gazebo image format
        if (gz_msg.pixel_format_type() == gz::msgs::PixelFormatType::RGB_INT8) {
            ros_msg->encoding = "rgb8";
            ros_msg->step = ros_msg->width * 3; // 3 bytes per pixel for RGB
        } else if (gz_msg.pixel_format_type() == gz::msgs::PixelFormatType::L_INT8) {
            ros_msg->encoding = "mono8";
            ros_msg->step = ros_msg->width; // 1 byte per pixel for grayscale
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unsupported pixel format");
            return;
        }

        // Calculate the expected size of the image data
        size_t expected_size = ros_msg->step * ros_msg->height;
        size_t actual_size = gz_msg.data().size();

        if (actual_size != expected_size) {
            RCLCPP_ERROR(this->get_logger(), "Image data size mismatch: expected %zu, got %zu", expected_size, actual_size);
            return;
        }

        // Copy image data
        ros_msg->data.resize(actual_size);
        std::memcpy(ros_msg->data.data(), gz_msg.data().c_str(), actual_size);

        ros_image_pub_->publish(*ros_msg);
    }

    /**
     * @brief Callback for Gazebo camera info messages.
     * Converts Gazebo CameraInfo messages to ROS CameraInfo messages.
     * 
     * @param gz_msg
     */
    void gz_camera_info_callback(const gz::msgs::CameraInfo &gz_msg) {
        auto ros_msg = std::make_shared<sensor_msgs::msg::CameraInfo>();

        ros_msg->header.stamp = this->now();
        ros_msg->header.frame_id = "thermal_camera_frame";

        ros_msg->width = gz_msg.width();
        ros_msg->height = gz_msg.height();
        ros_msg->distortion_model = "plumb_bob";

        ros_msg->d.resize(gz_msg.distortion().k_size());
        for (int i = 0; i < gz_msg.distortion().k_size(); ++i) {
            ros_msg->d[i] = gz_msg.distortion().k(i);
        }

        for (int i = 0; i < 9; ++i) {
            ros_msg->k[i] = gz_msg.intrinsics().k(i);
        }

        for (int i = 0; i < 9; ++i) {
            ros_msg->r[i] = gz_msg.rectification_matrix(i);
        }

        for (int i = 0; i < 12; ++i) {
            ros_msg->p[i] = gz_msg.projection().p(i);
        }

        ros_camera_info_pub_->publish(*ros_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ThermalCameraBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}