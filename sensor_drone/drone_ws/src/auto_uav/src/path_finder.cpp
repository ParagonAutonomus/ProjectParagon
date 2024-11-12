#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

/**
 * @class PathFinder
 * @brief Path finding calculations and algorithms for drone navigation.
 * 
 * @author Keetra Bippus, Nathan Svoboda 
 * @date November 2024
 * @version 1.0
 */
class PathFinder {
public:
    PathFinder(double step_length, double drone_radius) 
        : step_length_(step_length), drone_radius_(drone_radius) {}

    /**
     * @brief Find the next point in the path to the target.
     * 
     * @param current_position current position of the drone
     * @param target_position target position of the drone
     * @param point_cloud point cloud data from LIDAR sensor
     */
    geographic_msgs::msg::GeoPoseStamped find_next_point(
        const sensor_msgs::msg::NavSatFix &current_position,
        const geographic_msgs::msg::GeoPoseStamped &target_position,
        const sensor_msgs::msg::PointCloud2 &point_cloud
    ) {
        geographic_msgs::msg::GeoPoseStamped next_position;
        next_position.pose.position.latitude = current_position.latitude;
        next_position.pose.position.longitude = current_position.longitude;
        next_position.pose.position.altitude = current_position.altitude;

        // !!! Fill in the rest of the algorithm here

        return next_position;
    }
private:
    double step_length_;
    double drone_radius_;
};