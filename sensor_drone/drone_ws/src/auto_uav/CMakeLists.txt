cmake_minimum_required(VERSION 3.8)
project(auto_uav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(auto_uav_msgs REQUIRED)
find_package(gz-transport13 REQUIRED)

# Add the basic_flight_controller executable
add_executable(basic_flight_controller src/basic_flight_controller.cpp)
ament_target_dependencies(basic_flight_controller rclcpp mavros_msgs geographic_msgs auto_uav_msgs)

# Add the mission_controller executable
add_executable(mission_controller src/mission_controller.cpp)
ament_target_dependencies(mission_controller rclcpp mavros_msgs geographic_msgs auto_uav_msgs sensor_msgs)

# Add the navigator executable
add_executable(navigator src/navigator.cpp) 
ament_target_dependencies(navigator rclcpp mavros_msgs geographic_msgs auto_uav_msgs sensor_msgs nav_msgs geometry_msgs)

# Add the LidarBridge executable
add_executable(lidar_bridge src/lidar_bridge.cpp) 
ament_target_dependencies(lidar_bridge rclcpp sensor_msgs gz-transport13)

# Add the ThermalCameraBridge executable
add_executable(thermal_camera_bridge src/thermal_camera_bridge.cpp)
ament_target_dependencies(thermal_camera_bridge rclcpp sensor_msgs gz-transport13)

# Create library target
add_library(path_finder src/path_finder.cpp)
ament_target_dependencies(path_finder rclcpp mavros_msgs geographic_msgs sensor_msgs)
target_link_libraries(navigator path_finder)

# Install all executables
install(TARGETS
  basic_flight_controller
  mission_controller
  navigator
  lidar_bridge
  thermal_camera_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()