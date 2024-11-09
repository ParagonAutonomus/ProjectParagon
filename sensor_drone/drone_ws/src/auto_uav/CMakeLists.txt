cmake_minimum_required(VERSION 3.8)
project(auto_uav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(auto_uav_msgs REQUIRED)

# Add the basic_flight_controller executable
add_executable(basic_flight_controller src/basic_flight_controller.cpp)
ament_target_dependencies(basic_flight_controller rclcpp mavros_msgs geographic_msgs auto_uav_msgs)

# Add the mission_controller executable
add_executable(mission_controller src/mission_controller.cpp)
ament_target_dependencies(mission_controller rclcpp geographic_msgs auto_uav_msgs sensor_msgs)

# Add the navigator executable
add_executable(navigator src/navigator.cpp) 
ament_target_dependencies(navigator rclcpp geographic_msgs auto_uav_msgs sensor_msgs)

# Install all executables
install(TARGETS
  basic_flight_controller
  mission_controller
  navigator
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