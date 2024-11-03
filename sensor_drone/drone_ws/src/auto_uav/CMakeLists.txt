cmake_minimum_required(VERSION 3.8)
project(auto_uav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(auto_uav_msgs REQUIRED)

add_executable(basic_flight_controller src/basic_flight_controller.cpp)
ament_target_dependencies(basic_flight_controller rclcpp mavros_msgs geographic_msgs auto_uav_msgs)

install(TARGETS
  basic_flight_controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
