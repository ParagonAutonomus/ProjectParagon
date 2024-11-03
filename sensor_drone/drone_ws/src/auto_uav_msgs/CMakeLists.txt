cmake_minimum_required(VERSION 3.8)
project(auto_uav_msgs)

find_package(ament_cmake REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


# Generate the service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/UpdateNextTarget.srv"
  DEPENDENCIES geographic_msgs
)

ament_package()