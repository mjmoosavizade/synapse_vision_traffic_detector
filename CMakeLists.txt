cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

# Enable both C and CXX (C++) languages for the project.
project(synapse_vision_traffic_detector LANGUAGES C CXX)

# Find necessary ROS 2 packages and external libraries.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc dnn)
find_package(rosidl_default_generators REQUIRED)

# Generate custom ROS 2 message interfaces.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrafficSign.msg"
  DEPENDENCIES std_msgs
)

# Specify directories for header files.
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)

# Define the C++ executable.
add_executable(traffic_sign_detector_node
  src/traffic_sign_detector_node.cpp
)

ament_target_dependencies(traffic_sign_detector_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  builtin_interfaces
)

target_link_libraries(traffic_sign_detector_node
  ${OpenCV_LIBS}
)

# Link against generated message library (typesupport)
rosidl_target_interfaces(traffic_sign_detector_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Install targets and files.
install(TARGETS
  traffic_sign_detector_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch config models
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies for other packages.
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  builtin_interfaces
  OpenCV
)

ament_package()
