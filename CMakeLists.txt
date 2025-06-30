cmake_minimum_required(VERSION 3.8)
project(vargard_sensor_layer)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "vargard_sensor_layer/msg/SensorFrame.msg"
)

ament_python_install_package(${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()