# Vargard
  
## Sensor Layer (vargard_sensor_layer)

This ROS 2 package provides a plug-and-play sensor layer for Vargard Core on Jetson Orin Nano. It auto-detects connected sensors (USB, CSI, FLIR thermal, IP, UART radar) and publishes standardized data on ROS 2 topics.

### Building

Install dependencies:

  sudo apt update
  sudo apt install ros-<ros-distro>-rclpy ros-<ros-distro>-sensor-msgs ros-<ros-distro>-cv-bridge python3-opencv python3-serial

Build the package:

  colcon build --packages-select vargard_sensor_layer

Source the workspace:

  source install/setup.bash

### Usage

Run the sensor node (optional `config_file` parameter for YAML fallback):

  ros2 run vargard_sensor_layer sensor_node --ros-args -p config_file:=path/to/sensors.yaml

### Configuration (sensors.yaml)

```yaml
sensors:
  - type: usb_camera
    device_index: 0
  - type: csi_camera
    params:
      width: 1920
      height: 1080
      framerate: 30
  - type: flir_thermal
    device_path: /dev/video1
  - type: ip_camera
    rtsp_url: rtsp://192.168.1.10/stream
  - type: radar
    port: /dev/ttyUSB0
    baudrate: 115200
```  
