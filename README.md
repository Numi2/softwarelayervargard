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

Run the sensor node (with optional parameters for fallback config, calibration, hot‑plug, timestamps, diagnostics):

  ros2 run vargard_sensor_layer sensor_node --ros-args \
    -p config_file:=/path/to/sensors.yaml \
    -p calibration_folder:="/path/to/calibs" \
    -p use_hardware_timestamp:=True \
    -p hotplug_interval:=10.0 \
    -p enable_diagnostics:=True

Parameters:

  - config_file (string, default ""): YAML file for sensor definitions when auto‑detect fails
  - calibration_folder (string, default ""): base directory for camera calibration files
  - use_hardware_timestamp (bool, default False): use sensor‑provided timestamp if available
  - hotplug_interval (float, default 5.0): seconds between auto‑detect scans for hot‑plug/ removal
  - enable_diagnostics (bool, default False): publish sensor health on `/diagnostics`

### Configuration (sensors.yaml)

```yaml
sensors:
  - type: usb_camera
    device_index: 0
    # optional: local calibration YAML (file path relative to calibration_folder or absolute)
    calibration_file: cam0.yaml
    parent_frame: base_link
    extrinsics:
      translation: [0.1, 0.0, 0.2]
      rotation: [0.0, 0.0, 0.0, 1.0]
  - type: csi_camera
    params:
      width: 1920
      height: 1080
      framerate: 30
    calibration_file: csi0.yaml
    parent_frame: base_link
  - type: flir_thermal
    device_path: /dev/video1
  - type: ip_camera
    rtsp_url: rtsp://192.168.1.10/stream
  - type: radar
    port: /dev/ttyUSB0
    baudrate: 115200
```  

Extended configuration fields:
  - calibration_file: camera_info YAML (loaded via camera_info_manager)
  - parent_frame: TF frame ID of the sensor mount
  - extrinsics: pose of sensor frame relative to parent_frame

Topics published:
  - /sensor/<sensor_id> (Image or String)
  - /sensor/<sensor_id>/camera_info (CameraInfo)
  - /diagnostics (DiagnosticArray) [if enable_diagnostics=True]
  - TF:<sensor_id> w.r.t. parent_frame [if extrinsics provided]
