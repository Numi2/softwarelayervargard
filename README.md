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
  
## Security & Reliability (Sprint 4)

We’ve enhanced the system with SROS2 (DDS) security, container hardening, and runtime diagnostics.

### DDS Security

1. Generate a new keystore (node certificates) for each component:

   ```bash
   bash security/setup_security.sh
   ```

   This creates `security/keystore` containing keys for:
   - sensor_layer_node
   - inference_node
   - event_manager

2. Launch your nodes with the enclave argument:

   ```bash
   ros2 run vargard_sensor_layer sensor_node \
     --ros-args --enclave security/keystore \
     -p enable_diagnostics:=True

   ros2 run vargard_core inference_node \
     --ros-args --enclave security/keystore \
     -p enable_diagnostics:=True

   ros2 run vargard_core event_manager \
     --ros-args -p rules_file:=rules.yaml \
     --enclave security/keystore -p enable_diagnostics:=True
   ```

### Container Hardening

- All containers run as a non-root user `vargard` and drop unnecessary Linux capabilities.
- Logs are written in structured JSON to `/var/log/vargard/` inside each container:
  - sensor_node.log
  - inference_node.log
  - event_manager.log

### Healthchecks & Diagnostics

- When `enable_diagnostics` is `True`, each node publishes a `DiagnosticArray` on `/diagnostics` every second, reporting node heartbeat.
- The provided `docker-compose.yml` includes healthchecks that verify:
  - sensor topics (`/sensor/*`) are active
  - inference events (`/vargard/events/*`) are active
  - alerts topic (`/vargard/alerts`) is active

## Telemetry & OTA (Sprint 5)

### Telemetry Agent

A new ROS 2 node `telemetry_node` gathers CPU, memory, temperature, network, and GPU stats and publishes them to MQTT:

- Location: `vargard_core/telemetry_node.py`
- Run with:
  ```bash
  ros2 run vargard_core telemetry_node \
    --ros-args \
    -p mqtt_broker:=<broker> -p mqtt_port:=<port> \
    -p tls_ca_cert:=<ca.pem> -p tls_certfile:=<cert.pem> \
    -p tls_keyfile:=<key.pem> \
    -p telemetry_topic:=vargard/telemetry \
    -p config_topic:=vargard/config \
    -p publish_interval:=5.0
  ```

### Remote Config & OTA

`telemetry_node` also listens on MQTT topic `vargard/config`:

- `"sensors.yaml"` key: new sensors configuration YAML, written to `sensors.yaml`
- `"update_image"` key: Docker image tag, triggers `/workspace/supervisor.sh <image>`

### Dashboard Stub

A basic Flask app under `dashboard/` subscribes to telemetry and alerts and exposes:

- GET `/telemetry`: last N telemetry messages (`?n=10`)
- GET `/alerts`: last N alerts (`?n=10`)
- SSE streams:
  - `/telemetry/stream`
  - `/alerts/stream`

To run the dashboard:
```bash
cd dashboard
pip3 install -r requirements.txt
python3 app.py
```

## Management CLI (Sprint 6)

The `vargardctl` command provides easy orchestration of the Vargard stack:

- `vargardctl list-sensors` - Show sensors from sensors.yaml or auto-detected.
- `vargardctl list-plugins` - List registered inference plugins.
- `vargardctl start` - Start the stack using Docker Compose.
- `vargardctl stop` - Stop the stack.
- `vargardctl logs` - Stream container logs.

Install the CLI via pip:
```bash
pip install vargard_core
```

Run `vargardctl --help` for usage information.

## Web UI (Sprint 6 - Next.js)

A React-based dashboard built with Next.js is available in `web/`:

```bash
cd web
npm install
npm run dev
```

Open http://localhost:3000 to view real-time telemetry and alerts.

## End-to-End Deployment Tutorial

This guide walks through deploying the Vargard AI Brain on a Jetson Orin device, covering sensor setup, inference, event management, and alerting.

### Prerequisites
- NVIDIA Jetson Orin with JetPack 5.x and Ubuntu
- ROS 2 (e.g., Humble) installed and sourced
- Docker & Docker Compose installed
- Python 3.8+ with pip
- Git

1. Clone the Repository
   ```bash
   git clone https://github.com/your-org/vargard.git
   cd vargard
   ```

2. Install System Dependencies
   ```bash
   sudo apt update
   sudo apt install \
     python3-opencv python3-serial \
     ros-humble-rclpy ros-humble-sensor-msgs ros-humble-cv-bridge \
     docker.io docker-compose
   ```

3. Build the Sensor Layer
   ```bash
   cd vargard_sensor_layer
   colcon build --packages-select vargard_sensor_layer
   source install/setup.bash
   ```

4. Run the Sensor Node
   ```bash
   ros2 run vargard_sensor_layer sensor_node \
     --ros-args -p config_file:=../sensors.yaml \
                -p enable_diagnostics:=True
   ```

5. Build Core Packages
   ```bash
   cd ../vargard_core
   colcon build --packages-select vargard_core
   source install/setup.bash
   ```

6. Generate DDS Security Keys (optional)
   ```bash
   bash security/setup_security.sh
   ```
   This creates certificates under `security/keystore` for each node.

7. Launch Inference and Event Manager
   ```bash
   # Inference node
   ros2 run vargard_core inference_node \
     --ros-args --enclave security/keystore \
                -p model_path:=/path/to/model.onnx

   # Event manager
   ros2 run vargard_core event_manager \
     --ros-args --enclave security/keystore \
                -p rules_file:=../rules.yaml
   ```

8. Deploy with Docker Compose (optional)
   ```bash
   cd ..
   docker-compose up -d
   ```
   This spins up sensor, inference, and event containers with healthchecks, logs, and security enabled.

9. Use the CLI
   ```bash
   vargardctl list-sensors
   vargardctl list-plugins
   vargardctl logs
   ```

10. View the Web UI
    ```bash
    cd web
    npm install
    npm run dev
    ```
    Open http://localhost:3000 to monitor real-time telemetry and alerts.

## Documentation

Comprehensive documentation is available in `docs/`. To build the docs locally:

```bash
cd docs
pip install sphinx
make html
```

The generated HTML site will be in `docs/_build/html`.
