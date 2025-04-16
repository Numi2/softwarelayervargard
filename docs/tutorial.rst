End-to-End Deployment Tutorial
==================================

This guide walks through deploying the Vargard AI Brain on a Jetson Orin device,
covering sensor setup, inference, event management, and alerting.

Prerequisites
-------------

- NVIDIA Jetson Orin with JetPack 5.x and Ubuntu
- ROS 2 (e.g., Humble) installed and sourced
- Docker & Docker Compose installed
- Python 3.8+ with pip
- Git

1. Clone the Repository
------------------------

::

  git clone https://github.com/your-org/vargard.git
  cd vargard

2. Install System Dependencies
------------------------------

On the Jetson device:

::

  sudo apt update
  sudo apt install \
    python3-opencv python3-serial \
    ros-humble-rclpy ros-humble-sensor-msgs ros-humble-cv-bridge \
    docker.io docker-compose

3. Build the Sensor Layer
-------------------------

::

  cd vargard_sensor_layer
  colcon build --packages-select vargard_sensor_layer
  source install/setup.bash

4. Run the Sensor Node
----------------------

::

  ros2 run vargard_sensor_layer sensor_node \
    --ros-args -p config_file:=../sensors.yaml \
               -p enable_diagnostics:=True

5. Build Core Packages
----------------------

::

  cd ../vargard_core
  colcon build --packages-select vargard_core
  source install/setup.bash

6. Generate DDS Security Keys (optional)
---------------------------------------

::

  bash security/setup_security.sh

This creates certificates under `security/keystore` for each node.

7. Launch Inference and Event Manager
-------------------------------------

::

  # Inference node
  ros2 run vargard_core inference_node \
    --ros-args --enclave security/keystore \
               -p model_path:=/path/to/model.onnx

  # Event manager
  ros2 run vargard_core event_manager \
    --ros-args --enclave security/keystore \
               -p rules_file:=../rules.yaml

8. Deploy with Docker Compose (optional)
----------------------------------------

::

  cd ..
  docker-compose up -d

This spins up sensor, inference, and event containers with healthchecks, logs,
and security enabled.

9. Use the CLI
--------------

::

  vargardctl list-sensors
  vargardctl list-plugins
  vargardctl logs

10. View the Web UI
-------------------

Start the Next.js dashboard:

::

  cd web
  npm install
  npm run dev

Open http://localhost:3000 to monitor real-time telemetry and alerts.

Congratulations! You now have an end-to-end Vargard AI Brain deployment.