Tutorial 2 - Jetson Orin Quickstart with USB Webcam
===================================================

This tutorial walks you through setting up an NVIDIA Jetson Orin device, building the Vargard Sensor Layer, and testing it with a standard USB webcam.

Prerequisites
-------------
- A host Ubuntu machine (18.04/20.04/22.04) with NVIDIA SDK Manager installed
- An NVIDIA Jetson Orin device (Nano or AGX) and appropriate storage (microSD or eMMC)
- USB keyboard, mouse, HDMI display or SSH access over network
- A USB webcam (e.g., Logitech C270)

1. Flash Jetson Orin with JetPack
---------------------------------
1. On your host machine, install NVIDIA SDK Manager::

       sudo apt update
       sudo apt install nvidia-sdk-manager

2. Launch SDK Manager and follow the prompts to flash JetPack 5.x (include CUDA, cuDNN, TensorRT, Multimedia). Select your Jetson model and install Desktop/CLI components.

3. After flashing, power on the Jetson and log in (username `nvidia` / password `nvidia`) or connect via SSH.

2. Initial Software Setup on Jetson
-----------------------------------
1. Update package lists and upgrade the OS::

       sudo apt update && sudo apt upgrade -y

2. (Optional) Add your user to the `sudo` group for passwordless sudo privileges::

       sudo usermod -aG sudo $USER

3. Install common tools and Python 3 pip::

       sudo apt install git python3-pip curl gnupg2 lsb-release

3. Install ROS 2 and Dependencies
---------------------------------
1. Add the ROS	2 apt repository and key::

       sudo apt update && sudo apt install curl gnupg2 lsb-release
       curl -sSL http://repo.ros2.org/repos.key | sudo apt-key add -
       sudo sh -c 'echo "deb [arch=arm64] http://repo.ros2.org/ubuntu/main $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'
       sudo apt update

2. Install ROS	2 Humble Desktop and build tools::

       sudo apt install ros-humble-desktop python3-rosdep python3-colcon-common-extensions

3. Initialize rosdep and update::

       sudo rosdep init
       rosdep update

4. Install sensor and camera dependencies::

       sudo apt install python3-opencv python3-serial ros-humble-cv-bridge ros-humble-sensor-msgs ros-humble-camera-info-manager

5. (Optional) Install Docker and Docker Compose::

       sudo apt install docker.io docker-compose
       sudo usermod -aG docker $USER
       newgrp docker

4. Clone the Vargard Repository
-------------------------------
1. Create a workspace and clone the repo::

       mkdir -p ~/vargard_ws/src && cd ~/vargard_ws/src
       git clone https://github.com/your-org/vargard.git

2. Return to the workspace root::

       cd ~/vargard_ws

5. Build and Source the Sensor Layer
------------------------------------
1. Source ROS	2 environment::

       source /opt/ros/humble/setup.bash

2. Build the `vargard_sensor_layer` package::

       colcon build --packages-select vargard_sensor_layer --cmake-args -DCMAKE_BUILD_TYPE=Release

3. Source the local workspace::

       source install/setup.bash

6. Connect and Verify Your USB Webcam
-------------------------------------
1. Plug in your USB webcam to a free USB port.
2. Verify the device node exists::

       ls /dev/video*

   You should see `/dev/video0` (or `video1`, etc.).
3. (Optional) Test the camera with `cheese` or `v4l2-ctl`::

       sudo apt install v4l-utils cheese
       v4l2-ctl --list-formats-ext -d /dev/video0
       cheese --device=/dev/video0

7. Run the Sensor Node
----------------------
1. Launch the sensor node to auto-detect and publish your webcam feed::

       ros2 run vargard_sensor_layer sensor_node \\
         --ros-args \\
           -p use_hardware_timestamp:=False \\
           -p enable_diagnostics:=True \\
           -p hotplug_interval:=5.0

2. Confirm the node is running::

       ros2 node list

   You should see `/sensor_layer_node`.

8. Verify and Visualize the Camera Feed
---------------------------------------
1. List available topics::

       ros2 topic list

   Look for `/sensor/usb_camera_0` and `/sensor/usb_camera_0/camera_info`.
2. Echo the camera info topic::

       ros2 topic echo /sensor/usb_camera_0/camera_info

3. Display the live image stream::
   - Using `image_tools` (install via `sudo apt install ros-humble-image-tools` if needed) ::

       ros2 run image_tools showimage --ros-args -r image:=/sensor/usb_camera_0

   - Or using `rqt_image_view` ::

       ros2 run rqt_image_view rqt_image_view /sensor/usb_camera_0

Congratulations!
---------------

You’ve now streamed live video from your USB webcam into ROS 2 using the Vargard Sensor Layer on your Jetson Orin device. Next, explore inference and event-management tutorials to build complete perception pipelines.