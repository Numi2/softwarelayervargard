# Installing Dependencies for Vargard

This project has both standard Python dependencies and ROS 2 specific dependencies.

## Option 1: Full Installation (with ROS 2)

If you have ROS 2 installed:

```bash
# 1. Source your ROS 2 installation
source /opt/ros/<your-ros-distro>/setup.bash

# 2. Install ROS 2 Python dependencies
sudo apt update
sudo apt install -y \
    ros-$ROS_DISTRO-camera-info-manager-py \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-diagnostic-updater \
    ros-$ROS_DISTRO-diagnostic-msgs \
    ros-$ROS_DISTRO-sensor-msgs

# 3. Install Python dependencies
pip install -r requirements-no-ros.txt
```

## Option 2: Minimal Installation (without ROS 2)

For development or testing without full ROS 2:

```bash
# Install only non-ROS dependencies
pip install -r requirements-no-ros.txt
```

Note: Some features like camera calibration management will use fallback implementations.

## Option 3: Docker Installation

The recommended approach is to use the provided Docker setup:

```bash
docker-compose up -d
```

This handles all dependencies automatically.

## Troubleshooting

### camera-info-manager not found

If you see this error:
```
ERROR: Could not find a version that satisfies the requirement camera-info-manager>=0.2.0
```

This is because `camera-info-manager` is a ROS 2 package, not a PyPI package. Either:
1. Install it via apt as shown in Option 1
2. Use the minimal installation (Option 2) - the code will use a fallback implementation
3. Use Docker (Option 3)

### Other ROS 2 dependencies

Similar issues may occur with other ROS 2 packages. They should be installed via apt:
- `ros-<distro>-rclpy`
- `ros-<distro>-sensor-msgs`
- `ros-<distro>-std-msgs`

Replace `<distro>` with your ROS 2 distribution (e.g., humble, iron, rolling).