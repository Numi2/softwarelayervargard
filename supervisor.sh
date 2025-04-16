#!/usr/bin/env bash
# Supervisor script to launch and monitor all Vargard components
set -e

function cleanup {
  echo "[supervisor] Shutting down services..."
  pkill -f sensor_node || true
  pkill -f inference_node || true
  pkill -f event_manager || true
}
trap cleanup SIGINT SIGTERM

echo "[supervisor] Starting sensor_node..."
ros2 run vargard_sensor_layer sensor_node --ros-args -p config_file:=/workspace/sensors.yaml &
SENSOR_PID=$!

echo "[supervisor] Starting inference_node..."
ros2 run vargard_core inference_node &
INFER_PID=$!

echo "[supervisor] Starting event_manager..."
ros2 run vargard_core event_manager --ros-args -p rules_file:=/workspace/rules.yaml &
EVENT_PID=$!

wait $SENSOR_PID $INFER_PID $EVENT_PID