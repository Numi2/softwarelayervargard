#!/usr/bin/env bash
# Supervisor script to launch and monitor all Vargard components
set -e

# OTA update via Docker Compose when image argument is provided
if [ "$#" -gt 0 ]; then
  IMAGE="$1"
  echo "[supervisor] OTA update requested: ${IMAGE}"
  # Ensure docker-compose is available
  if command -v docker-compose >/dev/null 2>&1; then
    COMPOSE_CMD="docker-compose"
  elif command -v docker >/dev/null 2>&1; then
    COMPOSE_CMD="docker compose"
  else
    echo "[supervisor] Error: docker compose not found. Cannot perform OTA update." >&2
    exit 1
  fi
  # Ensure compose file exists
  if [ ! -f docker-compose.yml ]; then
    echo "[supervisor] Error: docker-compose.yml not found in $(pwd)." >&2
    exit 1
  fi
  # Pull latest images and restart containers
  echo "[supervisor] Pulling latest images..."
  $COMPOSE_CMD pull || { echo "[supervisor] docker-compose pull failed." >&2; exit 1; }
  echo "[supervisor] Restarting services..."
  $COMPOSE_CMD up -d || { echo "[supervisor] docker-compose up failed." >&2; exit 1; }
  echo "[supervisor] OTA update complete."
  exit 0
fi

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