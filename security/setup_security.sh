#!/usr/bin/env bash
set -e
echo "Generating SROS2 keystore..."
# Determine script directory
ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
KEYSTORE="$ROOT_DIR/keystore"
# Remove existing keystore, then create a new one
rm -rf "$KEYSTORE"
echo "Creating keystore at $KEYSTORE"
ros2 security create_keystore "$KEYSTORE"
echo "Generating keys for nodes"
# Generate keys for each node (must match ROS2 node names)
ros2 security create_key --keystore "$KEYSTORE" --node-name sensor_layer_node
ros2 security create_key --keystore "$KEYSTORE" --node-name inference_node
ros2 security create_key --keystore "$KEYSTORE" --node-name event_manager
echo "SROS2 keystore successfully generated at $KEYSTORE"