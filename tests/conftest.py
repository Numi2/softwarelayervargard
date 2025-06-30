"""
Pytest configuration and shared fixtures for Vargard tests.

This module provides common test utilities and fixtures to avoid
code duplication and ensure consistent test setup.
"""

import pytest
import tempfile
import yaml
from pathlib import Path
from typing import Dict, Any
from unittest.mock import Mock
import numpy as np

@pytest.fixture
def temp_dir():
    """Create a temporary directory for test files."""
    with tempfile.TemporaryDirectory() as tmp_dir:
        yield Path(tmp_dir)

@pytest.fixture
def sample_sensor_config():
    """Sample sensor configuration for testing."""
    return {
        "sensors": [
            {
                "id": "test_camera_0",
                "type": "usb_camera",
                "device_index": 0,
                "enabled": True
            },
            {
                "id": "test_camera_1",
                "type": "csi_camera",
                "device_path": "/dev/video1",
                "enabled": False
            }
        ]
    }

@pytest.fixture
def sample_rules_config():
    """Sample rules configuration for testing."""
    return {
        "rules": [
            {
                "id": "person_detection",
                "name": "Person Detection Alert",
                "plugin": "yolov8",
                "enabled": True,
                "conditions": {
                    "class": "person",
                    "confidence_gt": 0.7
                },
                "actions": {
                    "type": "webhook",
                    "url": "http://localhost:8000/alert"
                },
                "cooldown": 30
            },
            {
                "id": "vehicle_detection",
                "name": "Vehicle Detection",
                "plugin": "yolov8",
                "enabled": False,
                "conditions": {
                    "class": "vehicle",
                    "confidence_gt": 0.8
                },
                "actions": {
                    "type": "log",
                    "level": "info"
                },
                "cooldown": 60
            }
        ]
    }

@pytest.fixture
def sample_detection():
    """Sample detection result for testing."""
    return {
        "bbox": [100, 50, 200, 150],
        "confidence": 0.85,
        "class": "person",
        "class_id": 0,
        "metadata": {
            "model": "yolov8n",
            "inference_time": 0.045
        }
    }

@pytest.fixture
def sample_detections(sample_detection):
    """List of sample detections for testing."""
    return [
        sample_detection,
        {
            "bbox": [300, 100, 450, 250],
            "confidence": 0.92,
            "class": "vehicle",
            "class_id": 1,
            "metadata": {
                "model": "yolov8n",
                "inference_time": 0.038
            }
        }
    ]

@pytest.fixture
def mock_image():
    """Mock image data for testing."""
    return np.zeros((480, 640, 3), dtype=np.uint8)

@pytest.fixture
def mock_sensor():
    """Mock sensor object for testing."""
    sensor = Mock()
    sensor.sensor_id = "test_sensor"
    sensor.sensor_type = "usb_camera"
    sensor.is_connected.return_value = True
    sensor.get_frame.return_value = (
        True,
        np.zeros((480,
        640,
        3),
        dtype=np.uint8)
    )
    sensor.get_status.return_value = {
        "status": "healthy",
        "fps": 30.0,
        "frame_count": 1000,
        "error_count": 0
    }
    return sensor

@pytest.fixture
def mock_plugin():
    """Mock inference plugin for testing."""
    plugin = Mock()
    plugin.plugin_id = "test_plugin"
    plugin.plugin_name = "Test Plugin"
    plugin.is_ready.return_value = True
    plugin.process.return_value = []
    plugin.get_config.return_value = {"confidence_threshold": 0.5}
    plugin.get_stats.return_value = {
        "inference_count": 100,
        "average_time": 0.045,
        "error_count": 0
    }
    return plugin

@pytest.fixture
def config_file(temp_dir, sample_sensor_config):
    """Create a temporary config file for testing."""
    config_path = temp_dir / "test_config.yaml"
    with open(config_path, 'w') as f:
        yaml.dump(sample_sensor_config, f)
    return config_path


def create_test_config(config_dict: Dict[str, Any], file_path: Path) -> Path:
    """Utility function to create test configuration files."""
    with open(file_path, 'w') as f:
        yaml.dump(config_dict, f)
    return file_path


def assert_detection_valid(detection: Dict[str, Any]) -> None:
    """Utility function to validate detection format."""
    required_fields = ["bbox", "confidence", "class", "class_id"]
    for field in required_fields:
        assert field in detection, f"Detection missing required field: {field}"

    assert len(detection["bbox"]) == 4, "Bbox should have 4 coordinates"
    assert 0.0 <= detection["confidence"] <= 1.0, "Confidence should be between 0 and 1"
    assert isinstance(detection["class"], str), "Class should be a string"
    assert isinstance(
        detection["class_id"],
        int), "Class ID should be an integer"


def assert_rule_valid(rule: Dict[str, Any]) -> None:
    """Utility function to validate rule format."""
    required_fields = ["id", "name", "plugin", "conditions", "actions"]
    for field in required_fields:
        assert field in rule, f"Rule missing required field: {field}"

    assert isinstance(rule["enabled"], bool), "Enabled should be boolean"
    assert "class" in rule["conditions"], "Rule conditions should specify class"
    assert "type" in rule["actions"], "Rule actions should specify type"


class MockROS2Node:
    """Mock ROS2 node for testing without ROS2 dependencies."""

    def __init__(self, node_name: str):
        self.node_name = node_name
        self.logger = Mock()
        self.timers = []
        self.publishers = {}
        self.subscribers = {}

    def create_timer(self, timer_period, callback):
        timer = Mock()
        timer.timer_period = timer_period
        timer.callback = callback
        self.timers.append(timer)
        return timer

    def create_publisher(self, msg_type, topic, qos_profile):
        publisher = Mock()
        self.publishers[topic] = publisher
        return publisher

    def create_subscription(self, msg_type, topic, callback, qos_profile):
        subscription = Mock()
        self.subscribers[topic] = subscription
        return subscription

    def get_logger(self):
        return self.logger

@pytest.fixture
def mock_ros_node():
    """Mock ROS2 node fixture."""
    return MockROS2Node("test_node")
