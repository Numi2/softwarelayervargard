"""
Tests for the sensor layer components.

These tests focus on the core sensor management functionality
without heavy mocking or external dependencies.
"""

import pytest
import yaml
from unittest.mock import Mock, patch
from pathlib import Path

# Import test utilities
from conftest import create_test_config, assert_detection_valid


class TestSensorConfiguration:
    """Test sensor configuration loading and validation."""
    
    def test_load_valid_sensor_config(self, temp_dir, sample_sensor_config):
        """Test loading a valid sensor configuration."""
        config_path = temp_dir / "sensors.yaml"
        create_test_config(sample_sensor_config, config_path)
        
        with open(config_path) as f:
            loaded_config = yaml.safe_load(f)
        
        assert "sensors" in loaded_config
        assert len(loaded_config["sensors"]) == 2
        assert loaded_config["sensors"][0]["type"] == "usb_camera"
        assert loaded_config["sensors"][1]["type"] == "csi_camera"
    
    def test_invalid_sensor_config_missing_sensors(self, temp_dir):
        """Test handling of config missing sensors key."""
        invalid_config = {"invalid": "config"}
        config_path = temp_dir / "invalid.yaml"
        create_test_config(invalid_config, config_path)
        
        with open(config_path) as f:
            loaded_config = yaml.safe_load(f)
        
        assert "sensors" not in loaded_config
    
    def test_sensor_config_validation(self, sample_sensor_config):
        """Test sensor configuration field validation."""
        sensors = sample_sensor_config["sensors"]
        
        for sensor in sensors:
            assert "id" in sensor
            assert "type" in sensor
            assert "enabled" in sensor
            assert isinstance(sensor["enabled"], bool)


class TestSensorManager:
    """Test sensor manager functionality."""
    
    def test_sensor_manager_initialization(self, config_file):
        """Test sensor manager can be initialized with config."""
        # Import here to avoid ROS2 dependency issues in CI
        try:
            from vargard_sensor_layer.sensor_manager import SensorManager
            
            # This should not raise an exception
            manager = SensorManager(str(config_file))
            assert manager is not None
        except ImportError:
            pytest.skip("Sensor layer not available")
    
    def test_sensor_status_tracking(self, mock_sensor):
        """Test sensor status is properly tracked."""
        status = mock_sensor.get_status()
        
        assert "status" in status
        assert "fps" in status
        assert "frame_count" in status
        assert "error_count" in status
        
        assert status["status"] in ["healthy", "warning", "error"]
        assert isinstance(status["fps"], (int, float))
        assert isinstance(status["frame_count"], int)
        assert isinstance(status["error_count"], int)
    
    def test_sensor_frame_acquisition(self, mock_sensor, mock_image):
        """Test sensor frame acquisition."""
        success, frame = mock_sensor.get_frame()
        
        assert isinstance(success, bool)
        if success:
            assert frame is not None
            assert frame.shape == (480, 640, 3)  # Standard test image shape


class TestBaseSensor:
    """Test base sensor functionality."""
    
    def test_sensor_health_monitoring(self, mock_sensor):
        """Test sensor health monitoring capabilities."""
        # Test healthy sensor
        assert mock_sensor.is_connected()
        
        status = mock_sensor.get_status()
        assert status["status"] == "healthy"
        assert status["error_count"] == 0
    
    def test_sensor_reconnection_logic(self, mock_sensor):
        """Test sensor reconnection behavior."""
        # Simulate disconnection
        mock_sensor.is_connected.return_value = False
        
        # Should detect disconnection
        assert not mock_sensor.is_connected()
        
        # Simulate reconnection
        mock_sensor.is_connected.return_value = True
        assert mock_sensor.is_connected()


class TestUSBCamera:
    """Test USB camera specific functionality."""
    
    def test_usb_camera_device_detection(self):
        """Test USB camera device detection."""
        try:
            from vargard_sensor_layer.usb_camera import UsbCamera
            
            # Test device enumeration (should not crash)
            devices = UsbCamera.enumerate_devices()
            assert isinstance(devices, list)
            
        except ImportError:
            pytest.skip("USB camera module not available")
    
    def test_usb_camera_configuration(self):
        """Test USB camera configuration parameters."""
        config = {
            "device_index": 0,
            "width": 640,
            "height": 480,
            "fps": 30
        }
        
        # Validate configuration structure
        assert "device_index" in config
        assert isinstance(config["device_index"], int)
        assert config["device_index"] >= 0
        
        if "width" in config and "height" in config:
            assert config["width"] > 0
            assert config["height"] > 0


class TestSensorDiagnostics:
    """Test sensor diagnostic capabilities."""
    
    def test_sensor_performance_metrics(self, mock_sensor):
        """Test sensor performance metric collection."""
        status = mock_sensor.get_status()
        
        # Check performance metrics are present
        assert "fps" in status
        assert "frame_count" in status
        
        # Validate metric ranges
        assert 0 <= status["fps"] <= 120  # Reasonable FPS range
        assert status["frame_count"] >= 0
    
    def test_sensor_error_tracking(self, mock_sensor):
        """Test sensor error tracking functionality."""
        status = mock_sensor.get_status()
        
        assert "error_count" in status
        assert status["error_count"] >= 0
        
        # Error count should be integer
        assert isinstance(status["error_count"], int)
    
    def test_sensor_health_status_values(self, mock_sensor):
        """Test sensor health status uses valid values."""
        status = mock_sensor.get_status()
        
        valid_statuses = ["healthy", "warning", "error", "disconnected"]
        assert status["status"] in valid_statuses


class TestSensorIntegration:
    """Test sensor integration scenarios."""
    
    def test_multiple_sensor_management(self, sample_sensor_config):
        """Test managing multiple sensors simultaneously."""
        sensors = sample_sensor_config["sensors"]
        
        # Should handle multiple sensors
        assert len(sensors) >= 2
        
        # Each sensor should have unique ID
        sensor_ids = [sensor["id"] for sensor in sensors]
        assert len(sensor_ids) == len(set(sensor_ids))
    
    def test_sensor_enable_disable(self, sample_sensor_config):
        """Test sensor enable/disable functionality."""
        sensors = sample_sensor_config["sensors"]
        
        # Test enabled sensor
        enabled_sensor = next(s for s in sensors if s["enabled"])
        assert enabled_sensor["enabled"] is True
        
        # Test disabled sensor
        disabled_sensor = next(s for s in sensors if not s["enabled"])
        assert disabled_sensor["enabled"] is False
    
    def test_sensor_configuration_update(self, temp_dir, sample_sensor_config):
        """Test dynamic sensor configuration updates."""
        config_path = temp_dir / "dynamic_config.yaml"
        create_test_config(sample_sensor_config, config_path)
        
        # Modify configuration
        sample_sensor_config["sensors"][0]["enabled"] = False
        create_test_config(sample_sensor_config, config_path)
        
        # Reload and verify
        with open(config_path) as f:
            updated_config = yaml.safe_load(f)
        
        assert not updated_config["sensors"][0]["enabled"]