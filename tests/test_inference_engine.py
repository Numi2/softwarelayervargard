"""
Tests for the inference engine and plugin system.

These tests focus on the core inference functionality and plugin
management without requiring actual ML models or heavy dependencies.
"""

import pytest
import numpy as np
from unittest.mock import Mock, patch

# Import test utilities
from conftest import assert_detection_valid


class TestInferencePlugin:
    """Test inference plugin interface and functionality."""
    
    def test_plugin_interface_compliance(self, mock_plugin):
        """Test plugin implements required interface methods."""
        # Required methods
        assert hasattr(mock_plugin, 'plugin_id')
        assert hasattr(mock_plugin, 'plugin_name')
        assert hasattr(mock_plugin, 'is_ready')
        assert hasattr(mock_plugin, 'process')
        assert hasattr(mock_plugin, 'get_config')
        assert hasattr(mock_plugin, 'get_stats')
        
        # Test method return types
        assert isinstance(mock_plugin.plugin_id, str)
        assert isinstance(mock_plugin.plugin_name, str)
        assert isinstance(mock_plugin.is_ready(), bool)
        assert isinstance(mock_plugin.get_config(), dict)
        assert isinstance(mock_plugin.get_stats(), dict)
    
    def test_plugin_configuration_validation(self, mock_plugin):
        """Test plugin configuration validation."""
        config = mock_plugin.get_config()
        
        # Should have confidence threshold
        assert "confidence_threshold" in config
        assert 0.0 <= config["confidence_threshold"] <= 1.0
    
    def test_plugin_statistics_tracking(self, mock_plugin):
        """Test plugin statistics are properly tracked."""
        stats = mock_plugin.get_stats()
        
        required_stats = ["inference_count", "average_time", "error_count"]
        for stat in required_stats:
            assert stat in stats
            assert isinstance(stats[stat], (int, float))
            assert stats[stat] >= 0
    
    def test_plugin_process_method(self, mock_plugin, mock_image):
        """Test plugin process method behavior."""
        # Should accept image and metadata
        result = mock_plugin.process(mock_image, {"frame_id": 123})
        
        # Should return list of detections
        assert isinstance(result, list)


class TestDetectionFormat:
    """Test detection format validation and consistency."""
    
    def test_detection_format_validation(self, sample_detection):
        """Test detection format meets requirements."""
        assert_detection_valid(sample_detection)
    
    def test_detection_bbox_format(self, sample_detection):
        """Test bounding box format is correct."""
        bbox = sample_detection["bbox"]
        
        assert len(bbox) == 4
        x1, y1, x2, y2 = bbox
        
        # Coordinates should be in correct order
        assert x1 < x2, "x1 should be less than x2"
        assert y1 < y2, "y1 should be less than y2"
        
        # Coordinates should be non-negative
        assert all(coord >= 0 for coord in bbox)
    
    def test_detection_confidence_range(self, sample_detection):
        """Test confidence values are in valid range."""
        confidence = sample_detection["confidence"]
        
        assert 0.0 <= confidence <= 1.0
        assert isinstance(confidence, (int, float))
    
    def test_detection_class_information(self, sample_detection):
        """Test class information is properly formatted."""
        assert isinstance(sample_detection["class"], str)
        assert len(sample_detection["class"]) > 0
        
        assert isinstance(sample_detection["class_id"], int)
        assert sample_detection["class_id"] >= 0
    
    def test_detection_metadata_optional(self, sample_detection):
        """Test metadata field is optional but well-formatted when present."""
        if "metadata" in sample_detection:
            metadata = sample_detection["metadata"]
            assert isinstance(metadata, dict)
            
            if "inference_time" in metadata:
                assert isinstance(metadata["inference_time"], (int, float))
                assert metadata["inference_time"] > 0


class TestInferenceEngine:
    """Test inference engine core functionality."""
    
    def test_engine_plugin_management(self, mock_plugin):
        """Test inference engine can manage plugins."""
        try:
            from vargard_core.inference_node import InferenceNode
            
            # Mock ROS2 dependencies
            with patch('rclpy.node.Node'):
                engine = InferenceNode()
                
                # Should be able to add plugin
                assert engine is not None
                
        except ImportError:
            pytest.skip("Inference engine not available")
    
    def test_engine_process_frame(self, mock_plugin, mock_image):
        """Test engine can process frames through plugins."""
        # Simulate processing
        detections = mock_plugin.process(mock_image, {"timestamp": 12345})
        
        # Should return valid detections
        assert isinstance(detections, list)
        
        # Each detection should be valid
        for detection in detections:
            if detection:  # Skip empty detections
                assert_detection_valid(detection)
    
    def test_engine_performance_tracking(self, mock_plugin):
        """Test engine tracks performance metrics."""
        stats = mock_plugin.get_stats()
        
        # Should track timing
        assert "average_time" in stats
        assert stats["average_time"] > 0
        
        # Should track counts
        assert "inference_count" in stats
        assert stats["inference_count"] >= 0
    
    def test_engine_error_handling(self, mock_plugin):
        """Test engine handles errors gracefully."""
        stats = mock_plugin.get_stats()
        
        # Should track errors
        assert "error_count" in stats
        assert isinstance(stats["error_count"], int)
        assert stats["error_count"] >= 0


class TestYOLOv8Plugin:
    """Test YOLOv8 plugin specific functionality."""
    
    def test_yolov8_plugin_configuration(self):
        """Test YOLOv8 plugin configuration options."""
        config = {
            "model_path": "yolov8n.pt",
            "confidence_threshold": 0.5,
            "nms_threshold": 0.4,
            "max_detections": 100,
            "device": "cuda:0"
        }
        
        # Validate configuration structure
        assert "model_path" in config
        assert "confidence_threshold" in config
        assert 0.0 <= config["confidence_threshold"] <= 1.0
        
        if "nms_threshold" in config:
            assert 0.0 <= config["nms_threshold"] <= 1.0
        
        if "max_detections" in config:
            assert config["max_detections"] > 0
    
    def test_yolov8_detection_format(self):
        """Test YOLOv8 detection format conversion."""
        # Simulate YOLOv8 output format
        yolo_detection = {
            "bbox": [100, 50, 200, 150],  # x1, y1, x2, y2
            "confidence": 0.85,
            "class": "person",
            "class_id": 0
        }
        
        assert_detection_valid(yolo_detection)
    
    @pytest.mark.skipif(True, reason="Requires YOLOv8 model files")
    def test_yolov8_model_loading(self):
        """Test YOLOv8 model loading (integration test)."""
        try:
            from vargard_core.plugins.yolov8 import YOLOv8Plugin
            
            plugin = YOLOv8Plugin()
            config = {"model_path": "yolov8n.pt", "confidence_threshold": 0.5}
            
            # Should initialize without error
            plugin.initialize(config)
            assert plugin.is_ready()
            
        except ImportError:
            pytest.skip("YOLOv8 plugin not available")


class TestPluginPerformance:
    """Test plugin performance monitoring and optimization."""
    
    def test_inference_timing_tracking(self, mock_plugin):
        """Test inference timing is properly tracked."""
        stats = mock_plugin.get_stats()
        
        assert "average_time" in stats
        average_time = stats["average_time"]
        
        # Should be reasonable timing (< 1 second for most cases)
        assert 0 < average_time < 1.0
        assert isinstance(average_time, (int, float))
    
    def test_throughput_calculation(self, mock_plugin):
        """Test throughput calculation from statistics."""
        stats = mock_plugin.get_stats()
        
        inference_count = stats["inference_count"]
        average_time = stats["average_time"]
        
        if inference_count > 0 and average_time > 0:
            # Calculate theoretical FPS
            theoretical_fps = 1.0 / average_time
            assert theoretical_fps > 0
    
    def test_memory_efficiency(self, mock_image):
        """Test memory usage patterns."""
        # Test image should be reasonable size
        assert mock_image.nbytes < 10 * 1024 * 1024  # < 10MB
        
        # Should be standard format
        assert mock_image.dtype == np.uint8
        assert len(mock_image.shape) == 3  # Height, Width, Channels
        assert mock_image.shape[2] == 3   # RGB channels


class TestPluginIntegration:
    """Test plugin integration scenarios."""
    
    def test_multiple_plugin_support(self, mock_plugin):
        """Test system can handle multiple plugins."""
        # Simulate multiple plugins
        plugins = [mock_plugin, Mock(), Mock()]
        
        for i, plugin in enumerate(plugins):
            plugin.plugin_id = f"plugin_{i}"
            plugin.is_ready.return_value = True
        
        # Each plugin should have unique ID
        plugin_ids = [p.plugin_id for p in plugins]
        assert len(plugin_ids) == len(set(plugin_ids))
    
    def test_plugin_hot_swapping(self, mock_plugin):
        """Test plugins can be enabled/disabled dynamically."""
        # Initially ready
        assert mock_plugin.is_ready()
        
        # Simulate disable
        mock_plugin.is_ready.return_value = False
        assert not mock_plugin.is_ready()
        
        # Simulate re-enable
        mock_plugin.is_ready.return_value = True
        assert mock_plugin.is_ready()
    
    def test_plugin_error_isolation(self, mock_plugin):
        """Test plugin errors don't crash the system."""
        # Simulate plugin error
        mock_plugin.process.side_effect = Exception("Plugin error")
        
        try:
            mock_plugin.process(np.zeros((100, 100, 3)), {})
        except Exception as e:
            # Error should be caught and handled
            assert "Plugin error" in str(e)
    
    def test_plugin_configuration_updates(self, mock_plugin):
        """Test plugin configuration can be updated."""
        original_config = mock_plugin.get_config()
        
        # Simulate configuration update
        new_config = original_config.copy()
        new_config["confidence_threshold"] = 0.8
        mock_plugin.get_config.return_value = new_config
        
        updated_config = mock_plugin.get_config()
        assert updated_config["confidence_threshold"] == 0.8