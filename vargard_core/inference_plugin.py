"""
Defines the abstract base class for inference plugins with enhanced capabilities.
"""
import time
from abc import ABC, abstractmethod
from enum import Enum
from typing import Dict, List, Any

class PluginStatus(Enum):
    """Plugin status enumeration."""
    UNINITIALIZED = "uninitialized"
    INITIALIZING = "initializing"
    READY = "ready"
    PROCESSING = "processing"
    ERROR = "error"
    DISABLED = "disabled"

class InferencePlugin(ABC):
    def __init__(self):
        self.status = PluginStatus.UNINITIALIZED
        self.name = self.__class__.__name__
        self.version = "1.0.0"
        self.description = ""
        self.inference_count = 0
        self.total_inference_time = 0.0
        self.last_inference_time = None
        self.error_count = 0
        self.config = {}
        
    @abstractmethod
    def initialize(self, params: dict):
        """
        Initialize the plugin with given parameters.
        
        Args:
            params: Configuration parameters for the plugin
            
        Raises:
            RuntimeError: If initialization fails
        """
        pass

    @abstractmethod
    def process(self, image, camera_info) -> list:
        """
        Run inference on a single image frame.

        Args:
            image: numpy.ndarray (BGR image)
            camera_info: sensor_msgs.msg.CameraInfo or None

        Returns:
            List of detection dicts, each containing keys:
              - bbox: [x, y, w, h]
              - confidence: float
              - class: string
              - Additional keys specific to the plugin
        """
        pass
    
    def get_info(self) -> Dict[str, Any]:
        """Get plugin information and statistics."""
        avg_time = (self.total_inference_time / self.inference_count 
                   if self.inference_count > 0 else 0.0)
        
        return {
            'name': self.name,
            'version': self.version,
            'description': self.description,
            'status': self.status.value,
            'inference_count': self.inference_count,
            'error_count': self.error_count,
            'average_inference_time': avg_time,
            'last_inference_time': self.last_inference_time,
            'config': self.config.copy()
        }
    
    def validate_config(self, params: dict) -> bool:
        """
        Validate configuration parameters.
        
        Args:
            params: Configuration parameters to validate
            
        Returns:
            bool: True if configuration is valid
        """
        # Default implementation - override in subclasses for specific validation
        return True
    
    def get_supported_formats(self) -> List[str]:
        """
        Get list of supported input image formats.
        
        Returns:
            List of supported formats (e.g., ['BGR', 'RGB', 'GRAY'])
        """
        return ['BGR']  # Default to BGR
    
    def get_required_params(self) -> List[str]:
        """
        Get list of required configuration parameters.
        
        Returns:
            List of required parameter names
        """
        return []  # No required params by default
    
    def process_with_timing(self, image, camera_info) -> tuple:
        """
        Process image with automatic timing and error handling.
        
        Returns:
            tuple: (detections, inference_time)
        """
        start_time = time.time()
        
        try:
            self.status = PluginStatus.PROCESSING
            detections = self.process(image, camera_info)
            
            # Update statistics
            inference_time = time.time() - start_time
            self.inference_count += 1
            self.total_inference_time += inference_time
            self.last_inference_time = inference_time
            self.status = PluginStatus.READY
            
            return detections, inference_time
            
        except Exception as e:
            self.error_count += 1
            self.status = PluginStatus.ERROR
            raise e
    
    def reset_statistics(self):
        """Reset performance statistics."""
        self.inference_count = 0
        self.total_inference_time = 0.0
        self.last_inference_time = None
        self.error_count = 0
    
    def enable(self):
        """Enable the plugin."""
        if self.status != PluginStatus.ERROR:
            self.status = PluginStatus.READY
    
    def disable(self):
        """Disable the plugin."""
        self.status = PluginStatus.DISABLED
    
    def cleanup(self):
        """Clean up plugin resources."""
        pass