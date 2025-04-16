"""
Defines the abstract base class for inference plugins.
"""
from abc import ABC, abstractmethod

class InferencePlugin(ABC):
    @abstractmethod
    def initialize(self, params: dict):
        """
        Initialize the plugin with given parameters.
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
        """
        pass