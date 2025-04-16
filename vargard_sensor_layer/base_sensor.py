
"""
BaseSensor: abstract base class for all sensor drivers.
"""

from abc import ABC, abstractmethod

class BaseSensor(ABC):
    def __init__(self, sensor_id: str, sensor_type: str):
        self.sensor_id = sensor_id
        self.sensor_type = sensor_type

    @abstractmethod
    def get_frame(self):
        """
        Acquire data from the sensor.

        Returns:
            tuple: (data, metadata)
                data: numpy.ndarray for images or bytes for other sensors
                metadata: dict with keys 'sensor_id', 'sensor_type', 'timestamp'
        """
        pass
    def close(self):
        """
        Clean up any resources held by the sensor (e.g., close handles).
        """
        pass