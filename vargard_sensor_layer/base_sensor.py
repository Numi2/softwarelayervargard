
"""
BaseSensor: abstract base class for all sensor drivers with health monitoring.
"""

import time
from abc import ABC, abstractmethod
from enum import Enum

class SensorStatus(Enum):
    """Sensor health status enumeration."""
    HEALTHY = "healthy"
    DEGRADED = "degraded"
    FAILED = "failed"
    DISCONNECTED = "disconnected"

class BaseSensor(ABC):
    def __init__(self, sensor_id: str, sensor_type: str):
        self.sensor_id = sensor_id
        self.sensor_type = sensor_type
        self.status = SensorStatus.HEALTHY
        self.last_frame_time = None
        self.error_count = 0
        self.total_frames = 0
        self.reconnect_attempts = 0
        self.max_reconnect_attempts = 3

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

    @abstractmethod
    def reconnect(self):
        """
        Attempt to reconnect to the sensor.
        
        Returns:
            bool: True if reconnection successful, False otherwise
        """
        pass

    def update_health_status(self, success: bool):
        """Update sensor health based on operation success."""
        current_time = time.time()
        
        if success:
            self.last_frame_time = current_time
            self.total_frames += 1
            self.reconnect_attempts = 0
            
            # Improve status if we were degraded
            if self.status == SensorStatus.DEGRADED and self.error_count == 0:
                self.status = SensorStatus.HEALTHY
        else:
            self.error_count += 1
            
            # Degrade status based on error count
            if self.error_count >= 10:
                self.status = SensorStatus.FAILED
            elif self.error_count >= 3:
                self.status = SensorStatus.DEGRADED

    def get_health_info(self):
        """Get sensor health information."""
        return {
            'sensor_id': self.sensor_id,
            'sensor_type': self.sensor_type,
            'status': self.status.value,
            'error_count': self.error_count,
            'total_frames': self.total_frames,
            'last_frame_time': self.last_frame_time,
            'reconnect_attempts': self.reconnect_attempts
        }

    def attempt_reconnect(self):
        """Attempt to reconnect if sensor is failed."""
        if self.status == SensorStatus.FAILED and self.reconnect_attempts < self.max_reconnect_attempts:
            self.reconnect_attempts += 1
            if self.reconnect():
                self.status = SensorStatus.HEALTHY
                self.error_count = 0
                return True
        return False

    def close(self):
        """
        Clean up any resources held by the sensor (e.g., close handles).
        """
        pass