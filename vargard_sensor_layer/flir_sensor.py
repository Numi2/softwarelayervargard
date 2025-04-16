
"""
FLIR thermal sensor driver using OpenCV.
"""

import cv2
import time
from .base_sensor import BaseSensor

class FlirSensor(BaseSensor):
    def __init__(self, device_path: str):
        sensor_id = f'flir_{device_path.split('/')[-1]}'
        super().__init__(sensor_id, 'flir_thermal')
        self.cap = cv2.VideoCapture(device_path)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open FLIR device at {device_path}")

    def get_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError(f"Failed to get frame from {self.sensor_id}")
        metadata = {
            'sensor_id': self.sensor_id,
            'sensor_type': self.sensor_type,
            'timestamp': time.time()
        }
        return frame, metadata
    def close(self):
        """
        Release the FLIR video capture.
        """
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()