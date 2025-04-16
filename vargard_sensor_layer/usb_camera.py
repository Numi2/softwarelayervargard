
"""
USB camera driver using OpenCV.
"""

import cv2
import time
from .base_sensor import BaseSensor

class UsbCamera(BaseSensor):
    def __init__(self, device_index: int):
        sensor_id = f'usb_camera_{device_index}'
        super().__init__(sensor_id, 'usb_camera')
        self.cap = cv2.VideoCapture(device_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open USB camera at index {device_index}")

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
        Release the OpenCV video capture.
        """
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()