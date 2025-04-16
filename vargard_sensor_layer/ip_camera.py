
"""
IP camera (RTSP) driver using OpenCV.
"""

import cv2
import time
from .base_sensor import BaseSensor

class IPCamera(BaseSensor):
    def __init__(self, rtsp_url: str, sensor_id: str = None):
        sid = sensor_id or f'ip_camera_{rtsp_url}'
        super().__init__(sid, 'ip_camera')
        self.cap = cv2.VideoCapture(rtsp_url)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open IP camera at {rtsp_url}")

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