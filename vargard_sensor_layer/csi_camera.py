
"""
CSI camera driver for Jetson using GStreamer.
"""

import cv2
import time
from .base_sensor import BaseSensor

class CsiCamera(BaseSensor):
    def __init__(self, width=1920, height=1080, framerate=30, sensor_id: str = 'csi_camera_0'):
        super().__init__(sensor_id, 'csi_camera')
        gst_str = (
            f'nvarguscamerasrc ! video/x-raw(memory:NVMM), width={width}, height={height}, '
            f'format=NV12, framerate={framerate}/1 ! nvvidconv ! video/x-raw, format=BGRx ! '
            'videoconvert ! video/x-raw, format=BGR ! appsink'
        )
        self.cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise RuntimeError('Cannot open CSI camera')

    def get_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError('Failed to get frame from CSI camera')
        metadata = {
            'sensor_id': self.sensor_id,
            'sensor_type': self.sensor_type,
            'timestamp': time.time()
        }
        return frame, metadata