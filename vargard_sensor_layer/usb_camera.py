
"""
USB camera driver using OpenCV with health monitoring and reconnection.
"""

import cv2
import time
from .base_sensor import BaseSensor, SensorStatus


class UsbCamera(BaseSensor):
    def __init__(self, device_index: int):
        sensor_id = f'usb_camera_{device_index}'
        super().__init__(sensor_id, 'usb_camera')
        self.device_index = device_index
        self.cap = None
        self._initialize_camera()

    def _initialize_camera(self):
        """Initialize the camera connection."""
        try:
            self.cap = cv2.VideoCapture(self.device_index)
            if not self.cap.isOpened():
                raise RuntimeError(f"Cannot open USB camera at index {self.device_index}")

            # Set some basic properties for better performance
            self.cap.set(
                cv2.CAP_PROP_BUFFERSIZE,
                1)  # Reduce buffer to avoid lag
            self.cap.set(cv2.CAP_PROP_FPS, 30)  # Set target FPS

            # Test frame capture
            ret, frame = self.cap.read()
            if not ret or frame is None:
                raise RuntimeError(f"Cannot capture test frame from camera {self.device_index}")

        except Exception as e:
            if self.cap:
                self.cap.release()
                self.cap = None
            raise e

    def get_frame(self):
        """Get frame with health monitoring."""
        try:
            if self.cap is None or not self.cap.isOpened():
                self.update_health_status(False)
                if not self.attempt_reconnect():
                    raise RuntimeError(f"Camera {self.sensor_id} is disconnected and reconnection failed")

            ret, frame = self.cap.read()
            if not ret or frame is None:
                self.update_health_status(False)
                # Try to reconnect on frame failure
                if self.attempt_reconnect():
                    ret, frame = self.cap.read()

                if not ret or frame is None:
                    raise RuntimeError(f"Failed to get frame from {self.sensor_id}")

            self.update_health_status(True)
            metadata = {
                'sensor_id': self.sensor_id,
                'sensor_type': self.sensor_type,
                'timestamp': time.time(),
                'frame_width': int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                'frame_height': int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                'fps': self.cap.get(cv2.CAP_PROP_FPS)
            }
            return frame, metadata

        except Exception as e:
            self.update_health_status(False)
            raise e

    def reconnect(self):
        """Attempt to reconnect to the USB camera."""
        try:
            if self.cap:
                self.cap.release()
            self._initialize_camera()
            return True
        except Exception:
            self.status = SensorStatus.FAILED
            return False

    def close(self):
        """Release the OpenCV video capture."""
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
            self.cap = None
