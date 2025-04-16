
"""
Radar sensor driver over UART.
"""

import time
import serial
from .base_sensor import BaseSensor

class RadarSensor(BaseSensor):
    def __init__(self, port: str, baudrate: int = 115200):
        sensor_id = f'radar_{port.split('/')[-1]}'
        super().__init__(sensor_id, 'radar')
        self.ser = serial.Serial(port, baudrate, timeout=1)

    def get_frame(self):
        raw = self.ser.readline()
        metadata = {
            'sensor_id': self.sensor_id,
            'sensor_type': self.sensor_type,
            'timestamp': time.time()
        }
        return raw, metadata